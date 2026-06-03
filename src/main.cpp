// ============================================================
// Debug flags — comment out for production builds
// ============================================================
#define ENABLE_DEBUG_ENGINE_RPM
// #define ENABLE_TEST_OUTPUT_PIN

#include <Adafruit_ADS1X15.h>
#include <Adafruit_MLX90614.h>
#include <NMEA2000_esp32.h>

#include "n2k_senders.h"
#include "sensesp/net/discovery.h"
#include "sensesp/sensors/digital_input.h"
#include "sensesp/sensors/sensor.h"
#include "sensesp_onewire/onewire_temperature.h"
#include "sensesp/signalk/signalk_output.h"
#include "sensesp/system/lambda_consumer.h"
#include "sensesp/system/system_status_led.h"
#include "sensesp/transforms/lambda_transform.h"
#include "sensesp/transforms/linear.h"
#include "sensesp/transforms/threshold.h"
#include "sensesp/ui/config_item.h"
#include "sensesp_app_builder.h"
#define BUILDER_CLASS SensESPAppBuilder

#include "halmet_analog.h"
#include "halmet_const.h"
#include "halmet_digital.h"
#include "halmet_engine_hours.h"
#include "halmet_serial.h"
#include "sensesp/net/http_server.h"
#include "sensesp/net/networking.h"

using namespace sensesp;
using namespace halmet;
using namespace sensesp::onewire;

// ============================================================
// Global hardware handles
// ============================================================

tNMEA2000* nmea2000;
TwoWire* i2c;
Adafruit_ADS1115* ads1115;
DallasTemperatureSensors* dts;

N2kEngineParameterDynamicSender* engine_dynamic_sender;
N2kEngineParameterRapidSender* engine_rapid_sender;

// ============================================================
// Runtime state
// ============================================================

bool g_engine_running = false;
const float kEngineRunningThresholdRevsPerSec = 3.0f;

// ADS1115 gain — GAIN_ONE: ±4.096 V input (after 10:1 divider: ±40.96 V)
const adsGain_t kADS1115Gain = GAIN_ONE;

#ifdef ENABLE_DEBUG_ENGINE_RPM
// Simulated RPM value (rev/s). Set via serial: 'rpm 4.2' or 'rpm 0' for real.
float debug_engine_revs_per_sec = 0.0f;
#endif

#ifdef ENABLE_TEST_OUTPUT_PIN
// GPIO 33 outputs 380 Hz PWM (50% duty) to test the frequency counter input.
const int kTestOutputPin = GPIO_NUM_33;
const int kTestOutputFrequency = 380;
#endif

// ============================================================
// setup_framework — SensESP app, I2C bus, ADS1115, OneWire
// ============================================================

static void setup_framework() {
  BUILDER_CLASS builder;
  sensesp_app = (&builder)->set_hostname("yanmar")->get_app();

  i2c = new TwoWire(0);
  i2c->begin(kSDAPin, kSCLPin);

  ads1115 = new Adafruit_ADS1115();
  ads1115->setGain(kADS1115Gain);
  bool ads_ok = ads1115->begin(kADS1115Address, i2c);
  debugD("ADS1115 initialized: %d", ads_ok);

  dts = new DallasTemperatureSensors(kOneWirePin);
}

// ============================================================
// setup_nmea2000 — CAN bus, device identity, N2K sender objects
// ============================================================

static void setup_nmea2000() {
  nmea2000 = new tNMEA2000_esp32(kCANTxPin, kCANRxPin);
  nmea2000->SetN2kCANSendFrameBufSize(250);
  nmea2000->SetN2kCANReceiveFrameBufSize(250);

  nmea2000->SetProductInformation(
      "20231229",  // Manufacturer's Model serial code (max 32 chars)
      104,         // Manufacturer's product code
      "HALMET",    // Manufacturer's Model ID (max 33 chars)
      "1.0.0",     // Manufacturer's Software version code (max 40 chars)
      "1.0.0"      // Manufacturer's Model version (max 24 chars)
  );
  nmea2000->SetDeviceInformation(
      GetBoardSerialNumber(),
      140,   // Device function: Engine
      50,    // Device class: Propulsion
      2046   // Manufacturer code
  );
  nmea2000->SetMode(tNMEA2000::N2km_NodeOnly, 71);
  nmea2000->EnableForward(false);
  nmea2000->Open();

  event_loop()->onRepeat(1, []() { nmea2000->ParseMessages(); });

  engine_dynamic_sender = new N2kEngineParameterDynamicSender(
      "/NMEA 2000/Engine 1 Dynamic", 0, nmea2000);
  ConfigItem(engine_dynamic_sender)
      ->set_title("Engine 1 Dynamic")
      ->set_description("NMEA 2000 dynamic engine parameters for engine 1")
      ->set_sort_order(3010);

  engine_rapid_sender = new N2kEngineParameterRapidSender(
      "/NMEA 2000/Engine 1 Rapid Update", 0, nmea2000);
  ConfigItem(engine_rapid_sender)
      ->set_title("Engine 1 Rapid Update")
      ->set_description("NMEA 2000 rapid update engine parameters for engine 1")
      ->set_sort_order(3015);
}

// ============================================================
// setup_temperatures — OneWire (coolant, engine room, exhaust) + MLX90614
// ============================================================

static void setup_temperatures() {
  const uint32_t read_delay = 500;

  // Coolant temperature
  auto coolant_temp =
      new OneWireTemperature(dts, read_delay, "/coolantTemperature/oneWire");
  ConfigItem(coolant_temp)
      ->set_title("Coolant Temperature")
      ->set_description("Temperature of the engine coolant")
      ->set_sort_order(100);
  auto coolant_cal = new Linear(1.0, 0.0, "/coolantTemperature/linear");
  ConfigItem(coolant_cal)
      ->set_title("Coolant Temperature Calibration")
      ->set_description("Calibration for the coolant temperature sensor")
      ->set_sort_order(200);
  coolant_temp->connect_to(coolant_cal)
      ->connect_to(engine_dynamic_sender->temperature_);

  // Engine room temperature
  auto engine_room_temp = new OneWireTemperature(
      dts, read_delay, "/engineRoomTemperature/oneWire");
  ConfigItem(engine_room_temp)
      ->set_title("Engine Room Temperature")
      ->set_description("Temperature of the engine room")
      ->set_sort_order(300);
  auto engine_room_cal = new Linear(1.0, 0.0, "/engineRoomTemperature/linear");
  ConfigItem(engine_room_cal)
      ->set_title("Engine Room Temperature Calibration")
      ->set_description("Calibration for the engine room temperature sensor")
      ->set_sort_order(400);
  auto engine_room_n2k = new N2kTemperatureSender(
      "/NMEA 2000/Engine Room Temperature", 1,
      N2kts_EngineRoomTemperature, nmea2000);
  ConfigItem(engine_room_n2k)
      ->set_title("Engine Room Temperature NMEA 2000")
      ->set_description("NMEA 2000 PGN 130316 output for engine room temperature")
      ->set_sort_order(3035);
  engine_room_temp->connect_to(engine_room_cal)
      ->connect_to(engine_room_n2k->temperature_.get());

  // Exhaust temperature + overtemperature threshold
  auto exhaust_temp =
      new OneWireTemperature(dts, read_delay, "/exhaustTemperature/oneWire");
  ConfigItem(exhaust_temp)
      ->set_title("Exhaust Temperature")
      ->set_description("Temperature of the engine exhaust")
      ->set_sort_order(500);
  auto exhaust_cal = new Linear(1.0, 0.0, "/exhaustTemperature/linear");
  ConfigItem(exhaust_cal)
      ->set_title("Exhaust Temperature Calibration")
      ->set_description("Calibration for the exhaust temperature sensor")
      ->set_sort_order(600);
  auto exhaust_n2k = new N2kTemperatureSender(
      "/NMEA 2000/Exhaust Temperature", 2,
      N2kts_ExhaustGasTemperature, nmea2000);
  ConfigItem(exhaust_n2k)
      ->set_title("Exhaust Temperature NMEA 2000")
      ->set_description("NMEA 2000 PGN 130316 output for exhaust temperature")
      ->set_sort_order(3040);
  auto exhaust_threshold = new FloatThreshold(
      60.0f, 200.0f, true, "/exhaustTemperature/alarmThreshold");
  ConfigItem(exhaust_threshold)
      ->set_title("Exhaust Overtemperature Threshold")
      ->set_sort_order(601);
  exhaust_temp->connect_to(exhaust_cal)
      ->connect_to(exhaust_n2k->temperature_.get());
  // NOTE: over_temperature_ is written by both exhaust_threshold and D3 digital alarm.
  //       Both are gated by g_engine_running. Last writer wins (no OR-merge needed
  //       when engine is off; at runtime both alarm sources are valid).
  exhaust_temp->connect_to(exhaust_threshold)
      ->connect_to(new LambdaTransform<bool, bool>([](bool v) { return v && g_engine_running; }))
      ->connect_to(engine_dynamic_sender->over_temperature_);

  // Shaft bearing temperature (MLX90614 non-contact IR sensor)
  auto mlx = new Adafruit_MLX90614();
  bool mlx_ok = mlx->begin(MLX90614_I2CADDR, i2c);
  debugD("MLX90614 initialized: %d", mlx_ok);
  if (mlx_ok) {
    auto shaft_sensor = new RepeatSensor<float>(500, [mlx]() -> float {
      return static_cast<float>(mlx->readObjectTempC()) + 273.15f;
    });
    auto shaft_cal = new Linear(1.0, 0.0, "/shaftTemperature/calibration");
    ConfigItem(shaft_cal)
        ->set_title("Shaft Temperature Calibration")
        ->set_description("Offset/scale calibration for the MLX90614 sensor")
        ->set_sort_order(700);
    auto shaft_n2k = new N2kTemperatureSender(
        "/NMEA 2000/Shaft Temperature", 0,
        N2kts_ShaftSealTemperature, nmea2000);
    ConfigItem(shaft_n2k)
        ->set_title("Shaft Temperature NMEA 2000")
        ->set_description("NMEA 2000 PGN 130312 output for drive shaft temperature")
        ->set_sort_order(3030);
    shaft_sensor->connect_to(shaft_cal)
        ->connect_to(shaft_n2k->temperature_.get());
  }
}

// ============================================================
// setup_analog_inputs — fuel tank level + voltage measurement
// ============================================================

static void setup_analog_inputs() {
  const bool enable_sk = false;

  ConnectTankSender(ads1115, 0, "Fuel", "fuel.main", 3000, enable_sk);
  // Uncomment to add more tanks:
  // ConnectTankSender(ads1115, 1, "Water",    "water.main",  3100, enable_sk);
  // ConnectTankSender(ads1115, 2, "Ballast",  "ballast.1",   3200, enable_sk);
  // ConnectTankSender(ads1115, 3, "LiveWell", "liveWell.1",  3300, enable_sk);

  auto a2_voltage = new ADS1115VoltageInput(ads1115, 1, "/Voltage A2");
  ConfigItem(a2_voltage)
      ->set_title("Analog Voltage A2")
      ->set_description("Voltage level of analog input A2")
      ->set_sort_order(3000);
}

// ============================================================
// setup_alarms — D2 (low oil pressure) + D3 (overtemperature, active low)
// ============================================================

static void setup_alarms() {
  // D2: low oil pressure — open circuit with INPUT_PULLUP = alarm active
  auto alarm_d2 = ConnectAlarmSender(kDigitalInputPin2, "D2");
  alarm_d2->connect_to(new LambdaConsumer<bool>([](bool v) {
    static bool prev = !v;
    if (v != prev) { debugD("D2 oil pressure alarm: %d", v); prev = v; }
  }));
  // Gate: suppress alarm while engine is not running to avoid false positives at startup
  auto oil_gated = new LambdaTransform<bool, bool>(
      [](bool alarm) { return alarm && g_engine_running; });
  alarm_d2->connect_to(oil_gated)
      ->connect_to(engine_dynamic_sender->low_oil_pressure_);

  // D3: overtemperature — active-low signal, invert before forwarding
  // NOTE: over_temperature_ is also written by exhaust_threshold in setup_temperatures().
  auto alarm_d3 = ConnectAlarmSender(kDigitalInputPin3, "D3");
  auto alarm_d3_inv = alarm_d3->connect_to(
      new LambdaTransform<bool, bool>([](bool v) { return !v; }));
  alarm_d3_inv->connect_to(new LambdaConsumer<bool>([](bool v) {
    static bool prev = !v;
    if (v != prev) { debugD("D3 overtemp alarm: %d", v); prev = v; }
  }));
  auto alarm_d3_gated = alarm_d3_inv->connect_to(
      new LambdaTransform<bool, bool>([](bool v) { return v && g_engine_running; }));
  alarm_d3_gated->connect_to(engine_dynamic_sender->over_temperature_);
}

// ============================================================
// setup_tacho — engine RPM, engine hours counter, g_engine_running gate
// ============================================================

static void setup_tacho() {
  auto tacho = ConnectTachoSender(kDigitalInputPin1, "main");

  auto engine_hours =
      new EngineHoursCounter(kEngineRunningThresholdRevsPerSec, "/EngineHours");
  ConfigItem(engine_hours)
      ->set_title("Engine Hours")
      ->set_description("Engine running hours counter (persisted across restarts)")
      ->set_sort_order(3020);

  // In debug mode, allow overriding the RPM value via serial monitor.
  ValueProducer<float>* rpm_source = tacho;
#ifdef ENABLE_DEBUG_ENGINE_RPM
  auto rpm_override = new LambdaTransform<float, float>([](float real_rpm) {
    return debug_engine_revs_per_sec > 0.0f ? debug_engine_revs_per_sec : real_rpm;
  });
  tacho->connect_to(rpm_override);
  rpm_source = rpm_override;
#endif

  rpm_source->connect_to(&(engine_rapid_sender->engine_speed_));
  rpm_source->connect_to(engine_hours);
  rpm_source->connect_to(new LambdaConsumer<float>([](float rpm) {
    g_engine_running = rpm >= kEngineRunningThresholdRevsPerSec;
  }));
  engine_hours->connect_to(engine_dynamic_sender->total_engine_hours_.get());
}

/////////////////////////////////////////////////////////////////////
// Arduino entry points

void setup() {
  SetupLogging(ESP_LOG_DEBUG);
  Serial.begin(115200);

#ifdef ENABLE_DEBUG_ENGINE_RPM
  Serial.println("DEBUG RPM sim active. Type 'rpm 4.2' (rev/s) or 'rpm 0' for real signal.");
#endif

  setup_framework();
  setup_nmea2000();

  setup_temperatures();
  setup_analog_inputs();
  setup_alarms();
  setup_tacho();

#ifdef ENABLE_TEST_OUTPUT_PIN
  pinMode(kTestOutputPin, OUTPUT);
  ledcAttach(kTestOutputPin, kTestOutputFrequency, 13);
  ledcWrite(0, 4096);  // 50% duty cycle at 13-bit resolution
#endif

  // Prevent shared_ptr objects created in setup() from being garbage-collected
  while (true) { loop(); }
}

void loop() {
  event_loop()->tick();

#ifdef ENABLE_DEBUG_ENGINE_RPM
  if (Serial.available()) {
    String cmd = Serial.readStringUntil('\n');
    cmd.trim();
    if (cmd.startsWith("rpm ")) {
      debug_engine_revs_per_sec = cmd.substring(4).toFloat();
      Serial.printf("DEBUG RPM: %.2f rev/s (%.0f RPM)\n",
                    debug_engine_revs_per_sec, debug_engine_revs_per_sec * 60.0f);
    }
  }
#endif
}

