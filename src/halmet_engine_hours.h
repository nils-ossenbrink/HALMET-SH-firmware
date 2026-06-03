#ifndef HALMET_SRC_ENGINE_HOURS_H_
#define HALMET_SRC_ENGINE_HOURS_H_

#include "sensesp/sensors/sensor.h"
#include "sensesp/system/saveable.h"
#include "sensesp/system/valueconsumer.h"
#include "sensesp/system/valueproducer.h"
#include "sensesp_base_app.h"

namespace halmet {

/**
 * @brief Counts engine running hours based on a rev/s threshold.
 *
 * Increments a seconds counter once per second while the input value
 * (revolutions per second) is >= min_revs_per_second. The accumulated
 * total is persisted to the filesystem every 60 seconds so it survives
 * reboots. Outputs uint32_t seconds, matching NMEA 2000 PGN 127489
 * total_engine_hours field.
 */
class EngineHoursCounter : public sensesp::FileSystemSaveable,
                           public sensesp::ValueProducer<uint32_t>,
                           public sensesp::ValueConsumer<float> {
 public:
  EngineHoursCounter(float min_revs_per_second, String config_path)
      : sensesp::FileSystemSaveable{config_path},
        min_revs_per_second_{min_revs_per_second} {
    load();

    sensesp::event_loop()->onRepeat(1000, [this]() {
      if (current_revs_per_second_ >= min_revs_per_second_) {
        total_seconds_++;
        // Save to flash every 60 seconds to limit write cycles.
        if (total_seconds_ % 60 == 0) {
          save();
        }
        this->emit(total_seconds_);
      }
    });
  }

  void set(const float& value) override { current_revs_per_second_ = value; }

  // Gibt true zurück, wenn der Motor läuft (Drehzahl >= Schwelle)
  bool is_running() const { return current_revs_per_second_ >= min_revs_per_second_; }

  bool from_json(const JsonObject& config) override {
    if (config["total_seconds"].is<uint32_t>()) {
      total_seconds_ = config["total_seconds"].as<uint32_t>();
    }
    if (config["min_revs_per_second"].is<float>()) {
      min_revs_per_second_ = config["min_revs_per_second"].as<float>();
    }
    return true;
  }

  bool to_json(JsonObject& config) override {
    config["total_seconds"] = total_seconds_;
    config["min_revs_per_second"] = min_revs_per_second_;
    return true;
  }

 private:
  float min_revs_per_second_;
  float current_revs_per_second_ = 0.0f;
  uint32_t total_seconds_ = 0;
};

inline const String ConfigSchema(const EngineHoursCounter& obj) {
  return R"###({
    "type": "object",
    "properties": {
      "total_seconds": {
        "title": "Total engine seconds",
        "type": "integer",
        "description": "Total engine running time in seconds (can be edited for calibration)"
      },
      "min_revs_per_second": {
        "title": "Min rev/s threshold",
        "type": "number",
        "description": "Minimum rev/s to consider engine running (e.g. 3.0 = 180 RPM)"
      }
    }
  })###";
}

}  // namespace halmet

#endif  // HALMET_SRC_ENGINE_HOURS_H_
