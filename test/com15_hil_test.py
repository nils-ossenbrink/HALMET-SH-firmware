#!/usr/bin/env python3
"""Hardware-in-the-loop NMEA 2000 test via Actisense receiver on COM15.

The script can run in two modes:
1) Live mode (default): runs a command that emits canboat JSON lines.
2) File mode: reads canboat JSON lines from a file.

Pass criteria (default):
- PGN 127488 is present and has approximately 100 ms cadence.
- PGN 127489 is present and has approximately 500 ms cadence.

Optional criteria:
- PGN 130312 present (shaft temperature).
- RPM median is near an expected value.
"""

from __future__ import annotations

import argparse
import json
import re
import statistics
import subprocess
import sys
import time
from dataclasses import dataclass, field
from datetime import datetime
from pathlib import Path
from typing import Dict, Iterable, List, Optional, TextIO, Tuple


@dataclass
class PgnStats:
    times: List[float] = field(default_factory=list)
    rpm_values: List[float] = field(default_factory=list)


@dataclass
class CheckResult:
    name: str
    passed: bool
    details: str


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="HALMET COM15 HIL NMEA2000 test")
    parser.add_argument(
        "--port",
        default="COM15",
        help="Serial port for Actisense receiver (default: COM15)",
    )
    parser.add_argument(
        "--duration",
        type=int,
        default=30,
        help="Capture duration in seconds for live mode (default: 30)",
    )
    parser.add_argument(
        "--live-command",
        default='actisense-serial -r {port} | analyzer -json',
        help=(
            "Shell command producing canboat JSON lines. "
            "Use {port} as placeholder for --port."
        ),
    )
    parser.add_argument(
        "--input-jsonl",
        default=None,
        help="Path to canboat JSON-lines file for offline analysis",
    )
    parser.add_argument(
        "--require-shaft-temp",
        action="store_true",
        help="Require PGN 130312 to be present",
    )
    parser.add_argument(
        "--expect-rpm",
        type=float,
        default=None,
        help="Expected RPM median for PGN 127488 (optional)",
    )
    parser.add_argument(
        "--rpm-tolerance",
        type=float,
        default=30.0,
        help="Allowed absolute RPM error (default: 30)",
    )
    parser.add_argument(
        "--tolerance-fast-ms",
        type=float,
        default=40.0,
        help="Allowed cadence error for 127488 in ms (default: 40)",
    )
    parser.add_argument(
        "--tolerance-slow-ms",
        type=float,
        default=100.0,
        help="Allowed cadence error for 127489 in ms (default: 100)",
    )
    return parser.parse_args()


def parse_numeric(value: object) -> Optional[float]:
    if isinstance(value, (int, float)):
        return float(value)
    if isinstance(value, str):
        match = re.search(r"[-+]?\d+(?:\.\d+)?", value)
        if match:
            return float(match.group(0))
    return None


def parse_timestamp(value: object) -> Optional[float]:
    if isinstance(value, (int, float)):
        return float(value)
    if not isinstance(value, str):
        return None

    candidate = value.strip()
    if candidate.endswith("Z"):
        candidate = candidate[:-1] + "+00:00"
    try:
        return datetime.fromisoformat(candidate).timestamp()
    except ValueError:
        return None


def extract_rpm(fields: object) -> Optional[float]:
    if not isinstance(fields, dict):
        return None
    for key, value in fields.items():
        key_lc = str(key).lower()
        if "engine speed" in key_lc or key_lc == "speed":
            rpm = parse_numeric(value)
            if rpm is not None:
                return rpm
    return None


def consume_json_lines(lines: Iterable[str]) -> Dict[int, PgnStats]:
    stats: Dict[int, PgnStats] = {}
    now_fallback = time.time

    for raw in lines:
        line = raw.strip()
        if not line:
            continue
        if not line.startswith("{"):
            continue

        try:
            msg = json.loads(line)
        except json.JSONDecodeError:
            continue

        pgn_obj = msg.get("pgn")
        pgn = int(pgn_obj) if isinstance(pgn_obj, int) else parse_numeric(pgn_obj)
        if pgn is None:
            continue
        pgn_int = int(pgn)

        ts = parse_timestamp(msg.get("timestamp"))
        if ts is None:
            ts = now_fallback()

        st = stats.setdefault(pgn_int, PgnStats())
        st.times.append(ts)

        if pgn_int == 127488:
            rpm = extract_rpm(msg.get("fields"))
            if rpm is not None:
                st.rpm_values.append(rpm)

    return stats


def intervals_ms(times: List[float]) -> List[float]:
    if len(times) < 2:
        return []
    sorted_times = sorted(times)
    return [(sorted_times[i] - sorted_times[i - 1]) * 1000.0 for i in range(1, len(sorted_times))]


def check_presence(stats: Dict[int, PgnStats], pgn: int, min_msgs: int = 3) -> CheckResult:
    count = len(stats.get(pgn, PgnStats()).times)
    passed = count >= min_msgs
    return CheckResult(
        name=f"PGN {pgn} presence",
        passed=passed,
        details=f"messages={count}, required>={min_msgs}",
    )


def check_cadence(
    stats: Dict[int, PgnStats], pgn: int, expected_ms: float, tolerance_ms: float
) -> CheckResult:
    deltas = intervals_ms(stats.get(pgn, PgnStats()).times)
    if len(deltas) < 3:
        return CheckResult(
            name=f"PGN {pgn} cadence",
            passed=False,
            details="not enough intervals (need >=3)",
        )

    median = statistics.median(deltas)
    mean = statistics.fmean(deltas)
    err = abs(median - expected_ms)
    passed = err <= tolerance_ms
    return CheckResult(
        name=f"PGN {pgn} cadence",
        passed=passed,
        details=(
            f"median={median:.1f} ms, mean={mean:.1f} ms, "
            f"expected={expected_ms:.1f} ms, tolerance={tolerance_ms:.1f} ms"
        ),
    )


def check_rpm(stats: Dict[int, PgnStats], expected_rpm: float, tolerance: float) -> CheckResult:
    values = stats.get(127488, PgnStats()).rpm_values
    if not values:
        return CheckResult(
            name="RPM expectation",
            passed=False,
            details="no RPM values decoded from PGN 127488",
        )

    median = statistics.median(values)
    err = abs(median - expected_rpm)
    passed = err <= tolerance
    return CheckResult(
        name="RPM expectation",
        passed=passed,
        details=(
            f"median={median:.1f} RPM, expected={expected_rpm:.1f} RPM, "
            f"tolerance={tolerance:.1f} RPM"
        ),
    )


def read_live_json_lines(command: str, duration_s: int) -> Tuple[List[str], int]:
    proc = subprocess.Popen(
        command,
        shell=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        text=True,
        bufsize=1,
    )

    lines: List[str] = []
    deadline = time.monotonic() + duration_s

    try:
        assert proc.stdout is not None
        while time.monotonic() < deadline:
            line = proc.stdout.readline()
            if line:
                lines.append(line)
                continue

            if proc.poll() is not None:
                break

        # stop capture process after deadline
        if proc.poll() is None:
            proc.terminate()
            try:
                proc.wait(timeout=2)
            except subprocess.TimeoutExpired:
                proc.kill()
                proc.wait(timeout=2)
    finally:
        rc = proc.returncode if proc.returncode is not None else 0

    return lines, rc


def print_results(results: List[CheckResult]) -> int:
    failed = [r for r in results if not r.passed]
    for r in results:
        status = "PASS" if r.passed else "FAIL"
        print(f"[{status}] {r.name}: {r.details}")
    print()
    print(f"Summary: {len(results) - len(failed)} passed, {len(failed)} failed")
    return 1 if failed else 0


def main() -> int:
    args = parse_args()

    if args.input_jsonl:
        input_path = Path(args.input_jsonl)
        if not input_path.exists():
            print(f"ERROR: input file not found: {input_path}")
            return 2
        with input_path.open("r", encoding="utf-8", errors="ignore") as fh:
            stats = consume_json_lines(fh)
    else:
        live_command = args.live_command.format(port=args.port)
        print(f"Running live capture for {args.duration}s with command:")
        print(live_command)
        lines, rc = read_live_json_lines(live_command, args.duration)
        if not lines:
            print("ERROR: no output captured from live command")
            print("Hint: verify actisense-serial and analyzer are installed and in PATH")
            return 2
        stats = consume_json_lines(lines)

        if rc not in (0, -15):
            print(f"Note: live command exited with code {rc}")

    checks: List[CheckResult] = []
    checks.append(check_presence(stats, 127488))
    checks.append(check_presence(stats, 127489))
    checks.append(check_cadence(stats, 127488, 100.0, args.tolerance_fast_ms))
    checks.append(check_cadence(stats, 127489, 500.0, args.tolerance_slow_ms))

    if args.require_shaft_temp:
        checks.append(check_presence(stats, 130312))

    if args.expect_rpm is not None:
        checks.append(check_rpm(stats, args.expect_rpm, args.rpm_tolerance))

    return print_results(checks)


if __name__ == "__main__":
    sys.exit(main())
