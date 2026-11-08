#!/usr/bin/env python3
"""
BNO085 IMU characterization data logger (Jetson Orin Nano, I2C).

Logs every available BNO085 report to CSV, flushing each row as it is written
and fsync'ing periodically so data survives crashes / power loss.

Install:
    pip3 install adafruit-blinka adafruit-circuitpython-bno08x adafruit-extended-bus

Check wiring (header pins 3/5 are I2C bus 7 on the Orin Nano dev kit):
    sudo i2cdetect -y -r 7      # should show 0x4a (or 0x4b)

Example:
    python3 bno085_logger.py --bus 7 --rate 100 --duration 3600 --outdir ./imu_data
"""

import argparse
import csv
import json
import os
import platform
import signal
import sys
import time
from datetime import datetime, timezone

from adafruit_bno08x import (
    BNO_REPORT_ACCELEROMETER,
    BNO_REPORT_GYROSCOPE,
    BNO_REPORT_MAGNETOMETER,
    BNO_REPORT_LINEAR_ACCELERATION,
    BNO_REPORT_GRAVITY,
    BNO_REPORT_ROTATION_VECTOR,
    BNO_REPORT_GAME_ROTATION_VECTOR,
    BNO_REPORT_GEOMAGNETIC_ROTATION_VECTOR,
    BNO_REPORT_RAW_ACCELEROMETER,
    BNO_REPORT_RAW_GYROSCOPE,
    BNO_REPORT_RAW_MAGNETOMETER,
    BNO_REPORT_STEP_COUNTER,
    BNO_REPORT_STABILITY_CLASSIFIER,
    BNO_REPORT_ACTIVITY_CLASSIFIER,
    BNO_REPORT_SHAKE_DETECTOR,
)
from adafruit_bno08x.i2c import BNO08X_I2C

XYZ = ("x", "y", "z")
QUAT = ("i", "j", "k", "real")


def to_signed16(vec):
    """Library returns raw ADC values as unsigned 16-bit; they are really int16."""
    if vec is None:
        return None
    return tuple(v - 65536 if v > 32767 else v for v in vec)


def unit_quat(q):
    """Library scales the geomagnetic quaternion wrong (norm comes out ~4); renormalize."""
    if q is None:
        return None
    n = sum(c * c for c in q) ** 0.5
    return tuple(c / n for c in q) if n > 0 else q


# (name, report id, getter, column names)
FEATURES = [
    ("accel", BNO_REPORT_ACCELEROMETER, lambda s: s.acceleration,
     [f"accel_{a}_mps2" for a in XYZ]),
    ("gyro", BNO_REPORT_GYROSCOPE, lambda s: s.gyro,
     [f"gyro_{a}_rads" for a in XYZ]),
    ("mag", BNO_REPORT_MAGNETOMETER, lambda s: s.magnetic,
     [f"mag_{a}_uT" for a in XYZ]),
    ("lin_accel", BNO_REPORT_LINEAR_ACCELERATION, lambda s: s.linear_acceleration,
     [f"lin_accel_{a}_mps2" for a in XYZ]),
    ("gravity", BNO_REPORT_GRAVITY, lambda s: s.gravity,
     [f"gravity_{a}_mps2" for a in XYZ]),
    ("rot_vec", BNO_REPORT_ROTATION_VECTOR, lambda s: s.quaternion,
     [f"rotvec_{q}" for q in QUAT]),
    ("game_rot_vec", BNO_REPORT_GAME_ROTATION_VECTOR, lambda s: s.game_quaternion,
     [f"game_rotvec_{q}" for q in QUAT]),
    ("geomag_rot_vec", BNO_REPORT_GEOMAGNETIC_ROTATION_VECTOR,
     lambda s: unit_quat(s.geomagnetic_quaternion),
     [f"geomag_rotvec_{q}" for q in QUAT]),
    ("raw_accel", BNO_REPORT_RAW_ACCELEROMETER,
     lambda s: to_signed16(s.raw_acceleration),
     [f"raw_accel_{a}" for a in XYZ]),
    ("raw_gyro", BNO_REPORT_RAW_GYROSCOPE,
     lambda s: to_signed16(s.raw_gyro),
     [f"raw_gyro_{a}" for a in XYZ]),
    ("raw_mag", BNO_REPORT_RAW_MAGNETOMETER,
     lambda s: to_signed16(s.raw_magnetic),
     [f"raw_mag_{a}" for a in XYZ]),
    ("steps", BNO_REPORT_STEP_COUNTER, lambda s: s.steps, ["steps"]),
    ("stability", BNO_REPORT_STABILITY_CLASSIFIER,
     lambda s: s.stability_classification, ["stability"]),
    ("activity", BNO_REPORT_ACTIVITY_CLASSIFIER,
     lambda s: (s.activity_classification or {}).get("most_likely"),
     ["activity_most_likely"]),
    ("shake", BNO_REPORT_SHAKE_DETECTOR, lambda s: s.shake, ["shake"]),
]

# Default set for noise characterization: the sensor outputs, calibrated and raw.
# Fusion outputs and classifiers add bus load (lowering the sample rate) and file
# size without helping noise analysis; turn them on with --features or --all-features.
DEFAULT_FEATURES = ["accel", "gyro", "mag", "raw_accel", "raw_gyro", "raw_mag"]

TIME_COLS = ["sample_idx", "unix_time_s", "elapsed_s", "loop_dt_s"]
STATUS_COLS = ["mag_accuracy", "read_errors_total", "reinit_count"]


class Stop(Exception):
    pass


def _sigterm(_sig, _frm):
    raise Stop()


def open_i2c(bus, freq):
    try:
        from adafruit_extended_bus import ExtendedI2C
        return ExtendedI2C(bus, frequency=freq)
    except ImportError:
        import board
        import busio
        print("adafruit_extended_bus not installed; using board.SCL/SDA", file=sys.stderr)
        return busio.I2C(board.SCL, board.SDA, frequency=freq)


def init_sensor(i2c, address, report_interval_us, selected):
    sensor = BNO08X_I2C(i2c, address=address)
    enabled = []
    for name, rid, _, _ in FEATURES:
        if name not in selected:
            continue
        try:
            try:
                sensor.enable_feature(rid, report_interval_us)
            except TypeError:  # older library without report_interval arg
                sensor.enable_feature(rid)
            enabled.append(name)
        except Exception as e:  # keep going; log what failed
            print(f"[warn] could not enable {name}: {e}", file=sys.stderr)
        time.sleep(0.05)
    return sensor, enabled


def fmt(v):
    # 7 significant digits is more than the sensor's fixed-point resolution
    return f"{v:.7g}" if isinstance(v, float) else v


def flatten(value, ncols):
    if value is None:
        return [""] * ncols
    if isinstance(value, (tuple, list)):
        vals = [fmt(v) for v in list(value)[:ncols]]
        return vals + [""] * (ncols - len(vals))
    return [fmt(value)] + [""] * (ncols - 1)


def main():
    p = argparse.ArgumentParser(description="BNO085 characterization logger")
    p.add_argument("--bus", type=int, default=7, help="I2C bus number (default 7)")
    p.add_argument("--address", type=lambda x: int(x, 0), default=0x4A,
                   help="I2C address (0x4A default, 0x4B if DI pulled high)")
    p.add_argument("--i2c-freq", type=int, default=400000)
    p.add_argument("--rate", type=float, default=100.0,
                   help="Requested report rate and logging rate in Hz (default 100)")
    p.add_argument("--duration", type=float, default=0,
                   help="Seconds to log; 0 = until Ctrl-C")
    p.add_argument("--outdir", default="./imu_data")
    p.add_argument("--tag", default="", help="Optional label added to file name")
    p.add_argument("--fsync-interval", type=float, default=1.0,
                   help="Seconds between os.fsync (0 = every row)")
    p.add_argument("--features", nargs="*", default=DEFAULT_FEATURES,
                   help=f"Features to log (default {DEFAULT_FEATURES}). "
                        f"Choices: {[f[0] for f in FEATURES]}")
    p.add_argument("--all-features", action="store_true",
                   help="Log every report the BNO085 offers (slower, bigger files)")
    p.add_argument("--skip", nargs="*", default=[],
                   help="Features to remove from the selected set")
    p.add_argument("--max-consecutive-errors", type=int, default=20,
                   help="Errors in a row before the sensor is considered lost")
    p.add_argument("--reinit-on-error", action="store_true",
                   help="Re-initialize the sensor instead of stopping when it is lost. "
                        "Breaks data continuity and resets fusion/calibration state; "
                        "do NOT use for noise / Allan variance characterization.")
    args = p.parse_args()

    os.makedirs(args.outdir, exist_ok=True)
    stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    base = f"bno085_{stamp}" + (f"_{args.tag}" if args.tag else "")
    csv_path = os.path.join(args.outdir, base + ".csv")
    meta_path = os.path.join(args.outdir, base + "_meta.json")

    report_interval_us = int(1e6 / args.rate)
    period = 1.0 / args.rate

    signal.signal(signal.SIGTERM, _sigterm)

    i2c = open_i2c(args.bus, args.i2c_freq)
    all_names = [f[0] for f in FEATURES]
    unknown = set(args.features) - set(all_names)
    if unknown:
        p.error(f"unknown features {sorted(unknown)}; choices: {all_names}")
    selected = set(all_names if args.all_features else args.features) - set(args.skip)

    sensor, enabled = init_sensor(i2c, args.address, report_interval_us, selected)
    # Only write columns for features that actually enabled (keeps the CSV small)
    active = [f for f in FEATURES if f[0] in enabled]
    print(f"Enabled features: {enabled}")

    meta = {
        "start_utc": datetime.now(timezone.utc).isoformat(),
        "host": platform.node(),
        "platform": platform.platform(),
        "args": vars(args),
        "report_interval_us": report_interval_us,
        "enabled_features": enabled,
        "csv": os.path.basename(csv_path),
    }
    with open(meta_path, "w") as mf:
        json.dump(meta, mf, indent=2)

    header = TIME_COLS + [c for f in active for c in f[3]] + STATUS_COLS

    idx = 0
    errors_total = 0
    consecutive_errors = 0
    reinit_count = 0

    # line-buffered so every row goes to the OS immediately
    with open(csv_path, "w", newline="", buffering=1) as f:
        writer = csv.writer(f)
        writer.writerow(header)
        f.flush()
        os.fsync(f.fileno())

        t0 = time.monotonic()
        last_t = t0
        last_sync = t0
        next_t = t0
        print(f"Logging to {csv_path}  (Ctrl-C to stop)")

        try:
            while True:
                now = time.monotonic()
                if args.duration and now - t0 >= args.duration:
                    break

                row_data = []
                had_error = False
                for name, _, getter, cols in active:
                    if name not in enabled:
                        row_data += [""] * len(cols)
                        continue
                    try:
                        row_data += flatten(getter(sensor), len(cols))
                    except Exception as e:
                        had_error = True
                        errors_total += 1
                        row_data += [""] * len(cols)
                        if consecutive_errors == 0:
                            print(f"[warn] read error ({name}): {e}", file=sys.stderr)

                try:
                    mag_acc = sensor.calibration_status
                except Exception:
                    mag_acc = ""

                consecutive_errors = consecutive_errors + 1 if had_error else 0

                t = time.monotonic()
                row = [
                    idx,
                    f"{time.time():.6f}",
                    f"{t - t0:.6f}",
                    f"{t - last_t:.6f}",
                ] + row_data + [mag_acc, errors_total, reinit_count]
                writer.writerow(row)
                f.flush()
                last_t = t
                idx += 1

                if args.fsync_interval == 0 or t - last_sync >= args.fsync_interval:
                    os.fsync(f.fileno())
                    last_sync = t

                if idx % int(max(args.rate, 1) * 10) == 0:
                    print(f"{idx} samples, {t - t0:.1f} s, errors={errors_total}, "
                          f"reinits={reinit_count}")

                if consecutive_errors >= args.max_consecutive_errors:
                    if not args.reinit_on_error:
                        print(f"[error] sensor lost after {consecutive_errors} "
                              "consecutive errors; stopping to preserve a continuous "
                              "record (use --reinit-on-error to override)",
                              file=sys.stderr)
                        meta["stopped_reason"] = "sensor_lost"
                        break
                    print("[warn] too many consecutive errors, re-initializing sensor",
                          file=sys.stderr)
                    time.sleep(0.5)
                    try:
                        sensor, enabled = init_sensor(
                            i2c, args.address, report_interval_us, selected)
                        reinit_count += 1
                        consecutive_errors = 0
                    except Exception as e:
                        print(f"[error] re-init failed: {e}", file=sys.stderr)
                        time.sleep(1.0)

                next_t += period
                sleep = next_t - time.monotonic()
                if sleep > 0:
                    time.sleep(sleep)
                else:
                    next_t = time.monotonic()  # fell behind; don't try to catch up

        except (KeyboardInterrupt, Stop):
            print("\nStopping...")
        finally:
            f.flush()
            os.fsync(f.fileno())
            meta.update({
                "end_utc": datetime.now(timezone.utc).isoformat(),
                "samples": idx,
                "read_errors_total": errors_total,
                "reinit_count": reinit_count,
            })
            with open(meta_path, "w") as mf:
                json.dump(meta, mf, indent=2)
            print(f"Saved {idx} samples to {csv_path}")


if __name__ == "__main__":
    main()