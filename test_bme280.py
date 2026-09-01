"""Command-line test utility for the BME280 sensor."""
from __future__ import annotations

import argparse
import logging
import time
from typing import Iterable

from bme280_sensor import (
    BME280_I2C_ADDRESSES,
    BME280Reading,
    BME280NotFoundError,
    BME280Sensor,
    BME280Error,
)


def _parse_address(value: str) -> int:
    return int(value, 0)


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Test a BME280 sensor over I2C")
    parser.add_argument("--bus", type=int, default=1, help="I2C bus number (default: 1)")
    parser.add_argument(
        "--address",
        type=_parse_address,
        help="I2C address of the sensor (0x76 or 0x77)",
    )
    parser.add_argument(
        "--interval",
        type=float,
        default=2.0,
        help="Seconds between readings in continuous mode (default: 2.0)",
    )
    parser.add_argument(
        "--count",
        type=int,
        default=0,
        help="Number of readings to collect; 0 runs until Ctrl+C",
    )
    parser.add_argument(
        "--once",
        action="store_true",
        help="Read one measurement and exit",
    )
    parser.add_argument(
        "--verify",
        action="store_true",
        help="Read one measurement and validate it is within expected sensor ranges",
    )
    parser.add_argument(
        "--detect",
        action="store_true",
        help="Scan the common BME280 addresses and print any detected devices",
    )
    parser.add_argument(
        "--log-level",
        default="INFO",
        choices=("DEBUG", "INFO", "WARNING", "ERROR", "CRITICAL"),
        help="Logging verbosity (default: INFO)",
    )
    return parser


def _format_reading(reading: BME280Reading) -> str:
    return (
        f"Temp: {reading.temperature_c:6.2f} °C / {reading.temperature_f:6.2f} °F | "
        f"Humidity: {reading.humidity_percent:6.2f} % | "
        f"Pressure: {reading.pressure_hpa:7.2f} hPa | "
        f"Raw T/P/H: {reading.raw.temperature}/{reading.raw.pressure}/{reading.raw.humidity}"
    )


def _run_continuous(sensor: BME280Sensor, interval: float, count: int) -> int:
    remaining = count
    while remaining != 0:
        print(_format_reading(sensor.read()))
        if remaining > 0:
            remaining -= 1
        if remaining != 0:
            time.sleep(interval)
    return 0


def _detect(bus: int) -> int:
    try:
        found = BME280Sensor.detect(bus=bus)
    except OSError as exc:
        logging.error("Unable to scan I2C bus %s: %s", bus, exc)
        return 1
    if not found:
        print(f"No BME280 found on bus {bus}. Checked addresses: {', '.join(hex(a) for a in BME280_I2C_ADDRESSES)}")
        return 1
    print(f"Detected BME280 device(s) on bus {bus}: {', '.join(hex(a) for a in found)}")
    return 0


def main(argv: Iterable[str] | None = None) -> int:
    parser = build_parser()
    args = parser.parse_args(list(argv) if argv is not None else None)

    logging.basicConfig(
        level=getattr(logging, args.log_level),
        format="%(asctime)s %(levelname)s %(name)s: %(message)s",
    )

    if args.detect:
        return _detect(args.bus)

    addresses = (args.address,) if args.address is not None else BME280_I2C_ADDRESSES
    last_error: Exception | None = None

    for address in addresses:
        try:
            with BME280Sensor(bus=args.bus, address=address) as sensor:
                if not sensor.is_present:
                    raise BME280NotFoundError(f"No BME280 responded at 0x{address:02x}")
                print(f"BME280 detected at bus {args.bus}, address 0x{address:02x}")
                if args.verify:
                    reading = sensor.read()
                    ok = (
                        -40.0 <= reading.temperature_c <= 85.0
                        and 0.0 <= reading.humidity_percent <= 100.0
                        and 300.0 <= reading.pressure_hpa <= 1100.0
                    )
                    print(_format_reading(reading))
                    if ok:
                        print("Sensor verification passed.")
                        return 0
                    print("Sensor verification failed.")
                    return 1
                if args.once:
                    print(_format_reading(sensor.read()))
                    return 0
                interval = max(args.interval, 0.1)
                count = args.count if args.count > 0 else -1
                return _run_continuous(sensor, interval, count)
        except BME280Error as exc:
            last_error = exc
            logging.debug("Sensor error for address 0x%02x: %s", address, exc)
        except OSError as exc:
            last_error = exc
            logging.debug("I2C error for address 0x%02x: %s", address, exc)

    if last_error is not None:
        logging.error("Unable to communicate with BME280: %s", last_error)
    else:
        logging.error("Unable to find a BME280 sensor")
    return 1


if __name__ == "__main__":
    raise SystemExit(main())
