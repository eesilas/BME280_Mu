"""BME280 sensor driver for LattePanda Mu over I2C.

This module implements the Bosch BME280 compensation formulas directly so it can
run on Python 3.14 with a minimal dependency footprint.
"""
from __future__ import annotations

from dataclasses import dataclass
import logging
import time
from typing import Final

from smbus2 import SMBus

LOGGER = logging.getLogger(__name__)

BME280_I2C_ADDRESSES: Final[tuple[int, int]] = (0x76, 0x77)
BME280_CHIP_ID: Final[int] = 0x60
BME280_RESET_CMD: Final[int] = 0xB6

_REG_ID = 0xD0
_REG_RESET = 0xE0
_REG_CTRL_HUM = 0xF2
_REG_STATUS = 0xF3
_REG_CTRL_MEAS = 0xF4
_REG_CONFIG = 0xF5
_REG_DATA = 0xF7

_OSRS_MAP: Final[dict[int, int]] = {1: 0x01, 2: 0x02, 4: 0x03, 8: 0x04, 16: 0x05}
_FILTER_MAP: Final[dict[int, int]] = {0: 0x00, 2: 0x01, 4: 0x02, 8: 0x03, 16: 0x04}


class BME280Error(RuntimeError):
    """Base exception for BME280 driver errors."""


class BME280NotFoundError(BME280Error):
    """Raised when the sensor does not respond on the I2C bus."""


class BME280CalibrationError(BME280Error):
    """Raised when factory calibration data cannot be read."""


class BME280ReadError(BME280Error):
    """Raised when a measurement cannot be completed."""


@dataclass(frozen=True)
class BME280RawReading:
    """Uncompensated raw ADC values from the sensor."""

    temperature: int
    pressure: int
    humidity: int


@dataclass(frozen=True)
class BME280Reading:
    """Compensated measurement values."""

    temperature_c: float
    temperature_f: float
    humidity_percent: float
    pressure_hpa: float
    raw: BME280RawReading


@dataclass(frozen=True)
class _Calibration:
    dig_t1: int
    dig_t2: int
    dig_t3: int
    dig_p1: int
    dig_p2: int
    dig_p3: int
    dig_p4: int
    dig_p5: int
    dig_p6: int
    dig_p7: int
    dig_p8: int
    dig_p9: int
    dig_h1: int
    dig_h2: int
    dig_h3: int
    dig_h4: int
    dig_h5: int
    dig_h6: int


class BME280Sensor:
    """Driver for the BME280 temperature, humidity, and pressure sensor."""

    def __init__(
        self,
        bus: int = 1,
        address: int = 0x76,
        *,
        temperature_oversampling: int = 2,
        pressure_oversampling: int = 16,
        humidity_oversampling: int = 2,
        filter_coefficient: int = 16,
        measurement_timeout: float = 1.0,
        auto_reset: bool = True,
    ) -> None:
        self._bus_num = bus
        self._address = address
        self._measurement_timeout = measurement_timeout
        self._bus = SMBus(bus)
        self._t_fine: float | None = None
        self._calibration: _Calibration | None = None

        self._validate_address()
        if auto_reset:
            self.reset()
        self._calibration = self._read_calibration()
        self.configure(
            temperature_oversampling=temperature_oversampling,
            pressure_oversampling=pressure_oversampling,
            humidity_oversampling=humidity_oversampling,
            filter_coefficient=filter_coefficient,
        )

    @property
    def bus(self) -> int:
        return self._bus_num

    @property
    def address(self) -> int:
        return self._address

    @property
    def chip_id(self) -> int:
        return self._read_u8(_REG_ID)

    @property
    def is_present(self) -> bool:
        try:
            return self.chip_id == BME280_CHIP_ID
        except OSError:
            return False

    def close(self) -> None:
        self._bus.close()

    def __enter__(self) -> "BME280Sensor":
        return self

    def __exit__(self, exc_type, exc, tb) -> None:
        self.close()

    @classmethod
    def detect(
        cls,
        bus: int = 1,
        addresses: tuple[int, ...] = BME280_I2C_ADDRESSES,
    ) -> list[int]:
        found: list[int] = []
        with SMBus(bus) as probe_bus:
            for address in addresses:
                try:
                    if probe_bus.read_byte_data(address, _REG_ID) == BME280_CHIP_ID:
                        found.append(address)
                except OSError:
                    LOGGER.debug("No BME280 at I2C address 0x%02x", address)
        return found

    def reset(self) -> None:
        LOGGER.debug("Resetting BME280 at bus %s address 0x%02x", self._bus_num, self._address)
        self._write_u8(_REG_RESET, BME280_RESET_CMD)
        time.sleep(0.01)

    def configure(
        self,
        *,
        temperature_oversampling: int = 2,
        pressure_oversampling: int = 16,
        humidity_oversampling: int = 2,
        filter_coefficient: int = 16,
    ) -> None:
        ctrl_hum = self._oversampling_value(humidity_oversampling, "humidity")
        ctrl_meas = (
            self._oversampling_value(temperature_oversampling, "temperature") << 5
        ) | (
            self._oversampling_value(pressure_oversampling, "pressure") << 2
        )
        config = self._filter_value(filter_coefficient) << 2

        self._write_u8(_REG_CTRL_HUM, ctrl_hum)
        self._write_u8(_REG_CONFIG, config)
        self._write_u8(_REG_CTRL_MEAS, ctrl_meas)
        LOGGER.debug(
            "Configured BME280: temp_osrs=%sx pressure_osrs=%sx humidity_osrs=%sx filter=%s",
            temperature_oversampling,
            pressure_oversampling,
            humidity_oversampling,
            filter_coefficient,
        )

    def read_raw(self) -> BME280RawReading:
        self._start_forced_measurement()
        self._wait_for_measurement()
        data = self._read_block(_REG_DATA, 8)

        adc_p = ((data[0] << 12) | (data[1] << 4) | (data[2] >> 4))
        adc_t = ((data[3] << 12) | (data[4] << 4) | (data[5] >> 4))
        adc_h = (data[6] << 8) | data[7]
        return BME280RawReading(temperature=adc_t, pressure=adc_p, humidity=adc_h)

    def read(self) -> BME280Reading:
        raw = self.read_raw()
        temperature_c = self._compensate_temperature(raw.temperature)
        pressure_hpa = self._compensate_pressure(raw.pressure)
        humidity_percent = self._compensate_humidity(raw.humidity)
        return BME280Reading(
            temperature_c=temperature_c,
            temperature_f=temperature_c * 9.0 / 5.0 + 32.0,
            humidity_percent=humidity_percent,
            pressure_hpa=pressure_hpa,
            raw=raw,
        )

    def verify(self) -> bool:
        try:
            reading = self.read()
        except OSError as exc:
            raise BME280ReadError(f"Unable to read BME280: {exc}") from exc
        return (
            -40.0 <= reading.temperature_c <= 85.0
            and 0.0 <= reading.humidity_percent <= 100.0
            and 300.0 <= reading.pressure_hpa <= 1100.0
        )

    def _validate_address(self) -> None:
        chip_id = self.chip_id
        if chip_id != BME280_CHIP_ID:
            raise BME280NotFoundError(
                f"No BME280 detected at address 0x{self._address:02x} on bus {self._bus_num}"
            )

    def _start_forced_measurement(self) -> None:
        ctrl_meas = self._read_u8(_REG_CTRL_MEAS)
        self._write_u8(_REG_CTRL_MEAS, ctrl_meas | 0x01)

    def _wait_for_measurement(self) -> None:
        deadline = time.monotonic() + self._measurement_timeout
        while time.monotonic() < deadline:
            if self._read_u8(_REG_STATUS) & 0x08 == 0:
                return
            time.sleep(0.002)
        raise TimeoutError(
            f"Timed out waiting for BME280 measurement on bus {self._bus_num} address 0x{self._address:02x}"
        )

    def _read_calibration(self) -> _Calibration:
        try:
            calib1 = self._read_block(0x88, 26)
            calib2 = self._read_block(0xE1, 7)
        except OSError as exc:
            raise BME280CalibrationError("Failed to read BME280 calibration registers") from exc

        dig_t1 = self._u16le(calib1, 0)
        dig_t2 = self._s16le(calib1, 2)
        dig_t3 = self._s16le(calib1, 4)
        dig_p1 = self._u16le(calib1, 6)
        dig_p2 = self._s16le(calib1, 8)
        dig_p3 = self._s16le(calib1, 10)
        dig_p4 = self._s16le(calib1, 12)
        dig_p5 = self._s16le(calib1, 14)
        dig_p6 = self._s16le(calib1, 16)
        dig_p7 = self._s16le(calib1, 18)
        dig_p8 = self._s16le(calib1, 20)
        dig_p9 = self._s16le(calib1, 22)
        dig_h1 = calib1[25]
        dig_h2 = self._s16le(calib2, 0)
        dig_h3 = calib2[2]
        dig_h4 = self._sign_extend_12((calib2[3] << 4) | (calib2[4] & 0x0F))
        dig_h5 = self._sign_extend_12((calib2[5] << 4) | (calib2[4] >> 4))
        dig_h6 = self._sign_extend_8(calib2[6])

        LOGGER.debug(
            "Calibration: T=[%s,%s,%s] P=[%s,%s,%s,%s,%s,%s,%s,%s,%s] H=[%s,%s,%s,%s,%s,%s]",
            dig_t1,
            dig_t2,
            dig_t3,
            dig_p1,
            dig_p2,
            dig_p3,
            dig_p4,
            dig_p5,
            dig_p6,
            dig_p7,
            dig_p8,
            dig_p9,
            dig_h1,
            dig_h2,
            dig_h3,
            dig_h4,
            dig_h5,
            dig_h6,
        )

        return _Calibration(
            dig_t1=dig_t1,
            dig_t2=dig_t2,
            dig_t3=dig_t3,
            dig_p1=dig_p1,
            dig_p2=dig_p2,
            dig_p3=dig_p3,
            dig_p4=dig_p4,
            dig_p5=dig_p5,
            dig_p6=dig_p6,
            dig_p7=dig_p7,
            dig_p8=dig_p8,
            dig_p9=dig_p9,
            dig_h1=dig_h1,
            dig_h2=dig_h2,
            dig_h3=dig_h3,
            dig_h4=dig_h4,
            dig_h5=dig_h5,
            dig_h6=dig_h6,
        )

    def _compensate_temperature(self, adc_t: int) -> float:
        calib = self._require_calibration()
        var1 = (adc_t / 16384.0 - calib.dig_t1 / 1024.0) * calib.dig_t2
        var2 = ((adc_t / 131072.0 - calib.dig_t1 / 8192.0) ** 2) * calib.dig_t3
        self._t_fine = var1 + var2
        return self._t_fine / 5120.0

    def _compensate_pressure(self, adc_p: int) -> float:
        calib = self._require_calibration()
        if self._t_fine is None:
            self._compensate_temperature(self.read_raw().temperature)
        assert self._t_fine is not None
        var1 = self._t_fine / 2.0 - 64000.0
        var2 = var1 * var1 * calib.dig_p6 / 32768.0
        var2 = var2 + var1 * calib.dig_p5 * 2.0
        var2 = var2 / 4.0 + calib.dig_p4 * 65536.0
        var1 = (calib.dig_p3 * var1 * var1 / 524288.0 + calib.dig_p2 * var1) / 524288.0
        var1 = (1.0 + var1 / 32768.0) * calib.dig_p1
        if var1 == 0.0:
            raise BME280ReadError("Invalid pressure calibration value")
        pressure = 1048576.0 - adc_p
        pressure = ((pressure - var2 / 4096.0) * 6250.0) / var1
        var1 = calib.dig_p9 * pressure * pressure / 2147483648.0
        var2 = pressure * calib.dig_p8 / 32768.0
        pressure = pressure + (var1 + var2 + calib.dig_p7) / 16.0
        return pressure / 100.0

    def _compensate_humidity(self, adc_h: int) -> float:
        calib = self._require_calibration()
        if self._t_fine is None:
            self._compensate_temperature(self.read_raw().temperature)
        assert self._t_fine is not None
        var_h = self._t_fine - 76800.0
        var_h = (
            adc_h
            - (calib.dig_h4 * 64.0 + calib.dig_h5 / 16384.0 * var_h)
        ) * (
            calib.dig_h2
            / 65536.0
            * (1.0 + calib.dig_h6 / 67108864.0 * var_h * (1.0 + calib.dig_h3 / 67108864.0 * var_h))
        )
        var_h = var_h * (1.0 - calib.dig_h1 * var_h / 524288.0)
        if var_h > 100.0:
            return 100.0
        if var_h < 0.0:
            return 0.0
        return var_h

    def _require_calibration(self) -> _Calibration:
        if self._calibration is None:
            raise BME280CalibrationError("BME280 calibration data not loaded")
        return self._calibration

    def _oversampling_value(self, value: int, name: str) -> int:
        try:
            return _OSRS_MAP[value]
        except KeyError as exc:
            raise ValueError(f"Unsupported {name} oversampling value: {value}") from exc

    def _filter_value(self, value: int) -> int:
        try:
            return _FILTER_MAP[value]
        except KeyError as exc:
            raise ValueError(f"Unsupported filter coefficient: {value}") from exc

    def _read_u8(self, register: int) -> int:
        return self._bus.read_byte_data(self._address, register)

    def _write_u8(self, register: int, value: int) -> None:
        self._bus.write_byte_data(self._address, register, value & 0xFF)

    def _read_block(self, register: int, length: int) -> list[int]:
        return self._bus.read_i2c_block_data(self._address, register, length)

    @staticmethod
    def _u16le(data: list[int], index: int) -> int:
        return data[index] | (data[index + 1] << 8)

    @staticmethod
    def _s16le(data: list[int], index: int) -> int:
        value = BME280Sensor._u16le(data, index)
        return value - 0x10000 if value & 0x8000 else value

    @staticmethod
    def _sign_extend_8(value: int) -> int:
        return value - 0x100 if value & 0x80 else value

    @staticmethod
    def _sign_extend_12(value: int) -> int:
        value &= 0xFFF
        return value - 0x1000 if value & 0x800 else value
