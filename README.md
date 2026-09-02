# BME280_Mu

Python 3.14-compatible BME280 test program for LattePanda Mu over I2C.

## Files

- `bme280_sensor.py` - core BME280 driver
- `test_bme280.py` - command-line test tool
- `requirements.txt` - Python dependency list

## Hardware

- LattePanda Mu with Lite Carrier board
- BME280 wired to the 3.3V I2C pins
- I2C address `0x76` or `0x77`

## Setup

```bash
python3.14 -m venv .venv
source .venv/bin/activate
pip install -r requirements.txt
```

Enable I2C on the device if it is not already available, then connect the sensor to the 3.3V I2C bus.

## Usage

Detect the sensor:

```bash
python test_bme280.py --detect
```

Read one measurement:

```bash
python test_bme280.py --once
```

Read continuously every 2 seconds:

```bash
python test_bme280.py --interval 2
```

Read 10 samples with debug logging:

```bash
python test_bme280.py --count 10 --log-level DEBUG
```

## Output

The tool prints:

- Temperature in Celsius and Fahrenheit
- Humidity in percent
- Pressure in hPa
- Raw ADC values for troubleshooting

## Notes

- The driver reads the BME280 factory calibration registers and applies the Bosch compensation formulas.
- If the sensor is not detected, the script exits with a non-zero status.
- The implementation uses `smbus2`, which is a good fit for Python 3.14 on Linux-based I2C systems such as LattePanda Mu.
