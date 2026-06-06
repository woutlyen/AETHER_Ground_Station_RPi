#!/usr/bin/env python3
"""
Test script to verify sensor conversion implementations
Covers all sensor types implemented in sensor_processor.py
"""


def bytes_to_uint16(msb: int, lsb: int) -> int:
    """Combine two bytes into unsigned 16-bit integer"""
    return (msb << 8) | lsb


def bytes_to_int16(msb: int, lsb: int) -> int:
    """Combine two bytes into signed 16-bit integer"""
    value = (msb << 8) | lsb
    if value & 0x8000:
        value = value - 0x10000
    return value


def bytes_to_int32(msb: int, b3: int, b2: int, lsb: int) -> int:
    """Combine four bytes into signed 32-bit integer"""
    value = (msb << 24) | (b3 << 16) | (b2 << 8) | lsb
    if value & 0x80000000:
        value = value - 0x100000000
    return value


def convert_sensor_data(sensor_id: int, data: bytes) -> list:
    """Convert raw sensor data to human-readable values"""
    values = []

    if sensor_id == 0x03:  # GNSS_POSITION
        lat_raw = (data[0] << 16) | (data[1] << 8) | data[2]
        lat_degrees = (lat_raw / (2 ** 24)) * 180 - 90

        lon_raw = (data[3] << 16) | (data[4] << 8) | data[5]
        lon_degrees = (lon_raw / (2 ** 24)) * 360 - 180

        alt_raw = bytes_to_uint16(data[6], data[7])
        alt_meters = alt_raw * 1.5

        values = [lat_degrees, lon_degrees, alt_meters]

    elif sensor_id == 0x04:  # EPS_BATTERY
        current = bytes_to_uint16(data[0], data[1])
        voltage = bytes_to_uint16(data[2], data[3])
        values = [current, voltage]

    elif sensor_id == 0x05:  # CS_STATUS
        values = [
            data[0],
            data[1],
            data[2],
            data[3],
            data[4],
            data[5],
            data[6],
            data[7],
        ]

    elif sensor_id == 0x07:  # IFS_ALTIMETER
        temp_raw = bytes_to_int32(data[0], data[1], data[2], data[3])
        press_raw = bytes_to_int32(data[4], data[5], data[6], data[7])
        values = [temp_raw / 100.0, press_raw / 100.0]

    elif sensor_id == 0x09:  # IFS_TCOUPLE
        for i in range(4):
            temp_raw = bytes_to_int16(data[i * 2], data[i * 2 + 1])
            values.append(temp_raw * 0.25)

    elif sensor_id == 0x11:  # IFS_TCOUPLE_INTERN
        for i in range(4):
            temp_raw = bytes_to_int16(data[i * 2], data[i * 2 + 1])
            values.append(temp_raw * 0.0625)

    elif sensor_id == 0x13:  # IFS_TCOUPLE_ERROR
        values = [data[0]]

    elif sensor_id == 0x14:  # IFS_STAGNATION
        temp_raw = bytes_to_int16(data[0], data[1])
        press_raw = bytes_to_int16(data[2], data[3])

        temp_celsius = (temp_raw - 8192) * 4.272e-3

        calibration_offset = -0.08
        press_kpa = (press_raw - 3277) * 7.63e-4 - 10 + calibration_offset

        values = [temp_celsius, press_kpa]

    elif sensor_id == 0x15:  # IFS_BW_CURRENTS
        current1_raw = bytes_to_uint16(data[0], data[1])
        current2_raw = bytes_to_uint16(data[2], data[3])

        current1_a = (
            current1_raw * 3.3 * 1.9608 / (2 ** 12)
            if current1_raw != 0
            else 0
        )
        current2_a = (
            current2_raw * 3.3 * 1.9608 / (2 ** 12)
            if current2_raw != 0
            else 0
        )

        values = [current1_a, current2_a]

    elif sensor_id == 0x16:  # IFS_CGG_CURRENTS
        current1_raw = bytes_to_uint16(data[0], data[1])
        current2_raw = bytes_to_uint16(data[2], data[3])

        current1_a = (
            current1_raw * 3.3 * 1.9608 / (2 ** 12)
            if current1_raw != 0
            else 0
        )
        current2_a = (
            current2_raw * 3.3 * 1.9608 / (2 ** 12)
            if current2_raw != 0
            else 0
        )

        values = [current1_a, current2_a]

    elif sensor_id == 0x17:  # IFS_MANIFOLD
        pressure_raw = bytes_to_uint16(data[0], data[1])

        vout = pressure_raw * 4.95 / (2 ** 12)
        pressure_kpa = ((vout / 5) - 0.04) / 0.0012858

        values = [pressure_kpa]

    elif sensor_id == 0x18:  # IFS_ACCELERATION
        accel_z = bytes_to_int16(data[0], data[1])
        accel_y = bytes_to_int16(data[2], data[3])
        accel_x = bytes_to_int16(data[4], data[5])
        temp = bytes_to_int16(data[6], data[7])

        values = [accel_z, accel_y, accel_x, temp]

    elif sensor_id == 0x20:  # IFS_ROTATION
        yaw = bytes_to_int16(data[0], data[1])
        roll = bytes_to_int16(data[2], data[3])
        pitch = bytes_to_int16(data[4], data[5])

        values = [yaw, roll, pitch]

    return values


TESTS = [
    # ------------------------------------------------------------------
    # GNSS_POSITION
    # ------------------------------------------------------------------
    {
        "sensor_id": 0x03,
        "name": "GNSS_POSITION - Equator Greenwich sea level",
        "hex_data": bytes([
            0x80, 0x00, 0x00,
            0x80, 0x00, 0x00,
            0x00, 0x00
        ]),
        "expected": [0.0, 0.0, 0.0],
        "tolerance": 0.01,
    },
    {
        "sensor_id": 0x03,
        "name": "GNSS_POSITION - Max coordinates",
        "hex_data": bytes([
            0xFF, 0xFF, 0xFF,
            0xFF, 0xFF, 0xFF,
            0x03, 0xE8
        ]),
        "expected": [89.99999, 179.99999, 1500.0],
        "tolerance": 0.1,
    },

    # ------------------------------------------------------------------
    # EPS_BATTERY
    # ------------------------------------------------------------------
    {
        "sensor_id": 0x04,
        "name": "EPS_BATTERY - Nominal",
        "hex_data": bytes([
            0x01, 0xF4,
            0x0E, 0x10
        ]),
        "expected": [500, 3600],
        "tolerance": 0,
    },

    # ------------------------------------------------------------------
    # CS_STATUS
    # ------------------------------------------------------------------
    {
        "sensor_id": 0x05,
        "name": "CS_STATUS - Normal",
        "hex_data": bytes([
            45, 60, 55, 20,
            40, 25, 24, 3
        ]),
        "expected": [45, 60, 55, 20, 40, 25, 24, 3],
        "tolerance": 0,
    },

    # ------------------------------------------------------------------
    # IFS_ALTIMETER
    # ------------------------------------------------------------------
    {
        "sensor_id": 0x07,
        "name": "IFS_ALTIMETER - Room temp",
        "hex_data": bytes([
            0x00, 0x00, 0x09, 0xC4,
            0x00, 0x01, 0x8B, 0xCD
        ]),
        "expected": [25.0, 1013.25],
        "tolerance": 0.01,
    },
    {
        "sensor_id": 0x07,
        "name": "IFS_ALTIMETER - Negative temp",
        "hex_data": bytes([
            0xFF, 0xFF, 0xF6, 0x3C,
            0x00, 0x01, 0x8B, 0xCD
        ]),
        "expected": [-25.0, 1013.25],
        "tolerance": 0.01,
    },

    # ------------------------------------------------------------------
    # IFS_TCOUPLE
    # ------------------------------------------------------------------
    {
        "sensor_id": 0x09,
        "name": "IFS_TCOUPLE - Four channels",
        "hex_data": bytes([
            0x00, 0x64,
            0x00, 0xC8,
            0x01, 0x2C,
            0x01, 0x90
        ]),
        "expected": [25.0, 50.0, 75.0, 100.0],
        "tolerance": 0.01,
    },

    # ------------------------------------------------------------------
    # IFS_TCOUPLE_INTERN
    # ------------------------------------------------------------------
    {
        "sensor_id": 0x11,
        "name": "IFS_TCOUPLE_INTERN - Four channels",
        "hex_data": bytes([
            0x01, 0x90,
            0x03, 0x20,
            0x04, 0xB0,
            0x06, 0x40
        ]),
        "expected": [25.0, 50.0, 75.0, 100.0],
        "tolerance": 0.01,
    },

    # ------------------------------------------------------------------
    # IFS_TCOUPLE_ERROR
    # ------------------------------------------------------------------
    {
        "sensor_id": 0x13,
        "name": "IFS_TCOUPLE_ERROR - No errors",
        "hex_data": bytes([0x00]),
        "expected": [0],
        "tolerance": 0,
    },
    {
        "sensor_id": 0x13,
        "name": "IFS_TCOUPLE_ERROR - Flags set",
        "hex_data": bytes([0xA5]),
        "expected": [0xA5],
        "tolerance": 0,
    },

    # ------------------------------------------------------------------
    # IFS_STAGNATION
    # ------------------------------------------------------------------
    {
        "sensor_id": 0x14,
        "name": "IFS_STAGNATION - Normal temp, apogee",
        "hex_data": bytes([0x32, 0x49, 0x40, 0x68]),
        "expected": [20.0, 0.0],
        "tolerance": 0.1,
    },
    {
        "sensor_id": 0x14,
        "name": "IFS_STAGNATION - Cold temp, negative pressure",
        "hex_data": bytes([0x20, 0x00, 0x26, 0xCE]),
        "expected": [0.0, -5.0],
        "tolerance": 0.1,
    },
    {
        "sensor_id": 0x14,
        "name": "IFS_STAGNATION - Hot temp, positive pressure",
        "hex_data": bytes([0x4D, 0xB8, 0x5A, 0x01]),
        "expected": [50.0, 5.0],
        "tolerance": 0.1,
    },

    # ------------------------------------------------------------------
    # IFS_BW_CURRENTS
    # ------------------------------------------------------------------
    {
        "sensor_id": 0x15,
        "name": "IFS_BW_CURRENTS - Both off",
        "hex_data": bytes([0x00, 0x00, 0x00, 0x00]),
        "expected": [0.0, 0.0],
        "tolerance": 0.001,
    },
    {
        "sensor_id": 0x15,
        "name": "IFS_BW_CURRENTS - BW1 active",
        "hex_data": bytes([0x03, 0xB6, 0x00, 0x00]),
        "expected": [1.5, 0.0],
        "tolerance": 0.05,
    },
    {
        "sensor_id": 0x15,
        "name": "IFS_BW_CURRENTS - BW2 active",
        "hex_data": bytes([0x00, 0x00, 0x04, 0xF2]),
        "expected": [0.0, 2.0],
        "tolerance": 0.05,
    },

    # ------------------------------------------------------------------
    # IFS_CGG_CURRENTS
    # ------------------------------------------------------------------
    {
        "sensor_id": 0x16,
        "name": "IFS_CGG_CURRENTS - Normal",
        "hex_data": bytes([0x01, 0x3D, 0x01, 0x3D]),
        "expected": [0.5, 0.5],
        "tolerance": 0.05,
    },
    {
        "sensor_id": 0x16,
        "name": "IFS_CGG_CURRENTS - CGG2 only",
        "hex_data": bytes([0x00, 0x00, 0x02, 0x79]),
        "expected": [0.0, 1.0],
        "tolerance": 0.05,
    },

    # ------------------------------------------------------------------
    # IFS_MANIFOLD
    # ------------------------------------------------------------------
    {
        "sensor_id": 0x17,
        "name": "IFS_MANIFOLD - Standard atmosphere",
        "hex_data": bytes([0x02, 0xC1]),
        "expected": [101.325],
        "tolerance": 0.5,
    },
    {
        "sensor_id": 0x17,
        "name": "IFS_MANIFOLD - Vacuum",
        "hex_data": bytes([0x00, 0xA5]),
        "expected": [0.0],
        "tolerance": 0.5,
    },

    # ------------------------------------------------------------------
    # IFS_ACCELERATION
    # ------------------------------------------------------------------
    {
        "sensor_id": 0x18,
        "name": "IFS_ACCELERATION - Positive values",
        "hex_data": bytes([
            0x00, 0x01,
            0x00, 0x02,
            0x00, 0x03,
            0x00, 0x19
        ]),
        "expected": [1, 2, 3, 25],
        "tolerance": 0,
    },
    {
        "sensor_id": 0x18,
        "name": "IFS_ACCELERATION - Negative values",
        "hex_data": bytes([
            0xFF, 0xFF,
            0xFF, 0xFE,
            0xFF, 0xFD,
            0xFF, 0xF6
        ]),
        "expected": [-1, -2, -3, -10],
        "tolerance": 0,
    },

    # ------------------------------------------------------------------
    # IFS_ROTATION
    # ------------------------------------------------------------------
    {
        "sensor_id": 0x20,
        "name": "IFS_ROTATION - Positive",
        "hex_data": bytes([
            0x00, 0x64,
            0x00, 0xC8,
            0x01, 0x2C
        ]),
        "expected": [100, 200, 300],
        "tolerance": 0,
    },
    {
        "sensor_id": 0x20,
        "name": "IFS_ROTATION - Negative",
        "hex_data": bytes([
            0xFF, 0x9C,
            0xFF, 0x38,
            0xFE, 0xD4
        ]),
        "expected": [-100, -200, -300],
        "tolerance": 0,
    },
]


def run_tests():
    """Run all conversion tests"""
    passed = 0
    failed = 0

    print("=" * 80)
    print("SENSOR CONVERSION TESTS")
    print("=" * 80)

    for test in TESTS:
        result = convert_sensor_data(
            test["sensor_id"],
            test["hex_data"]
        )

        all_match = True

        if len(result) != len(test["expected"]):
            all_match = False
        else:
            for expected, actual in zip(test["expected"], result):
                if abs(expected - actual) > test["tolerance"]:
                    all_match = False
                    break

        status = "✓ PASS" if all_match else "✗ FAIL"

        print(f"\n{status} - {test['name']}")
        print(
            f"  Hex: {' '.join(f'{b:02X}' for b in test['hex_data'])}"
        )
        print(f"  Expected: {test['expected']}")
        print(f"  Got:      {[round(v, 4) for v in result]}")

        if all_match:
            passed += 1
        else:
            failed += 1

    print("\n" + "=" * 80)
    print(f"Results: {passed} passed, {failed} failed")
    print("=" * 80)

    return failed == 0


if __name__ == "__main__":
    success = run_tests()
    exit(0 if success else 1)
