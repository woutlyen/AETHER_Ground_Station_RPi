"""
Sensor Data Processor

Listens on UDP port 6000 for sensor payloads from spi_demux.
Parses sensor packets and logs telemetry.
Ready to forward to external Graphical UI via UDP
"""

import socket
import json
import signal
import sys
import struct
import logging
import os

logging_level = os.getenv("LOGGING_LEVEL", "INFO").upper()

# Configure logging
logging.basicConfig(
    level=getattr(logging, logging_level, logging.INFO),
    format="%(asctime)s [SENSOR_PROCESSOR] [%(levelname)s] %(message)s",
)
logger = logging.getLogger(__name__)

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
CONFIG_FILE = os.path.join(SCRIPT_DIR, "config.json")

def load_config():
    """Load configuration from JSON file."""
    try:
        with open(CONFIG_FILE, "r") as f:
            return json.load(f)
    except Exception as e:
        logger.error(f"Error loading config: {e}")
        sys.exit(1)

config = load_config()

sensor_config = config.get("sensor_config", {})

# Load sensor definitions
sensor_lengths_str = sensor_config.get("sensor_data_lengths", {})
SENSOR_DATA_LENGTHS = {}
for key_str, value in sensor_lengths_str.items():
    try:
        key = int(key_str, 16)
        SENSOR_DATA_LENGTHS[key] = value
    except ValueError:
        pass

SENSOR_NAMES = {}
sensor_names_str = sensor_config.get("sensor_names", {})
for key_str, value in sensor_names_str.items():
    try:
        key = int(key_str, 16)
        SENSOR_NAMES[key] = value
    except ValueError:
        pass

LISTEN_PORT = 6000

SENSOR_DATA_OUTPUT_ADDRESS = config.get("sensor_data_output_address", "127.0.0.1")
SENSOR_DATA_OUTPUT_BASE_PORT = config.get("sensor_data_output_base_port", 7000)

# Create sensor sockets (SENSOR_DATA_OUTPUT_BASE_PORT + sensor_id)
sensor_socks = {}
for sensor_id in SENSOR_DATA_LENGTHS.keys():
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sensor_socks[sensor_id] = sock

shutdown_requested = False

def signal_handler(sig, frame):
    """Handle graceful shutdown."""
    global shutdown_requested
    shutdown_requested = True
    logger.info("Shutdown signal received")

signal.signal(signal.SIGTERM, signal_handler)
signal.signal(signal.SIGINT, signal_handler)


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


def bytes_to_uint16(msb: int, lsb: int) -> int:
    """Combine two bytes into unsigned 16-bit integer"""
    return (msb << 8) | lsb


def convert_sensor_data(sensor_id: int, data: bytes) -> list:
    """Convert raw sensor data to human-readable values"""
    values = []
    
    if sensor_id == 0x03:  # GNSS_POSITION
        # Latitude (uint24): mapped to -90° to +90° range
        # Formula: latitude_degrees = (value / 2^24) * 180 - 90
        lat_raw = (data[0] << 16) | (data[1] << 8) | data[2]
        lat_degrees = (lat_raw / (2 ** 24)) * 180 - 90
        
        # Longitude (uint24): mapped to -180° to +180° range
        # Formula: longitude_degrees = (value / 2^24) * 360 - 180
        lon_raw = (data[3] << 16) | (data[4] << 8) | data[5]
        lon_degrees = (lon_raw / (2 ** 24)) * 360 - 180
        
        # Altitude (uint16): scaled by factor of 1.5 m per LSB
        alt_raw = bytes_to_uint16(data[6], data[7])
        alt_meters = alt_raw * 1.5
        
        values = [lat_degrees, lon_degrees, alt_meters]
        
    elif sensor_id == 0x04:  # EPS_BATTERY
        # Current (uint16), Voltage (uint16)
        current = bytes_to_uint16(data[0], data[1])
        voltage = bytes_to_uint16(data[2], data[3])
        values = [current, voltage]
        
    elif sensor_id == 0x05:  # CS_STATUS
        # 7x uint8, 1x status byte with bit flags
        values = [
            data[0],  # CPU_Usage %
            data[1],  # CPU_Temp °C
            data[2],  # RAM_Usage %
            data[3],  # eMMC_Usage %
            data[4],  # SD_Usage %
            data[5],  # Cam1_RTP
            data[6],  # Cam2_RTP
            data[7],  # Status byte (bit flags)
        ]
        
    elif sensor_id == 0x07:  # IFS_ALTIMETER
        # Temperature (int32, /100 °C), Pressure (int32, /100 mbar)
        temp_raw = bytes_to_int32(data[0], data[1], data[2], data[3])
        press_raw = bytes_to_int32(data[4], data[5], data[6], data[7])
        values = [temp_raw / 100.0, press_raw / 100.0]
        
    elif sensor_id == 0x09:  # IFS_TCOUPLE
        # 4x Temperature (int16, 0.25/LSB °C)
        for i in range(4):
            temp_raw = bytes_to_int16(data[i*2], data[i*2+1])
            temp_celsius = temp_raw * 0.25
            values.append(temp_celsius)
        
    elif sensor_id == 0x11:  # IFS_TCOUPLE_INTERN
        # 4x Temperature (int16, 0.0625/LSB °C)
        for i in range(4):
            temp_raw = bytes_to_int16(data[i*2], data[i*2+1])
            temp_celsius = temp_raw * 0.0625
            values.append(temp_celsius)
        
    elif sensor_id == 0x13:  # IFS_TCOUPLE_ERROR
        # Error flags (bit-mapped)
        values = [data[0], data[1], data[2], data[3]]
        
    elif sensor_id == 0x14:  # IFS_STAGNATION
        # Temperature (int16) and Pressure (int16) conversions
        temp_raw = bytes_to_int16(data[0], data[1])
        press_raw = bytes_to_int16(data[2], data[3])
        
        temp_celsius = (temp_raw - 8192) * 4.272e-3
        
        calibration_offset = -0.08
        press_kpa = (press_raw - 3277) * 7.63e-4 - 10 + calibration_offset
        
        values = [temp_celsius, press_kpa]
        
    elif sensor_id == 0x15:  # IFS_BW_CURRENTS
        # 2x Current (uint16) conversion to Amperes
        current1_raw = bytes_to_uint16(data[0], data[1])
        current2_raw = bytes_to_uint16(data[2], data[3])
        
        current1_a = current1_raw * 3.3 * 1.9608 / (2 ** 12) if current1_raw != 0 else 0
        current2_a = current2_raw * 3.3 * 1.9608 / (2 ** 12) if current2_raw != 0 else 0
        
        values = [current1_a, current2_a]
        
    elif sensor_id == 0x16:  # IFS_CGG_CURRENTS
        # 2x Current (uint16) conversion to Amperes
        current1_raw = bytes_to_uint16(data[0], data[1])
        current2_raw = bytes_to_uint16(data[2], data[3])
        
        current1_a = current1_raw * 3.3 * 1.9608 / (2 ** 12) if current1_raw != 0 else 0
        current2_a = current2_raw * 3.3 * 1.9608 / (2 ** 12) if current2_raw != 0 else 0
        
        values = [current1_a, current2_a]
        
    elif sensor_id == 0x17:  # IFS_MANIFOLD
        # Manifold Pressure (uint16) conversion to kPa
        pressure_raw = bytes_to_uint16(data[0], data[1])
        
        vout = pressure_raw * 4.95 / (2 ** 12)
        pressure_kpa = ((vout / 5) - 0.04) / 0.0012858
        
        values = [pressure_kpa]
        
    elif sensor_id == 0x18:  # IFS_ACCELERATION
        # 3x Acceleration (int16) + Temperature (int16)
        accel_z = bytes_to_int16(data[0], data[1])
        accel_y = bytes_to_int16(data[2], data[3])
        accel_x = bytes_to_int16(data[4], data[5])
        temp = bytes_to_int16(data[6], data[7])
        values = [accel_z, accel_y, accel_x, temp]
        
    elif sensor_id == 0x20:  # IFS_ROTATION
        # 3x Rotation Rate (int16)
        yaw = bytes_to_int16(data[0], data[1])
        roll = bytes_to_int16(data[2], data[3])
        pitch = bytes_to_int16(data[4], data[5])
        values = [yaw, roll, pitch]
    
    return values


def parse_sensor_payload(buffer: bytes):
    """Parse sensor data messages from payload (ID + data pairs)"""
    sensors = []
    i = 0
    
    while i < len(buffer):
        if i >= len(buffer):
            break
        
        sensor_id = buffer[i]
        
        if sensor_id not in SENSOR_DATA_LENGTHS:
            break
        
        data_len = SENSOR_DATA_LENGTHS[sensor_id]
        
        if i + 1 + data_len > len(buffer):
            break
        
        data = buffer[i + 1:i + 1 + data_len]
        sensors.append((sensor_id, data))
        i += 1 + data_len
    
    return sensors

def format_sensor_data(sensor_id: int, data: bytes, values: list) -> str:
    """Format sensor data for debug printing"""
    name = SENSOR_NAMES.get(sensor_id, "UNKNOWN")
    value_str = ', '.join(f'{v:.4g}' if isinstance(v, float) else str(v) for v in values)
    return f"[{name:20s}] ID: 0x{sensor_id:02X} | {value_str}"

def main():
    """Main sensor processor loop."""
    logger.info(f"Sensor Processor starting - listening on UDP port {LISTEN_PORT}")

    try:
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.bind(("127.0.0.1", LISTEN_PORT))
        sock.settimeout(1.0)
        logger.info(f"Listening on UDP 127.0.0.1:{LISTEN_PORT}")
    except Exception as e:
        logger.error(f"Failed to bind UDP socket: {e}")
        sys.exit(1)

    try:
        while not shutdown_requested:
            try:
                payload, addr = sock.recvfrom(4096)
                
                if len(payload) == 0:
                    continue

                # Parse sensor data from payload
                sensors = parse_sensor_payload(payload)
                
                for sensor_id, sensor_raw_data in sensors:
                    values = convert_sensor_data(sensor_id, sensor_raw_data)

                    formatted_data = format_sensor_data(sensor_id, sensor_raw_data, values)

                    logger.info(formatted_data)
                    
                    # Create comma-separated ASCII string with '\n' at the end as delimiter
                    ascii_values = ','.join(f'{v:.6g}' if isinstance(v, float) else str(v) for v in values)
                    ascii_message = (ascii_values + '\n').encode('ascii')

                    # Send to external UI via UDP over sensor specific UDP port (SENSOR_DATA_OUTPUT_BASE_PORT + sensor_id)
                    output_port = SENSOR_DATA_OUTPUT_BASE_PORT + sensor_id
                    try:
                        sensor_socks[sensor_id].sendto(ascii_message, (SENSOR_DATA_OUTPUT_ADDRESS, output_port))
                    except Exception as e:
                        logger.error(f"Failed to send sensor data to {SENSOR_DATA_OUTPUT_ADDRESS}:{output_port}: {e}")

            except socket.timeout:
                continue
            except Exception as e:
                logger.error(f"Error processing data: {e}")
                continue

    except KeyboardInterrupt:
        logger.info("Interrupted")
    except Exception as e:
        logger.error(f"Processor error: {e}")
    finally:
        logger.info("Cleaning up...")
        try:
            sock.close()
        except:
            pass
        for sock in sensor_socks.values():
            try:
                sock.close()
            except:
                pass
        logger.info("Sensor processor shut down")


if __name__ == "__main__":
    main()
