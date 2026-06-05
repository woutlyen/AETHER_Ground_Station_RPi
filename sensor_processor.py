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

# Configure logging
logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s [SENSOR] %(message)s",
)
logger = logging.getLogger(__name__)

def load_config(config_path="config.json"):
    """Load configuration from JSON file."""
    try:
        with open(config_path, "r") as f:
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


def parse_sensor_data(sensor_id: int, data: bytes) -> list:
    """Parse raw sensor data to readable values"""
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
        values = [data[i] for i in range(7)] + [data[7]]
    elif sensor_id == 0x07:  # IFS_ALTIMETER
        temp_raw = bytes_to_int32(data[0], data[1], data[2], data[3])
        press_raw = bytes_to_int32(data[4], data[5], data[6], data[7])
        values = [temp_raw / 100.0, press_raw / 100.0]
    elif sensor_id == 0x09:  # IFS_TCOUPLE
        for i in range(4):
            temp_raw = bytes_to_int16(data[i*2], data[i*2+1])
            values.append(temp_raw * 0.25)
    elif sensor_id == 0x11:  # IFS_TCOUPLE_INTERN
        for i in range(4):
            temp_raw = bytes_to_int16(data[i*2], data[i*2+1])
            values.append(temp_raw * 0.0625)
    elif sensor_id == 0x13:  # IFS_TCOUPLE_ERROR
        values = [data[0]]
    elif sensor_id == 0x14:  # IFS_STAGNATION
        temp_raw = bytes_to_int16(data[0], data[1])
        press_raw = bytes_to_int16(data[2], data[3])
        values = [temp_raw, press_raw]
    elif sensor_id == 0x15:  # IFS_BW_CURRENTS
        current1 = bytes_to_uint16(data[0], data[1])
        current2 = bytes_to_uint16(data[2], data[3])
        values = [current1, current2]
    elif sensor_id == 0x16:  # IFS_CGG_CURRENTS
        current1 = bytes_to_uint16(data[0], data[1])
        current2 = bytes_to_uint16(data[2], data[3])
        values = [current1, current2]
    elif sensor_id == 0x17:  # IFS_MANIFOLD
        pressure = bytes_to_uint16(data[0], data[1])
        values = [pressure]
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
                    values = parse_sensor_data(sensor_id, sensor_raw_data)

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
