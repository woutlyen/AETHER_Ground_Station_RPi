"""
SPI Data Demultiplexer

Receives multiplexed SPI data and forwards to appropriate processors.
Focus: Receive data, validate CRC, forward payload via UDP. Nothing else.

Routing:
  - Stream 0 (sensors) → UDP 127.0.0.1:6000 (sensor_processor)
  - Stream 1 (camera1) → UDP 127.0.0.1:6001 (udp_mjpeg)
  - Stream 2 (camera2) → UDP 127.0.0.1:6002 (udp_mjpeg)
  - etc.
"""

import spidev
import socket
import RPi.GPIO as GPIO
import json
import signal
import sys
import logging
import os

logging_level = os.getenv("LOGGING_LEVEL", "INFO").upper()

# Configure logging
logging.basicConfig(
    level=getattr(logging, logging_level, logging.INFO),
    format="%(asctime)s [SPI DEMULTIPLEXER] [%(levelname)s] %(message)s",
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
spi_config = config.get("spi_demux", {})

SPI_BUS = spi_config.get("bus", 0)
SPI_DEVICE = spi_config.get("device", 0)
SPI_SPEED = spi_config.get("speed", 3000000)
DRDY_PIN = spi_config.get("drdy_pin", 25)
CHUNK_SIZE = 1024

# UDP forwarding setup
FORWARD_BASE_ADDRESS = "127.0.0.1"
FORWARD_SENSOR_PORT = 6000  # Stream 0
FORWARD_CAMERA_BASE_PORT = 6001  # Stream 1 → 6001, Stream 2 → 6002, etc.

# Initialize SPI
try:
    spi = spidev.SpiDev()
    spi.open(SPI_BUS, SPI_DEVICE)
    spi.max_speed_hz = SPI_SPEED
    spi.mode = 0

    logger.info(
        f"SPI initialized: bus={SPI_BUS}, device={SPI_DEVICE}, speed={SPI_SPEED}"
    )
except Exception as e:
    logger.error(f"Failed to initialize SPI: {e}")
    sys.exit(1)

# Initialize GPIO
GPIO.setmode(GPIO.BCM)
GPIO.setup(DRDY_PIN, GPIO.IN)

# Create UDP socket for forwarding
forward_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

shutdown_requested = False

def signal_handler(sig, frame):
    """Handle graceful shutdown."""
    global shutdown_requested
    shutdown_requested = True
    logger.info("Shutdown signal received")

signal.signal(signal.SIGTERM, signal_handler)
signal.signal(signal.SIGINT, signal_handler)


def stm32_crc32(data: bytes) -> int:
    """Calculate STM32 CRC-32"""
    crc = 0xFFFFFFFF
    for byte in data:
        crc ^= (byte << 24)
        for _ in range(8):
            if crc & 0x80000000:
                crc = (crc << 1) ^ 0x04C11DB7
            else:
                crc <<= 1
            crc &= 0xFFFFFFFF
    return crc


def parse_and_forward(buffer):
    """Parse SPI buffer, validate CRC, forward payloads via UDP"""
    i = 0
    length_buf = len(buffer)

    while i < length_buf:
        # End-of-data marker
        if buffer[i] == 0x00:
            break

        # Need at least minimal packet + info bytes
        if i + 8 > length_buf:
            break

        length = buffer[i]
        total_size = length + 2  # include CC1200 info bytes

        if i + total_size > length_buf:
            break

        stream_id = buffer[i + 1]
        payload_end = i + length - 4
        crc_received = int.from_bytes(
            buffer[payload_end:i + length], 'big'
        )

        header_payload = buffer[i:payload_end]
        crc_calc = stm32_crc32(header_payload)

        if crc_calc != crc_received:
            logger.warning(
                f"CRC validation failed for stream {stream_id}: "
                f"expected={crc_received:08x}, calculated={crc_calc:08x}"
            )
            i += total_size
            continue

        # CRC OK - extract and forward payload
        payload = buffer[i + 2:payload_end]

        # Determine destination port based on stream_id
        if stream_id == 0:
            # Sensor data
            dest_port = FORWARD_SENSOR_PORT
            logger.debug(
                f"Forwarded sensor payload ({len(payload)} bytes) "
                f"to UDP port {dest_port}"
            )
        else:
            # Camera data (stream_id 1, 2, etc.)
            dest_port = FORWARD_CAMERA_BASE_PORT + stream_id - 1
            logger.debug(
                f"Forwarded camera stream {stream_id} payload "
                f"({len(payload)} bytes) to UDP port {dest_port}"
            )

        # Forward payload via UDP
        try:
            forward_sock.sendto(payload, (FORWARD_BASE_ADDRESS, dest_port))
        except Exception as e:
            logger.error(
                f"Failed to forward payload to UDP port {dest_port}: {e}"
            )

        i += total_size


logger.info("SPI demultiplexer started")
logger.info(
    f"SPI: bus={SPI_BUS}, device={SPI_DEVICE}, speed={SPI_SPEED}"
)
logger.info(f"GPIO DRDY pin: {DRDY_PIN}")
logger.info(
    f"Sensor forwarding: {FORWARD_BASE_ADDRESS}:{FORWARD_SENSOR_PORT}"
)
logger.info(
    f"Camera forwarding base port: {FORWARD_CAMERA_BASE_PORT}"
)

try:
    while not shutdown_requested:
        # Wait for DRDY signal
        while GPIO.input(DRDY_PIN) == 0 and not shutdown_requested:
            pass

        if shutdown_requested:
            break

        # Read SPI data
        raw = spi.xfer2([0x00] * CHUNK_SIZE)
        buffer = bytes(raw)

        # Parse, validate CRC, and forward
        parse_and_forward(buffer)

        # Wait for DRDY low
        while GPIO.input(DRDY_PIN) == 1 and not shutdown_requested:
            pass

except KeyboardInterrupt:
    pass
finally:
    logger.info("Shutting down SPI demultiplexer...")
    spi.close()
    GPIO.cleanup()
    forward_sock.close()
    logger.info("SPI demultiplexer shut down")
