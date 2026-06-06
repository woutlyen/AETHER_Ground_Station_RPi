"""
System Supervisor

Always-on process manager for AETHER ground station.
Manages:
  - spi_demux: SPI receiver, CRC validator, UDP forwarder
  - sensor_processor: UDP 6000 listener, sensor parser
  - udp_mjpeg: MJPEG transcoders (one per camera stream)

All services always enabled. Single entry point.
"""

import subprocess
import time
import signal
import sys
import json
import os
import logging

# Configure logging
logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s [SUPERVISOR] [%(levelname)s] %(message)s",
)
logger = logging.getLogger(__name__)

running = {}  # {process_name: subprocess.Popen}

def load_config(config_path="config.json"):
    """Load configuration from JSON file."""
    try:
        with open(config_path, "r") as f:
            return json.load(f)
    except Exception as e:
        logger.error(f"Error loading config: {e}")
        sys.exit(1)

config = load_config()

LOGGING_LEVEL = config.get("features", {}).get("logging_level", "INFO").upper()
if LOGGING_LEVEL in ["DEBUG", "INFO", "WARNING", "ERROR", "CRITICAL"]:
    logger.setLevel(getattr(logging, LOGGING_LEVEL))
else:
    logger.warning(f"Invalid logging level '{LOGGING_LEVEL}' in config, defaulting to INFO")
    logger.setLevel(logging.INFO)


def graceful_shutdown(sig, frame):
    """Handle graceful shutdown on SIGINT/SIGTERM."""
    logger.info("Shutting down all processes gracefully...")
    for name, proc in running.items():
        try:
            proc.terminate()
            proc.wait(timeout=10)
            logger.info(f"Stopped {name}")
        except subprocess.TimeoutExpired:
            proc.kill()
            logger.warning(f"Killed {name} (timeout)")
        except Exception as e:
            logger.error(f"Error terminating {name}: {e}")

    sys.exit(0)


signal.signal(signal.SIGINT, graceful_shutdown)
signal.signal(signal.SIGTERM, graceful_shutdown)


def start_process(name, cmd):
    """Start a subprocess with error handling."""
    try:
        env = os.environ.copy()
        running[name] = subprocess.Popen(cmd, env=env)
        logger.info(f"Started {name} (PID: {running[name].pid})")
    except Exception as e:
        logger.error(f"Failed to start {name}: {e}")


def stop_process(name):
    """Stop a subprocess gracefully."""
    if name not in running:
        return

    try:
        running[name].terminate()
        running[name].wait(timeout=10)
        logger.info(f"Stopped {name}")
    except subprocess.TimeoutExpired:
        running[name].kill()
        logger.warning(f"Killed {name} (timeout)")
    except Exception as e:
        logger.error(f"Error stopping {name}: {e}")
    finally:
        if name in running:
            del running[name]


def manage_processes(config):
    """
    Manage all core services.
    Always starts: spi_demux, sensor_processor, MJPEG transcoders

    Args:
        config: Full configuration dict
    """
    desired_processes = {}
    script_dir = os.path.dirname(os.path.abspath(__file__))

    # ====================================================================
    # SPI DEMULTIPLEXER (always enabled)
    # ====================================================================

    desired_processes["spi_demux"] = [
        "/usr/bin/python3",
        os.path.join(script_dir, "spi_demux.py")
    ]

    # ====================================================================
    # SENSOR PROCESSOR (always enabled)
    # Listens on UDP 127.0.0.1:6000
    # ====================================================================

    desired_processes["sensor_processor"] = [
        "/usr/bin/python3",
        os.path.join(script_dir, "sensor_processor.py")
    ]

    # ====================================================================
    # MJPEG TRANSCODERS (always enabled, one per camera stream)
    # Listens on UDP 127.0.0.1:6001, 6002, etc.
    # ====================================================================

    for stream in config.get("camera_streams", []):
        name = f"mjpeg_{stream['name']}"
        # UDP port = 6000 + stream_id
        rtp_port = 6000 + stream["stream_id"]
        
        desired_processes[name] = [
            "/usr/bin/python3",
            os.path.join(script_dir, "udp_mjpeg.py"),
            str(rtp_port),
            stream["mjpeg_output_address"],
            str(stream["mjpeg_output_port"]),
            stream["name"],
            str(stream.get("quality", 50))
        ]

    # ====================================================================
    # START / STOP LOGIC
    # ====================================================================

    # Start desired processes
    for name, cmd in desired_processes.items():
        if name not in running or running[name].poll() is not None:
            start_process(name, cmd)

    # Stop undesired processes (shouldn't happen with fixed config)
    for name in list(running.keys()):
        if name not in desired_processes:
            stop_process(name)


def main():
    """Main supervisor loop."""
    logger.info("System Supervisor starting (always-on mode)...")

    config = load_config()
    if config is None:
        logger.error("Failed to load config on startup")
        sys.exit(1)

    num_cameras = len(config.get("camera_streams", []))
    logger.info(f"Config loaded: {num_cameras} camera stream(s)")
    logger.info("Services: spi_demux + sensor_processor + mjpeg transcoders")

    try:
        while True:
            config = load_config()
            if config is None:
                logger.error("Failed to load config, retrying...")
                time.sleep(5)
                continue

            manage_processes(config)
            time.sleep(1)

    except Exception as e:
        logger.error(f"Supervisor error: {e}")
        sys.exit(1)


if __name__ == "__main__":
    main()
