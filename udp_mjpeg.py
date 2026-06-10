"""
UDP MJPEG Stream Processor

Converts H264 RTP stream to MJPEG for UI display.
Receives H264 data on local UDP port, transcodes to MJPEG, outputs to network.
Managed by supervisor - always enabled.
"""

import gi
import sys
import signal
import socket
import time
import logging
import json
import os

gi.require_version('Gst', '1.0')
from gi.repository import Gst, GLib

Gst.init(None)

logging_level = os.getenv("LOGGING_LEVEL", "INFO").upper()

# Configure logging
logging.basicConfig(
    level=getattr(logging, logging_level, logging.INFO),
    format="%(asctime)s [UDP MJPEG] [%(levelname)s] %(message)s",
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

# Parse command line arguments
if len(sys.argv) < 5:
    logger.error(
    "Usage: udp_mjpeg.py <udp_port_in> <udp_address_out> "
    "<udp_port_out> <label> [quality]"
    )
    logger.error(
        "Example: udp_mjpeg.py 6001 127.0.0.1 7001 camera1 50"
    )
    sys.exit(1)

udp_port_in = sys.argv[1]
udp_address_out = sys.argv[2]
udp_port_out = sys.argv[3]
label = sys.argv[4]
quality = int(sys.argv[5]) if len(sys.argv) > 5 else 50

# Clamp quality to valid range (1-100)
quality = max(1, min(100, quality))

frame_count = 0
fps = 0
last_time = time.time()
frames_in_window = 0

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)


def send_metadata():
    global fps, frame_count

    # Format: AB CD EF <fps>,<frame_count> FE ED
    payload = f"{fps},{frame_count}".encode()
    packet = b'\xAB\xCD\xEF' + payload + b'\xFE\xED'

    try:
        sock.sendto(packet, (udp_address_out, int(udp_port_out)))
    except Exception as e:
        logger.error(
            f"{label}: Failed to send metadata: {e}"
        )


def send_frame(frame_bytes):
    # logger.info(f"JPEG size = {len(frame_bytes)} bytes")
    max_payload = 8192*4
    for i in range(0, len(frame_bytes), max_payload):
        chunk = frame_bytes[i:i + max_payload]
        try:
            sock.sendto(chunk, (udp_address_out, int(udp_port_out)))
        except Exception as e:
            logger.error(
                f"{label}: Failed to send MJPEG frame chunk: {e}"
            )


def on_new_sample(appsink):
    global frame_count, fps, last_time, frames_in_window
    
    try:
        sample = appsink.emit("pull-sample")
        buffer = sample.get_buffer()

        success, map_info = buffer.map(Gst.MapFlags.READ)
        if not success:
            return Gst.FlowReturn.ERROR

        try:
            frame_data = map_info.data
            
            # Send frame
            send_frame(frame_data)

            # Update counters
            frame_count += 1
            frames_in_window += 1

            now = time.time()
            if now - last_time >= 1.0:
                fps = frames_in_window
                logger.info(
                    f"{label}: FPS={fps}, total_frames={frame_count}"
                )
                frames_in_window = 0
                last_time = now

            # Send metadata packet
            send_metadata()

        finally:
            buffer.unmap(map_info)

        return Gst.FlowReturn.OK
    except Exception as e:
        logger.error(
            f"{label}: Error processing sample: {e}"
        )
        return Gst.FlowReturn.ERROR


# Create GStreamer pipeline
pipeline = Gst.parse_launch(f"""
udpsrc port={udp_port_in} !
application/x-rtp, encoding-name=H264, payload=96 !
rtpjitterbuffer !
rtph264depay !
h264parse !
avdec_h264 !
jpegenc quality={quality} !
appsink name=sink emit-signals=true sync=false max-buffers=1 drop=true
""")

appsink = pipeline.get_by_name("sink")
appsink.connect("new-sample", on_new_sample)


def shutdown_pipeline(sig, frame):
    logger.info(
        f"{label}: Shutting down gracefully..."
    )
    pipeline.set_state(Gst.State.NULL)
    sock.close()
    sys.exit(0)


signal.signal(signal.SIGINT, shutdown_pipeline)
signal.signal(signal.SIGTERM, shutdown_pipeline)

logger.info(
    f"{label}: Starting MJPEG transcoder (quality={quality})"
)

logger.info(
    f"{label}: Input RTP port={udp_port_in}"
)

logger.info(
    f"{label}: Output MJPEG={udp_address_out}:{udp_port_out}"
)

ret = pipeline.set_state(Gst.State.PLAYING)
if ret == Gst.StateChangeReturn.FAILURE:
    logger.error(
        f"{label}: Failed to set pipeline to PLAYING state"
    )
    sys.exit(1)

logger.info(
    f"{label}: Pipeline started successfully"
)

try:
    bus = pipeline.get_bus()
    msg = bus.timed_pop_filtered(
        Gst.CLOCK_TIME_NONE,
        Gst.MessageType.ERROR | Gst.MessageType.EOS
    )
    
    if msg:
        if msg.type == Gst.MessageType.ERROR:
            err, debug = msg.parse_error()
            logger.error(
                f"{label}: ERROR: {err.message}"
            )
            logger.debug(
                f"{label}: Debug: {debug}"
            )
except KeyboardInterrupt:
    pass
except Exception as e:
    logger.error(
        f"{label}: Unexpected exception: {e}"
    )
finally:
    pipeline.set_state(Gst.State.NULL)
    sock.close()
    logger.info(
        f"{label}: Shut down"
    )
    sys.exit(1)
