# AETHER Ground Station (Raspberry Pi)

Receiver-side software for the AETHER ground station. It runs on a Raspberry Pi
connected to the downlink receiver over SPI, demultiplexes the incoming telemetry
and video streams, converts the raw sensor words into physical units, transcodes
the H.264 camera streams to MJPEG, and forwards everything over UDP to the
ground station UI.

## Architecture

```
      ┌─────────────┐
      │  Receiver   │  (SPI master data source, DRDY on GPIO)
      └──────┬──────┘
             │ SPI + DRDY
      ┌──────▼──────────────────────────────────────────┐
      │ spi_demux.py                                    │
      │  read chunk → parse packets → verify STM32 CRC32│
      └──┬────────────────────────────┬─────────────────┘
         │ stream 0                   │ stream 1..N
         │ UDP 127.0.0.1:6000         │ UDP 127.0.0.1:6001, 6002, ...
   ┌─────▼──────────────┐      ┌──────▼──────────────┐
   │ sensor_processor.py│      │ udp_mjpeg.py (xN)   │
   │ ID+data parsing,   │      │ GStreamer:          │
   │ unit conversion    │      │ RTP/H.264 → JPEG    │
   └─────┬──────────────┘      └──────┬──────────────┘
         │ ASCII CSV per sensor       │ JPEG chunks + metadata
         │ UDP <addr>:7000+sensor_id  │ UDP <addr>:<mjpeg_output_port>
         └────────────┬───────────────┘
                      ▼
              Ground station UI
```

All processes are started and kept alive by `supervisor.py`, which is itself run
as a systemd service.

## Components

| File | Role |
| --- | --- |
| [supervisor.py](supervisor.py) | Single entry point. Starts and restarts `spi_demux`, `sensor_processor` and one `udp_mjpeg` per configured camera stream; handles graceful shutdown on SIGINT/SIGTERM. |
| [spi_demux.py](spi_demux.py) | Waits for the DRDY GPIO, reads a 1024-byte SPI chunk, walks the packet framing, validates the STM32 CRC-32 and forwards the payload to a local UDP port chosen by stream ID. |
| [sensor_processor.py](sensor_processor.py) | Listens on UDP 6000, splits the payload into `sensor_id + data` records, converts raw values to physical units and emits one ASCII CSV line per sensor to a per-sensor UDP port. |
| [udp_mjpeg.py](udp_mjpeg.py) | One instance per camera. GStreamer pipeline `udpsrc → rtpjitterbuffer → rtph264depay → h264parse → avdec_h264 → jpegenc → appsink`, sending JPEG frames in 32 KiB chunks plus an FPS/frame-count metadata packet. |
| [config.json](config.json) | All runtime configuration (see below). |
| [test_sensor_conversions.py](test_sensor_conversions.py) | Standalone check of the sensor conversion formulas against known hex payloads. |
| [services/groundstation.service](services/groundstation.service) | systemd unit for the supervisor. |

## Port map

| Direction | Port | Notes |
| --- | --- | --- |
| `spi_demux` → `sensor_processor` | `127.0.0.1:6000` | SPI stream ID 0 |
| `spi_demux` → `udp_mjpeg` | `127.0.0.1:6000 + stream_id` | SPI stream ID ≥ 1 |
| `sensor_processor` → UI | `sensor_output_address:7000 + sensor_id` | base port from config, e.g. `IFS_ALTIMETER` (0x07) → 7007 |
| `udp_mjpeg` → UI | `mjpeg_output_address:mjpeg_output_port` | per camera stream |

Because the sensor output port is `base + sensor_id`, the base port itself and
`base + 1`/`base + 2` stay free for the camera streams with the default config.

## SPI packet framing

Each packet inside an SPI chunk is laid out as:

```
┌────────┬───────────┬──────────────┬──────────────┬──────────────┐
│ length │ stream_id │   payload    │  CRC32 (BE)  │ CC1200 info  │
│ 1 byte │  1 byte   │   n bytes    │   4 bytes    │   2 bytes    │
└────────┴───────────┴──────────────┴──────────────┴──────────────┘
 └──────────── covered by CRC ──────────────┘
 └──────────────── length ─────────────────┘
 └────────────────── length + 2 ──────────────────────────────────┘
```

`length` is the byte count from the length field through the CRC; the two
CC1200 status bytes follow it. A `0x00` length byte marks the end of valid data
in the chunk. Packets that fail CRC validation are logged and skipped.

## Sensor telemetry

Sensor IDs, payload lengths and display names come from `sensor_config` in
[config.json](config.json); the conversion formulas live in `convert_sensor_data()`
in [sensor_processor.py:107](sensor_processor.py#L107).

| ID | Name | Bytes | Values |
| --- | --- | --- | --- |
| 0x03 | `GNSS_POSITION` | 8 | latitude °, longitude ° (uint24 mapped to ±90 / ±180), altitude m (1.5 m/LSB) |
| 0x04 | `EPS_BATTERY` | 4 | current, voltage (raw uint16) |
| 0x05 | `CS_STATUS` | 8 | CPU %, CPU °C, RAM %, eMMC %, SD %, Cam1 RTP, Cam2 RTP, status flags |
| 0x07 | `IFS_ALTIMETER` | 8 | temperature °C, pressure mbar (int32 / 100) |
| 0x09 | `IFS_TCOUPLE` | 8 | 4 × temperature °C (int16, 0.25 °C/LSB) |
| 0x11 | `IFS_TCOUPLE_INTERN` | 8 | 4 × temperature °C (int16, 0.0625 °C/LSB) |
| 0x13 | `IFS_TCOUPLE_ERROR` | 4 | 4 × error flag byte |
| 0x14 | `IFS_STAGNATION` | 4 | temperature °C, pressure kPa (with calibration offset) |
| 0x15 | `IFS_BW_CURRENTS` | 4 | 2 × current A (12-bit ADC, 3.3 V ref, gain 1.9608) |
| 0x16 | `IFS_CGG_CURRENTS` | 4 | 2 × current A (same scaling) |
| 0x17 | `IFS_MANIFOLD` | 2 | manifold pressure kPa |
| 0x18 | `IFS_ACCELERATION` | 8 | accel Z, Y, X, temperature (raw int16) |
| 0x20 | `IFS_ROTATION` | 6 | yaw, roll, pitch rate (raw int16) |

Each sensor is sent to the UI as a newline-terminated ASCII line of
comma-separated values, e.g. `21.5,1013.25\n`.

## MJPEG output format

`udp_mjpeg.py` sends two kinds of datagrams to the same output port:

- **Frame data** — the raw JPEG, split into chunks of at most 32 768 bytes.
- **Metadata** — `AB CD EF <fps>,<frame_count> FE ED`, sent once after every frame.

## Configuration

All settings live in [config.json](config.json), which is read from the script
directory (so the working directory does not matter). The supervisor re-reads it
every second; the child processes read it once at startup, so changing their
settings requires restarting the service.

```json
{
    "features": {
        "logging_level": "INFO"
    },
    "spi_demux": {
        "bus": 0,
        "device": 0,
        "speed": 3000000,
        "drdy_pin": 25
    },
    "sensor_processor": {
        "sensor_output_address": "192.168.0.100",
        "sensor_output_base_port": 7000
    },
    "camera_streams": [
        {
            "name": "camera1",
            "stream_id": 1,
            "mjpeg_output_address": "192.168.0.100",
            "mjpeg_output_port": 7000,
            "quality": 50
        }
    ]
}
```

| Key | Meaning |
| --- | --- |
| `features.logging_level` | `DEBUG`, `INFO`, `WARNING`, … Propagated to all children via the `LOGGING_LEVEL` environment variable. |
| `spi_demux.bus` / `.device` | SPI bus and chip select (`/dev/spidev<bus>.<device>`). |
| `spi_demux.speed` | SPI clock in Hz. |
| `spi_demux.drdy_pin` | Data-ready GPIO in BCM numbering. |
| `sensor_processor.sensor_output_address` | Destination host for sensor telemetry (the UI machine). |
| `sensor_processor.sensor_output_base_port` | Sensor UDP port base; actual port is `base + sensor_id`. |
| `camera_streams[].stream_id` | SPI stream ID; determines the local input port `6000 + stream_id`. |
| `camera_streams[].mjpeg_output_address` / `_port` | Where the MJPEG frames are sent. |
| `camera_streams[].quality` | JPEG quality 1–100 (clamped), default 50. |
| `sensor_config.sensor_data_lengths` | Payload byte count per sensor ID (hex string keys). |
| `sensor_config.sensor_names` | Display name per sensor ID. |

Adding a camera means adding an entry to `camera_streams` — the supervisor picks
it up and spawns another transcoder automatically.

