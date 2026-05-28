# JONNY5 — VR-Teleoperated 6-DoF Robot Arm

Low-latency, immersive teleoperation platform for a 6-DoF robot arm, built on
**open web standards** (WebXR + WebRTC) and consumer hardware. An operator
wearing a VR headset controls the arm with their own head/hand pose and receives
low-latency stereo video feedback from cameras mounted on the end-effector.

This repository contains everything needed to **build and deploy** the system:
STM32 firmware, Raspberry Pi control-plane, browser dashboard and VR viewer.

## Architecture (3 tiers)

```
                    ┌───────────────────────────────┐
                    │  VR Headset (Quest) / Browser  │
                    │   WebXR pose · WebRTC video     │
                    └──────────────┬────────────────┘
                                   │ WebSocket (wss://…:8557)
                                   │ HTTPS (…:8443)
                    ┌──────────────▼────────────────┐
                    │   Raspberry Pi 5 (middleware)  │
                    │   WS server · IK/FK solver     │
                    │   video encoding · SPI master  │
                    └──────────────┬────────────────┘
                                   │ SPI @ 100 Hz (J5VR frames)
                    ┌──────────────▼────────────────┐
                    │   STM32 Nucleo-F446RE (Zephyr) │
                    │   real-time actuation loop     │
                    │   servo PWM · IMU BNO085        │
                    │   safety gating (deadman/WDT)  │
                    └───────────────────────────────┘
```

## Directory layout

| Path | Contents |
|---|---|
| `firmware/stm32/` | Zephyr firmware for STM32 Nucleo-F446RE: real-time loop, servo PWM, IMU (BNO085), SPI slave, safety gating |
| `raspberry/controller/` | Python control-plane: WebSocket handlers, IK/FK solver (PoE model), head-assist logic, SPI bridge, settings manager |
| `raspberry/config_runtime/` | Runtime configuration (joint offsets/limits, PoE params, IMU calibration, TLS placeholders) |
| `raspberry/systemd/` | systemd service units for the Pi daemons |
| `raspberry/networkmanager/` | dnsmasq captive-portal config (first-time headset setup) |
| `web/dashboard/` | Operator dashboard (FK/IK/IMU compare, joints, settings, live Test page) |
| `web/vr/` | VR viewer (stereo WebRTC pipeline) |
| `scripts/` | Helper scripts (video pipeline, MediaMTX launcher) |
| `deploy.sh` / `deploy.bat` | rsync + ssh deploy to the Pi |
| `setup_pi.sh` | One-time Pi bootstrap |

## Prerequisites

**Raspberry Pi 5** (Raspberry Pi OS, 64-bit):
- `python3` + `python3-venv`, `curl` (MediaMTX is auto-downloaded by `deploy.sh`)
- 2× camera modules (Sony IMX708) on the CSI ports

**STM32 Nucleo-F446RE** firmware build host:
- [PlatformIO](https://platformio.org/) (`pip install platformio`)

**Development host** (for deploy): `ssh` and `rsync`.

## Quick start

### 1. Flash the firmware (STM32)

```bash
cd firmware/stm32
pio run -t upload        # builds the Zephyr image and flashes via ST-Link
```

### 2. Deploy the Pi control-plane

```bash
./deploy.sh jonny5@<pi-host>     # default: jonny5@10.42.0.1
```

`deploy.sh` rsyncs `raspberry/`, `web/` and `scripts/` to the Pi, creates a
Python venv from `raspberry/requirements-controller.txt`, downloads MediaMTX if
missing, generates self-signed TLS certs on first run, and installs + restarts
the systemd services.

> First-time only: run `./setup_pi.sh` on the Pi to bootstrap system packages
> and the NetworkManager hotspot/captive-portal.

### 3. Connect

- Dashboard: `https://<pi-host>/` (self-signed cert — accept the browser warning)
- VR viewer: open the dashboard from the headset browser and start the WebXR session

## Runtime services (Pi)

```
jonny5-ws-teleop       — WebSocket server @ 8557 (dispatcher, IK/FK, head-assist)
jonny5-spi-j5vr        — SPI master (Pi → STM32), 100 Hz J5VR frames
jonny5-https           — static files + dashboard over HTTPS @ 8443
jonny5-https-443-proxy — 443 → 8443 forwarder (headset browser compatibility)
jonny5-captive-portal  — captive portal for first-time headset setup
jonny5-mediamtx        — WebRTC media server for stereo video
```

Live logs:

```bash
ssh jonny5@<pi-host> 'journalctl -fu jonny5-ws-teleop'
```

## Key subsystems

- **Kinematics** — Product of Exponentials (PoE) forward/inverse kinematics;
  `raspberry/controller/.../ik_solver.py`, PoE params in
  `config_runtime/kinematics/`.
- **IMU pipeline** — BNO085 (Rotation Vector @ ~100 Hz) with a frame-alignment
  chain (mount → world bias → home reference).
- **Video** — MediaMTX (WebRTC/WHEP, H.264 hardware encode); low-latency profile
  800×450 @ 120 fps.
- **Safety** — firmware-side deadman + heartbeat watchdog (100 ms) and servo
  limits, independent of the general-purpose Pi node.

## Code entry points

- Control-plane: `raspberry/controller/web_services/ws_server.py`
- Firmware real-time loop: `firmware/stm32/src/core/rt_loop.c`
- Authoritative runtime config: `raspberry/config_runtime/robot/routing_config.json`

---

Developed by **Alessandro Corgiolu** as part of an MSc thesis in Management
Engineering, Università Telematica Internazionale UNINETTUNO.
