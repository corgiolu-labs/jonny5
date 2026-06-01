# Panoramica di sistema — JONNY5 VR

## Descrizione

JONNY5 VR è una piattaforma di teleoperazione in bassa latenza: un operatore usa un visore VR e una dashboard web per comandare un braccio robotico reale. Il flusso unisce controllo immersivo, cinematica, streaming video WebRTC e un dataplane SPI verso il microcontrollore che comanda i servo.

## Architettura (sommario)

A grandi linee:

1. **Frontend** (`web/`): dashboard operativa, viewer VR stereoscopico, asset condivisi (JS/CSS).
2. **Backend Python** (`raspberry/controller/`): server WebSocket per teleop e diagnostica, server HTTPS per statici e API, bridge SPI verso il firmware, tool di analisi e test.
3. **Configurazione runtime** (`raspberry/config_runtime/`): JSON/YAML per routing, cinematica, video, TLS (solo sul dispositivo), calibrazioni VR.
4. **Firmware** (`firmware/stm32/`): applicazione Zephyr su STM32 (PlatformIO), loop di controllo, SPI slave, UART, IMU e attuatori.

## Ruolo dell’STM32

- Esegue il **controllo real-time** (lettura IMU, stato macchina, comandi verso i servo).
- Espone uno **SPI slave** per scambiare frame binari ad alta frequenza con il Raspberry Pi (comandi/telemetria coerenti con il protocollo J5VR).
- Gestisce **UART** e altri canali per diagnostica o profili di test ove previsti nel firmware.

## Ruolo del Raspberry Pi

- **WebSocket** (es. porta 8557): intent di teleoperazione, IMU remota, kinematics, impostazioni; hub tra browser/VR e il resto del sistema.
- **HTTPS** (es. 8443/443): servizio degli statici in `web/` e endpoint API; supporto a WebRTC/WHEP tramite stack video (MediaMTX) configurato sotto `raspberry/config_runtime/video/`.
- **SPI**: processo dedicato che invia/riceve frame verso l’STM32 in sincronia con lo stato condiviso del teleop.
- **Systemd**: unità in `raspberry/systemd/` per avvio automatico (path di deploy di esempio: `/home/jonny5/raspberry5`).

## Comunicazioni

| Mezzo | Uso principale |
|--------|----------------|
| **SPI** | Trasporto deterministico Raspberry ↔ STM32 (controllo e feedback ciclici). |
| **UART** | Canale seriale firmware (monitor, profili di comando/diagnostica secondo build). |
| **WebSocket** | Teleop e telemetria tra client web/VR e backend Python. |
| **HTTPS** | File statici, API di configurazione, TLS per viewer e captive portal ove abilitato. |

## Flusso di controllo VR

1. Il client VR o la dashboard apre una **connessione WebSocket** al backend.
2. Pose, pulsanti e modalità vengono tradotte in **intent** e passate al loop di teleop.
3. Il backend aggiorna lo **stato condiviso** e il processo **SPI** impacchetta i comandi nel formato atteso dal firmware.
4. L’STM32 applica i comandi ai **servo** e restituisce telemetria (stato, IMU, ecc.).
5. Il video a bassa latenza è fornito dal pipeline **WebRTC** (MediaMTX + configurazione in `raspberry/config_runtime/video/`).

Per uno schema visivo sintetico vedi la figura dell’architettura in `../media/images/vr_teleoperation_system_architecture.png` e la sezione architettura nel `README.md` alla radice del repository.
