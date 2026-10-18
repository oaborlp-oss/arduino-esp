# Monitor de clima para cultivo indoor

Monitor de **temperatura, humedad y VPD** con ESP32, pantalla táctil TFT 2.8" y sensor SHT30.
Muestra valores en vivo, gráfico, reportes de picos fuera de rango, etapas de cultivo
(fotoperiódica / automática, ciclos de luz, rangos día/noche, contador de días) e incluye
el juego de plataformas **COGOLLO**.

## Estructura

| Archivo | Para qué sirve |
|---|---|
| `monitor_clima_cultivo.ino` | Firmware principal |
| `secrets.example.h` | Plantilla de WiFi. Cópiala como `secrets.h` |
| `secrets.h` | Tu red y clave reales. **No se sube a GitHub** |
| `equipo_cam_oled/` | Segundo equipo simple: ESP32-CAM + OLED 0.96" + SHT30 |
| `pruebas/test_pantalla_directo` | Prueba de la pantalla sin librerías |
| `CHANGELOG.md` | Historial de cambios por versión y autor |
| `CONTRIBUTING.md` | Reglas para programadores: ramas, commits y changelog |
| `pruebas/test_diagnostico` | Prueba de placa, pantalla y sensor I2C |

## Librerías (Arduino IDE → Herramientas → Gestionar librerías)

- **LovyanGFX** (lovyan03)
- **Adafruit SHT31 Library** (Adafruit) → aceptar "Instalar todo" (Adafruit BusIO)

Soporte de placas: en Preferencias agrega
`https://raw.githubusercontent.com/espressif/arduino-esp32/gh-pages/package_esp32_index.json`
y en el Gestor de placas instala **esp32 de Espressif Systems**.

## Placa A: ESP32 clásica DevKit V1 (probada ✅)

**Herramientas:** Placa `ESP32 Dev Module` · Partition Scheme `Huge APP (3MB No OTA/1MB SPIFFS)`

| Pantalla | ESP32 |
|---|---|
| VCC | VIN (5 V) |
| GND | GND |
| CS | D5 |
| RESET | D22 |
| DC | D4 |
| SDI (MOSI) | D23 |
| SCK | D18 |
| LED | D25 |
| SDO (MISO) | sin conectar |
| T_CLK | D18 (compartido con SCK) |
| T_DIN | D23 (compartido con SDI) |
| T_DO | D19 |
| T_CS | D21 |
| T_IRQ | sin conectar |

| SHT30 | ESP32 |
|---|---|
| Rojo (VDD) | 3V3 |
| Negro (GND) | GND |
| Verde (SDA) | D32 |
| Amarillo (SCL) | D33 |

## Placa B: ESP32-S3 DevKitC-1 N16R8 (por probar ⚠️)

> No usar el adaptador negro "ESP32-S3 扩展板": su orden de pines no coincide con la placa UICPAL
> y provocó cortes de alimentación. Montar en protoboard o con cables hembra-hembra directos.

**Herramientas:** Placa `ESP32S3 Dev Module` · Flash Size `16MB (128Mb)` · PSRAM `OPI PSRAM` ·
Partition Scheme `16M Flash (3MB APP/9.9MB FATFS)` · USB CDC On Boot:

- `Disabled` si conectas por el puerto **USB-TTL** (derecha, recomendado)
- `Enabled` si conectas por el puerto **USB-OTG** (izquierda)

| Pantalla | ESP32-S3 |
|---|---|
| VCC | 5V |
| GND | GND |
| CS | G10 |
| RESET | G8 |
| DC | G9 |
| SDI (MOSI) | G11 |
| SCK | G12 |
| LED | G7 |
| SDO (MISO) | sin conectar |
| T_CLK | G12 (compartido con SCK) |
| T_DIN | G11 (compartido con SDI) |
| T_DO | G13 |
| T_CS | G14 |
| T_IRQ | sin conectar |

| SHT30 | ESP32-S3 |
|---|---|
| Rojo (VDD) | 3V3 |
| Negro (GND) | GND |
| Verde (SDA) | G4 |
| Amarillo (SCL) | G5 |

No usar GPIO 19–20 (USB) ni 26–37 (flash/PSRAM en el N16R8).

## Equipo simple: ESP32-CAM + OLED 0.96" (carpeta `equipo_cam_oled`)

Muestra temperatura, humedad y VPD con etiquetas OK / ALTO / BAJO (rangos generales
18–28 °C, 40–70 %, 0.4–1.6 kPa). Sin WiFi ni táctil. La cámara no se usa.

**Herramientas:** Placa `AI Thinker ESP32-CAM`. Abre `equipo_cam_oled/equipo_cam_oled.ino`.

OLED y SHT30 comparten el bus I2C (dos cables a cada pin):

| Señal | ESP32-CAM | OLED | SHT30 |
|---|---|---|---|
| 3.3 V | 3V3 | VCC | Rojo |
| GND | GND | GND | Negro |
| SDA | GPIO 14 | SDA | Verde |
| SCL | GPIO 15 | SCL | Amarillo |

**Subir el código:** la ESP32-CAM no trae USB.

- Con base **ESP32-CAM-MB**: conecta el USB de la base y sube normal. Si queda en
  `Connecting...`, mantén IO0, pulsa RST y suelta IO0.
- Con adaptador **USB-TTL** (5 V): TX→U0R, RX→U0T, 5V→5V, GND→GND y un puente
  **GPIO0→GND** para subir. Después quita el puente y pulsa RST.

Alimentación: 5 V con buena corriente (cargador USB 5 V 1 A). Con poca corriente la
ESP32-CAM se reinicia ("Brownout detector").

## WiFi

1. Copia `secrets.example.h` y renómbralo `secrets.h`.
2. Escribe tu red (solo 2.4 GHz) y tu clave.
3. Sin `secrets.h` el equipo funciona igual, sin hora real.

## Subir el código

1. Abre `monitor_clima_cultivo.ino` en el Arduino IDE.
2. Elige la placa y el puerto (Herramientas → Puerto).
3. Presiona Subir (→). Si queda en `Connecting...`: mantén **BOOT**, pulsa y suelta **RST**, suelta **BOOT**.
4. Monitor Serie a **115200**: la primera línea dice qué placa detectó.

## Próximas etapas

Watchdog · historial guardado en flash · OTA · MQTT para la app · PCB y caja IP.
