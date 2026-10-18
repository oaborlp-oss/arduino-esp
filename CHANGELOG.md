# Changelog

Todos los cambios importantes de este proyecto se registran aquí.

El formato sigue [Keep a Changelog](https://keepachangelog.com/es-ES/1.1.0/)
y el proyecto usa [Versionado Semántico](https://semver.org/lang/es/):

- **MAYOR** (1.0.0 → 2.0.0): cambios que obligan a recablear o que rompen ajustes guardados.
- **MENOR** (0.1.0 → 0.2.0): funciones nuevas, equipos nuevos o placas nuevas.
- **PARCHE** (0.1.0 → 0.1.1): correcciones de errores sin funciones nuevas.

Cada entrada indica el equipo afectado (**[Monitor]**, **[CAM-OLED]**, **[Pruebas]**, **[Docs]**)
y el autor (`@usuario-de-github`). Cómo agregar entradas: ver [CONTRIBUTING.md](CONTRIBUTING.md).

## [Sin publicar]

### Agregado
- **[Docs]** CHANGELOG.md, CONTRIBUTING.md, plantilla de Pull Request y verificación
  automática de que cada Pull Request actualice el changelog. — @oaborlp-oss

## [0.2.0] - 2026-10-18

### Agregado
- **[CAM-OLED]** Segundo equipo simple: ESP32-CAM (AI Thinker) + OLED 0.96" 128x64 I2C +
  SHT30. Muestra temperatura, humedad y VPD con etiquetas OK / ALTO / BAJO.
  Sin librerías nuevas. — @oaborlp-oss
- **[Docs]** README: conexiones de la ESP32-CAM, alimentación y cómo subir el código. — @oaborlp-oss

## [0.1.0] - 2026-10-18

Primera versión en el repositorio.

### Agregado
- **[Monitor]** Firmware del monitor de clima con pantalla táctil TFT 2.8" (ILI9341 + XPT2046)
  y sensor SHT30: vista en vivo, gráfico, reportes de picos fuera de rango, etapas de
  cultivo (fotoperiódica / automática), ciclos de luz, rangos día/noche, contador de días,
  secado/curado, calibración del sensor, apagado automático de pantalla y juego COGOLLO. — @oaborlp-oss
- **[Monitor]** Soporte para ESP32 clásica (probada) y ESP32-S3 N16R8 (por probar):
  los pines se eligen solos según la placa seleccionada. — @oaborlp-oss
- **[Pruebas]** Sketches `test_pantalla_directo` y `test_diagnostico`. — @oaborlp-oss
- **[Docs]** README con conexiones y configuración del Arduino IDE. — @oaborlp-oss

### Seguridad
- **[Monitor]** Credenciales WiFi movidas a `secrets.h`, excluido de Git. — @oaborlp-oss

[Sin publicar]: https://github.com/oaborlp-oss/arduino-esp/compare/v0.2.0...HEAD
[0.2.0]: https://github.com/oaborlp-oss/arduino-esp/compare/v0.1.0...v0.2.0
[0.1.0]: https://github.com/oaborlp-oss/arduino-esp/releases/tag/v0.1.0
