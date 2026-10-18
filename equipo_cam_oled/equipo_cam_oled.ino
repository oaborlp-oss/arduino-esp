/*
  ============================================================================
   EQUIPO SIMPLE — Temperatura / Humedad / VPD
   ESP32-CAM (AI Thinker) + OLED 0.96" 128x64 I2C (SSD1306) + SHT30
  ============================================================================
   Conexiones (OLED y SHT30 comparten el mismo bus I2C):
     OLED VCC  -> 3V3        SHT30 rojo     -> 3V3
     OLED GND  -> GND        SHT30 negro    -> GND
     OLED SDA  -> GPIO 14    SHT30 verde    -> GPIO 14
     OLED SCL  -> GPIO 15    SHT30 amarillo -> GPIO 15

   Librerías (ya usadas por el equipo principal):
     Adafruit SHT31, Adafruit GFX, Adafruit BusIO

   Placa en Arduino IDE: "AI Thinker ESP32-CAM"
   Para subir: base ESP32-CAM-MB por USB, o adaptador USB-TTL con GPIO0 a GND.
  ============================================================================
*/

#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SHT31.h>

// ---------------------------------------------------------------------------
// Pines y ajustes
// ---------------------------------------------------------------------------
#define PIN_SDA        14
#define PIN_SCL        15
#define PIN_FLASH_LED   4    // LED blanco de la ESP32-CAM: se mantiene apagado

const float LEAF_OFFSET = -1.5f;   // temp. hoja - temp. aire (LED: -1 a -2 °C)

// Rangos generales (mismos del modo "Monitor" del equipo principal)
struct Range { float lo, hi; };
const Range R_T   = { 18.0f, 28.0f };
const Range R_RH  = { 40.0f, 70.0f };
const Range R_VPD = {  0.4f,  1.6f };

const uint32_t READ_MS = 2000;

// ---------------------------------------------------------------------------
// OLED SSD1306 mínimo: se dibuja en un lienzo de Adafruit GFX y se envía por I2C
// ---------------------------------------------------------------------------
GFXcanvas1 canvas(128, 64);
uint8_t oledAddr = 0;

void oledCmd(uint8_t c) {
  Wire.beginTransmission(oledAddr);
  Wire.write(0x00);
  Wire.write(c);
  Wire.endTransmission();
}

bool oledBegin() {
  for (uint8_t a : { 0x3C, 0x3D }) {
    Wire.beginTransmission(a);
    if (Wire.endTransmission() == 0) { oledAddr = a; break; }
  }
  if (!oledAddr) return false;
  const uint8_t init[] = {
    0xAE, 0xD5, 0x80, 0xA8, 0x3F, 0xD3, 0x00, 0x40, 0x8D, 0x14, 0x20, 0x00,
    0xA1, 0xC8, 0xDA, 0x12, 0x81, 0xCF, 0xD9, 0xF1, 0xDB, 0x40, 0xA4, 0xA6, 0xAF
  };
  for (uint8_t c : init) oledCmd(c);
  return true;
}

// El lienzo guarda filas de bits; el SSD1306 espera "páginas" de 8 filas verticales.
void oledShow() {
  if (!oledAddr) return;
  oledCmd(0x21); oledCmd(0); oledCmd(127);
  oledCmd(0x22); oledCmd(0); oledCmd(7);
  const uint8_t* buf = canvas.getBuffer();
  for (int page = 0; page < 8; page++) {
    for (int x0 = 0; x0 < 128; x0 += 16) {
      Wire.beginTransmission(oledAddr);
      Wire.write(0x40);
      for (int x = x0; x < x0 + 16; x++) {
        uint8_t b = 0;
        for (int bit = 0; bit < 8; bit++) {
          int y = page * 8 + bit;
          if (buf[y * 16 + (x >> 3)] & (0x80 >> (x & 7))) b |= (1 << bit);
        }
        Wire.write(b);
      }
      Wire.endTransmission();
    }
  }
}

// ---------------------------------------------------------------------------
// Sensor y VPD
// ---------------------------------------------------------------------------
Adafruit_SHT31 sht;
bool shtOk = false;

float svp(float t) { return 0.61078f * expf(17.27f * t / (t + 237.3f)); }
float calcVpd(float tAir, float rh) { return svp(tAir + LEAF_OFFSET) - svp(tAir) * rh / 100.0f; }

const char* rangeTag(float v, const Range& r) {
  if (v < r.lo) return "BAJO";
  if (v > r.hi) return "ALTO";
  return "OK";
}

// ---------------------------------------------------------------------------
// Pantalla
// ---------------------------------------------------------------------------
void drawRow(int y, const char* label, const char* value, const char* tag) {
  canvas.setTextSize(1);
  canvas.setCursor(0, y + 4);
  canvas.print(label);
  canvas.setTextSize(2);
  canvas.setCursor(28, y);
  canvas.print(value);
  canvas.setTextSize(1);
  int16_t x1, y1; uint16_t w, h;
  canvas.getTextBounds(tag, 0, 0, &x1, &y1, &w, &h);
  if (strcmp(tag, "OK") != 0) {   // fuera de rango: etiqueta en negativo
    canvas.fillRect(127 - w - 3, y + 2, w + 4, 11, 1);
    canvas.setTextColor(0);
  }
  canvas.setCursor(127 - w - 1, y + 4);
  canvas.print(tag);
  canvas.setTextColor(1);
}

void drawScreen(float t, float rh, float vpd) {
  canvas.fillScreen(0);
  canvas.setTextColor(1);
  canvas.setTextSize(1);
  canvas.setCursor(0, 0);
  canvas.print("Clima indoor");
  if (shtOk) {
    char b[12];
    snprintf(b, sizeof b, "%.1fC", t);    drawRow(12, "Temp", b, rangeTag(t, R_T));
    snprintf(b, sizeof b, "%.1f%%", rh);  drawRow(30, "HR",   b, rangeTag(rh, R_RH));
    snprintf(b, sizeof b, "%.2f", vpd);   drawRow(48, "VPD",  b, rangeTag(vpd, R_VPD));
  } else {
    canvas.setTextSize(2);
    canvas.setCursor(0, 22);
    canvas.print("Sin sensor");
    canvas.setTextSize(1);
    canvas.setCursor(0, 46);
    canvas.print("SDA 14  SCL 15");
  }
  oledShow();
}

// ---------------------------------------------------------------------------
void setup() {
  pinMode(PIN_FLASH_LED, OUTPUT);
  digitalWrite(PIN_FLASH_LED, LOW);

  Serial.begin(115200);
  delay(500);
  Serial.println("\nEquipo simple ESP32-CAM: T / HR / VPD");

  Wire.begin(PIN_SDA, PIN_SCL);
  Wire.setClock(400000);

  if (oledBegin()) Serial.printf("OLED en 0x%02X\n", oledAddr);
  else             Serial.println("OLED NO encontrada: revisa SDA 14 / SCL 15");

  shtOk = sht.begin(0x44) || sht.begin(0x45);
  Serial.println(shtOk ? "SHT30 OK" : "SHT30 NO encontrado");
  drawScreen(0, 0, 0);
}

void loop() {
  static uint32_t last = 0;
  if (millis() - last < READ_MS) return;
  last = millis();

  if (!shtOk) shtOk = sht.begin(0x44) || sht.begin(0x45);   // reintenta si se desconectó

  float t = NAN, rh = NAN;
  if (shtOk) {
    t  = sht.readTemperature();
    rh = sht.readHumidity();
    if (isnan(t) || isnan(rh)) shtOk = false;
  }

  float vpd = shtOk ? calcVpd(t, rh) : NAN;
  if (shtOk) Serial.printf("T=%.1f C  HR=%.1f %%  VPD=%.2f kPa\n", t, rh, vpd);
  else       Serial.println("Sin lectura del sensor");
  drawScreen(t, rh, vpd);
}
