/*
  ============================================================================
   MONITOR DE CLIMA PARA CULTIVO INDOOR  —  Temperatura / Humedad / VPD
   Versión 2: tipo de cultivo, ciclo de luz, rangos día/noche,
              contador de días, secado/curado y calibración del sensor.
  ============================================================================
   Hardware:
     - ESP32 clásica DevKit V1 (30 pines)        -> probado y funcionando
     - ESP32-S3 DevKitC-1 N16R8 (en protoboard)  -> pines definidos, por probar
     - TFT 2.8" 240x320 SPI (ILI9341 + táctil XPT2046)
     - Sensor SHT30 I2C en sonda IP68
   Los pines se eligen solos según la placa seleccionada en el Arduino IDE.

   Librerías: LovyanGFX, Adafruit SHT31 (+ Adafruit BusIO)

   Placa en Arduino IDE (ver README.md):
     ESP32 clásica: "ESP32 Dev Module"
       - Partition Scheme: "Huge APP (3MB No OTA/1MB SPIFFS)"
     ESP32-S3 N16R8: "ESP32S3 Dev Module"
       - Flash Size: 16MB, PSRAM: OPI PSRAM
       - Partition Scheme: "16M Flash (3MB APP/9.9MB FATFS)"
       - USB CDC On Boot: Disabled si usas el puerto USB-TTL
     Upload Speed: 115200 si la carga falla

   WiFi: copia secrets.example.h como secrets.h y escribe tu red ahí.
   secrets.h no se sube a GitHub.

   "Día" = luces de cultivo encendidas, según el horario configurado en Ajustes.
   El horario y el contador de días necesitan la hora real (WiFi).
   Sin WiFi el equipo funciona igual, asumiendo siempre "día".
  ============================================================================
*/

#define LGFX_USE_V1
#include <LovyanGFX.hpp>
#include <Wire.h>
#include <Adafruit_SHT31.h>
#include <Preferences.h>
#include <WiFi.h>
#include <time.h>
#include "esp_timer.h"

// Tipos usados como parámetros (deben ir antes de cualquier función)
struct Btn { int x, y, w, h; };
struct Range { float lo, hi; };
// Juego "Brote"
struct Body   { float x, y, vx, vy; int16_t w, h; bool ground; };
struct GEnemy { Body b; float baseX, baseY; int16_t t; int8_t dir; uint8_t type, state, timer, hp; bool active, awake; };
struct GItem  { Body b; int8_t dir; uint8_t state, timer, kind; bool active; };   // kind 0 hoja, 1 sol
struct GShot  { float x, y, vx, vy; uint8_t life, kind; bool active; };
struct GPopup { float x, y, vx, vy; int16_t val; uint8_t kind, timer; bool active; };

// ---------------------------------------------------------------------------
// WiFi (para hora real: horario de luces, contador de días y reportes)
// ---------------------------------------------------------------------------
// La red y la clave van en secrets.h (no se sube a GitHub).
// Si no existe, el equipo funciona sin WiFi.
#if __has_include("secrets.h")
  #include "secrets.h"
#else
  const char* WIFI_SSID = "";
  const char* WIFI_PASS = "";
#endif
const char* TZ_CHILE  = "<-04>4<-03>,M9.1.6/24,M4.1.6/24";   // Chile continental

// ---------------------------------------------------------------------------
// PINES — se eligen según la placa seleccionada en el Arduino IDE
// ---------------------------------------------------------------------------
#if CONFIG_IDF_TARGET_ESP32S3
  // ESP32-S3 DevKitC-1 N16R8 (protoboard). No usar GPIO 19-20 (USB) ni 26-37 (flash/PSRAM).
  #define BOARD_NAME    "ESP32-S3"
  #define LCD_SPI_HOST  SPI2_HOST
  #define PIN_SCK       12   // TFT SCK  + T_CLK
  #define PIN_MOSI      11   // TFT SDI  + T_DIN
  #define PIN_MISO      13   // solo T_DO (SDO de la TFT SIN conectar)
  #define PIN_TFT_CS    10
  #define PIN_TFT_DC     9
  #define PIN_TFT_RST    8
  #define PIN_TFT_BL     7   // LED de la pantalla (permite apagar la pantalla)
  #define PIN_TOUCH_CS  14
  #define PIN_SDA        4   // SHT30 verde
  #define PIN_SCL        5   // SHT30 amarillo
#elif CONFIG_IDF_TARGET_ESP32
  // ESP32 clásica DevKit V1 (30 pines) — configuración probada
  #define BOARD_NAME    "ESP32"
  #define LCD_SPI_HOST  SPI3_HOST
  #define PIN_SCK       18   // TFT SCK  + T_CLK
  #define PIN_MOSI      23   // TFT SDI  + T_DIN
  #define PIN_MISO      19   // solo T_DO (SDO de la TFT SIN conectar)
  #define PIN_TFT_CS     5
  #define PIN_TFT_DC     4
  #define PIN_TFT_RST   22
  #define PIN_TFT_BL    25   // LED de la pantalla -> D25 (permite apagar la pantalla)
  #define PIN_TOUCH_CS  21
  #define PIN_SDA       32   // SHT30 verde
  #define PIN_SCL       33   // SHT30 amarillo
#else
  #error "Placa no soportada: elige 'ESP32 Dev Module' o 'ESP32S3 Dev Module'"
#endif

// ---------------------------------------------------------------------------
// Pantalla ILI9341 + táctil XPT2046
// ---------------------------------------------------------------------------
class LGFX : public lgfx::LGFX_Device {
  lgfx::Panel_ILI9341 _panel;
  lgfx::Bus_SPI       _bus;
  lgfx::Light_PWM     _light;
  lgfx::Touch_XPT2046 _touch;
public:
  LGFX() {
    { auto cfg = _bus.config();
      cfg.spi_host = LCD_SPI_HOST; cfg.spi_mode = 0;
      cfg.freq_write = 40000000; cfg.freq_read = 16000000;
      cfg.spi_3wire = false; cfg.use_lock = true;
      cfg.dma_channel = SPI_DMA_CH_AUTO;
      cfg.pin_sclk = PIN_SCK; cfg.pin_mosi = PIN_MOSI; cfg.pin_miso = PIN_MISO; cfg.pin_dc = PIN_TFT_DC;
      _bus.config(cfg); _panel.setBus(&_bus); }
    { auto cfg = _panel.config();
      cfg.pin_cs = PIN_TFT_CS; cfg.pin_rst = PIN_TFT_RST; cfg.pin_busy = -1;
      cfg.panel_width = 240; cfg.panel_height = 320;
      cfg.offset_x = 0; cfg.offset_y = 0; cfg.offset_rotation = 0;
      cfg.readable = false; cfg.invert = false; cfg.rgb_order = false;
      cfg.dlen_16bit = false; cfg.bus_shared = true;
      _panel.config(cfg); }
    if (PIN_TFT_BL >= 0) {
      auto cfg = _light.config();
      cfg.pin_bl = PIN_TFT_BL; cfg.invert = false; cfg.freq = 44100; cfg.pwm_channel = 7;
      _light.config(cfg); _panel.setLight(&_light); }
    { auto cfg = _touch.config();
      cfg.x_min = 300; cfg.x_max = 3900; cfg.y_min = 200; cfg.y_max = 3700;
      cfg.pin_int = -1; cfg.bus_shared = true; cfg.offset_rotation = 0;
      cfg.spi_host = LCD_SPI_HOST; cfg.freq = 1000000;
      cfg.pin_sclk = PIN_SCK; cfg.pin_mosi = PIN_MOSI; cfg.pin_miso = PIN_MISO; cfg.pin_cs = PIN_TOUCH_CS;
      _touch.config(cfg); _panel.setTouch(&_touch); }
    setPanel(&_panel);
  }
};

LGFX lcd;
Adafruit_SHT31 sht;
Preferences prefs;

// ---------------------------------------------------------------------------
// Colores (RGB565)
// ---------------------------------------------------------------------------
constexpr uint16_t rgb(uint8_t r, uint8_t g, uint8_t b) {
  return (uint16_t)(((r & 0xF8) << 8) | ((g & 0xFC) << 3) | (b >> 3));
}
const uint16_t C_BG    = rgb(10, 14, 24);
const uint16_t C_PANEL = rgb(21, 29, 44);
const uint16_t C_GRID  = rgb(48, 60, 82);
const uint16_t C_TXT   = rgb(230, 236, 245);
const uint16_t C_DIM   = rgb(138, 151, 173);
const uint16_t C_OK    = rgb(61, 220, 151);
const uint16_t C_HIGH  = rgb(255, 107, 90);
const uint16_t C_LOW   = rgb(90, 176, 255);
const uint16_t C_WARN  = rgb(240, 180, 60);
const uint16_t C_BAND  = rgb(26, 60, 48);
const uint16_t C_SUN   = rgb(250, 200, 60);
const uint16_t C_MOON  = rgb(170, 180, 230);

// ---------------------------------------------------------------------------
// Etapas y rangos (día = luces encendidas, noche = luces apagadas)
// Ajusta a tu genética y sala.
// ---------------------------------------------------------------------------
enum { V_T = 0, V_RH = 1, V_VPD = 2 };
enum { ST_ESQ = 0, ST_PLANT = 1, ST_VEGE = 2, ST_FLORA1 = 3, ST_FLORA2 = 4, ST_SECADO = 5, N_STAGES = 6, ST_NONE = 6 };
struct Stage { const char* name; const char* shortName; Range day[3]; Range night[3]; };

const Stage STAGES[N_STAGES + 1] = {
  //  nombre               corto       DIA: T        HR         VPD             NOCHE: T      HR         VPD
  { "Esquejes",         "Esqueje", { {22, 26}, {70, 85}, {0.4f, 0.8f} }, { {21, 25}, {70, 85}, {0.4f, 0.8f} } },
  { "Plantulas",        "Plantula",{ {22, 26}, {60, 70}, {0.6f, 1.0f} }, { {20, 24}, {60, 70}, {0.5f, 0.9f} } },
  { "Vegetativo",       "Vege",    { {22, 28}, {55, 70}, {0.8f, 1.2f} }, { {18, 24}, {55, 70}, {0.6f, 1.1f} } },
  { "Flora temprana",   "Flora 1", { {20, 26}, {45, 55}, {1.0f, 1.5f} }, { {18, 23}, {45, 55}, {0.8f, 1.3f} } },
  { "Flora tardia",     "Flora 2", { {18, 24}, {35, 45}, {1.2f, 1.6f} }, { {16, 21}, {35, 45}, {1.0f, 1.5f} } },
  { "Secado/Curado",    "Secado",  { {18, 21}, {55, 62}, {0.75f,1.1f} }, { {18, 21}, {55, 62}, {0.75f,1.1f} } },
  // Sin cultivo activo: monitor general con rangos amplios
  { "Sin cultivo",      "Monitor", { {18, 28}, {40, 70}, {0.4f, 1.6f} }, { {18, 28}, {40, 70}, {0.4f, 1.6f} } },
};

// Ciclos de luz (horas encendidas)
struct Cycle { const char* name; uint8_t on; };
const Cycle CYCLES[4] = { { "18/6", 18 }, { "20/4", 20 }, { "24/0", 24 }, { "12/12", 12 } };

const char*   VAR_SHORT[3] = { "Temp", "HR", "VPD" };
const char*   VAR_UNIT[3]  = { "C", "%", "kPa" };
const uint8_t VAR_DEC[3]   = { 1, 1, 2 };
const float   HYST[3]      = { 0.3f, 1.5f, 0.03f };
#define FLORA_AUTO_DAYS 28   // Flora 1 -> Flora 2 automático a los 28 días de flora

// ---------------------------------------------------------------------------
// Configuración (guardada en memoria)
// ---------------------------------------------------------------------------
uint8_t  cropType   = 0;      // 0 = fotoperiódica, 1 = automática
uint8_t  cycleIdx   = 0;      // índice en CYCLES
uint16_t lightsOnMin = 8 * 60; // hora de encendido de luces (minutos desde 00:00)
uint8_t  stage      = ST_VEGE;
uint32_t stageStart = 0;      // epoch inicio de etapa (0 = desconocido)
uint32_t floraStart = 0;      // epoch inicio de floración
float    leafOff    = -1.5f;  // temp. hoja - temp. aire (solo con luces encendidas)
float    tOff       = 0.0f;   // calibración temperatura
float    rhOff      = 0.0f;   // calibración humedad
uint8_t  scrIdx     = 1;      // apagado de pantalla (índice en SCR_SEC)
uint32_t cropStart  = 0;      // inicio del cultivo: 0 = sin cultivo, 1 = iniciado sin hora, otro = epoch
#define CROP_PENDING 1

// Borrador de "Nuevo cultivo" (pantalla de configuración)
uint8_t  nType = 0, nCycle = 0, nStage = ST_ESQ;
uint16_t nLon = 8 * 60;
int      nDays = 0;           // días respecto a hoy (- pasado, + futuro)
bool     confirmEnd = false;  // confirmación para terminar cultivo

// Apagado automático de pantalla
const uint32_t SCR_SEC[5] = { 30, 60, 300, 900, 0 };       // 0 = nunca
const char*    SCR_LBL[5] = { "30 seg", "1 min", "5 min", "15 min", "Nunca" };
#define BRIGHTNESS 200
bool     screenOn = true;
uint32_t lastActivity = 0;   // millis del último toque

// ---------------------------------------------------------------------------
// Estado
// ---------------------------------------------------------------------------
float cur[3] = { NAN, NAN, NAN };
float rawT = NAN, rawH = NAN;
bool  sensorOk = false;
uint8_t sensorFails = 0;
bool  dayNow = true;

#define HIST_LEN 1440
#define HIST_SEC 60
float    hist[3][HIST_LEN];
uint32_t histTime[HIST_LEN];
bool     histDay[HIST_LEN];
int histHead = 0, histCount = 0;
float acc[3] = { 0, 0, 0 };
int   accN = 0;
uint32_t lastHistPush = 0;

#define EV_MAX     40
#define EV_MIN_SEC 60
struct Event { uint8_t var; bool high; uint32_t start; uint32_t dur; float peak; };
Event events[EV_MAX];
int evHead = 0, evCount = 0;
struct Track { bool active; bool high; uint32_t start; float peak; } track[3];

bool timeSynced = false;

// UI
enum { S_LIVE = 0, S_GRAPH = 1, S_REPORT = 2, S_SET = 3 };
uint8_t screen   = S_LIVE;
uint8_t setPage  = 0;      // 0 = Cultivo, 1 = Sensor
uint8_t graphVar = V_VPD;
uint8_t graphWin = 2;
const uint32_t WIN_SEC[3] = { 3600, 21600, 86400 };
const char*    WIN_LBL[3] = { "1 h", "6 h", "24 h" };

const int W = 320, H = 240, TOP_H = 22, TAB_Y = 204, TAB_H = 36;
const int CONTENT_Y = TOP_H, CONTENT_H = TAB_Y - TOP_H;

bool inBtn(const Btn& b, int x, int y) { return x >= b.x && x < b.x + b.w && y >= b.y && y < b.y + b.h; }

// ---------------------------------------------------------------------------
// Utilidades
// ---------------------------------------------------------------------------
uint32_t uptimeSec() { return (uint32_t)(esp_timer_get_time() / 1000000ULL); }
uint32_t nowEpoch()  { return timeSynced ? (uint32_t)time(nullptr) : 0; }

float svp(float t) { return 0.61078f * expf(17.27f * t / (t + 237.3f)); }
float calcVpd(float tAir, float rh, float off) {
  float v = svp(tAir + off) - svp(tAir) * rh / 100.0f;
  return v < 0 ? 0 : v;
}

bool isFlora(uint8_t s) { return s == ST_FLORA1 || s == ST_FLORA2; }

// Estado del cultivo
bool cropActive()     { return cropStart == CROP_PENDING || (cropStart > CROP_PENDING && (!timeSynced || nowEpoch() >= cropStart)); }
bool cropProgrammed() { return cropStart > CROP_PENDING && timeSynced && nowEpoch() < cropStart; }
uint8_t curStage()    { return cropActive() ? stage : ST_NONE; }

// ¿Luces encendidas ahora? (según horario)
bool computeDay() {
  if (!cropActive()) return true;                  // sin cultivo: sin horario de luces
  if (stage == ST_SECADO) return false;            // secado: sala a oscuras
  if (CYCLES[cycleIdx].on >= 24) return true;      // 24/0
  if (!timeSynced) return true;                    // sin hora: asumimos día
  time_t e = time(nullptr); struct tm t; localtime_r(&e, &t);
  int mins = t.tm_hour * 60 + t.tm_min;
  int diff = (mins - (int)lightsOnMin + 1440) % 1440;
  return diff < CYCLES[cycleIdx].on * 60;
}

const Range& rangeOf(uint8_t v, bool day) {
  const Stage& s = STAGES[curStage()];
  return day ? s.day[v] : s.night[v];
}

int statusAt(uint8_t v, float x, bool day) {
  if (isnan(x)) return 0;
  const Range& r = rangeOf(v, day);
  if (x > r.hi) return 1;
  if (x < r.lo) return -1;
  return 0;
}
int statusOf(uint8_t v, float x) { return statusAt(v, x, dayNow); }
uint16_t statusColor(int s) { return s > 0 ? C_HIGH : (s < 0 ? C_LOW : C_OK); }

void fmtDur(uint32_t s, char* out, size_t n) {
  if (s < 60)        snprintf(out, n, "%lus", (unsigned long)s);
  else if (s < 3600) snprintf(out, n, "%lum", (unsigned long)(s / 60));
  else               snprintf(out, n, "%luh%02lum", (unsigned long)(s / 3600), (unsigned long)((s % 3600) / 60));
}

void fmtAt(uint32_t up, char* out, size_t n) {
  uint32_t now = uptimeSec();
  uint32_t ago = now > up ? now - up : 0;
  if (timeSynced) {
    time_t e = time(nullptr) - (time_t)ago;
    struct tm tmv; localtime_r(&e, &tmv);
    strftime(out, n, "%H:%M", &tmv);
  } else {
    char d[12]; fmtDur(ago, d, sizeof d);
    snprintf(out, n, "-%s", d);
  }
}

int daysSince(uint32_t start) {
  if (!timeSynced || start <= CROP_PENDING) return -1;
  uint32_t n = nowEpoch();
  return n > start ? (int)((n - start) / 86400) + 1 : 1;
}

// Medianoche local de hoy + desplazamiento en días
uint32_t todayPlusDays(int d) {
  time_t e = time(nullptr); struct tm t; localtime_r(&e, &t);
  t.tm_hour = 0; t.tm_min = 0; t.tm_sec = 0;
  t.tm_mday += d;
  t.tm_isdst = -1;
  return (uint32_t)mktime(&t);
}

void fmtDate(uint32_t epoch, char* out, size_t n) {
  time_t e = epoch; struct tm t; localtime_r(&e, &t);
  strftime(out, n, "%d/%m/%Y", &t);
}

void saveConfig() {
  prefs.putUChar("type", cropType);
  prefs.putUChar("cycle", cycleIdx);
  prefs.putUShort("lon", lightsOnMin);
  prefs.putUChar("stage", stage);
  prefs.putUChar("cfgv", 2);
  prefs.putULong("sstart", stageStart);
  prefs.putULong("fstart", floraStart);
  prefs.putFloat("leaf", leafOff);
  prefs.putFloat("toff", tOff);
  prefs.putFloat("rhoff", rhOff);
  prefs.putUChar("scr", scrIdx);
  prefs.putULong("cstart", cropStart);
}

void loadConfig() {
  cropType    = prefs.getUChar("type", 0) % 2;
  cycleIdx    = prefs.getUChar("cycle", 0) % 4;
  lightsOnMin = prefs.getUShort("lon", 8 * 60) % 1440;
  stage       = prefs.getUChar("stage", ST_VEGE);
  if (prefs.getUChar("cfgv", 1) < 2 && prefs.isKey("stage") && stage >= ST_PLANT) stage++;   // config antigua: se agregó "Plantulas"
  if (stage >= N_STAGES) stage = ST_VEGE;
  stageStart  = prefs.getULong("sstart", 0);
  floraStart  = prefs.getULong("fstart", 0);
  leafOff     = prefs.getFloat("leaf", -1.5f);
  tOff        = prefs.getFloat("toff", 0.0f);
  rhOff       = prefs.getFloat("rhoff", 0.0f);
  scrIdx      = prefs.getUChar("scr", 1) % 5;
  cropStart   = prefs.getULong("cstart", 0);     // por defecto: sin cultivo
}

// ---------------------------------------------------------------------------
// Sensor, historial y eventos
// ---------------------------------------------------------------------------
void recalc() {
  if (isnan(rawT) || isnan(rawH)) { cur[0] = cur[1] = cur[2] = NAN; return; }
  cur[V_T]  = rawT + tOff;
  cur[V_RH] = constrain(rawH + rhOff, 0.0f, 100.0f);
  float off = dayNow ? leafOff : 0.0f;               // de noche hoja = aire
  cur[V_VPD] = calcVpd(cur[V_T], cur[V_RH], off);
}

void readSensor() {
  float t, h;
  if (sht.readBoth(&t, &h) && !isnan(t) && !isnan(h)) {
    sensorOk = true; sensorFails = 0;
    rawT = t; rawH = h;
    recalc();
  } else if (++sensorFails >= 3) {
    sensorOk = false;
    rawT = rawH = NAN; recalc();
    sht.begin(0x44);
  }
}

void pushEvent(uint8_t v, bool high, uint32_t start, uint32_t dur, float peak) {
  events[evHead] = { v, high, start, dur, peak };
  evHead = (evHead + 1) % EV_MAX;
  if (evCount < EV_MAX) evCount++;
}
void resetTracks() { for (auto& t : track) t.active = false; }

void updateEvents() {
  uint32_t now = uptimeSec();
  for (uint8_t v = 0; v < 3; v++) {
    float x = cur[v];
    if (isnan(x)) continue;
    const Range& r = rangeOf(v, dayNow);
    Track& t = track[v];
    if (!t.active) {
      if (x > r.hi)      t = { true, true,  now, x };
      else if (x < r.lo) t = { true, false, now, x };
    } else {
      if (t.high) t.peak = max(t.peak, x); else t.peak = min(t.peak, x);
      bool back = t.high ? (x <= r.hi - HYST[v]) : (x >= r.lo + HYST[v]);
      if (back) {
        uint32_t d = now - t.start;
        if (d >= EV_MIN_SEC) pushEvent(v, t.high, t.start, d, t.peak);
        t.active = false;
      }
    }
  }
}

void accumulate() { for (int v = 0; v < 3; v++) acc[v] += cur[v]; accN++; }

bool pushHistory() {
  if (accN == 0) return false;
  for (int v = 0; v < 3; v++) { hist[v][histHead] = acc[v] / accN; acc[v] = 0; }
  histTime[histHead] = uptimeSec();
  histDay[histHead]  = dayNow;
  accN = 0;
  histHead = (histHead + 1) % HIST_LEN;
  if (histCount < HIST_LEN) histCount++;
  return true;
}
int histIdx(int k, int n) { return (histHead - n + k + HIST_LEN) % HIST_LEN; }

// Cambio de etapa (con lógica de floración y ciclo de luz)
void setStage(uint8_t s) {
  if (s == stage) return;
  bool enteringFlora = isFlora(s) && !isFlora(stage);
  stage = s;
  stageStart = nowEpoch();
  if (enteringFlora) {
    floraStart = stageStart;
    if (cropType == 0) cycleIdx = 3;                 // fotoperiódica -> 12/12
  }
  if (!isFlora(stage) && stage != ST_SECADO && cropType == 0 && cycleIdx == 3) cycleIdx = 0; // vuelve a 18/6
  resetTracks();
  saveConfig();
}

// Paso automático Flora 1 -> Flora 2
void checkAutoAdvance() {
  if (!cropActive() || stage != ST_FLORA1) return;
  int d = daysSince(floraStart);
  if (d > FLORA_AUTO_DAYS) {
    stage = ST_FLORA2;
    stageStart = nowEpoch();
    resetTracks();
    saveConfig();
  }
}

// Completa fechas si el cultivo se inició sin hora real
void fillMissingDates() {
  if (!timeSynced || cropStart == 0) return;
  bool ch = false;
  if (cropStart == CROP_PENDING) { cropStart = todayPlusDays(0); ch = true; }
  if (stageStart <= CROP_PENDING) { stageStart = cropStart; ch = true; }
  if (isFlora(stage) && floraStart <= CROP_PENDING) { floraStart = stageStart; ch = true; }
  if (ch) saveConfig();
}

// Inicia (o programa) un cultivo con los datos del borrador
void startCrop() {
  cropType = nType; cycleIdx = nCycle; lightsOnMin = nLon; stage = nStage;
  if (timeSynced) cropStart = todayPlusDays(nDays);
  else            cropStart = CROP_PENDING;          // sin hora: inicia hoy, fecha se completa al sincronizar
  stageStart = cropStart;
  floraStart = isFlora(stage) ? cropStart : 0;
  resetTracks();
  saveConfig();
}

// Termina el cultivo y vuelve a modo monitor
void endCrop() {
  cropStart = 0; stageStart = 0; floraStart = 0;
  resetTracks();
  saveConfig();
}

// Prepara el borrador de nuevo cultivo con valores por defecto
void initDraft() {
  nType = 0; nCycle = 0; nLon = lightsOnMin; nStage = ST_ESQ; nDays = 0;
}

// ---------------------------------------------------------------------------
// Dibujo
// ---------------------------------------------------------------------------
void txt(const char* s, int x, int y, uint16_t col, const lgfx::IFont* f = &fonts::Font2,
         textdatum_t d = textdatum_t::top_left) {
  lcd.setFont(f); lcd.setTextDatum(d); lcd.setTextColor(col);
  lcd.drawString(s, x, y);
}

void button(const Btn& b, const char* label, bool selected) {
  lcd.fillRoundRect(b.x, b.y, b.w, b.h, 5, selected ? C_OK : C_PANEL);
  lcd.drawRoundRect(b.x, b.y, b.w, b.h, 5, selected ? C_OK : C_GRID);
  txt(label, b.x + b.w / 2, b.y + b.h / 2, selected ? C_BG : C_TXT, &fonts::Font2, textdatum_t::middle_center);
}

void clearContent() { lcd.fillRect(0, CONTENT_Y, W, CONTENT_H, C_BG); }

void drawSunMoon(int x, int y, bool day) {
  if (day) {
    lcd.fillCircle(x, y, 4, C_SUN);
    for (int k = 0; k < 8; k++) {
      float a = k * 0.785f;
      lcd.drawLine(x + cosf(a) * 6, y + sinf(a) * 6, x + cosf(a) * 8, y + sinf(a) * 8, C_SUN);
    }
  } else {
    lcd.fillCircle(x, y, 5, C_MOON);
    lcd.fillCircle(x + 3, y - 2, 4, C_BG);
  }
}

// Nombre de la etapa para la barra superior
const char* stageTitle(uint8_t cs) {
  switch (cs) {
    case ST_ESQ:    return "ESQUEJES";
    case ST_PLANT:  return "PLANTULAS";
    case ST_VEGE:   return "VEGETATIVO";
    case ST_FLORA1: return "FLORA TEMPRANA";
    case ST_FLORA2: return "FLORA TARDIA";
    case ST_SECADO: return "SECADO";
    default:        return "MONITOR";
  }
}

// Cantidad de alertas: eventos registrados + los que están ocurriendo ahora
int alertCount() {
  int n = evCount;
  for (auto& t : track) if (t.active && uptimeSec() - t.start >= EV_MIN_SEC) n++;
  return n;
}

void drawTopBar() {
  lcd.fillRect(0, 0, W, TOP_H, C_BG);
  uint8_t cs = curStage();
  const char* st = stageTitle(cs);
  txt(st, 8, TOP_H / 2 + 1, cs == ST_NONE ? C_DIM : C_OK, &fonts::Font2, textdatum_t::middle_left);
  int x = 8 + lcd.textWidth(st) + 10;
  char b[16];
  if (cropActive()) {
    int d = daysSince(cropStart);
    if (d > 0) {
      snprintf(b, sizeof b, "DIA %d", d);
      txt(b, x, TOP_H / 2 + 1, C_DIM, &fonts::Font2, textdatum_t::middle_left);
      x += lcd.textWidth(b) + 12;
    }
  }
  if (cs != ST_SECADO && cs != ST_NONE) drawSunMoon(x + 4, TOP_H / 2, dayNow);

  if (timeSynced) {
    time_t e = time(nullptr); struct tm tmv; localtime_r(&e, &tmv);
    strftime(b, sizeof b, "%H:%M", &tmv);
  } else {
    char d[12]; fmtDur(uptimeSec(), d, sizeof d); snprintf(b, sizeof b, "up %s", d);
  }
  txt(b, W - 40, TOP_H / 2 + 1, C_TXT, &fonts::Font2, textdatum_t::middle_right);

  // WiFi: barras según intensidad real de la señal
  uint16_t wc = C_GRID; int bars = 3;
  if (strlen(WIFI_SSID) > 0) {
    if (WiFi.status() == WL_CONNECTED) {
      int rssi = WiFi.RSSI();
      bars = rssi > -60 ? 3 : (rssi > -72 ? 2 : 1);
      wc = bars == 1 ? C_WARN : C_OK;
    } else wc = C_HIGH;
  }
  for (int i = 0; i < 3; i++) {
    int bh = 4 + i * 3;
    lcd.fillRect(W - 32 + i * 5, TOP_H / 2 + 6 - bh, 3, bh, i < bars ? wc : C_GRID);
  }
  lcd.fillCircle(W - 9, TOP_H / 2, 4, sensorOk ? C_OK : C_HIGH);   // estado del sensor
}

// ---- Menú inferior: 4 botones con ícono ----
void drawTabIcon(int i, int cx, int cy, uint16_t c) {
  switch (i) {
    case 0:                                                       // medidor
      lcd.drawArc(cx, cy + 4, 10, 8, 180, 360, c);
      lcd.drawLine(cx, cy + 4, cx + 5, cy - 3, c);
      lcd.drawLine(cx + 1, cy + 4, cx + 6, cy - 3, c);
      lcd.fillCircle(cx, cy + 4, 2, c);
      break;
    case 1:                                                       // gráfico
      lcd.drawFastHLine(cx - 10, cy + 8, 20, c);
      lcd.drawLine(cx - 9, cy + 4, cx - 4, cy - 1, c); lcd.drawLine(cx - 9, cy + 5, cx - 4, cy, c);
      lcd.drawLine(cx - 4, cy - 1, cx, cy + 2, c);     lcd.drawLine(cx - 4, cy, cx, cy + 3, c);
      lcd.drawLine(cx, cy + 2, cx + 9, cy - 7, c);     lcd.drawLine(cx, cy + 3, cx + 9, cy - 6, c);
      break;
    case 2:                                                       // lista
      for (int k = -1; k <= 1; k++) {
        lcd.fillRect(cx - 10, cy + k * 6 - 1, 3, 3, c);
        lcd.fillRect(cx - 5, cy + k * 6 - 1, 15, 2, c);
      }
      break;
    case 3:                                                       // engranaje
      lcd.drawCircle(cx, cy, 6, c); lcd.drawCircle(cx, cy, 5, c);
      lcd.fillCircle(cx, cy, 2, c);
      for (int k = 0; k < 8; k++) {
        float a = k * 0.785f;
        lcd.drawLine(cx + cosf(a) * 7, cy + sinf(a) * 7, cx + cosf(a) * 10, cy + sinf(a) * 10, c);
      }
      break;
  }
}

int lastAlertsDrawn = -1;

void drawTabs() {
  lcd.fillRect(0, TAB_Y, W, TAB_H, C_BG);
  int alerts = alertCount();
  lastAlertsDrawn = alerts;
  for (int i = 0; i < 4; i++) {
    bool sel = (i == screen);
    int x = 6 + i * 78, y = TAB_Y + 3, w = 72, h = TAB_H - 6;
    lcd.fillRoundRect(x, y, w, h, 6, sel ? C_OK : C_PANEL);
    uint16_t ic = sel ? C_BG : rgb(195, 204, 218);
    int cx = x + w / 2, cy = y + h / 2;
    if (i == 2 && alerts > 0) {
      drawTabIcon(i, cx - 9, cy, ic);
      char b[6]; snprintf(b, sizeof b, "%d", alerts > 99 ? 99 : alerts);
      txt(b, cx + 6, cy + 1, sel ? C_BG : C_HIGH, &fonts::Font2, textdatum_t::middle_left);
    } else drawTabIcon(i, cx, cy, ic);
  }
}

// ---- En vivo: medidores circulares (estilo B) ----
LGFX_Sprite gaugeSpr(&lcd);
bool gaugeOk = false;
#define GAUGE_W   106
#define GAUGE_H   126
#define GAUGE_Y   (CONTENT_Y + 2)
const float GAUGE_LO[3]   = { 12.0f, 20.0f, 0.0f };     // escala de cada medidor
const float GAUGE_HI[3]   = { 36.0f, 100.0f, 2.0f };
const char* GAUGE_NAME[3] = { "TEMPERATURA", "HUMEDAD", "VPD" };
const uint16_t C_TRACK = rgb(26, 34, 51);
const uint16_t C_ZONE  = rgb(31, 110, 80);

void gaugeAllocate() {
  if (gaugeOk) return;
  gaugeSpr.setColorDepth(16);
  gaugeOk = gaugeSpr.createSprite(GAUGE_W, GAUGE_H) != nullptr;
}
void gaugeRelease() { if (gaugeOk) { gaugeSpr.deleteSprite(); gaugeOk = false; } }

// Arco de a0 a a1 grados (0 = derecha, sentido horario). Puede pasar de 360.
void gaugeArc(int cx, int cy, int r0, int r1, float a0, float a1, uint16_t col) {
  if (a1 <= a0) return;
  if (a0 >= 360) { a0 -= 360; a1 -= 360; }
  if (a1 > 360) { gaugeSpr.fillArc(cx, cy, r0, r1, a0, 360, col); gaugeSpr.fillArc(cx, cy, r0, r1, 0, a1 - 360, col); }
  else gaugeSpr.fillArc(cx, cy, r0, r1, a0, a1, col);
}

float gaugeFrac(uint8_t v, float x) { return constrain((x - GAUGE_LO[v]) / (GAUGE_HI[v] - GAUGE_LO[v]), 0.0f, 1.0f); }

void drawGauge(uint8_t v) {
  if (!gaugeOk) return;
  float x = cur[v];
  int st = statusOf(v, x);
  uint16_t col = isnan(x) ? C_DIM : statusColor(st);
  const Range& r = rangeOf(v, dayNow);
  const int cx = GAUGE_W / 2, cy = 44;
  const float A0 = 135, SPAN = 270;

  gaugeSpr.fillScreen(C_BG);
  gaugeArc(cx, cy, 40, 42, A0 + SPAN * gaugeFrac(v, r.lo), A0 + SPAN * gaugeFrac(v, r.hi), C_ZONE);   // zona ideal
  gaugeArc(cx, cy, 30, 37, A0, A0 + SPAN, C_TRACK);                                                   // pista
  if (!isnan(x)) {
    float a1 = A0 + SPAN * gaugeFrac(v, x);
    gaugeArc(cx, cy, 30, 37, A0, max(a1, A0 + 3.0f), col);                                              // valor
    gaugeSpr.fillCircle(cx + cosf(a1 * 0.01745f) * 33.5f, cy + sinf(a1 * 0.01745f) * 33.5f, 4, col);
  }

  char b[24];
  if (isnan(x)) snprintf(b, sizeof b, "--"); else snprintf(b, sizeof b, "%.*f", VAR_DEC[v], x);
  gaugeSpr.setFont(&fonts::FreeSansBold12pt7b);
  gaugeSpr.setTextDatum(textdatum_t::middle_center);
  gaugeSpr.setTextColor(col);
  gaugeSpr.drawString(b, cx, cy - 2);
  gaugeSpr.setFont(&fonts::Font2);
  gaugeSpr.setTextColor(C_DIM);
  gaugeSpr.drawString(v == V_T ? "C" : VAR_UNIT[v], cx, cy + 17);
  if (v == V_T) gaugeSpr.drawCircle(cx - 6, cy + 13, 2, C_DIM);   // símbolo de grado

  gaugeSpr.setTextColor(C_TXT);
  gaugeSpr.drawString(GAUGE_NAME[v], cx, 92);

  // Chip de estado con el rango ideal
  if (isnan(x)) snprintf(b, sizeof b, "SIN DATO");
  else {
    const char* sw = st > 0 ? (v == V_VPD ? "ALTO" : "ALTA") : (st < 0 ? (v == V_VPD ? "BAJO" : "BAJA") : "OK");
    if (v == V_VPD) snprintf(b, sizeof b, "%s %.1f-%.1f", sw, r.lo, r.hi);
    else            snprintf(b, sizeof b, "%s %.0f-%.0f", sw, r.lo, r.hi);
  }
  int tw = gaugeSpr.textWidth(b) + 12;
  if (tw > GAUGE_W - 2) tw = GAUGE_W - 2;
  if (st == 0 && !isnan(x)) {
    gaugeSpr.drawRoundRect(cx - tw / 2, 104, tw, 18, 4, C_GRID);
    gaugeSpr.setTextColor(C_DIM);
  } else {
    gaugeSpr.fillRoundRect(cx - tw / 2, 104, tw, 18, 4, col);
    gaugeSpr.setTextColor(C_BG);
  }
  gaugeSpr.drawString(b, cx, 113);

  gaugeSpr.pushSprite(v * 107, GAUGE_Y);
}

void drawLive() { for (uint8_t v = 0; v < 3; v++) drawGauge(v); }

// Franja inferior: tendencia del VPD en 6 h + estado de las luces
void drawTrendStrip() {
  const int y = GAUGE_Y + GAUGE_H + 2, h = TAB_Y - y - 2;
  lcd.fillRoundRect(6, y, W - 12, h, 8, C_PANEL);

  // Texto de tendencia
  txt("VPD 6h", 14, y + 6, C_DIM, &fonts::Font2);
  const char* tr = "sin datos"; uint16_t tc = C_DIM;
  if (histCount >= 60) {
    float a = 0, b2 = 0;
    for (int k = 0; k < 30; k++) { a += hist[V_VPD][histIdx(histCount - 30 + k, histCount)]; b2 += hist[V_VPD][histIdx(histCount - 60 + k, histCount)]; }
    float d = (a - b2) / 30;
    if (d > 0.03f) { tr = "subiendo"; tc = C_HIGH; }
    else if (d < -0.03f) { tr = "bajando"; tc = C_LOW; }
    else { tr = "estable"; tc = C_OK; }
  }
  txt(tr, 14, y + 24, tc, &fonts::Font2);

  // Mini gráfico con la zona ideal
  const int gx = 84, gw = 132, gy = y + 6, gh = h - 12;
  const Range& r = rangeOf(V_VPD, dayNow);
  auto yOf = [&](float v) { return gy + gh - 1 - (int)(constrain(v / 2.0f, 0.0f, 1.0f) * (gh - 1)); };
  lcd.fillRect(gx, yOf(r.hi), gw, yOf(r.lo) - yOf(r.hi) + 1, C_BAND);
  int n = min(histCount, 360);
  if (n >= 2) {
    int px = 0, py = 0;
    for (int k = 0; k < gw; k++) {
      int idx = histIdx((int)((long)k * (n - 1) / (gw - 1)), n);
      int xx = gx + k, yy = yOf(hist[V_VPD][idx]);
      if (k) { lcd.drawLine(px, py, xx, yy, C_LOW); lcd.drawLine(px, py + 1, xx, yy + 1, C_LOW); }
      px = xx; py = yy;
    }
  }

  // Estado de las luces
  const int lx = 228;
  char b[24];
  uint8_t cs = curStage();
  if (cs == ST_NONE) {
    txt("Sin", lx, y + 6, C_DIM, &fonts::Font2); txt("cultivo", lx, y + 24, C_TXT, &fonts::Font2);
  } else if (cs == ST_SECADO) {
    txt("Secado", lx, y + 6, C_DIM, &fonts::Font2); txt("oscuro", lx, y + 24, C_TXT, &fonts::Font2);
  } else if (CYCLES[cycleIdx].on >= 24) {
    drawSunMoon(lx + 6, y + 14, true);
    txt("Luz", lx + 18, y + 6, C_DIM, &fonts::Font2); txt("24/0", lx, y + 24, C_TXT, &fonts::Font2);
  } else {
    drawSunMoon(lx + 6, y + 14, dayNow);
    snprintf(b, sizeof b, "Luz %s", CYCLES[cycleIdx].name);
    txt(b, lx + 16, y + 6, C_DIM, &fonts::Font2);
    int t = dayNow ? (lightsOnMin + CYCLES[cycleIdx].on * 60) % 1440 : lightsOnMin;
    snprintf(b, sizeof b, "%s %02d:%02d", dayNow ? "off" : "on", t / 60, t % 60);
    txt(b, lx, y + 24, C_TXT, &fonts::Font2);
  }
}

void drawLiveFull() { clearContent(); drawLive(); drawTrendStrip(); }

// ---- Gráfico ----
Btn gVarBtn[3] = { { 4, 26, 58, 22 }, { 66, 26, 58, 22 }, { 128, 26, 58, 22 } };
Btn gWinBtn    = { 256, 26, 60, 22 };
const int PX = 40, PY = 54, PW = 274, PH = 144;

void drawGraph() {
  clearContent();
  for (int i = 0; i < 3; i++) button(gVarBtn[i], VAR_SHORT[i], i == graphVar);
  button(gWinBtn, WIN_LBL[graphWin], false);

  uint8_t v = graphVar;
  const Range& r = rangeOf(v, dayNow);
  int winS = WIN_SEC[graphWin] / HIST_SEC;
  int n = min(histCount, winS);

  lcd.drawRect(PX - 1, PY - 1, PW + 2, PH + 2, C_GRID);
  if (n < 2) {
    txt("Recopilando datos...", PX + PW / 2, PY + PH / 2 - 8, C_DIM, &fonts::Font2, textdatum_t::middle_center);
    txt("(1 punto por minuto)", PX + PW / 2, PY + PH / 2 + 10, C_DIM, &fonts::Font2, textdatum_t::middle_center);
    return;
  }

  float lo = r.lo, hi = r.hi;
  for (int k = 0; k < n; k++) { float x = hist[v][histIdx(k, n)]; lo = min(lo, x); hi = max(hi, x); }
  float pad = (hi - lo) * 0.1f; if (pad <= 0) pad = 1;
  lo -= pad; hi += pad;
  auto yOf = [&](float val) { return PY + PH - 1 - (int)((val - lo) / (hi - lo) * (PH - 1)); };

  int yHi = yOf(r.hi), yLo = yOf(r.lo);
  lcd.fillRect(PX, yHi, PW, yLo - yHi, C_BAND);

  char b[12];
  for (int k = 0; k <= 4; k++) {
    float val = lo + (hi - lo) * k / 4;
    int y = yOf(val);
    lcd.drawFastHLine(PX, y, PW, C_GRID);
    snprintf(b, sizeof b, "%.*f", v == V_RH ? 0 : VAR_DEC[v], val);
    txt(b, PX - 3, y, C_DIM, &fonts::Font0, textdatum_t::middle_right);
  }

  int px = 0, py = 0;
  for (int k = 0; k < n; k++) {
    int i = histIdx(k, n);
    float val = hist[v][i];
    int x = PX + (PW - 1) - (int)((long)(n - 1 - k) * (PW - 1) / (winS - 1));
    int y = yOf(val);
    // franja inferior: día (amarillo) / noche (azul)
    if (curStage() != ST_SECADO && curStage() != ST_NONE) lcd.drawFastVLine(x, PY + PH - 3, 3, histDay[i] ? C_SUN : C_MOON);
    if (k > 0) {
      int s = statusAt(v, val, histDay[i]);
      uint16_t c = s == 0 ? C_TXT : statusColor(s);
      lcd.drawLine(px, py, x, y, c);
      lcd.drawLine(px, py + 1, x, y + 1, c);
    }
    px = x; py = y;
  }
  txt("ahora", PX + PW - 2, PY + 2, C_DIM, &fonts::Font0, textdatum_t::top_right);
}

// ---- Reportes ----
Btn rClearBtn = { 252, 102, 64, 20 };

void drawReport() {
  clearContent();
  int y0 = CONTENT_Y + 4;
  txt("24 h", 8, y0, C_DIM);
  txt("Minimo", 62, y0, C_DIM);
  txt("Maximo", 152, y0, C_DIM);
  txt("En rango", 248, y0, C_DIM);

  char b[48], t1[10];
  for (uint8_t v = 0; v < 3; v++) {
    int y = y0 + 20 + v * 18;
    txt(VAR_SHORT[v], 8, y, C_TXT);
    if (histCount == 0) { txt("--", 62, y, C_DIM); txt("--", 152, y, C_DIM); txt("--", 248, y, C_DIM); continue; }
    int iMin = histIdx(0, histCount), iMax = iMin, inR = 0;
    for (int k = 0; k < histCount; k++) {
      int i = histIdx(k, histCount);
      if (hist[v][i] < hist[v][iMin]) iMin = i;
      if (hist[v][i] > hist[v][iMax]) iMax = i;
      if (statusAt(v, hist[v][i], histDay[i]) == 0) inR++;
    }
    fmtAt(histTime[iMin], t1, sizeof t1);
    snprintf(b, sizeof b, "%.*f %s", VAR_DEC[v], hist[v][iMin], t1);
    txt(b, 62, y, statusColor(statusAt(v, hist[v][iMin], histDay[iMin])));
    fmtAt(histTime[iMax], t1, sizeof t1);
    snprintf(b, sizeof b, "%.*f %s", VAR_DEC[v], hist[v][iMax], t1);
    txt(b, 152, y, statusColor(statusAt(v, hist[v][iMax], histDay[iMax])));
    int pct = inR * 100 / histCount;
    snprintf(b, sizeof b, "%d%%", pct);
    txt(b, 248, y, pct >= 90 ? C_OK : (pct >= 70 ? C_WARN : C_HIGH));
  }

  lcd.drawFastHLine(4, CONTENT_Y + 78, W - 8, C_GRID);
  int active = 0;
  for (auto& t : track) if (t.active && uptimeSec() - t.start >= EV_MIN_SEC) active++;
  snprintf(b, sizeof b, "Fuera de rango (%d)", evCount + active);
  txt(b, 8, 104, C_TXT);
  button(rClearBtn, "Borrar", false);

  int y = 126, lines = 0;
  char d[12];
  for (uint8_t v = 0; v < 3 && lines < 4; v++) {
    Track& t = track[v];
    uint32_t dur = uptimeSec() - t.start;
    if (!t.active || dur < EV_MIN_SEC) continue;
    fmtDur(dur, d, sizeof d);
    snprintf(b, sizeof b, "AHORA %s %s pico %.*f  %s", VAR_SHORT[v], t.high ? "ALTO" : "BAJO", VAR_DEC[v], t.peak, d);
    txt(b, 8, y, statusColor(t.high ? 1 : -1));
    y += 18; lines++;
  }
  for (int k = 0; k < evCount && lines < 4; k++) {
    const Event& e = events[(evHead - 1 - k + EV_MAX) % EV_MAX];
    fmtAt(e.start, t1, sizeof t1);
    fmtDur(e.dur, d, sizeof d);
    snprintf(b, sizeof b, "%s %s %s pico %.*f  %s", t1, VAR_SHORT[e.var], e.high ? "ALTO" : "BAJO", VAR_DEC[e.var], e.peak, d);
    txt(b, 8, y, statusColor(e.high ? 1 : -1));
    y += 18; lines++;
  }
  if (lines == 0) txt("Sin eventos. Todo en rango.", 8, y, C_OK);
}

// ---- Ajustes ----
Btn sPageBtn[3]  = { { 6, 25, 100, 20 }, { 110, 25, 100, 20 }, { 214, 25, 100, 20 } };
// Página Cultivo
Btn sTypeBtn[2]  = { { 70, 50, 120, 22 }, { 194, 50, 120, 22 } };
Btn sCycleBtn[4] = { { 70, 77, 58, 22 }, { 132, 77, 58, 22 }, { 194, 77, 58, 22 }, { 256, 77, 58, 22 } };
Btn sOnMinus     = { 70, 104, 36, 22 };
Btn sOnPlus      = { 170, 104, 36, 22 };
Btn sStPrev      = { 70, 131, 36, 22 };
Btn sStNext      = { 278, 131, 36, 22 };
Btn sEndBtn      = { 222, 158, 92, 22 };
Btn sNewCrop     = { 60, 118, 200, 32 };
Btn sCancelProg  = { 60, 150, 200, 26 };
Btn sDaysM       = { 70, 158, 36, 22 };
Btn sDaysP       = { 278, 158, 36, 22 };
Btn sCancelNew   = { 6, 182, 110, 20 };
Btn sStartNew    = { 204, 182, 110, 20 };
// Página Sensor
Btn sLeafM = { 170, 49, 36, 22 },  sLeafP = { 278, 49, 36, 22 };
Btn sTM    = { 170, 75, 36, 22 },  sTP    = { 278, 75, 36, 22 };
Btn sHM    = { 170, 101, 36, 22 }, sHP    = { 278, 101, 36, 22 };
Btn sScrBtn = { 170, 127, 144, 22 };
Btn sCalBtn = { 6, 153, 140, 22 };

// Filas comunes: tipo, ciclo de luz, hora de encendido y etapa
void drawCropForm(uint8_t type, uint8_t cyc, uint16_t lon, uint8_t st, const char* stageLabel) {
  char b[40];
  txt("Tipo", 8, 54, C_DIM);
  button(sTypeBtn[0], "Fotoperiodica", type == 0);
  button(sTypeBtn[1], "Automatica", type == 1);

  txt("Luz", 8, 81, C_DIM);
  for (int i = 0; i < 4; i++) button(sCycleBtn[i], CYCLES[i].name, i == cyc);

  txt("Enciende", 8, 108, C_DIM);
  button(sOnMinus, "-", false);
  button(sOnPlus, "+", false);
  snprintf(b, sizeof b, "%02d:%02d", lon / 60, lon % 60);
  txt(b, 138, 115, C_TXT, &fonts::Font2, textdatum_t::middle_center);
  if (CYCLES[cyc].on < 24) {
    int off = (lon + CYCLES[cyc].on * 60) % 1440;
    snprintf(b, sizeof b, "apaga %02d:%02d", off / 60, off % 60);
    txt(b, 214, 108, C_DIM);
  }

  txt(stageLabel, 8, 135, C_DIM);
  button(sStPrev, "<", false);
  button(sStNext, ">", false);
  txt(STAGES[st].name, 192, 142, C_OK, &fonts::Font2, textdatum_t::middle_center);
}

// Aviso según tipo, ciclo y etapa
const char* cropWarning(uint8_t type, uint8_t cyc, uint8_t st) {
  if (type == 0 && isFlora(st) && cyc != 3) return "Fotoperiodica en flora: usa 12/12";
  if (type == 0 && !isFlora(st) && st != ST_SECADO && cyc == 3) return "12/12 hace florecer a la fotoperiodica";
  if (type == 1 && cyc == 3) return "Automatica: mejor 18/6, 20/4 o 24/0";
  if (!timeSynced && CYCLES[cyc].on < 24) return "Sin hora (WiFi): se asume luz encendida";
  return nullptr;
}

// Estado 1: sin cultivo
void drawNoCrop() {
  const Stage& s = STAGES[ST_NONE];
  char b[64];
  txt("No hay un cultivo activo.", 8, 56, C_TXT);
  txt("El equipo funciona como monitor general:", 8, 76, C_DIM);
  snprintf(b, sizeof b, "T %.0f-%.0fC   HR %.0f-%.0f%%   VPD %.1f-%.1f",
           s.day[0].lo, s.day[0].hi, s.day[1].lo, s.day[1].hi, s.day[2].lo, s.day[2].hi);
  txt(b, 8, 94, C_DIM);
  button(sNewCrop, "+ Nuevo cultivo", true);
  txt("Puedes iniciarlo hoy o programar una fecha", 8, 166, C_DIM);
}

// Estado 2: cultivo programado para una fecha futura
void drawProgrammed() {
  char b[64], d[16];
  fmtDate(cropStart, d, sizeof d);
  uint32_t n = nowEpoch();
  int faltan = (int)((cropStart - n + 86399) / 86400);
  txt("Cultivo programado", 8, 54, C_OK);
  snprintf(b, sizeof b, "Inicia el %s (en %d dia%s)", d, faltan, faltan == 1 ? "" : "s");
  txt(b, 8, 74, C_TXT);
  snprintf(b, sizeof b, "%s  |  Luz %s desde %02d:%02d",
           cropType == 0 ? "Fotoperiodica" : "Automatica", CYCLES[cycleIdx].name, lightsOnMin / 60, lightsOnMin % 60);
  txt(b, 8, 94, C_DIM);
  snprintf(b, sizeof b, "Etapa inicial: %s", STAGES[stage].name);
  txt(b, 8, 112, C_DIM);
  button(sCancelProg, "Cancelar programacion", false);
  txt("Mientras tanto: modo monitor general", 8, 184, C_DIM);
}

// Estado 3: cultivo activo
void drawActiveCrop() {
  char b[64];
  drawCropForm(cropType, cycleIdx, lightsOnMin, stage, "Etapa");

  int dc = daysSince(cropStart);
  if (dc < 0) snprintf(b, sizeof b, "Dias: requiere WiFi");
  else if (isFlora(stage)) { int df = daysSince(floraStart); snprintf(b, sizeof b, "Dia %d | Flora sem %d", dc, df < 1 ? 1 : (df - 1) / 7 + 1); }
  else { int ds = daysSince(stageStart); snprintf(b, sizeof b, "Dia %d | %s dia %d", dc, STAGES[stage].shortName, ds); }
  txt(b, 8, 163, C_TXT);
  button(sEndBtn, confirmEnd ? "Seguro?" : "Terminar", confirmEnd);

  const char* warn = cropWarning(cropType, cycleIdx, stage);
  if (confirmEnd) txt("Toca 'Seguro?' para terminar el cultivo", 8, 186, C_WARN);
  else if (warn) txt(warn, 8, 186, C_WARN);
  else {
    const Stage& s = STAGES[stage];
    snprintf(b, sizeof b, "Dia T%.0f-%.0f HR%.0f-%.0f  Noche T%.0f-%.0f",
             s.day[0].lo, s.day[0].hi, s.day[1].lo, s.day[1].hi, s.night[0].lo, s.night[0].hi);
    txt(b, 8, 186, C_DIM);
  }
}

// Pantalla "Nuevo cultivo" (borrador)
void drawNewCrop() {
  char b[48], d[16];
  drawCropForm(nType, nCycle, nLon, nStage, "Inicia en");

  txt("Fecha", 8, 162, C_DIM);
  button(sDaysM, "-", false);
  button(sDaysP, "+", false);
  if (!timeSynced) snprintf(b, sizeof b, "Hoy (sin WiFi)");
  else {
    fmtDate(todayPlusDays(nDays), d, sizeof d);
    if (nDays == 0)     snprintf(b, sizeof b, "Hoy %s", d);
    else if (nDays > 0) snprintf(b, sizeof b, "%s (+%d d)", d, nDays);
    else                snprintf(b, sizeof b, "%s (%d d)", d, nDays);
  }
  txt(b, 192, 169, nDays > 0 ? C_WARN : C_TXT, &fonts::Font2, textdatum_t::middle_center);

  button(sCancelNew, "Cancelar", false);
  button(sStartNew, nDays > 0 ? "Programar" : "Iniciar", true);
  const char* warn = cropWarning(nType, nCycle, nStage);
  if (warn) txt(warn, 316, 35, C_WARN, &fonts::Font0, textdatum_t::middle_right);
}

void drawSetCultivo() {
  if (cropProgrammed())   drawProgrammed();
  else if (cropActive())  drawActiveCrop();
  else                    drawNoCrop();
}

void drawSetSensor() {
  char b[48];
  txt("Temp. hoja vs aire", 8, 53, C_TXT);
  button(sLeafM, "-", false); button(sLeafP, "+", false);
  snprintf(b, sizeof b, "%+.1f C", leafOff);
  txt(b, 242, 60, C_TXT, &fonts::Font2, textdatum_t::middle_center);

  txt("Ajuste temperatura", 8, 79, C_TXT);
  button(sTM, "-", false); button(sTP, "+", false);
  snprintf(b, sizeof b, "%+.1f C", tOff);
  txt(b, 242, 86, C_TXT, &fonts::Font2, textdatum_t::middle_center);

  txt("Ajuste humedad", 8, 105, C_TXT);
  button(sHM, "-", false); button(sHP, "+", false);
  snprintf(b, sizeof b, "%+.1f %%", rhOff);
  txt(b, 242, 112, C_TXT, &fonts::Font2, textdatum_t::middle_center);

  txt("Apagar pantalla", 8, 131, C_TXT);
  snprintf(b, sizeof b, "tras %s", SCR_LBL[scrIdx]);
  button(sScrBtn, scrIdx == 4 ? "Nunca" : b, false);

  button(sCalBtn, "Calibrar tactil", false);
  if (strlen(WIFI_SSID) == 0) snprintf(b, sizeof b, "WiFi: no configurado");
  else if (WiFi.status() == WL_CONNECTED) snprintf(b, sizeof b, "WiFi OK %d dBm", WiFi.RSSI());
  else {
    wl_status_t st = WiFi.status();
    const char* why = st == WL_NO_SSID_AVAIL ? "red no encontrada" :
                      st == WL_CONNECT_FAILED ? "clave incorrecta?" :
                      st == WL_CONNECTION_LOST ? "conexion perdida" : "conectando...";
    snprintf(b, sizeof b, "WiFi: %s", why);
  }
  txt(b, 156, 157, C_DIM);

  if (isnan(rawT)) snprintf(b, sizeof b, "Lectura sin ajuste: --");
  else snprintf(b, sizeof b, "Lectura sin ajuste: %.2f C  %.1f %%", rawT, rawH);
  txt(b, 8, 183, C_DIM);
}

void drawSettings() {
  clearContent();
  if (setPage == 2) {                       // Nuevo cultivo
    txt("Nuevo cultivo", 8, 35, C_OK, &fonts::Font2, textdatum_t::middle_left);
    drawNewCrop();
    return;
  }
  button(sPageBtn[0], "Cultivo", setPage == 0);
  button(sPageBtn[1], "Sensor", setPage == 1);
  button(sPageBtn[2], "Juego", false);
  if (setPage == 0) drawSetCultivo(); else drawSetSensor();
}

void drawContent() {
  switch (screen) {
    case S_LIVE:   drawLiveFull(); break;
    case S_GRAPH:  drawGraph();    break;
    case S_REPORT: drawReport();   break;
    case S_SET:    drawSettings(); break;
  }
}
void drawAll() { lcd.fillScreen(C_BG); drawTopBar(); drawContent(); drawTabs(); }

// ---------------------------------------------------------------------------
// Táctil
// ---------------------------------------------------------------------------
void runCalibration() {
  lcd.fillScreen(C_BG);
  txt("Calibracion tactil", W / 2, 100, C_TXT, &fonts::Font4, textdatum_t::middle_center);
  txt("Toca la punta de cada flecha", W / 2, 130, C_DIM, &fonts::Font2, textdatum_t::middle_center);
  uint16_t cal[8];
  delay(500);                              // espera a que levantes el dedo
  lcd.calibrateTouch(cal, C_TXT, C_BG, 15);
  lcd.setTouchCalibrate(cal);
  prefs.putBytes("tcal", cal, sizeof cal);
}

void afterSensorSettingChange() { saveConfig(); recalc(); drawSettings(); }

// Toques en la pantalla "Nuevo cultivo"
void handleNewCropTouch(int x, int y) {
  for (int i = 0; i < 2; i++) if (inBtn(sTypeBtn[i], x, y)) {
    nType = i;
    if (nType == 1 && nCycle == 3) nCycle = 0;
    if (nType == 0 && isFlora(nStage)) nCycle = 3;
    drawSettings(); return;
  }
  for (int i = 0; i < 4; i++) if (inBtn(sCycleBtn[i], x, y)) { nCycle = i; drawSettings(); return; }
  if (inBtn(sOnMinus, x, y) || inBtn(sOnPlus, x, y)) {
    nLon = (nLon + 1440 + (inBtn(sOnPlus, x, y) ? 30 : -30)) % 1440;
    drawSettings(); return;
  }
  if (inBtn(sStPrev, x, y) || inBtn(sStNext, x, y)) {
    int s = nStage + (inBtn(sStNext, x, y) ? 1 : -1);
    nStage = (s + N_STAGES) % N_STAGES;
    if (nType == 0 && isFlora(nStage)) nCycle = 3;
    drawSettings(); return;
  }
  if (timeSynced && inBtn(sDaysM, x, y)) { nDays = max(nDays - 1, -120); drawSettings(); return; }
  if (timeSynced && inBtn(sDaysP, x, y)) { nDays = min(nDays + 1, 120);  drawSettings(); return; }
  if (inBtn(sCancelNew, x, y)) { setPage = 0; drawSettings(); return; }
  if (inBtn(sStartNew, x, y)) {
    startCrop();
    setPage = 0;
    dayNow = computeDay(); recalc();
    drawAll(); return;
  }
}

void handleSettingsTouch(int x, int y) {
  if (setPage == 2) { handleNewCropTouch(x, y); return; }

  bool wasConfirm = confirmEnd;
  confirmEnd = false;
  if (inBtn(sPageBtn[0], x, y)) { setPage = 0; drawSettings(); return; }
  if (inBtn(sPageBtn[1], x, y)) { setPage = 1; drawSettings(); return; }
  if (inBtn(sPageBtn[2], x, y)) { startGame(); return; }

  if (setPage == 0 && !cropActive()) {
    // Sin cultivo o programado
    if (!cropProgrammed() && inBtn(sNewCrop, x, y)) { initDraft(); setPage = 2; drawSettings(); return; }
    if (cropProgrammed() && inBtn(sCancelProg, x, y)) { endCrop(); drawAll(); return; }
    return;
  }

  if (setPage == 0) {
    if (inBtn(sEndBtn, x, y)) {
      if (wasConfirm) { endCrop(); dayNow = computeDay(); recalc(); drawAll(); }
      else { confirmEnd = true; drawSettings(); }
      return;
    }
    if (wasConfirm) { drawSettings(); }   // cualquier otro toque cancela la confirmación
    for (int i = 0; i < 2; i++) if (inBtn(sTypeBtn[i], x, y)) {
      cropType = i;
      if (cropType == 1 && cycleIdx == 3) cycleIdx = 0;          // auto no usa 12/12
      if (cropType == 0 && isFlora(stage)) cycleIdx = 3;         // foto en flora -> 12/12
      saveConfig(); dayNow = computeDay(); drawAll(); return;
    }
    for (int i = 0; i < 4; i++) if (inBtn(sCycleBtn[i], x, y)) {
      cycleIdx = i; saveConfig(); dayNow = computeDay(); recalc(); drawAll(); return;
    }
    if (inBtn(sOnMinus, x, y) || inBtn(sOnPlus, x, y)) {
      int step = inBtn(sOnPlus, x, y) ? 30 : -30;
      lightsOnMin = (lightsOnMin + 1440 + step) % 1440;
      saveConfig(); dayNow = computeDay(); recalc(); drawAll(); return;
    }
    if (inBtn(sStPrev, x, y) || inBtn(sStNext, x, y)) {
      int s = stage + (inBtn(sStNext, x, y) ? 1 : -1);
      s = (s + N_STAGES) % N_STAGES;
      setStage(s); dayNow = computeDay(); recalc(); drawAll(); return;
    }
  } else {
    if (inBtn(sLeafM, x, y)) { leafOff = constrain(leafOff - 0.5f, -5.0f, 3.0f); afterSensorSettingChange(); return; }
    if (inBtn(sLeafP, x, y)) { leafOff = constrain(leafOff + 0.5f, -5.0f, 3.0f); afterSensorSettingChange(); return; }
    if (inBtn(sTM, x, y))    { tOff  = constrain(tOff - 0.1f, -5.0f, 5.0f);      afterSensorSettingChange(); return; }
    if (inBtn(sTP, x, y))    { tOff  = constrain(tOff + 0.1f, -5.0f, 5.0f);      afterSensorSettingChange(); return; }
    if (inBtn(sHM, x, y))    { rhOff = constrain(rhOff - 0.5f, -15.0f, 15.0f);   afterSensorSettingChange(); return; }
    if (inBtn(sHP, x, y))    { rhOff = constrain(rhOff + 0.5f, -15.0f, 15.0f);   afterSensorSettingChange(); return; }
    if (inBtn(sScrBtn, x, y)) { scrIdx = (scrIdx + 1) % 5; saveConfig(); drawSettings(); return; }
    if (inBtn(sCalBtn, x, y)) { runCalibration(); lastActivity = millis(); drawAll(); return; }
  }
}

void handleTouch(int x, int y) {
  if (y >= TAB_Y) {
    uint8_t s = constrain(x / 80, 0, 3);
    if (s != screen) {
      if (screen == S_SET && setPage == 2) setPage = 0;   // salir de "Nuevo cultivo" sin guardar
      confirmEnd = false;
      screen = s; drawAll();
    }
    return;
  }
  switch (screen) {
    case S_GRAPH:
      for (int i = 0; i < 3; i++) if (inBtn(gVarBtn[i], x, y)) { graphVar = i; drawGraph(); return; }
      if (inBtn(gWinBtn, x, y)) { graphWin = (graphWin + 1) % 3; drawGraph(); }
      break;
    case S_REPORT:
      if (inBtn(rClearBtn, x, y)) { evCount = 0; evHead = 0; resetTracks(); drawReport(); }
      break;
    case S_SET:
      handleSettingsTouch(x, y);
      break;
  }
}

// ---------------------------------------------------------------------------
// Apagado / encendido de pantalla
// ---------------------------------------------------------------------------
void screenSleep() {
  if (!screenOn) return;
  screenOn = false;
  lcd.setBrightness(0);   // apaga la luz de fondo
  lcd.sleep();            // pone el controlador en reposo
}

void screenWake() {
  lastActivity = millis();
  if (screenOn) return;
  screenOn = true;
  lcd.wakeup();
  drawAll();
  lcd.setBrightness(BRIGHTNESS);
}

// ¿Hay alguna variable fuera de rango hace más de 1 minuto?
bool alarmActive() {
  for (auto& t : track) if (t.active && uptimeSec() - t.start >= EV_MIN_SEC) return true;
  return false;
}

// ===========================================================================
//  JUEGO "COGOLLO" — plataformas original en 3 mundos
//   Mundo 1: El indoor (con maceta-portal al subsuelo de raices)
//   Mundo 2: La mata (nivel vertical: trepar la planta hasta el cogollo principal)
//   Mundo 3: Invernadero de noche (termina con la Reina Arana)
//  Personajes, enemigos y niveles propios.
// ===========================================================================
#define G_T        16
#define G_VROWS    12      // filas visibles
#define G_HUD      16
#define G_VW       320
#define G_VH       208     // HUD + 12 filas
#define G_CTRL_Y   208     // franja de controles táctiles
#define G_MAPSZ    2208
#define G_GRAV     0.5f
#define G_MAXFALL  9.0f
#define G_ACC      0.45f
#define G_MAXSPD   3.0f
#define G_JUMPV    9.0f      // salto un poco más alto: más cómodo en pantalla táctil
#define G_MAXEN    32
#define G_MAXIT    4
#define G_MAXPOP   16
#define G_UCOLS    44
#define G_EXIT_COL 118
#define G_ARENA_COL 144

const uint16_t G_WHITE = rgb(255, 255, 255);
const uint16_t G_BLACK = rgb(0, 0, 0);
const uint16_t G_DROP  = rgb(60, 145, 235);

enum { EN_MITE = 0, EN_APHID, EN_MOSQ, EN_BOSS };
enum { PW_NONE = 0, PW_SUN, PW_WATER };
enum { G_TITLE = 0, G_INTRO, G_PLAY, G_PAUSE, G_DYING, G_WIN, G_OVER, G_WARP, G_END };

LGFX_Sprite gfx(&lcd);
bool gameMode = false;

char    gMapBuf[G_MAPSZ];
char    gMapSave[G_MAPSZ];
int     gW = 176, gH = 12;
uint8_t gWorld = 1, gMaxWorld = 1, gArea = 0;

uint8_t  gState = G_TITLE;
Body     gp;
bool     gBig = false;
uint8_t  gPower = PW_NONE;
uint8_t  gInv = 0, gDropT = 0;
int8_t   gFace = 1;
uint16_t gAnim = 0, gStateT = 0;
int      gLives = 3, gScore = 0, gDrops = 0, gTime = 300, gBest = 0;
uint8_t  gTimeTick = 0;
float    gCam = 0, gCamY = 0;
int      gSpawnX = 0, gSpawnY = 0, gCheckX = 0, gCheckY = 0, gGoalX = 0;
bool     gCheckOk = false;
bool     gIgnorePlat = false;
GEnemy   gEn[G_MAXEN];     int gEnN = 0;
GEnemy   gEnSave[G_MAXEN]; int gEnSaveN = 0;
GItem    gIt[G_MAXIT];
GPopup   gPop[G_MAXPOP];
GShot    gShot[2];
uint8_t  gShotCd = 0;
uint8_t  gCoyote = 0, gJumpBuf = 0;
uint8_t  gBtn = 0, gPrevBtn = 0, gCtrlDrawn = 0xFF;
bool     gTouchPrev = false;
int      gBumpC = -1, gBumpR = -1; uint8_t gBumpT = 0;
uint32_t gLastFrame = 0;
uint8_t  gWarpT = 0, gWarpPhase = 0;
bool     gBossOn = false, gBossDone = false;

Btn gResumeBtn = { 70, 92, 180, 30 };
Btn gQuitBtn   = { 70, 132, 180, 30 };
Btn gExitBtn   = { 226, 176, 88, 26 };
Btn gContBtn   = { 12, 146, 150, 26 };

const char* G_WNAME[4] = { "", "El indoor", "La mata", "Invernadero de noche" };

// ---------- Mapa ----------
char gAt(int c, int r) {
  if (c < 0 || c >= gW || r < 0 || r >= gH) return '.';
  return gMapBuf[r * gW + c];
}
void gSet(int c, int r, char ch) { if (c >= 0 && c < gW && r >= 0 && r < gH) gMapBuf[r * gW + c] = ch; }
void gClearMap(int w, int h) { gW = w; gH = h; memset(gMapBuf, '.', sizeof gMapBuf); gEnN = 0; }
bool gVertical() { return gWorld == 2 && gArea == 0; }
int  gSY(float y) { return G_HUD + (int)y - (int)gCamY; }

void gGround(int a, int b) { for (int c = a; c <= b; c++) { gSet(c, gH - 2, '#'); gSet(c, gH - 1, '#'); } }
void gRowFill(int a, int b, int r, char ch) { for (int c = a; c <= b; c++) gSet(c, r, ch); }
void gStack(int c, int h) { for (int i = 0; i < h; i++) gSet(c, gH - 3 - i, 'K'); }
void gPot(int c, int h) {
  for (int i = 0; i < h; i++) {
    int r = gH - 3 - i; bool top = (i == h - 1);
    gSet(c, r, top ? 'T' : 't'); gSet(c + 1, r, top ? 'Y' : 'y');
  }
}

// y = superficie donde se apoya (en píxeles)
void gAddEnemy(float x, float y, uint8_t type) {
  if (gEnN >= G_MAXEN) return;
  GEnemy& e = gEn[gEnN++];
  int w = 14, h = 12;
  if (type == EN_MOSQ) { w = 12; h = 10; }
  if (type == EN_BOSS) { w = 30; h = 24; }
  e.b = { x, y - h, 0, 0, (int16_t)w, (int16_t)h, false };
  e.baseX = x; e.baseY = y - h; e.t = 0; e.dir = -1; e.type = type;
  e.state = 0; e.timer = 0; e.hp = (type == EN_BOSS) ? 8 : 1;
  e.active = true; e.awake = false;
}
void gAddWalker(int c, int surfRow, uint8_t type) { gAddEnemy(c * G_T + 1, surfRow * G_T, type); }
void gAddMosq(int c, int row) { gAddEnemy(c * G_T, row * G_T + 10, EN_MOSQ); }

// Leyenda: '#' tierra  'B' ladrillo  '?' bloque gota  'G' bloque hoja  'F' bloque sol
// 'U' bloque usado  'K' cajón  'T','Y','t','y' maceta  'P','Q' maceta-portal
// 'R' tierra con raíces  'o' gota  'S' tallo  'L' hoja (plataforma)  'l' hojita de brote
// 'C' cogollo  'X' cogollo principal  'b','d','e' brote seco (sube / diagonal der / izq)
// 'g' brote regado  'w' regadera

// ---------- Mundo 1: El indoor ----------
void gBuildW1() {
  gArea = 0; gClearMap(176, 12);
  gGround(0, 38);
  gSet(14, 6, '?');
  gRowFill(19, 23, 6, 'B'); gSet(20, 6, '?'); gSet(21, 6, 'G'); gSet(22, 6, '?');
  gSet(21, 2, '?');
  gAddWalker(25, 10, EN_MITE);
  gPot(28, 2);
  gAddWalker(32, 10, EN_APHID);
  gPot(35, 3);
  gRowFill(39, 41, 6, 'o');
  gGround(42, 73);
  gPot(45, 4); gSet(45, 6, 'P'); gSet(46, 6, 'Q');       // maceta-portal al subsuelo
  gRowFill(50, 54, 6, 'B'); gSet(52, 6, '?'); gRowFill(50, 54, 5, 'o');
  gAddWalker(56, 10, EN_MITE); gAddWalker(58, 10, EN_MITE);
  gRowFill(60, 62, 7, 'K');
  gSet(61, 3, 'G');
  gSet(66, 6, '?'); gSet(68, 6, '?');
  gAddWalker(67, 10, EN_APHID); gAddWalker(70, 10, EN_MITE);
  gCheckX = 72 * G_T; gCheckY = 10 * G_T;
  gSet(74, 7, 'o'); gSet(75, 6, 'o'); gSet(76, 7, 'o');
  gGround(77, 86);
  for (int i = 0; i < 4; i++) gStack(83 + i, i + 1);
  gGround(89, 122);
  for (int i = 0; i < 4; i++) gStack(89 + i, 4 - i);
  gAddWalker(97, 10, EN_MITE); gAddWalker(99, 10, EN_APHID); gAddWalker(101, 10, EN_MITE);
  gRowFill(103, 109, 6, 'B'); gSet(104, 6, '?'); gSet(106, 6, 'F'); gRowFill(104, 108, 2, 'o');
  gPot(113, 2); gPot(118, 3);
  gRowFill(124, 125, 7, 'K');
  gGround(127, 175);
  gAddWalker(131, 10, EN_MITE); gAddWalker(133, 10, EN_APHID); gAddWalker(136, 10, EN_MITE);
  gAddMosq(140, 3);
  gRowFill(138, 142, 6, 'B'); gSet(140, 6, '?'); gRowFill(138, 142, 5, 'o');
  for (int i = 0; i < 8; i++) gStack(146 + i, i + 1);
  gSpawnX = 3 * G_T; gSpawnY = 10 * G_T;
  gGoalX = 162 * G_T + 6;
}

// Subsuelo del mundo 1: cueva de raíces llena de gotas
void gBuildUnder() {
  gArea = 1; gClearMap(G_UCOLS, 12);
  for (int c = 0; c < G_UCOLS; c++) { gSet(c, 10, 'R'); gSet(c, 11, 'R'); if (c < 1 || c > 4) gSet(c, 0, 'R'); }
  for (int r = 0; r < 12; r++) { gSet(0, r, 'R'); gSet(G_UCOLS - 1, r, 'R'); }
  gRowFill(33, 42, 1, 'R');
  for (int r = 8; r <= 9; r++) gRowFill(5, 12, r, 'o');
  gRowFill(14, 19, 6, 'R'); gRowFill(14, 19, 4, 'o'); gRowFill(14, 19, 5, 'o');
  gSet(22, 6, '?'); gSet(23, 6, 'F'); gSet(24, 6, '?');
  gRowFill(27, 31, 7, 'R'); gRowFill(27, 31, 5, 'o');
  for (int r = 8; r <= 9; r++) gRowFill(33, 36, r, 'o');
  gAddWalker(17, 10, EN_MITE); gAddWalker(29, 10, EN_APHID);
  gPot(39, 2); gSet(39, 8, 'P'); gSet(40, 8, 'Q');
}

// ---------- Mundo 2: La mata (vertical) ----------
void gBuildW2() {
  gArea = 0; gClearMap(20, 110);
  for (int c = 0; c < 20; c++) { gSet(c, 108, '#'); gSet(c, 109, '#'); }
  for (int r = 9; r <= 107; r++) { gSet(9, r, 'S'); gSet(10, r, 'S'); }
  for (int i = 0; i < 24; i++) {
    int r = 104 - i * 4;
    if (i == 10 || i == 11) continue;                       // hueco: hay que regar el brote
    bool left = (i % 2 == 0);
    if (left) gRowFill(2, 9, r, 'L'); else gRowFill(10, 17, r, 'L');
    if (i % 3 == 2) gSet(left ? 2 : 17, r - 1, 'C');        // cogollo en la punta de la hoja
    if (i % 4 == 1) gRowFill(left ? 4 : 12, left ? 7 : 15, r - 2, 'o');
  }
  gSet(5, 93, 'F');                                         // bloque con sol
  gSet(3, 67, 'w');                                         // regadera
  gSet(6, 67, 'd');                                         // brote seco (crece hacia la derecha)
  gCheckX = 13 * G_T; gCheckY = 56 * G_T;
  gRowFill(4, 15, 8, 'L');                                  // repisa de la cima
  for (int r = 2; r <= 7; r++)
    for (int c = 6; c <= 13; c++) if (abs(c * 2 - 19) <= r + 1) gSet(c, r, 'X');
  const uint8_t mites[] = { 1, 4, 6, 9, 13, 15, 18, 21 };
  for (uint8_t i : mites) { int r = 104 - i * 4; gAddWalker((i % 2 == 0) ? 6 : 13, r, EN_MITE); }
  const uint8_t aph[] = { 7, 14, 19, 22 };
  for (uint8_t i : aph) { int r = 104 - i * 4; gAddWalker((i % 2 == 0) ? 4 : 15, r, EN_APHID); }
  gAddMosq(14, 78); gAddMosq(4, 50); gAddMosq(14, 36); gAddMosq(5, 20);
  gSpawnX = 3 * G_T; gSpawnY = 108 * G_T;
  gGoalX = 0;
}

// ---------- Mundo 3: Invernadero de noche ----------
void gBuildW3() {
  gArea = 0; gClearMap(176, 12);
  gGround(0, 60);
  gSet(10, 6, '?');
  gRowFill(14, 18, 6, 'B'); gSet(15, 6, '?'); gSet(16, 6, 'F');
  gAddWalker(13, 10, EN_APHID); gAddWalker(20, 10, EN_APHID); gAddMosq(24, 5);
  gSet(28, 9, 'w');                                          // regadera
  gSet(31, 9, 'b');                                          // brote seco: crece hacia arriba
  for (int r = 3; r <= 9; r++) { gSet(34, r, 'K'); gSet(35, r, 'K'); }   // muro de cajones
  gRowFill(34, 35, 2, 'o');
  gAddWalker(46, 10, EN_MITE); gAddWalker(50, 10, EN_MITE); gAddWalker(55, 10, EN_APHID);
  gAddMosq(40, 4); gAddMosq(52, 5);
  gRowFill(42, 46, 6, 'B'); gSet(44, 6, 'G'); gRowFill(42, 46, 5, 'o');
  gGround(64, 96);
  gCheckX = 66 * G_T; gCheckY = 10 * G_T;
  gAddMosq(70, 3); gAddMosq(76, 5); gAddMosq(82, 4);
  gAddWalker(72, 10, EN_APHID); gAddWalker(80, 10, EN_APHID);
  gAddWalker(88, 10, EN_MITE); gAddWalker(90, 10, EN_MITE);
  gRowFill(74, 78, 6, 'B'); gSet(76, 6, '?');
  gSet(83, 9, 'w'); gSet(85, 9, 'b'); gRowFill(86, 90, 2, 'o');   // brote opcional (bonus)
  gPot(92, 2);
  gRowFill(98, 99, 7, 'K');
  gGround(101, 175);
  for (int i = 0; i < 3; i++) gStack(104 + i, i + 1);
  for (int i = 0; i < 3; i++) gStack(107 + i, 3 - i);
  gAddWalker(111, 10, EN_MITE); gAddWalker(113, 10, EN_MITE); gAddMosq(112, 4);
  gRowFill(115, 119, 6, 'B'); gSet(117, 6, 'F'); gRowFill(115, 119, 5, 'o');
  gAddWalker(121, 10, EN_APHID); gAddWalker(124, 10, EN_APHID);
  gPot(127, 3); gPot(133, 2);
  gAddMosq(130, 3); gAddWalker(137, 10, EN_MITE);
  // Arena de la Reina Arana
  for (int r = 0; r <= 9; r++) gSet(163, r, 'K');
  gRowFill(148, 149, 6, 'K'); gRowFill(157, 158, 6, 'K');
  gAddWalker(156, 10, EN_BOSS);
  gSpawnX = 3 * G_T; gSpawnY = 10 * G_T;
  gGoalX = 170 * G_T;
}

void gBuildWorld() {
  if (gWorld == 2) gBuildW2();
  else if (gWorld == 3) gBuildW3();
  else gBuildW1();
}

// ---------- Física ----------
int  gFloorDiv(float v) { return (int)floorf(v / G_T); }
bool gSolidCh(char c) {
  return c == '#' || c == 'B' || c == '?' || c == 'G' || c == 'F' || c == 'U' || c == 'K' ||
         c == 'T' || c == 'Y' || c == 't' || c == 'y' || c == 'P' || c == 'Q' || c == 'R' || c == 'C';
}
bool gPlatCh(char c) { return c == 'L' || c == 'l'; }      // plataformas: se atraviesan desde abajo
bool gSolidAt(int tx, int ty) {
  if (tx < 0 || tx >= gW) return true;
  if (ty < 0 || ty >= gH) return false;
  return gSolidCh(gMapBuf[ty * gW + tx]);
}
bool gOverlap(const Body& a, const Body& b) {
  return a.x < b.x + b.w && a.x + a.w > b.x && a.y < b.y + b.h && a.y + a.h > b.y;
}

void gMoveX(Body& b, bool& hit) {
  hit = false;
  b.x += b.vx;
  int top = gFloorDiv(b.y), bot = gFloorDiv(b.y + b.h - 1);
  if (b.vx > 0) {
    int tx = gFloorDiv(b.x + b.w - 1);
    for (int ty = top; ty <= bot; ty++) if (gSolidAt(tx, ty)) { b.x = tx * G_T - b.w; b.vx = 0; hit = true; break; }
  } else if (b.vx < 0) {
    int tx = gFloorDiv(b.x);
    for (int ty = top; ty <= bot; ty++) if (gSolidAt(tx, ty)) { b.x = (tx + 1) * G_T; b.vx = 0; hit = true; break; }
  }
}

void gMoveY(Body& b, int& bumpC, int& bumpR) {
  bumpC = -1; bumpR = -1;
  float oldBot = b.y + b.h;
  b.vy += G_GRAV; if (b.vy > G_MAXFALL) b.vy = G_MAXFALL;
  b.y += b.vy;
  b.ground = false;
  int l = gFloorDiv(b.x), r = gFloorDiv(b.x + b.w - 1);
  if (b.vy > 0) {
    int ty = gFloorDiv(b.y + b.h - 1);
    for (int tx = l; tx <= r; tx++) {
      bool solid = gSolidAt(tx, ty) || (!gIgnorePlat && gPlatCh(gAt(tx, ty)) && oldBot <= ty * G_T + 0.5f);
      if (solid) { b.y = ty * G_T - b.h; b.vy = 0; b.ground = true; break; }
    }
  } else if (b.vy < 0) {
    int ty = gFloorDiv(b.y);
    for (int tx = l; tx <= r; tx++) if (gSolidAt(tx, ty)) {
      b.y = (ty + 1) * G_T; b.vy = 0;
      int cx = gFloorDiv(b.x + b.w / 2);
      bumpC = gSolidAt(cx, ty) ? cx : tx; bumpR = ty;
      break;
    }
  }
}

bool gGroundAhead(const Body& b, int dir) {
  int tx = gFloorDiv(dir > 0 ? b.x + b.w + 1 : b.x - 2);
  int ty = gFloorDiv(b.y + b.h + 2);
  return gSolidAt(tx, ty) || gPlatCh(gAt(tx, ty));
}

// ---------- Eventos ----------
void gPopup(float x, float y, int val, uint8_t kind, float vx, float vy) {
  for (auto& p : gPop) if (!p.active) { p = { x, y, vx, vy, (int16_t)val, kind, 30, true }; return; }
}

void gSaveBest() { if (gScore > gBest) { gBest = gScore; prefs.putInt("gbest", gBest); } }

void gAddDrop(float x, float y) {
  gDrops++; gScore += 10;
  if (gDrops >= 30) { gDrops -= 30; gLives++; gPopup(x, y - 10, -1, 0, 0, -1); }
}

void gFlipEnemy(GEnemy& e, float dirx) {
  e.state = 2; e.b.vy = -4.5f; e.b.vx = dirx;
  gScore += 100; gPopup(e.b.x, e.b.y - 4, 100, 0, 0, -1);
}

void gBossHit(GEnemy& e, uint8_t inv) {
  e.timer = inv; gScore += 200;
  if (e.hp > 0) e.hp--;
  if (e.hp == 0) {
    e.state = 2; e.b.vy = -6; e.b.vx = 0;
    gScore += 5000; gPopup(e.b.x, e.b.y - 6, 5000, 0, 0, -1);
    gBossDone = true;
    for (int r = 0; r <= 9; r++) gSet(163, r, '.');           // se abre el camino a la luz
  }
}

void gBump(int c, int r) {
  char ch = gAt(c, r);
  float bx = c * G_T, by = r * G_T;
  if (ch == '?') { gSet(c, r, 'U'); gAddDrop(bx, by); gPopup(bx + 8, by - 6, 0, 1, 0, -3); }
  else if (ch == 'G' || ch == 'F') {
    gSet(c, r, 'U');
    for (auto& it : gIt) if (!it.active) {
      it.b = { bx + 2, by, 0, 0, 12, 12, false }; it.dir = 1; it.state = 0; it.timer = 0;
      it.kind = (ch == 'F') ? 1 : 0; it.active = true; break;
    }
  } else if (ch == 'B' && gBig) {
    gSet(c, r, '.'); gScore += 50;
    gPopup(bx + 3, by + 3, 0, 2, -1.5f, -5); gPopup(bx + 11, by + 3, 0, 2, 1.5f, -5);
    gPopup(bx + 3, by + 11, 0, 2, -1.0f, -3); gPopup(bx + 11, by + 11, 0, 2, 1.0f, -3);
    return;
  }
  if (gSolidCh(gAt(c, r))) { gBumpC = c; gBumpR = r; gBumpT = 8; }
  for (int i = 0; i < gEnN; i++) {
    GEnemy& e = gEn[i];
    if (!e.active || e.state != 0 || e.type == EN_BOSS || e.type == EN_MOSQ) continue;
    if (fabsf(e.b.y + e.b.h - by) <= 3 && e.b.x + e.b.w > bx && e.b.x < bx + G_T)
      gFlipEnemy(e, (e.b.x < bx + 8) ? -1 : 1);
  }
  if (gAt(c, r - 1) == 'o') { gSet(c, r - 1, '.'); gAddDrop(bx, by - G_T); gPopup(bx + 8, by - 20, 0, 1, 0, -3); }
}

// Riego: el brote crece y forma una escalera de hojitas
void gGrow(int c, int r, char kind) {
  int dx = (kind == 'd') ? 1 : (kind == 'e' ? -1 : 0);
  gSet(c, r, 'g');
  for (int k = 1; k <= 4; k++) {
    int cc = c + dx * (k - 1), rr = r - 2 * k;
    char ch = gAt(cc, rr);
    if (ch == '.' || ch == 'S' || ch == 'o') gSet(cc, rr, 'l');
  }
  gScore += 200; gPopup(c * G_T, r * G_T - 10, 200, 0, 0, -1);
  for (int k = 0; k < 5; k++) gPopup(c * G_T + 8, r * G_T + 4, 0, 5, (k - 2) * 1.2f, -3.5f);
}

void gDie() { gState = G_DYING; gStateT = 0; gp.vy = -7; gp.vx = 0; gBig = false; gPower = PW_NONE; }

void gHurt() {
  if (gInv) return;
  if (gPower) { gPower = PW_NONE; gInv = 60; }
  else if (gBig) { gBig = false; gp.y += 8; gp.h = 14; gInv = 60; }
  else gDie();
}

// ---------- Disparos (sol / agua) ----------
void gShoot() {
  if (!gPower || gShotCd) return;
  for (auto& s : gShot) if (!s.active) {
    s.kind = gPower;
    s.x = gFace > 0 ? gp.x + gp.w : gp.x - 6;
    s.y = gp.y + 6;
    if (s.kind == PW_SUN) { s.vx = 4.5f * gFace; s.vy = 1.0f; s.life = 90; }
    else                  { s.vx = 4.2f * gFace; s.vy = -1.4f; s.life = 55; }
    s.active = true; gShotCd = 10;
    return;
  }
}

void gUpdateShots() {
  if (gShotCd) gShotCd--;
  for (auto& s : gShot) {
    if (!s.active) continue;
    s.x += s.vx;
    int cx = gFloorDiv(s.x + 3);
    if (gSolidAt(cx, gFloorDiv(s.y + 3))) { s.active = false; gPopup(s.x, s.y, 0, s.kind == PW_WATER ? 4 : 3, 0, 0); continue; }
    if (s.kind == PW_SUN) {
      s.vy += 0.45f; s.y += s.vy;
      int by = gFloorDiv(s.y + 6);
      if (s.vy > 0 && (gSolidAt(cx, by) || gPlatCh(gAt(cx, by)))) { s.y = by * G_T - 6; s.vy = -3.6f; }
    } else {
      s.vy += 0.15f; s.y += s.vy;
      int cy = gFloorDiv(s.y + 3);
      char ch = gAt(cx, cy);
      if (ch == 'b' || ch == 'd' || ch == 'e') { gGrow(cx, cy, ch); gPopup(s.x, s.y, 0, 4, 0, 0); s.active = false; continue; }
    }
    if (--s.life == 0 || s.x < gCam - 16 || s.x > gCam + G_VW + 16 || s.y > gH * G_T ||
        s.y < gCamY - 40 || s.y > gCamY + G_VROWS * G_T + 40) { s.active = false; continue; }
    for (int i = 0; i < gEnN; i++) {
      GEnemy& e = gEn[i];
      if (!e.active || !e.awake || e.state != 0) continue;
      if (s.x < e.b.x + e.b.w && s.x + 6 > e.b.x && s.y < e.b.y + e.b.h && s.y + 6 > e.b.y) {
        if (e.type == EN_BOSS) { if (!e.timer) gBossHit(e, 20); }
        else gFlipEnemy(e, s.vx > 0 ? 1 : -1);
        gPopup(s.x, s.y, 0, s.kind == PW_WATER ? 4 : 3, 0, 0);
        s.active = false; break;
      }
    }
  }
}

// ---------- Macetas-portal (mundo 1) ----------
int gPotUnderPlayer() {
  if (!gp.ground) return -1;
  int row = gFloorDiv(gp.y + gp.h);
  int cx  = gFloorDiv(gp.x + gp.w / 2);
  char ch = gAt(cx, row);
  if (ch == 'P') return cx;
  if (ch == 'Q') return cx - 1;
  return -1;
}

void gStartWarp(int potCol) {
  gp.x = potCol * G_T + G_T - gp.w / 2;
  gp.vx = 0; gp.vy = 0;
  gState = G_WARP; gWarpPhase = 0; gWarpT = 0;
  for (auto& s : gShot) s.active = false;
}

void gUpdateWarp() {
  gWarpT++;
  if (gWarpPhase == 0) {
    gp.y += 1;
    if (gWarpT >= gp.h + 4) {
      for (auto& it : gIt) it.active = false;
      for (auto& p : gPop) p.active = false;
      if (gArea == 0) {                                     // superficie -> subsuelo
        memcpy(gMapSave, gMapBuf, sizeof gMapBuf);
        memcpy(gEnSave, gEn, sizeof gEn); gEnSaveN = gEnN;
        gBuildUnder();
        gp.x = 2 * G_T; gp.y = 0; gp.vy = 0;
        gCam = 0; gCamY = 0; gFace = 1;
        gState = G_PLAY;
      } else {                                              // subsuelo -> superficie
        gArea = 0; gW = 176; gH = 12;
        memcpy(gMapBuf, gMapSave, sizeof gMapBuf);
        memcpy(gEn, gEnSave, sizeof gEn); gEnN = gEnSaveN;
        gSet(45, 6, 'T'); gSet(46, 6, 'Y');                 // el portal se cierra
        gp.x = G_EXIT_COL * G_T + G_T - gp.w / 2;
        gp.y = 7 * G_T;
        gCam = constrain(gp.x - 130, 0.0f, (float)(gW * G_T - G_VW));
        gCheckOk = true;
        gWarpPhase = 1; gWarpT = 0;
      }
    }
  } else {
    gp.y -= 1;
    if (gp.y + gp.h <= 7 * G_T) { gp.y = 7 * G_T - gp.h; gp.vy = 0; gState = G_PLAY; }
  }
}

// ---------- Inicio de vida / mundo ----------
void gResetPlayer() {
  int sx = gCheckOk ? gCheckX : gSpawnX;
  int sy = gCheckOk ? gCheckY : gSpawnY;
  gp = { (float)sx, (float)(sy - 14), 0, 0, 12, 14, false };
  gBig = false; gPower = PW_NONE; gInv = 0; gFace = 1; gShotCd = 0; gDropT = 0;
  for (auto& s : gShot) s.active = false;
  for (auto& it : gIt) it.active = false;
  for (auto& p : gPop) p.active = false;
  gTime = (gWorld == 1) ? 300 : 400; gTimeTick = 0; gBumpT = 0;
  gJumpBuf = gCoyote = 0;
  gBossOn = false; gBossDone = false;
  if (gVertical()) { gCam = 0; gCamY = constrain(gp.y - 110, 0.0f, (float)(gH * G_T - G_VROWS * G_T)); }
  else { gCamY = 0; gCam = max(0.0f, sx - 130.0f); }
}
void gNewLife() { gBuildWorld(); gResetPlayer(); gState = G_INTRO; gStateT = 0; }
void gStartWorld(uint8_t w) {
  gWorld = w; gCheckOk = false;
  if (w > gMaxWorld) { gMaxWorld = w; prefs.putUChar("gworld", w); }
  gNewLife();
}
void gNewGame(uint8_t w) { gLives = 3; gScore = 0; gDrops = 0; gStartWorld(w); }
void gToTitle() { gWorld = 1; gBuildWorld(); gCam = 0; gCamY = 0; gState = G_TITLE; gStateT = 0; }

// ---------- Enemigos ----------
void gUpdateBoss(GEnemy& e) {
  if (e.timer) e.timer--;
  float px = gp.x + gp.w / 2, ex = e.b.x + e.b.w / 2;
  e.dir = (px < ex) ? -1 : 1;
  if (e.b.ground) {
    e.b.vx = e.dir * (e.hp <= 3 ? 1.3f : 0.9f);
    if (e.t % 100 == 0) { e.b.vy = -8.5f; e.b.vx = e.dir * 2.0f; }
  }
  bool hit; gMoveX(e.b, hit);
  int bc, br; gMoveY(e.b, bc, br);
  if (gState == G_PLAY && gOverlap(gp, e.b)) {
    if (gp.vy > 0 && gp.y + gp.h - e.b.y < 12) {
      gp.vy = (gBtn & 4) ? -9.0f : -7.5f;
      if (!e.timer) gBossHit(e, 40);
    } else if (!e.timer) gHurt();
  }
}

void gUpdateEnemies() {
  for (int i = 0; i < gEnN; i++) {
    GEnemy& e = gEn[i];
    if (!e.active) continue;
    if (!e.awake) {
      bool nearV = e.b.x < gCam + G_VW + 24 && e.b.x + e.b.w > gCam - 24 &&
                   e.b.y < gCamY + G_VROWS * G_T + 48 && e.b.y + e.b.h > gCamY - 48;
      if (e.type == EN_BOSS) nearV = gBossOn;
      if (nearV) e.awake = true; else continue;
    }
    if (!gVertical() && e.type != EN_BOSS && e.b.x + e.b.w < gCam - 48) { e.active = false; continue; }
    e.t++;
    if (e.state == 0) {
      if (e.type == EN_BOSS) { gUpdateBoss(e); continue; }
      if (e.type == EN_MOSQ) {
        e.b.x += e.dir * 0.8f;
        int ax = gFloorDiv(e.dir > 0 ? e.b.x + e.b.w : e.b.x);
        if (gSolidAt(ax, gFloorDiv(e.b.y + 5)) || fabsf(e.b.x - e.baseX) > 70) {
          e.dir = (e.b.x > e.baseX) ? -1 : 1;
          if (gSolidAt(ax, gFloorDiv(e.b.y + 5))) e.dir = -e.dir;
        }
        e.b.y = e.baseY + sinf(e.t * 0.07f) * 14;
      } else {
        float spd = (e.type == EN_MITE) ? 0.7f : 0.45f;
        if (e.b.ground && !gGroundAhead(e.b, e.dir)) e.dir = -e.dir;
        e.b.vx = e.dir * spd;
        bool hit; gMoveX(e.b, hit); if (hit) e.dir = -e.dir;
        if (e.type == EN_APHID && e.b.ground && (e.t % 110) == 0) e.b.vy = -4.5f;
        int bc, br; gMoveY(e.b, bc, br);
        if (e.b.y > gH * G_T) { e.active = false; continue; }
        for (int j = 0; j < gEnN; j++)
          if (j != i && gEn[j].active && gEn[j].state == 0 && gEn[j].type <= EN_APHID && gOverlap(e.b, gEn[j].b))
            e.dir = (e.b.x < gEn[j].b.x) ? -1 : 1;
      }
      if (gState == G_PLAY && gOverlap(gp, e.b)) {
        if (gp.vy > 0 && gp.y + gp.h - e.b.y < 10) {        // pisotón
          e.state = 1; e.timer = 20;
          gp.vy = (gBtn & 4) ? -8.5f : -5.5f;
          gScore += 100; gPopup(e.b.x, e.b.y - 4, 100, 0, 0, -1);
        } else gHurt();
      }
    } else if (e.state == 1) {
      if (--e.timer == 0) e.active = false;
    } else {
      e.b.vy += G_GRAV; e.b.x += e.b.vx; e.b.y += e.b.vy;
      if (e.b.y > gH * G_T + 32 || e.b.y > gCamY + G_VROWS * G_T + 64) e.active = false;
    }
  }
}

void gUpdateItems() {
  for (auto& it : gIt) {
    if (!it.active) continue;
    if (it.state == 0) { it.b.y -= 1; if (++it.timer >= 14) it.state = 1; continue; }
    if (it.kind == 0) {
      it.b.vx = it.dir * 1.2f;
      bool hit; gMoveX(it.b, hit); if (hit) it.dir = -it.dir;
      int bc, br; gMoveY(it.b, bc, br);
      if (it.b.y > gH * G_T) { it.active = false; continue; }
    }
    if (gOverlap(gp, it.b)) {
      it.active = false; gScore += 1000; gPopup(it.b.x, it.b.y - 6, 1000, 0, 0, -1);
      if (!gBig) { gBig = true; gp.y -= 8; gp.h = 22; }
      if (it.kind == 1) { gPower = PW_SUN; gPopup(gp.x - 4, gp.y - 14, -4, 0, 0, -1); }
    }
  }
}

void gUpdatePopups() {
  for (auto& p : gPop) {
    if (!p.active) continue;
    p.x += p.vx; p.y += p.vy;
    if (p.kind == 2 || p.kind == 5) p.vy += G_GRAV; else if (p.kind == 1) p.vy *= 0.9f;
    if (--p.timer == 0) p.active = false;
  }
}

// ---------- Jugador ----------
void gUpdatePlay() {
  bool L = gBtn & 1, R = gBtn & 2, J = gBtn & 4;
  bool Jp = J && !(gPrevBtn & 4);
  bool Dp = (gBtn & 8) && !(gPrevBtn & 8);
  bool Fp = (gBtn & 16) && !(gPrevBtn & 16);

  if (Dp) {
    int pc = gPotUnderPlayer();
    if (pc >= 0) { gStartWarp(pc); return; }
    if (gp.ground && gPlatCh(gAt(gFloorDiv(gp.x + gp.w / 2), gFloorDiv(gp.y + gp.h)))) { gDropT = 10; gp.y += 1; }   // bajar de la hoja
  }
  if (Fp) gShoot();

  if (L && !R)      { gp.vx -= G_ACC; gFace = -1; }
  else if (R && !L) { gp.vx += G_ACC; gFace = 1; }
  else gp.vx *= gp.ground ? 0.75f : 0.985f;
  if (fabsf(gp.vx) < 0.05f) gp.vx = 0;
  gp.vx = constrain(gp.vx, -G_MAXSPD, G_MAXSPD);

  if (Jp) gJumpBuf = 6; else if (gJumpBuf) gJumpBuf--;
  if (gp.ground) gCoyote = 6; else if (gCoyote) gCoyote--;
  if (gJumpBuf && gCoyote) { gp.vy = -G_JUMPV; gJumpBuf = 0; gCoyote = 0; }
  if (!J && gp.vy < -3.5f) gp.vy = -3.5f;
  if (gDropT) gDropT--;

  bool hit; gMoveX(gp, hit);
  if (!gVertical() && gp.x < gCam) { gp.x = gCam; if (gp.vx < 0) gp.vx = 0; }
  int bc, br;
  gIgnorePlat = gDropT > 0; gMoveY(gp, bc, br); gIgnorePlat = false;
  if (bc >= 0) gBump(bc, br);

  // Gotas y regadera
  for (int ty = gFloorDiv(gp.y); ty <= gFloorDiv(gp.y + gp.h - 1); ty++)
    for (int tx = gFloorDiv(gp.x); tx <= gFloorDiv(gp.x + gp.w - 1); tx++) {
      char ch = gAt(tx, ty);
      if (ch == 'o') { gSet(tx, ty, '.'); gAddDrop(tx * G_T, ty * G_T); }
      else if (ch == 'w' && gPower != PW_WATER) { gPower = PW_WATER; gPopup(gp.x - 4, gp.y - 14, -3, 0, 0, -1); }
    }

  if (gp.y > gH * G_T) { gDie(); gp.vy = 0; return; }

  if (gArea == 0) {
    if (!gCheckOk && gCheckX > 0) {
      bool pass = gVertical() ? (gp.ground && gp.y + gp.h <= gCheckY + 1) : (gp.x > gCheckX);
      if (pass) { gCheckOk = true; gPopup(gCheckX - 10, gCheckY - 4 * G_T, -2, 0, 0, -1); }
    }
    bool won = false;
    if (gWorld == 2) won = gp.ground && gp.y + gp.h <= 8 * G_T + 1;
    else if (gWorld == 1 || gBossDone) won = gp.x + gp.w >= gGoalX;
    if (won) { gState = G_WIN; gStateT = 0; return; }
    if (gWorld == 3 && !gBossOn && !gBossDone && gp.x > (G_ARENA_COL + 2) * G_T) {
      gBossOn = true;
      for (int r = 0; r <= 9; r++) gSet(G_ARENA_COL - 1, r, 'K');   // se cierra la arena
    }
  }

  if (++gTimeTick >= 15) { gTimeTick = 0; if (--gTime <= 0) { gDie(); return; } }
  if (gInv) gInv--;

  if (gVertical()) {
    gCam = 0;
    gCamY += (gp.y - 110 - gCamY) * 0.25f;
    gCamY = constrain(gCamY, 0.0f, (float)(gH * G_T - G_VROWS * G_T));
  } else {
    gCamY = 0;
    if (gBossOn && !gBossDone) gCam += (G_ARENA_COL * G_T - gCam) * 0.2f;
    else { float target = gp.x - 130; if (target > gCam) gCam = target; }
    gCam = constrain(gCam, 0.0f, (float)(gW * G_T - G_VW));
  }

  gUpdateEnemies();
  gUpdateItems();
  gUpdateShots();
}

void gUpdateDying() {
  gStateT++;
  if (gStateT > 20) { gp.vy += G_GRAV; gp.y += gp.vy; }
  if (gStateT > 90) {
    if (--gLives <= 0) { gState = G_OVER; gStateT = 0; gSaveBest(); }
    else gNewLife();
  }
}

void gUpdateWin() {
  gStateT++;
  if (gStateT < 60) {
    if (gWorld != 2) { gp.vx = 1.2f; gFace = 1; bool hit; gMoveX(gp, hit); }
    else if (gp.ground && gStateT % 20 == 1) gp.vy = -5;    // salta de alegría en la cima
    int bc, br; gMoveY(gp, bc, br);
  }
  if (gStateT == 60) { gScore += gTime * 10; gTime = 0; gSaveBest(); }
}

// ---------- Dibujo ----------
void gText(const char* s, int x, int y, uint16_t col, const lgfx::IFont* f, textdatum_t d) {
  gfx.setFont(f); gfx.setTextDatum(d); gfx.setTextColor(col); gfx.drawString(s, x, y);
}

void gDrawDrop(int cx, int cy) {
  gfx.fillCircle(cx, cy + 2, 3, G_DROP);
  gfx.fillTriangle(cx, cy - 4, cx - 3, cy + 1, cx + 3, cy + 1, G_DROP);
  gfx.drawPixel(cx - 1, cy + 1, G_WHITE);
}

// Protagonista: cogollo. power: 0 normal, 1 sol (dorado), 2 agua (celeste)
void gDrawBud(int cx, int by, int s, bool big, int8_t face, uint16_t anim, bool dead, uint8_t power) {
  const uint16_t cBody  = power == PW_SUN ? rgb(120, 190, 70) : rgb(86, 160, 66);
  const uint16_t cDark  = rgb(48, 112, 44);
  const uint16_t cCalyx = power == PW_SUN ? rgb(170, 220, 100) : rgb(130, 196, 92);
  const uint16_t cLeaf  = rgb(40, 120, 46);
  const uint16_t cPist  = power == PW_SUN ? rgb(255, 170, 40) : rgb(232, 128, 44);
  const uint16_t cTric  = power == PW_SUN ? rgb(255, 222, 90) : (power == PW_WATER ? rgb(140, 210, 255) : rgb(245, 250, 240));
  const uint16_t cFoot  = rgb(70, 50, 30);

  int H = (big ? 22 : 14) * s;
  int step = anim ? (anim / 3) % 2 : 0;
  gfx.fillEllipse(cx - 3 * s, by - s - (step ? s : 0), 2 * s + 1, s + 1, cFoot);
  gfx.fillEllipse(cx + 3 * s, by - s - ((anim && !step) ? s : 0), 2 * s + 1, s + 1, cFoot);

  int bottom = by - 2 * s, top = by - H;
  int ry = (big ? 7 : 5) * s;
  int cyL = bottom - ry;
  gfx.fillEllipse(cx - 7 * s, cyL + s, 3 * s, s + 1, cLeaf);
  gfx.fillEllipse(cx + 7 * s, cyL + s, 3 * s, s + 1, cLeaf);
  gfx.fillTriangle(cx - 5 * s, cyL, cx + 5 * s, cyL, cx, top, cBody);
  gfx.fillEllipse(cx, (cyL + top) / 2 + s, 4 * s, (cyL - top) / 2, cBody);
  gfx.fillEllipse(cx, cyL, 6 * s, ry, cBody);
  gfx.drawEllipse(cx, cyL, 6 * s, ry, cDark);
  gfx.fillEllipse(cx - 3 * s, cyL + 2 * s, 2 * s, s + 1, cCalyx);
  gfx.fillEllipse(cx + 3 * s, cyL - s, 2 * s, s + 1, cCalyx);
  gfx.fillEllipse(cx - s, (cyL + top) / 2, 2 * s, s + 1, cCalyx);
  if (big) gfx.fillEllipse(cx + 2 * s, (cyL + top) / 2 + 3 * s, 2 * s, s + 1, cCalyx);
  gfx.drawLine(cx - 5 * s, cyL - 2 * s, cx - 8 * s, cyL - 4 * s, cPist);
  gfx.drawLine(cx + 4 * s, cyL - 4 * s, cx + 7 * s, cyL - 7 * s, cPist);
  gfx.drawLine(cx, top, cx - 2 * s, top - 3 * s, cPist);
  gfx.drawLine(cx, top, cx + 2 * s, top - 3 * s, cPist);
  if (big) gfx.drawLine(cx - 3 * s, (cyL + top) / 2, cx - 6 * s, (cyL + top) / 2 - 3 * s, cPist);
  const int8_t tx[7] = { -4, 3, -1, 2, -3, 4, 0 };
  const int8_t ty[7] = { -1, 2, -4, -6, 3, -2, -8 };
  for (int k = 0; k < (big ? 7 : 5); k++) {
    int px = cx + tx[k] * s, py = cyL + ty[k] * s;
    if (py > top) gfx.fillCircle(px, py, s > 1 ? s / 2 + 1 : 0, cTric);
  }
  if (power && (gAnim / 4) % 2) gfx.drawPixel(cx + 5 * s, top + 2 * s, G_WHITE);
  int ey = cyL - (big ? 2 : 1) * s, ex = cx + face * 2 * s;
  if (dead) {
    for (int k = -1; k <= 1; k += 2) {
      int x0 = ex + k * 2 * s;
      gfx.drawLine(x0 - s, ey - s, x0 + s, ey + s, G_BLACK);
      gfx.drawLine(x0 - s, ey + s, x0 + s, ey - s, G_BLACK);
    }
  } else {
    gfx.fillEllipse(ex - 2 * s, ey, s + 1, 2 * s, G_WHITE);
    gfx.fillEllipse(ex + 2 * s, ey, s + 1, 2 * s, G_WHITE);
    int pr = max(1, s * 2 / 3);
    gfx.fillCircle(ex - 2 * s + face * (s / 2), ey + s / 3, pr, G_BLACK);
    gfx.fillCircle(ex + 2 * s + face * (s / 2), ey + s / 3, pr, G_BLACK);
    if (s > 1) gfx.drawArc(ex, ey + 3 * s, 2 * s, 2 * s - 1, 20, 160, cDark);
  }
}

// Arañita roja (s = escala; s > 1 es la Reina, con corona)
void gDrawMite(int x, int y, int w, int h, uint8_t state, uint16_t anim, int8_t dir, int s) {
  const uint16_t cR = rgb(214, 44, 32), cRd = rgb(150, 24, 18), cSpot = rgb(90, 12, 10);
  const uint16_t cLeg = rgb(120, 20, 16), cHi = rgb(255, 120, 100);
  int cx = x + w / 2, by = y + h;
  if (state == 1) {
    gfx.fillEllipse(cx, by - 2, 8 * s, 3, cRd);
    gfx.fillEllipse(cx, by - 3, 6 * s, 2, cR);
    return;
  }
  int sg = (state == 2) ? -1 : 1;
  int cy = by - 5 * s;
  int st = (anim / 3) % 2;
  for (int k = 0; k < 4; k++) {
    int ox = (-4 + k * 3) * s;
    int sw = (((k + st) % 2) ? 2 : -1) * s;
    gfx.drawLine(cx + ox, cy, cx + ox - 3 * s + sw, cy + sg * 5 * s, cLeg);
    gfx.drawLine(cx + ox, cy, cx + ox + 3 * s + sw, cy + sg * 5 * s, cLeg);
    if (s > 1) gfx.drawLine(cx + ox + 1, cy, cx + ox + 3 * s + sw + 1, cy + sg * 5 * s, cLeg);
  }
  gfx.fillEllipse(cx - dir * s, cy - sg * s, 6 * s, 4 * s, cR);
  gfx.fillEllipse(cx + dir * 5 * s, cy - sg * s, 3 * s, 3 * s, cRd);
  gfx.fillCircle(cx - dir * 2 * s, cy - sg * 2 * s, s, cSpot);
  gfx.fillCircle(cx + dir * s, cy, s, cSpot);
  gfx.fillCircle(cx - dir * 3 * s, cy - sg * 3 * s, s > 1 ? 2 : 0, cHi);
  gfx.fillCircle(cx + dir * 6 * s, cy - sg * 2 * s, s > 1 ? 2 : 0, G_WHITE);
  gfx.drawPixel(cx + dir * 6 * s, cy - sg * s, G_BLACK);
  gfx.drawLine(cx + dir * 7 * s, cy - sg * 2 * s, cx + dir * 9 * s, cy - sg * 4 * s, cLeg);
  if (s > 1 && state == 0) {                                // corona de la Reina
    int kx = cx + dir * 5 * s, ky = cy - 4 * s;
    uint16_t gold = rgb(255, 205, 50);
    gfx.fillRect(kx - 5, ky, 11, 3, gold);
    gfx.fillTriangle(kx - 5, ky, kx - 3, ky - 5, kx - 1, ky, gold);
    gfx.fillTriangle(kx - 2, ky, kx, ky - 7, kx + 2, ky, gold);
    gfx.fillTriangle(kx + 1, ky, kx + 3, ky - 5, kx + 5, ky, gold);
    gfx.drawPixel(kx, ky - 3, rgb(255, 60, 80));
  }
}

// Pulgón: verde, cuerpo de pera, antenas y cornículos
void gDrawAphid(int x, int y, int w, int h, uint8_t state, uint16_t anim, int8_t dir) {
  const uint16_t cB = rgb(150, 210, 70), cBd = rgb(96, 150, 40), cLeg = rgb(60, 90, 20);
  int cx = x + w / 2, by = y + h;
  if (state == 1) { gfx.fillEllipse(cx, by - 2, 8, 3, cBd); gfx.fillEllipse(cx, by - 3, 6, 2, cB); return; }
  int sg = (state == 2) ? -1 : 1;
  int cy = by - 6;
  int st = (anim / 4) % 2;
  for (int k = -1; k <= 1; k++) gfx.drawLine(cx + k * 4, cy, cx + k * 4 + (st ? 2 : -2), cy + sg * 6, cLeg);
  gfx.fillEllipse(cx - dir, cy - sg, 7, 5, cB);
  gfx.fillEllipse(cx + dir * 5, cy - sg * 2, 3, 3, cBd);
  gfx.drawLine(cx - dir * 5, cy - sg * 4, cx - dir * 8, cy - sg * 7, cBd);       // cornículos
  gfx.drawLine(cx - dir * 3, cy - sg * 5, cx - dir * 5, cy - sg * 8, cBd);
  gfx.drawLine(cx + dir * 6, cy - sg * 4, cx + dir * 10, cy - sg * 10, cLeg);    // antenas
  gfx.drawLine(cx + dir * 5, cy - sg * 4, cx + dir * 7, cy - sg * 10, cLeg);
  gfx.drawPixel(cx + dir * 7, cy - sg * 2, G_BLACK);
  gfx.fillEllipse(cx - dir * 2, cy - sg * 3, 2, 1, rgb(220, 250, 160));
}

// Mosquito: vuela ondulando
void gDrawMosq(int x, int y, int w, int h, uint8_t state, uint16_t anim, int8_t dir) {
  const uint16_t cBody = rgb(70, 70, 82), cStripe = rgb(160, 160, 170), cLeg = rgb(50, 50, 60);
  int cx = x + w / 2, cy = y + h / 2;
  if (state == 1) { gfx.fillEllipse(cx, y + h - 2, 7, 2, cBody); return; }
  int sg = (state == 2) ? -1 : 1;
  for (int k = 0; k < 3; k++) gfx.drawLine(cx - 2 + k * 2, cy + sg, cx - 5 + k * 3, cy + sg * 7, cLeg);
  gfx.fillEllipse(cx - dir * 4, cy + sg, 5, 2, cBody);                          // abdomen
  gfx.drawFastVLine(cx - dir * 3, cy - 1, 3, cStripe);
  gfx.drawFastVLine(cx - dir * 6, cy - 1, 3, cStripe);
  gfx.fillCircle(cx + dir * 1, cy, 2, cBody);                                    // tórax
  gfx.fillCircle(cx + dir * 4, cy - sg, 2, rgb(90, 60, 60));                     // cabeza
  gfx.drawPixel(cx + dir * 5, cy - sg * 2, rgb(255, 80, 80));
  gfx.drawLine(cx + dir * 6, cy, cx + dir * 10, cy + sg * 3, cLeg);             // probóscide
  uint16_t wing = rgb(200, 228, 255);
  if ((anim / 2) % 2) { gfx.fillEllipse(cx - dir, cy - sg * 5, 4, 2, wing); gfx.fillEllipse(cx - dir * 4, cy - sg * 4, 3, 2, wing); }
  else { gfx.fillEllipse(cx - dir * 3, cy - sg * 2, 5, 1, wing); }
}

void gDrawEnemy(const GEnemy& e, int cam) {
  int x = (int)e.b.x - cam, y = gSY(e.b.y);
  switch (e.type) {
    case EN_MITE:  gDrawMite(x, y, e.b.w, e.b.h, e.state, gAnim, e.dir, 1); break;
    case EN_APHID: gDrawAphid(x, y, e.b.w, e.b.h, e.state, gAnim, e.dir); break;
    case EN_MOSQ:  gDrawMosq(x, y, e.b.w, e.b.h, e.state, gAnim, e.dir); break;
    case EN_BOSS:
      if (e.timer && (e.timer / 3) % 2) break;            // parpadeo al recibir daño
      gDrawMite(x, y, e.b.w, e.b.h, e.state, gAnim, e.dir, 2);
      break;
  }
}

void gDrawSunItem(int x, int y) {
  int cx = x + 6, cy = y + 6;
  uint16_t ray = rgb(255, 200, 40);
  int rot = (gAnim / 3) % 2;
  for (int k = 0; k < 8; k++) {
    float a = (k * 45 + rot * 22) * 0.01745f;
    gfx.drawLine(cx + cosf(a) * 5, cy + sinf(a) * 5, cx + cosf(a) * 8, cy + sinf(a) * 8, ray);
  }
  gfx.fillCircle(cx, cy, 5, rgb(255, 214, 60));
  gfx.fillCircle(cx - 1, cy - 1, 2, rgb(255, 246, 180));
}

void gDrawLeafItem(int x, int y) {
  int cx = x + 6, cy = y + 6;
  gfx.fillEllipse(cx, cy, 7, 4, rgb(60, 190, 70));
  gfx.drawLine(cx - 6, cy + 1, cx + 6, cy - 1, rgb(30, 110, 40));
  gfx.fillEllipse(cx - 2, cy - 2, 2, 1, rgb(180, 245, 160));
  if ((gAnim / 4) % 2) { gfx.drawFastHLine(cx + 4, cy - 6, 5, G_WHITE); gfx.drawFastVLine(cx + 6, cy - 8, 5, G_WHITE); }
}

void gDrawShot(const GShot& s, int cam) {
  int x = (int)s.x - cam, y = gSY(s.y);
  if (s.kind == PW_SUN) {
    bool f = (gAnim / 2) % 2;
    gfx.fillCircle(x + 3, y + 3, 4, f ? rgb(255, 150, 30) : rgb(255, 190, 50));
    gfx.fillCircle(x + 3, y + 3, 2, rgb(255, 240, 150));
  } else {
    gfx.fillCircle(x + 3, y + 4, 3, rgb(70, 160, 245));
    gfx.fillTriangle(x + 3, y - 2, x, y + 3, x + 6, y + 3, rgb(70, 160, 245));
    gfx.drawPixel(x + 2, y + 3, G_WHITE);
  }
}

void gDrawCan(int x, int y, uint16_t col) {          // regadera
  gfx.fillRoundRect(x + 3, y + 6, 9, 8, 2, col);
  gfx.drawLine(x + 12, y + 9, x + 15, y + 5, col);
  gfx.drawLine(x + 12, y + 10, x + 15, y + 6, col);
  gfx.drawArc(x + 7, y + 7, 4, 3, 180, 360, col);
}

void gDrawTile(char ch, int x, int y, int c, int r) {
  const uint16_t potBody = rgb(186, 92, 52), potRim = rgb(210, 108, 62), potDark = rgb(130, 58, 34);
  const uint16_t potHi = rgb(226, 136, 92), potSh = rgb(150, 70, 40), soil = rgb(90, 55, 30);
  switch (ch) {
    case '#': {
      gfx.fillRect(x, y, G_T, G_T, rgb(128, 78, 42));
      uint8_t hsh = (c * 7 + r * 13) & 7;
      gfx.fillRect(x + 2 + hsh, y + 5 + (hsh & 3), 2, 2, rgb(96, 56, 28));
      gfx.fillRect(x + 10 - (hsh & 3), y + 11, 2, 2, rgb(96, 56, 28));
      if (!gSolidCh(gAt(c, r - 1)) && r > 0) {
        gfx.fillRect(x, y, G_T, 4, rgb(76, 176, 64));
        gfx.drawFastHLine(x, y + 4, G_T, rgb(50, 130, 45));
        for (int k = 1; k < G_T; k += 4) gfx.drawFastVLine(x + k + (hsh & 1), y - 2, 3, rgb(76, 176, 64));
      }
    } break;
    case 'B': {
      uint16_t m = rgb(130, 58, 34);
      gfx.fillRect(x, y, G_T, G_T, rgb(188, 96, 58));
      gfx.drawFastHLine(x, y, G_T, rgb(225, 140, 100));
      gfx.drawFastHLine(x, y + 7, G_T, m); gfx.drawFastHLine(x, y + 15, G_T, m);
      gfx.drawFastVLine(x + 7, y + 1, 6, m); gfx.drawFastVLine(x + 3, y + 8, 7, m); gfx.drawFastVLine(x + 12, y + 8, 7, m);
    } break;
    case '?': case 'G': case 'F': {
      gfx.fillRect(x, y, G_T, G_T, rgb(232, 178, 52));
      gfx.drawRect(x, y, G_T, G_T, rgb(160, 100, 20));
      gfx.drawFastHLine(x + 1, y + 1, 14, rgb(255, 226, 130));
      gDrawDrop(x + 8, y + 6 + ((gAnim / 10) % 2));
    } break;
    case 'U':
      gfx.fillRect(x, y, G_T, G_T, rgb(150, 112, 72));
      gfx.drawRect(x, y, G_T, G_T, rgb(100, 70, 40));
      break;
    case 'K': {
      uint16_t d = rgb(110, 72, 32);
      gfx.fillRect(x, y, G_T, G_T, rgb(176, 124, 64));
      gfx.drawRect(x, y, G_T, G_T, d); gfx.drawRect(x + 2, y + 2, 12, 12, d);
      gfx.drawLine(x + 2, y + 2, x + 13, y + 13, d); gfx.drawLine(x + 2, y + 13, x + 13, y + 2, d);
    } break;
    case 'T':
      gfx.fillRect(x, y + 7, G_T, 9, potBody); gfx.fillRect(x + 3, y + 7, 3, 9, potHi);
      gfx.fillRect(x - 2, y, 18, 7, potRim); gfx.drawFastHLine(x - 2, y + 6, 18, potDark);
      gfx.fillRect(x + 1, y + 1, 15, 2, soil);
      break;
    case 'Y':
      gfx.fillRect(x, y + 7, G_T, 9, potBody); gfx.fillRect(x + 11, y + 7, 5, 9, potSh);
      gfx.fillRect(x, y, 18, 7, potRim); gfx.drawFastHLine(x, y + 6, 18, potDark);
      gfx.fillRect(x, y + 1, 15, 2, soil);
      break;
    case 'P': {
      uint16_t rimP = rgb(90, 160, 90), darkP = rgb(40, 100, 50);
      gfx.fillRect(x, y + 7, G_T, 9, potBody); gfx.fillRect(x + 3, y + 7, 3, 9, potHi);
      gfx.fillRect(x - 2, y, 18, 7, rimP); gfx.drawFastHLine(x - 2, y + 6, 18, darkP);
      gfx.fillRect(x + 1, y + 1, 15, 3, rgb(20, 14, 10));
      if ((gAnim / 8) % 2) gfx.fillTriangle(x + 11, y - 10, x + 21, y - 10, x + 16, y - 4, rgb(255, 230, 90));
    } break;
    case 'Q': {
      uint16_t rimP = rgb(90, 160, 90), darkP = rgb(40, 100, 50);
      gfx.fillRect(x, y + 7, G_T, 9, potBody); gfx.fillRect(x + 11, y + 7, 5, 9, potSh);
      gfx.fillRect(x, y, 18, 7, rimP); gfx.drawFastHLine(x, y + 6, 18, darkP);
      gfx.fillRect(x, y + 1, 15, 3, rgb(20, 14, 10));
    } break;
    case 't': gfx.fillRect(x, y, G_T, G_T, potBody); gfx.fillRect(x + 3, y, 3, G_T, potHi); break;
    case 'y': gfx.fillRect(x, y, G_T, G_T, potBody); gfx.fillRect(x + 11, y, 5, G_T, potSh); break;
    case 'R': {
      uint8_t hsh = (c * 5 + r * 11) & 7;
      uint16_t root = rgb(150, 112, 70);
      gfx.fillRect(x, y, G_T, G_T, rgb(64, 42, 28));
      gfx.drawLine(x, y + 4 + (hsh & 3), x + 8, y + 7, root);
      gfx.drawLine(x + 8, y + 7, x + 15, y + 3 + (hsh & 3), root);
      gfx.drawLine(x + 5, y + 7, x + 3 + (hsh & 1) * 4, y + 14, root);
    } break;
    case 'o': gDrawDrop(x + 8, y + 7 + (((gAnim + c * 3) / 6) % 2)); break;
    case 'S': {                                                  // tallo
      uint16_t st = rgb(70, 140, 55), stD = rgb(50, 110, 40);
      bool leftHalf = gAt(c + 1, r) == 'S' || gAt(c + 1, r) == 'L';
      if (leftHalf) { gfx.fillRect(x + 9, y, 7, G_T, st); gfx.drawFastVLine(x + 9, y, G_T, stD); }
      else          { gfx.fillRect(x, y, 7, G_T, st); gfx.drawFastVLine(x + 6, y, G_T, stD); }
      if (r % 4 == 2) gfx.drawFastHLine(leftHalf ? x + 9 : x, y + 8, 7, stD);
    } break;
    case 'L': {                                                  // hoja (plataforma)
      uint16_t lf = rgb(60, 150, 60), lfD = rgb(36, 104, 40), lfH = rgb(120, 200, 100);
      bool lt = gAt(c - 1, r) != 'L', rt = gAt(c + 1, r) != 'L';
      int x0 = x + (lt ? 4 : 0), x1 = x + (rt ? 12 : 16);
      gfx.fillRect(x0, y + 1, x1 - x0, 7, lf);
      gfx.drawFastHLine(x0, y + 1, x1 - x0, lfH);
      gfx.drawFastHLine(x0, y + 4, x1 - x0, lfD);                // nervio
      for (int k = 2; k < 16; k += 5) gfx.fillTriangle(x + k, y + 8, x + k + 3, y + 8, x + k + 1, y + 11, lf);   // dientes
      if (lt) gfx.fillTriangle(x + 4, y + 1, x + 4, y + 8, x - 2, y + 4, lf);
      if (rt) gfx.fillTriangle(x + 12, y + 1, x + 12, y + 8, x + 18, y + 4, lf);
    } break;
    case 'l': {                                                  // hojita del brote regado
      gfx.drawLine(x + 8, y + 6, x + 8, y + 22, rgb(70, 150, 60));
      gfx.fillEllipse(x + 8, y + 4, 8, 3, rgb(80, 180, 70));
      gfx.drawFastHLine(x + 2, y + 4, 12, rgb(40, 120, 46));
    } break;
    case 'C': {                                                  // cogollo en la punta de la hoja
      gfx.fillTriangle(x + 2, y + 15, x + 14, y + 15, x + 8, y + 1, rgb(86, 160, 66));
      gfx.fillEllipse(x + 8, y + 11, 6, 5, rgb(86, 160, 66));
      gfx.drawLine(x + 4, y + 9, x + 1, y + 6, rgb(232, 128, 44));
      gfx.drawLine(x + 11, y + 7, x + 14, y + 4, rgb(232, 128, 44));
      gfx.drawPixel(x + 6, y + 10, G_WHITE); gfx.drawPixel(x + 10, y + 12, G_WHITE); gfx.drawPixel(x + 8, y + 6, G_WHITE);
    } break;
    case 'X': {                                                  // cogollo principal (decorado)
      uint8_t hsh = (c * 3 + r * 7) & 7;
      gfx.fillRect(x, y, G_T, G_T, rgb(92, 168, 70));
      gfx.fillEllipse(x + 4 + (hsh & 3), y + 5, 4, 3, rgb(130, 200, 92));
      gfx.fillEllipse(x + 11, y + 11 - (hsh & 1) * 2, 4, 3, rgb(130, 200, 92));
      gfx.drawLine(x + 2 + hsh, y + 2, x + hsh, y - 2, rgb(240, 140, 50));
      gfx.drawPixel(x + 9, y + 4, G_WHITE); gfx.drawPixel(x + 3, y + 12, G_WHITE); gfx.drawPixel(x + 13, y + 7, G_WHITE);
    } break;
    case 'b': case 'd': case 'e': {                              // brote seco (regar!)
      gfx.fillEllipse(x + 8, y + 15, 7, 3, rgb(110, 76, 44));
      gfx.drawLine(x + 8, y + 14, x + 8, y + 7, rgb(150, 130, 60));
      gfx.drawLine(x + 8, y + 8, x + 4, y + 11, rgb(190, 170, 70));
      gfx.drawLine(x + 8, y + 7, x + 12, y + 10, rgb(190, 170, 70));
      if ((gAnim / 12) % 2) gDrawDrop(x + 8, y - 6);
    } break;
    case 'g': {                                                  // brote regado
      gfx.fillEllipse(x + 8, y + 15, 7, 3, rgb(90, 60, 34));
      gfx.drawLine(x + 8, y + 14, x + 8, y, rgb(70, 150, 60));
      gfx.fillEllipse(x + 4, y + 8, 4, 2, rgb(80, 180, 70));
      gfx.fillEllipse(x + 12, y + 6, 4, 2, rgb(80, 180, 70));
    } break;
    case 'w': {                                                  // estación de regadera
      gfx.fillRect(x + 7, y - 4, 2, 6, rgb(120, 120, 130));
      gDrawCan(x, y + 1 - ((gAnim / 10) % 2), rgb(70, 160, 245));
      if ((gAnim / 6) % 3 == 0) gfx.drawPixel(x + 14, y + 2, G_WHITE);
    } break;
  }
}

void gDrawBackground() {
  if (gArea == 1) {                                         // subsuelo
    gfx.fillRect(0, G_HUD, G_VW, G_VROWS * G_T, rgb(26, 18, 14));
    for (int i = 0; i < 9; i++) {
      int rx = ((i * 83 - (int)(gCam * 0.5f)) % 760 + 760) % 760 - 20;
      int len = 40 + (i * 29) % 70;
      for (int k = 0; k < len; k += 6)
        gfx.drawLine(rx + (int)(sinf((k + i * 13) * 0.15f) * 3), G_HUD + 16 + k,
                     rx + (int)(sinf((k + 6 + i * 13) * 0.15f) * 3), G_HUD + 22 + k, rgb(58, 40, 28));
    }
    return;
  }
  if (gWorld == 2) {                                        // carpa de cultivo (mylar)
    const uint16_t band[4] = { rgb(150, 154, 162), rgb(128, 132, 140), rgb(112, 116, 124), rgb(96, 100, 108) };
    for (int i = 0; i < 4; i++) gfx.fillRect(0, G_HUD + i * 48, G_VW, 48, band[i]);
    int off = (int)(gCamY * 0.4f);
    for (int k = 0; k < 9; k++) {
      int lx = k * 40 + 6;
      for (int y = 0; y < 192; y += 12) {
        int yy = G_HUD + ((y - off) % 192 + 192) % 192;
        gfx.drawLine(lx + ((y / 12 + k) % 3) * 2, yy, lx + 3, yy + 10, rgb(185, 188, 196));
      }
    }
    int ly = G_HUD - (int)gCamY + 4;                         // lámpara LED arriba
    if (ly > -40) {
      for (int k = 0; k < 6; k++) gfx.drawLine(60 + k * 40, ly + 10, 30 + k * 52, ly + 70, rgb(230, 200, 240));
      gfx.fillRect(40, ly, 240, 10, rgb(36, 36, 44));
      for (int k = 0; k < 20; k++) gfx.fillRect(46 + k * 12, ly + 7, 6, 3, (k % 3) ? rgb(255, 240, 230) : rgb(255, 120, 200));
    }
    return;
  }
  if (gWorld == 3) {                                        // invernadero de noche
    const uint16_t sky[4] = { rgb(10, 14, 40), rgb(16, 22, 54), rgb(24, 30, 66), rgb(34, 40, 78) };
    for (int i = 0; i < 4; i++) gfx.fillRect(0, G_HUD + i * 48, G_VW, 48, sky[i]);
    for (int i = 0; i < 28; i++) {
      int sx = ((i * 97 - (int)(gCam * 0.1f)) % 340 + 340) % 340 - 10;
      int sy = G_HUD + 6 + (i * 53) % 110;
      if ((gAnim / 8 + i) % 5) gfx.drawPixel(sx, sy, rgb(230, 230, 255));
    }
    gfx.fillCircle(262, G_HUD + 30, 12, rgb(240, 240, 210));
    gfx.fillCircle(268, G_HUD + 26, 11, sky[0]);
    int gx = -(int)(gCam * 0.5f) % 80;
    for (int k = 0; k < 6; k++) gfx.drawFastVLine(gx + k * 80, G_HUD + 40, 152, rgb(60, 80, 110));
    gfx.drawFastHLine(0, G_HUD + 40, G_VW, rgb(60, 80, 110));
    for (int i = 0; i < 6; i++) {
      int hx = ((i * 190 - (int)(gCam * 0.6f)) % 1140 + 1140) % 1140 - 80;
      gfx.fillEllipse(hx, G_HUD + 165, 60, 30, rgb(20, 40, 32));
    }
    return;
  }
  // Mundo 1: cielo, nubes y colinas
  const uint16_t sky[4] = { rgb(110, 190, 240), rgb(135, 205, 245), rgb(165, 220, 248), rgb(195, 232, 250) };
  for (int i = 0; i < 4; i++) gfx.fillRect(0, G_HUD + i * 48, G_VW, 48, sky[i]);
  for (int i = 0; i < 6; i++) {
    int cx = ((i * 170 + 40 - (int)(gCam * 0.3f)) % 1020 + 1020) % 1020 - 60;
    int cy = G_HUD + 22 + (i * 37) % 44;
    gfx.fillEllipse(cx, cy, 22, 8, G_WHITE);
    gfx.fillEllipse(cx + 12, cy - 6, 13, 8, G_WHITE);
  }
  int gy = G_HUD + 10 * G_T;
  for (int i = 0; i < 5; i++) {
    int hx = ((i * 230 - (int)(gCam * 0.5f)) % 1150 + 1150) % 1150 - 100;
    gfx.fillEllipse(hx, gy + 6, 90, 46 + (i % 2) * 16, rgb(120, 190, 110));
  }
}

void gDrawCheckpoint(int x, int gy, bool ok) {
  if (x < -20 || x > G_VW + 20 || gy < 0 || gy > G_VH + 40) return;
  gfx.fillRect(x, gy - 40, 2, 40, rgb(90, 90, 100));
  gfx.fillTriangle(x + 2, gy - 40, x + 2, gy - 28, x + 15, gy - 34, ok ? rgb(80, 200, 80) : rgb(150, 150, 150));
}

void gDrawGoal(int x, int gy) {
  if (x < -60 || x > G_VW + 60) return;
  int top = gy - 150;
  uint16_t ray = ((gAnim / 6) % 2) ? rgb(255, 240, 160) : rgb(255, 222, 110);
  for (int k = -2; k <= 2; k++) gfx.drawLine(x - 20, top + 16, x - 20 + k * 12, top + 46, ray);
  gfx.fillRect(x - 1, top, 3, gy - top, rgb(120, 120, 130));
  gfx.fillRect(x - 20, top, 22, 3, rgb(120, 120, 130));
  gfx.fillTriangle(x - 34, top + 12, x - 6, top + 12, x - 20, top + 2, rgb(60, 60, 70));
  gfx.fillRect(x - 32, top + 12, 24, 3, rgb(255, 230, 120));
}

void gDrawHUD() {
  char b[20];
  gfx.fillRect(0, 0, G_VW, G_HUD, rgb(24, 32, 40));
  gfx.fillTriangle(4, 11, 12, 11, 8, 2, rgb(86, 160, 66)); gfx.fillCircle(8, 10, 4, rgb(86, 160, 66));
  gfx.drawPixel(6, 6, rgb(232, 128, 44)); gfx.drawPixel(10, 8, rgb(232, 128, 44));
  snprintf(b, sizeof b, "x%d", gLives);        gText(b, 15, 8, G_WHITE, &fonts::Font0, textdatum_t::middle_left);
  if (gPower == PW_SUN) gfx.fillCircle(36, 8, 3, rgb(255, 214, 60));
  if (gPower == PW_WATER) gDrawDrop(36, 7);
  gDrawDrop(50, 8);
  snprintf(b, sizeof b, "x%02d", gDrops);      gText(b, 56, 8, G_WHITE, &fonts::Font0, textdatum_t::middle_left);
  snprintf(b, sizeof b, "PTS %06d", gScore);   gText(b, 90, 8, G_WHITE, &fonts::Font0, textdatum_t::middle_left);
  snprintf(b, sizeof b, "T %03d", gTime);      gText(b, 176, 8, gTime < 60 ? C_HIGH : G_WHITE, &fonts::Font0, textdatum_t::middle_left);
  snprintf(b, sizeof b, "MUNDO %d", gWorld);   gText(b, 220, 8, rgb(120, 220, 90), &fonts::Font0, textdatum_t::middle_left);
  gfx.fillRect(298, 4, 3, 9, G_WHITE); gfx.fillRect(304, 4, 3, 9, G_WHITE);
}

void gPanel(int x, int y, int w, int h) {
  gfx.fillRoundRect(x, y, w, h, 8, rgb(20, 28, 36));
  gfx.drawRoundRect(x, y, w, h, 8, C_OK);
}
void gButton(const Btn& b, const char* s) {
  gfx.fillRoundRect(b.x, b.y, b.w, b.h, 6, C_PANEL);
  gfx.drawRoundRect(b.x, b.y, b.w, b.h, 6, C_OK);
  gText(s, b.x + b.w / 2, b.y + b.h / 2, G_WHITE, &fonts::Font2, textdatum_t::middle_center);
}

void gDrawPlayer(int cam) {
  if (gState == G_TITLE || gState == G_OVER || gState == G_END) return;
  if (gInv && ((gInv / 3) % 2)) return;
  uint16_t a = (fabsf(gp.vx) > 0.3f && gp.ground) ? gAnim + 1 : 0;
  gDrawBud((int)gp.x - cam + gp.w / 2, gSY(gp.y + gp.h), 1, gBig, gFace, a, gState == G_DYING, gPower);
}

void gRender() {
  int cam = (int)gCam, camY = (int)gCamY;
  gDrawBackground();
  if (gArea == 0) {
    if (gCheckX > 0) gDrawCheckpoint(gCheckX - cam, gSY(gCheckY), gCheckOk);
    if (gWorld == 1 || (gWorld == 3 && gBossDone)) gDrawGoal(gGoalX - cam, gSY(10 * G_T));
  }
  if (gState == G_WARP) gDrawPlayer(cam);

  int c0 = cam / G_T, r0 = camY / G_T;
  for (int c = max(0, c0 - 1); c <= min(gW - 1, c0 + G_VW / G_T + 1); c++) {
    int sx = c * G_T - cam;
    for (int r = max(0, r0 - 1); r <= min(gH - 1, r0 + G_VROWS + 1); r++) {
      char ch = gMapBuf[r * gW + c];
      if (ch == '.') continue;
      int sy = G_HUD + r * G_T - camY;
      if (gBumpT && c == gBumpC && r == gBumpR) sy -= (gBumpT > 4) ? (8 - gBumpT) : gBumpT;
      gDrawTile(ch, sx, sy, c, r);
    }
  }

  for (auto& it : gIt) {
    if (!it.active) continue;
    if (it.kind == 1) gDrawSunItem((int)it.b.x - cam, gSY(it.b.y));
    else gDrawLeafItem((int)it.b.x - cam, gSY(it.b.y));
  }
  for (int i = 0; i < gEnN; i++) if (gEn[i].active && gEn[i].awake) gDrawEnemy(gEn[i], cam);
  for (auto& s : gShot) if (s.active) gDrawShot(s, cam);
  if (gState != G_WARP) gDrawPlayer(cam);

  char b[32];
  for (auto& p : gPop) {
    if (!p.active) continue;
    int px = (int)p.x - cam, py = gSY(p.y);
    if (p.kind == 0) {
      if (p.val == -1) snprintf(b, sizeof b, "+1 VIDA");
      else if (p.val == -2) snprintf(b, sizeof b, "CONTROL");
      else if (p.val == -3) snprintf(b, sizeof b, "AGUA!");
      else if (p.val == -4) snprintf(b, sizeof b, "SOL!");
      else snprintf(b, sizeof b, "%d", p.val);
      gText(b, px + 1, py + 1, G_BLACK, &fonts::Font0, textdatum_t::top_left);
      gText(b, px, py, G_WHITE, &fonts::Font0, textdatum_t::top_left);
    } else if (p.kind == 1) gDrawDrop(px, py);
    else if (p.kind == 3 || p.kind == 4) {
      if (p.timer > 20) {
        uint16_t c1 = p.kind == 3 ? rgb(255, 200, 60) : rgb(90, 170, 250);
        gfx.drawCircle(px + 3, py + 3, 31 - p.timer, c1);
      }
    } else if (p.kind == 5) gfx.fillCircle(px, py, 2, rgb(120, 230, 100));
    else gfx.fillRect(px, py, 4, 4, rgb(188, 96, 58));
  }

  // Barra de vida de la Reina
  if (gBossOn && !gBossDone) {
    for (int i = 0; i < gEnN; i++) if (gEn[i].type == EN_BOSS && gEn[i].active) {
      gText("REINA ARANA", 160, G_HUD + 8, rgb(255, 120, 100), &fonts::Font0, textdatum_t::middle_center);
      gfx.drawRect(100, G_HUD + 14, 120, 7, G_WHITE);
      gfx.fillRect(101, G_HUD + 15, 118 * gEn[i].hp / 8, 5, rgb(214, 44, 32));
    }
  }

  gDrawHUD();

  if (gState == G_TITLE) {
    gfx.fillRect(0, G_HUD, G_VW, 70, rgb(24, 32, 40));
    gText("COGOLLO", 162, G_HUD + 30, rgb(30, 60, 30), &fonts::FreeSansBold24pt7b, textdatum_t::middle_center);
    gText("COGOLLO", 160, G_HUD + 28, rgb(120, 220, 90), &fonts::FreeSansBold24pt7b, textdatum_t::middle_center);
    gText("Aventura en 3 mundos", 160, G_HUD + 58, rgb(200, 210, 220), &fonts::Font2, textdatum_t::middle_center);
    gDrawBud(40, G_HUD + 10 * G_T, 3, true, 1, 0, false, PW_NONE);
    gDrawMite(214, G_HUD + 10 * G_T - 12, 14, 12, 0, gAnim, -1, 1);
    gDrawAphid(240, G_HUD + 10 * G_T - 12, 14, 12, 0, gAnim, -1);
    gDrawMosq(272, G_HUD + 118 + (int)(sinf(gAnim * 0.1f) * 6), 12, 10, 0, gAnim, -1);
    if ((gAnim / 15) % 2) gText("Toca para jugar", 176, 110, G_WHITE, &fonts::Font4, textdatum_t::middle_center);
    if (gMaxWorld > 1) { snprintf(b, sizeof b, "Seguir mundo %d", gMaxWorld); gButton(gContBtn, b); }
    snprintf(b, sizeof b, "Record: %d", gBest);
    gText(b, 12, 189, G_WHITE, &fonts::Font2, textdatum_t::middle_left);
    gButton(gExitBtn, "Salir");
  } else if (gState == G_INTRO) {
    gfx.fillRect(0, G_HUD, G_VW, G_VROWS * G_T, rgb(12, 16, 20));
    snprintf(b, sizeof b, "MUNDO %d", gWorld);
    gText(b, 160, 80, G_WHITE, &fonts::FreeSansBold18pt7b, textdatum_t::middle_center);
    gText(G_WNAME[gWorld], 160, 112, rgb(120, 220, 90), &fonts::Font4, textdatum_t::middle_center);
    gDrawBud(140, 160, 1, false, 1, 0, false, PW_NONE);
    snprintf(b, sizeof b, "x %d", gLives);
    gText(b, 156, 154, G_WHITE, &fonts::Font2, textdatum_t::middle_left);
  } else if (gState == G_PAUSE) {
    gPanel(50, 46, 220, 130);
    gText("PAUSA", 160, 68, G_WHITE, &fonts::Font4, textdatum_t::middle_center);
    gButton(gResumeBtn, "Continuar");
    gButton(gQuitBtn, "Volver al inicio");
  } else if (gState == G_OVER) {
    gPanel(40, 56, 240, 100);
    gText("FIN DEL JUEGO", 160, 82, C_HIGH, &fonts::Font4, textdatum_t::middle_center);
    snprintf(b, sizeof b, "Puntaje: %d", gScore);  gText(b, 160, 110, G_WHITE, &fonts::Font2, textdatum_t::middle_center);
    snprintf(b, sizeof b, "Record: %d", gBest);    gText(b, 160, 130, rgb(200, 210, 220), &fonts::Font2, textdatum_t::middle_center);
  } else if (gState == G_WIN && gStateT >= 60) {
    gPanel(30, 50, 260, 110);
    snprintf(b, sizeof b, "MUNDO %d COMPLETO!", gWorld);
    gText(b, 160, 76, rgb(255, 225, 110), &fonts::Font4, textdatum_t::middle_center);
    snprintf(b, sizeof b, "Puntaje: %d", gScore);  gText(b, 160, 104, G_WHITE, &fonts::Font2, textdatum_t::middle_center);
    snprintf(b, sizeof b, "Gotas: %d   Vidas: %d", gDrops, gLives); gText(b, 160, 124, rgb(200, 210, 220), &fonts::Font2, textdatum_t::middle_center);
    if (gStateT > 90) gText("Toca para continuar", 160, 145, C_OK, &fonts::Font0, textdatum_t::middle_center);
  } else if (gState == G_END) {
    gPanel(20, 40, 280, 130);
    gText("SALVASTE LA COSECHA!", 160, 66, rgb(255, 225, 110), &fonts::Font4, textdatum_t::middle_center);
    gDrawBud(160, 120, 2, true, 1, gAnim, false, PW_SUN);
    snprintf(b, sizeof b, "Puntaje final: %d", gScore); gText(b, 160, 136, G_WHITE, &fonts::Font2, textdatum_t::middle_center);
    snprintf(b, sizeof b, "Record: %d", gBest);         gText(b, 160, 154, rgb(200, 210, 220), &fonts::Font2, textdatum_t::middle_center);
  }

  gfx.pushSprite(0, 0);
}

// ---------- Controles: ◀ ▶ ▼ | ↖ ▲ ↗ ● ----------
const int     G_BX[7] = { 2, 47, 92, 141, 186, 231, 276 };
const uint8_t G_BM[7] = { 1, 2, 8, 1 | 4, 4, 2 | 4, 16 };
#define G_BW 43

uint8_t gCtrlKey() { return gBtn | (gPower << 6); }

void gDrawControls() {
  gCtrlDrawn = gCtrlKey();
  lcd.fillRect(0, G_CTRL_Y, 320, 240 - G_CTRL_Y, C_BG);
  for (int i = 0; i < 7; i++) {
    bool on = (gBtn == G_BM[i]);
    int x = G_BX[i], y = G_CTRL_Y + 2, w = G_BW, h = 28;
    int cx = x + w / 2, cy = y + h / 2;
    lcd.fillRoundRect(x, y, w, h, 6, on ? C_OK : C_PANEL);
    lcd.drawRoundRect(x, y, w, h, 6, C_GRID);
    uint16_t ic = on ? C_BG : C_TXT;
    switch (i) {
      case 0: lcd.fillTriangle(cx + 6, cy - 8, cx + 6, cy + 8, cx - 8, cy, ic); break;
      case 1: lcd.fillTriangle(cx - 6, cy - 8, cx - 6, cy + 8, cx + 8, cy, ic); break;
      case 2: lcd.fillTriangle(cx - 8, cy - 6, cx + 8, cy - 6, cx, cy + 8, ic); break;
      case 3: lcd.fillTriangle(cx - 8, cy - 8, cx + 6, cy - 6, cx - 6, cy + 6, ic); break;
      case 4: lcd.fillTriangle(cx - 9, cy + 6, cx + 9, cy + 6, cx, cy - 8, ic); break;
      case 5: lcd.fillTriangle(cx + 8, cy - 8, cx - 6, cy - 6, cx + 6, cy + 6, ic); break;
      case 6: {
        uint16_t fc = on ? C_BG : (gPower == PW_SUN ? rgb(255, 200, 50) : (gPower == PW_WATER ? rgb(80, 170, 250) : C_GRID));
        lcd.fillCircle(cx, cy, 7, fc);
      } break;
    }
  }
}

// ---------- Un cuadro del juego (~30 por segundo) ----------
void gFrame() {
  int32_t tx = 0, ty = 0;
  bool down = lcd.getTouch(&tx, &ty);
  gPrevBtn = gBtn; gBtn = 0;
  if (down && ty >= G_CTRL_Y) {
    for (int i = 0; i < 7; i++) if (tx >= G_BX[i] - 1 && tx < G_BX[i] + G_BW + 2) { gBtn = G_BM[i]; break; }
  }
  bool tap = down && !gTouchPrev;
  gTouchPrev = down;
  if (down) lastActivity = millis();
  if (gCtrlKey() != gCtrlDrawn) gDrawControls();

  switch (gState) {
    case G_TITLE:
      if (tap && ty < G_CTRL_Y) {
        if (inBtn(gExitBtn, tx, ty)) { exitGame(); return; }
        if (gMaxWorld > 1 && inBtn(gContBtn, tx, ty)) gNewGame(gMaxWorld);
        else gNewGame(1);
      }
      break;
    case G_INTRO:
      if (++gStateT >= 60) { gState = G_PLAY; gStateT = 0; }
      break;
    case G_PLAY:
      if (tap && ty < G_HUD + 10 && tx > 280) { gState = G_PAUSE; break; }
      gUpdatePlay();
      break;
    case G_PAUSE:
      if (tap) {
        if (inBtn(gResumeBtn, tx, ty)) gState = G_PLAY;
        else if (inBtn(gQuitBtn, tx, ty)) { gSaveBest(); gToTitle(); }
      }
      break;
    case G_DYING: gUpdateDying(); break;
    case G_WARP:  gUpdateWarp();  break;
    case G_WIN:
      gUpdateWin();
      if (gStateT > 90 && tap && ty < G_CTRL_Y) {
        if (gWorld < 3) gStartWorld(gWorld + 1);
        else { gState = G_END; gStateT = 0; gSaveBest(); }
      }
      break;
    case G_OVER:
      gStateT++;
      if (gStateT > 30 && tap && ty < G_CTRL_Y) gToTitle();
      break;
    case G_END:
      gStateT++;
      if (gStateT > 60 && tap && ty < G_CTRL_Y) gToTitle();
      break;
  }
  if (gBumpT) gBumpT--;
  gAnim++;
  gUpdatePopups();
  gRender();
}

void startGame() {
  gaugeRelease();                          // libera memoria para el juego
  gfx.setColorDepth(16);
  if (!gfx.createSprite(G_VW, G_VH)) {
    gfx.setColorDepth(8);
    if (!gfx.createSprite(G_VW, G_VH)) { gaugeAllocate(); return; }
  }
  gameMode = true;
  gBest = prefs.getInt("gbest", 0);
  gMaxWorld = constrain(prefs.getUChar("gworld", 1), 1, 3);
  gToTitle();
  gAnim = 0;
  gBtn = gPrevBtn = 0; gCtrlDrawn = 0xFF;
  gTouchPrev = true;
  lcd.fillScreen(C_BG);
  gDrawControls();
  gLastFrame = millis();
}

void exitGame() {
  gSaveBest();
  gfx.deleteSprite();
  gaugeAllocate();
  gameMode = false;
  screen = S_LIVE;
  lastActivity = millis();
  drawAll();
}

// ---------------------------------------------------------------------------
// Setup / Loop
// ---------------------------------------------------------------------------
void setup() {
  Serial.begin(115200);
  Serial.printf("\nMonitor de clima — placa: %s\n", BOARD_NAME);

  prefs.begin("clima", false);
  loadConfig();

  lcd.init();
  lcd.setRotation(1);
  lcd.setBrightness(BRIGHTNESS);
  gaugeAllocate();
  lcd.fillScreen(C_BG);

  if (prefs.getBytesLength("tcal") == 16) {
    uint16_t cal[8]; prefs.getBytes("tcal", cal, sizeof cal);
    lcd.setTouchCalibrate(cal);
    // Recalibración de emergencia: si mantienes el dedo en la pantalla al encender
    txt("Manten presionada la pantalla", W / 2, 105, C_DIM, &fonts::Font2, textdatum_t::middle_center);
    txt("para recalibrar el touch", W / 2, 125, C_DIM, &fonts::Font2, textdatum_t::middle_center);
    uint32_t t0 = millis(); int hits = 0;
    while (millis() - t0 < 2000) {
      int32_t x, y;
      if (lcd.getTouch(&x, &y)) hits++;
      delay(20);
    }
    if (hits > 20) runCalibration();
    lcd.fillScreen(C_BG);
  } else {
    runCalibration();
  }

  Wire.begin(PIN_SDA, PIN_SCL);
  if (!sht.begin(0x44)) Serial.println("SHT30 no encontrado en 0x44");
  Wire.setClock(50000);

  if (strlen(WIFI_SSID) > 0) {
    WiFi.mode(WIFI_STA);
    WiFi.setHostname("monitor-clima");
    WiFi.setSleep(false);              // sin ahorro de energía: conexión más estable
    WiFi.setAutoReconnect(true);
    WiFi.begin(WIFI_SSID, WIFI_PASS);
    Serial.printf("WiFi: conectando a '%s'...\n", WIFI_SSID);
    configTzTime(TZ_CHILE, "pool.ntp.org", "time.google.com");
  }

  resetTracks();
  dayNow = computeDay();
  readSensor();
  drawAll();
  lastActivity = millis();
}

void loop() {
  static uint32_t lastRead = 0, lastTouch = 0, lastWifiCheck = 0, lastMinute = 0;
  static bool wasDown = false, wasConnected = false, prevAlarm = false;
  uint32_t ms = millis();

  // Lectura cada 2 s (cada 5 s mientras se juega). Sigue midiendo aunque
  // la pantalla esté apagada o haya un juego abierto.
  if (ms - lastRead >= (gameMode ? 5000UL : 2000UL)) {
    lastRead = ms;
    readSensor();
    if (sensorOk) { updateEvents(); accumulate(); }

    // Una alarma nueva cierra el juego / enciende la pantalla y muestra "En vivo"
    bool alarm = alarmActive();
    if (alarm && !prevAlarm) {
      if (gameMode) exitGame();
      else if (!screenOn) { screen = S_LIVE; screenWake(); }
    }
    prevAlarm = alarm;

    if (screenOn && !gameMode) {
      drawTopBar();
      if (screen == S_LIVE) drawLive();
      if (alertCount() != lastAlertsDrawn) drawTabs();
    }
    Serial.printf("T=%.2f HR=%.1f VPD=%.2f %s\n", cur[0], cur[1], cur[2], dayNow ? "DIA" : "NOCHE");
  }

  // Apagado automático por inactividad
  if (screenOn && SCR_SEC[scrIdx] > 0 && ms - lastActivity >= SCR_SEC[scrIdx] * 1000UL) {
    if (gameMode) exitGame();             // juego abandonado: se cierra y la pantalla se apaga
    screenSleep();
  }

  // Cada minuto: historial, día/noche, avance automático
  if (uptimeSec() - lastMinute >= HIST_SEC) {
    lastMinute = uptimeSec();
    bool redraw = pushHistory() && (screen == S_GRAPH || screen == S_REPORT);
    // ¿Comenzó hoy un cultivo programado?
    static bool lastActive = cropActive();
    bool act = cropActive();
    if (act != lastActive) { lastActive = act; resetTracks(); redraw = true; }
    bool d = computeDay();
    if (d != dayNow) { dayNow = d; resetTracks(); recalc(); redraw = true; }
    uint8_t before = stage;
    checkAutoAdvance();
    if (stage != before) redraw = true;
    if (redraw && screenOn && !gameMode) drawAll();
    else if (screenOn && !gameMode && screen == S_LIVE) drawTrendStrip();
  }

  // WiFi: revisa cada 10 s. Solo fuerza una reconexión si lleva más de
  // 60 s desconectado (antes se dejaba reconectar solo a la ESP32).
  static uint32_t wifiLostAt = 0;
  if (strlen(WIFI_SSID) > 0 && ms - lastWifiCheck >= 10000) {
    lastWifiCheck = ms;
    wl_status_t st = WiFi.status();
    bool conn = (st == WL_CONNECTED);
    if (conn) {
      wifiLostAt = 0;
    } else {
      if (wifiLostAt == 0) wifiLostAt = ms;
      if (ms - wifiLostAt >= 60000) {
        Serial.printf("WiFi: sin conexion hace 60 s (estado %d). Reintentando...\n", (int)st);
        WiFi.disconnect(true);
        delay(100);
        WiFi.mode(WIFI_STA);
        WiFi.setSleep(false);
        WiFi.begin(WIFI_SSID, WIFI_PASS);
        wifiLostAt = ms;
      }
    }
    if (conn != wasConnected) {
      wasConnected = conn;
      if (conn) Serial.printf("WiFi conectado: %s  senal %d dBm\n", WiFi.localIP().toString().c_str(), WiFi.RSSI());
      else      Serial.printf("WiFi desconectado (estado %d)\n", (int)st);
      if (screenOn && !gameMode) drawTopBar();
      if (screenOn && !gameMode && screen == S_SET) drawSettings();
    }
  }

  // Hora por NTP
  if (!timeSynced && strlen(WIFI_SSID) > 0 && time(nullptr) > 1700000000) {
    timeSynced = true;
    fillMissingDates();
    dayNow = computeDay(); recalc();
    if (screenOn && !gameMode) drawAll();
    Serial.println("Hora sincronizada");
  }

  // Juego: un cuadro cada ~33 ms
  if (gameMode) {
    if (ms - gLastFrame >= 33) { gLastFrame = ms; gFrame(); }
    wasDown = true;                       // al salir del juego no se arrastra el toque
    delay(1);
    return;
  }

  // Táctil
  int32_t tx, ty;
  bool down = lcd.getTouch(&tx, &ty);
  if (down && !wasDown && ms - lastTouch > 200) {
    lastTouch = ms;
    Serial.printf("Touch: x=%ld y=%ld\n", (long)tx, (long)ty);   // diagnóstico
    if (!screenOn) screenWake();          // el primer toque solo enciende la pantalla
    else { lastActivity = ms; handleTouch(tx, ty); }
  }
  wasDown = down;

  delay(10);
}
