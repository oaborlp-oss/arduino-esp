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
struct GEnemy { Body b; int8_t dir; uint8_t state, timer; bool active, awake; };
struct GItem  { Body b; int8_t dir; uint8_t state, timer, kind; bool active; };   // kind 0 hoja, 1 sol
struct GShot  { float x, y, vx, vy; uint8_t life; bool active; };
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
const uint16_t C_BG    = rgb(12, 14, 18);
const uint16_t C_PANEL = rgb(28, 32, 40);
const uint16_t C_GRID  = rgb(55, 60, 72);
const uint16_t C_TXT   = rgb(232, 234, 238);
const uint16_t C_DIM   = rgb(135, 142, 155);
const uint16_t C_OK    = rgb(70, 205, 125);
const uint16_t C_HIGH  = rgb(245, 95, 75);
const uint16_t C_LOW   = rgb(80, 165, 245);
const uint16_t C_WARN  = rgb(240, 180, 60);
const uint16_t C_BAND  = rgb(22, 58, 40);
const uint16_t C_SUN   = rgb(250, 200, 60);
const uint16_t C_MOON  = rgb(170, 180, 230);

// ---------------------------------------------------------------------------
// Etapas y rangos (día = luces encendidas, noche = luces apagadas)
// Ajusta a tu genética y sala.
// ---------------------------------------------------------------------------
enum { V_T = 0, V_RH = 1, V_VPD = 2 };
enum { ST_ESQ = 0, ST_VEGE = 1, ST_FLORA1 = 2, ST_FLORA2 = 3, ST_SECADO = 4, N_STAGES = 5, ST_NONE = 5 };
struct Stage { const char* name; const char* shortName; Range day[3]; Range night[3]; };

const Stage STAGES[N_STAGES + 1] = {
  //  nombre               corto       DIA: T        HR         VPD             NOCHE: T      HR         VPD
  { "Esqueje/Plantula", "Esqueje", { {22, 26}, {65, 80}, {0.4f, 0.8f} }, { {20, 24}, {65, 80}, {0.4f, 0.8f} } },
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
  stage       = prefs.getUChar("stage", ST_VEGE); if (stage >= N_STAGES) stage = ST_VEGE;
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
    lcd.fillCircle(x, y, 5, C_SUN);
  } else {
    lcd.fillCircle(x, y, 5, C_MOON);
    lcd.fillCircle(x + 3, y - 2, 4, C_PANEL);
  }
}

void drawTopBar() {
  lcd.fillRect(0, 0, W, TOP_H, C_PANEL);
  uint8_t cs = curStage();
  txt(STAGES[cs].shortName, 6, TOP_H / 2, cs == ST_NONE ? C_DIM : C_OK, &fonts::Font2, textdatum_t::middle_left);
  if (cs != ST_SECADO && cs != ST_NONE) drawSunMoon(lcd.textWidth(STAGES[cs].shortName) + 16, TOP_H / 2, dayNow);

  const char* titles[4] = { "Clima en vivo", "Historial", "Reportes", "Ajustes" };
  txt(titles[screen], W / 2 + 10, TOP_H / 2, C_TXT, &fonts::Font2, textdatum_t::middle_center);

  char b[16];
  if (timeSynced) {
    time_t e = time(nullptr); struct tm tmv; localtime_r(&e, &tmv);
    strftime(b, sizeof b, "%H:%M", &tmv);
  } else {
    char d[12]; fmtDur(uptimeSec(), d, sizeof d); snprintf(b, sizeof b, "up %s", d);
  }
  txt(b, W - 32, TOP_H / 2, C_DIM, &fonts::Font2, textdatum_t::middle_right);

  // Barras según intensidad real de la señal
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
    lcd.fillRect(W - 28 + i * 4, TOP_H / 2 + 6 - bh, 3, bh, i < bars ? wc : C_GRID);
  }
  lcd.fillCircle(W - 8, TOP_H / 2, 4, sensorOk ? C_OK : C_HIGH);
}

void drawTabs() {
  const char* lbl[4] = { "En vivo", "Grafico", "Reportes", "Ajustes" };
  for (int i = 0; i < 4; i++) {
    int x = i * 80; bool sel = (i == screen);
    lcd.fillRect(x, TAB_Y, 80, TAB_H, sel ? C_PANEL : C_BG);
    if (sel) lcd.fillRect(x + 8, TAB_Y, 64, 3, C_OK);
    txt(lbl[i], x + 40, TAB_Y + TAB_H / 2 + 1, sel ? C_TXT : C_DIM, &fonts::Font2, textdatum_t::middle_center);
  }
  lcd.drawFastHLine(0, TAB_Y, W, C_GRID);
}

// ---- En vivo ----
void drawLiveRow(uint8_t v) {
  int y = CONTENT_Y + 3 + v * 60;
  float x = cur[v];
  int st = statusOf(v, x);
  uint16_t col = isnan(x) ? C_DIM : statusColor(st);
  const Range& r = rangeOf(v, dayNow);

  lcd.fillRoundRect(4, y, W - 8, 56, 6, C_PANEL);
  lcd.fillRoundRect(4, y, 6, 56, 3, col);

  char b[44];
  if (v == V_T)       snprintf(b, sizeof b, "Temperatura");
  else if (v == V_RH) snprintf(b, sizeof b, "Humedad relativa");
  else if (curStage() == ST_SECADO) snprintf(b, sizeof b, "VPD aire");
  else if (curStage() == ST_NONE) snprintf(b, sizeof b, "VPD hoja (hoja %+.1fC)", leafOff);
  else if (dayNow)    snprintf(b, sizeof b, "VPD hoja (hoja %+.1fC)", leafOff);
  else                snprintf(b, sizeof b, "VPD (luces apagadas)");
  txt(b, 16, y + 5, C_DIM);

  uint8_t cs = curStage();
  const char* when = (cs == ST_SECADO) ? "Rango" : (cs == ST_NONE ? "General" : (dayNow ? "Dia" : "Noche"));
  snprintf(b, sizeof b, "%s %.*f - %.*f %s", when, VAR_DEC[v], r.lo, VAR_DEC[v], r.hi, VAR_UNIT[v]);
  txt(b, 16, y + 21, C_DIM);

  const char* sw = isnan(x) ? "Sin lectura" : (st > 0 ? "ALTO" : (st < 0 ? "BAJO" : "OK"));
  txt(sw, 16, y + 37, col);

  if (isnan(x)) snprintf(b, sizeof b, "--"); else snprintf(b, sizeof b, "%.*f", VAR_DEC[v], x);
  txt(b, 266, y + 29, col, &fonts::FreeSansBold18pt7b, textdatum_t::middle_right);

  int ux = 272;
  if (v == V_T) { lcd.drawCircle(ux + 2, y + 21, 2, C_TXT); ux += 7; }
  txt(VAR_UNIT[v], ux, y + 18, C_TXT);
}
void drawLive() { for (uint8_t v = 0; v < 3; v++) drawLiveRow(v); }

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
    case S_LIVE:   clearContent(); drawLive(); break;
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
//  JUEGO "COGOLLO" — plataformas original (personaje, enemigos y nivel propios)
//  Un cogollo recorre el indoor: junta gotas, vence arañitas rojas, baja por
//  macetas-portal al subsuelo de raíces y llega al foco de luz.
// ===========================================================================
#define G_T       16      // tamaño de bloque
#define G_ROWS    12
#define G_COLS    176
#define G_HUD     16
#define G_VW      320
#define G_VH      208     // HUD + 12 filas
#define G_CTRL_Y  208     // franja de controles táctiles
#define G_GRAV    0.5f
#define G_MAXFALL 9.0f
#define G_ACC     0.45f
#define G_MAXSPD  3.0f
#define G_JUMPV   8.6f
#define G_MAXEN   24
#define G_MAXIT   4
#define G_MAXPOP  16

const uint16_t G_WHITE = rgb(255, 255, 255);
const uint16_t G_BLACK = rgb(0, 0, 0);
const uint16_t G_DROP  = rgb(60, 145, 235);

LGFX_Sprite gfx(&lcd);
bool gameMode = false;
char gMap[G_ROWS][G_COLS];

enum { G_TITLE = 0, G_PLAY, G_PAUSE, G_DYING, G_WIN, G_OVER, G_WARP };
uint8_t  gState = G_TITLE;
Body     gp;
bool     gBig = false;
uint8_t  gInv = 0;
int8_t   gFace = 1;
uint16_t gAnim = 0, gStateT = 0;
int      gLives = 3, gScore = 0, gDrops = 0, gTime = 300, gBest = 0;
uint8_t  gTimeTick = 0;
float    gCam = 0;
int      gCheckX = 0, gGoalX = 0;
bool     gCheckOk = false;
GEnemy   gEn[G_MAXEN]; int gEnN = 0;
GItem    gIt[G_MAXIT];
GPopup   gPop[G_MAXPOP];
uint8_t  gCoyote = 0, gJumpBuf = 0;
uint8_t  gBtn = 0, gPrevBtn = 0, gCtrlDrawn = 0xFF;
bool     gTouchPrev = false;
int      gBumpC = -1, gBumpR = -1; uint8_t gBumpT = 0;
uint32_t gLastFrame = 0;

// Áreas: 0 = superficie, 1 = subsuelo (raíces)
#define G_UCOLS 44
int      gW = G_COLS;                 // ancho (columnas) del área actual
uint8_t  gArea = 0;
char     gMapSave[G_ROWS][G_COLS];    // superficie guardada mientras se está abajo
GEnemy   gEnSave[G_MAXEN]; int gEnSaveN = 0;
#define  G_EXIT_COL 118               // maceta de salida en la superficie
uint8_t  gWarpT = 0, gWarpPhase = 0;  // 0 = bajando, 1 = saliendo

// Poder del sol: bolitas de luz
bool     gFire = false;
GShot    gShot[2];
uint8_t  gShotCd = 0;

Btn gResumeBtn = { 70, 92, 180, 30 };
Btn gQuitBtn   = { 70, 132, 180, 30 };
Btn gExitBtn   = { 226, 176, 88, 26 };

// ---------- Nivel ----------
void gSet(int c, int r, char ch) { if (c >= 0 && c < gW && r >= 0 && r < G_ROWS) gMap[r][c] = ch; }
void gGround(int a, int b) { for (int c = a; c <= b; c++) { gSet(c, 10, '#'); gSet(c, 11, '#'); } }
void gRowFill(int a, int b, int r, char ch) { for (int c = a; c <= b; c++) gSet(c, r, ch); }
void gStack(int c, int h) { for (int i = 0; i < h; i++) gSet(c, 9 - i, 'K'); }
void gPot(int c, int h) {
  for (int i = 0; i < h; i++) {
    int r = 9 - i; bool top = (i == h - 1);
    gSet(c, r, top ? 'T' : 't'); gSet(c + 1, r, top ? 'Y' : 'y');
  }
}
void gAddEnemy(int c) {
  if (gEnN >= G_MAXEN) return;
  GEnemy& e = gEn[gEnN++];
  e.b = { (float)(c * G_T + 1), (float)(10 * G_T - 12), 0, 0, 14, 12, false };
  e.dir = -1; e.state = 0; e.timer = 0; e.active = true; e.awake = false;
}

// Mapa: '#' tierra, 'B' ladrillo, '?' bloque con gota, 'G' bloque con hoja,
// 'F' bloque con sol, 'U' bloque usado, 'K' cajón, 'T','Y','t','y' maceta,
// 'P','Q' boca de maceta-portal, 'R' tierra de raíces (subsuelo), 'o' gota
void gBuildLevel() {
  gArea = 0; gW = G_COLS;
  memset(gMap, '.', sizeof gMap);
  gEnN = 0;
  // Tramo 1: el semillero
  gGround(0, 38);
  gSet(14, 6, '?');
  gRowFill(19, 23, 6, 'B'); gSet(20, 6, '?'); gSet(21, 6, 'G'); gSet(22, 6, '?');
  gSet(21, 2, '?');
  gAddEnemy(25);
  gPot(28, 2);
  gAddEnemy(32);
  gPot(35, 3);
  gRowFill(39, 41, 6, 'o');                       // gotas guía sobre el hueco
  // Tramo 2: macetas y repisas
  gGround(42, 73);
  gPot(45, 4); gSet(45, 6, 'P'); gSet(46, 6, 'Q');   // maceta-portal al subsuelo
  gRowFill(50, 54, 6, 'B'); gSet(52, 6, '?'); gRowFill(50, 54, 5, 'o');
  gAddEnemy(56); gAddEnemy(58);
  gRowFill(60, 62, 7, 'K');
  gSet(61, 3, 'G');
  gSet(66, 6, '?'); gSet(68, 6, '?');
  gAddEnemy(67); gAddEnemy(70);
  gCheckX = 72 * G_T;                             // punto de control
  gSet(74, 7, 'o'); gSet(75, 6, 'o'); gSet(76, 7, 'o');
  // Tramo 3: escalones de cajones
  gGround(77, 86);
  for (int i = 0; i < 4; i++) gStack(83 + i, i + 1);
  gGround(89, 122);
  for (int i = 0; i < 4; i++) gStack(89 + i, 4 - i);
  gAddEnemy(97); gAddEnemy(99); gAddEnemy(101);
  gRowFill(103, 109, 6, 'B'); gSet(104, 6, '?'); gSet(106, 6, 'F'); gRowFill(104, 108, 2, 'o');
  gPot(113, 2); gPot(118, 3);
  gRowFill(124, 125, 7, 'K');                     // cajón flotante sobre el hueco
  // Tramo 4: la recta final
  gGround(127, G_COLS - 1);
  gAddEnemy(131); gAddEnemy(133); gAddEnemy(136);
  gRowFill(138, 142, 6, 'B'); gSet(140, 6, '?'); gRowFill(138, 142, 5, 'o');
  for (int i = 0; i < 8; i++) gStack(146 + i, i + 1);   // gran escalera
  gGoalX = 162 * G_T + 6;                         // foco de luz (meta)
}

// Subsuelo: cueva de raíces llena de gotas, con un bloque de sol y salida por maceta
void gBuildUnder() {
  gArea = 1; gW = G_UCOLS;
  memset(gMap, '.', sizeof gMap);
  gEnN = 0;
  for (int c = 0; c < G_UCOLS; c++) { gSet(c, 10, 'R'); gSet(c, 11, 'R'); if (c < 1 || c > 4) gSet(c, 0, 'R'); }
  for (int r = 0; r < G_ROWS; r++) { gSet(0, r, 'R'); gSet(G_UCOLS - 1, r, 'R'); }
  gRowFill(33, 42, 1, 'R');                       // techo bajo al fondo
  for (int r = 8; r <= 9; r++) gRowFill(5, 12, r, 'o');
  gRowFill(14, 19, 6, 'R'); gRowFill(14, 19, 4, 'o'); gRowFill(14, 19, 5, 'o');
  gSet(22, 6, '?'); gSet(23, 6, 'F'); gSet(24, 6, '?');
  gRowFill(27, 31, 7, 'R'); gRowFill(27, 31, 5, 'o');
  for (int r = 8; r <= 9; r++) gRowFill(33, 36, r, 'o');
  gAddEnemy(17); gAddEnemy(29);
  gPot(39, 2); gSet(39, 8, 'P'); gSet(40, 8, 'Q');   // maceta de salida
}

// ---------- Física ----------
int  gFloorDiv(float v) { return (int)floorf(v / G_T); }
bool gSolidCh(char c) {
  return c == '#' || c == 'B' || c == '?' || c == 'G' || c == 'F' || c == 'U' || c == 'K' ||
         c == 'T' || c == 'Y' || c == 't' || c == 'y' || c == 'P' || c == 'Q' || c == 'R';
}
bool gSolidAt(int tx, int ty) {
  if (tx < 0 || tx >= gW) return true;
  if (ty < 0 || ty >= G_ROWS) return false;
  return gSolidCh(gMap[ty][tx]);
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
  b.vy += G_GRAV; if (b.vy > G_MAXFALL) b.vy = G_MAXFALL;
  b.y += b.vy;
  b.ground = false;
  int l = gFloorDiv(b.x), r = gFloorDiv(b.x + b.w - 1);
  if (b.vy > 0) {
    int ty = gFloorDiv(b.y + b.h - 1);
    for (int tx = l; tx <= r; tx++) if (gSolidAt(tx, ty)) { b.y = ty * G_T - b.h; b.vy = 0; b.ground = true; break; }
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

// ---------- Eventos de juego ----------
void gPopup(float x, float y, int val, uint8_t kind, float vx, float vy) {
  for (auto& p : gPop) if (!p.active) { p = { x, y, vx, vy, (int16_t)val, kind, 30, true }; return; }
}

void gSaveBest() { if (gScore > gBest) { gBest = gScore; prefs.putInt("gbest", gBest); } }

void gAddDrop(float x, float y) {
  gDrops++; gScore += 10;
  if (gDrops >= 30) { gDrops -= 30; gLives++; gPopup(x, y - 10, -1, 0, 0, -1); }   // vida extra
}

void gBump(int c, int r) {
  char ch = gMap[r][c];
  float bx = c * G_T, by = r * G_T;
  if (ch == '?') { gMap[r][c] = 'U'; gAddDrop(bx, by); gPopup(bx + 8, by - 6, 0, 1, 0, -3); }
  else if (ch == 'G' || ch == 'F') {
    gMap[r][c] = 'U';
    for (auto& it : gIt) if (!it.active) {
      it.b = { bx + 2, by, 0, 0, 12, 12, false }; it.dir = 1; it.state = 0; it.timer = 0;
      it.kind = (ch == 'F') ? 1 : 0; it.active = true; break;
    }
  } else if (ch == 'B' && gBig) {                 // la semilla grande rompe ladrillos
    gMap[r][c] = '.'; gScore += 50;
    gPopup(bx + 3, by + 3, 0, 2, -1.5f, -5); gPopup(bx + 11, by + 3, 0, 2, 1.5f, -5);
    gPopup(bx + 3, by + 11, 0, 2, -1.0f, -3); gPopup(bx + 11, by + 11, 0, 2, 1.0f, -3);
    return;
  }
  if (gSolidCh(gMap[r][c])) { gBumpC = c; gBumpR = r; gBumpT = 8; }
  // Golpe desde abajo: derrota plagas que estén encima y junta gotas
  for (int i = 0; i < gEnN; i++) {
    GEnemy& e = gEn[i];
    if (!e.active || e.state != 0) continue;
    if (fabsf(e.b.y + e.b.h - by) <= 3 && e.b.x + e.b.w > bx && e.b.x < bx + G_T) {
      e.state = 2; e.b.vy = -5; e.b.vx = (e.b.x < bx + 8) ? -1 : 1;
      gScore += 100; gPopup(e.b.x, e.b.y, 100, 0, 0, -1);
    }
  }
  if (r > 0 && gMap[r - 1][c] == 'o') { gMap[r - 1][c] = '.'; gAddDrop(bx, by - G_T); gPopup(bx + 8, by - 20, 0, 1, 0, -3); }
}

void gDie() { gState = G_DYING; gStateT = 0; gp.vy = -7; gp.vx = 0; gBig = false; gFire = false; }

void gHurt() {
  if (gInv) return;
  if (gFire) { gFire = false; gInv = 60; }                     // pierde el poder del sol
  else if (gBig) { gBig = false; gp.y += 8; gp.h = 14; gInv = 60; }
  else gDie();
}

// ---------- Bolitas de luz ----------
void gShoot() {
  if (!gFire || gShotCd) return;
  for (auto& s : gShot) if (!s.active) {
    s.x = gFace > 0 ? gp.x + gp.w : gp.x - 6;
    s.y = gp.y + 6;
    s.vx = 4.5f * gFace; s.vy = 1.0f; s.life = 90; s.active = true;
    gShotCd = 10;
    return;
  }
}

void gUpdateShots() {
  if (gShotCd) gShotCd--;
  for (auto& s : gShot) {
    if (!s.active) continue;
    s.x += s.vx;
    if (gSolidAt(gFloorDiv(s.x + 3), gFloorDiv(s.y + 3))) {      // choca con un muro
      s.active = false; gPopup(s.x, s.y, 0, 3, 0, 0); continue;
    }
    s.vy += 0.45f; s.y += s.vy;
    if (s.vy > 0 && gSolidAt(gFloorDiv(s.x + 3), gFloorDiv(s.y + 6))) {   // rebota en el suelo
      s.y = gFloorDiv(s.y + 6) * G_T - 6; s.vy = -3.6f;
    }
    if (--s.life == 0 || s.x < gCam - 16 || s.x > gCam + G_VW + 16 || s.y > G_ROWS * G_T) { s.active = false; continue; }
    for (int i = 0; i < gEnN; i++) {
      GEnemy& e = gEn[i];
      if (!e.active || !e.awake || e.state != 0) continue;
      if (s.x < e.b.x + e.b.w && s.x + 6 > e.b.x && s.y < e.b.y + e.b.h && s.y + 6 > e.b.y) {
        e.state = 2; e.b.vy = -4; e.b.vx = s.vx > 0 ? 1 : -1;
        gScore += 100; gPopup(e.b.x, e.b.y - 4, 100, 0, 0, -1);
        gPopup(s.x, s.y, 0, 3, 0, 0);
        s.active = false; break;
      }
    }
  }
}

// ---------- Macetas-portal ----------
// ¿Está la semilla parada sobre la boca de una maceta-portal? Devuelve su columna izquierda.
int gPotUnderPlayer() {
  if (!gp.ground) return -1;
  int row = gFloorDiv(gp.y + gp.h);
  int cx  = gFloorDiv(gp.x + gp.w / 2);
  if (row < 0 || row >= G_ROWS || cx < 0 || cx >= gW) return -1;
  char ch = gMap[row][cx];
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
  if (gWarpPhase == 0) {                       // bajando dentro de la maceta
    gp.y += 1;
    if (gWarpT >= gp.h + 4) {
      for (auto& it : gIt) it.active = false;
      for (auto& p : gPop) p.active = false;
      if (gArea == 0) {                        // superficie -> subsuelo
        memcpy(gMapSave, gMap, sizeof gMap);
        memcpy(gEnSave, gEn, sizeof gEn); gEnSaveN = gEnN;
        gBuildUnder();
        gp.x = 2 * G_T; gp.y = 0; gp.vy = 0;
        gCam = 0; gFace = 1;
        gState = G_PLAY;                       // cae desde la abertura del techo
      } else {                                 // subsuelo -> superficie
        gArea = 0; gW = G_COLS;
        memcpy(gMap, gMapSave, sizeof gMap);
        memcpy(gEn, gEnSave, sizeof gEn); gEnN = gEnSaveN;
        gSet(45, 6, 'T'); gSet(46, 6, 'Y');    // el portal se cierra (el subsuelo se visita una vez)
        int topRow = 7;                        // boca de la maceta de salida (altura 3)
        gp.x = G_EXIT_COL * G_T + G_T - gp.w / 2;
        gp.y = topRow * G_T;
        gCam = constrain(gp.x - 130, 0.0f, (float)(G_COLS * G_T - G_VW));
        if (!gCheckOk) gCheckOk = true;
        gWarpPhase = 1; gWarpT = 0;
      }
    }
  } else {                                     // saliendo de la maceta
    gp.y -= 1;
    if (gp.y + gp.h <= 7 * G_T) { gp.y = 7 * G_T - gp.h; gp.vy = 0; gState = G_PLAY; }
  }
}

void gResetPlayer() {
  float sx = gCheckOk ? gCheckX : 3 * G_T;
  gp = { sx, (float)(10 * G_T - 14), 0, 0, 12, 14, false };
  gBig = false; gFire = false; gInv = 0; gFace = 1; gShotCd = 0;
  for (auto& s : gShot) s.active = false;
  gCam = max(0.0f, sx - 130);
  gTime = 300; gTimeTick = 0; gBumpT = 0;
  gJumpBuf = gCoyote = 0;
  for (auto& it : gIt) it.active = false;
  for (auto& p : gPop) p.active = false;
}
void gNewLife() { gBuildLevel(); gResetPlayer(); gState = G_PLAY; }
void gNewGame() { gLives = 3; gScore = 0; gDrops = 0; gCheckOk = false; gNewLife(); }

// ---------- Actualización ----------
void gUpdateEnemies() {
  for (int i = 0; i < gEnN; i++) {
    GEnemy& e = gEn[i];
    if (!e.active) continue;
    if (!e.awake) { if (e.b.x < gCam + G_VW + 24) e.awake = true; else continue; }
    if (e.b.x + e.b.w < gCam - 48) { e.active = false; continue; }
    if (e.state == 0) {
      e.b.vx = e.dir * 0.55f;
      bool hit; gMoveX(e.b, hit); if (hit) e.dir = -e.dir;
      int bc, br; gMoveY(e.b, bc, br);
      if (e.b.y > G_ROWS * G_T) { e.active = false; continue; }
      for (int j = 0; j < gEnN; j++)
        if (j != i && gEn[j].active && gEn[j].state == 0 && gOverlap(e.b, gEn[j].b)) e.dir = (e.b.x < gEn[j].b.x) ? -1 : 1;
      if (gState == G_PLAY && gOverlap(gp, e.b)) {
        if (gp.vy > 0 && gp.y + gp.h - e.b.y < 10) {          // ¡pisotón!
          e.state = 1; e.timer = 20;
          gp.vy = (gBtn & 4) ? -8.5f : -5.5f;
          gScore += 100; gPopup(e.b.x, e.b.y - 4, 100, 0, 0, -1);
        } else gHurt();
      }
    } else if (e.state == 1) {
      if (--e.timer == 0) e.active = false;
    } else {
      e.b.vy += G_GRAV; e.b.x += e.b.vx; e.b.y += e.b.vy;
      if (e.b.y > G_ROWS * G_T + 32) e.active = false;
    }
  }
}

void gUpdateItems() {
  for (auto& it : gIt) {
    if (!it.active) continue;
    if (it.state == 0) { it.b.y -= 1; if (++it.timer >= 14) it.state = 1; continue; }   // brota del bloque
    if (it.kind == 0) {                          // la hoja se desplaza; el sol se queda quieto
      it.b.vx = it.dir * 1.2f;
      bool hit; gMoveX(it.b, hit); if (hit) it.dir = -it.dir;
      int bc, br; gMoveY(it.b, bc, br);
      if (it.b.y > G_ROWS * G_T) { it.active = false; continue; }
    }
    if (gOverlap(gp, it.b)) {
      it.active = false; gScore += 1000; gPopup(it.b.x, it.b.y - 6, 1000, 0, 0, -1);
      if (!gBig) { gBig = true; gp.y -= 8; gp.h = 22; }
      if (it.kind == 1) gFire = true;
    }
  }
}

void gUpdatePopups() {
  for (auto& p : gPop) {
    if (!p.active) continue;
    p.x += p.vx; p.y += p.vy;
    if (p.kind == 2) p.vy += G_GRAV; else if (p.kind == 1) p.vy *= 0.9f;
    if (--p.timer == 0) p.active = false;
  }
}

void gUpdatePlay() {
  bool L = gBtn & 1, R = gBtn & 2, J = gBtn & 4;
  bool Jp = J && !(gPrevBtn & 4);
  bool Dp = (gBtn & 8) && !(gPrevBtn & 8);
  bool Fp = (gBtn & 16) && !(gPrevBtn & 16);

  if (Dp) { int pc = gPotUnderPlayer(); if (pc >= 0) { gStartWarp(pc); return; } }
  if (Fp) gShoot();
  if (L && !R)      { gp.vx -= G_ACC; gFace = -1; }
  else if (R && !L) { gp.vx += G_ACC; gFace = 1; }
  else gp.vx *= gp.ground ? 0.75f : 0.985f;       // en el aire conserva el impulso
  if (fabsf(gp.vx) < 0.05f) gp.vx = 0;
  gp.vx = constrain(gp.vx, -G_MAXSPD, G_MAXSPD);

  // Salto con "memoria": tolera tocar un poco antes de aterrizar o justo después de caer
  if (Jp) gJumpBuf = 6; else if (gJumpBuf) gJumpBuf--;
  if (gp.ground) gCoyote = 6; else if (gCoyote) gCoyote--;
  if (gJumpBuf && gCoyote) { gp.vy = -G_JUMPV; gJumpBuf = 0; gCoyote = 0; }
  if (!J && gp.vy < -3.5f) gp.vy = -3.5f;         // soltar = salto corto

  bool hit; gMoveX(gp, hit);
  if (gp.x < gCam) { gp.x = gCam; if (gp.vx < 0) gp.vx = 0; }
  int bc, br; gMoveY(gp, bc, br);
  if (bc >= 0) gBump(bc, br);

  for (int ty = gFloorDiv(gp.y); ty <= gFloorDiv(gp.y + gp.h - 1); ty++)
    for (int tx = gFloorDiv(gp.x); tx <= gFloorDiv(gp.x + gp.w - 1); tx++)
      if (ty >= 0 && ty < G_ROWS && tx >= 0 && tx < gW && gMap[ty][tx] == 'o') {
        gMap[ty][tx] = '.'; gAddDrop(tx * G_T, ty * G_T);
      }

  if (gp.y > G_ROWS * G_T) { gDie(); gp.vy = 0; return; }
  if (gArea == 0) {
    if (!gCheckOk && gp.x > gCheckX) { gCheckOk = true; gPopup(gCheckX - 10, 6 * G_T, -2, 0, 0, -1); }
    if (gp.x + gp.w >= gGoalX) { gState = G_WIN; gStateT = 0; return; }
  }

  if (++gTimeTick >= 15) { gTimeTick = 0; if (--gTime <= 0) { gDie(); return; } }
  if (gInv) gInv--;

  float target = gp.x - 130;
  if (target > gCam) gCam = target;
  gCam = min(gCam, (float)(gW * G_T - G_VW));

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
  if (gStateT < 60) { gp.vx = 1.2f; gFace = 1; bool hit; gMoveX(gp, hit); int bc, br; gMoveY(gp, bc, br); }
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

// Protagonista: un cogollo con patitas, pistilos naranjos y tricomas.
// (cx, by) = centro inferior, s = escala. Con el poder del sol, los tricomas brillan dorados.
void gDrawBud(int cx, int by, int s, bool big, int8_t face, uint16_t anim, bool dead, bool fire) {
  const uint16_t cBody  = fire ? rgb(120, 190, 70) : rgb(86, 160, 66);
  const uint16_t cDark  = rgb(48, 112, 44);
  const uint16_t cCalyx = fire ? rgb(170, 220, 100) : rgb(130, 196, 92);
  const uint16_t cLeaf  = rgb(40, 120, 46);
  const uint16_t cPist  = fire ? rgb(255, 170, 40) : rgb(232, 128, 44);
  const uint16_t cTric  = fire ? rgb(255, 222, 90) : rgb(245, 250, 240);
  const uint16_t cFoot  = rgb(70, 50, 30);

  int H = (big ? 22 : 14) * s;
  int step = anim ? (anim / 3) % 2 : 0;
  // patitas
  gfx.fillEllipse(cx - 3 * s, by - s - (step ? s : 0), 2 * s + 1, s + 1, cFoot);
  gfx.fillEllipse(cx + 3 * s, by - s - ((anim && !step) ? s : 0), 2 * s + 1, s + 1, cFoot);

  int bottom = by - 2 * s, top = by - H;
  int ry = (big ? 7 : 5) * s;
  int cyL = bottom - ry;                                  // centro de la parte baja (más ancha)
  // hojitas de azúcar a los lados
  gfx.fillEllipse(cx - 7 * s, cyL + s, 3 * s, s + 1, cLeaf);
  gfx.fillEllipse(cx + 7 * s, cyL + s, 3 * s, s + 1, cLeaf);
  // cuerpo: base redonda + punta cónica
  gfx.fillTriangle(cx - 5 * s, cyL, cx + 5 * s, cyL, cx, top, cBody);
  gfx.fillEllipse(cx, (cyL + top) / 2 + s, 4 * s, (cyL - top) / 2, cBody);
  gfx.fillEllipse(cx, cyL, 6 * s, ry, cBody);
  gfx.drawEllipse(cx, cyL, 6 * s, ry, cDark);
  // cálices (relieve)
  gfx.fillEllipse(cx - 3 * s, cyL + 2 * s, 2 * s, s + 1, cCalyx);
  gfx.fillEllipse(cx + 3 * s, cyL - s, 2 * s, s + 1, cCalyx);
  gfx.fillEllipse(cx - s, (cyL + top) / 2, 2 * s, s + 1, cCalyx);
  if (big) gfx.fillEllipse(cx + 2 * s, (cyL + top) / 2 + 3 * s, 2 * s, s + 1, cCalyx);
  // pistilos
  gfx.drawLine(cx - 5 * s, cyL - 2 * s, cx - 8 * s, cyL - 4 * s, cPist);
  gfx.drawLine(cx + 4 * s, cyL - 4 * s, cx + 7 * s, cyL - 7 * s, cPist);
  gfx.drawLine(cx, top, cx - 2 * s, top - 3 * s, cPist);
  gfx.drawLine(cx, top, cx + 2 * s, top - 3 * s, cPist);
  if (big) gfx.drawLine(cx - 3 * s, (cyL + top) / 2, cx - 6 * s, (cyL + top) / 2 - 3 * s, cPist);
  // tricomas
  const int8_t tx[7] = { -4, 3, -1, 2, -3, 4, 0 };
  const int8_t ty[7] = { -1, 2, -4, -6, 3, -2, -8 };
  for (int k = 0; k < (big ? 7 : 5); k++) {
    int px = cx + tx[k] * s, py = cyL + ty[k] * s * (big ? 1 : 1);
    if (py > top) gfx.fillCircle(px, py, s > 1 ? s / 2 + 1 : 0, cTric);
  }
  if (fire && (anim / 2 + gAnim / 4) % 2) gfx.drawPixel(cx + 5 * s, top + 2 * s, G_WHITE);   // destello
  // carita
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

// Enemigo: arañita roja — state 0 camina, 1 aplastada, 2 volteada
void gDrawMite(int x, int y, int w, int h, uint8_t state, uint16_t anim, int8_t dir) {
  const uint16_t cR = rgb(214, 44, 32), cRd = rgb(150, 24, 18), cSpot = rgb(90, 12, 10);
  const uint16_t cLeg = rgb(120, 20, 16), cHi = rgb(255, 120, 100);
  int cx = x + w / 2, by = y + h;
  if (state == 1) {                                       // aplastada
    gfx.fillEllipse(cx, by - 2, 8, 3, cRd);
    gfx.fillEllipse(cx, by - 3, 6, 2, cR);
    for (int k = -3; k <= 3; k += 2) gfx.drawFastHLine(cx + k * 2 - 1, by - 1, 3, cLeg);
    return;
  }
  int sg = (state == 2) ? -1 : 1;                         // volteada: patas hacia arriba
  int cy = by - 5;
  int st = (anim / 3) % 2;
  // 8 patas (4 por lado), alternando el paso
  for (int k = 0; k < 4; k++) {
    int ox = -4 + k * 3;
    int sw = ((k + st) % 2) ? 2 : -1;
    gfx.drawLine(cx + ox, cy, cx + ox - 3 + sw, cy + sg * 5, cLeg);
    gfx.drawLine(cx + ox, cy, cx + ox + 3 + sw, cy + sg * 5, cLeg);
  }
  gfx.fillEllipse(cx - dir, cy - sg, 6, 4, cR);           // abdomen
  gfx.fillEllipse(cx + dir * 5, cy - sg, 3, 3, cRd);      // cabeza
  gfx.fillCircle(cx - dir * 2, cy - sg * 2, 1, cSpot);    // manchitas oscuras
  gfx.fillCircle(cx + dir, cy, 1, cSpot);
  gfx.drawPixel(cx - dir * 3, cy - sg * 3, cHi);          // brillo
  gfx.drawPixel(cx + dir * 6, cy - sg * 2, G_WHITE);      // ojitos
  gfx.drawPixel(cx + dir * 6, cy - sg * 1, G_BLACK);
  gfx.drawLine(cx + dir * 7, cy - sg * 2, cx + dir * 9, cy - sg * 4, cLeg);   // palpos
}

// Sol (poder): gira y brilla
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
  gfx.drawPixel(cx - 2, cy + 1, rgb(140, 90, 20)); gfx.drawPixel(cx + 2, cy + 1, rgb(140, 90, 20));
}

// Bolita de luz
void gDrawShot(int x, int y) {
  bool f = (gAnim / 2) % 2;
  gfx.fillCircle(x + 3, y + 3, 4, f ? rgb(255, 150, 30) : rgb(255, 190, 50));
  gfx.fillCircle(x + 3, y + 3, 2, rgb(255, 240, 150));
  gfx.drawPixel(x + 2, y + 2, G_WHITE);
}

void gDrawLeafItem(int x, int y) {
  int cx = x + 6, cy = y + 6;
  gfx.fillEllipse(cx, cy, 7, 4, rgb(60, 190, 70));
  gfx.drawLine(cx - 6, cy + 1, cx + 6, cy - 1, rgb(30, 110, 40));
  gfx.fillEllipse(cx - 2, cy - 2, 2, 1, rgb(180, 245, 160));
  if ((gAnim / 4) % 2) { gfx.drawFastHLine(cx + 4, cy - 6, 5, G_WHITE); gfx.drawFastVLine(cx + 6, cy - 8, 5, G_WHITE); }
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
      gfx.drawPixel(x + 6, y + 9 + (hsh & 1), rgb(165, 112, 70));
      if (r > 0 && !gSolidCh(gMap[r - 1][c])) {
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
      gfx.drawPixel(x + 2, y + 2, rgb(160, 100, 20)); gfx.drawPixel(x + 13, y + 2, rgb(160, 100, 20));
      gfx.drawPixel(x + 2, y + 13, rgb(160, 100, 20)); gfx.drawPixel(x + 13, y + 13, rgb(160, 100, 20));
      gDrawDrop(x + 8, y + 6 + ((gAnim / 10) % 2));
    } break;
    case 'U':
      gfx.fillRect(x, y, G_T, G_T, rgb(150, 112, 72));
      gfx.drawRect(x, y, G_T, G_T, rgb(100, 70, 40));
      gfx.drawPixel(x + 2, y + 2, rgb(100, 70, 40)); gfx.drawPixel(x + 13, y + 13, rgb(100, 70, 40));
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
      gfx.fillRect(x + 1, y + 1, 15, 2, soil); gfx.drawFastVLine(x - 2, y, 7, potDark);
      break;
    case 'Y':
      gfx.fillRect(x, y + 7, G_T, 9, potBody); gfx.fillRect(x + 11, y + 7, 5, 9, potSh);
      gfx.fillRect(x, y, 18, 7, potRim); gfx.drawFastHLine(x, y + 6, 18, potDark);
      gfx.fillRect(x, y + 1, 15, 2, soil); gfx.drawFastVLine(x + 17, y, 7, potDark);
      break;
    case 'P': {                                   // boca de maceta-portal (izquierda)
      uint16_t rimP = rgb(90, 160, 90), darkP = rgb(40, 100, 50);
      gfx.fillRect(x, y + 7, G_T, 9, potBody); gfx.fillRect(x + 3, y + 7, 3, 9, potHi);
      gfx.fillRect(x - 2, y, 18, 7, rimP); gfx.drawFastHLine(x - 2, y + 6, 18, darkP);
      gfx.fillRect(x + 1, y + 1, 15, 3, rgb(20, 14, 10)); gfx.drawFastVLine(x - 2, y, 7, darkP);
      if ((gAnim / 8) % 2) gfx.fillTriangle(x + 11, y - 10, x + 21, y - 10, x + 16, y - 4, rgb(255, 230, 90));
    } break;
    case 'Q': {                                   // boca de maceta-portal (derecha)
      uint16_t rimP = rgb(90, 160, 90), darkP = rgb(40, 100, 50);
      gfx.fillRect(x, y + 7, G_T, 9, potBody); gfx.fillRect(x + 11, y + 7, 5, 9, potSh);
      gfx.fillRect(x, y, 18, 7, rimP); gfx.drawFastHLine(x, y + 6, 18, darkP);
      gfx.fillRect(x, y + 1, 15, 3, rgb(20, 14, 10)); gfx.drawFastVLine(x + 17, y, 7, darkP);
    } break;
    case 'R': {                                   // tierra con raíces (subsuelo)
      uint8_t hsh = (c * 5 + r * 11) & 7;
      uint16_t root = rgb(150, 112, 70);
      gfx.fillRect(x, y, G_T, G_T, rgb(64, 42, 28));
      gfx.drawLine(x, y + 4 + (hsh & 3), x + 8, y + 7, root);
      gfx.drawLine(x + 8, y + 7, x + 15, y + 3 + (hsh & 3), root);
      gfx.drawLine(x + 5, y + 7, x + 3 + (hsh & 1) * 4, y + 14, root);
      gfx.fillRect(x + 10, y + 11, 2, 2, rgb(96, 70, 48));
      gfx.drawPixel(x + 2 + hsh, y + 12, rgb(120, 92, 66));
    } break;
    case 't': gfx.fillRect(x, y, G_T, G_T, potBody); gfx.fillRect(x + 3, y, 3, G_T, potHi); break;
    case 'y': gfx.fillRect(x, y, G_T, G_T, potBody); gfx.fillRect(x + 11, y, 5, G_T, potSh); break;
    case 'o': gDrawDrop(x + 8, y + 7 + (((gAnim + c * 3) / 6) % 2)); break;
  }
}

void gDrawBackground() {
  if (gArea == 1) {                                                 // subsuelo: cueva de raíces
    gfx.fillRect(0, G_HUD, G_VW, G_ROWS * G_T, rgb(26, 18, 14));
    for (int i = 0; i < 9; i++) {                                   // raíces colgantes (parallax)
      int rx = ((i * 83 - (int)(gCam * 0.5f)) % 760 + 760) % 760 - 20;
      int len = 40 + (i * 29) % 70;
      uint16_t rc = rgb(58, 40, 28);
      for (int k = 0; k < len; k += 6) {
        int ox = (int)(sinf((k + i * 13) * 0.15f) * 3);
        gfx.drawLine(rx + ox, G_HUD + 16 + k, rx + (int)(sinf((k + 6 + i * 13) * 0.15f) * 3), G_HUD + 22 + k, rc);
      }
      gfx.fillCircle(rx, G_HUD + 16 + len, 1, rgb(90, 66, 44));
    }
    for (int i = 0; i < 12; i++) {                                  // brillitos en la tierra
      int px = ((i * 57 - (int)(gCam * 0.5f)) % 400 + 400) % 400 - 40;
      if ((gAnim / 10 + i) % 3 == 0) gfx.drawPixel(px, G_HUD + 30 + (i * 23) % 120, rgb(120, 200, 255));
    }
    return;
  }
  const uint16_t sky[4] = { rgb(110, 190, 240), rgb(135, 205, 245), rgb(165, 220, 248), rgb(195, 232, 250) };
  for (int i = 0; i < 4; i++) gfx.fillRect(0, G_HUD + i * 48, G_VW, 48, sky[i]);
  for (int i = 0; i < 6; i++) {                                     // nubes (lejanas, se mueven lento)
    int cx = ((i * 170 + 40 - (int)(gCam * 0.3f)) % 1020 + 1020) % 1020 - 60;
    int cy = G_HUD + 22 + (i * 37) % 44;
    gfx.fillEllipse(cx, cy, 22, 8, G_WHITE);
    gfx.fillEllipse(cx + 12, cy - 6, 13, 8, G_WHITE);
    gfx.fillEllipse(cx - 11, cy - 3, 11, 6, G_WHITE);
  }
  int gy = G_HUD + 10 * G_T;
  for (int i = 0; i < 5; i++) {                                     // colinas
    int hx = ((i * 230 - (int)(gCam * 0.5f)) % 1150 + 1150) % 1150 - 100;
    gfx.fillEllipse(hx, gy + 6, 90, 46 + (i % 2) * 16, rgb(120, 190, 110));
    gfx.fillEllipse(hx + 80, gy + 6, 50, 28, rgb(100, 170, 95));
  }
  // Invernadero al final del nivel (decorado)
  int ix = 166 * G_T - (int)gCam;
  if (ix > -110 && ix < G_VW + 10) {
    uint16_t glass = rgb(205, 236, 246), frame = rgb(240, 248, 250);
    gfx.fillRect(ix, gy - 56, 96, 56, glass);
    gfx.fillTriangle(ix - 6, gy - 56, ix + 102, gy - 56, ix + 48, gy - 86, glass);
    gfx.drawRect(ix, gy - 56, 96, 56, frame);
    for (int k = 24; k < 96; k += 24) gfx.drawFastVLine(ix + k, gy - 56, 56, frame);
    gfx.drawLine(ix - 6, gy - 56, ix + 48, gy - 86, frame); gfx.drawLine(ix + 102, gy - 56, ix + 48, gy - 86, frame);
    gfx.fillRect(ix + 38, gy - 30, 20, 30, rgb(120, 170, 120));
  }
}

void gDrawCheckpoint(int x, int gy, bool ok) {
  if (x < -20 || x > G_VW + 20) return;
  gfx.fillRect(x, gy - 40, 2, 40, rgb(90, 90, 100));
  gfx.fillTriangle(x + 2, gy - 40, x + 2, gy - 28, x + 15, gy - 34, ok ? rgb(80, 200, 80) : rgb(150, 150, 150));
  gfx.fillCircle(x + 1, gy - 41, 2, rgb(230, 230, 230));
}

void gDrawGoal(int x, int gy) {
  if (x < -60 || x > G_VW + 60) return;
  int top = gy - 150;
  uint16_t ray = ((gAnim / 6) % 2) ? rgb(255, 240, 160) : rgb(255, 222, 110);
  for (int k = -2; k <= 2; k++) gfx.drawLine(x - 20, top + 16, x - 20 + k * 12, top + 46, ray);
  gfx.fillRect(x - 1, top, 3, gy - top, rgb(120, 120, 130));
  gfx.fillRect(x - 6, gy - 4, 13, 4, rgb(90, 90, 100));
  gfx.fillRect(x - 20, top, 22, 3, rgb(120, 120, 130));
  gfx.fillTriangle(x - 34, top + 12, x - 6, top + 12, x - 20, top + 2, rgb(60, 60, 70));
  gfx.fillRect(x - 32, top + 12, 24, 3, rgb(255, 230, 120));
}

void gDrawHUD() {
  char b[20];
  gfx.fillRect(0, 0, G_VW, G_HUD, rgb(24, 32, 40));
  gfx.fillTriangle(4, 11, 12, 11, 8, 2, rgb(86, 160, 66)); gfx.fillCircle(8, 10, 4, rgb(86, 160, 66));   // cogollo
  gfx.drawPixel(6, 6, rgb(232, 128, 44)); gfx.drawPixel(10, 8, rgb(232, 128, 44));
  if (gFire) gfx.fillCircle(3, 3, 2, rgb(255, 214, 60));
  snprintf(b, sizeof b, "x%d", gLives);       gText(b, 15, 8, G_WHITE, &fonts::Font0, textdatum_t::middle_left);
  gDrawDrop(50, 8);
  snprintf(b, sizeof b, "x%02d", gDrops);     gText(b, 56, 8, G_WHITE, &fonts::Font0, textdatum_t::middle_left);
  snprintf(b, sizeof b, "PTS %06d", gScore);  gText(b, 96, 8, G_WHITE, &fonts::Font0, textdatum_t::middle_left);
  snprintf(b, sizeof b, "TIEMPO %03d", gTime); gText(b, 186, 8, gTime < 60 ? C_HIGH : G_WHITE, &fonts::Font0, textdatum_t::middle_left);
  gfx.fillRect(298, 4, 3, 9, G_WHITE); gfx.fillRect(304, 4, 3, 9, G_WHITE);   // pausa
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
  if (gState == G_TITLE || gState == G_OVER) return;
  bool blink = gInv && ((gInv / 3) % 2);
  if (blink) return;
  uint16_t a = (fabsf(gp.vx) > 0.3f && gp.ground) ? gAnim + 1 : 0;
  gDrawBud((int)gp.x - cam + gp.w / 2, G_HUD + (int)(gp.y + gp.h), 1, gBig, gFace, a, gState == G_DYING, gFire);
}

void gRender() {
  int cam = (int)gCam;
  gDrawBackground();
  int gy = G_HUD + 10 * G_T;
  if (gArea == 0) {
    gDrawCheckpoint(gCheckX - cam, gy, gCheckOk);
    gDrawGoal(gGoalX - cam, gy);
  }

  if (gState == G_WARP) gDrawPlayer(cam);           // dentro de la maceta: queda detrás

  int c0 = cam / G_T;
  for (int c = max(0, c0 - 1); c <= c0 + G_VW / G_T + 1 && c < gW; c++) {
    int sx = c * G_T - cam;
    for (int r = 0; r < G_ROWS; r++) {
      char ch = gMap[r][c];
      if (ch == '.') continue;
      int sy = G_HUD + r * G_T;
      if (gBumpT && c == gBumpC && r == gBumpR) sy -= (gBumpT > 4) ? (8 - gBumpT) : gBumpT;
      gDrawTile(ch, sx, sy, c, r);
    }
  }

  for (auto& it : gIt) {
    if (!it.active) continue;
    if (it.kind == 1) gDrawSunItem((int)it.b.x - cam, G_HUD + (int)it.b.y);
    else gDrawLeafItem((int)it.b.x - cam, G_HUD + (int)it.b.y);
  }
  for (int i = 0; i < gEnN; i++) {
    GEnemy& e = gEn[i];
    if (!e.active || !e.awake) continue;
    gDrawMite((int)e.b.x - cam, G_HUD + (int)e.b.y, e.b.w, e.b.h, e.state, gAnim, e.dir);
  }
  for (auto& s : gShot) if (s.active) gDrawShot((int)s.x - cam, G_HUD + (int)s.y);

  if (gState != G_WARP) gDrawPlayer(cam);

  char b[24];
  for (auto& p : gPop) {
    if (!p.active) continue;
    int px = (int)p.x - cam, py = G_HUD + (int)p.y;
    if (p.kind == 0) {
      if (p.val == -1) snprintf(b, sizeof b, "+1 VIDA");
      else if (p.val == -2) snprintf(b, sizeof b, "CONTROL");
      else snprintf(b, sizeof b, "%d", p.val);
      gText(b, px + 1, py + 1, G_BLACK, &fonts::Font0, textdatum_t::top_left);
      gText(b, px, py, G_WHITE, &fonts::Font0, textdatum_t::top_left);
    } else if (p.kind == 1) gDrawDrop(px, py);
    else if (p.kind == 3) {                       // destello de bolita de luz
      if (p.timer > 20) {
        int rr = (31 - p.timer);
        gfx.drawCircle(px + 3, py + 3, rr, rgb(255, 200, 60));
        gfx.drawCircle(px + 3, py + 3, rr / 2, rgb(255, 240, 160));
      }
    }
    else gfx.fillRect(px, py, 4, 4, rgb(188, 96, 58));
  }

  gDrawHUD();

  // Pantallas superpuestas
  if (gState == G_TITLE) {
    gfx.fillRect(0, G_HUD, G_VW, 70, rgb(24, 32, 40));
    gText("COGOLLO", 162, G_HUD + 30, rgb(30, 60, 30), &fonts::FreeSansBold24pt7b, textdatum_t::middle_center);
    gText("COGOLLO", 160, G_HUD + 28, rgb(120, 220, 90), &fonts::FreeSansBold24pt7b, textdatum_t::middle_center);
    gText("Aventura en el indoor", 160, G_HUD + 58, rgb(200, 210, 220), &fonts::Font2, textdatum_t::middle_center);
    gDrawBud(40, G_HUD + 10 * G_T, 3, true, 1, 0, false, false);
    gDrawMite(250, G_HUD + 10 * G_T - 12, 14, 12, 0, gAnim, -1);
    gDrawMite(278, G_HUD + 10 * G_T - 12, 14, 12, 0, gAnim + 2, -1);
    if ((gAnim / 15) % 2) gText("Toca para jugar", 168, 110, G_WHITE, &fonts::Font4, textdatum_t::middle_center);
    snprintf(b, sizeof b, "Record: %d", gBest);
    gText(b, 12, 189, G_WHITE, &fonts::Font2, textdatum_t::middle_left);
    gButton(gExitBtn, "Salir");
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
    gText("LLEGASTE A LA LUZ!", 160, 76, rgb(255, 225, 110), &fonts::Font4, textdatum_t::middle_center);
    snprintf(b, sizeof b, "Puntaje: %d", gScore);  gText(b, 160, 104, G_WHITE, &fonts::Font2, textdatum_t::middle_center);
    snprintf(b, sizeof b, "Record: %d", gBest);    gText(b, 160, 124, rgb(200, 210, 220), &fonts::Font2, textdatum_t::middle_center);
    if (gStateT > 90) gText("Toca para continuar", 160, 145, C_OK, &fonts::Font0, textdatum_t::middle_center);
  }

  gfx.pushSprite(0, 0);
}

// Controles táctiles (se dibujan directo en la pantalla, bajo el juego)
// 7 botones: ◀ ▶ ▼ | ↖ ▲ ↗ ●   (bits: 1 izq, 2 der, 4 saltar, 8 abajo, 16 disparar)
const int     G_BX[7] = { 2, 47, 92, 141, 186, 231, 276 };
const uint8_t G_BM[7] = { 1, 2, 8, 1 | 4, 4, 2 | 4, 16 };
#define G_BW 43

uint8_t gCtrlKey() { return gBtn | (gFire ? 0x80 : 0); }

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
        uint16_t fc = on ? C_BG : (gFire ? rgb(255, 200, 50) : C_GRID);
        lcd.fillCircle(cx, cy, 7, fc);
        if (gFire && !on) lcd.fillCircle(cx - 2, cy - 2, 2, rgb(255, 245, 190));
      } break;
    }
  }
}

// Un cuadro del juego (~30 por segundo)
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
        gNewGame();
      }
      break;
    case G_PLAY:
      if (tap && ty < G_HUD + 10 && tx > 280) { gState = G_PAUSE; break; }
      gUpdatePlay();
      break;
    case G_PAUSE:
      if (tap) {
        if (inBtn(gResumeBtn, tx, ty)) gState = G_PLAY;
        else if (inBtn(gQuitBtn, tx, ty)) { gSaveBest(); gBuildLevel(); gCam = 0; gState = G_TITLE; }
      }
      break;
    case G_DYING: gUpdateDying(); break;
    case G_WARP:  gUpdateWarp();  break;
    case G_WIN:
      gUpdateWin();
      if (gStateT > 90 && tap && ty < G_CTRL_Y) { gBuildLevel(); gCam = 0; gState = G_TITLE; }
      break;
    case G_OVER:
      gStateT++;
      if (gStateT > 30 && tap && ty < G_CTRL_Y) { gBuildLevel(); gCam = 0; gState = G_TITLE; }
      break;
  }
  if (gBumpT) gBumpT--;
  gAnim++;
  gUpdatePopups();
  gRender();
}

void startGame() {
  gfx.setColorDepth(16);
  if (!gfx.createSprite(G_VW, G_VH)) {          // si no alcanza la memoria, usa 8 bits de color
    gfx.setColorDepth(8);
    if (!gfx.createSprite(G_VW, G_VH)) return;
  }
  gameMode = true;
  gBest = prefs.getInt("gbest", 0);
  gBuildLevel();
  gCam = 0; gState = G_TITLE; gStateT = 0; gAnim = 0;
  gBtn = gPrevBtn = 0; gCtrlDrawn = 0xFF;
  gTouchPrev = true;                              // ignora el toque que abrió el juego
  lcd.fillScreen(C_BG);
  gDrawControls();
  gLastFrame = millis();
}

void exitGame() {
  gSaveBest();
  gfx.deleteSprite();
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
  lcd.fillScreen(C_BG);

  if (prefs.getBytesLength("tcal") == 16) {
    uint16_t cal[8]; prefs.getBytes("tcal", cal, sizeof cal);
    lcd.setTouchCalibrate(cal);
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
    if (!screenOn) screenWake();          // el primer toque solo enciende la pantalla
    else { lastActivity = ms; handleTouch(tx, ty); }
  }
  wasDown = down;

  delay(10);
}
