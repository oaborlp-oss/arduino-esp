#define P_SCK   18
#define P_MOSI  23
#define P_CS     5
#define P_DC     4
#define P_RST   22

void spiByte(uint8_t b) {
  for (int i = 7; i >= 0; i--) {
    digitalWrite(P_MOSI, (b >> i) & 1);
    digitalWrite(P_SCK, HIGH);
    digitalWrite(P_SCK, LOW);
  }
}

void cmd(uint8_t c) {
  digitalWrite(P_DC, LOW);
  digitalWrite(P_CS, LOW);
  spiByte(c);
  digitalWrite(P_CS, HIGH);
}

void dat(uint8_t d) {
  digitalWrite(P_DC, HIGH);
  digitalWrite(P_CS, LOW);
  spiByte(d);
  digitalWrite(P_CS, HIGH);
}

void fill(uint16_t color) {
  cmd(0x2A); dat(0); dat(0); dat(0); dat(239);
  cmd(0x2B); dat(0); dat(0); dat(1); dat(0x3F);
  cmd(0x2C);
  digitalWrite(P_DC, HIGH);
  digitalWrite(P_CS, LOW);
  for (uint32_t i = 0; i < 240UL * 320UL; i++) {
    spiByte(color >> 8);
    spiByte(color & 0xFF);
  }
  digitalWrite(P_CS, HIGH);
}

void setup() {
  Serial.begin(115200);
  delay(2000);
  Serial.println("===== TEST DIRECTO DE PANTALLA (ESP32 clasica) =====");

  pinMode(P_CS, OUTPUT);
  digitalWrite(P_CS, HIGH);
  pinMode(P_DC, OUTPUT);
  pinMode(P_SCK, OUTPUT);
  digitalWrite(P_SCK, LOW);
  pinMode(P_MOSI, OUTPUT);
  pinMode(P_RST, OUTPUT);

  Serial.println("Reset de pantalla...");
  digitalWrite(P_RST, HIGH);
  delay(50);
  digitalWrite(P_RST, LOW);
  delay(50);
  digitalWrite(P_RST, HIGH);
  delay(150);

  cmd(0x01);
  delay(150);
  cmd(0x11);
  delay(150);
  cmd(0x3A);
  dat(0x55);
  cmd(0x36);
  dat(0x48);
  cmd(0x29);
  delay(50);
  Serial.println("Init enviado. Pintando colores...");

  Serial.println("ROJO");
  fill(0xF800);
  Serial.println("VERDE");
  fill(0x07E0);
  Serial.println("AZUL");
  fill(0x001F);
  Serial.println("NEGRO");
  fill(0x0000);
  Serial.println("===== FIN =====");
}

void loop() {
  static uint32_t n = 0;
  Serial.printf("Vivo %lu s\n", (unsigned long)n);
  n++;
  delay(1000);
}
