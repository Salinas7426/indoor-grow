#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <DHT.h>
#include <OneWire.h>
#include <DallasTemperature.h>

// ————————————————
//   CONFIGURACIÓN OLED hghjgjhkgkj
// ————————————————
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 32
#define OLED_RESET    -1
#define OLED_ADDR     0x3C
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

// ————————————————
//   PINES SENSORES/ACTUADORES
// ————————————————
#define DHTPIN        33
#define DHTTYPE       DHT11
#define ONEWIRE_PIN   25
#define CO2Pin        14
#define HumGroundPin  26
#define Alerta        17
#define VentPinOut    32
#define VentPinIn     35
#define LampContPin   13

DHT dht(DHTPIN, DHTTYPE);
OneWire oneWire(ONEWIRE_PIN);
DallasTemperature sensors(&oneWire);

// ————————————————
//   PROTOTIPOS
// ————————————————
void mostrarEnOLED(const char* etiqueta, const String& valor);

void setup() {
  Serial.begin(115200);

  // Inicializar I²C en SDA=21, SCL=22
  Wire.begin(21, 22);

  // Iniciar OLED
  if (!display.begin(SSD1306_SWITCHCAPVCC, OLED_ADDR)) {
    Serial.println("ERROR: OLED no encontrado");
    while (true);
  }
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);

  // Sensores
  dht.begin();
  sensors.begin();

  // Pines
  pinMode(CO2Pin, INPUT);
  pinMode(HumGroundPin, INPUT);
  pinMode(Alerta, OUTPUT);
  pinMode(VentPinOut, OUTPUT);
  pinMode(VentPinIn, OUTPUT);
  ledcSetup(0, 5000, 8);         // PWM lámpara
  ledcAttachPin(LampContPin, 0);
}

void loop() {
  // Lecturas
  float h       = dht.readHumidity();
  float t       = dht.readTemperature();
  sensors.requestTemperatures();
  float tSoil   = sensors.getTempCByIndex(0);
  int   gas     = digitalRead(CO2Pin);
  int   humSoil = analogRead(HumGroundPin);

  // Control alarma
  if (!isnan(h) && !isnan(t) && (h > 80.0f || t > 24.0f)) {
    digitalWrite(Alerta, HIGH);
  } else {
    digitalWrite(Alerta, LOW);
  }

  // Control ventilador (ejemplo)
  if (t > 20.0f && h > 70.0f) {
    digitalWrite(VentPinOut, HIGH);
    digitalWrite(VentPinIn, HIGH);
  } else {
    digitalWrite(VentPinOut, LOW);
    digitalWrite(VentPinIn, LOW);
  }

  // Secuencia PWM lámpara (puedes ajustar o quitar)
  for (int i = 0; i <= 17; i++) {
    ledcWrite(0, i);
    delay(100);
  }

  // Despliega valores uno a uno
  mostrarEnOLED("Humedad:",   isnan(h)      ? "ERR" : String(h,1)+"%");
  mostrarEnOLED("Temp Aire:", isnan(t)      ? "ERR" : String(t,1)+"C");
  mostrarEnOLED("Temp Suel:", tSoil < -50   ? "ERR" : String(tSoil,1)+"C");
  mostrarEnOLED("Gas CO2:",   gas==LOW      ? "Detectado" : "OK");
  mostrarEnOLED("Hum Suelo:", String(humSoil));
}

// ————————————————
//   DEFINICIÓN mostrarEnOLED
// ————————————————
void mostrarEnOLED(const char* etiqueta, const String& valor) {
  display.clearDisplay();
  display.setCursor(0, 0);
  display.print(etiqueta);
  display.setCursor(0, 12);
  display.print(valor);
  display.display();
  delay(2000);
}
