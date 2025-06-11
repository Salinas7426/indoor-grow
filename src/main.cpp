#include <Arduino.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include "DHT.h"

// ————————————————
//   CONFIGURACIÓN LCD I²C
// ————————————————
#define LCD_ADDR     0x27  // o 0x3F según tu módulo
#define LCD_COLUMNS  16
#define LCD_ROWS      2
LiquidCrystal_I2C lcd(LCD_ADDR, LCD_COLUMNS, LCD_ROWS);

// Define tus umbrales según calibración
const int SOIL_DRY_THRESHOLD    = 2600;
const int SOIL_MOIST_THRESHOLD  = 1500;

// Umbrales de lectura analógica (0–4095 en ESP32)
const int MQ135_EXCELLENT = 1000;   // Aire muy limpio
const int MQ135_GOOD      = 2000;   // Aire aceptable
const int MQ135_MODERATE  = 2200;   // Moderada contaminación
const int MQ135_POOR      = 2800;   // Alta contaminación
// >3500 → Peligroso

// ————————————————
//   PINES SENSORES/ACTUADORES
// ————————————————
#define VentPinOut    32
#define VentPinIn     27
#define oneWireBus    25
#define DHTPIN        33
#define LampContPin   13
#define CO2Pin        14
#define HumGroundPin  26
#define Alerta        17

#define DHTTYPE       DHT11

DHT dht(DHTPIN, DHTTYPE);
OneWire oneWire(oneWireBus);
String getSoilStatus(int raw);
DallasTemperature sensors(&oneWire);

// Prototipo helper para el LCD
void lampLoop(void *pvParams);
String getAirQualityStatus(int raw);
void displaySensor(const char* label, const String& value);

void setup() {
  // Inicializa serial
  Serial.begin(115200);

  // Inicializa I²C (bus compartido para LCD y otros dispositivos I²C)
  Wire.begin(21, 22);  // SDA = GPIO21, SCL = GPIO22

  xTaskCreatePinnedToCore(lampLoop, "Lamp Monitor", 16384, NULL, 1, NULL, 0);


  // Inicializa LCD
  lcd.init();
  lcd.backlight();

  // Pines de actuadores
  pinMode(Alerta, OUTPUT);
  pinMode(VentPinIn, OUTPUT);
  pinMode(VentPinOut, OUTPUT);
  ledcSetup(0, 5000, 8);
  ledcAttachPin(LampContPin, 0);

  // Inicializa sensores
  dht.begin();
  sensors.begin();
  pinMode(CO2Pin, INPUT);
}

void loop() {
  // — Lecturas —
  float humedad    = dht.readHumidity();
  float temperatura= dht.readTemperature();
  sensors.requestTemperatures();
  float tempSuelo  = sensors.getTempCByIndex(0);
  int   gasState   = analogRead(CO2Pin);
  int   humSuelo   = analogRead(HumGroundPin);

  // — Alarma —
  if (!isnan(humedad) && !isnan(temperatura) && (humedad > 80.0 || temperatura > 24.0)) {
    digitalWrite(Alerta, HIGH);
  } else {
    digitalWrite(Alerta, LOW);
  }

  // — Ventilador de ejemplo —
  if (temperatura > 20.0 && humedad > 70.0) {
    digitalWrite(VentPinOut, HIGH);
    digitalWrite(VentPinIn, HIGH);
  } else {
    digitalWrite(VentPinOut, LOW);
    digitalWrite(VentPinIn, LOW);
  }



  // — Despliega valores uno a uno en LCD —
  displaySensor("Humedad:",   isnan(humedad)    ? "ERR" : String(humedad,1)   + "%");
  displaySensor("Temp Aire:", isnan(temperatura)? "ERR" : String(temperatura,1)+"C");
  displaySensor("Temp Suel:", isnan(tempSuelo)   ? "ERR" : String(tempSuelo,1)  + "C");
  displaySensor("Gas CO2:",  getAirQualityStatus(gasState));
  displaySensor("Hum Suel:",  getSoilStatus(humSuelo));

  // Repite ciclo
}

// — Helper para mostrar una etiqueta y su valor en la LCD — 
void displaySensor(const char* label, const String& value) {
  Serial.print(label);
  Serial.println(value);
  
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print(label);
  lcd.setCursor(0, 1);
  lcd.print(value);
  delay(2000);
}

void lampLoop(void *pvParams){
    // — Secuencia PWM lámpara —
  for (int i = 0; i <= 18; i++) {
    ledcWrite(0, i);
    vTaskDelay(3000/portTICK_PERIOD_MS);
  }
  for (int i = 14; i >= 0; i--) {
    ledcWrite(0, i);
    vTaskDelay(3000/portTICK_PERIOD_MS);
  }
}

// Función que devuelve "Dry", "Moist" o "Wet" según la lectura
String getSoilStatus(int raw) {
  Serial.println(raw);
  if (raw < SOIL_MOIST_THRESHOLD) {
    return "Wet";
  } else if (raw < SOIL_DRY_THRESHOLD) {
    return "Moist";
  } else {
    return "Dry";
  }
}

String getAirQualityStatus(int raw) {
  // int raw = analogRead(pin);         // 0–4095 en ESP32
    Serial.println(raw);

  if (raw < MQ135_EXCELLENT) {
    return "Excelente";
  } 
  else if (raw < MQ135_GOOD) {
    return "Buena";
  } 
  else if (raw < MQ135_MODERATE) {
    return "Moderada";
  } 
  else if (raw < MQ135_POOR) {
    return "Mala";
  } 
  else {
    return "Peligrosa";
  }
}