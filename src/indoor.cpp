#include <Arduino.h>
#include <FS.h>
#include <SPIFFS.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include "DHT.h"

// --- Pines de hardware ---
#define VentPinOut    32   // Pin para ventilador (outs/entrada de aire)
#define VentPinIn     35   // Pin para ventilador (return/salida de aire)
#define oneWireBus    25   // Pin para DS18B20
#define DHTPIN        33   // Pin para DHT11
#define LampContPin   13   // Pin PWM para lámpara
#define CO2Pin        14   // Pin para sensor de CO2/gas (digital)
#define HumGroundPin  26   // Pin analógico para sensor de humedad de suelo
#define Alerta        17   // Pin para la alarma (LED o buzzer)

// --- Tipo de DHT ---
#define DHTTYPE       DHT11

// Instancias de sensores
DHT dht(DHTPIN, DHTTYPE);
OneWire oneWire(oneWireBus);
DallasTemperature sensors(&oneWire);

// ——— Prototipos de funciones ———
float GroTemMes(void);
void  AmTeHumMes(float &humedad, float &temperatura, float &sensacionT);
int   CO2Mes(void);
int   HumGroundMes(void);
void  ControlVent(float hum, float temp, float Stemp, float CO2);
void  OnOffLampFunction(void);
String TimelapseOn(void);


void setup() {
  // Configurar pines de salida
  pinMode(Alerta, OUTPUT);
  pinMode(VentPinIn, OUTPUT);
  pinMode(VentPinOut, OUTPUT);
  pinMode(LampContPin, OUTPUT);

  Serial.begin(115200);

  // Inicializar sensores
  sensors.begin();            // DS18B20
  Serial.println(F("DHT11 test!"));
  dht.begin();                // DHT11

  // Configurar PWM para lámpara (canal 0, 5 kHz, 8 bits)
  ledcSetup(0, 5000, 8);
  ledcAttachPin(LampContPin, 0);
}

void loop() {
  // Variables para lecturas
  float Humedad = 0, Temperatura = 0, SensacionTermica = 0, TemperaturaSuelo = 0;
  int estadoGas = 0;
  int HumGround = 0;

  // 1) Leer estado del sensor de CO2/gas
  estadoGas = CO2Mes();

  // 2) Leer DHT (humedad, temperatura, sensación térmica)
  AmTeHumMes(Humedad, Temperatura, SensacionTermica);

  // 3) Activar alarma si humedad > 68% Y temperatura > 24°C
  if (Humedad > 80.0f || Temperatura > 24.0f) {
    digitalWrite(Alerta, HIGH);
    Serial.println(" ALERTA: Humedad > 68% y Temperatura > 24°C");
  } else {
    digitalWrite(Alerta, LOW);
  }

  // 4) Leer DS18B20 (temperatura de suelo)
  TemperaturaSuelo = GroTemMes();

  // 5) Leer sensor de humedad de suelo (analógico)
  HumGround = HumGroundMes();

  // 6) Control de lámpara (ejemplo de PWM secuencial)
  OnOffLampFunction();

  // 7) Pausa antes de la siguiente iteración (2 segundos)
  delay(2000);
}


// ================= Funciones de lectura =================

// Lectura del sensor DS18B20 (temperatura de suelo)
float GroTemMes(void) {
  sensors.requestTemperatures();
  int no_sensors = sensors.getDeviceCount();
  Serial.print("Número de sensores DS18B20: ");
  Serial.println(no_sensors);

  float temp = sensors.getTempCByIndex(0);
  Serial.print("Temperatura (suelo): ");
  Serial.print(temp);
  Serial.println(" °C");
  return temp;
}

// Lectura del sensor DHT11 (humedad, temperatura, sensación térmica)
void AmTeHumMes(float &humedad, float &temperatura, float &sensacionT) {
  float h = dht.readHumidity();
  float t = dht.readTemperature();

  if (isnan(h) || isnan(t)) {
    Serial.println(F("¡Error al leer DHT11!"));
    return;
  }

  float hic = dht.computeHeatIndex(t, h, false);

  Serial.print(F("Humedad: "));
  Serial.print(h);
  Serial.print(F("%  Temperatura: "));
  Serial.print(t);
  Serial.print(F(" °C  Sensación térmica: "));
  Serial.print(hic);
  Serial.println(F(" °C"));

  humedad     = h;
  temperatura = t;
  sensacionT  = hic;
}

// Lectura del sensor digital de CO2/gas
int CO2Mes(void) {
  int estadoGas = digitalRead(CO2Pin);
  Serial.print("Estado sensor de gas: ");
  Serial.println(estadoGas == HIGH ? "No hay gas" : "Gas detectado");
  delay(1000);
  return estadoGas;
}

// Lectura del sensor analógico de humedad de suelo
int HumGroundMes(void) {
  int valor_analogico = analogRead(HumGroundPin);
  Serial.print("Humedad de suelo (valor analógico): ");
  Serial.println(valor_analogico);
  delay(1000);
  return valor_analogico;
}


// ================= Funciones de actuadores / utilidad =================

// Ejemplo de control de ventilador según temperatura y humedad
void ControlVent(float hum, float temp, float Stemp, float CO2) {
  // Si temp>20°C Y hum>70%, enciende ventilador
  if (temp > 20.0f && hum > 70.0f) {
    digitalWrite(VentPinOut, HIGH);
    digitalWrite(VentPinIn, HIGH);
    Serial.println("Ventilador ON");
  } else {
    digitalWrite(VentPinOut, LOW);
    digitalWrite(VentPinIn, LOW);
    Serial.println("Ventilador OFF");
  }
}

// Ejemplo de secuencia PWM para lámpara (0–17 en incrementos)
void OnOffLampFunction(void) {
  for (int i = 0; i <= 17; i++) {
    ledcWrite(0, i);   // Canal 0 aumenta PWM
    Serial.print("PWM lámpara: ");
    Serial.println(i);
    delay(5000);        // Espera 5 segundos entre cada paso
  }
}

// Función timelapse (no usada directamente en loop)
String TimelapseOn(void) {
  unsigned long ms = millis();
  unsigned long segundos = ms / 1000;
  unsigned long minutos  = segundos / 60;
  unsigned long horas    = minutos / 60;

  segundos = segundos % 60;
  minutos  = minutos % 60;

  char buffer[30];
  snprintf(buffer, sizeof(buffer), "%02lu:%02lu:%02lu",
           horas, minutos, segundos);
  return String(buffer);
}

