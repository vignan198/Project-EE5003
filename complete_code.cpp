#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_ILI9341.h>
#include <DHT.h>
#include <RTClib.h>

#define TFT_DC 17
#define TFT_CS 16
#define TFT_RST 2
#define TFT_MOSI 23
#define TFT_SCK 18

Adafruit_ILI9341 tft = Adafruit_ILI9341(TFT_CS, TFT_DC, TFT_RST);

// DHT22 sensor
#define DHTPIN 4
#define DHTTYPE DHT22
DHT dht(DHTPIN, DHTTYPE);

// Define pins for fan, heater, humidifier, dehumidifier, bulb, LDR, potentiometer, and buzzer
const int fanPin = 25;
const int heaterPin = 26;
const int humidifierPin = 27;
const int dehumidifierPin = 14;
const int bulbPin = 12;
const int ldrPin = 34;
const int potPin = 35;  // Potentiometer pin
const int buzzerPin = 32;  // Buzzer pin

// Desired temperature and humidity settings
const float desiredTemperature = 24.0;
const float temperatureRange = 2.0;
const float desiredHumidity = 50.0;
const float humidityRange = 10.0;

// LDR thresholds for controlling the bulb
const int ldrThreshold = 2000;

// Noise threshold for controlling the buzzer
const int noiseThreshold = 2000;

unsigned long previousMillis = 0;
const long updateInterval = 10000;

float lastTemperature = -999;
float lastHumidity = -999;
int lastLdrValue = -1;
int lastFanState = -1;
int lastHeaterState = -1;
int lastHumidifierState = -1;
int lastDehumidifierState = -1;
int lastBulbState = -1;
int lastNoiseLevel = -1;
int lastBuzzerState = -1;

RTC_DS1307 rtc;

void setup() {
  pinMode(fanPin, OUTPUT);
  pinMode(heaterPin, OUTPUT);
  pinMode(humidifierPin, OUTPUT);
  pinMode(dehumidifierPin, OUTPUT);
  pinMode(bulbPin, OUTPUT);
  pinMode(ldrPin, INPUT);
  pinMode(potPin, INPUT);
  pinMode(buzzerPin, OUTPUT);

  Serial.begin(115200);

  tft.begin();
  tft.setRotation(0);
  tft.fillScreen(ILI9341_BLACK);

  dht.begin();

  if (!rtc.begin()) {
    Serial.println(F("Couldn't find RTC"));
    while (1);
  }

  if (!rtc.isrunning()) {
    Serial.println(F("RTC is NOT running, let's set the time!"));
    rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
  }
}

void loop() {
  unsigned long currentMillis = millis();

  float temperature = dht.readTemperature();
  float humidity = dht.readHumidity();

  if (isnan(temperature) || isnan(humidity)) {
    Serial.println(F("Failed to read from DHT sensor!"));
    return;
  }

  int ldrValue = analogRead(ldrPin);
  int noiseLevel = analogRead(potPin);

  controlBulb(ldrValue);
  controlTemperature(temperature);
  controlHumidity(humidity);
  controlNoise(noiseLevel);

  if (shouldUpdateDisplay(temperature, humidity, ldrValue, noiseLevel)) {
    if (currentMillis - previousMillis >= updateInterval) {
      previousMillis = currentMillis;
      updateDisplay(temperature, humidity, ldrValue, noiseLevel);
    }
  }
}

void controlBulb(int ldrValue) {
  if (ldrValue < ldrThreshold) {
    digitalWrite(bulbPin, HIGH);
  } else {
    digitalWrite(bulbPin, LOW);
  }
}

void controlTemperature(float temperature) {
  if (temperature > desiredTemperature + temperatureRange) {
    digitalWrite(fanPin, HIGH);
    digitalWrite(heaterPin, LOW);
  } else if (temperature < desiredTemperature - temperatureRange) {
    digitalWrite(fanPin, LOW);
    digitalWrite(heaterPin, HIGH);
  } else {
    digitalWrite(fanPin, LOW);
    digitalWrite(heaterPin, LOW);
  }
}

void controlHumidity(float humidity) {
  if (humidity < desiredHumidity - humidityRange) {
    digitalWrite(humidifierPin, HIGH);
    digitalWrite(dehumidifierPin, LOW);
  } else if (humidity > desiredHumidity + humidityRange) {
    digitalWrite(humidifierPin, LOW);
    digitalWrite(dehumidifierPin, HIGH);
  } else {
    digitalWrite(humidifierPin, LOW);
    digitalWrite(dehumidifierPin, LOW);
  }
}

void controlNoise(int noiseLevel) {
  if (noiseLevel > noiseThreshold) {
    digitalWrite(buzzerPin, HIGH);
  } else {
    digitalWrite(buzzerPin, LOW);
  }
}

bool shouldUpdateDisplay(float temperature, float humidity, int ldrValue, int noiseLevel) {
  bool update = false;
  if (abs(temperature - lastTemperature) > 0.5 ||
      abs(humidity - lastHumidity) > 5 ||
      ldrValue != lastLdrValue ||
      noiseLevel != lastNoiseLevel ||
      digitalRead(fanPin) != lastFanState ||
      digitalRead(heaterPin) != lastHeaterState ||
      digitalRead(humidifierPin) != lastHumidifierState ||
      digitalRead(dehumidifierPin) != lastDehumidifierState ||
      digitalRead(bulbPin) != lastBulbState ||
      digitalRead(buzzerPin) != lastBuzzerState) {
    update = true;
    lastTemperature = temperature;
    lastHumidity = humidity;
    lastLdrValue = ldrValue;
    lastNoiseLevel = noiseLevel;
    lastFanState = digitalRead(fanPin);
    lastHeaterState = digitalRead(heaterPin);
    lastHumidifierState = digitalRead(humidifierPin);
    lastDehumidifierState = digitalRead(dehumidifierPin);
    lastBulbState = digitalRead(bulbPin);
    lastBuzzerState = digitalRead(buzzerPin);
  }
  return update;
}

void updateDisplay(float temperature, float humidity, int ldrValue, int noiseLevel) {
  tft.fillScreen(ILI9341_BLACK);

  tft.setTextSize(2);
  tft.setTextColor(ILI9341_WHITE);

  DateTime now = rtc.now();

  tft.setCursor(10, 10);
  tft.print(now.hour(), DEC);
  tft.print(':');
  tft.print(now.minute(), DEC);
  tft.print(':');
  tft.print(now.second(), DEC);

  tft.setCursor(10, 40);
  tft.print(now.year(), DEC);
  tft.print('/');
  tft.print(now.month(), DEC);
  tft.print('/');
  tft.print(now.day(), DEC);

  tft.setCursor(10, 70);
  tft.print(F("Temp: "));
  tft.print(temperature);
  tft.println(F(" C"));

  tft.setCursor(10, 100);
  tft.print(F("Humid: "));
  tft.print(humidity);
  tft.println(F(" %"));

  tft.setCursor(10, 130);
  tft.print(F("Fan: "));
  tft.println(digitalRead(fanPin) ? F("ON") : F("OFF"));

  tft.setCursor(10, 160);
  tft.print(F("Heater: "));
  tft.println(digitalRead(heaterPin) ? F("ON") : F("OFF"));

  tft.setCursor(10, 190);
  tft.print(F("Humidifier: "));
  tft.println(digitalRead(humidifierPin) ? F("ON") : F("OFF"));

  tft.setCursor(10, 220);
  tft.print(F("Dehumidifier: "));
  tft.println(digitalRead(dehumidifierPin) ? F("ON") : F("OFF"));

  tft.setCursor(10, 250);
  tft.print(F("Bulb: "));
  tft.println(digitalRead(bulbPin) ? F("ON") : F("OFF"));

  tft.setCursor(10, 280);
  tft.print(F("Noise: "));
  tft.println(noiseLevel);

  Serial.print(F("Temp: "));
  Serial.print(temperature);
  Serial.println(F(" C"));

  Serial.print(F("Humid: "));
  Serial.print(humidity);
  Serial.println(F(" %"));

  Serial.print(F("LDR: "));
  Serial.println(ldrValue);

  Serial.print(F("Noise: "));
  Serial.println(noiseLevel);

  Serial.print(F("Fan: "));
  Serial.println(digitalRead(fanPin) ? F("ON") : F("OFF"));

  Serial.print(F("Heater: "));
  Serial.println(digitalRead(heaterPin) ? F("ON") : F("OFF"));

  Serial.print(F("Humidifier: "));
  Serial.println(digitalRead(humidifierPin) ? F("ON") : F("OFF"));

  Serial.print(F("Dehumidifier: "));
  Serial.println(digitalRead(dehumidifierPin) ? F("ON") : F("OFF"));
}
