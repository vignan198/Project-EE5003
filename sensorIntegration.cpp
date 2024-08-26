#include <Wire.h>
#include <DHT.h>
#include <RTClib.h>

// DHT22 sensor
#define DHTPIN 4
#define DHTTYPE DHT22
DHT dht(DHTPIN, DHTTYPE);

// Define pins for fan, heater, humidifier, and dehumidifier
const int fanPin = 25;
const int heaterPin = 26;
const int humidifierPin = 27;
const int dehumidifierPin = 14;

// Desired temperature and humidity settings
const float desiredTemperature = 24.0;
const float temperatureRange = 2.0;
const float desiredHumidity = 50.0;
const float humidityRange = 10.0;

unsigned long previousMillis = 0;
const long updateInterval = 10000;

float lastTemperature = -999;
float lastHumidity = -999;
int lastFanState = -1;
int lastHeaterState = -1;
int lastHumidifierState = -1;
int lastDehumidifierState = -1;

RTC_DS1307 rtc;

void setup() {
  pinMode(fanPin, OUTPUT);
  pinMode(heaterPin, OUTPUT);
  pinMode(humidifierPin, OUTPUT);
  pinMode(dehumidifierPin, OUTPUT);

  Serial.begin(115200);

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

  controlTemperature(temperature);
  controlHumidity(humidity);

  if (shouldLogData(temperature, humidity)) {
    if (currentMillis - previousMillis >= updateInterval) {
      previousMillis = currentMillis;
      logData(temperature, humidity);
    }
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

bool shouldLogData(float temperature, float humidity) {
  bool log = false;
  if (abs(temperature - lastTemperature) > 0.5 ||
      abs(humidity - lastHumidity) > 5 ||
      digitalRead(fanPin) != lastFanState ||
      digitalRead(heaterPin) != lastHeaterState ||
      digitalRead(humidifierPin) != lastHumidifierState ||
      digitalRead(dehumidifierPin) != lastDehumidifierState) {
    log = true;
    lastTemperature = temperature;
    lastHumidity = humidity;
    lastFanState = digitalRead(fanPin);
    lastHeaterState = digitalRead(heaterPin);
    lastHumidifierState = digitalRead(humidifierPin);
    lastDehumidifierState = digitalRead(dehumidifierPin);
  }
  return log;
}

void logData(float temperature, float humidity) {
  DateTime now = rtc.now();

  Serial.print(F("Time: "));
  Serial.print(now.hour(), DEC);
  Serial.print(':');
  Serial.print(now.minute(), DEC);
  Serial.print(':');
  Serial.print(now.second(), DEC);

  Serial.print(F(" Date: "));
  Serial.print(now.year(), DEC);
  Serial.print('/');
  Serial.print(now.month(), DEC);
  Serial.print('/');
  Serial.println(now.day(), DEC);

  Serial.print(F("Temp: "));
  Serial.print(temperature);
  Serial.println(F(" C"));

  Serial.print(F("Humid: "));
  Serial.print(humidity);
  Serial.println(F(" %"));

  Serial.print(F("Fan: "));
  Serial.println(digitalRead(fanPin) ? F("ON") : F("OFF"));

  Serial.print(F("Heater: "));
  Serial.println(digitalRead(heaterPin) ? F("ON") : F("OFF"));

  Serial.print(F("Humidifier: "));
  Serial.println(digitalRead(humidifierPin) ? F("ON") : F("OFF"));

  Serial.print(F("Dehumidifier: "));
  Serial.println(digitalRead(dehumidifierPin) ? F("ON") : F("OFF"));
}
