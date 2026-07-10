/*
  ============================================================
  CognoSpace Sensor Lab — ESP32 Firmware
  ============================================================
  Companion firmware for the CognoSpace Sensor Lab web app.
  Communicates over USB Serial (115200 baud) using line-delimited JSON.

  INCOMING (from browser -> ESP32):
    {"cmd":"setpins","pins":[{"pin":32,"sensor":"dht11"}, ...],
     "dedicated":"none|ultrasonic|display4",
     "dedicatedRoles":{"trig":16,"echo":17} or {"clk":16,"dio":17} or null,
     "oled":true|false}
    {"cmd":"gpio","pin":18,"state":true|false}
    {"cmd":"servo","pin":25,"angle":90}
    {"cmd":"oled_text","text":"Hello"}
    {"cmd":"display_number","value":42}

  OUTGOING (from ESP32 -> browser), one JSON object per line:
    {"pin":32,"value":1}                     -> digital sensors
    {"pin":"32_t","value":25.4}              -> DHT11 temperature (°C)
    {"pin":"32_h","value":60.0}              -> DHT11 humidity (%)
    {"pin":"16/17","value":23.5}             -> ultrasonic distance (cm)

  Libraries required (Arduino Library Manager):
    - ArduinoJson (by Benoit Blanchon)
    - DHT sensor library (by Adafruit) + Adafruit Unified Sensor
    - ESP32Servo
    - Adafruit SSD1306 + Adafruit GFX (for OLED)
    - TM1637Display (for 4-digit display)
  ============================================================
*/

#include <ArduinoJson.h>
#include <DHT.h>
#include <ESP32Servo.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <TM1637Display.h>

/* ---------------- Fixed components ---------------- */
#define LED1_PIN     18
#define LED2_PIN     19
#define BUZZER_PIN   4
#define OLED_SDA     21
#define OLED_SCL     22

#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
Adafruit_SSD1306 oledDisplay(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);

TM1637Display *tm1637 = nullptr;

/* ---------------- Configurable pins ---------------- */
const int CONFIG_PINS[] = {32, 33, 25, 15, 12, 23};
const int NUM_CONFIG_PINS = 6;

String pinSensorType[40];   // indexed by GPIO number, holds sensor name or ""
DHT* dhtInstances[40]     = { nullptr };
Servo* servoInstances[40] = { nullptr };

String dedicatedChoice = "none";  // none | ultrasonic | display4
int dedicatedTrig = 16, dedicatedEcho = 17;
int dedicatedClk  = 16, dedicatedDio  = 17;
bool oledEnabled = false;

unsigned long lastRead = 0;
const unsigned long READ_INTERVAL = 800; // ms between sensor broadcasts

/* ============================================================ */
void setup(){
  Serial.begin(115200);
  delay(300);

  pinMode(LED1_PIN, OUTPUT);
  pinMode(LED2_PIN, OUTPUT);
  pinMode(BUZZER_PIN, OUTPUT);
  digitalWrite(LED1_PIN, LOW);
  digitalWrite(LED2_PIN, LOW);
  digitalWrite(BUZZER_PIN, LOW);

  for(int i=0;i<40;i++) pinSensorType[i] = "";

  Serial.println("{\"status\":\"ESP32 ready\"}");
}

/* ============================================================ */
void loop(){
  readIncomingSerial();

  if(millis() - lastRead >= READ_INTERVAL){
    lastRead = millis();
    readAndBroadcastSensors();
  }
}

/* ============================================================
   Handle incoming JSON commands from the browser
   ============================================================ */
void readIncomingSerial(){
  static String buffer = "";
  while(Serial.available()){
    char c = Serial.read();
    if(c == '\n'){
      if(buffer.length() > 0) handleCommand(buffer);
      buffer = "";
    } else {
      buffer += c;
    }
  }
}

void handleCommand(const String &line){
  StaticJsonDocument<1024> doc;
  DeserializationError err = deserializeJson(doc, line);
  if(err) return;

  const char* cmd = doc["cmd"] | "";

  if(strcmp(cmd, "setpins") == 0){
    applyPinConfig(doc);
  }
  else if(strcmp(cmd, "gpio") == 0){
    int pin = doc["pin"] | -1;
    bool state = doc["state"] | false;
    if(pin >= 0){
      pinMode(pin, OUTPUT);
      digitalWrite(pin, state ? HIGH : LOW);
    }
  }
  else if(strcmp(cmd, "servo") == 0){
    int pin = doc["pin"] | -1;
    int angle = doc["angle"] | 90;
    if(pin >= 0 && servoInstances[pin] != nullptr){
      servoInstances[pin]->write(angle);
    }
  }
  else if(strcmp(cmd, "oled_text") == 0){
    const char* text = doc["text"] | "";
    showOledText(text);
  }
  else if(strcmp(cmd, "display_number") == 0){
    int value = doc["value"] | 0;
    if(tm1637 != nullptr){
      tm1637->showNumberDec(value);
    }
  }
}

/* ============================================================
   Apply new pin configuration sent from Set Pin Mode
   ============================================================ */
void applyPinConfig(JsonDocument &doc){
  // Clean up previous instances
  for(int i=0;i<40;i++){
    if(dhtInstances[i]){ delete dhtInstances[i]; dhtInstances[i] = nullptr; }
    if(servoInstances[i]){ servoInstances[i]->detach(); delete servoInstances[i]; servoInstances[i] = nullptr; }
    pinSensorType[i] = "";
  }

  JsonArray pins = doc["pins"].as<JsonArray>();
  for(JsonObject p : pins){
    int pin = p["pin"] | -1;
    const char* sensor = p["sensor"] | "";
    if(pin < 0 || pin >= 40) continue;
    pinSensorType[pin] = String(sensor);

    if(strcmp(sensor, "dht11") == 0){
      dhtInstances[pin] = new DHT(pin, DHT11);
      dhtInstances[pin]->begin();
    }
    else if(strcmp(sensor, "servo") == 0){
      servoInstances[pin] = new Servo();
      servoInstances[pin]->attach(pin);
      servoInstances[pin]->write(90);
    }
    else if(strcmp(sensor, "ir")==0 || strcmp(sensor,"button")==0 ||
            strcmp(sensor, "rain")==0 || strcmp(sensor,"soil")==0 ||
            strcmp(sensor, "ldr")==0){
      pinMode(pin, INPUT);
    }
  }

  dedicatedChoice = String((const char*)(doc["dedicated"] | "none"));
  if(dedicatedChoice == "ultrasonic"){
    JsonObject roles = doc["dedicatedRoles"];
    dedicatedTrig = roles["trig"] | 16;
    dedicatedEcho = roles["echo"] | 17;
    pinMode(dedicatedTrig, OUTPUT);
    pinMode(dedicatedEcho, INPUT);
  } else if(dedicatedChoice == "display4"){
    JsonObject roles = doc["dedicatedRoles"];
    dedicatedClk = roles["clk"] | 16;
    dedicatedDio = roles["dio"] | 17;
    if(tm1637 != nullptr) delete tm1637;
    tm1637 = new TM1637Display(dedicatedClk, dedicatedDio);
    tm1637->setBrightness(5);
    tm1637->clear();
  }

  oledEnabled = doc["oled"] | false;
  if(oledEnabled){
    Wire.begin(OLED_SDA, OLED_SCL);
    if(oledDisplay.begin(SSD1306_SWITCHCAPVCC, 0x3C)){
      oledDisplay.clearDisplay();
      oledDisplay.setTextColor(SSD1306_WHITE);
      oledDisplay.setTextSize(1);
      oledDisplay.setCursor(0,0);
      oledDisplay.println("CognoSpace Sensor Lab");
      oledDisplay.display();
    }
  }
}

/* ============================================================
   Read all configured sensors and stream JSON back to browser
   ============================================================ */
void readAndBroadcastSensors(){
  StaticJsonDocument<512> doc;
  JsonObject readings = doc.createNestedObject("readings");
  bool any = false;

  for(int i=0;i<40;i++){
    if(pinSensorType[i] == "") continue;
    String sensor = pinSensorType[i];

    if(sensor == "dht11" && dhtInstances[i] != nullptr){
      float t = dhtInstances[i]->readTemperature();
      float h = dhtInstances[i]->readHumidity();
      if(!isnan(t)) { readings[String(i)+"_t"] = t; any = true; }
      if(!isnan(h)) { readings[String(i)+"_h"] = h; any = true; }
    }
    else if(sensor == "ir" || sensor == "button" || sensor == "rain" ||
            sensor == "soil" || sensor == "ldr"){
      int val = digitalRead(i);
      readings[String(i)] = val;
      any = true;
    }
    // servo has no readback; actuated only via commands
  }

  if(dedicatedChoice == "ultrasonic"){
    float dist = readUltrasonicCM(dedicatedTrig, dedicatedEcho);
    if(dist >= 0){ readings["16/17"] = dist; any = true; }
  }

  if(any){
    serializeJson(doc, Serial);
    Serial.println();
  }
}

float readUltrasonicCM(int trigPin, int echoPin){
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  long duration = pulseIn(echoPin, HIGH, 30000UL);
  if(duration == 0) return -1;
  return duration * 0.0343 / 2.0;
}

void showOledText(const char* text){
  if(!oledEnabled) return;
  oledDisplay.clearDisplay();
  oledDisplay.setTextSize(1);
  oledDisplay.setCursor(0,0);
  oledDisplay.println("CognoSpace Sensor Lab");
  oledDisplay.setCursor(0, 20);
  oledDisplay.setTextSize(2);
  oledDisplay.println(text);
  oledDisplay.display();
}
