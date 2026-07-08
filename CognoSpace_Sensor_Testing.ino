/* =====================================================================
   CognoSpace Sensor Testing — ESP32 Firmware
   ---------------------------------------------------------------------
   Pairs with index.html (Web Serial API dashboard).
   Protocol: newline-delimited JSON over USB Serial @ 115200 baud.

   Browser -> ESP32:
     {"cmd":"setpins","pins":[{"pin":12,"sensor":"dht11"}, ...],
      "dedicated":"ultrasonic"|"display4"|"none","oled":true|false}
     {"cmd":"servo","pin":25,"angle":90}
     {"cmd":"gpio","pin":18,"state":true}

   ESP32 -> Browser (sent ~every 400ms per configured sensor):
     {"pin":12,"value":26.5}

   Configurable GPIO pins (choose sensor per pin): 32, 33, 25, 15, 12, 23
   Dedicated shared pins: 16 & 17  -> Ultrasonic (TRIG/ECHO) OR TM1637 4-Digit Display (CLK/DIO)
   I2C (OLED):  SDA 21 / SCL 22  (ESP32 default)
   Fixed components: LED1 = GPIO 18, LED2 = GPIO 19, Buzzer = GPIO 4
   ===================================================================== */

#include <ArduinoJson.h>      // Install via Library Manager: "ArduinoJson" by Benoit Blanchon
#include <ESP32Servo.h>       // Install via Library Manager: "ESP32Servo"
#include <DHT.h>              // Install via Library Manager: "DHT sensor library" by Adafruit
#include <Wire.h>
#include <Adafruit_GFX.h>     // Install via Library Manager: "Adafruit GFX Library"
#include <Adafruit_SSD1306.h> // Install via Library Manager: "Adafruit SSD1306"
#include <TM1637Display.h>    // Install via Library Manager: "TM1637" by Avishay Orpaz

// ---------------- Fixed / constant pins ----------------
#define LED1_PIN   18
#define LED2_PIN   19
#define BUZZER_PIN 4

// ---------------- Dedicated shared pins ----------------
// GPIO 16 and 17 are shared between the Ultrasonic Sensor and the 4-Digit Display.
// Which physical pin plays which role (TRIG/ECHO or CLK/DIO) is chosen from the
// dashboard's Set Pin Mode and sent here via the "setpins" command.
int trigPin = 16, echoPin = 17;   // ultrasonic role assignment (defaults)
int clkPin  = 16, dioPin  = 17;   // 4-digit display role assignment (defaults)

// ---------------- I2C OLED ----------------
#define OLED_SDA 21
#define OLED_SCL 22
#define OLED_WIDTH  128
#define OLED_HEIGHT 64
Adafruit_SSD1306 oled(OLED_WIDTH, OLED_HEIGHT, &Wire, -1);
bool oledEnabled = false;

// ---------------- Dedicated device state ----------------
String dedicatedMode = "none"; // "none" | "ultrasonic" | "display4"
TM1637Display* tm1637 = nullptr;

// ---------------- Configurable pin -> sensor map ----------------
struct PinAssignment {
  int pin = -1;
  String sensor = "";
  bool active = false;
  Servo servo;
  DHT* dht = nullptr;
};
#define MAX_PINS 6
PinAssignment assignments[MAX_PINS];
const int CONFIG_PINS[MAX_PINS] = {32, 33, 25, 15, 12, 23};

unsigned long lastSend = 0;
const unsigned long SEND_INTERVAL = 400; // ms

void setup() {
  Serial.begin(115200);
  delay(300);

  pinMode(LED1_PIN, OUTPUT);
  pinMode(LED2_PIN, OUTPUT);
  pinMode(BUZZER_PIN, OUTPUT);
  digitalWrite(LED1_PIN, LOW);
  digitalWrite(LED2_PIN, LOW);
  digitalWrite(BUZZER_PIN, LOW);

  Wire.begin(OLED_SDA, OLED_SCL);

  Serial.println("{\"status\":\"ready\"}");
}

void loop() {
  readSerialCommands();
  sendLiveReadings();
}

/* =====================================================================
   Handle incoming JSON commands from the browser
   ===================================================================== */
void readSerialCommands() {
  static String lineBuf = "";
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n') {
      if (lineBuf.length() > 0) {
        processCommand(lineBuf);
        lineBuf = "";
      }
    } else {
      lineBuf += c;
    }
  }
}

void processCommand(const String& line) {
  StaticJsonDocument<1024> doc;
  DeserializationError err = deserializeJson(doc, line);
  if (err) return;

  String cmd = doc["cmd"] | "";

  if (cmd == "setpins") {
    applyPinConfig(doc);
  } else if (cmd == "servo") {
    int pin = doc["pin"] | -1;
    int angle = doc["angle"] | 90;
    for (int i = 0; i < MAX_PINS; i++) {
      if (assignments[i].active && assignments[i].pin == pin && assignments[i].sensor == "servo") {
        assignments[i].servo.write(angle);
      }
    }
  } else if (cmd == "gpio") {
    int pin = doc["pin"] | -1;
    bool state = doc["state"] | false;
    if (pin == LED1_PIN || pin == LED2_PIN || pin == BUZZER_PIN) {
      digitalWrite(pin, state ? HIGH : LOW);
    }
  } else if (cmd == "oled_text") {
    String text = doc["text"] | "";
    if (oledEnabled) showOledText(text);
  } else if (cmd == "display_number") {
    long value = doc["value"] | 0;
    if (dedicatedMode == "display4" && tm1637) {
      tm1637->showNumberDec(value);
    }
  }
}

/* =====================================================================
   Apply a fresh pin configuration sent from the dashboard
   ===================================================================== */
void applyPinConfig(JsonDocument& doc) {
  // reset previous configurable pin assignments
  for (int i = 0; i < MAX_PINS; i++) {
    if (assignments[i].dht) { delete assignments[i].dht; assignments[i].dht = nullptr; }
    assignments[i] = PinAssignment();
  }

  JsonArray pins = doc["pins"].as<JsonArray>();
  int idx = 0;
  for (JsonObject p : pins) {
    if (idx >= MAX_PINS) break;
    int pin = p["pin"] | -1;
    String sensor = p["sensor"] | "";
    if (pin < 0 || sensor == "") continue;

    assignments[idx].pin = pin;
    assignments[idx].sensor = sensor;
    assignments[idx].active = true;

    if (sensor == "servo") {
      assignments[idx].servo.attach(pin);
      assignments[idx].servo.write(90);
    } else if (sensor == "dht11") {
      assignments[idx].dht = new DHT(pin, DHT11);
      assignments[idx].dht->begin();
    } else {
      // ir, button, rain, soil, ldr -> simple digital input
      pinMode(pin, INPUT);
    }
    idx++;
  }

  // dedicated shared pins (GPIO 16 & 17) — role assignment comes from the dashboard
  dedicatedMode = String(doc["dedicated"] | "none");
  if (tm1637) { delete tm1637; tm1637 = nullptr; }

  JsonObject roles = doc["dedicatedRoles"];
  if (dedicatedMode == "ultrasonic") {
    trigPin = roles["trig"] | 16;
    echoPin = roles["echo"] | 17;
    pinMode(trigPin, OUTPUT);
    pinMode(echoPin, INPUT);
  } else if (dedicatedMode == "display4") {
    clkPin = roles["clk"] | 16;
    dioPin = roles["dio"] | 17;
    tm1637 = new TM1637Display(clkPin, dioPin);
    tm1637->setBrightness(5);
  }

  // OLED
  oledEnabled = doc["oled"] | false;
  if (oledEnabled) {
    if (oled.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
      oled.clearDisplay();
      oled.setTextColor(SSD1306_WHITE);
      oled.setTextSize(1);
      oled.setCursor(0, 0);
      oled.println("CognoSpace");
      oled.println("Sensor Test Ready");
      oled.display();
    }
  }

  Serial.println("{\"status\":\"pins_saved\"}");
}

/* =====================================================================
   Read every configured sensor and stream JSON to the browser
   ===================================================================== */
void sendLiveReadings() {
  if (millis() - lastSend < SEND_INTERVAL) return;
  lastSend = millis();

  for (int i = 0; i < MAX_PINS; i++) {
    if (!assignments[i].active) continue;
    String sensor = assignments[i].sensor;
    int pin = assignments[i].pin;

    if (sensor == "dht11" && assignments[i].dht) {
      float t = assignments[i].dht->readTemperature();
      if (!isnan(t)) sendReading(pin, t);
    } else if (sensor == "servo") {
      // actuator — no periodic read needed
      continue;
    } else {
      // ir, button, rain, soil, ldr -> plain digital read
      int val = digitalRead(pin);
      sendReading(pin, val);
    }
  }

  if (dedicatedMode == "ultrasonic") {
    long distance = readUltrasonicCM();
    sendReading("16/17", distance);
  }
  // Note: the 4-Digit Display and OLED are output-only devices — they show
  // whatever text/number the user sends from the Live Mode dashboard
  // (see "oled_text" and "display_number" commands above), so nothing to read here.
}

void sendReading(int pin, float value) {
  StaticJsonDocument<128> doc;
  doc["pin"] = pin;
  doc["value"] = value;
  serializeJson(doc, Serial);
  Serial.println();
}
void sendReading(const char* pinLabel, float value) {
  StaticJsonDocument<128> doc;
  doc["pin"] = pinLabel;
  doc["value"] = value;
  serializeJson(doc, Serial);
  Serial.println();
}

long readUltrasonicCM() {
  digitalWrite(trigPin, LOW);  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH); delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  long duration = pulseIn(echoPin, HIGH, 25000); // 25ms timeout
  long cm = duration / 29 / 2;
  return cm;
}

// Displays whatever text the user types into the OLED text box in Live Mode.
// Wraps long text across multiple lines so it fits the 128x64 screen.
void showOledText(String text) {
  oled.clearDisplay();
  oled.setTextSize(1);
  oled.setTextColor(SSD1306_WHITE);
  oled.setCursor(0, 0);
  oled.println("CognoSpace Live:");
  oled.setTextSize(2);
  oled.setCursor(0, 16);
  oled.println(text);
  oled.display();
}
