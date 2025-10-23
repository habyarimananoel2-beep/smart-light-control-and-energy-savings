#include <WiFi.h>
#include <UbidotsEsp32Mqtt.h>

/****************************************
 * Wi-Fi and Ubidots Credentials
 ****************************************/
#define WIFI_SSID       "DESKTOP-JKNH2PT4993"
#define WIFI_PASSWORD   "3D045@7s"
#define UBIDOTS_TOKEN   "BBUS-H7akBGyO4ziOtg7Yc7yCdH7JDReNBi"
#define DEVICE_LABEL    "smart_lamp"

/****************************************
 * Hardware Pins and Thresholds
 ****************************************/
#define LDR_PIN         34
#define PIR_PIN         27
#define RELAY1_PIN      26
#define RELAY2_PIN      25
#define RELAY3_PIN      33
#define RED_LED         12
#define GREEN_LED       14
#define ACS_PIN         35
#define LDR_THRESHOLD   500

/****************************************
 * ACS712 20A Sensor Constants
 ****************************************/
#define ADC_RESOLUTION  4095
#define VREF            3.3
#define SENSITIVITY     0.100  // 100 mV/A

/****************************************
 * Global Variables
 ****************************************/
Ubidots ubidots(UBIDOTS_TOKEN);
bool manualControl1 = false;
bool manualControl2 = false;
bool manualControl3 = false;

int lampStatus1 = 0;
int lampStatus2 = 0;
int lampStatus3 = 0;

/****************************************
 * Extract Variable Label from Topic
 ****************************************/
String extractVariableLabel(const char* topic) {
  String topicStr = String(topic);
  int lastSlash = topicStr.lastIndexOf('/');
  if (lastSlash < 1) return String("");
  int secondLastSlash = topicStr.lastIndexOf('/', lastSlash - 1);
  if (secondLastSlash < 0) return String("");
  return topicStr.substring(secondLastSlash + 1, lastSlash);
}

/****************************************
 * MQTT Callback
 ****************************************/
void callback(char *topic, byte *payload, unsigned int length) {
  String variableLabel = extractVariableLabel(topic);
  String value = "";

  for (unsigned int i = 0; i < length; i++) value += (char)payload[i];
  float switchValue = value.toFloat();

  Serial.print("Received -> ");
  Serial.print(variableLabel);
  Serial.print(": ");
  Serial.println(value);

  if (variableLabel == "manual_switch1") {
    manualControl1 = (switchValue == 1.0);
  } else if (variableLabel == "manual_switch2") {
    manualControl2 = (switchValue == 1.0);
  } else if (variableLabel == "manual_switch3") {
    manualControl3 = (switchValue == 1.0);
  }
}

/****************************************
 * Measure AC Current (RMS)
 ****************************************/
float getACCurrent() {
  int numSamples = 500;
  float sumSq = 0;

  for (int i = 0; i < numSamples; i++) {
    int adcValue = analogRead(ACS_PIN);
    float voltage = (adcValue * VREF) / ADC_RESOLUTION;
    float offset = voltage - 2.5;
    sumSq += offset * offset;
  }

  float meanSq = sumSq / numSamples;
  float rmsVoltage = sqrt(meanSq);
  float current = rmsVoltage / SENSITIVITY;
  return current;
}

/****************************************
 * Setup
 ****************************************/
void setup() {
  Serial.begin(115200);

  pinMode(LDR_PIN, INPUT);
  pinMode(PIR_PIN, INPUT);

  pinMode(RELAY1_PIN, OUTPUT);
  pinMode(RELAY2_PIN, OUTPUT);
  pinMode(RELAY3_PIN, OUTPUT);
  digitalWrite(RELAY1_PIN, LOW);
  digitalWrite(RELAY2_PIN, LOW);
  digitalWrite(RELAY3_PIN, LOW);

  pinMode(RED_LED, OUTPUT);
  pinMode(GREEN_LED, OUTPUT);
  digitalWrite(RED_LED, LOW);
  digitalWrite(GREEN_LED, LOW);

  Serial.print("Connecting Wi-Fi");
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\n✅ Wi-Fi connected");

  ubidots.setCallback(callback);
  ubidots.setup();
  ubidots.reconnect();

  // Subscribe each manual control variable
  ubidots.subscribeLastValue(DEVICE_LABEL, "manual_switch1");
  ubidots.subscribeLastValue(DEVICE_LABEL, "manual_switch2");
  ubidots.subscribeLastValue(DEVICE_LABEL, "manual_switch3");
  Serial.println("✅ Subscribed to manual switches");
}

/****************************************
 * Loop
 ****************************************/
void loop() {
  if (!ubidots.connected()) {
    ubidots.reconnect();
    ubidots.subscribeLastValue(DEVICE_LABEL, "manual_switch1");
    ubidots.subscribeLastValue(DEVICE_LABEL, "manual_switch2");
    ubidots.subscribeLastValue(DEVICE_LABEL, "manual_switch3");
  }

  int ldrValue = analogRead(LDR_PIN);
  int motionDetected = digitalRead(PIR_PIN);
  Serial.print("LDR: "); Serial.print(ldrValue);
  Serial.print(" | Motion: "); Serial.println(motionDetected);

  digitalWrite(RED_LED, LOW);
  digitalWrite(GREEN_LED, LOW);

  // === AUTO & MANUAL LOGIC ===
  for (int i = 1; i <= 3; i++) {
    bool manualControl = (i == 1 ? manualControl1 : i == 2 ? manualControl2 : manualControl3);
    int relayPin = (i == 1 ? RELAY1_PIN : i == 2 ? RELAY2_PIN : RELAY3_PIN);
    int *lampStatus = (i == 1 ? &lampStatus1 : i == 2 ? &lampStatus2 : &lampStatus3);

    if (manualControl) {
      // Manual mode ON
      digitalWrite(relayPin, HIGH);
      *lampStatus = 1;
      Serial.printf("Lamp %d ON (Manual Mode)\n", i);
    } else {
      // Auto mode
      if (ldrValue < LDR_THRESHOLD && motionDetected == HIGH) {
        digitalWrite(relayPin, HIGH);
        *lampStatus = 1;
        digitalWrite(RED_LED, HIGH);  // Person in dark
        Serial.printf("Lamp %d ON (Auto Dark + Motion)\n", i);
      } else if (ldrValue >= LDR_THRESHOLD && motionDetected == HIGH) {
        digitalWrite(relayPin, LOW);
        *lampStatus = 0;
        digitalWrite(GREEN_LED, HIGH);  // Person in daylight
        Serial.printf("Lamp %d OFF (Auto Bright + Motion)\n", i);
      } else if (ldrValue < LDR_THRESHOLD && motionDetected == LOW) {
        Serial.printf("Lamp %d no motion, dark — unchanged\n", i);
      } else {
        digitalWrite(relayPin, LOW);
        *lampStatus = 0;
        Serial.printf("Lamp %d OFF (Bright + No Motion)\n", i);
      }
    }
  }

  // Measure current
  float current = getACCurrent();
  Serial.print("Current (A): "); Serial.println(current, 3);

  // === Publish Data ===
  ubidots.add("ldr_value", ldrValue);
  ubidots.add("motion", motionDetected);
  ubidots.add("current", current);
  ubidots.add("lamp1_status", lampStatus1);
  ubidots.add("lamp2_status", lampStatus2);
  ubidots.add("lamp3_status", lampStatus3);
  ubidots.publish(DEVICE_LABEL);

  ubidots.loop();
  delay(3000);
}
