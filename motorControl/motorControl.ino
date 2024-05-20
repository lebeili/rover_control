#include <ESP32Servo.h>
#include <TB6612_ESP32.h>
#include <WebSocketsClient.h>
#include <WiFi.h>

#include "secrets.h"

#define INDICATOR 15

#define STBY 2

#define AIN1 16
#define AIN2 17
#define PWMA 18

#define BIN1 21
#define BIN2 22
#define PWMB 23

#define SERVO 4

const char *ssid = "aalto open";
const char *password = "";
const char *wsUrl = "109.204.233.236:3000";

const int offsetA = 1;
const int offsetB = 1;

Motor motorA = Motor(AIN1, AIN2, PWMA, offsetA, STBY, 5000, 8, 1);
Motor motorB = Motor(BIN1, BIN2, PWMB, offsetB, STBY, 5000, 8, 2);
int speedA = 0;
int speedB = 0;

Servo servo;

WebSocketsClient webSocket;

// Do lerp for changing motor speeds to reduce back EMF
void lerpChangeMotors(int targetB, int targetA) {
  Serial.print("LERPing to A: ");
  Serial.print(targetA);
  Serial.print(" B: ");
  Serial.println(targetB);
  int na = abs((targetA - speedA) / 20);
  int nb = abs((targetB - speedB) / 20);
  int n = max(1, max(na, nb));

  int stepA = (targetA - speedA) / n;
  int stepB = (targetB - speedB) / n;
  for (int i = 0; i < n; i++) {
    Serial.print("A: ");
    Serial.println(speedA + stepA);
    Serial.print("B: ");
    Serial.println(speedB + stepB);
    speedA += stepA;
    speedB += stepB;
    delay(50);
    motorA.drive(speedA + stepA);
    motorB.drive(speedB + stepB);
  }

  speedA = targetA;
  speedB = targetB;

  motorA.drive(targetA);
  motorB.drive(targetB);
}

void handleDirectionChange(signed char rawX, signed char rawY) {
  int x = (int)(rawX | 0);
  int y = (int)(rawY | 0);

  // No turning required, y is just speed
  if (x == 0) {
    lerpChangeMotors(y, y);
    if (y == 0) {
      motorA.standby();
      motorB.standby();
    }
    return;
  }

  int adjX = map(abs(x), 1, 100, y, -y);

  // Turning left
  if (x < 0) {
    lerpChangeMotors(adjX, y);
  }
  // Turning right
  else if (x > 0) {
    lerpChangeMotors(y, adjX);
  }
}

void webSocketEvent(WStype_t type, uint8_t *payload, size_t length) {
  switch (type) {
  case WStype_DISCONNECTED:
    Serial.println("[WSc] Disconnected!\n");
    Serial.println((char *)payload);
    break;
  case WStype_ERROR:
    Serial.println("[WSc] There was an oopsie!\n");
    break;
  case WStype_CONNECTED: {
    Serial.print("[WSc] Connected to url: ");
    Serial.println((char *)payload);
    //  send message to server when Connected
    webSocket.sendTXT("Hello server, this is MECHA-POUTA speaking.");
  } break;
  case WStype_TEXT:
    Serial.print("[WSc] get text: ");
    Serial.println((char *)payload);
    break;
  case WStype_BIN:
    Serial.print("[WSc] get binary length: ");
    Serial.println(length);
    if (length == 1) {
      // Reverse angle values as servo is installed backwards
      servo.write(map((int)(payload[0] | 0), 0, 180, 180, 0));
    } else if (length == 2) {
      handleDirectionChange((signed char)payload[0], (signed char)payload[1]);
    }
    break;
  }
}

void setup() {
  motorA.standby();
  motorB.standby();

  servo.setPeriodHertz(50);
  servo.attach(SERVO, 1000, 2000);

  pinMode(INDICATOR, OUTPUT);

  Serial.begin(9600);

  // Connect to wifi
  digitalWrite(INDICATOR, HIGH);

  WiFi.begin(ssid, password);
  delay(1000);
  digitalWrite(INDICATOR, LOW);

  // Wait some time to connect to wifi
  for (int i = 0; i < 10 && WiFi.status() != WL_CONNECTED; i++) {
    digitalWrite(INDICATOR, HIGH);
    delay(1000);
    digitalWrite(INDICATOR, LOW);
    Serial.println("Connecting...");
    WiFi.begin(ssid, password);
    delay(100);
  }

  webSocket.begin(wsUrl, 3010);
  webSocket.onEvent(webSocketEvent);
  webSocket.setReconnectInterval(3000);
}

void loop() { webSocket.loop(); }
