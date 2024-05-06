#include <TB6612_ESP32.h>
#include <WebSocketsClient.h>
#include <WiFi.h>

#include "secrets.h"

#define INDICATOR 14

#define STBY 2

#define AIN1 16
#define AIN2 17
#define PWMA 18

#define BIN1 21
#define BIN2 22
#define PWMB 23

const char *ssid = SECRET_SSID;
const char *password = SECRET_PWD;
const char *wsUrl = SECRET_URL;

const int MAX_SPEED = 50;
const int TURN_SPEED = 100;
// const int LOW_SPEED = 30;

const int offsetA = 1;
const int offsetB = 1;

Motor motorA = Motor(AIN1, AIN2, PWMA, offsetA, STBY, 5000, 8, 1);
Motor motorB = Motor(BIN1, BIN2, PWMB, offsetB, STBY, 5000, 8, 2);
int speedA = 0;
int speedB = 0;

WebSocketsClient webSocket;

// Do lerp for changing motor speeds to reduce back EMF
void lerpChangeMotors(int targetA, int targetB) {
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

void handleDirectionChange(char *dir) {
  if (strcmp(dir, "fwd") == 0) {
    lerpChangeMotors(MAX_SPEED, MAX_SPEED);
  } else if (strcmp(dir, "right") == 0) {
    lerpChangeMotors(TURN_SPEED, -TURN_SPEED);
  } else if (strcmp(dir, "back") == 0) {
    lerpChangeMotors(-MAX_SPEED, -MAX_SPEED);
  } else if (strcmp(dir, "left") == 0) {
    lerpChangeMotors(-TURN_SPEED, TURN_SPEED);
  } else if (strcmp(dir, "stby") == 0) {
    lerpChangeMotors(0, 0);
    motorA.standby();
    motorB.standby();
  }
}

void webSocketEvent(WStype_t type, uint8_t *payload, size_t length) {
  digitalWrite(INDICATOR, HIGH);
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
    handleDirectionChange((char *)payload);
    break;
  case WStype_BIN:
    Serial.print("[WSc] get binary length: ");
    Serial.println(length);
    //  hexdump(payload, length);

    // send data to server
    // webSocket.sendBIN(payload, length);
    break;
  }
  delay(100);
  digitalWrite(INDICATOR, LOW);
}

void setup() {
  motorA.standby();
  motorB.standby();
  pinMode(INDICATOR, OUTPUT);

  Serial.begin(9600);

  // Connect to wifi
  digitalWrite(INDICATOR, HIGH);

  motorA.drive(50);
  motorB.drive(50);
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

  webSocket.begin(wsUrl, 3000);
  webSocket.onEvent(webSocketEvent);
  webSocket.setReconnectInterval(3000);
}

void loop() { webSocket.loop(); }
