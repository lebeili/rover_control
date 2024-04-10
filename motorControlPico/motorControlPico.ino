#include <Arduino.h>
#include <string.h>

#include <WebSocketsClient.h>
#include <WiFi.h>

#include "secrets.h"

#define FWD 19
#define RIGHT 18
#define BACK 17
#define LEFT 16

const char *ssid = SECRET_SSID;
const char *password = SECRET_PWD;
const char *wsIp = SECRET_IP;

int prev = -1;

WebSocketsClient webSocket;

void flashLed() {
  digitalWrite(LED_BUILTIN, HIGH);
  delay(100);
  digitalWrite(LED_BUILTIN, LOW);
}

void handleDirectionChange(char *dir) {
  Serial.print("Switching to direction: ");
  Serial.println(dir);
  if (prev != -1) {
    digitalWrite(prev, LOW);
  }
  if (strcmp(dir, "fwd") == 0) {
    digitalWrite(FWD, HIGH);
    prev = FWD;
  } else if (strcmp(dir, "right") == 0) {
    digitalWrite(RIGHT, HIGH);
    prev = RIGHT;
  } else if (strcmp(dir, "back") == 0) {
    digitalWrite(BACK, HIGH);
    prev = BACK;
  } else if (strcmp(dir, "left") == 0) {
    digitalWrite(LEFT, HIGH);
    prev = LEFT;
  }
}

void webSocketEvent(WStype_t type, uint8_t *payload, size_t length) {
  switch (type) {
  case WStype_DISCONNECTED:
    Serial.println("[WSc] Disconnected!\n");
    break;
  case WStype_CONNECTED: {
    Serial.print("[WSc] Connected to url: ");
    Serial.println((char *)payload);
    // send message to server when Connected
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
    // hexdump(payload, length);

    // send data to server
    // webSocket.sendBIN(payload, length);
    break;
  }
}

void setup() {
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(LEFT, OUTPUT);
  pinMode(BACK, OUTPUT);
  pinMode(RIGHT, OUTPUT);
  pinMode(FWD, OUTPUT);

  // Open serial communications and wait for port to open:
  Serial.begin(115200);

  // Connect to wifi
  WiFi.begin(ssid, password);

  // Wait some time to connect to wifi
  for (int i = 0; i < 10 && WiFi.status() != WL_CONNECTED; i++) {
    Serial.print(".");
    for (int j = 0; j < 5; j++) {
      flashLed();
      delay(200);
    }
    WiFi.begin(ssid, password);
  }
  Serial.println("Connected to WiFi");
  digitalWrite(LED_BUILTIN, HIGH);

  webSocket.begin(wsIp, 3000);
  webSocket.onEvent(webSocketEvent);
  webSocket.setReconnectInterval(1000);
}

void loop() { webSocket.loop(); }
