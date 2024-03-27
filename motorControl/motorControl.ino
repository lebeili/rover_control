#include <ArduinoWebsockets.h>
#include <TB6612_ESP32.h>
#include <WiFi.h>

#include "secrets.h"

using namespace websockets;

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

// At the moment, all motors are hooked up the "wrong" way
const int offsetA = -1;
const int offsetB = -1;

Motor motor1 = Motor(AIN1, AIN2, PWMA, offsetA, STBY, 5000, 8, 1);
Motor motor2 = Motor(BIN1, BIN2, PWMB, offsetB, STBY, 5000, 8, 2);

WebsocketsClient client;

void onMessageCallback(WebsocketsMessage message) {
  Serial.print("Got Message: ");
  Serial.println(message.data());
}

void onEventsCallback(WebsocketsEvent event, String data) {
  if (event == WebsocketsEvent::ConnectionOpened) {
    Serial.println("Connnection Opened");
  } else if (event == WebsocketsEvent::ConnectionClosed) {
    Serial.println("Connnection Closed");
  } else if (event == WebsocketsEvent::GotPing) {
    Serial.println("Got a Ping!");
    Serial.println(data);
  } else if (event == WebsocketsEvent::GotPong) {
    Serial.println("Got a Pong!");
    Serial.println(data);
  }
}

// Connect to WebSocket server
void connectToWsServer() {
  client.connect(wsUrl);
  client.send("Hello Server! This is MECHA-POUTA speaking");
}

void setup() {
  motor1.standby();
  motor2.standby();

  Serial.begin(115200);
  // Connect to wifi
  WiFi.begin(ssid, password);

  // Wait some time to connect to wifi
  for (int i = 0; i < 10 && WiFi.status() != WL_CONNECTED; i++) {
    Serial.print(".");
    delay(1000);
  }

  // Setup Callbacks
  client.onMessage(onMessageCallback);
  client.onEvent(onEventsCallback);
}

void loop() {
  if (client.available()) {
    client.poll();
  } else {
    connectToWsServer();
    delay(1000);
  }
}
