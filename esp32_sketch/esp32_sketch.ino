// Sensors + actuators drivers:
#include <Adafruit_BMP280.h>
#include <Adafruit_MPU6050.h>
#include <TinyGPSPlus.h>
#include <ESP32Servo.h> // ESP32 library for controlling servo motors (Arduino built in one doesn't work)
#include <TB6612_ESP32.h> // The motor driver IC's driver

// Networking drivers:
#include <WebSocketsClient.h>
#include <WiFi.h>
#include <HTTPClient.h>
#include <WiFiMulti.h>

#include <SoftwareSerial.h> // Software serial for the GPS module communication


#define SERVER_IP "109.204.233.236:8080" // put your server address here
#ifndef STASSID
  #define STASSID "aalto open"  // and your wi-fi credentials here:
  #define STAPSK ""
#endif
const char *ssid = "aalto open";
const char *password = "";
const char *wsUrl = "109.204.233.236";

WiFiMulti wifiMulti;
#define BMP_SCK  (13)
#define BMP_MISO (12)
#define BMP_MOSI (11)
#define BMP_CS   (10)
Adafruit_BMP280 bmp;

Adafruit_MPU6050 mpu;
Adafruit_Sensor *mpu_temp, *mpu_accel, *mpu_gyro;

static const int RXPin = 13, TXPin = 12;
static const uint32_t GPSBaud = 9600;
TinyGPSPlus gps;
SoftwareSerial ss(RXPin, TXPin);


// WiFi SSID & password + server URL are pulled from a secrets file to not leak
// them to github
//#include "secrets.h"

// LED indicator pin
#define INDICATOR 15

// Pin definitions for TB6612FNG motor controller wiring
#define STBY 2
#define AIN1 16
#define AIN2 17
#define PWMA 18

#define BIN1 21
#define BIN2 22
#define PWMB 23

// Servo motor data pin
#define SERVO 4


const int offsetA = 1;
const int offsetB = 1;

Motor motorA = Motor(AIN1, AIN2, PWMA, offsetA, STBY, 5000, 8, 1);
Motor motorB = Motor(BIN1, BIN2, PWMB, offsetB, STBY, 5000, 8, 2);

int speedA = 0;
int speedB = 0;

Servo servo;

WebSocketsClient webSocket;

// Accelerate and decelerate motor speed with linear interpolation
// to reduce back-EMF caused by motor coils
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
    motorA.drive((speedA + stepA)*2);
    motorB.drive((speedB + stepB)*2);
  }

  speedA = targetA;
  speedB = targetB;

  motorA.drive(targetA*2);
  motorB.drive(targetB*2);
}

//
// Take X and Y values from joystick input and calculate
// how the motors should react to them:
//
// Roughly speaking, the higher the (absolute value of) Y, the faster the rover
// should move either forward or backward. The higher the value of X, the more
// the rover should be turning either left or right.
//
void handleDirectionChange(signed char rawX, signed char rawY) {
  // values are 8bit, convert them to 32bit integers for easier use down the
  // line
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

  // The higher the X, the further its value should be from the value of Y to
  // turn the rover more or less. The furthest possible is -Y, so map X to a
  // range within [-Y, Y]
  int adjustedX = map(abs(x), 1, 100, y, -y);

  // Turning left, X becomes the value of the left motors
  if (x < 0) {
    lerpChangeMotors(adjustedX, y);
  }
  // Turning right, same for the right motors
  else if (x > 0) {
    lerpChangeMotors(y, adjustedX);
  }
}

// WebSocket event handler
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
    // Length == 1 means the command is for the camera servo angle (value is a
    // degree within [0, 180])
    if (length == 1) {
      // Reverse angle values as servo is installed backwards
      servo.write(map((int)(payload[0] | 0), 0, 180, 180, 0));
    }
    // Length == 2 means the command is for motors. [0] is X and [1] is Y
    else if (length == 2) {
      handleDirectionChange((signed char)payload[0], (signed char)payload[1]);
    }
    break;
  }
}

// Init motors, and connect to WiFi + WebSocket server
void setup() {
    
  Serial.begin(9600);
  #define I2C_SDA 33
  #define I2C_SCL 32
  Wire.begin(I2C_SDA, I2C_SCL);
  pinMode(34, INPUT);

  wifiMulti.addAP(STASSID, STAPSK);

  while((wifiMulti.run() != WL_CONNECTED)) {
    delay(50);
    Serial.print(".");
  }
  Serial.println("");
  Serial.print("Connected! IP address: ");
  Serial.println(WiFi.localIP());
  
  Serial.println(F("BMP280 Forced Mode Test."));

  if (!bmp.begin(0x76)) {
  //if (!bmp.begin()) {
    Serial.println(F("Could not find a valid BMP280 sensor, check wiring or "
                      "try a different address!"));
    while (1) delay(10);
  }

  /* Default settings from datasheet. */
  bmp.setSampling(Adafruit_BMP280::MODE_FORCED,     /* Operating Mode. */
                  Adafruit_BMP280::SAMPLING_X2,     /* Temp. oversampling */
                  Adafruit_BMP280::SAMPLING_X16,    /* Pressure oversampling */
                  Adafruit_BMP280::FILTER_X16,      /* Filtering. */
                  Adafruit_BMP280::STANDBY_MS_500); /* Standby time. */

  Serial.println("Adafruit MPU6050 test!");

  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    while (1) {
      delay(10);
    }
  }

  Serial.println("MPU6050 Found!");
  mpu_temp = mpu.getTemperatureSensor();
  mpu_temp->printSensorDetails();

  mpu_accel = mpu.getAccelerometerSensor();
  mpu_accel->printSensorDetails();

  mpu_gyro = mpu.getGyroSensor();
  mpu_gyro->printSensorDetails();



    ss.begin(GPSBaud);

    
  motorA.standby();
  motorB.standby();

  // Standard servo parameters
  servo.setPeriodHertz(50);
  servo.attach(SERVO, 1000, 2000);

  pinMode(INDICATOR, OUTPUT);


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

  // Connect to WebSocket
  webSocket.begin(wsUrl, 3010);
  webSocket.onEvent(webSocketEvent);
  webSocket.setReconnectInterval(3000);
}



void loop() {
  webSocket.loop();
   //  /* Get a new normalized sensor event */
  sensors_event_t accel;
  sensors_event_t gyro;
  sensors_event_t temp;
  mpu_temp->getEvent(&temp);
  mpu_accel->getEvent(&accel);
  mpu_gyro->getEvent(&gyro);
 if ((wifiMulti.run() == WL_CONNECTED)) {

    HTTPClient http;

    Serial.print("[HTTP] begin...\n");
    // configure traged server and url
    http.begin("http://" SERVER_IP "/api/updateData");  // HTTP
    http.addHeader("Content-Type", "application/json");

    Serial.print("[HTTP] POST...\n");
    // start connection and send HTTP header and body
    int httpCode = http.POST("{\"temperature\":\""+String(bmp.readTemperature())+"\",\"speed\":\""+accel.acceleration.x+"\",\"brightness\":\""+String(analogRead(34))+"\",\"pressure\":\""+String(bmp.readPressure())+"\",\"location\":\""+String(gps.location.lat(),6)+","+String(gps.location.lng(),6)+"\"}");

    // httpCode will be negative on error
    if (httpCode > 0) {
      // HTTP header has been send and Server response header has been handled
      Serial.printf("[HTTP] POST... code: %d\n", httpCode);

      // file found at server
      if (httpCode == HTTP_CODE_OK) {
        const String& payload = http.getString();
        Serial.println("received payload:\n<<");
        Serial.println(payload);
        Serial.println(">>");
      }
    } else {
      Serial.printf("[HTTP] POST... failed, error: %s\n", http.errorToString(httpCode).c_str());
    }

    http.end();
  }

  webSocket.loop();

  // must call this to wake sensor up and get new measurement data
  // it blocks until measurement is complete
 if (bmp.takeForcedMeasurement()) {
    // can now print out the new measurements
    Serial.print(F("Temperature = "));
    Serial.print(bmp.readTemperature());
    Serial.println(" *C");

    Serial.print(F("Pressure = "));
    Serial.print(bmp.readPressure());
    Serial.println(" Pa");

    Serial.print(F("Approx altitude = "));
    Serial.print(bmp.readAltitude(1013.25));
    Serial.println(" m");

    Serial.println();
    
  } else {
    Serial.println("Forced measurement failed!");
  }


 Serial.print("Temperature ");
  Serial.print(temp.temperature);
  Serial.println(" deg C");


  webSocket.loop();


static const double LONDON_LAT = 51.508131, LONDON_LON = -0.128002;

  if (millis() > 5000 && gps.charsProcessed() < 10) {
    Serial.println(F("No GPS data received: check wiring"));
  }
}





// This custom version of delay() ensures that the gps object
// is being "fed".
static void smartDelay(unsigned long ms) {
  unsigned long start = millis();
  do {
    while (ss.available()) {
      gps.encode(ss.read());
    }
  } while (millis() - start < ms);
}

static void printFloat(float val, bool valid, int len, int prec) {
  if (!valid) {
    while (len-- > 1) {
      Serial.print('*');
    }
    Serial.print(' ');
  } else {
    Serial.print(val, prec);
    int vi = abs((int)val);
    int flen = prec + (val < 0.0 ? 2 : 1); // . and -
    flen += vi >= 1000 ? 4 : vi >= 100 ? 3 : vi >= 10 ? 2 : 1;
    for (int i=flen; i<len; ++i) {
      Serial.print(' ');
    }
  }
  smartDelay(0);
}

static void printInt(unsigned long val, bool valid, int len) {
  char sz[32] = "*****************";
  if (valid) {
    sprintf(sz, "%ld", val);
  }
  sz[len] = 0;
  for (int i=strlen(sz); i<len; ++i) {
    sz[i] = ' ';
  }
  if (len > 0) {
    sz[len-1] = ' ';
  }
  Serial.print(sz);
  smartDelay(0);
}

static void printDateTime(TinyGPSDate &d, TinyGPSTime &t) {
  if (!d.isValid()) {
    Serial.print(F("********** "));
  } else {
    char sz[32];
    sprintf(sz, "%02d/%02d/%02d ", d.month(), d.day(), d.year());
    Serial.print(sz);
  }

  if (!t.isValid()) {
    Serial.print(F("******** "));
  } else {
    char sz[32];
    sprintf(sz, "%02d:%02d:%02d ", t.hour(), t.minute(), t.second());
    Serial.print(sz);
  }

  printInt(d.age(), d.isValid(), 5);
  smartDelay(0);
}

static void printStr(const char *str, int len) {
  int slen = strlen(str);
  for (int i=0; i<len; ++i) {
    Serial.print(i<slen ? str[i] : ' ');
  }
  smartDelay(0);
}



void hexdump(const void *mem, uint32_t len, uint8_t cols = 16) {
  const uint8_t* src = (const uint8_t*) mem;
  for(uint32_t i = 0; i < len; i++) {
    if(i % cols == 0) {
    }
    src++;
  }
  Serial.print("\n");
}
