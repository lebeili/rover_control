#include <ESP32Servo.h>
#include <TB6612_ESP32.h>
#include <WebSocketsClient.h>
#include <WiFi.h>


#include <WiFi.h>
#include <HTTPClient.h>
#include <WiFiMulti.h>
#define SERVER_IP "109.204.233.236:8080"
#ifndef STASSID
#define STASSID "Tuoppias"
#define STAPSK "k2tmjv9i"
#endif
WiFiMulti wifiMulti;
#include <Adafruit_BMP280.h>
#include <Adafruit_MPU6050.h>
#include <TinyGPSPlus.h>
#include <SoftwareSerial.h>
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


//#include "secrets.h"

#define INDICATOR 15

#define STBY 2

#define AIN1 16
#define AIN2 17
#define PWMA 18

#define BIN1 21
#define BIN2 22
#define PWMB 23

#define SERVO 4

const char *ssid = "Tuoppias";
const char *password = "k2tmjv9i";
const char *wsUrl = "109.204.233.236";

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

  webSocket.begin(wsUrl, 3010);
  webSocket.onEvent(webSocketEvent);
  webSocket.setReconnectInterval(3000);
}

//void loop() { webSocket.loop(); }











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

  /* Display the results (acceleration is measured in m/s^2) */
/*Serial.print("Accel X: ");
  Serial.print(accel.acceleration.x);
  Serial.print(" \tY: ");
  Serial.print(accel.acceleration.y);
  Serial.print(" \tZ: ");
  Serial.print(accel.acceleration.z);
  Serial.println(" m/s^2 ");
*/
  /* Display the results (rotation is measured in rad/s) */
 /* Serial.print("Gyro X: ");
  Serial.print(gyro.gyro.x);
  Serial.print(" \tY: ");
  Serial.print(gyro.gyro.y);
  Serial.print(" \tZ: ");
  Serial.print(gyro.gyro.z);
  Serial.println(" radians/s ");
  Serial.println();
*/


  webSocket.loop();


static const double LONDON_LAT = 51.508131, LONDON_LON = -0.128002;
/*Serial.print("Satellite value:");
  printInt(gps.satellites.value(), gps.satellites.isValid(), 5);
  Serial.print("\nHDOP:");
  printFloat(gps.hdop.hdop(), gps.hdop.isValid(), 6, 1);
  Serial.print("\nLatitude:");
  printFloat(gps.location.lat(), gps.location.isValid(), 11, 6);
  Serial.print("\nLongitude:");
  printFloat(gps.location.lng(), gps.location.isValid(), 12, 6);
  Serial.print("\nAge:");
  printInt(gps.location.age(), gps.location.isValid(), 5);
  Serial.print("\nDate and time:");
  printDateTime(gps.date, gps.time);
  Serial.print("\nAltitude:");
  printFloat(gps.altitude.meters(), gps.altitude.isValid(), 7, 2);
  Serial.print("\nCourse:");
  printFloat(gps.course.deg(), gps.course.isValid(), 7, 2);
  Serial.print("\nSpeed:");
  printFloat(gps.speed.kmph(), gps.speed.isValid(), 6, 2);
Serial.print("\nCardinal:");  
  printStr(gps.course.isValid() ? TinyGPSPlus::cardinal(gps.course.deg()) : "*** ", 6);

  
  Serial.println();
Serial.println("--------------------------------------");  */
  //smartDelay(500);


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
//  sprintf("\n[HEXDUMP] Address: 0x%08X len: 0x%X (%d)", (ptrdiff_t)src, len, len);
  for(uint32_t i = 0; i < len; i++) {
    if(i % cols == 0) {
//      sprintf("\n[0x%08X] 0x%08X: ", (ptrdiff_t)src, i);
    }
 //   sprintf("%02X ", *src);
    src++;
  }
  Serial.print("\n");
}
