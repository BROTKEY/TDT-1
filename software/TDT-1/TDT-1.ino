#include <WiFi.h>
#include <esp_now.h>
#include <PID_v2.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include "Adafruit_BMP3XX.h"
#include <utility/imumaths.h>
#include <pwmWrite.h>

#define SEALEVELPRESSURE_HPA (1013.25)
#define READ_BMP true

//THIS IS THE ROCKET

Pwm pwm = Pwm();

uint8_t ground[] = { 0x34, 0x85, 0x18, 0x02, 0xFE, 0xC8 };

PID_v2 xPid(2, 5, 1, PID::Direct);
PID_v2 yPid(2, 5, 1, PID::Direct);
PID_v2 zPid(2, 5, 1, PID::Direct);

Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28);
Adafruit_BMP3XX bmp;

typedef struct command {
  uint8_t commandCode;
} command;

typedef struct telemetry {
  uint8_t x;
  uint8_t y;
  uint8_t z;
  uint8_t alt;
} telemetry;

command uplink;
telemetry downlink;
esp_now_peer_info_t peerInfo;


void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  Serial.println("Last Package Send Status: ");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Good" : "Bad");
}

void receiveCommand(const uint8_t *mac, const uint8_t *incomingData, int len) {
  memcpy(&uplink, incomingData, sizeof(incomingData));
}

void setup() {
  Serial.begin(115200);

  WiFi.mode(WIFI_MODE_STA);

  Serial.println(WiFi.macAddress());

  WiFi.setTxPower(WIFI_POWER_19_5dBm);
  int a = WiFi.getTxPower();

  Serial.print("Power: ");
  Serial.println(a);

  esp_now_register_send_cb(OnDataSent);
  esp_now_register_recv_cb(receiveCommand);

  memcpy(peerInfo.peer_addr, ground, 6);
  peerInfo.channel = 0;
  peerInfo.encrypt = false;


  if (!bno.begin()) {
    Serial.print("No BNO055 detected");
    while (1)
      ;
  }

#if READ_BMP
  if (!bmp.begin_I2C()) {
    Serial.println("Could not find a valid BMP3 sensor, check wiring!");
    while (1)
      ;
  }

  if (!bmp.performReading()) {
    Serial.println("Failed to perform reading: BMP390");
  }
#endif

  sensors_event_t orientation;

  bno.getEvent(&orientation, Adafruit_BNO055::VECTOR_EULER);

  xPid.Start(orientation.orientation.x, orientation.orientation.x, 180);
  yPid.Start(orientation.orientation.y+180, orientation.orientation.y+180, orientation.orientation.y+180);
  zPid.Start(orientation.orientation.z+180, orientation.orientation.z+180, orientation.orientation.z+180);

#if READ_BMP
  bmp.setTemperatureOversampling(BMP3_OVERSAMPLING_8X);
  bmp.setPressureOversampling(BMP3_OVERSAMPLING_4X);
  bmp.setIIRFilterCoeff(BMP3_IIR_FILTER_COEFF_3);
  bmp.setOutputDataRate(BMP3_ODR_200_HZ);
#endif
}

int maxAngle = 45;

void setXServoAngle(int xAngle, int zAngle) {
  pwm.writeServo(10, (90 + xAngle) + zAngle);
  pwm.writeServo(9, (90 - xAngle) + zAngle);
}

void setYServoAngle(int yAngle, int zAngle) {
  pwm.writeServo(8, (90 + yAngle) + zAngle);
  //pwm.writeServo(7, (90 - yAngle) + zAngle);
}

void loop() {
#if READ_BMP
  if (!bmp.performReading()) {
    Serial.println("Failed to perform reading: BMP390");
  } else {
    Serial.print("\tPressure = ");
    Serial.print(bmp.pressure / 100.0);
    Serial.println(" hPa");

    Serial.print("Approx. Altitude = ");
    Serial.print(bmp.readAltitude(SEALEVELPRESSURE_HPA));
    Serial.println(" m");
  }
#endif

  sensors_event_t orientation, acceleration;

  bno.getEvent(&orientation, Adafruit_BNO055::VECTOR_EULER);
  bno.getEvent(&acceleration, Adafruit_BNO055::VECTOR_LINEARACCEL);


  //if (uplink.commandCode == 0b00000010) {
  // abort
  //} else if (uplink.commandCode == 0b00000001) {
  //fly
  Serial.println("StartCalcing");

  const double xPidValue = xPid.Run(orientation.orientation.x);
  const double yPidValue = yPid.Run(orientation.orientation.y+180);
  const double zPidValue = zPid.Run(orientation.orientation.z+180);


  int xAngle = (maxAngle / 2) * ((xPidValue - 127.5) / 127.5);
  int yAngle = (maxAngle) * ((yPidValue - 127.5) / 127.5);
  int zAngle = (maxAngle) * ((zPidValue - 127.5) / 127.5);

  Serial.print("x:");
  Serial.println(orientation.orientation.x);
  Serial.print("y:");
  Serial.println(orientation.orientation.y);
  Serial.print("z:");
  Serial.println(orientation.orientation.z);

  Serial.print("X: ");
  Serial.print(xAngle);
  Serial.print(" PID: ");
  Serial.println(xPidValue);

  Serial.print("Y: ");
  Serial.print(yAngle);
  Serial.print(" PID: ");
  Serial.println(yPidValue);

  Serial.print("Z: ");
  Serial.print(zAngle);
  Serial.print(" PID: ");
  Serial.println(zPidValue);

  setXServoAngle(zAngle, xAngle);
  setYServoAngle(yAngle, xAngle);

  Serial.println("DoneServoing");
  //}
  delay(100);
}