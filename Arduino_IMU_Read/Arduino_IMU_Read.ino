#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Smoothed.h>
#include <Wire.h>

//Initialize sensor object
Adafruit_MPU6050 mpu;

//Initialize smoothed objects for data filtering
Smoothed<float>sensorXa;
Smoothed<float>sensorYa;
Smoothed<float>sensorZa;
Smoothed<float>sensorXg;
Smoothed<float>sensorYg;
Smoothed<float>sensorZg;

void setup(void) {

  //High baud rate needed for the sensor
  Serial.begin(250000);

  //Wait for serial connection
  while (!Serial) {
    delay(10);
  }

  // Try to initialize and loop infinitely if not
  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    while (1) {
      delay(10);
    }
  }

  //Set the ranges for accelerometer, gyroscope, and filter bandwidth settings
  mpu.setAccelerometerRange(MPU6050_RANGE_16_G);
  mpu.setGyroRange(MPU6050_RANGE_250_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_44_HZ);

  //Begin the smoothed reading objects
  sensorXa.begin(SMOOTHED_AVERAGE,10);
  sensorYa.begin(SMOOTHED_AVERAGE,10);
  sensorZa.begin(SMOOTHED_AVERAGE,10);
  sensorXg.begin(SMOOTHED_AVERAGE,10);
  sensorYg.begin(SMOOTHED_AVERAGE,10);
  sensorZg.begin(SMOOTHED_AVERAGE,10);

  delay(100);
}

void loop() {

  // Get new sensor events with the readings
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  //Add sensor readings to the smoothed reading object
  sensorXa.add(a.acceleration.x-0.5);
  sensorYa.add(a.acceleration.y+0.15);
  sensorZa.add(a.acceleration.z);
  sensorXg.add(g.gyro.x+0.08);
  sensorYg.add(g.gyro.y);
  sensorZg.add(g.gyro.z);

  //Get sensor values and send to PC
  double valXa = sensorXa.get();
  double valYa = sensorYa.get();
  double valZa = sensorZa.get();
  double valXg = sensorXg.get();
  double valYg = sensorYg.get();
  double valZg = sensorZg.get();
  sendToPC(&valXa, &valYa, &valZg);
}

void sendToPC(double* data1, double* data2, double* data3, double* data4, double* data5, double* data6)
{
  byte* byteData1 = (byte*)(data1);
  byte* byteData2 = (byte*)(data2);
  byte* byteData3 = (byte*)(data3);
  byte* byteData4 = (byte*)(data4);
  byte* byteData5 = (byte*)(data5);
  byte* byteData6 = (byte*)(data6);
  byte buf[24] = {byteData1[0], byteData1[1], byteData1[2], byteData1[3],
                 byteData2[0], byteData2[1], byteData2[2], byteData2[3],
                 byteData3[0], byteData3[1], byteData3[2], byteData3[3],
                 byteData4[0], byteData4[1], byteData4[2], byteData4[3],
                 byteData5[0], byteData5[1], byteData5[2], byteData5[3],
                 byteData6[0], byteData6[1], byteData6[2], byteData6[3]};
  Serial.write(buf, 24);
}

void sendToPC(double* data1, double* data2, double* data3)
{
  byte* byteData1 = (byte*)(data1);
  byte* byteData2 = (byte*)(data2);
  byte* byteData3 = (byte*)(data3);
  byte buf[12] = {byteData1[0], byteData1[1], byteData1[2], byteData1[3],
                 byteData2[0], byteData2[1], byteData2[2], byteData2[3],
                 byteData3[0], byteData3[1], byteData3[2], byteData3[3]};
  Serial.write(buf, 12);
}