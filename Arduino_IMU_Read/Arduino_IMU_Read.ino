#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include <Mouse.h>
#include <Smoothed.h>

//Initialize sensor object
Adafruit_MPU6050 mpu;

//Initialize smoothed object for data filtering
Smoothed<float>sensorX;
Smoothed<float>sensorY;

void setup(void) {

  //High baud rate needed for the sensor
  Serial.begin(115200);

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

  //Begin the mouse
  Mouse.begin();

  //Begin the smoothed reading object
  sensorX.begin(SMOOTHED_AVERAGE,10);
  sensorY.begin(SMOOTHED_AVERAGE,10);

  delay(100);
}

void loop() {

  // Get new sensor events with the readings
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  //Add sensor readings to the smoothed reading object
  sensorX.add(a.acceleration.x-0.5);
  sensorY.add(a.acceleration.y+0.15);

  //Print data
  Serial.print("X: ");
  Serial.print(sensorX.get());
  Serial.print(", Y: ");
  Serial.print(sensorY.get());
  Serial.println();

  //Mouse.move(sensorX.get(),sensorY.get());
  
  delay(10);
}
