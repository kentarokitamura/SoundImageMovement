#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>

#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif

Adafruit_MPU6050 mpu;


#define INTERRUPT_PIN 2  // use pin 2 on Arduino Uno & most boards
#define LED_PIN 13 // (Arduino is 13, Teensy is 11, Teensy++ is 6)
bool blinkState = false;

bool dmpReady = false;
uint8_t mpuIntStatus;
uint8_t devStatus;
uint16_t packetSize;
uint16_t fifoCount;
uint8_t fifoBuffer[64];

Quaternion q;           // [w, x, y, z]         quaternion container
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector
float euler[3];         // [psi, theta, phi]    Euler angle container
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector


uint8_t teapotPacket[14] = { '$', 0x02, 0,0, 0,0, 0,0, 0,0, 0x00, 0x00, '\r', '\n' };
volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
    mpuInterrupt = true;
}


void Init_MPU6050(void){
    SerialPort.begin(9600);
  while (!SerialPort)
    delay(10); // will pause Zero, Leonardo, etc until serial console opens

  SerialPort.println("Adafruit MPU6050 test!");

  // Try to initialize!
  if (!mpu.begin()) {
    SerialPort.println("Failed to find MPU6050 chip");
    while (1) {
      delay(10);
    }
  }
  SerialPort.println("MPU6050 Found!");

  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  SerialPort.print("Accelerometer range set to: ");
  switch (mpu.getAccelerometerRange()) {
  case MPU6050_RANGE_2_G:
    SerialPort.println("+-2G");
    break;
  case MPU6050_RANGE_4_G:
    SerialPort.println("+-4G");
    break;
  case MPU6050_RANGE_8_G:
    SerialPort.println("+-8G");
    break;
  case MPU6050_RANGE_16_G:
    SerialPort.println("+-16G");
    break;
  }
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  SerialPort.print("Gyro range set to: ");
  switch (mpu.getGyroRange()) {
  case MPU6050_RANGE_250_DEG:
    SerialPort.println("+- 250 deg/s");
    break;
  case MPU6050_RANGE_500_DEG:
    SerialPort.println("+- 500 deg/s");
    break;
  case MPU6050_RANGE_1000_DEG:
    SerialPort.println("+- 1000 deg/s");
    break;
  case MPU6050_RANGE_2000_DEG:
    SerialPort.println("+- 2000 deg/s");
    break;
  }

  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
  SerialPort.print("Filter bandwidth set to: ");
  switch (mpu.getFilterBandwidth()) {
  case MPU6050_BAND_260_HZ:
    SerialPort.println("260 Hz");
    break;
  case MPU6050_BAND_184_HZ:
    SerialPort.println("184 Hz");
    break;
  case MPU6050_BAND_94_HZ:
    SerialPort.println("94 Hz");
    break;
  case MPU6050_BAND_44_HZ:
    SerialPort.println("44 Hz");
    break;
  case MPU6050_BAND_21_HZ:
    SerialPort.println("21 Hz");
    break;
  case MPU6050_BAND_10_HZ:
    SerialPort.println("10 Hz");
    break;
  case MPU6050_BAND_5_HZ:
    SerialPort.println("5 Hz");
    break;
  }

  SerialPort.println("");
  delay(100);
}


void MPU6050_task(void){
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);
    control_data_str.accel_x = a.acceleration.x;
    control_data_str.accel_y = a.acceleration.y;
    control_data_str.accel_z = a.acceleration.z;
    control_data_str.yaw_raw = g.gyro.z;
    control_data_str.pitch_raw = g.gyro.x;
    control_data_str.roll_raw = g.gyro.y;

}
