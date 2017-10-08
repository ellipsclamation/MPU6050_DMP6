// ROS Library
#include <ros.h>
#include <std_msgs/Float32.h>

// I2Cdev and MP6050 Libraries
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include "Wire.h"

MPU6050 mpu;
uint16_t packetSize;
uint16_t fifoCount;
uint8_t fifoBuffer[64];
Quaternion q;
VectorFloat gravity;
float ypr[3];

// Ros setup
ros::NodeHandle nh;

std_msgs::Float32 yaw_msg;
std_msgs::Float32 pitch_msg;
std_msgs::Float32 roll_msg;

ros::Publisher chatter_yaw("yaw", &yaw_msg);
ros::Publisher chatter_pitch("pitch", &pitch_msg);
ros::Publisher chatter_roll("roll", &roll_msg);

// Setup
void setup() {
  // Advertise Publisher
  pinMode(13, OUTPUT);
  nh.initNode();
  nh.advertise(chatter_yaw);
  nh.advertise(chatter_pitch);
  nh.advertise(chatter_roll);
  
  Wire.begin();
  mpu.initialize();
  mpu.dmpInitialize();
  mpu.setZAccelOffset(1788);
  mpu.setXGyroOffset(220);
  mpu.setYGyroOffset(76);
  mpu.setZGyroOffset(-85);
  mpu.setDMPEnabled(true);
  packetSize = mpu.dmpGetFIFOPacketSize();
  fifoCount = mpu.getFIFOCount();
  
  Serial.begin(115200);
}

// Main Loop
void loop() {
  while (fifoCount < packetSize) {
    fifoCount = mpu.getFIFOCount();
  }
  
  if (fifoCount == 1024) {
    mpu.resetFIFO();
    Serial.println(F("FIFO overflow!"));
  }
  else{
    if (fifoCount % packetSize != 0) {
      mpu.resetFIFO();
    }
    else{
      while (fifoCount >= packetSize) {
        mpu.getFIFOBytes(fifoBuffer,packetSize);
        fifoCount -= packetSize;
      }
    
      mpu.dmpGetQuaternion(&q,fifoBuffer);
      mpu.dmpGetGravity(&gravity,&q);
      mpu.dmpGetYawPitchRoll(ypr,&q,&gravity);
      
      Serial.print("ypr\t");
      Serial.print(ypr[0] * 180/M_PI);
      Serial.print("\t");
      Serial.print(ypr[1] * 180/M_PI);
      Serial.print("\t");
      Serial.print(ypr[2] * 180/M_PI);
      Serial.println();
      
      // Publish message to ROS
      yaw_msg.data = ypr[0] * 180/M_PI;
      pitch_msg.data = ypr[1] * 180/M_PI;
      roll_msg.data = ypr[2] * 180/M_PI;
      
      chatter_yaw.publish(&yaw_msg);
      chatter_pitch.publish(&pitch_msg);
      chatter_roll.publish(&roll_msg);
      
      nh.spinOnce();
    }
    
  }

}
