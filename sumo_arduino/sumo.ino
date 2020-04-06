#include <ZumoShield.h>
#include <Wire.h>

//#include "AccMeter.h"

#include <ros.h>
#include <sumo_driver/Zop.h>

ZumoMotors motors;
ZumoBuzzer buzzer;
//Accelerometer lsm303(3);

ros::NodeHandle nh;

double left,right;

// 指令値のコールバック関数
void op_callback(const sumo_driver::Zop& operation) {
  buzzer.playNote(NOTE_G(3), 50, 12);
  left = operation.left;
  right = operation.right;
  
}

ros::Subscriber<sumo_driver::Zop> sub("operation" , &op_callback);

void setup() {
  motors.flipLeftMotor(true);
  /*Wire.begin();
    lsm303.init();
    lsm303.enable();
    Serial.begin(9600);
    lsm303.getLogHeader();*/
  nh.initNode();
  //nh.getHardware()->setBaud(9600);
  nh.subscribe(sub);
  buzzer.playMode(PLAY_AUTOMATIC);
  left = 0.0;
  right = 0.0;
}

void loop() {
  //lsm303.readAcceleration(millis());
  motors.setSpeeds(left, right);
  delay(70);
  motors.setSpeeds(0, 0);
  nh.spinOnce();
  delay(500);
}
