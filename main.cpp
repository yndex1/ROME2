/*
 * Main.cpp
 * Copyright (c) 2024, ZHAW
 * All rights reserved.
 */

#include <stdio.h>
#include <mbed.h>
#include "IRSensor.h"
#include "EncoderCounter.h"
#include "IMU.h"
#include "LIDAR.h"
#include "Controller.h"
#include "StateMachine.h"
#include "HTTPServer.h"
#include "HTTPScriptLIDAR.h"

int main() {
    
    // create miscellaneous periphery objects
    
    DigitalIn button(BUTTON1);
    DigitalOut led(LED1);
    
    DigitalOut led0(PD_4);
    DigitalOut led1(PD_3);
    DigitalOut led2(PD_6);
    DigitalOut led3(PD_2);
    DigitalOut led4(PD_7);
    DigitalOut led5(PD_5);
    
    // create IR sensor objects
    
    AnalogIn distance(PA_0);
    DigitalOut enable(PG_1);
    DigitalOut bit0(PF_0);
    DigitalOut bit1(PF_1);
    DigitalOut bit2(PF_2);
    
    IRSensor irSensor0(distance, bit0, bit1, bit2, 0);
    IRSensor irSensor1(distance, bit0, bit1, bit2, 1);
    IRSensor irSensor2(distance, bit0, bit1, bit2, 2);
    IRSensor irSensor3(distance, bit0, bit1, bit2, 3);
    IRSensor irSensor4(distance, bit0, bit1, bit2, 4);
    IRSensor irSensor5(distance, bit0, bit1, bit2, 5);
    
    enable = 1;
    
    // create motor control objects
    
    DigitalOut enableMotorDriver(PG_0); 
    DigitalIn motorDriverFault(PD_1);
    DigitalIn motorDriverWarning(PD_0);
    
    PwmOut pwmLeft(PF_9);
    PwmOut pwmRight(PF_8);
    
    EncoderCounter counterLeft(PD_12, PD_13);
    EncoderCounter counterRight(PB_4, PC_7);
    
    // create inertial measurement unit object
    
    SPI spi(PC_12, PC_11, PC_10);
    DigitalOut csAG(PC_8);
    DigitalOut csM(PC_9);
    
    IMU imu(spi, csAG, csM);

    // create LIDAR device driver
    
    PwmOut pwm(PE_9);
    pwm.period(0.00005f);
    pwm.write(0.5f);
    
    ThisThread::sleep_for(500ms);
    
    UnbufferedSerial* serial = new UnbufferedSerial(PG_14, PG_9);
    LIDAR* lidar = new LIDAR(*serial);
    
    // create robot controller objects
    
    Controller controller(pwmLeft, pwmRight, counterLeft, counterRight);
    StateMachine stateMachine(controller, enableMotorDriver, led0, led1, led2, led3, led4, led5, button, irSensor0, irSensor1, irSensor2, irSensor3, irSensor4, irSensor5);
    
    // create ethernet interface and webserver
    
    DigitalOut enableRouter(PB_15);
    enableRouter = 1;
    
    EthernetInterface* ethernet = new EthernetInterface();
    ethernet->set_network("192.168.0.10", "255.255.255.0", "192.168.0.1"); // configure IP address, netmask and gateway address
    ethernet->connect();
    
    HTTPServer* httpServer = new HTTPServer(*ethernet);
    httpServer->add("lidar", new HTTPScriptLIDAR(*lidar));

    while (true) {
        
        led = !led;
        
        ThisThread::sleep_for(100ms);

        deque<Point> beacons = lidar->getBeacons();


        Point rohr1;
        rohr1.x = 0.0f;
        rohr1.y = 0.5f;
        Point rohr2;
        rohr2.x = 2.0f;
        rohr2.y = 0.5f;
        Point rohr3;
        rohr3.x = 4.0f;
        rohr3.y = 0.5f;
        Point rohr4;
        rohr4.x = 6.0f;
        rohr4.y = 0.5f;

        for (int i; i < beacons.size(); i++) {
          float x = controller.getX();
          float y = controller.getY();
          float alpha = controller.getAlpha();
          Point measuredBeacon;

          measuredBeacon.x = cos(alpha) * beacons[i].x - sin(alpha) * beacons[i].y + x;
          measuredBeacon.y = sin(alpha) * beacons[i].x + cos(alpha) * beacons[i].y + y;

          if (rohr1.distance(measuredBeacon) < 0.4f) {
            controller.correctPoseWithBeacon(rohr1, measuredBeacon);
          } else if (rohr2.distance(measuredBeacon) < 0.4f) {
            controller.correctPoseWithBeacon(rohr2, measuredBeacon);
          } else if (rohr3.distance(measuredBeacon) < 0.4f) {
            controller.correctPoseWithBeacon(rohr3, measuredBeacon);
          } else if (rohr4.distance(measuredBeacon) < 0.4f) {
            controller.correctPoseWithBeacon(rohr4, measuredBeacon);
          }
        }
    }
}
