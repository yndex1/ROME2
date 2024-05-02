/*
 * Main.cpp
 * Copyright (c) 2024, ZHAW
 * All rights reserved.
 */

#include <stdio.h>
#include <mbed.h>
<<<<<<< HEAD
#include "IRSensor.h"
#include "EncoderCounter.h"
#include "IMU.h"
#include "LIDAR.h"
#include "Controller.h"
#include "StateMachine.h"
=======
#include "IMU.h"
#include "SensorFusion.h"
>>>>>>> 5ddcca0ae7d80c1dd85be0a99991d67cc1582dc3
#include "HTTPServer.h"
#include "HTTPScriptSensorFusion.h"

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
    
<<<<<<< HEAD
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
=======
    // create inertial measurement unit object
>>>>>>> 5ddcca0ae7d80c1dd85be0a99991d67cc1582dc3
    
    SPI spi(PC_12, PC_11, PC_10);
    DigitalOut csAG(PC_8);
    DigitalOut csM(PC_9);
    
    IMU imu(spi, csAG, csM);
    
    SensorFusion sensorFusion(imu);
    
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
<<<<<<< HEAD
    httpServer->add("lidar", new HTTPScriptLIDAR(*lidar));

=======
    httpServer->add("sensorfusion", new HTTPScriptSensorFusion(sensorFusion));
    
>>>>>>> 5ddcca0ae7d80c1dd85be0a99991d67cc1582dc3
    while (true) {
        
        led = !led;
        
        ThisThread::sleep_for(100ms);
        
        
        
    }
}
