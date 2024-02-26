/* mbed Microcontroller Library
 * Copyright (c) 2019 ARM Limited
 * SPDX-License-Identifier: Apache-2.0
 */

#include "mbed.h"
#include "IRSensor.h"


// Blinking rate in milliseconds
#define BLINKING_RATE     500ms


int main()
{
    // Initialise the digital pin LED1 as an output
    DigitalOut led(LED1);
    DigitalOut led0(PD_4);
    DigitalOut led1(PD_3);
    DigitalOut led2(PD_6);
    DigitalOut led3(PD_2);
    DigitalOut led4(PD_7);
    DigitalOut led5(PD_5);

    //Distanzsensoren 0.1m - 0.8m
    AnalogIn distance(PA_0); // Kreieren der Ein- und Ausgangsobjekte
    DigitalOut enable(PG_1);
    //Bits zum setzen welcher Sensor Tabelle siehe Praktikum 1
    DigitalOut bit0(PF_0);
    DigitalOut bit1(PF_1);
    DigitalOut bit2(PF_2);

    //IR Sensoren initialisieren
    IRSensor irSensor0(distance, bit0, bit1, bit2, 0); // Objekte kreieren
    IRSensor irSensor1(distance, bit0, bit1, bit2, 1);
    IRSensor irSensor2(distance, bit0, bit1, bit2, 2);
    IRSensor irSensor3(distance, bit0, bit1, bit2, 3);
    IRSensor irSensor4(distance, bit0, bit1, bit2, 4);
    IRSensor irSensor5(distance, bit0, bit1, bit2, 5);

    while (true) {
    enable = 1; // schaltet die Sensoren ein
    if(irSensor0.read() < 0.2) led0 = 1;
    else led0 = 0; // gibt die Distanz in [m] zurueck

    if(irSensor1.read() < 0.2) led1 = 1;
    else led1 = 0; // gibt die Distanz in [m] zurueck

    if(irSensor2.read() < 0.2) led2 = 1;
    else led2 = 0; // gibt die Distanz in [m] zurueck

    if(irSensor3.read() < 0.2) led3 = 1;
    else led3 = 0; // gibt die Distanz in [m] zurueck

    if(irSensor4.read() < 0.2) led4 = 1;
    else led4 = 0; // gibt die Distanz in [m] zurueck

    if(irSensor5.read() < 0.2) led5 = 1;
    else led5 = 0; // gibt die Distanz in [m] zurueck

    printf("distance=%dmm\r\n", (int)(1000.0f*irSensor5.read()));
    }
}
