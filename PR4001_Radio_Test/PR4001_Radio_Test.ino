/*
THE PUBLIC RADIO
WORKING TEST CODE
Written by Spencer Wright and Zach Dunham
with thanks to SparkFun's si470x team

This code runs on our semi-custom hardware (PR5001 and PR5003) but can also be used
on Sparkfun+Adruino parts.

The basic rundown is as follows:
  -Intialize everything on the Arduino
  -Intialize si470x
  -Enable shutdown on the amp
  -Power up si470x
  -Set tuning frequency & set volume on si470x
  -Disable shutdown on the amp
  -Turn the MCU off for power savings
  -The MCU never gets to the loop

LAST EDITED 2014.01.28
*/

#include <PR_Header.h>                      //this is our custom library, adapted from sparkfun
#include <avr/sleep.h>                      //the standard AVR sleep library
#include <Wire.h>                           //for i2c communication with si470x


int resetPin = 12;                          //this is the reset pin for si470x
int SDIO = A4;                              //for i2c with si470x
int SCLK = A5;                              //for i2c with si470x
int shutdown = 11;                          //the shutdown pin for the amp, used to mute output duing powerup
int led = 13;                               //not used - for debugging only

Public_Radio radio(resetPin, SDIO, SCLK);   //initializing the radio
int channel = 939;                          //setting channel. 97.1 MHz = "971"
int volume = 15;                            //setting volume, 0-15. 15 works well

void setup()
{
  pinMode(shutdown, OUTPUT);                //setting the amp shutdown pin
  digitalWrite(shutdown, LOW);              //shutting down the amp
  radio.powerOn();                          //powering on si470x
  radio.setVolume(volume);                  //setting output volume on si470x
  radio.setChannel(channel);                //setting channel on si470x
  digitalWrite(shutdown, HIGH);             //disabling shutdown on amp (turns amp on) 
  sleepNow();                               //sleeps the MCU
}

void sleepNow()                             //the sleep function
{
  set_sleep_mode(SLEEP_MODE_PWR_DOWN);      //this is the most power-saving mode in avr/sleep.h
  sleep_enable();                           //enables the sleep bit in the mcucr register
                                            //so sleep is possible. just a safety pin
  sleep_mode();                             //here the device is actually put to sleep!!
}

void loop()                                  //this needs to be in the code but is unused
{                                            //the processor has already been put to sleep
}
