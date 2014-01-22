#include <PR_Header.h>
#include <avr/sleep.h>
#include <Wire.h>


int resetPin = 12;
int SDIO = A4;
int SCLK = A5;
int shutdown = 11;
int led = 13;

Public_Radio radio(resetPin, SDIO, SCLK);
int channel = 883;
int volume = 15;

void setup()
{
  pinMode(shutdown, OUTPUT);
  digitalWrite(shutdown, LOW);
  radio.powerOn();
  radio.setVolume(volume);
  radio.setChannel(channel);
  digitalWrite(shutdown, HIGH);
}

void sleepNow()         // here we put the arduino to sleep
{
  set_sleep_mode(SLEEP_MODE_PWR_DOWN);
  sleep_enable();          // enables the sleep bit in the mcucr register
                             // so sleep is possible. just a safety pin
  sleep_mode();            // here the device is actually put to sleep!!
}

void loop()
{
  digitalWrite(led, HIGH);
  delay(1000);
  digitalWrite(led, LOW);
  delay(1000);
}
