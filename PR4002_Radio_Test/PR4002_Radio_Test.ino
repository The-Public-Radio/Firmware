#include <PR_Header.h>
#include <Wire.h>

int resetPin = 2;
int SDIO = A4;
int SCLK = A5;

Public_Radio radio(resetPin, SDIO, SCLK);
int channel = 971;
int volume = 15;

void setup()
{
  radio.powerOn();
  radio.setVolume(volume);
  radio.setChannel(channel);
}

void loop()
{
}
