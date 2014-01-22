#include <PR_Header.h>
#include <Wire.h>

int resetPin = 12;
int SDIO = A4;
int SCLK = A5;

Public_Radio radio(resetPin, SDIO, SCLK);
int channel = 883;
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
