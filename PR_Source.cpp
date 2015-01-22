/*
THE PUBLIC RADIO
2013.12.15
Spencer Wright
Adapted from Aaron Weiss, Simon Monk, Nathan Seidle.
STILL BEERWARE.

This is a wrapper for the subset of functions required to operate The Public Radio. It is based on the Si4703_FM_Tuner_Evaluation_Board
software found at https://github.com/sparkfun/Si4703_FM_Tuner_Evaluation_Board

The Public Radio requires only a few functions: The ability to power on, set channel, and set volume. This software attempts to 
strip out everything not necessary for performing those functions.

:::CHANGELOG:::

2013.12.15
Removed seek functions.
Removed RDS functions
Removed some superfluous commenting.
Renamed many parts referring specifically to Si4703 and the breakout board.


*/

#include "Arduino.h"
#include "PR_Header.h"
#include "Wire.h"

Public_Radio::Public_Radio(int resetPin, int sdioPin, int sclkPin)
{
  _resetPin = resetPin;
  _sdioPin = sdioPin;
  _sclkPin = sclkPin;
}

void Public_Radio::powerOn()
{
    publicradio_init();
}

void Public_Radio::setChannel(int channel)
{
  //Freq(MHz) = 0.200(in USA) * Channel + 87.5MHz
  //97.3 = 0.2 * Chan + 87.5
  //9.8 / 0.2 = 49
  int newChannel = channel * 10; //973 * 10 = 9730
  newChannel -= 8750; //9730 - 8750 = 980
  newChannel /= 10; //980 / 10 = 98

  //These steps come from AN230 page 20 rev 0.5
  readRegisters();
  si4703_registers[CHANNEL] &= 0xFE00; //Clear out the channel bits
  si4703_registers[CHANNEL] |= newChannel; //Mask in the new channel
  si4703_registers[CHANNEL] |= (1<<TUNE); //Set the TUNE bit to start
  updateRegisters();

  //delay(60); //Wait 60ms - you can use or skip this delay

  //Poll to see if STC is set
  while(1) {
    readRegisters();
    if( (si4703_registers[STATUSRSSI] & (1<<STC)) != 0) break; //Tuning complete!
  }

  readRegisters();
  si4703_registers[CHANNEL] &= ~(1<<TUNE); //Clear the tune after a tune has completed
  updateRegisters();

  //Wait for the si4703 to clear the STC as well
  while(1) {
    readRegisters();
    if( (si4703_registers[STATUSRSSI] & (1<<STC)) == 0) break; //Tuning complete!
  }
}


void Public_Radio::setVolume(int volume)
{
  readRegisters(); //Read the current register set
  if(volume < 0) volume = 0;
  if (volume > 15) volume = 15;
  si4703_registers[SYSCONFIG2] &= 0xFFF0; //Clear volume bits
  si4703_registers[SYSCONFIG2] |= volume; //Set new volume
  updateRegisters(); //Update
}



//To get the Si4703 inito 2-wire mode, SEN needs to be high and SDIO needs to be low after a reset
//The breakout board has SEN pulled high, but also has SDIO pulled high. Therefore, after a normal power up
//The Si4703 will be in an unknown state. RST must be controlled
void Public_Radio::publicradio_init() 
{
  pinMode(_resetPin, OUTPUT);
  pinMode(_sdioPin, OUTPUT); //SDIO is connected to A4 for I2C
  digitalWrite(_sdioPin, LOW); //A low SDIO indicates a 2-wire interface
  digitalWrite(_resetPin, LOW); //Put Si4703 into reset
  delay(1); //Some delays while we allow pins to settle
  digitalWrite(_resetPin, HIGH); //Bring Si4703 out of reset with SDIO set to low and SEN pulled high with on-board resistor
  delay(1); //Allow Si4703 to come out of reset
  Wire.begin(); //Now that the unit is reset and I2C inteface mode, we need to begin I2C
  readRegisters(); //Read the current register set
  //si4703_registers[0x07] = 0xBC04; //Enable the oscillator, from AN230 page 9, rev 0.5 (DOES NOT WORK, wtf Silicon Labs datasheet?)
  si4703_registers[0x07] = 0x8100; //Enable the oscillator, from AN230 page 9, rev 0.61 (works)
  updateRegisters(); //Update

  delay(500); //Wait for clock to settle - from AN230 page 9

  readRegisters(); //Read the current register set
  si4703_registers[POWERCFG] = 0x4001; //Enable the IC
  //  si4703_registers[POWERCFG] |= (1<<SMUTE) | (1<<DMUTE); //Disable Mute, disable softmute
  si4703_registers[SYSCONFIG1] |= (1<<RDS); //Enable RDS

  si4703_registers[SYSCONFIG1] |= (1<<DE); //50kHz Europe setup
  si4703_registers[SYSCONFIG2] |= (1<<SPACE0); //100kHz channel spacing for Europe

  si4703_registers[SYSCONFIG2] &= 0xFFF0; //Clear volume bits
  si4703_registers[SYSCONFIG2] |= 0x0001; //Set volume to lowest
  updateRegisters(); //Update

  delay(110); //Max powerup time, from datasheet page 13
}

void Public_Radio::readRegisters(){
  //Si4703 begins reading from register upper register of 0x0A and reads to 0x0F, then loops to 0x00.
  Wire.requestFrom(SI4703, 32); //We want to read the entire register set from 0x0A to 0x09 = 32 bytes.
  while(Wire.available() < 32) ; //Wait for 16 words/32 bytes to come back from slave I2C device
  //We may want some time-out error here
  //Remember, register 0x0A comes in first so we have to shuffle the array around a bit
  for(int x = 0x0A ; ; x++) { //Read in these 32 bytes
    if(x == 0x10) x = 0; //Loop back to zero
    si4703_registers[x] = Wire.read() << 8;
    si4703_registers[x] |= Wire.read();
    if(x == 0x09) break; //We're done!
  }
}


//Write the current 9 control registers (0x02 to 0x07) to the Si4703
//It's a little weird, you don't write an I2C addres
//The Si4703 assumes you are writing to 0x02 first, then increments
byte Public_Radio::updateRegisters() {

  Wire.beginTransmission(SI4703);
  //A write command automatically begins with register 0x02 so no need to send a write-to address
  //First we send the 0x02 to 0x07 control registers
  //In general, we should not write to registers 0x08 and 0x09
  for(int regSpot = 0x02 ; regSpot < 0x08 ; regSpot++) {
    byte high_byte = si4703_registers[regSpot] >> 8;
    byte low_byte = si4703_registers[regSpot] & 0x00FF;

    Wire.write(high_byte); //Upper 8 bits
    Wire.write(low_byte); //Lower 8 bits
  }

  //End this transmission
  byte ack = Wire.endTransmission();
  if(ack != 0) { //We have a problem! 
    return(FAIL);
  }

  return(SUCCESS);
}

