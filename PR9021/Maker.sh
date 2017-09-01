#!/usr/bin/env bash
dir='dirname $0' # get the directory this script is in
echo ""
echo "Hi - So you're ready to flash that radio of yours? Awesome, let's do it!"
echo ""
read -p "What frequency do you want your radio tuned to? [93.9]: " frequency
[ -z $frequency ] && frequency=93.9
echo "OK, we are tuning to ${frequency}."
echo ""
 
read -p "What programmer should we use? Tested versions include usbtiny, usbasp, avrisp2 [usbtiny]: " programmer
[ -z $programmer ] && programmer=usbtiny
echo "OK, we are tuning to $frequency, using a $programmer programmer."
echo ""
 
tempfile=`mktemp -t eeprom.XXXXXX`
echo $tempfile
if [ ! -f $tempfile ]; then
  echo "tempfile not found"
  exit 1
fi
 
# hexfile=`python ${dir}/eeprom.py -f $frequency -M -S > $tempfile`
hexfile=`python eeprom.py -f $frequency > $tempfile`
echo "contents of $tempfile:"
echo ""
cat $tempfile
 
avrcommand="avrdude -P usb -c $programmer -p attiny45 -e -U flash:w:pr.hex -U eeprom:w:$tempfile"
while true; do
  read -p "Great! Ready to upload '$avrcommand'? This will take a few seconds, make sure the Radio's potentiometer is on. [Y/n]: " yn
  case $yn in
    [Nn]*) echo "you should run $avrcommand"; exit 0;;
    [Yy]*) $avrcommand; break;;
    *) $avrcommand; break;;
  esac
done