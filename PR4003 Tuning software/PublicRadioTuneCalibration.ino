/*
A tuning routine for the Public Radio to test for station freqs, by Zach Dunham.
Averaging function from Tom Igoe at http://arduino.cc/en/Tutorial/Smoothing
www.thepublicrad.io
***steps***
connect A0 to tuning pot, center pin 2
connect grounds 
connect radio pwr to 3.3V 
run sketch, open serial monitor and tune until the appropriate station
appears in the serial monitor
*/
const int numReadings = 5;        // number of readings, indicies for the averaging array   
float readings[numReadings];      // the readings from the analog input
int index = 0;                  // the index of the current reading
float total = 0;                  // the running total
float average = 0;                // the average

int PotPin2 = A0;               //input from the radio, wiper of the tune pot

float tune = 0.00;             //variable for station id 


void setup() {
  for (int thisReading = 0; thisReading < numReadings; thisReading++)
    readings[thisReading] = 0;
  // initialize serial communication at 9600 bits per second:
  Serial.begin(9600);
}

// the loop routine runs over and over again forever:
void loop() {
  // read the input on analog pin 0:
  tune = analogRead(PotPin2); //read the pot value 
  tune = constrain(tune, 0.0, 212.0); //constrain it's range 
  tune = mapfloat(tune, 0.0, 212.0, 86.33, 108.43); //map the potpin2 readings into the FM range with floats
  tune = averagestation(tune); //smooth the readings with the average function
  Serial.println(tune); // print the station
  delay(100);        
}


//maps from floating point number to floating point number 
float mapfloat(float x, float in_min, float in_max, float out_min, float out_max)
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

//taken from the great, Tom Igoe!
//this averages the readings to help smooth data in the serial monitor
float averagestation(float tune){
  // subtract the last reading:
  total= total - readings[index];         
  // read from the sensor:  
  readings[index] = tune; 
  // add the reading to the total:
  total= total + readings[index];       
  // advance to the next position in the array:  
  index = index + 1;                    

  // if we're at the end of the array...
  if (index >= numReadings)              
    // ...wrap around to the beginning: 
    index = 0;                           

  // calculate the average:
  average = total / numReadings;         
  // send it to the computer as ASCII digits
  return (average);   
  delay(1);        // delay in between reads for stability            
}

