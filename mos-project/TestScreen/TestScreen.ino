#include "EasyNextionLibrary.h" // Include EasyNextionLibrary

EasyNex myNex(Serial);   // Create an object of EasyNex class with the name < myNex >
                        // Set as parameter the Hardware Serial you are going to use
                        
uint16_t voltage;  // a variable to store the reading
                 // for simplicity Reasons we do not use float and we are going to take the measure in millivolts
 

const int REFRESH_TIME = 100;           // time to refresh the Nextion page every 100 ms
unsigned long refresh_timer = millis();  // timer for refreshing Nextion's page



void setup() {

  pinMode(A0, INPUT); // set A0 pin as INPUT
 
  myNex.begin(9600); // Begin the object with a baud rate of 9600
                     // If no parameter was given in the begin(), the default baud rate of 9600 will be used

}

void loop() {
  
  if((millis()-refresh_timer) > REFRESH_TIME){ //IMPORTANT do not have serial print commands in the loop without a delay
                                              // or an if statement with a timer condition like this one.
                                              
      int tempData = analogRead(A0);  // Read the analog pin
      voltage = map(tempData, 0, 1024, 0, 5000); // same like: voltage = analogRead(A0)*5000/1024
    
      /* We Re-map the value of tempData from 0-1024 (steps) to 0-5000 millivolt
       * connect the pins of a Potentiometer on A0 pin, 5v (5000 millivolt) and GND. Outer pins to 5v and GND, middle pin to A0
       * https://www.arduino.cc/en/tutorial/potentiometer
       * Turn it over and read values from 0 to 5000 millivolts
       */
               
      myNex.writeNum("n1.val", voltage); // Nvoltage.val is a variable that we have create on Nextion.        
                                              // we send the value of the voltage variable on Arduino
                             // you can use the same name for variables on Nextion for easy recognition with a capital N in front
                             // Avoid the use of big variable names as every character is one byte to serial. 
                             // In here we use big names for the sake of example.
      refresh_timer = millis();  // Set the timer equal to millis, create a time stamp to start over the "delay"
     
    }       
}