#include <Encoder.h>

// Pins for the encoder signals
#define ENCODER_CLK_PIN 2 // S1
#define ENCODER_DT_PIN 3 // S2
#define ENCODER_KEY_PIN 4 // KEY BUTTON 

// Create an encoder object
Encoder myEncoder(ENCODER_CLK_PIN, ENCODER_DT_PIN);

// To track button press state
bool buttonPressed = false;

uint32_t lastPosition = -999; 

const uint16_t maxValue = 1024;

uint32_t newPosition = 0;

void limitPotentiometer()
{
    
    newPosition = myEncoder.read();

    
    if (newPosition != lastPosition) {
      
      
        if (newPosition < 0) {
            newPosition = maxValue - 1;  
            myEncoder.write(newPosition);  
        } 
       
        else if (newPosition >= maxValue) {
            newPosition = 0;  
            myEncoder.write(newPosition);  
        }

 
        Serial.print("New Position: ");
        Serial.println(newPosition);
        
       
        lastPosition = newPosition;
    }
}
/*
// Check if the button is pressed
  if (digitalRead(ENCODER_KEY_PIN) == LOW) {
    if (!buttonPressed) {
      buttonPressed = true;
      Serial.println("Button Pressed!");
    }
  } else {
    buttonPressed = false;
  }
*/