#include <Wire.h>


#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BMP280.h>
#include <Adafruit_AHTX0.h>


#include <EasyNextionLibrary.h>

#include <math.h>

#include <Encoder.h>


// Pins for the encoder signals
#define ENCODER_CLK_PIN 2 // S1
#define ENCODER_DT_PIN 3  // S2
#define ENCODER_KEY_PIN 4 // KEY BUTTON


// Create an encoder object
Encoder myEncoder(ENCODER_CLK_PIN, ENCODER_DT_PIN);
EasyNex myNex(Serial);




// To track button press state
bool buttonPressed = false;

int32_t lastPosition = -999; 
const int32_t maxValue = 28; 

int32_t newPosition = 0; 


const uint16_t maxButtonCount = 2;

const uint16_t SELECTED_COLOR = 2016;
const uint16_t UNSELECTED_COLOR = 50712;

const char* buttonNames[maxButtonCount] = { "b0.bco", "b1.bco"};

uint8_t selectedPage = 0;


Adafruit_MPU6050 mpu;
Adafruit_BMP280 bmp;
Adafruit_AHTX0 aht;

//variables 
const int REFRESH_TIME = 100;     
unsigned long refresh_timer = millis();



void setup() 
{
  Serial.begin(9600);
  myNex.begin(9600);
  
  checkConnectIMUAndConfigure();
  initBMP280();

 

  pinMode(ENCODER_KEY_PIN, INPUT_PULLUP);

}

void loop() 
{


  // Get data from MPU6050
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  limitPotentiometer();



  // Check whether the potentiometer button has been pressed to go to the selected page
  if (digitalRead(ENCODER_KEY_PIN) == LOW && !buttonPressed) 
  {
    buttonPressed = true;  // Setting the click flag
    changePage(selectedPage);  // Go to the selected page
  }
  
  // Resetting the button flag when releasing it
  if (digitalRead(ENCODER_KEY_PIN) == HIGH) 
  {
    buttonPressed = false;
  }
  


  //updating the screen
  if ((millis() - refresh_timer) > REFRESH_TIME) 
  { 

    // show axis oY on screen
    degres();
    
    //color button
    updateButtonColorsByPotValue(newPosition);

    readAndDisplayBMP280Data();
    readAndDisplayAHT20Data();

    //update the timer
    refresh_timer = millis(); 
  }
}