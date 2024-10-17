#include "EasyNextionLibrary.h"

EasyNex myNex(Serial);

const int REFRESH_TIME = 100;           
unsigned long refresh_timer = millis(); 

const uint16_t maxButtonCount = 5;


void setup() 
{
  Serial.begin(9600);
  myNex.begin(9600);

  
  pinMode(ENCODER_KEY_PIN, INPUT_PULLUP);  

}


void loop() 
{
  
  
  limitPotentiometer();

  
  if ((millis() - refresh_timer) > REFRESH_TIME) 
  { 
   
    selectedButtonOnDisplay(newPosition);


  refresh_timer = millis();  
  }

}


void selectedButtonOnDisplay(uint32_t selectPosition)
{
  currentSelectedButton = selectPosition;

  for (int i = 0; i < maxButtonCount; i++) 
  {

    String buttonName = "b" + String(i) + ".bco";
    
    if (i == currentSelectedButton) 
    {
      myNex.writeNum(buttonName.c_str(), 2016); 
    } 
    else 
    {
      myNex.writeNum(buttonName.c_str(), 50712); 
    }
  }
}

/*
void showOnDisplay()
{
  myNex.writeNum("n0.val", newPosition);
  myNex.writeNum("h0.val", newPosition);
}
*/
