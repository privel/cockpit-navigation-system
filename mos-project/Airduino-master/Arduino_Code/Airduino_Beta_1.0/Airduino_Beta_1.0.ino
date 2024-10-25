/*****************************************************************************
* Open Source Backup_PFD code.  Written by Don Morris.  Provided for 
* situational awareness only.  Please do not use as primary flight or
* navigation instruments.  Use of program is waiver of any and all 
* liability.
* 
* Loosely based on the paper "2 Fly with Raspberry Pi" by Don Morris.
* Made in cooperation with Chongwen Chen.
* Copyright (C) 2018 Don Morris and Chongwen Chen.    
* Visit www.theopencockpit.org for more details, including copyright details.
*****************************************************************************/


#include "SPI.h"
#include "Wire.h"
#include "FT_NHD_43CTP_SHIELD.h"
#include <Encoder.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BMP280.h>

/*****************************************************************************
 * 
 * This section contains "define"statements that are used to configure the 
 * unit for your particular application.  Unless you are a programmer, this
 * is the only part of the code that you should change.  Please view the 
 * manual for details beyond the comments.  You change the numbers and 
 * true or false values only.  Do not change anything else.
 * 
******************************************************************************/

#define MaxVertSpeed 4000 // will determine the size of the VSI scale on the indicator.
#define PrefSpeedUnits 2  //1 is kts, 2 is mph, 3 is km/h
#define PrefPressureUnits 1 // 1 is inHG, 2 is mmHg, 3 is hPa
#define PrefAltUnits 1 // 1 is ft, 2 is meters
#define WAStart 45  // the low side of the airspeed indicator white arc in PrefSpeedUnits
#define WAEnd 100   // the high side of the airspeed indicator white arc
#define GAStart 52  // the low side of the airspeed indicator green arc
#define GAEnd 116   // the high side of the airspeed indicator green arc
#define BLine 0     // the airspeed indicator blue line.  Use 0 if you do not need a blue line.
#define RLine 142   // the airspeed indicator red line.
#define MaxSpeed 250  // this is the top number on the gauge.  This should be a little higher than your red line.
#define SpeedTics 50  // this determines how manyy ticks will print out on the speed slider
#define SpeedSlider false  //false will give gauge like readout.  True gives ticker type readout
#define HeadingNum false  // Black box with the heading on the DG strip
#define NumAverageTrend 10 // Higher numbers are less jittery.  Lower numbers are more responsive.  Applies to VSI and Rate of Turn
#define ECPD 2 // number of ecoder clicks per detent.  Probably 1, 2, or 4.
#define StartMenu 6  // defines where the cursor starts.  6 is the Altimeter Adjust
#define StartBrightness 255  // how bright the unit is on power-on.  255 is full bright.  0 is completely dark.  
#define StartHeadingBug 360  // where the heading bug shows up.
#define StartingPressure 29.92  // Put this as the pressure in your preferred unit system.  29.92, 1013.25, 760.0; 
#define StaticCalibrationOffset 3  // This is a value used to calibrate your static BMP280.  Start with 0, and make tiny changes. Each 1 you add adds about 25 feet (8 meters).  Calibrate this one first.
#define AirspeedCalibrationOffset 0  // This is a value used to calibrate your airspeed.   Start with 0.  Calibrate this one after the Static. Each one is 1 mph.
#define CenterDisplayType 1 // 1 is yellow line moving with the horizon.  2 Yellow lines that stay level.  3 gives yellow plane.
#define dynamicsky true // false does not change the display color based on pitch.  true does.  Sky gets deeper blue as you point up.
#define dynamicground false // false does not change the display color based on pitch.  true does.  Groiund gets reddish as you point down.
#define PermRollOffset 0 // this is used to calibrate for AHRS offsets and mounting.  (Not used in this version.)
#define PermPitchOffset 0 // this is used to calibrate for AHRS offsets and mounting.  (Not used in this version.)  Use calibration routine instead.
#define PermCompassOffset 0 // this is used to calibrate for AHRS offsets and mounting. (Not used in this version.)

// Used in Serial Data Transfer. This binaryInt allows the sensor to recieve floating point numbers and automatically decode them
typedef union {
 float floatingPoint;
 byte binary[4];
} binaryFloat;
binaryFloat CH;
binaryFloat CP;
binaryFloat CR;
byte CalibrationStatus = 0;
// end data types and variables used in serial transfer of data .


int BotWhA; 
int TopWhA; 
int BotGA; 
int TopGA; 
int RL;
int BL; 
int NumTics;
int TicMap;
int HalfSpeedTics;
int CurrentSpeed;
float CurrentRoll;  // angle in radians from -Pi to + Pi.  - is to the left
float CurrentPitch; // angle in radians from -Pi to + Pi.  - is down
float CurrentHeading = 0;  //  this will actually come from the IMU unit, but we will start with a number
float CurrentRateOfTurn = -0.0115;  // this is the number of radians per second.  Number is for test purposes only, and cooresponds to a 2 minute left turn
float CurrentRateOfClimb = 0;
float CurrentAltitude = 400;
float CurrentGs = 1;
float PreviousHeading = 0;
float PreviousAltitude = 0;
float CurrentPressure = StartingPressure;  // needed to switch on the fly.
int HeadingBug = StartHeadingBug;
int DispDegs;
int brightness = StartBrightness;  //Max = 255, min = 0
int SpeedUnits = PrefSpeedUnits;
int AltUnits = PrefAltUnits;
int PressureUnits = PrefPressureUnits;
int PressureDigits[] = {2,9,9,2};
long OldEncoderPosition = -999;
long NewEncoderPosition = -999;
int Menu = StartMenu;  // 1 is brightness, 2 is speed units, 3 is bug, 4 is trim, 5 is pressure, 6 is pressure units, 7 is alt units
boolean MenuSelected = false;
boolean ButtonDown = false;
boolean ButtonReleased = false;
int EncMove = 0;
int EncMult = 1;
unsigned long currentMicros; // = micros();
unsigned long previousMicros;
unsigned long intervalMicros;
int eraseme = 0;
boolean SensorError = false;
int ErrorStreak = 0;
float RollOffset = PermRollOffset;
float PitchOffset = PermPitchOffset;
float CompassOffset = PermCompassOffset;

Encoder myEnc(19, 18);  // these pins are interrupt capable.  Reverse these if your encoder moves the menus the wrong way.
Adafruit_BMP280 Static; // I2C communication with the static
Adafruit_BMP280 Pitot; // I2C communication with the static



/* Global object for transport */
FT801IMPL_SPI FTImpl(FT_CS_PIN,FT_PDN_PIN,FT_INT_PIN);


/* Api to bootup FT801, verify FT801 hardware and configure display/audio pins */
/* Returns 0 in case of success and 1 in case of failure */
int16_t BootupConfigure(){
	uint32_t chipid = 0;
	FTImpl.Init(FT_DISPLAY_RESOLUTION);//configure the display to the WQVGA

	delay(20);//for safer side
	chipid = FTImpl.Read32(FT_ROM_CHIPID);
	
	/* Identify the chip */
	if(FT801_CHIPID != chipid)
	{
		Serial.print("Error in chip id read ");
		Serial.println(chipid,HEX);
		return 1;
	}
	
	/* Set the Display & audio pins */
	FTImpl.SetDisplayEnablePin(FT_DISPENABLE_PIN);
    FTImpl.SetAudioEnablePin(FT_AUDIOENABLE_PIN); 
	FTImpl.DisplayOn(); 
	FTImpl.AudioOn(); 
	return 0;
}

void FlipNum(int Xloc, int Yloc, int TopDigit, int BottomDigit, int Twentiethsflip)
{  // This function is called from within a drawing list, and paints a 10 by 15 block with a rolling number display
   // location is upper right referenced.  The requested area of the screen is blacked and blanked.  This cannot be 
   // used for more than two digits at a time. Twentiethsflip is the amount that the numbers have slid - as in 5/20ths
   // 0/20ths prints just the TopDigit.  20/20ths prints just the BottomDigit 
   // use for the TopDigit or the BottomDigit if you want them to show up as blank.   
   int NumDigits = 1;
   if ((BottomDigit >= 10) or (TopDigit >= 10)) NumDigits = 2;
   FTImpl.ColorRGB(0,0,0);              // set the color to Black
   FTImpl.ClearStencil(0);              // set the stencil to blank
   FTImpl.StencilOp(FT_INCR,FT_INCR);   // set the stencil operation to mask the blank
   FTImpl.Begin(FT_RECTS);              // get ready to make the box
   FTImpl.Vertex2ii(Xloc,Yloc,0,0);  FTImpl.Vertex2ii(Xloc - (10 * NumDigits),Yloc + 16,0,0);  // then draw the box that blanks the space and DEFINES THE STENCIL ....
   FTImpl.StencilOp(FT_KEEP,FT_KEEP);   // and we are done building the stencil.  Clear the stencil creating function
   FTImpl.ColorRGB(255,255,255);        // set the color to White
   FTImpl.StencilFunc(FT_GREATER, 0, 255);  // and olnly draw in the stencil
   if (TopDigit != -1) FTImpl.Cmd_Number(Xloc, Yloc-Twentiethsflip-1, 27, FT_OPT_RIGHTX, TopDigit);        // print the part of the numbers that shows up in the box
   if (BottomDigit != -1) FTImpl.Cmd_Number(Xloc, Yloc+19-Twentiethsflip, 27, FT_OPT_RIGHTX, BottomDigit); // print the part of the numbers that shows up in the box
   FTImpl.StencilFunc(FT_ALWAYS, 1, 255); //go back to ignoring the stencil
}

void DrawScreen()
{
//**  FTImpl.SetCTouchMode(FT_CTOUCH_MODE_EXTENDED);  //set mode to extended for FT801    
    int16_t xoffset,yoffset;
    xoffset = ((FT_DISPLAYWIDTH - 480)/2);
    yoffset = ((FT_DISPLAYHEIGHT - 272)/2);

//**  Begin building angle gauge stencil  
      FTImpl.DLStart();
             // this should set the screen brightness
             FTImpl.Write(REG_PWM_DUTY,map(brightness, 0, 255, 0, 100));

      FTImpl.ClearColorRGB(0,0,0);
      FTImpl.Clear(1,0,1);
      FTImpl.ClearStencil(0);
      FTImpl.StencilOp(FT_INCR,FT_INCR);
      // Draw the rays 
      FTImpl.LineWidth(6);
      FTImpl.Begin(FT_LINES);
      FTImpl.Vertex2ii(145,70,0,0);  FTImpl.Vertex2ii(157,81,0,0);
      FTImpl.Vertex2ii(165,61,0,0);  FTImpl.Vertex2ii(171,67,0,0);
      FTImpl.Vertex2ii(178,43,0,0);  FTImpl.Vertex2ii(186,56,0,0);
      FTImpl.Vertex2ii(200,41,0,0);  FTImpl.Vertex2ii(203,49,0,0);
      FTImpl.Vertex2ii(220,36,0,0);  FTImpl.Vertex2ii(221,44,0,0);
      FTImpl.Vertex2ii(260,36,0,0);  FTImpl.Vertex2ii(259,44,0,0);
      FTImpl.Vertex2ii(280,41,0,0);  FTImpl.Vertex2ii(277,49,0,0);
      FTImpl.Vertex2ii(302,43,0,0);  FTImpl.Vertex2ii(294,56,0,0);
      FTImpl.Vertex2ii(315,61,0,0);  FTImpl.Vertex2ii(309,67,0,0);
      FTImpl.Vertex2ii(335,70,0,0);  FTImpl.Vertex2ii(323,81,0,0);
      // Draw a circle centered on 240, 150. 
      FTImpl.ColorRGB(255,255,255);//set the color of the string to WHITE color
      FTImpl.PointSize(110*16);//circle
      FTImpl.Begin(FT_POINTS);
      FTImpl.Vertex2ii(240,150,0,0);
      FTImpl.StencilOp(FT_ZERO,FT_ZERO);
      FTImpl.ColorRGB(0,0,0);//set the color of the string to BLACK color
      FTImpl.PointSize(107 * 16);//inner circle
      FTImpl.Vertex2ii(240,150,0,0);
      //Trim the bottom of the circle off 
      FTImpl.Begin(FT_RECTS);
      FTImpl.Vertex2ii(120,82,0,0);    FTImpl.Vertex2ii(360,270,0,0); 
      FTImpl.StencilOp(FT_KEEP,FT_KEEP);

//**  Stencil complete.  Clear Screen to Sky Color
      int darkensky = 0;
      if (dynamicsky) darkensky = (int)(70*sin(CurrentPitch));  // darkensky won't change the sky's color unless dynamicsky is chosen
      if (darkensky > 0) darkensky = 0;
      FTImpl.ClearColorRGB(150+darkensky,150+darkensky,255);
      FTImpl.Clear(1,0,0);  // the zero leaves the stencil in place
   





//** Math work in progress.  Draw in a red line.  First, use only shallow angles and assume rightside up
      FTImpl.LineWidth(12);
      int darkenground = 0;
      if (dynamicground) darkenground = (int)(60*sin(CurrentPitch));  // darkenground won't change the ground's color unless dynamicground is chosen
      if (darkenground < 0) darkenground = 0;
      FTImpl.ColorRGB(120+darkenground,80-(darkenground/2),20-(darkenground/6)); //);//set the color to brown, with a redish adjustment as you head down,  
      int LS; // left side intersect
      int RS; // right side intersect
      int LO = 0; // Left Offset - used when the line does not extend across the screen  
      int RO = 0; // Right Offset - used when the line does not extend across the screen
      float CXP; // Center X Position
      float CYP; // Center Y Pos
      float TanCurrentRoll = tan(CurrentRoll);  // Do math as few times as possible
      float SinCurrentRoll = sin(CurrentRoll);
      float CosCurrentRoll = cos(CurrentRoll);
      CXP = 240-SinCurrentRoll*((CurrentPitch/.0175)*3.6);
      CYP = 150-CosCurrentRoll*((CurrentPitch/.0175)*3.6);

      if ((CurrentRoll >= -0.7853) and (CurrentRoll <= 0.7853))  // the aircraft is within 45 degrees of upright
        {
          if ((CXP >= 0) and (CXP <= 480)) // the center of the line is on the screen
            {

              LS = CYP+(CXP*TanCurrentRoll);
              RS = CYP-((480-CXP)*TanCurrentRoll); 
              if (LS < 0) // The line begins above the display. 
                {LO = CXP-((CYP*CXP)/(CYP-LS)); LS = 0;} // similar triangles
              else if (LS > 272) // The line begins below the display.
                {LO = CXP-((CXP*(272-CYP))/(LS-CYP)); LS = 272;} // similar triangles
              if (RS < 0) // The line ends above the display
                {RO = (480-CXP)-(((480-CXP)*CYP)/(CYP-RS)); RS = 0;} // similar triangles
              else if (RS > 272) // The line ends below the display
                {RO = (480-CXP)-(((480-CXP)*(270-CYP))/(RS-CYP)); RS = 272;} // similar triangles     
              FTImpl.Begin(FT_EDGE_STRIP_B); // the use of 4 points ensures complete usability
                FTImpl.Vertex2ii(0,LS,0,0);
                FTImpl.Vertex2ii(0+LO,LS,0,0);
                FTImpl.Vertex2ii(480-RO,RS,0,0); 
                FTImpl.Vertex2ii(480,RS,0,0); 

               
            }
        }

// VERY IMPORTANT.  The ground should not show up if the sensor is offline.  
      if (ErrorStreak > 16) // set to about the number of frames per second...
        {
          FTImpl.LineWidth(24);
          FTImpl.ClearColorRGB(0,0,0); // set the color to black
          FTImpl.Clear(1,0,0);  // the zero leaves the stencil in place, but clears the screen to black
          FTImpl.ClearColorRGB(255,0,0); // set the color to red
          FTImpl.Begin(FT_LINES);
          FTImpl.Vertex2ii(140,20,0,0);  FTImpl.Vertex2ii(340,252,0,0);   // Draw the gigantic red x that says not to navigate...
          FTImpl.Vertex2ii(140,252,0,0);  FTImpl.Vertex2ii(340,20,0,0); 
        }

      
//**  Reprint Angle Gauge from Stencil
      FTImpl.StencilFunc(FT_NOTEQUAL, 0, 255); //only operate on the stencil
      FTImpl.ColorRGB(255,255,255);//set the color to White 
      FTImpl.PointSize(126*16);//circle
      FTImpl.Begin(FT_POINTS);
      FTImpl.Vertex2ii(240,150,0,0);
      FTImpl.StencilFunc(FT_ALWAYS, 1, 255); //clear the stencil function

//**  Add angle numbers to the stencil gauge
      FTImpl.Cmd_Number(140 , 62, 26, FT_OPT_CENTER, 50);
      FTImpl.Cmd_Text(160 , 36, 26, FT_OPT_CENTER, "2 Min (30)");
      FTImpl.Cmd_Number(305 , 36, 26, FT_OPT_CENTER, 30);
      FTImpl.Cmd_Number(340 , 62, 26, FT_OPT_CENTER, 50);
         
//      FTImpl.ColorRGB(255,255,255);//set the color of the string back to WHITE color
      FTImpl.LineWidth(16);

 

      
      


//** Given CurrentHeading (float), overlay the DG Strip
      //   Display is 480 pixels wide, so the center is at 240.  5 pixels is one degree
      // Draw the line strip line  
      FTImpl.ColorRGB(255,255,255);//set the color of the string to WHITE color
      FTImpl.Begin(FT_LINES);
      FTImpl.Vertex2ii(75,22,0,0);  FTImpl.Vertex2ii(405,22,0,0);
      int Nearest5 = round(CurrentHeading/5) * 5;
      int Nearest10 = round(CurrentHeading/10) * 10;
      boolean NumTic = false;
      if (Nearest5 == Nearest10) NumTic = true;   
      for (int degs = Nearest5 - 30; degs <= (Nearest5 + 30); degs = degs + 5)
         {
            DispDegs = degs;
            if (DispDegs > 360) DispDegs = DispDegs - 360;
            if (DispDegs < 1) DispDegs = DispDegs + 360;
            if (NumTic) // This one is divisible by 10, and gets a number and a tic
              {
                NumTic = false;
                FTImpl.Begin(FT_LINES);
                FTImpl.Vertex2ii((degs-(CurrentHeading-40))*5+40,25,0,0);
                FTImpl.Vertex2ii((degs-(CurrentHeading-40))*5+40,22,0,0);
                FTImpl.Cmd_Number( (degs-(CurrentHeading-40))*5+40, 13, 26, FT_OPT_CENTER, DispDegs);   
              }
            else        // This one is between numbers, and gets a tic only
              {
                FTImpl.Begin(FT_LINES);
                FTImpl.Vertex2ii((degs-(CurrentHeading-40))*5+40,25,0,0);
                FTImpl.Vertex2ii((degs-(CurrentHeading-40))*5+40,15,0,0);
                NumTic = true;  
              }
         }
    // Now draw the white arrow for no rate of turn
      FTImpl.Begin(FT_LINE_STRIP);
      FTImpl.Vertex2ii(240,0,0,0);
      FTImpl.Vertex2ii(240,41,0,0);
      FTImpl.Vertex2ii(238,29,0,0);
      FTImpl.Vertex2ii(242,29,0,0);
      FTImpl.Vertex2ii(240,41,0,0);
      FTImpl.Vertex2ii(236,27,0,0);
      FTImpl.Vertex2ii(240,31,0,0);
      FTImpl.Vertex2ii(244,27,0,0);      
      FTImpl.Vertex2ii(240,41,0,0);


    // if option is selected, draw the Heading Number
      if (HeadingNum) 
        {
          FTImpl.Begin(FT_RECTS);
          FTImpl.ColorRGB(160,160,160);//set the color to White
          FTImpl.Vertex2ii(240-18, 0);  FTImpl.Vertex2ii(240+18,15);
          FTImpl.ColorRGB(0,0,0);//set the color to Black
          FTImpl.Vertex2ii(240-16, 0);  FTImpl.Vertex2ii(240+16,13);
          FTImpl.ColorRGB(255,255,255);//set the color to White
          FTImpl.Cmd_Number(240, 6, 27, FT_OPT_CENTER, CurrentHeading);
        }

    // Draw heading Bug if it shows up, and the Bug setting Regardless  
      FTImpl.Cmd_Text(43, 260, 27, FT_OPT_CENTER, "*   Bug:");
      FTImpl.Cmd_Number(87, 260, 27, FT_OPT_CENTER, HeadingBug);  
      FTImpl.ColorRGB(255,165,4);//set the color to Orange
      //FTImpl.Cmd_Number(159, 260, 27, FT_OPT_CENTER, difference);
      int BugCalc = HeadingBug;
      if ((CurrentHeading < 41) and (HeadingBug > 319)) BugCalc = BugCalc - 360;
      if ((CurrentHeading > 319) and (HeadingBug < 41)) BugCalc = BugCalc + 360;
      if (abs(CurrentHeading-BugCalc) < 33) // it should be on the strip, so lets draw this sucker! - was 41
        {
          FTImpl.Begin(FT_LINES);
          FTImpl.LineWidth(32);
          FTImpl.Vertex2ii(240-((CurrentHeading-BugCalc)*5),23,0,0);
          FTImpl.Vertex2ii(240-((CurrentHeading-BugCalc)*5),2,0,0);
        }
                  
//** End overlay the DG Strip

//** Overlay Brightness Slider
      // remember brightness is 0-255
      FTImpl.ColorRGB(128,128,128);//set the color of the string to GREY color
      FTImpl.LineWidth(48);
      FTImpl.Begin(FT_LINES);
      FTImpl.Vertex2ii(20,235,0,0);  FTImpl.Vertex2ii(20,235-(255*.76),0,0); //255 scaled by 76%
      FTImpl.PointSize(10*16);//circle
      FTImpl.Begin(FT_POINTS);
      FTImpl.Vertex2ii(20,235-(brightness*.76),0,0); // this is the actual dot
      FTImpl.ColorRGB(255,255,255);//set the color of the string to WHITE color
      FTImpl.PointSize(8*16);//circle
      FTImpl.Vertex2ii(20,235-(brightness*.76),0,0); // this is the actual dot

      
      // include the Empty Battery Icon.  This is greyed out in this version, but in the next version with the board, it will 
      // be a live icon showing charging and discharging...
      FTImpl.ColorRGB(170,170,170);//set the color to GREY
      FTImpl.LineWidth(32);
      FTImpl.Begin(FT_RECTS);
      FTImpl.Vertex2ii(9,8,0,0);  FTImpl.Vertex2ii(32,17,0,0);  
      FTImpl.Vertex2ii(30,11,0,0);  FTImpl.Vertex2ii(35,14,0,0); 
      //FTImpl.ColorRGB(150,150,255);//set the color to SKY
      FTImpl.Vertex2ii(11,10,0,0);  FTImpl.Vertex2ii(30,15,0,0); 
      FTImpl.ColorRGB(170,170,170);//set the color to GREY
      // WILL NEED to read battery info on pin A2, charge status on pin ?, and charging status on pin ?  These are provided on the future board.
      FTImpl.Vertex2ii(13,12,0,0);  FTImpl.Vertex2ii(28,13,0,0); // for now, just put a greyed out spot in the icon.
      

            
//**  Overlay Speed Strip - this will likely change to a software selectable optional slider and ticker
      // print out units...
      switch (SpeedUnits) {  // overlay the Speed Units.  These can change on the fly...
        case 1: FTImpl.Cmd_Text(55, 17, 27, FT_OPT_CENTER, "kts"); break;
        case 2: FTImpl.Cmd_Text(55, 17, 27, FT_OPT_CENTER, "mph"); break;
        case 3: FTImpl.Cmd_Text(55, 17, 27, FT_OPT_CENTER, "km/h"); break; }
 
      if (!SpeedSlider)
        {// this is the gauge type speed readout
           // lay out the speed line
             FTImpl.LineWidth(16);
             FTImpl.Begin(FT_LINES);
             FTImpl.Vertex2ii(80,235,0,0);  FTImpl.Vertex2ii(80,40,0,0);
             // lay out the color bands
             // White Arc
             FTImpl.LineWidth(40);
             FTImpl.Vertex2ii(78,235-BotWhA,0,0);  FTImpl.Vertex2ii(78,235-TopWhA,0,0);
             // Yellow Line
             FTImpl.LineWidth(24);           
             FTImpl.ColorRGB(255,232,0);//set the color of the string to YELLOW color
             FTImpl.Vertex2ii(82,235-TopGA,0,0);  FTImpl.Vertex2ii(82,235-RL,0,0);
             // Green Line
             FTImpl.ColorRGB(0,185,0);//set the color of the string to Green color
             FTImpl.Vertex2ii(82,235-TopGA,0,0);  FTImpl.Vertex2ii(82,235-BotGA,0,0);
             // Red Line
             FTImpl.LineWidth(24);           
             FTImpl.ColorRGB(185,0,0);//set the color of the string to Red color
             FTImpl.Vertex2ii(84,235-RL,0,0);  FTImpl.Vertex2ii(76,235-RL,0,0);
             // Blue Line
             if (BL > 5) { // only draw it if we need it
             FTImpl.LineWidth(24);           
             FTImpl.ColorRGB(32,32,185);//set the color of the string to Blue color
             FTImpl.Vertex2ii(84,235-BL,0,0);  FTImpl.Vertex2ii(76,235-BL,0,0); }
           // lay out the tic marks and scale text
             int count = 0;
             FTImpl.LineWidth(16);           
             FTImpl.ColorRGB(255,255,255);//set the color of the string to White color
             for (int tics = 0; tics <= NumTics; tics++)  
               {
                 TicMap = map(tics*SpeedTics,0,MaxSpeed,0,195);  // 195 is the number of pixels it displays
                 FTImpl.Begin(FT_LINES); 
                 FTImpl.Vertex2ii(81, 235-TicMap);  FTImpl.Vertex2ii(74,235-TicMap); 
                 FTImpl.Cmd_Number(71, 227-TicMap, 26, FT_OPT_RIGHTX, tics*SpeedTics);
                 if (tics != 0) {FTImpl.Begin(FT_LINES); FTImpl.Vertex2ii(80, 235-TicMap+HalfSpeedTics); FTImpl.Vertex2ii(70,235-TicMap+HalfSpeedTics);}
               }
           // lay out the current speed tic and number
             TicMap = map(CurrentSpeed,0,MaxSpeed,0,195);  // 195 is the number of pixels it displays
             FTImpl.Vertex2ii(89, 235-TicMap);  FTImpl.Vertex2ii(78,235-TicMap);
             FTImpl.Begin(FT_RECTS);
             if (CurrentSpeed > 99) 
               {
                  FTImpl.ColorRGB(255,255,255);//set the color to White
                  FTImpl.Vertex2ii(90, 226-TicMap);  FTImpl.Vertex2ii(125,243-TicMap);
                  FTImpl.ColorRGB(0,0,0);//set the color to Black
                  FTImpl.Vertex2ii(92, 228-TicMap);  FTImpl.Vertex2ii(123,241-TicMap);
               }
             else if (CurrentSpeed > 9) 
               {
                  FTImpl.ColorRGB(255,255,255);//set the color to White
                  FTImpl.Vertex2ii(90, 226-TicMap);  FTImpl.Vertex2ii(113,243-TicMap);
                  FTImpl.ColorRGB(0,0,0);//set the color to Black
                  FTImpl.Vertex2ii(92, 228-TicMap);  FTImpl.Vertex2ii(111,241-TicMap);
               }
             else  
               {
                  FTImpl.ColorRGB(255,255,255);//set the color to White
                  FTImpl.Vertex2ii(90, 226-TicMap);  FTImpl.Vertex2ii(102,243-TicMap);
                  FTImpl.ColorRGB(0,0,0);//set the color to Black
                  FTImpl.Vertex2ii(92, 228-TicMap);  FTImpl.Vertex2ii(100,241-TicMap);
               }
                
             FTImpl.ColorRGB(255,255,255);//set the color to White
             FTImpl.Cmd_Number(92, 225-TicMap, 27, 0, CurrentSpeed);   
        }
      else
        {
          // this routine still needs to be written, but I am not in a hurry for it!
        }

//**  Overlay the Angle Gauge, based on CurrentRoll
      // this will involve angle calculations.

      // the Center Strips, which can be one of three styles.
      int XOffset1 = 0; int YOffset1 = 0; int XOffset2 = 0; int YOffset2 = 0; int XOffset3 = 0; int YOffset3 = 0;
      if (CenterDisplayType == 1) // this is the standard yellow lines move with the horizon
        {  
          FTImpl.ColorRGB(255,255,0);//set the color to Yellow
          FTImpl.LineWidth(24);
          FTImpl.Begin(FT_LINES);
          XOffset2 = 15*sin(CurrentRoll+(-1.5708));
          YOffset2 = 15*cos(CurrentRoll+(-1.5708));
          XOffset3 = 45*sin(CurrentRoll+(-1.5708));
          YOffset3 = 45*cos(CurrentRoll+(-1.5708));     
          FTImpl.Vertex2ii(240,150,0,0);  FTImpl.Vertex2ii(240,150,0,0); // center dot
          FTImpl.Vertex2ii(240-XOffset2,150-YOffset2,0,0);  FTImpl.Vertex2ii(240-XOffset3,150-YOffset3,0,0); // left
          FTImpl.Vertex2ii(240+XOffset2,150+YOffset2,0,0);  FTImpl.Vertex2ii(240+XOffset3,150+YOffset3,0,0); // right
        }
      else if (CenterDisplayType == 2) // the yellow lines stay straight
        {  
          FTImpl.ColorRGB(0,0,0);//set the color to black
          FTImpl.LineWidth(80);
          FTImpl.Begin(FT_LINES);
          FTImpl.Vertex2ii(240,150,0,0);  FTImpl.Vertex2ii(240,150,0,0); // center dot
          FTImpl.LineWidth(48);
          FTImpl.Vertex2ii(225,150,0,0);  FTImpl.Vertex2ii(195,150,0,0); // left
          FTImpl.Vertex2ii(255,150,0,0);  FTImpl.Vertex2ii(285,150,0,0); // left         
          FTImpl.Vertex2ii(225,150,0,0);  FTImpl.Vertex2ii(225,157,0,0); // right
          FTImpl.Vertex2ii(255,150,0,0);  FTImpl.Vertex2ii(255,157,0,0); // right
          FTImpl.ColorRGB(255,255,0);//set the color to Yellow
          FTImpl.LineWidth(64);
          FTImpl.Begin(FT_LINES);
          FTImpl.Vertex2ii(240,150,0,0);  FTImpl.Vertex2ii(240,150,0,0); // center dot
          FTImpl.LineWidth(32);
          FTImpl.Vertex2ii(225,150,0,0);  FTImpl.Vertex2ii(195,150,0,0); // left
          FTImpl.Vertex2ii(255,150,0,0);  FTImpl.Vertex2ii(285,150,0,0); // left         
          FTImpl.Vertex2ii(225,150,0,0);  FTImpl.Vertex2ii(225,157,0,0); // right
          FTImpl.Vertex2ii(255,150,0,0);  FTImpl.Vertex2ii(255,157,0,0); // right
        }
        else // CenterDisplayType == 3 -  basically does both
        {
          FTImpl.ColorRGB(255,255,255);//set the color to white
          FTImpl.LineWidth(16);
          FTImpl.Begin(FT_LINES);
          XOffset2 = 15*sin(CurrentRoll+(-1.5708));
          YOffset2 = 15*cos(CurrentRoll+(-1.5708));
          XOffset3 = 45*sin(CurrentRoll+(-1.5708));
          YOffset3 = 45*cos(CurrentRoll+(-1.5708));     
          FTImpl.Vertex2ii(240,150,0,0);  FTImpl.Vertex2ii(240,150,0,0); // center dot
          FTImpl.Vertex2ii(240-XOffset2,150-YOffset2,0,0);  FTImpl.Vertex2ii(240-XOffset3,150-YOffset3,0,0); // left
          FTImpl.Vertex2ii(240+XOffset2,150+YOffset2,0,0);  FTImpl.Vertex2ii(240+XOffset3,150+YOffset3,0,0); // right
          FTImpl.ColorRGB(0,0,0);//set the color to black
          FTImpl.LineWidth(80);
          FTImpl.Begin(FT_LINES);
          FTImpl.Vertex2ii(240,150,0,0);  FTImpl.Vertex2ii(240,150,0,0); // center dot
          FTImpl.LineWidth(32);
          FTImpl.Vertex2ii(240,150,0,0);  FTImpl.Vertex2ii(240,140,0,0); // tail
          FTImpl.LineWidth(48);
          FTImpl.Vertex2ii(225,150,0,0);  FTImpl.Vertex2ii(195,150,0,0); // left
          FTImpl.Vertex2ii(255,150,0,0);  FTImpl.Vertex2ii(285,150,0,0); // left         
          FTImpl.Vertex2ii(225,150,0,0);  FTImpl.Vertex2ii(225,157,0,0); // right
          FTImpl.Vertex2ii(255,150,0,0);  FTImpl.Vertex2ii(255,157,0,0); // right
          FTImpl.ColorRGB(255,255,0);//set the color to Yellow
          FTImpl.LineWidth(64);
          FTImpl.Begin(FT_LINES);
          FTImpl.Vertex2ii(240,150,0,0);  FTImpl.Vertex2ii(240,150,0,0); // center dot
          FTImpl.LineWidth(16);
          FTImpl.Vertex2ii(240,150,0,0);  FTImpl.Vertex2ii(240,140,0,0); // tail
          FTImpl.LineWidth(32);
          FTImpl.Vertex2ii(225,150,0,0);  FTImpl.Vertex2ii(195,150,0,0); // left
          FTImpl.Vertex2ii(255,150,0,0);  FTImpl.Vertex2ii(285,150,0,0); // left         
          FTImpl.Vertex2ii(225,150,0,0);  FTImpl.Vertex2ii(225,157,0,0); // right
          FTImpl.Vertex2ii(255,150,0,0);  FTImpl.Vertex2ii(255,157,0,0); // right          
        }
        //20 degree marks
        FTImpl.ColorRGB(255,255,255);//set the color to White
        FTImpl.LineWidth(16);
        XOffset1 = 80.5*sin(CurrentRoll+(0.4794));;
        YOffset1 = 80.5*cos(CurrentRoll+(0.4794));;
        XOffset2 = 80.5*sin(CurrentRoll+(-0.4794));;
        YOffset2 = 80.5*cos(CurrentRoll+(-0.4794));;
        FTImpl.Vertex2ii(240+XOffset1,150+YOffset1,0,0);  FTImpl.Vertex2ii(240+XOffset2,150+YOffset2,0,0); // 20 deg down
        FTImpl.Vertex2ii(240-XOffset1,150-YOffset1,0,0);  FTImpl.Vertex2ii(240-XOffset2,150-YOffset2,0,0); // 20 deg up

        //10 degree marks
        FTImpl.ColorRGB(255,255,255);//set the color to White
        FTImpl.LineWidth(16);
        XOffset1 = 51*sin(CurrentRoll+(0.8415));;
        YOffset1 = 51*cos(CurrentRoll+(0.84154));;
        XOffset2 = 51*sin(CurrentRoll+(-0.8415));;
        YOffset2 = 51*cos(CurrentRoll+(-0.8415));;
        FTImpl.Vertex2ii(240+XOffset1,150+YOffset1,0,0);  FTImpl.Vertex2ii(240+XOffset2,150+YOffset2,0,0); // 10 deg down
        FTImpl.Vertex2ii(240-XOffset1,150-YOffset1,0,0);  FTImpl.Vertex2ii(240-XOffset2,150-YOffset2,0,0); // 10 deg up

        // 5 degree marks.  These work out as half the offsetts of the 10, so less math!
        FTImpl.LineWidth(12);
        FTImpl.Vertex2ii(240+XOffset1/2,150+YOffset1/2,0,0);  FTImpl.Vertex2ii(240+XOffset2/2,150+YOffset2/2,0,0); // 20 deg down
        FTImpl.Vertex2ii(240-XOffset1/2,150-YOffset1/2,0,0);  FTImpl.Vertex2ii(240-XOffset2/2,150-YOffset2/2,0,0); // 20 deg up
        
        // 15 degree marks
        XOffset1 = 56*sin(CurrentRoll+(0.3272));;
        YOffset1 = 56*cos(CurrentRoll+(0.3272));;
        XOffset2 = 56*sin(CurrentRoll+(-0.3272));;
        YOffset2 = 56*cos(CurrentRoll+(-0.3272));;
        FTImpl.Vertex2ii(240+XOffset1,150+YOffset1,0,0);  FTImpl.Vertex2ii(240+XOffset2,150+YOffset2,0,0); // 10 deg down
        FTImpl.Vertex2ii(240-XOffset1,150-YOffset1,0,0);  FTImpl.Vertex2ii(240-XOffset2,150-YOffset2,0,0); // 10 deg up

        // the 20 degree numbers.  These will always show up upright.
        XOffset1 = 87.7*sin(CurrentRoll+(-0.6400));;
        YOffset1 = 87.7*cos(CurrentRoll+(-0.6400));;
        XOffset2 = 87.7*sin(CurrentRoll+(-2.5));;
        YOffset2 = 87.7*cos(CurrentRoll+(-2.5));;
        FTImpl.Cmd_Number(240-XOffset1, 150-YOffset1, 26, FT_OPT_CENTER, 20);
        FTImpl.Cmd_Number(240-XOffset2, 150-YOffset2, 26, FT_OPT_CENTER, 20);
        
        // the 10 degree numbers.  These will always show up upright.
        XOffset1 = 61.6*sin(CurrentRoll+(-0.9835));;
        YOffset1 = 61.6*cos(CurrentRoll+(-0.9835));;
        XOffset2 = 61.6*sin(CurrentRoll+(-2.15));;
        YOffset2 = 61.6*cos(CurrentRoll+(-2.15));;
        FTImpl.Cmd_Number(240-XOffset1, 150-YOffset1, 26, FT_OPT_CENTER, 10);
        FTImpl.Cmd_Number(240-XOffset2, 150-YOffset2, 26, FT_OPT_CENTER, 10);
        
        // The Arrow...
        FTImpl.ColorRGB(255,255,0);//set the color to Yellow
        FTImpl.LineWidth(24);
        FTImpl.Begin(FT_LINE_STRIP);
        XOffset1 = 90*sin(CurrentRoll);
        YOffset1 = 90*cos(CurrentRoll);
        XOffset2 = 107*sin(CurrentRoll);
        YOffset2 = 107*cos(CurrentRoll);
        XOffset3 = XOffset2;
        YOffset3 = YOffset2;
        FTImpl.Vertex2ii(240-XOffset1,150-YOffset1,0,0);  
        FTImpl.Vertex2ii(240-XOffset2,150-YOffset2,0,0);
        XOffset1 = 100*sin(CurrentRoll+(-0.040));
        YOffset1 = 100*cos(CurrentRoll+(-0.040));
        XOffset2 = 100*sin(CurrentRoll+( 0.040));
        YOffset2 = 100*cos(CurrentRoll+( 0.040));
        FTImpl.Vertex2ii(240-XOffset1,150-YOffset1,0,0);  
        FTImpl.Vertex2ii(240-XOffset2,150-YOffset2,0,0);
        FTImpl.Vertex2ii(240-XOffset3,150-YOffset3,0,0);

        // The Rate of Turn Arrow
        // CurrentRateOfTurn is the rate of turn in Radians/Sec.  -(neg) is to the left
        // It must be scaled to indicate rate of turn in 2 minutes, so 
        float TwoMinAngle = -1*(CurrentRateOfTurn * 45.7);
        FTImpl.ColorRGB(255,128,0);//set the color to Orange
        if (TwoMinAngle > 0.9) {TwoMinAngle = 0.9; FTImpl.ColorRGB(255,0,0);}
        else if (TwoMinAngle < -0.9) {TwoMinAngle = -0.9; FTImpl.ColorRGB(255,0,0);};
        FTImpl.Begin(FT_LINE_STRIP);
        XOffset1 = 130*sin(TwoMinAngle);
        YOffset1 = 130*cos(TwoMinAngle);
        XOffset2 = 115*sin(TwoMinAngle);
        YOffset2 = 115*cos(TwoMinAngle);
        XOffset3 = XOffset2;
        YOffset3 = YOffset2;
        FTImpl.Vertex2ii(240-XOffset1,150-YOffset1,0,0);  
        FTImpl.Vertex2ii(240-XOffset2,150-YOffset2,0,0);
        XOffset1 = 120*sin(TwoMinAngle+(-0.0250));
        YOffset1 = 120*cos(TwoMinAngle+(-0.0250));
        XOffset2 = 120*sin(TwoMinAngle+( 0.0250));
        YOffset2 = 120*cos(TwoMinAngle+( 0.0250));
        FTImpl.Vertex2ii(240-XOffset1,150-YOffset1,0,0);  
        FTImpl.Vertex2ii(240-XOffset2,150-YOffset2,0,0);
        FTImpl.Vertex2ii(240-XOffset3,150-YOffset3,0,0);


        
//** Overlay the Altitude Strip


        // Print out the Labels, Kohlsman Window, and Units
        FTImpl.ColorRGB(255,255,255);//set the color to White
        FTImpl.Cmd_Text(337, 260, 27, FT_OPT_CENTER, "Altimeter:");
        switch (PressureUnits) {  // overlay the Pressure Units.  These can change on the fly...    
           case 1: {
             if (CurrentPressure > 31.0) CurrentPressure = 31.0;
             if (CurrentPressure < 28.1) CurrentPressure = 28.1;
             FTImpl.Cmd_Text(450, 260, 27, FT_OPT_CENTER, "inHg");    // inHg 
             FTImpl.Cmd_Number(397, 260, 27, FT_OPT_CENTER, (int)(CurrentPressure*100)); // keep 2 digits past the number
             FTImpl.Begin(FT_LINES);
             FTImpl.Vertex2ii(397,266,0,0);  FTImpl.Vertex2ii(397,266,0,0);
           break;}
           case 2: {
             if (CurrentPressure > 787.5) CurrentPressure = 787.5;
             if (CurrentPressure < 713.5) CurrentPressure = 713.5;
             FTImpl.Cmd_Text(450, 260, 27, FT_OPT_CENTER, "mmHg");    // mmHg
             FTImpl.Cmd_Number(397, 260, 27, FT_OPT_CENTER, (int)(CurrentPressure*10));  // keep 1 digit past the number
             FTImpl.Begin(FT_LINES);
             FTImpl.Vertex2ii(407,266,0,0);  FTImpl.Vertex2ii(407,266,0,0);
           break; }
           case 3: {
             if (CurrentPressure > 1050.0) CurrentPressure = 1050.0;
             if (CurrentPressure < 951.5) CurrentPressure = 951.5;
             FTImpl.Cmd_Text(450, 260, 27, FT_OPT_CENTER, "hPa");     // hPa
             FTImpl.Cmd_Number(370, 260, 27, FT_OPT_CENTERY | FT_OPT_RIGHTX, (int)(CurrentPressure*10)); // keep 1 digit past the number
             FTImpl.Begin(FT_LINES);
             FTImpl.Vertex2ii(360,266,0,0);  FTImpl.Vertex2ii(360,266,0,0);
           break; }}
// NEED TO Verify and FIX CASE 2, 3 - SETTINGS ON THE VERTICAL BOUNDS ___________________________________________________________________________________________________________NEED

//** Overlay the VSI Strip
        FTImpl.BlendFunc(FT_SRC_ALPHA,FT_ONE); // set for a blend instead of a paint
             FTImpl.Begin(FT_RECTS);
             FTImpl.ColorRGB(15,15,15);  //set the color to a slightly lighter hue
             FTImpl.Vertex2ii(429, 60);  FTImpl.Vertex2ii(480,240);
        FTImpl.BlendFunc(FT_SRC_ALPHA, FT_ONE_MINUS_SRC_ALPHA); // set back to paint

        FTImpl.LineWidth(16);
        FTImpl.ColorRGB(255,255,255);//set the color to White
        FTImpl.Begin(FT_LINES);
        FTImpl.Vertex2ii(435,230,0,0);  FTImpl.Vertex2ii(435,70,0,0);
        boolean ticker = true;
        for (int tics = 0; tics < 9; tics ++)
          {
            FTImpl.Begin(FT_LINES);
            FTImpl.Vertex2ii(433,70+(tics*20),0,0);  FTImpl.Vertex2ii(439,70+(tics*20),0,0);
            if (ticker)FTImpl.Cmd_Number(442, 62+(tics*20), 26, 0, abs(MaxVertSpeed-(tics*MaxVertSpeed/4)));
            ticker = !ticker;
          }
        // now the value of CurrentRateOfClimb in a nice little box
        TicMap = map(CurrentRateOfClimb, MaxVertSpeed*-1, MaxVertSpeed, 230, 70);  // maps into display units
        FTImpl.Begin(FT_LINES);
             FTImpl.Vertex2ii(435, TicMap);  FTImpl.Vertex2ii(445,TicMap);
        FTImpl.Begin(FT_RECTS);
             FTImpl.ColorRGB(128,128,128);//set the color to Grey
             FTImpl.Vertex2ii(442, TicMap+9);  FTImpl.Vertex2ii(480,TicMap-9);
             FTImpl.ColorRGB(0,0,0);//set the color to Black
             FTImpl.Vertex2ii(444, TicMap+7);  FTImpl.Vertex2ii(480,TicMap-7);
        FTImpl.ColorRGB(255,255,255);//set the color to White
        if ((CurrentRateOfClimb < 0) and (CurrentRateOfClimb > -1000))  // include the sign only for less than 1000 in the negative scale
        FTImpl.Cmd_Number(462, TicMap, 26, FT_OPT_CENTER | FT_OPT_SIGNED, (CurrentRateOfClimb));
        else FTImpl.Cmd_Number(462, TicMap, 26, FT_OPT_CENTER | FT_OPT_SIGNED, abs(CurrentRateOfClimb));
        // print out units...
        switch (AltUnits) {  // overlay the Speed Units.  These can change on the fly...
          case 1: FTImpl.Cmd_Text(440, 43, 27, FT_OPT_CENTER, "ft,   ft/s"); break; 
          case 2: FTImpl.Cmd_Text(440, 43, 27, FT_OPT_CENTER, "m,     m/s"); break; } 
 

//**  Overlay the Altimeter.
        FTImpl.LineWidth(16);
        FTImpl.ColorRGB(255,255,255);//set the color to White
        FTImpl.Begin(FT_LINES);
        FTImpl.Vertex2ii(365,38,0,0);  FTImpl.Vertex2ii(365,238,0,0);  // this is a 200 pixel long line, with a 2000 foot range, so we will get 10 feet per pixel...
        //now lay out the tics
        int Nearestth = 500*(round(CurrentAltitude/500));  //This finds the nearest 500 unit mark.
        int altoffset = (CurrentAltitude - Nearestth)/10; // this is in pixel units
        int bottomshow = -1; int topshow = 3; 
        if (altoffset < -12) {topshow = 2; bottomshow = -2;};
        for (int i = bottomshow; i < topshow; i++) //higher number shows on top
          { 
                 FTImpl.Begin(FT_LINES);
                 FTImpl.Vertex2ii(362,150+altoffset-(50*i),0,0);  FTImpl.Vertex2ii(367,150+altoffset-(50*i),0,0);  // 10 tics
                 FTImpl.Cmd_Number(368, 150+altoffset-(50*i), 26, FT_OPT_CENTERY | FT_OPT_SIGNED, ((Nearestth)+(i*500)));
          } 
        FTImpl.ColorRGB(0,0,0);//set the color to Black
        FTImpl.Begin(FT_LINES);
        FTImpl.Vertex2ii(362,150,0,0);  FTImpl.Vertex2ii(380,150,0,0);  // black tic
        FTImpl.Vertex2ii(362,151,0,0);  FTImpl.Vertex2ii(380,151,0,0);  // black tic
        FTImpl.Begin(FT_RECTS);       
        FTImpl.Vertex2ii(376,142,0,0);  FTImpl.Vertex2ii(408,159,0,0);  // first the box for the hundreds and up
        FTImpl.Vertex2ii(409,107,0,0);  FTImpl.Vertex2ii(427,194,0,0);  // then the box for the cool number slider for the 20's...  
        FTImpl.ClearStencil(0);  // set the stencil to blank
        FTImpl.StencilOp(FT_INCR,FT_INCR);
        FTImpl.Vertex2ii(411,108,0,0);  FTImpl.Vertex2ii(427,193,0,0);  // then the box THAT DEFINES THE STENCIL ....
        FTImpl.StencilOp(FT_KEEP,FT_KEEP);  // and we are done building the stencil

        if (CurrentAltitude >= 0) 
          {
            FTImpl.ColorRGB(200,200,140);                                                   //set the color to Creamy White
                int twenties = (CurrentAltitude - (100 * (int)(CurrentAltitude/100)))/20;   // everything but the last 2 digits off/then divide by 20.  the fact that it is an int does the rest
            int teens = (CurrentAltitude - (100 * (int)(CurrentAltitude/100)));          // everything but the last two digits are gone.
              while (abs(teens) >= 20) teens = teens - 20;  //Should knock us down to the number of feet less then 20.  THis will be the numerical offset for the scroll  
            FTImpl.StencilFunc(FT_GREATER, 0, 255);                                                                           //only operate on the stencil we just set up
            for (int i = -1; i < 7; i++)  // we are going to "print" 9 numbers onto the stencil
              { 
                FTImpl.Cmd_Number(414, 190-(i*20)+(teens), 26, FT_OPT_CENTER | FT_OPT_SIGNED, 260+((i+twenties)*20));      //  print the 20's, adjusted for height and for number                            
              }
            FTImpl.StencilFunc(FT_ALWAYS, 1, 255); //go back to ignoring the stencil
            FTImpl.ColorRGB(255,255,255); // white me up
 
            // now we do the other numbers as slow flipping sliders
            // start with the hundreds.  This will be the one to determine how all the numbers flip IF they do flip, so twentieths only need be calculated once.
            int twentieths = 0; int tenthousands = 0;  int thousands = 0;  int hundreds = 0;  int hundredsdigit = 0;  int thousandsdigit = 0;  int tenthousandsdigit = 0;
            int Destructor = abs(CurrentAltitude);  // we are going to take this sucker apart.  get rid of the minus sign if needed
            if (Destructor >= 10000) {tenthousandsdigit = Destructor/10000; Destructor = Destructor - (tenthousandsdigit * 10000);}  // int gets rid of any extra. then we trim
            if (Destructor >= 1000) {thousandsdigit = Destructor/1000; Destructor = Destructor - (thousandsdigit * 1000);}           // int gets rid of any extra. then we trim
            if (Destructor >= 100) hundredsdigit = Destructor/100;  // int gets rid of any extra.  no need to trim, as we dont care
            Destructor = abs(CurrentAltitude);  // reset the sucker.  get rid of the minu sign if needed
            tenthousands = Destructor - tenthousandsdigit*10000;  Destructor = tenthousands;
            thousands = Destructor - thousandsdigit*1000;  Destructor = thousands;
            hundreds = Destructor - hundredsdigit*100;  // and I no longer give a rip about Destructor!
                
            int topdigit = -1;  int bottomdigit = -1; 
            if (abs(CurrentAltitude) > 80) // we have to put numbers in the left box
              {
              // we will start out with the hunderedsdigit   
              if (hundreds > 82) // we are flipping to the higher number 
                {
                  twentieths = hundreds - 82;
                  topdigit = hundredsdigit + 1;
                  bottomdigit = hundredsdigit;
                  if (CurrentAltitude < 100) bottomdigit = -1;
                  //if ((CurrentAltitude > 1050) and (topdigit == 10)) topdigit = 0;
                  FlipNum(409, 143, topdigit, bottomdigit, 20-twentieths);
                }              // 0-10 as we approach the number
              else 
                  { FTImpl.Cmd_Number(400, 141, 27, 0, hundredsdigit); };  // we get to just print it out.
              }    
            // now we do the thousands digit
            if (abs(CurrentAltitude) > 982) // we have to put numbers in the left box 
              {  
              if (thousands > 982) // we are flipping to the higher number 
                {
                  topdigit = thousandsdigit + 1;
                  bottomdigit = thousandsdigit;
                  if (CurrentAltitude < 1000) bottomdigit = -1;
                  if ((CurrentAltitude > 9900) and (bottomdigit == 9)) topdigit = 10;
                  FlipNum(400, 143, topdigit, bottomdigit, 20-twentieths);
                }              // 0-10 as we approach the number
              else 
                  { FTImpl.Cmd_Number(391, 141, 27, 0, thousandsdigit); };  // we get to just print it out.  
              }
            // now we do the ten thousands digit
            if (abs(CurrentAltitude) > 9982) // we have to put numbers in the left box 
              {  
              if (tenthousands > 9982) // we are flipping to the higher number 
                {
                  topdigit = tenthousandsdigit + 1;
                  bottomdigit = tenthousandsdigit;
                  if (bottomdigit == 0) bottomdigit = -1;
                  if (topdigit == 0) topdigit = -1;
                  FlipNum(391, 143, topdigit, bottomdigit, 20-twentieths);
                }              // 0-10 as we approach the number
              else 
                  { FTImpl.Cmd_Number(382, 141, 27, 0, tenthousandsdigit); };  // we get to just print it out.  
              }                
          }  
        else // CurrentAltitude is negative!
          {
            FTImpl.ColorRGB(200,200,140);                                                   //set the color to Creamy White
            FTImpl.Cmd_Text(386, 144, 27, FT_OPT_CENTER , "_");
                int twenties = 20 - (CurrentAltitude - (100 * (int)(CurrentAltitude/100)))/20;   // everything but the last 2 digits off/then divide by 20.  the fact that it is an int does the rest
            int teens = -1 * (CurrentAltitude - (100 * (int)(CurrentAltitude/100)));          // everything but the last two digits are gone.
              while (abs(teens) >= 20) teens = teens - 20;  //Should knock us down to the number of feet less then 20.  THis will be the numerical offset for the scroll  
            FTImpl.StencilFunc(FT_GREATER, 0, 255);                                                                           //only operate on the stencil we just set up
            for (int i = -4; i < 3; i++)  // we are going to "print" 7 numbers onto the stencil
              { 
                FTImpl.Cmd_Number(414, 190+(i*20)-(teens), 26, FT_OPT_CENTER | FT_OPT_SIGNED, 240+((i+twenties)*20));      //  print the 20's, adjusted for height and for number                            
              }
            FTImpl.StencilFunc(FT_ALWAYS, 1, 255); //go back to ignoring the stencil
            FTImpl.ColorRGB(255,255,255); // white me up
 
            // now we do the other numbers as slow flipping sliders
            // start with the hundreds.  This will be the one to determine how all the numbers flip IF they do flip, so twentieths only need be calculated once.
            int twentieths = 0; int tenthousands = 0;  int thousands = 0;  int hundreds = 0;  int hundredsdigit = 0;  int thousandsdigit = 0;  int tenthousandsdigit = 0;
            int Destructor = abs(CurrentAltitude);  // we are going to take this sucker apart.  get rid of the minus sign if needed
            if (Destructor >= 10000) {tenthousandsdigit = Destructor/10000; Destructor = Destructor - (tenthousandsdigit * 10000);}  // int gets rid of any extra. then we trim
            if (Destructor >= 1000) {thousandsdigit = Destructor/1000; Destructor = Destructor - (thousandsdigit * 1000);}           // int gets rid of any extra. then we trim
            if (Destructor >= 100) hundredsdigit = Destructor/100;  // int gets rid of any extra.  no need to trim, as we dont care
            Destructor = abs(CurrentAltitude);  // reset the sucker.  get rid of the minu sign if needed
            tenthousands = Destructor - tenthousandsdigit*10000;  Destructor = tenthousands;
            thousands = Destructor - thousandsdigit*1000;  Destructor = thousands;
            hundreds = Destructor - hundredsdigit*100;  // and I no longer give a rip about Destructor!
                
            int topdigit = -1;  int bottomdigit = -1; 
            if (CurrentAltitude < -80) // we have to put numbers in the left box
              {
              // we will start out with the hunderedsdigit   
              if (hundreds > 80) // we are flipping to the higher number 
                {
                  twentieths = (-1*hundreds) + 80;
                  topdigit = hundredsdigit;
                  bottomdigit = hundredsdigit + 1;
                  if (CurrentAltitude > -100) topdigit = -1;
                  //if ((CurrentAltitude > 1050) and (topdigit == 10)) topdigit = 0;
                  FlipNum(409, 143, topdigit, bottomdigit, (-1*twentieths)+1);
                }              // 0-10 as we approach the number
              else 
                  { FTImpl.Cmd_Number(400, 141, 27, 0, hundredsdigit); };  // we get to just print it out.
              }    
            // now we do the thousands digit
            if (CurrentAltitude < -980) // we have to put numbers in the left box 
              {  
              if (thousands > 980) // we are flipping to the higher number 
                {
                  topdigit = thousandsdigit;
                  bottomdigit = thousandsdigit + 1;
                  if (CurrentAltitude > -1000) topdigit = -1;
                  FlipNum(400, 143, topdigit, bottomdigit, (-1*twentieths)+1);
                }              // 0-10 as we approach the number
              else 
                  { FTImpl.Cmd_Number(391, 141, 27, 0, thousandsdigit); };  // we get to just print it out.  
              }
               // there is no tenthousands digit in the negative range...
                                        
          }


//** Overlay the Control Knob Functions
        // trim gyro buttons
        FTImpl.ColorRGB(255,255,255);//set the color to White      
        FTImpl.Cmd_Text(201, 260, 27, FT_OPT_CENTER, "Compass   Roll   Pitch");
        
        // Sensor Error Flag
        if (SensorError)
        FTImpl.ColorRGB(200,0,0);//set the color to RED because we have a sensor error.     
        else FTImpl.ColorRGB(170,170,170);//set the color to grey becuase our last sensor read was successful.     
        FTImpl.Cmd_Text(455, 12, 18, FT_OPT_CENTER, "Snsr");
        FTImpl.Cmd_Text(455, 25, 18, FT_OPT_CENTER, "Err");



//** DRAW THIS LAST - The Menu items on the bottom of the screen.        
        if (Menu == 1) {FTImpl.ColorRGB(210,180,40); if (MenuSelected) FTImpl.ColorRGB(255,255,0);} else FTImpl.ColorRGB(180,120,30); // brightness box color
        FTImpl.Begin(FT_LINE_STRIP);  // *
        FTImpl.Vertex2ii(3,250,0,0); FTImpl.Vertex2ii(27,250,0,0); 
        FTImpl.Vertex2ii(27,269,0,0); FTImpl.Vertex2ii(3,269,0,0);
        FTImpl.Vertex2ii(3,250,0,0);         
        if (Menu == 2) {FTImpl.ColorRGB(210,180,40); if (MenuSelected) FTImpl.ColorRGB(255,255,0);} else FTImpl.ColorRGB(180,120,30); // speedunits box color
        FTImpl.Begin(FT_LINE_STRIP);  // BUG
        FTImpl.Vertex2ii(31,250,0,0); FTImpl.Vertex2ii(108,250,0,0); 
        FTImpl.Vertex2ii(108,269,0,0); FTImpl.Vertex2ii(31,269,0,0);
        FTImpl.Vertex2ii(31,250,0,0);         
        if (Menu == 3) {FTImpl.ColorRGB(210,180,40); if (MenuSelected) FTImpl.ColorRGB(255,255,0);} else FTImpl.ColorRGB(180,120,30); // bug box color
        FTImpl.Begin(FT_LINE_STRIP);  // Compass
        FTImpl.Vertex2ii(112,250,0,0); FTImpl.Vertex2ii(196,250,0,0); 
        FTImpl.Vertex2ii(196,269,0,0); FTImpl.Vertex2ii(112,269,0,0);
        FTImpl.Vertex2ii(112,250,0,0);         
        if (Menu == 4) {FTImpl.ColorRGB(210,180,40); if (MenuSelected) FTImpl.ColorRGB(255,255,0);} else FTImpl.ColorRGB(180,120,30); // trim box color
        FTImpl.Begin(FT_LINE_STRIP);  // Roll - where I was working
        FTImpl.Vertex2ii(200,250,0,0); FTImpl.Vertex2ii(238,250,0,0); 
        FTImpl.Vertex2ii(238,269,0,0); FTImpl.Vertex2ii(200,269,0,0);
        FTImpl.Vertex2ii(200,250,0,0);         
        if (Menu == 5) {FTImpl.ColorRGB(210,180,40); if (MenuSelected) FTImpl.ColorRGB(255,255,0);} else FTImpl.ColorRGB(180,120,30); // altimeter setting box color
        FTImpl.Begin(FT_LINE_STRIP);
        FTImpl.Vertex2ii(242,250,0,0); FTImpl.Vertex2ii(291,250,0,0); 
        FTImpl.Vertex2ii(291,269,0,0); FTImpl.Vertex2ii(242,269,0,0);
        FTImpl.Vertex2ii(242,250,0,0);         
        if (Menu == 6) {FTImpl.ColorRGB(210,180,40); if (MenuSelected) FTImpl.ColorRGB(255,255,0);}  else FTImpl.ColorRGB(180,120,30); // pressure units box color
        FTImpl.Begin(FT_LINE_STRIP);
        FTImpl.Vertex2ii(423,250,0,0); FTImpl.Vertex2ii(295,250,0,0); 
        FTImpl.Vertex2ii(295,269,0,0); FTImpl.Vertex2ii(423,269,0,0);
        FTImpl.Vertex2ii(423,250,0,0);         
        if (Menu == 7) {FTImpl.ColorRGB(210,180,40); if (MenuSelected) FTImpl.ColorRGB(255,255,0);}  else FTImpl.ColorRGB(180,120,30); // altitude units box color
        FTImpl.Begin(FT_LINE_STRIP);
        FTImpl.Vertex2ii(427,250,0,0); FTImpl.Vertex2ii(475,250,0,0); 
        FTImpl.Vertex2ii(475,269,0,0); FTImpl.Vertex2ii(427,269,0,0);
        FTImpl.Vertex2ii(427,250,0,0);         
          





// ****************************** End My Routine *******************************

	
		FTImpl.DLEnd();//end the display list
		FTImpl.Finish();//render the display list and wait for the completion of the DL
//	}

}

void Calibrate_AHRS()  // on powering up, the button was pressed.
  {
  boolean LiftButton = false;
  boolean PressButton = false;  
  boolean DoZero = false;
  while (!PressButton)  // this is the menu we will draw for the calibration screen
    {
    FTImpl.DLStart();              // begin a new drawing list
    FTImpl.Write(REG_PWM_DUTY,map(brightness, 0, 255, 0, 100));  // go maximum brightness
    FTImpl.ClearColorRGB(255,255,255);   // set the background color to white
    FTImpl.Clear(1,1,1);           // clear the screen
    FTImpl.ColorRGB(0,0,0);  //set the color to Black.
    FTImpl.Cmd_Text(240, 24, 28, FT_OPT_CENTER,  "Zero the AHRS screen.  Do not zero");
    FTImpl.Cmd_Text(240, 46, 28, FT_OPT_CENTER,  "the AHRS unless it is in level flight");
    FTImpl.Cmd_Text(240, 68, 28, FT_OPT_CENTER,  "attitude and facing 0 degrees due North.");
    FTImpl.Cmd_Text(240, 90, 28, FT_OPT_CENTER,  "It may be easier to zero the AHRS when");
    FTImpl.Cmd_Text(240, 112, 28, FT_OPT_CENTER, "it is not mounted in the aircraft.  In any");
    FTImpl.Cmd_Text(240, 134, 28, FT_OPT_CENTER, "case, twist the knob to the right several");
    FTImpl.Cmd_Text(240, 156, 28, FT_OPT_CENTER, "times until the ZERO option is selected");
    FTImpl.Cmd_Text(240, 178, 28, FT_OPT_CENTER, "to perform a calibration.");
    FTImpl.Cmd_Text(240, 250, 28, FT_OPT_CENTER, "Exit without saving.     Zero AHRS and Save.");
      // and now highlight the choice we have made
    FTImpl.ColorRGB(0,0,0);  //set the color to black.
    FTImpl.Begin(FT_LINE_STRIP);   // and get ready to draw a box
    if (DoZero)
      {  // draw a box around "Zero
        FTImpl.Vertex2ii(232,239,0,0); FTImpl.Vertex2ii(460,239,0,0); 
        FTImpl.Vertex2ii(460,261,0,0); FTImpl.Vertex2ii(232,261,0,0);
        FTImpl.Vertex2ii(232,239,0,0);         
      }
    else
      { // draw a box around "Don't Zero
        FTImpl.Vertex2ii(15,239,0); FTImpl.Vertex2ii(230,239,0); 
        FTImpl.Vertex2ii(230,261,0,0); FTImpl.Vertex2ii(15,261,0,0);
        FTImpl.Vertex2ii(15,239,0,0);                
      }
    // close out the screen
    FTImpl.DLEnd();//end the display list
    FTImpl.Finish();//render the display list and wait for the completion of the DL
    
    if (digitalRead(A15) == HIGH) LiftButton = true; // we won't sense the button press until this is so
    if (((digitalRead(A15) == LOW) and (LiftButton))) PressButton = true;  // we have lifted the button and pressed it again 
    NewEncoderPosition = myEnc.read(); // read the encoder
    if (OldEncoderPosition - 50 < NewEncoderPosition) DoZero = false; else DoZero = true;
    } 
  if (DoZero) // we need to tare the sensor and save the results
    {
      // Tare the Sensor
      Serial2.write(247);// "Start of Packet Byte"
      Serial2.write(96); // "Set Compass Range"
      Serial2.write(96); // "Checksum" of the one byte we just sent.
      // Save the new Zero
//      Serial2.write(247); // "Start of Packet Byte"
//      Serial2.write(105); // "Set Reference Vector Mode"
//      Serial2.write(3);   // "Range set to "Single Auto Continual" (2)  Other possibilites include "Single Auto" (1) and "Multiple" (3)
//      Serial2.write(107); // "Checksum" - sum of the last two bytes mod 256.  
    }
  }

void setup()
{
  String AHRS_Version = "N";  // this is used later on to sense that we have connected to the AHRS chip
  // set up the pins we will need to be using...
  pinMode(A15,INPUT_PULLUP); // A 15 is the select button on the encoder
  pinMode(19, INPUT_PULLUP); // 19 is an interrupt pin, and is used as the encoder quadrature signal A 
  pinMode(18, INPUT_PULLUP); // 18 is an interrupt pin, and is used as the encoder quadrature signal B 
  // start tracking the encoder knob 
  OldEncoderPosition = myEnc.read();
  boolean agree = false;
  boolean Iagree = false;
  // enable the display
  BootupConfigure();
  
  // Serial2 is the Remote AHRS sensor.
  Serial2.begin(115200);  // begin serial communications with Yost Labs 3-Space Nano AHRS Chip
  // AHRS chip can be made to change speeds, but defaults to 115200.  Mode is 8N1 (8 data, no parity, 1 stop)  This is the arduino default.
  delay(500);  // give a bit of time to ensure everything is good.
  while (Serial2.available()) Serial2.read();  // empty the receive serial port
  // write a "get version" request to the AHRS chip
  Serial2.write(247); // "Start of Packet Byte"
  Serial2.write(230); // "Get Version"
  Serial2.write(230); // "Checksum" - in this case the exact same as the message, because it only sums the message byte.
  if (!digitalRead(A15)) Calibrate_AHRS();  // this is the calibration routine
  // begin a while loop here that draws the startup screen over and over until you select "I agree"
  while (!Iagree)  
  {
  if ((digitalRead(A15) == LOW) and (agree)) Iagree = true;
  // start drawing the startup screen
  FTImpl.DLStart();              // begin a new drawing list
  FTImpl.Write(REG_PWM_DUTY,map(brightness, 0, 255, 0, 100));  // go maximum brightness
  FTImpl.ClearColorRGB(0,0,0);   // set the background color to black
  FTImpl.Clear(1,1,1);           // clear the screen
  FTImpl.ColorRGB(255,255,255);  //set the color to white.
  FTImpl.Cmd_Text(240, 24, 28, FT_OPT_CENTER, "TheOpenCockpit Airduino 4.3 version 1.0 Beta");
  FTImpl.Cmd_Text(240, 46, 28, FT_OPT_CENTER, "Power-Up Report and Legal Disclaimer");
  if (!Static.begin()) {  // initialize the Static Port                                            // BMP280 setup
    FTImpl.ColorRGB(255,0,0);  //set the color to red.
    FTImpl.Cmd_Text(240, 78, 26, FT_OPT_CENTER, "Static Port BMP 280 Sensor ......... NOT FOUND @ 0x77");
  }
  else {
    FTImpl.ColorRGB(255,255,0);  //set the color to yellow.
    FTImpl.Cmd_Text(240, 78, 26, FT_OPT_CENTER, "Static Port BMP 280 Sensor ......... FOUND & 0x77");
  }
  if (!Pitot.begin(0x76)) {  // initialize the Pitot Port                                            // BMP280 setup
    FTImpl.ColorRGB(255,0,0);  //set the color to red.
    FTImpl.Cmd_Text(240, 96, 26, FT_OPT_CENTER, "Pitot Port BMP 280 Sensor ......... NOT FOUND @ 0x76");
  }
  else {
    FTImpl.ColorRGB(255,255,0);  //set the color to yellow.
    FTImpl.Cmd_Text(240, 96, 26, FT_OPT_CENTER, "Pitot Port BMP 280 Sensor ......... FOUND & 0x76");
  }
  if (Serial2.available() > 11) // it is time to read the AHRS response
    { 
      AHRS_Version = "";
      while (Serial2.available()) AHRS_Version = AHRS_Version + Serial2.read(); 
    } 
  if (AHRS_Version == "N") {// we have not heard from the AHRS sensor
    FTImpl.ColorRGB(255,0,0);  //set the color to red.
    FTImpl.Cmd_Text(240, 118, 26, FT_OPT_CENTER, "Searching for AHRS Sensor on UART2");
  }
  else {
    FTImpl.ColorRGB(255,255,0);  //set the color to yellow.
    FTImpl.Cmd_Text(240, 118, 26, FT_OPT_CENTER, "Yost Labs AHRS Sensor .......... FOUND on UART2");
  }         


  // Disclaimer
    FTImpl.ColorRGB(255,255,255);  //set the color to white.
    FTImpl.Cmd_Text(240, 145, 27, FT_OPT_CENTER, "This instrument is experimental in nature, and should");
    FTImpl.Cmd_Text(240, 165, 27, FT_OPT_CENTER, "be used to enhance situational awareness only.  Any");
    FTImpl.Cmd_Text(240, 185, 27, FT_OPT_CENTER, "and all use is at the operator's discretion and risk.");
  // choices
    FTImpl.Cmd_Text(240, 217, 27, FT_OPT_CENTER, "I do not agree.            I agree.");
  // more info
    FTImpl.Cmd_Text(132, 250, 26, FT_OPT_CENTER, "For more information, please visit");
    FTImpl.ColorRGB(100,100,255);  //set the color to light blue.
    FTImpl.Cmd_Text(348, 250, 26, FT_OPT_CENTER, "http://www.TheOpenCockpit.org");
  // and now highlight the choice we have made
    FTImpl.ColorRGB(255,255,255);  //set the color to white.
    FTImpl.Begin(FT_LINE_STRIP);   // and get ready to draw a box
  if (agree)
    {  // draw a box around "agree"
      FTImpl.Vertex2ii(278,206,0,0); FTImpl.Vertex2ii(358,206,0,0); 
      FTImpl.Vertex2ii(358,229,0,0); FTImpl.Vertex2ii(278,229,0,0);
      FTImpl.Vertex2ii(278,206,0,0);         
    }
  else
    { // draw a box around "I do not agree" 
      FTImpl.Vertex2ii(120,206,0,0); FTImpl.Vertex2ii(252,206,0,0); 
      FTImpl.Vertex2ii(252,229,0,0); FTImpl.Vertex2ii(120,229,0,0);
      FTImpl.Vertex2ii(120,206,0,0);                
    }
  // close out the startup screen
    FTImpl.DLEnd();//end the display list
    FTImpl.Finish();//render the display list and wait for the completion of the DL
  NewEncoderPosition = myEnc.read(); // read the encoder
  if (OldEncoderPosition != NewEncoderPosition) // the knob has moved
    if (agree) agree = false; else agree = true; // so we need to change our choice
  OldEncoderPosition = NewEncoderPosition;  
  // end the while loop here, so that you cannot move on without selecting "I Agree"
  } 

  // now set up the AHRS chip
  // Set Accelerometer range
  Serial2.write(247); // "Start of Packet Byte"
  Serial2.write(121); // "Set Accelerometer Range"
  Serial2.write(16);   // "Range set to +- 4G"  Could be 2G (0) 4G (16) and 8G (32)
  Serial2.write(137); // "Checksum" - sum of the last two bytes mod 256.
  // Set Gyroscope range
  Serial2.write(247); // "Start of Packet Byte"
  Serial2.write(125); // "Set Gyroscope Range"
  Serial2.write(1);   // "Range set to +- 500dps"  Could also be 250 (0) and 2000 (2)
  Serial2.write(126); // "Checksum" - sum of the last two bytes mod 256.
  // Set Compass range
  Serial2.write(247); // "Start of Packet Byte"
  Serial2.write(126); // "Set Compass Range"
  Serial2.write(1);   // "Range set to +- 1.33Ga"  Could also be +-.88 Ga (0) and many more
  Serial2.write(127); // "Checksum" - sum of the last two bytes mod 256.
  // Set Reference Vector Mode
  Serial2.write(247); // "Start of Packet Byte"
  Serial2.write(105); // "Set Reference Vector Mode"
  Serial2.write(3);   // "Range set to "Single Auto Continual" (2)  Other possibilites include "Single Auto" (1) and "Multiple" (3)
  Serial2.write(107); // "Checksum" - sum of the last two bytes mod 256.
  


 
  // setup variables from defined values so that this doesn't have to be done over and over again
 
  BotWhA = map(WAStart,  0,MaxSpeed, 0, 195);  // 195 is the number of pixels it displays
  TopWhA = map(WAEnd,    0,MaxSpeed, 0, 195);  // 195 is the number of pixels it displays
  BotGA = map(GAStart,   0,MaxSpeed, 0, 195);  // 195 is the number of pixels it displays
  TopGA = map(GAEnd,     0,MaxSpeed, 0, 195);  // 195 is the number of pixels it displays
  RL = map(RLine,        0,MaxSpeed, 0, 195);  // 195 is the number of pixels it displays
  BL = map(BLine,        0,MaxSpeed, 0, 195);  // 195 is the number of pixels it displays
  switch (PressureUnits) {  // overlay the Pressure Units.  These can change on the fly...
     case 1: PressureDigits[0] = 2; PressureDigits[1] = 9; PressureDigits[2] = 9; PressureDigits[3] = 2; break;   // inHg
     case 2: PressureDigits[0] = 7; PressureDigits[1] = 6; PressureDigits[2] = 0; PressureDigits[3] = 0; break;   // mmHg
     case 3: PressureDigits[0] = 1; PressureDigits[1] = 0; PressureDigits[2] = 1; PressureDigits[3] = 8; break; } // hPa
  NumTics = MaxSpeed/SpeedTics;  // This is how many tics we will need
  HalfSpeedTics = (map(SpeedTics,0,MaxSpeed,0,195)/2);  // 195 is the number of pixels it displays
  CurrentSpeed = 84;
  CurrentRoll = -0.5236;  // Radians.  30 degrees to the left
  CurrentPitch = -0.17453/2; // Radians.  10 degrees up


}

void loop()
{
  // Calculate Altitude
    PreviousAltitude = CurrentAltitude;
    float AltCallUnits = CurrentPressure;
    if (PressureUnits == 1) AltCallUnits = CurrentPressure * 25.4 * 1.3332; // inHg to mmHg to hPa
    if (PressureUnits == 2) AltCallUnits = CurrentPressure * 1.3332; // mmHg to hPa
    if (AltUnits == 1)CurrentAltitude = (3.28*Static.readAltitude(AltCallUnits+StaticCalibrationOffset)); // call this with the altitude in hPa ...  Reads out feet... 
      else            CurrentAltitude = (     Static.readAltitude(AltCallUnits+StaticCalibrationOffset)); // call this with the altitude in hPa ...  Reads out meters...
  // Calculate airspeed
    float CalcSpeed = 0;
    switch(SpeedUnits) {  // Playing with units to make it come out...
//      case 1: CalcSpeed = (59.2484 * sqrt(122.45*(Pitot.readPressure() - Static.readPressure()))) + (AirspeedCalibrationOffset * 1.15); break;   // kts/hr  
//      case 2: CalcSpeed = (68.1818 * sqrt(122.45*(Pitot.readPressure() - Static.readPressure()))) + (AirspeedCalibrationOffset); break;    // m/h
//      case 3: CalcSpeed = (109.7280 * sqrt(122.45*(Pitot.readPressure() - Static.readPressure()))) + (AirspeedCalibrationOffset * 0.62); break; }  // km/h
      case 1: CalcSpeed = (0.592484 * sqrt(122.45*(Pitot.readPressure() - Static.readPressure()))) + (AirspeedCalibrationOffset * 1.15); break;   // kts/hr  
      case 2: CalcSpeed = (0.681818 * sqrt(122.45*(Pitot.readPressure() - Static.readPressure()))) + (AirspeedCalibrationOffset); break;    // m/h
      case 3: CalcSpeed = (1.097280 * sqrt(122.45*(Pitot.readPressure() - Static.readPressure()))) + (AirspeedCalibrationOffset * 0.62); break; }  // km/h
    CurrentSpeed = CalcSpeed;
  // handle the input encoder section.  This is activated by raising the button.
    NewEncoderPosition = myEnc.read();
    EncMove = (OldEncoderPosition - NewEncoderPosition)/ECPD;
    if (ButtonDown and digitalRead(A15)) ButtonReleased = true;
    ButtonDown = !digitalRead(A15);   
    if (ButtonReleased and !MenuSelected) { MenuSelected = true;  ButtonReleased = false; }  
    int oldunits;
    if (MenuSelected) // we are in a menu, and need to handle it
      {
        if (ButtonReleased) // anything that happens when we leave a menu needs to happen now
          {
            MenuSelected = false; // this has to happen
          }
        switch(Menu) {
          case 1: // brightness
             if (ButtonDown) EncMult = 25; else EncMult = 3;  // higher speed if you hold down the button
             brightness = brightness + (EncMove * EncMult);  
             if (brightness > 255) brightness = 255;
             if (brightness < 6) brightness = 6; //don't want to be so low that you cant see to brighten it back up
             break;  
          case 2:  // bug
              if (ButtonDown) EncMult = 10; else EncMult = 1;    // higher speed if you hold down the button
             HeadingBug = HeadingBug + (EncMove * EncMult);  
             if (HeadingBug > 360) HeadingBug = HeadingBug - 360;
             if (HeadingBug < 0) HeadingBug = HeadingBug + 360;  //  corresponds to degrees.  Wraps around the 360 point
             break;  
          case 3:  // compass
             if (ButtonDown) EncMult = 10; else EncMult = 1;    // higher speed if you hold down the button
             CompassOffset = CompassOffset + (EncMove * EncMult * 0.5); // 0.5 is a half degree in degrees  ;-) 
             break;
          case 4:  // Roll
             if (ButtonDown) EncMult = 10; else EncMult = 1;    // higher speed if you hold down the button
             RollOffset = RollOffset + (EncMove * EncMult * 0.00872);  // 0.00872 is a half degree in Radians         
             break;  
          case 5:  // Pitch
             if (ButtonDown) EncMult = 10; else EncMult = 1;    // higher speed if you hold down the button
             PitchOffset = PitchOffset + (EncMove * EncMult * 0.00872);  // 0.00872 is a half degree in Radians         
             break; 
          case 6: // altimeter
             if (ButtonDown) EncMult = 10; else EncMult = 1;    // higher speed if you hold down the button
             switch (PressureUnits) {  // overlay the Pressure Units.  These can change on the fly...    
               case 1: CurrentPressure = CurrentPressure + ((float)(EncMove * EncMult)/100); break;    // inHg
               case 2: CurrentPressure = CurrentPressure + ((float)(EncMove * EncMult)/10); break;    // mmHg
               case 3: CurrentPressure = CurrentPressure + ((float)(EncMove * EncMult)/10); break; }  // hPa         
             break;           
          case 7:   // altimeter units
             if (EncMove != 0)                   // we gonna covert us some units
               if (EncMove > 0)                  // we are going up
                 {
                   if (PressureUnits == 1) CurrentPressure = CurrentPressure * 25.4;    // Converting inHg to mmHg
                   if (PressureUnits == 2) CurrentPressure = CurrentPressure * 1.3332;  // Converting mmHg to hPa
                 }  // note - not conversion necessary for 3 becuase we have no other units
               else                              // we are going down
                 {  // and again, we aren't going down farther if we are at 1...
                   if (PressureUnits == 2) CurrentPressure = CurrentPressure / 25.4;    // Converting mmHg to inHg
                   if (PressureUnits == 3) CurrentPressure = CurrentPressure / 1.3332;  // Converting hPa to mmHg
                 }  
             PressureUnits = PressureUnits + EncMove;
             if (PressureUnits > 3) PressureUnits = 3;
             if (PressureUnits < 1) PressureUnits = 1;  //  corresponds to altimeter units.  no wrap
             break; } // and close the switch           
      if (ButtonDown and ButtonReleased) {MenuSelected = false; } // you just moved out of whatever menu you are in
      }
    else // we are not in a menu, and the encoder has turned to switch between menus
      { 
        Menu = Menu + EncMove;
        if (Menu > 7) Menu = 7;
        if (Menu < 1) Menu = 1;
      }
    ButtonReleased = false;
    OldEncoderPosition = NewEncoderPosition;  // this is the end of the menu routine

    PreviousHeading = CurrentHeading;
    
    //  now handle the time calculation that allows us to calculate rate of turn and vsi
    previousMicros = currentMicros;
    currentMicros = micros();
    intervalMicros = currentMicros - previousMicros;
    
    CurrentRateOfTurn = 500*(float)(CurrentHeading - PreviousHeading)/(intervalMicros);                      //  ROUTINE STILL NEEDS CALIBRATED AND SMOOTHED.  I think I also need a more precise clock.  May also need to look into setting BNO055 chip modes.

    //Read the Euler Angle packet from the AHRS
    if (Serial2.available() > 11)
      {                                                               // we have a packet.  Procede to decode
          //Serial2.read(); no longer needed
          CP.binary[3] = Serial2.read(); CP.binary[2] = Serial2.read(); CP.binary[1] = Serial2.read(); CP.binary[0] = Serial2.read();  //will go into CurrentHeading
          CH.binary[3] = Serial2.read(); CH.binary[2] = Serial2.read(); CH.binary[1] = Serial2.read(); CH.binary[0] = Serial2.read(); //will go into CurrentPitch
          CR.binary[3] = Serial2.read(); CR.binary[2] = Serial2.read(); CR.binary[1] = Serial2.read(); CR.binary[0] = Serial2.read(); //will go into CurrentRoll
          // CalibrationStatus = Serial2.read();  no longer available
          CurrentHeading = CH.floatingPoint * 57.2958 + CompassOffset;    // now make the data more easily accessible to the rest of the program.  Note that heading is in degrees, while the others are in radians.
          CurrentPitch = CP.floatingPoint + PitchOffset; //- (0.008726 * 26.0);     // now make the data more easily accessible to the rest of the program.
          CurrentRoll = -1*CR.floatingPoint + RollOffset; //+ (0.008726 * 4.5);      // now make the data more easily accessible to the rest of the program.
          if (CurrentHeading > 360) CurrentHeading = CurrentHeading - 360; // correct for any values out of range.
          if (CurrentHeading <= 0) CurrentHeading = CurrentHeading + 360;  // correct for any values out of range.       
        SensorError = false; // we were able to make a reading
        ErrorStreak = 0;     // and that ends any "streak" of bad readings...
      }
    else 
      {
        SensorError = true;  // so we failed to read the sensor
        ErrorStreak = ErrorStreak + 1; // and extended any streak of fails that may be going on
      }  
    if (ErrorStreak > 25) ErrorStreak = 25;  // this tells the display to go offline in the case of long term sensor error

  // serial data correction and request section - communicating with AHRS chip
    while (Serial2.available()) Serial2.read();  // empty out any garbage from the receive serial port.  This must be done in order to restore lost communications.
  
  Serial2.write(247); // Start of Packet Byte
  Serial2.write(1); // Command Byte - "Read Filtered, Tared Orientation data in Euler Angle Format"
  Serial2.write(1); // Checksum Byte - in this case the exact same as the message, because it only sums the message byte.  
  
  // update the screen
  DrawScreen();
} 















