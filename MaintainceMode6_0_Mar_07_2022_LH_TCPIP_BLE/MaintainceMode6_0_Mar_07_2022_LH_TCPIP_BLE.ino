#include <Ethernet2.h>
#include <RS232.h>
//#include <RS485.h>
#include <String.h>
//#include <Timer.h>

/*defines for Macros*/
#define ver "version: MAR-12-2022"
#define ON_OUTPUT HIGH
#define OFF_OUTPUT LOW
#define INPUT_ON        1
#define INPUT_OFF       0

#define timval 25

#define AXEL_2          1
#define AXEL_1          2
#define MEASURE_AXEL_2  3
#define AXEL_HOME       4
#define MEASURE_AXEL_1  5
#define AXEL_1_REACHED  6
#define MAINTAINCE_MODE 7
#define MEASURMENT_DUMMY 8

#define DISABLED        0
#define ENABLED         1

/*defines for all Outputs*/
#define pinQ0_1_SEW_SPEED_1 37
#define pinQ0_2_SEW_SPEED_2 38
#define pinQ0_4_SEW_FWD     40
#define pinQ0_5_SEW_REV     4
#define pinQ0_6_CROSS_FWD   5
#define pinQ0_7_CROSS_REV   6   
#define pinQ1_0_JACK_UP    42 
#define pinQ1_1_JACK_DN    41  
#define pinQ1_2_INDUCTION_MOTOR_FWD  43
#define pinQ1_3_INDUCTION_MOTOR_REV  44
#define pinQ1_4_AC_ORIENTAL_MOTOR_CLCK      45  
#define pinQ1_5_AC_ORIENTAL_MOTOR_ANTI_CLCK  8
#define pinQ2_5_TOWER_LAMP_RED 12
#define pinQ2_6_TOWER_LAMP_GREEN 13
#define pinQ0_3_TOWER_LAMP_YELLOW 39
#define pinQ2_3_TOWER_LAMP_BUZZER 49

//#define pinQ1_6_MIST_SPEED_1  9
//#define pinQ1_7_MIST_SPEED_2  7
//#define pinQ2_1_MIST_FWD_DIR  47
#define pinQ2_2_MIST_REV_DIR  48

#define pinQ0_0_MIST_DIR       36 
#define pinQ2_0_MIST_PULSE    53

#define pinQ1_6_SEW_PIN_2  9
#define pinQ1_7_SEW_PIN_3  7
#define pinQ2_1_SEW_PIN_4  47

//#define pinQ1_6_MEGA_RST      9
/*end of defines for Output*/

/*Defines for Inputs*/
//#define pinI0_0_GNTRY_HOME       22 //63 
//#define pinI0_1_GNTRY_START      23
//#define pinI0_2_GNTRY_MODEL_1    24
#define pinI0_0_GNTRY_MOVING       22
#define pinI0_1_GNTRY_HOME_POS     23 
#define pinI0_2_GNTRY_AXEL_2_POS   56 // Replacing 0.2 24 with 0.9 Pin 56

//#define pinI0_5_GNTRY_MODEL_2     2
#define pinI0_3_CROSS_OVER_HOME    55 // replacing 0.3 25 to 0.8 pin no 55
#define pinI0_4_CROSS_OVER_END   26
#define pinI1_0_JACK_UPDN_HOME   27  
#define pinI0_7_JACK_UPDN_END    30 //54  // Repakcing with I1.3 pin number 30
#define pinI1_1_JACK_FWDREV_HOME 28
#define pinI1_2_JACK_FWDREV_END  29 
#define pinI1_4_PHOTO_SENSOR     31

#define pinI1_7_EMERGENCY_CNTRL   60
#define pinI1_11_GANTRY_CNTRL     64
#define pinI1_12_CROSS_CNTRL     65
#define pinI2_7_JACKUPDN_CNTRL   66
#define pinI2_8_JACKFWREV_CNTRL  67
#define pinI2_9_WHEEL_CNTRL      68
#define pinI2_1_AUTO_MANUAL_SEL  33
#define pinI2_3_AUTO_MODE        35

#define pinI1_8_FORWARD_BUTTON   61 // // old I18 to I2_4 old pin 61 
#define pinI1_10_REVRSE_BUTTON   63   // old I1_10 to I1_3 old pin 63
#define pinI2_0_START_BUTTON     32

#define pinI0_5_ROT_SENSOR_FRONT  2
#define pinI0_6_ROT_SENSOR_BACK  3

/*End of Defines for Inputs*/

/*define for Input Bits*/
#define bitSTART  0
#define bitForward 1
#define bitReverse 2
#define bitAutoMode 3
#define bitAutoManualSel 4
#define bitWheelCntrl   5
#define bitJackFwdRevCntrl 6
#define bitJackUpdnCntrl   7
#define bitCrossCntrl      8
#define bitGantryCntrl     9
#define bitEmergencyCntrl  10
#define bitJackFwdRevEnd  11
#define bitJackFwdRevHome 12
#define bitJackUpDnEnd    13
#define bitJackUpDnHome   14
#define bitCrossOverEnd   15
#define bitCrossOverHome  16
#define bitGntryMode1     17
#define bitGntyStart      18
#define bitGntryHome      19
/*end of defines for Input Bits/
/*Variable for Input Buttons*/

const char compile_date[] = __FILE__" "__DATE__ " " __TIME__;

int buttonStateGantryHome = 0;         // variable for reading the pushbutton status
int buttonStateGantryStart = 0;
int buttonStateGantryModel_1 = 0;
int buttonStateGantryModel_2 = 0;
int buttonStateCrossOverHome = 0;
int buttonStateCrossOverEnd = 0;
int buttonStateJackUpDnHome = 0;
int buttonStateJackUpDnEnd = 0;
int buttonStateJackFwdRevHome = 0;
int buttonStateJackFwdRevEnd = 0;  
int buttonStateEmergency = 0;
int buttonStateGantryControl = 0;
int buttonStateCrossOverControl = 0;
int buttonStateJackUpDnControl = 0;
int buttonStateJackFwdRevControl = 0;
int buttonStateWheelControl = 0;

int buttonStateAutoManualSel = 0;
int buttonStateAutoMode = 0;

int buttonStateForwardButton = 0;
int buttonStateReverseButton = 0;
int buttonStateStartButton = 0;
int buttonStatePhotoSensor = 0;

int buttonStateRotSensorFront = 0;
int buttonStateRotSensorBack = 0;

int buttonStateGantryMoving = 0;
int buttonStateGantryHomePos = 0;
int buttonStateGantryAxel2Pos = 0;

unsigned long previousTime = 0;
unsigned char Re_buf[11],counter=0;
unsigned char sign=0;
unsigned char sensorFault = 0;
String CommandString = "";
int flagBitForInput = 0;
char startFlag = 0;
char startJackFlag = 0;
unsigned char reachedFront = 0;
unsigned char reachedBack = 0;
unsigned char MoveFront = 0;
unsigned char MoveBack = 0;
unsigned char forIndex  = 0;
unsigned char ReachedFlag = 0;
unsigned char jackReverseFlag = 0;
unsigned char wheelRoationFlag = 0;
unsigned int timerValue = 0;
unsigned long noOfPulsesCount = 0;
unsigned char index = 0;
float a[3],w[3],angle[3],T;
float fRotationalAngle = 0.0;
float fzAxis = 0;
String mystring;
bool Mode = 0; 
unsigned char PrvswState = 0;
// mac address for M-Duino
//byte mac[] = { 0xBE, 0xAD, 0xBE, 0xEF, 0xFE, 0xED };
// Ip address for M-Duino
//byte ip[] = { 192, 168, 10, 101 };
//int tcp_port = 5566;
unsigned char wc;
unsigned char array_string[20];

// mac address for M-Duino
//#define RH 1
#define LH 1

#if RH 
byte mac[] = { 0xBE, 0xAD, 0xBE, 0xEF, 0xFE, 0xED };
// Ip address for M-Duino
byte ip[] = { 192, 168, 10, 102 };
EthernetServer server = EthernetServer(5568);
EthernetServer server1 = EthernetServer(5569);
EthernetClient client;
EthernetClient client1;
#endif 

#if LH 
byte mac[] = { 0xBE, 0xAD, 0xBE, 0xEF, 0xFE, 0xED };
// Ip address for M-Duino
byte ip[] = { 192, 168, 10, 101 };
EthernetServer server = EthernetServer(5566);
EthernetServer server1 = EthernetServer(5567);
EthernetClient client;
EthernetClient client1;
#endif 

int tcp_port = 5566;
unsigned char TCPIP_data;
unsigned char array_stringTcp[20];
//unsigned char array_string[20];
unsigned char indexTcp;
unsigned char indexWr;
unsigned char commadFlag = 0;
unsigned char AlignILRFormeasurment = 0;
unsigned char DataReadFlag =0; 
unsigned int ui16counterJackFwd = 0;
/*EthernetServer server = EthernetServer(5566);
EthernetServer server1 = EthernetServer(5567);
EthernetClient client;
EthernetClient client1; */

union FloatChar {
 float f;
 char c[sizeof(float)];
};

FloatChar fc;
//EthernetServer server = EthernetServer(5566);
//EthernetClient client;
/*end of Variabled for Input Button*/

/*Function Prototypes*/
void offGantry();
void onGantryForward();
void onGantryReverse();
void offWheel();
void onWheelForward();
void onWheelReverse();
void offJack();
void onJackForward();
void onJackReverse();
void offJackUpDown();
void onJackUp();
void onJackDown();
void offCrossOver();
void onCrossOverForward();
void onCrossOverReverse();
void angleCalculation();
void stopFrontILR();
void stopbackILR();
//void serialEvent();
void EthernetRead();
void speedMistHS();
void speedMistMS();
void speedMistLS();
void onMistForward();
void onMistReverse();
void offMist();
void dirMistFwd();
void pulseMistON();
void dirMistRev();
void pulseMistOFF();
void onJackForwardForCorrection();
void onGantryAxelTwo();
void onGantryAxelOne();
void onGantryHome();
void fineTuneILRValues();
void readMasterCommands();
/*End of Function Prototypes*/

void initGpioOutPut()
{
  pinMode(pinQ0_1_SEW_SPEED_1, OUTPUT);
  pinMode(pinQ0_2_SEW_SPEED_2, OUTPUT);
 
  pinMode(pinQ0_2_SEW_SPEED_2, OUTPUT);
  pinMode(pinQ0_4_SEW_FWD, OUTPUT); 
  pinMode(pinQ0_5_SEW_REV, OUTPUT);
  pinMode(pinQ0_6_CROSS_FWD, OUTPUT);
  pinMode(pinQ0_7_CROSS_REV, OUTPUT);
  pinMode(pinQ1_0_JACK_UP, OUTPUT);
  pinMode(pinQ1_1_JACK_DN, OUTPUT);
  pinMode(pinQ1_2_INDUCTION_MOTOR_FWD, OUTPUT);
  pinMode(pinQ1_3_INDUCTION_MOTOR_REV, OUTPUT);
  pinMode(pinQ1_4_AC_ORIENTAL_MOTOR_CLCK, OUTPUT);
  pinMode(pinQ1_5_AC_ORIENTAL_MOTOR_ANTI_CLCK, OUTPUT);

  pinMode(pinQ2_5_TOWER_LAMP_RED, OUTPUT);
  pinMode(pinQ2_6_TOWER_LAMP_GREEN, OUTPUT);
  pinMode(pinQ0_3_TOWER_LAMP_YELLOW, OUTPUT);
  pinMode(pinQ2_3_TOWER_LAMP_BUZZER, OUTPUT);

  //pinMode(pinQ1_6_MIST_SPEED_1, OUTPUT);  
  //pinMode(pinQ1_7_MIST_SPEED_2, OUTPUT); 
  //pinMode(pinQ2_1_MIST_FWD_DIR, OUTPUT);  
  pinMode(pinQ2_2_MIST_REV_DIR, OUTPUT);  
  pinMode(pinQ0_0_MIST_DIR, OUTPUT);
  pinMode(pinQ2_0_MIST_PULSE, OUTPUT);
  pinMode(pinQ1_6_SEW_PIN_2, OUTPUT);  
  pinMode(pinQ1_7_SEW_PIN_3, OUTPUT);  
  pinMode(pinQ2_1_SEW_PIN_4, OUTPUT);  

//  pinMode(pinQ1_6_MEGA_RST,OUTPUT);
}
void initiGPIOInPut()
{
  //pinMode(pinI0_0_GNTRY_HOME, INPUT);
  //pinMode(pinI0_1_GNTRY_START, INPUT);
  //pinMode(pinI0_2_GNTRY_MODEL_1, INPUT);
  pinMode(pinI0_0_GNTRY_MOVING, INPUT); 
  pinMode(pinI0_1_GNTRY_HOME_POS, INPUT);
  pinMode(pinI0_2_GNTRY_AXEL_2_POS, INPUT); 
 
//  pinMode(pinI0_5_GNTRY_MODEL_2, INPUT);
  pinMode(pinI0_3_CROSS_OVER_HOME, INPUT);
  pinMode(pinI0_4_CROSS_OVER_END, INPUT);
  pinMode(pinI1_0_JACK_UPDN_HOME, INPUT);
  pinMode(pinI0_7_JACK_UPDN_END, INPUT);
  pinMode(pinI1_4_PHOTO_SENSOR,INPUT);
  pinMode(pinI1_1_JACK_FWDREV_HOME, INPUT);
  pinMode(pinI1_2_JACK_FWDREV_END, INPUT);

  pinMode(pinI1_7_EMERGENCY_CNTRL, INPUT);
  pinMode(pinI1_11_GANTRY_CNTRL, INPUT);
  pinMode(pinI1_12_CROSS_CNTRL, INPUT);
  pinMode(pinI2_7_JACKUPDN_CNTRL, INPUT);
  pinMode(pinI2_8_JACKFWREV_CNTRL, INPUT);  
  pinMode(pinI2_9_WHEEL_CNTRL, INPUT);  

  pinMode(pinI2_1_AUTO_MANUAL_SEL, INPUT);
   
  pinMode(pinI2_3_AUTO_MODE, INPUT); 

  pinMode(pinI2_0_START_BUTTON, INPUT);
  pinMode(pinI1_10_REVRSE_BUTTON, INPUT);
  pinMode(pinI1_8_FORWARD_BUTTON, INPUT);

  pinMode(pinI0_5_ROT_SENSOR_FRONT, INPUT);
  pinMode(pinI0_6_ROT_SENSOR_BACK, INPUT);
}

void PrintSerial()
{
  Serial.println("Gantry Model 1 Position");
}

void testingInPut()
{
 /*  buttonStateGantryHome = digitalRead(pinI0_0_GNTRY_HOME);
 //  if (buttonStateGantryHome == HIGH)
 //   Serial.println("Home ");
   buttonStateGantryStart = digitalRead(pinI0_1_GNTRY_START);
//   if (buttonStateGantryStart == HIGH)
 //   Serial.println("Start ");

   buttonStateGantryModel_1 = digitalRead(pinI0_2_GNTRY_MODEL_1);
 //  if (buttonStateGantryModel_1 == HIGH)
 //      Serial.println("Gntry Model  ");
  */
    buttonStateGantryMoving = digitalRead(pinI0_0_GNTRY_MOVING);
    buttonStateGantryHomePos = digitalRead(pinI0_1_GNTRY_HOME_POS);
    
   // if (buttonStateGantryHomePos == HIGH)
   //   Serial.println("Gntry Home ");
    buttonStateGantryAxel2Pos = digitalRead(pinI0_2_GNTRY_AXEL_2_POS);
    //if (buttonStateGantryAxel2Pos == HIGH)
    //  Serial.println("Axel Postion 2 ");
/*   buttonStateGantryModel_2 = digitalRead(pinI0_5_GNTRY_MODEL_2);
   if (buttonStateGantryModel_2 == HIGH)
      Serial.println("Gntry Model 2  ");
  */ 
   buttonStatePhotoSensor =digitalRead(pinI1_4_PHOTO_SENSOR);
  // if (buttonStatePhotoSensor == HIGH)
  //     Serial.println("Photo ");
     
   buttonStateCrossOverHome = digitalRead(pinI0_3_CROSS_OVER_HOME);
   //if (buttonStateCrossOverHome == HIGH)
   //   Serial.println("crossOver Home ");
   
   buttonStateCrossOverEnd = digitalRead(pinI0_4_CROSS_OVER_END);
   //if (buttonStateCrossOverEnd == HIGH)
   //    Serial.println("Cross End ");
   buttonStateJackUpDnHome = digitalRead(pinI1_0_JACK_UPDN_HOME);
   //if (buttonStateJackUpDnHome == HIGH)
   //   Serial.println("Jack Up/Dn Home ");
   buttonStateJackUpDnEnd = digitalRead(pinI0_7_JACK_UPDN_END);
   //if (buttonStateJackUpDnEnd == HIGH)
   //    Serial.println("Jack Up end");
   buttonStateJackFwdRevHome = digitalRead(pinI1_1_JACK_FWDREV_HOME);
   //if (buttonStateJackFwdRevHome == HIGH)
   //   Serial.println("Jack Fw/Rev Home ");
   
   buttonStateJackFwdRevEnd = digitalRead(pinI1_2_JACK_FWDREV_END);
   //if (buttonStateJackFwdRevEnd == HIGH)
   //    Serial.println("Rev End ");
   buttonStateEmergency = digitalRead(pinI1_7_EMERGENCY_CNTRL);
   buttonStateGantryControl = digitalRead(pinI1_11_GANTRY_CNTRL);
   buttonStateCrossOverControl = digitalRead(pinI1_12_CROSS_CNTRL);
   buttonStateJackUpDnControl = digitalRead(pinI2_7_JACKUPDN_CNTRL);
   buttonStateJackFwdRevControl = digitalRead(pinI2_8_JACKFWREV_CNTRL);
   buttonStateWheelControl = digitalRead(pinI2_9_WHEEL_CNTRL);

   buttonStateAutoManualSel = digitalRead(pinI2_1_AUTO_MANUAL_SEL);
   buttonStateAutoMode = digitalRead(pinI2_3_AUTO_MODE);  
   
   buttonStateStartButton  = digitalRead(pinI2_0_START_BUTTON);
   buttonStateReverseButton = digitalRead(pinI1_10_REVRSE_BUTTON);
   buttonStateForwardButton = digitalRead(pinI1_8_FORWARD_BUTTON);

   buttonStatePhotoSensor =digitalRead(pinI1_4_PHOTO_SENSOR);
   //if (buttonStatePhotoSensor == LOW) //changed on 8-jun-2021
   if (buttonStatePhotoSensor == HIGH)
   {
    buttonStateRotSensorFront = digitalRead(pinI0_5_ROT_SENSOR_FRONT);
    /*if (buttonStateRotSensorFront == HIGH)
    {
       RS232.println("{MFNT}");
       Serial.println("{MFNT}");
       client1.write("{MFNT}");
    } */ 
    buttonStateRotSensorBack = digitalRead(pinI0_6_ROT_SENSOR_BACK);
    /*if (buttonStateRotSensorBack == HIGH)
     {   RS232.println("{MBCK}");
         Serial.println("{MBCK}");
         client1.write("{MBCK}");
     }*/
   }
}

void offAllOuputs()
{
  offGantry();
  offWheel();
  offJack();
  offJackUpDown();
  offCrossOver();          
}


void dirMistFwd()
{
  digitalWrite(pinQ0_0_MIST_DIR, HIGH);  
}

void pulseMistON()
{
  digitalWrite(pinQ2_0_MIST_PULSE, HIGH);
  
}

void dirMistRev()
{
 digitalWrite(pinQ0_0_MIST_DIR, LOW);  
}

void pulseMistOFF()
{
  digitalWrite(pinQ2_0_MIST_PULSE, LOW);
}

void speedMistHS()
{
 //digitalWrite( pinQ1_6_MIST_SPEED_1, HIGH);
 //digitalWrite( pinQ1_7_MIST_SPEED_2, HIGH); 
}

void speedMistMS()
{
 //digitalWrite( pinQ1_6_MIST_SPEED_1, HIGH);
 //digitalWrite( pinQ1_7_MIST_SPEED_2, LOW); 
}

void speedMistLS()
{
 //digitalWrite( pinQ1_6_MIST_SPEED_1, LOW);
 //digitalWrite( pinQ1_7_MIST_SPEED_2, LOW); 
}

void onMistForward()
{
  speedMistMS();
  //digitalWrite( pinQ2_1_MIST_FWD_DIR, HIGH);
  digitalWrite( pinQ2_2_MIST_REV_DIR, LOW);
} 
void onMistReverse()
{
  speedMistMS();
  //digitalWrite( pinQ2_1_MIST_FWD_DIR, LOW);
  digitalWrite( pinQ2_2_MIST_REV_DIR, HIGH);
} 

void offMist()
{
 // digitalWrite( pinQ2_1_MIST_FWD_DIR, LOW);
  digitalWrite( pinQ2_2_MIST_REV_DIR, LOW);
//  digitalWrite( pinQ1_6_MIST_SPEED_1, LOW);
//  digitalWrite( pinQ1_7_MIST_SPEED_2, LOW);
}

void BuzzerBeep()
{
  digitalWrite( pinQ2_3_TOWER_LAMP_BUZZER, HIGH);
  delay(300);
  digitalWrite( pinQ2_3_TOWER_LAMP_BUZZER, LOW);  
}


void goHome()
{
  if( startFlag == 2)
  {
    if (buttonStateJackFwdRevEnd  == INPUT_ON)
    {
      onJackReverse();  
    }
    if (buttonStateJackFwdRevHome   == INPUT_ON)
    {
      offJack();
     if(buttonStateJackUpDnEnd  == INPUT_ON)
     {
      onJackDown(); 
     }  
    }
    
    if( buttonStateJackUpDnHome  == INPUT_ON)
    {
      offJackUpDown();
      if( buttonStateCrossOverEnd == INPUT_ON)
      {
        onCrossOverReverse();  
      }      
    }
    if(buttonStateCrossOverHome  == INPUT_ON)
    { 
       offCrossOver();
     if( buttonStateGantryAxel2Pos == INPUT_ON)
     {
       
       while (buttonStateGantryMoving == INPUT_OFF)
       {
             onGantryHome();
             buttonStateGantryMoving = digitalRead(pinI0_0_GNTRY_MOVING);
       }

       while (buttonStateGantryHomePos == INPUT_OFF)
       {
            onGantryHome();
            buttonStateGantryHomePos = digitalRead(pinI0_1_GNTRY_HOME_POS);
       } 
         offGantry();
         BuzzerBeep();    
     }
      if( (buttonStateGantryHomePos == INPUT_ON ))// || (buttonStateGantryStart == INPUT_ON))
      {
        offGantry();
        startFlag = 0;
     }             
    }   
  }  
}

void goHomeAxelOne()
{
  if( startFlag == 5)
  {
    if(buttonStateJackFwdRevEnd  == INPUT_ON)
    {
      onJackReverse();  
    }
    if (buttonStateJackFwdRevHome   == INPUT_ON)
    {
      offJack();
     if(buttonStateJackUpDnEnd  == INPUT_ON)
     {
      onJackDown(); 
     }  
    }
    
    if( buttonStateJackUpDnHome  == INPUT_ON)
    {
      offJackUpDown();
      if( buttonStateCrossOverEnd == INPUT_ON)
      {
        while(buttonStateCrossOverHome == INPUT_OFF)
        {
          onCrossOverReverse();
          buttonStateCrossOverHome = digitalRead(pinI0_3_CROSS_OVER_HOME);
        }
        offGantry();
        BuzzerBeep(); 
        startFlag = 0; 
      }      
    }
    /*
    if(buttonStateCrossOverHome  == INPUT_ON)
    { 
       offCrossOver();
     if( buttonStateGantryHomePos == INPUT_ON)
     {
       
       while (buttonStateGantryMoving == INPUT_OFF)
       {
             onGantryHome();
             buttonStateGantryMoving = digitalRead(pinI0_0_GNTRY_MOVING);
       }

       while (buttonStateGantryHomePos == INPUT_OFF)
       {
            onGantryHome();
            buttonStateGantryHomePos = digitalRead(pinI0_1_GNTRY_HOME_POS);
       } 
         offGantry();
         BuzzerBeep();    
     }
      if( (buttonStateGantryHomePos == INPUT_ON ))// || (buttonStateGantryStart == INPUT_ON))
      {
        offGantry();
        startFlag = 0;
     }
                  
    } */   
  }  
}

/*
int buttonStateGantryMoving = 0;
int buttonStateGantryHomePos = 0;
int buttonStateGantryAxel2Pos = 0;

buttonStateGantryMoving = digitalRead(pinI0_0_GNTRY_MOVING);
buttonStateGantryHomePos = digitalRead(pinI0_1_GNTRY_HOME_POS);
buttonStateGantryAxel2Pos = digitalRead(pinI0_2_GNTRY_AXEL_2_POS)
*/   

void operationCrossOverJackUpDnFwRev()
{
    testingInPut();
 // if (buttonStateGantryAxel2Pos == INPUT_ON)
 // {
      if((buttonStateCrossOverHome == INPUT_ON))
       {
         while (buttonStateCrossOverEnd == INPUT_OFF)    
         {
           onCrossOverForward();
           buttonStateCrossOverEnd = digitalRead(pinI0_4_CROSS_OVER_END);
         }         
          offCrossOver();
          BuzzerBeep();
       }
         
         
      if((buttonStateCrossOverEnd == INPUT_ON) && (startJackFlag == DISABLED))
      {
         //offCrossOver();
         if (buttonStateJackUpDnHome  == INPUT_ON)
         {
          buttonStateJackUpDnEnd = digitalRead(pinI0_7_JACK_UPDN_END);
          //BuzzerBeep();
          while (buttonStateJackUpDnEnd == INPUT_OFF)
          {
            onJackUp();
            buttonStateJackUpDnEnd = digitalRead(pinI0_7_JACK_UPDN_END);
          }          
           offJackUpDown();
           BuzzerBeep();
         }
      }
      if((buttonStateCrossOverEnd == INPUT_ON) && (buttonStateJackUpDnEnd == INPUT_ON) && (startJackFlag == ENABLED))
      {
         if (buttonStateJackFwdRevHome == INPUT_ON)
         {
            buttonStateJackFwdRevEnd = digitalRead(pinI1_2_JACK_FWDREV_END);
            while(buttonStateJackFwdRevEnd == INPUT_OFF)
            {
              onJackForward();
              buttonStateJackFwdRevEnd = digitalRead(pinI1_2_JACK_FWDREV_END);  
            }
             offJack();
             BuzzerBeep();
             if(startFlag == AXEL_2)
             {
                startFlag = MEASURE_AXEL_2;
             }
             if( startFlag == AXEL_1)
             {
                startFlag = MEASURE_AXEL_2;
             }
         }
      }         
 // }  
}

void startOperation()
{
  if (startFlag == AXEL_2)
   {
      testingInPut();
       if(( buttonStateGantryHomePos == INPUT_ON))
       {
           BuzzerBeep();
           digitalWrite( pinQ2_6_TOWER_LAMP_GREEN, HIGH);
           buttonStateGantryAxel2Pos = digitalRead(pinI0_2_GNTRY_AXEL_2_POS);
           while (buttonStateGantryAxel2Pos == INPUT_OFF)
           {
             onGantryAxelTwo();
             buttonStateGantryAxel2Pos = digitalRead(pinI0_2_GNTRY_AXEL_2_POS);
           }
           BuzzerBeep();
           offGantry();
           //Serial.println("Gantry Moving");
        }
        if (buttonStateGantryAxel2Pos == INPUT_ON)
          operationCrossOverJackUpDnFwRev();
    } // end of if (startFlag == AXEL_2)
 //} 
    
    if(startFlag == AXEL_1)
    {
      goToAxelOne();
      //startFlag = AXEL_1_REACHED;
    }
    if(startFlag == AXEL_1_REACHED)
     {
      operationCrossOverJackUpDnFwRev();
     }
    if(startFlag == AXEL_HOME)
    {
      goToHomePostion();
    }
    /*if(startFlag == AXEL_HOME)
    {
      goToBackHome();
    } */
    
}
void goToHomePostion()
{
    startJackFlag = DISABLED;
    buttonStateJackFwdRevHome = digitalRead(pinI1_1_JACK_FWDREV_HOME);
    while(buttonStateJackFwdRevHome == INPUT_OFF)
    {
      onJackReverse();
      buttonStateJackFwdRevHome = digitalRead(pinI1_1_JACK_FWDREV_HOME);  
    }
     offJack();
     BuzzerBeep();
     
    while (buttonStateJackUpDnHome == INPUT_OFF)
    {
      onJackDown();
      buttonStateJackUpDnHome = digitalRead(pinI1_0_JACK_UPDN_HOME);
    }          
     offJackUpDown();
     BuzzerBeep();
     
    while (buttonStateCrossOverHome == INPUT_OFF)    
    {
      onCrossOverReverse();
      buttonStateCrossOverHome = digitalRead(pinI0_3_CROSS_OVER_HOME);
    }         
    offCrossOver();
    BuzzerBeep();
    while (buttonStateGantryHomePos == INPUT_OFF)    
    {
       onGantryHome();
       buttonStateGantryHomePos = digitalRead(pinI0_1_GNTRY_HOME_POS);
       startJackFlag = DISABLED;     
    }     
     BuzzerBeep();
    offGantry();
    startFlag = 0;
  
}
void goToAxelOne()
{   testingInPut();
    buttonStateJackFwdRevHome = digitalRead(pinI1_1_JACK_FWDREV_HOME);
    while(buttonStateJackFwdRevHome == INPUT_OFF)
    {
      onJackReverse();
      buttonStateJackFwdRevHome = digitalRead(pinI1_1_JACK_FWDREV_HOME);  
    }
     offJack();
     BuzzerBeep();
     
    while (buttonStateJackUpDnHome == INPUT_OFF)
    {
      onJackDown();
      buttonStateJackUpDnHome = digitalRead(pinI1_0_JACK_UPDN_HOME);
    }          
     offJackUpDown();
     BuzzerBeep();
     
    while (buttonStateCrossOverHome == INPUT_OFF)    
    {
      onCrossOverReverse();
      buttonStateCrossOverHome = digitalRead(pinI0_3_CROSS_OVER_HOME);
    }         
    offCrossOver();
    BuzzerBeep();

    while (buttonStateGantryHomePos == INPUT_OFF)    
    {
       onGantryHome();
       buttonStateGantryHomePos = digitalRead(pinI0_1_GNTRY_HOME_POS);
       startJackFlag = DISABLED;     
    }     
     BuzzerBeep();
     offGantry();
     startJackFlag = DISABLED;
     startFlag = AXEL_1_REACHED;
     operationCrossOverJackUpDnFwRev();
     

}


void startOperationAxelOne()
{
  if (startFlag == 4)
     {
       if((buttonStateGantryHomePos  == INPUT_ON))
       {
         offGantry();  
        // startFlag = 0;       
             
         if( ((buttonStateGantryHomePos  == INPUT_ON) && ((buttonStateCrossOverHome == INPUT_ON) ||(buttonStateCrossOverEnd == INPUT_OFF) )))
         {
            buttonStateCrossOverEnd = digitalRead(pinI0_4_CROSS_OVER_END);
            while (buttonStateCrossOverEnd == INPUT_OFF)
            { 
              onCrossOverForward();
              buttonStateCrossOverEnd = digitalRead(pinI0_4_CROSS_OVER_END);
            }
          // startFlag = 0;      
         }
          
          
         if ((buttonStateGantryHomePos  == INPUT_ON) &&  (buttonStateCrossOverEnd  == INPUT_ON))
         {
            
            offCrossOver();
           //if (buttonStateJackUpDnEnd  == INPUT_OFF)
           //   onJackUp();
              
           // startFlag = 0;
           if (buttonStateJackUpDnHome  == INPUT_ON)
           {
               buttonStateJackUpDnEnd = digitalRead(pinI0_7_JACK_UPDN_END);
               
               while(buttonStateJackUpDnEnd  == INPUT_OFF)
               {
                  onJackUp();
                  buttonStateJackUpDnEnd = digitalRead(pinI0_7_JACK_UPDN_END);
               }
               offJackUpDown();
           }
           
           if (buttonStateJackUpDnEnd  == INPUT_ON)
           {
             offJackUpDown();
             //if (startJackFlag == 0)
             //   offJack();

            if(startJackFlag == 5)
            { 
             // startFlag = 0;
             if (buttonStateJackFwdRevHome == INPUT_ON)
             {
               onJackForward();            
             }
            if (buttonStateJackFwdRevEnd == INPUT_OFF)
            {
              onJackForward();
            }
                      
           if (buttonStateJackFwdRevEnd == INPUT_ON)   
           {
             offJack();
             
             
             //angleCalculation();
             /*
             if (buttonStateWheelControl == INPUT_OFF)
             {
               if ((buttonStateForwardButton == INPUT_ON) &&  (buttonStateReverseButton == INPUT_OFF))  
               {
                onWheelForward();  
               }
               if ((buttonStateForwardButton == INPUT_OFF) &&  (buttonStateReverseButton == INPUT_ON))  
               {
                onWheelReverse();  
               }
               if ((buttonStateForwardButton == INPUT_OFF) &&  (buttonStateReverseButton == INPUT_OFF))  
               {
                 offWheel();  
               } // startFlag = 0; 
             }*/
             if (buttonStateWheelControl == INPUT_ON)
             {
              //if ((reachedFront == 0) && (buttonStatePhotoSensor == HIGH)) //changed on 8 jun 2021
                if ((reachedFront == 0) && (buttonStatePhotoSensor == LOW)) 
                {
                 /* if(AlignILRFormeasurment == 1)
                     angleCalculation(); */
                } 
                //if ((buttonStatePhotoSensor == LOW)) //Changed on 8 jun 2021
                  if ((buttonStatePhotoSensor == HIGH))             
                    fineTuneILRValues();
              /*
               if ((buttonStateForwardButton == INPUT_ON) &&  (buttonStateReverseButton == INPUT_OFF))  
               {
                 onMistForward();                
               // onWheelForward();  
               }
               if ((buttonStateForwardButton == INPUT_OFF) &&  (buttonStateReverseButton == INPUT_ON))  
               {
                 onMistReverse();  
               }
               if ((buttonStateForwardButton == INPUT_OFF) &&  (buttonStateReverseButton == INPUT_OFF))  
               {
                 offMist();  
               } // startFlag = 0; */
             }
                        
           }
          } // end of if startJackFlag = 1;           
         }                  
       } 
      }                  
     }    
}


void autoMode()
{
   if((buttonStateAutoManualSel  == INPUT_ON) && (buttonStateAutoMode  == INPUT_ON))
   {
    if(buttonStateStartButton == INPUT_ON)
    {

//     if((startFlag == 0 ) && (buttonStateGantryHomePos == INPUT_ON) && (buttonStatePhotoSensor == HIGH) && \                    
//     (buttonStateCrossOverEnd == INPUT_ON) && (buttonStateJackUpDnEnd == INPUT_ON ) && (buttonStateJackFwdRevEnd == INPUT_ON))        //chnaged on 8 jun 2021
     if((startFlag == 0 ) && (buttonStateGantryHomePos == INPUT_ON) && (buttonStatePhotoSensor == LOW) && \ 
     (buttonStateCrossOverEnd == INPUT_ON) && (buttonStateJackUpDnEnd == INPUT_ON ) && (buttonStateJackFwdRevEnd == INPUT_ON))
     {
       startFlag = AXEL_HOME;
     }
    //if((buttonStatePhotoSensor == LOW) && (buttonStateJackFwdRevEnd == INPUT_ON) ) //changed on 8 jun 2021
    if((buttonStatePhotoSensor == HIGH) && (buttonStateJackFwdRevEnd == INPUT_ON) )
    {
     // Serial.println("Pressed");
     //#if 0
       if (buttonStateGantryHomePos == INPUT_ON)
       {
          
          Serial1.println("{DDAX}");
          Serial1.println("{DDAX}");
          Serial1.println("{DDAX}");
          
          RS232.println("{DDAX}");
          RS232.println("{DDAX}");
          RS232.println("{DDAX}");         
          Serial.println("{DDAX}");
          CommandString =  "{DDAX}";
          #if RH
          client1.write("{DDAX}");
          #endif
        }
       //#endif
        if( buttonStateGantryAxel2Pos == INPUT_ON)
        {
          RS232.println("{DRAX}");
          Serial1.println("{DRAX}");         
          Serial.println("{DRAX}");
          CommandString = "{DRAX}";
          #if RH
          client1.write("{DRAX}");
          #endif
         }
         startFlag = MEASURMENT_DUMMY;
    }
          
   if((startFlag == 0 ) && (buttonStateJackFwdRevControl == INPUT_OFF) && (buttonStateGantryHomePos == INPUT_ON) && \ 
       (buttonStateCrossOverHome == INPUT_ON) && (buttonStateJackUpDnHome == INPUT_ON ) && (buttonStateJackFwdRevHome == INPUT_ON))
     {
        startFlag = AXEL_2;        
        startJackFlag = DISABLED;
     }
     if (startFlag == 0)
       BuzzerBeep(); 
     
     if (buttonStateGantryAxel2Pos == INPUT_ON)
     {
      if((buttonStateJackFwdRevControl == INPUT_OFF) && (buttonStateCrossOverEnd == INPUT_ON)\ 
       && (buttonStateJackUpDnEnd == INPUT_ON ) && (buttonStateJackFwdRevHome == INPUT_ON))
      { 
          startFlag = AXEL_2;
          startJackFlag = ENABLED;        
      }
//       else if( (buttonStatePhotoSensor == LOW) && (buttonStateJackFwdRevControl == INPUT_OFF) && (buttonStateCrossOverEnd == INPUT_ON)\ 
//       && (buttonStateJackUpDnEnd == INPUT_ON ) && (buttonStateJackFwdRevEnd == INPUT_ON))                                                      //changed on 8 jun 2021
      else if( (buttonStatePhotoSensor == HIGH) && (buttonStateJackFwdRevControl == INPUT_OFF) && (buttonStateCrossOverEnd == INPUT_ON)\ 
       && (buttonStateJackUpDnEnd == INPUT_ON ) && (buttonStateJackFwdRevEnd == INPUT_ON))
      { 
          startFlag = MEASURE_AXEL_2;
          startJackFlag = DISABLED;
          buttonStateStartButton  = digitalRead(pinI2_0_START_BUTTON);
          while (buttonStateStartButton == INPUT_ON)
          {
              buttonStateStartButton  = digitalRead(pinI2_0_START_BUTTON);
              buttonStateGantryHomePos = digitalRead(pinI0_1_GNTRY_HOME_POS);
              buttonStateGantryAxel2Pos = digitalRead(pinI0_2_GNTRY_AXEL_2_POS);
              if (buttonStateGantryAxel2Pos == HIGH)
              {
                 RS232.println("{DRAX}");         
                 Serial1.println("{DRAX}");
                 Serial.println("{DRAX}");
                 #if RH
                  client1.write("{DRAX}");
                 #endif
                 CommandString = "{DRAX}";
                 //RS485.println("{DRAX}");
              }
              if (buttonStateGantryHomePos == HIGH)
              {
                 RS232.println("{DDAX}");
                 Serial1.println("{DDAX}");         
                 Serial.println("{DDAX}");
                 #if RH
                  client1.write("{DDAX}");
                  #endif
                  CommandString = "{DRAX}";
                 //RS485.println("{DDAX}");                
              }
              
              //RS232.println("FSTP");         
              //Serial.println("FSTP");
                     
             // readMasterCommands();
         }                  
      }
//      else if( (buttonStatePhotoSensor == HIGH) && (buttonStateJackFwdRevControl == INPUT_OFF) && (buttonStateCrossOverEnd == INPUT_ON)\ 
//       && (buttonStateJackUpDnEnd == INPUT_ON ) && (buttonStateJackFwdRevEnd == INPUT_ON) && (startFlag == MEASURE_AXEL_2) )                //changed on 8 jun 2021
        else if( (buttonStatePhotoSensor == LOW) && (buttonStateJackFwdRevControl == INPUT_OFF) && (buttonStateCrossOverEnd == INPUT_ON)\ 
       && (buttonStateJackUpDnEnd == INPUT_ON ) && (buttonStateJackFwdRevEnd == INPUT_ON) && (startFlag == MEASURE_AXEL_2) )
      {
        startFlag = AXEL_1;
        while (buttonStateStartButton == INPUT_ON)
        {
          buttonStateStartButton  = digitalRead(pinI2_0_START_BUTTON);
          RS232.println("{AXL1}");
          Serial1.println("{AXL1}");         
          Serial.println("{AXL1}");
          #if RH
          client1.write("{AXL1}");
          #endif          
          CommandString = "{AXL1}";
          //RS485.println("{AAXL1}");
        }
      }
      else
      {
         BuzzerBeep();        
       }
     }// end of Postion Axel 2
     if ((buttonStateGantryHomePos == INPUT_ON) && (startFlag == AXEL_1_REACHED))
     {
       if((buttonStateJackFwdRevControl == INPUT_OFF) && (buttonStateCrossOverEnd == INPUT_ON)\ 
       && (buttonStateJackUpDnEnd == INPUT_ON ) && (buttonStateJackFwdRevHome == INPUT_ON))
      { 
          startFlag = AXEL_1_REACHED;
          startJackFlag = ENABLED;        
      }
//      else if( (buttonStatePhotoSensor == LOW) && (buttonStateJackFwdRevControl == INPUT_OFF) && (buttonStateCrossOverEnd == INPUT_ON)\ 
//       && (buttonStateJackUpDnEnd == INPUT_ON ) && (buttonStateJackFwdRevEnd == INPUT_ON))                                                    //changed on 8 jun 2021
        else if( (buttonStatePhotoSensor == HIGH) && (buttonStateJackFwdRevControl == INPUT_OFF) && (buttonStateCrossOverEnd == INPUT_ON)\ 
       && (buttonStateJackUpDnEnd == INPUT_ON ) && (buttonStateJackFwdRevEnd == INPUT_ON))
      { 
          startFlag = AXEL_1_REACHED;
          startJackFlag = DISABLED;
          buttonStateStartButton  = digitalRead(pinI2_0_START_BUTTON);
          {
              buttonStateStartButton  = digitalRead(pinI2_0_START_BUTTON);
              buttonStateGantryHomePos = digitalRead(pinI0_1_GNTRY_HOME_POS);
              buttonStateGantryAxel2Pos = digitalRead(pinI0_2_GNTRY_AXEL_2_POS);
              if (buttonStateGantryAxel2Pos == HIGH)
              {
                 RS232.println("{DRAX}");         
                 Serial1.println("{DRAX}");
                 Serial.println("{DRAX}");
                 #if RH 
                 client1.write("{DRAX}");
                 #endif 
                 CommandString = "{DRAX}";
               //  RS485.println("{DRAX}");
              }
              if (buttonStateGantryHomePos == HIGH)
              {
                 RS232.println("{DDAX}");
                 Serial1.println("{DDAX}");         
                 Serial.println("{DDAX}");
                 #if RH
                 client1.write("{DDAX}");
                 #endif
                 CommandString =  "{DDAX}";
                // RS485.println("{DDAX}");                
              }                     
             // readMasterCommands();
         }                  
      }
//      else if( (buttonStatePhotoSensor == HIGH) && (buttonStateJackFwdRevControl == INPUT_OFF) && (buttonStateCrossOverEnd == INPUT_ON)\ 
//       && (buttonStateJackUpDnEnd == INPUT_ON ) && (buttonStateJackFwdRevEnd == INPUT_ON) && (startFlag == AXEL_1_REACHED) )                    //changed on 8 jun 2021
      else if( (buttonStatePhotoSensor == LOW) && (buttonStateJackFwdRevControl == INPUT_OFF) && (buttonStateCrossOverEnd == INPUT_ON)\ 
       && (buttonStateJackUpDnEnd == INPUT_ON ) && (buttonStateJackFwdRevEnd == INPUT_ON) && (startFlag == AXEL_1_REACHED) )
      {
        startFlag = AXEL_HOME;
        while (buttonStateStartButton == INPUT_ON)
        {
          buttonStateStartButton  = digitalRead(pinI2_0_START_BUTTON);
          RS232.println("{HOME}");
          Serial1.println("{HOME}");         
          Serial.println("{HOME}");
          #if RH
          client1.write("{HOME}");
          #endif
          CommandString = "{HOME}";
         // RS485.println("{HOME}");
        }
      }      
      else
      {
         BuzzerBeep();
      }
     } // end of Home Position     
    } // end of start button
    /**********************************************************/
    //reverse button action
    testingInPut();
    if ((buttonStateGantryControl == INPUT_OFF) && (buttonStateCrossOverControl == INPUT_OFF) && (buttonStateJackUpDnControl == INPUT_OFF) \ 
    && (buttonStateJackFwdRevControl == INPUT_OFF) && (buttonStateWheelControl == INPUT_OFF))
    {
      Serial.println("All Switches off");
    //if ((buttonStatePhotoSensor == HIGH) && (buttonStateJackFwdRevEnd == INPUT_ON)) //&& (buttonStateJackUpDnEnd == INPUT_ON) && (buttonStateCrossOverEnd == INPUT_ON) //changed on 8 jun 2021
     if ((buttonStatePhotoSensor == LOW) && (buttonStateJackFwdRevEnd == INPUT_ON)) //&& (buttonStateJackUpDnEnd == INPUT_ON) && (buttonStateCrossOverEnd == INPUT_ON)
    {
      Serial.println("All states at end position");
    if (buttonStateReverseButton == INPUT_ON )
    {
      Serial.println("Reverse button pressed");
     // All to Home position
     testingInPut();
    buttonStateJackFwdRevHome = digitalRead(pinI1_1_JACK_FWDREV_HOME);
    while(buttonStateJackFwdRevHome == INPUT_OFF)
    {
      onJackReverse();
      buttonStateJackFwdRevHome = digitalRead(pinI1_1_JACK_FWDREV_HOME);  
    }
     offJack();
     BuzzerBeep();
     
    while (buttonStateJackUpDnHome == INPUT_OFF)
    {
      onJackDown();
      buttonStateJackUpDnHome = digitalRead(pinI1_0_JACK_UPDN_HOME);
    }          
     offJackUpDown();
     BuzzerBeep();
     
    while (buttonStateCrossOverHome == INPUT_OFF)    
    {
      onCrossOverReverse();
      buttonStateCrossOverHome = digitalRead(pinI0_3_CROSS_OVER_HOME);
    }         
    offCrossOver();
    BuzzerBeep();
 
    }  
    }
    }
    startOperation();
   } // of selction 
} // end of auto mode
     
        
     

void scanSencorValues()
{
   if(startFlag == 3)
   {
      fineTuneILRValues();
   }
}

void maintainceMode()
{  
  testingInPut();
  if((buttonStateAutoManualSel  == INPUT_ON) && (buttonStateAutoMode  == INPUT_OFF))
  {
   // if((buttonStatePhotoSensor == LOW) && (buttonStateJackFwdRevEnd == INPUT_ON) )//changed on 8 jun 2021
    if((buttonStatePhotoSensor == HIGH) && (buttonStateJackFwdRevEnd == INPUT_ON) )
    {
      if(buttonStateStartButton == INPUT_ON)
      {
          Serial1.println("{DRAX}");
          Serial1.println("{DRAX}");
          Serial1.println("{DRAX}");
          
          RS232.println("{DRAX}");
          RS232.println("{DRAX}");
          RS232.println("{DRAX}");         
          Serial.println("{DRAX}");
          CommandString =  "{DRAX}";
     }
     if(buttonStateForwardButton == INPUT_ON)
      {
          Serial1.println("{DDAX}");
          Serial1.println("{DDAX}");
          Serial1.println("{DDAX}");
          
          RS232.println("{DDAX}");
          RS232.println("{DDAX}");
          RS232.println("{DDAX}");         
          Serial.println("{DDAX}");
          CommandString =  "{DDAX}";
     }
    }
  }
  #if 0
  //testingInPut();
  /************************************************************************/
   if((buttonStateAutoManualSel  == INPUT_ON) && (buttonStateAutoMode  == INPUT_OFF))
  {
    //if((buttonStatePhotoSensor == LOW) && (buttonStateJackFwdRevEnd == INPUT_ON) ) //changed on 8 jun 2021 
     if((buttonStatePhotoSensor == HIGH) && (buttonStateJackFwdRevEnd == INPUT_ON) )
    {
      if(buttonStateForwardButton == INPUT_ON)
      {
          Serial1.println("{DDAX}");
          Serial1.println("{DDAX}");
          Serial1.println("{DDAX}");
          
          RS232.println("{DDAX}");
          RS232.println("{DDAX}");
          RS232.println("{DDAX}");         
          Serial.println("{DDAX}");
          CommandString =  "{DDAX}";
     }
    }
  }
  #endif
  /***************************************************************************/
  //if((buttonStatePhotoSensor == LOW) && (buttonStateJackFwdRevEnd == INPUT_ON) )
   // {
  
  if((buttonStateAutoManualSel  == INPUT_OFF) && (buttonStateAutoMode  == INPUT_OFF))
  {
     testingInPut();
     while(((buttonStateGantryControl  == INPUT_ON) && (buttonStateCrossOverControl == INPUT_OFF)) && ( (buttonStateJackUpDnControl == INPUT_OFF) && (buttonStateJackFwdRevControl == INPUT_OFF)) \
           && ((buttonStateWheelControl == INPUT_OFF)) && (buttonStateAutoManualSel  == INPUT_OFF) && (buttonStateAutoMode  == INPUT_OFF) )
    {
      testingInPut();
//      if ((buttonStatePhotoSensor == HIGH) && (buttonStateJackFwdRevEnd != INPUT_ON)){ //changed on 8 jun 2021
if ((buttonStatePhotoSensor == LOW) && (buttonStateJackFwdRevEnd != INPUT_ON)){
      if(buttonStateForwardButton  == INPUT_ON)
     {
      onGantryAxelTwo();
      Serial.println("Axel 2");
     }
     if(buttonStateReverseButton == INPUT_ON)
     {
       onGantryAxelOne();
       Serial.println("Axel 1");
     }
     if(buttonStateStartButton == INPUT_ON)
     {
       onGantryHome();
       Serial.println("Axel Home");

     }}
          
     if((buttonStateForwardButton == INPUT_OFF) && (buttonStateReverseButton == INPUT_OFF) && (buttonStateStartButton == INPUT_OFF) )
     {
       offGantry();
       break;
     }  
    } // end of while Gantery
    if (buttonStateGantryControl  == INPUT_OFF)
    {
      offGantry();
    }

  while((buttonStateGantryControl  == INPUT_OFF) && (buttonStateCrossOverControl == INPUT_OFF) && (buttonStateJackUpDnControl == INPUT_OFF) && (buttonStateJackFwdRevControl == INPUT_OFF) \
       && (buttonStateWheelControl == INPUT_ON) && (buttonStateAutoManualSel  == INPUT_OFF) && (buttonStateAutoMode  == INPUT_OFF))
   { 
    testingInPut();   
    if(buttonStateForwardButton == INPUT_ON)
     {

      //speedMistMS();
      //onMistForward();
      
      onWheelForward();
     }
     if(buttonStateReverseButton == INPUT_ON)
     {
      //speedMistMS();
      //onMistReverse();
      
      onWheelReverse();
     }
     if((buttonStateForwardButton == INPUT_OFF) && (buttonStateReverseButton == INPUT_OFF) )
     {
      //offMist();
       offWheel();
       testingInPut();
     }
   }
   if ((buttonStateWheelControl == INPUT_OFF))
    {
      offWheel();
    }
     /* Jack Forward Reveser Checking */
   while ((buttonStateGantryControl  == INPUT_OFF) && (buttonStateCrossOverControl == INPUT_OFF) && (buttonStateJackUpDnControl == INPUT_OFF) && (buttonStateJackFwdRevControl == INPUT_ON) \
       && (buttonStateWheelControl == INPUT_OFF) && (buttonStateAutoManualSel  == INPUT_OFF) && (buttonStateAutoMode  == INPUT_OFF))
   {
    testingInPut();
    if(buttonStateForwardButton == INPUT_ON)
     {
      onJackForward();
     }
     if(buttonStateReverseButton == INPUT_ON)
     {
      onJackReverse();
     }
     if((buttonStateForwardButton == INPUT_OFF) && (buttonStateReverseButton == INPUT_OFF) )
     {
       testingInPut();
       offJack();
     }    
   }
    if (buttonStateJackFwdRevControl == INPUT_OFF)
    {
      offJack();
    }

    /* Jack Up down Control*/
  while ((buttonStateGantryControl  == INPUT_OFF) && (buttonStateCrossOverControl == INPUT_OFF) && (buttonStateJackUpDnControl == INPUT_ON) && (buttonStateJackFwdRevControl == INPUT_OFF) \
       && (buttonStateWheelControl == INPUT_OFF) && (buttonStateAutoManualSel  == INPUT_OFF) && (buttonStateAutoMode  == INPUT_OFF))
   {
    testingInPut();
    if(buttonStateForwardButton == INPUT_ON)
     {
      onJackUp();
     }
     if(buttonStateReverseButton == INPUT_ON)
     {
      onJackDown();
     }
     if((buttonStateForwardButton == INPUT_OFF) && (buttonStateReverseButton == INPUT_OFF) )
     {
       testingInPut();
       offJackUpDown();
     }    
   }
    if ( buttonStateJackUpDnControl == INPUT_OFF)
    {
      offJackUpDown();
     }

     /* Cross Over Control*/
  while ((buttonStateGantryControl  == INPUT_OFF) && (buttonStateCrossOverControl == INPUT_ON) && (buttonStateJackUpDnControl == INPUT_OFF) && (buttonStateJackFwdRevControl == INPUT_OFF) \
       && (buttonStateWheelControl == INPUT_OFF) && (buttonStateAutoManualSel  == INPUT_OFF) && (buttonStateAutoMode  == INPUT_OFF))
   {
      testingInPut();
      if(buttonStateForwardButton == INPUT_ON)
      {
        onCrossOverForward();
      }
       if(buttonStateReverseButton == INPUT_ON)
      {
        onCrossOverReverse();
      }
     if((buttonStateForwardButton == INPUT_OFF) && (buttonStateReverseButton == INPUT_OFF) )
     {       
       offCrossOver();
     }    
   }
   if (buttonStateCrossOverControl == INPUT_OFF)
   {
    offCrossOver();
    }   
   }  
}

void maintainceMode_old()
{
  
 if((buttonStateAutoManualSel  == INPUT_OFF) && (buttonStateAutoMode  == INPUT_OFF))
 {
    //startFlag = 0;
    testingInPut();
    startFlag = MAINTAINCE_MODE;
    if (  ((buttonStateGantryControl  == INPUT_OFF) && (buttonStateCrossOverControl == INPUT_OFF)) && ( (buttonStateJackUpDnControl == INPUT_OFF) && (buttonStateJackFwdRevControl == INPUT_OFF)) \
           && ((buttonStateWheelControl == INPUT_OFF)))
      {        
        offAllOuputs(); 
      }
/*
   if ((buttonStateGantryControl  == INPUT_OFF) && (buttonStateCrossOverControl == INPUT_OFF) && (buttonStateJackUpDnControl = INPUT_OFF) && (buttonStateJackFwdRevControl == INPUT_OFF) \
       && (buttonStateWheelControl == INPUT_OFF))
   {
      offAllOuputs();
   }
   else*/
   { 
    /*SEW Motor Checking*/
    if (  ((buttonStateGantryControl  == INPUT_ON) && (buttonStateCrossOverControl == INPUT_OFF)) && ( (buttonStateJackUpDnControl == INPUT_OFF) && (buttonStateJackFwdRevControl == INPUT_OFF)) \
           && ((buttonStateWheelControl == INPUT_OFF)) && (buttonStateAutoManualSel  == INPUT_OFF) && (buttonStateAutoMode  == INPUT_OFF) )
    {
      // buttonStateReverseButton = digitalRead(pinI1_10_REVRSE_BUTTON);
      // buttonStateForwardButton = digitalRead(pinI1_8_FORWARD_BUTTON);
     testingInPut();
     if(buttonStateForwardButton  == INPUT_ON)
     {
      onGantryAxelTwo();

     //  onGantryForward();
      }
     if(buttonStateReverseButton == INPUT_ON)
     {
       onGantryAxelOne();
       

//        onGantryReverse();  
      //  Serial.print("Reserve");    
     }
     if(buttonStateStartButton == INPUT_ON)
     {
       onGantryHome();
//        onGantryReverse();  
      //  Serial.print("Reserve");    
     }
          
     /*if((buttonStateForwardButton == INPUT_OFF) && (buttonStateReverseButton == INPUT_OFF) && (buttonStateStartButton == INPUT_OFF) )
     {
       offGantry();
       testingInPut();
      // BuzzerBeep();
       delay(100);
     }*/
   }
   /* Wheel Checking Oriental Motor*/
  if ((buttonStateGantryControl  == INPUT_OFF) && (buttonStateCrossOverControl == INPUT_OFF) && (buttonStateJackUpDnControl == INPUT_OFF) && (buttonStateJackFwdRevControl == INPUT_OFF) \
       && (buttonStateWheelControl == INPUT_ON) && (buttonStateAutoManualSel  == INPUT_OFF) && (buttonStateAutoMode  == INPUT_OFF))
   { 
    testingInPut();   
    if(buttonStateForwardButton == INPUT_ON)
     {

      //speedMistMS();
      //onMistForward();
      
      onWheelForward();
     }
     if(buttonStateReverseButton == INPUT_ON)
     {
      //speedMistMS();
      //onMistReverse();
      
      onWheelReverse();
     }
     if((buttonStateForwardButton == INPUT_OFF) && (buttonStateReverseButton == INPUT_OFF) )
     {
      //offMist();
       offWheel();
       testingInPut();
     }
   }
    /* Jack Forward Reveser Checking */
   if ((buttonStateGantryControl  == INPUT_OFF) && (buttonStateCrossOverControl == INPUT_OFF) && (buttonStateJackUpDnControl == INPUT_OFF) && (buttonStateJackFwdRevControl == INPUT_ON) \
       && (buttonStateWheelControl == INPUT_OFF) && (buttonStateAutoManualSel  == INPUT_OFF) && (buttonStateAutoMode  == INPUT_OFF))
   {
    testingInPut();
    if(buttonStateForwardButton == INPUT_ON)
     {
      onJackForward();
     }
     if(buttonStateReverseButton == INPUT_ON)
     {
      onJackReverse();
     }
     if((buttonStateForwardButton == INPUT_OFF) && (buttonStateReverseButton == INPUT_OFF) )
     {
       testingInPut();
       offJack();
     }    
   }
   /* Jack Up down Control*/
  if ((buttonStateGantryControl  == INPUT_OFF) && (buttonStateCrossOverControl == INPUT_OFF) && (buttonStateJackUpDnControl == INPUT_ON) && (buttonStateJackFwdRevControl == INPUT_OFF) \
       && (buttonStateWheelControl == INPUT_OFF) && (buttonStateAutoManualSel  == INPUT_OFF) && (buttonStateAutoMode  == INPUT_OFF))
   {
    if(buttonStateForwardButton == INPUT_ON)
     {
      onJackUp();
     }
     if(buttonStateReverseButton == INPUT_ON)
     {
      onJackDown();
     }
     if((buttonStateForwardButton == INPUT_OFF) && (buttonStateReverseButton == INPUT_OFF) )
     {
       testingInPut();
       offJackUpDown();
     }    
   }
 /* Cross Over Control*/
  if ((buttonStateGantryControl  == INPUT_OFF) && (buttonStateCrossOverControl == INPUT_ON) && (buttonStateJackUpDnControl == INPUT_OFF) && (buttonStateJackFwdRevControl == INPUT_OFF) \
       && (buttonStateWheelControl == INPUT_OFF) && (buttonStateAutoManualSel  == INPUT_OFF) && (buttonStateAutoMode  == INPUT_OFF))
   {
    if(buttonStateForwardButton == INPUT_ON)
     {
      onCrossOverForward();
     }
     if(buttonStateReverseButton == INPUT_ON)
     {
      onCrossOverReverse();
     }
     if((buttonStateForwardButton == INPUT_OFF) && (buttonStateReverseButton == INPUT_OFF) )
     {
       
       offCrossOver();
     }    
   }   
  } // end of big else
  } // end if(bitRead(flagBitForInput,bitAutoManualSel) == INPUT_ON)
  else
  {
    /*
    if(startFlag == MAINTAINCE_MODE)
    {
      offAllOuputs();
      startFlag = 0;
      BuzzerBeep();
      delay(1000);
      testingInPut();
    }
   */ 
  }   
}

void midSpeedGantry()
{
  digitalWrite(pinQ0_1_SEW_SPEED_1, ON_OUTPUT);
  digitalWrite(pinQ0_2_SEW_SPEED_2, OFF_OUTPUT);   
}
void offSpeedGantry()
{
  digitalWrite(pinQ0_1_SEW_SPEED_1, OFF_OUTPUT);
  digitalWrite(pinQ0_2_SEW_SPEED_2, OFF_OUTPUT);   
}

void onGantryForward()
{
  digitalWrite(pinQ1_6_SEW_PIN_2, ON_OUTPUT);  
  digitalWrite(pinQ1_7_SEW_PIN_3, ON_OUTPUT);  
  digitalWrite(pinQ2_1_SEW_PIN_4, OFF_OUTPUT);
  /*
 // Serial.println("Moving Gantry Forward");
  if (buttonStateGantryModel_1 == HIGH)
  {
  // Serial.println("Moving Gantry Forward stopped");
   offGantry();
   
  }
  else
  {
   if( buttonStateCrossOverHome == HIGH)
   {
     midSpeedGantry();
     digitalWrite(pinQ0_5_SEW_REV, OFF_OUTPUT); 
     digitalWrite(pinQ0_4_SEW_FWD, ON_OUTPUT);
   }
   else
   {
      offGantry();
   }
  } */
}

void offGantry()
{
 //Serial.println("Gantry Stopped");
  digitalWrite(pinQ1_6_SEW_PIN_2, OFF_OUTPUT);  
  digitalWrite(pinQ1_7_SEW_PIN_3, OFF_OUTPUT);  
  digitalWrite(pinQ2_1_SEW_PIN_4, OFF_OUTPUT);
/*
 offSpeedGantry();
 digitalWrite(pinQ0_5_SEW_REV, OFF_OUTPUT); 
 digitalWrite(pinQ0_4_SEW_FWD, OFF_OUTPUT); */
// startFlag = 0; 
 //if (buttonStateGantryModel_1 == HIGH)
 // {
 //  Serial.println("Moving Gantry Forward stopped");
 // }   
}

void onGantryHome()
{
  digitalWrite(pinQ1_6_SEW_PIN_2, ON_OUTPUT);  
  digitalWrite(pinQ1_7_SEW_PIN_3, OFF_OUTPUT);  
  digitalWrite(pinQ2_1_SEW_PIN_4, ON_OUTPUT);
}

void onGantryAxelOne()
{
  digitalWrite(pinQ1_6_SEW_PIN_2, ON_OUTPUT);  
  digitalWrite(pinQ1_7_SEW_PIN_3, ON_OUTPUT);  
  digitalWrite(pinQ2_1_SEW_PIN_4, OFF_OUTPUT);
}
void onGantryAxelTwo()
{
  digitalWrite(pinQ1_6_SEW_PIN_2, ON_OUTPUT);  
  digitalWrite(pinQ1_7_SEW_PIN_3, ON_OUTPUT);  
  digitalWrite(pinQ2_1_SEW_PIN_4, ON_OUTPUT);
}

void onGantryReverse()
{
  digitalWrite(pinQ1_6_SEW_PIN_2, ON_OUTPUT);  
  digitalWrite(pinQ1_7_SEW_PIN_3, OFF_OUTPUT);  
  digitalWrite(pinQ2_1_SEW_PIN_4, ON_OUTPUT);
  
  /*
  //Serial.println("Moving Gantry Reverse");
  if((buttonStateGantryHome == INPUT_ON) || (buttonStateGantryStart  == INPUT_ON)) 
  {
    offGantry();
  }
  else
  {
    if( buttonStateCrossOverHome == HIGH)
   {
    midSpeedGantry();
    digitalWrite(pinQ0_4_SEW_FWD, OFF_OUTPUT);  
    digitalWrite(pinQ0_5_SEW_REV, ON_OUTPUT); 
   }
   else
   {
    offGantry();
    }
  }*/
}

void onWheelForward()
{
  //Serial.println("On Wheel Forward");
  digitalWrite(pinQ1_5_AC_ORIENTAL_MOTOR_ANTI_CLCK, OFF_OUTPUT);
  digitalWrite(pinQ1_4_AC_ORIENTAL_MOTOR_CLCK, ON_OUTPUT);
}

void onWheelReverse()
{
 // Serial.println("Moving Wheel Reverse");
  digitalWrite( pinQ1_4_AC_ORIENTAL_MOTOR_CLCK, OFF_OUTPUT);
  digitalWrite(pinQ1_5_AC_ORIENTAL_MOTOR_ANTI_CLCK,ON_OUTPUT);  
}

void offWheel()
{
 //Serial.println("Wheel Stopped");
 digitalWrite( pinQ1_4_AC_ORIENTAL_MOTOR_CLCK, OFF_OUTPUT);
 digitalWrite(pinQ1_5_AC_ORIENTAL_MOTOR_ANTI_CLCK,OFF_OUTPUT);  
}

void onJackForward()
{
  //Serial.println("Jack Forward");
  
 if(buttonStateJackFwdRevEnd == INPUT_ON) 
  {
    offJack();
  }
  else
  {
    digitalWrite(pinQ1_3_INDUCTION_MOTOR_REV, OFF_OUTPUT);
    digitalWrite(pinQ1_2_INDUCTION_MOTOR_FWD, ON_OUTPUT);
  }  
}

void onJackForwardForCorrection()
{
    digitalWrite(pinQ1_3_INDUCTION_MOTOR_REV, OFF_OUTPUT);
    digitalWrite(pinQ1_2_INDUCTION_MOTOR_FWD, ON_OUTPUT);
}

void onJackReverse()
{
 // Serial.println("Jack Reverse");
  buttonStateJackFwdRevHome = digitalRead(pinI1_1_JACK_FWDREV_HOME);
  if(buttonStateJackFwdRevHome == INPUT_ON) 
  {
    offJack();
  }
  else
  {
    digitalWrite( pinQ1_2_INDUCTION_MOTOR_FWD, OFF_OUTPUT);
    digitalWrite(pinQ1_3_INDUCTION_MOTOR_REV,ON_OUTPUT); 
  }
}

void offJack()
{
// Serial.println("Jack Stopped");
 digitalWrite( pinQ1_2_INDUCTION_MOTOR_FWD, OFF_OUTPUT);
 digitalWrite(pinQ1_3_INDUCTION_MOTOR_REV,OFF_OUTPUT);  
}

void onJackUp()
{
 // Serial.println("Jack Up");
  if(buttonStateJackUpDnEnd == INPUT_ON) 
  {
    offJackUpDown();
  }
  else
  {
    digitalWrite(pinQ1_1_JACK_DN, OFF_OUTPUT);
    digitalWrite(pinQ1_0_JACK_UP, ON_OUTPUT);
  }
 /* while (((bitRead(flagBitForInput,bitJackUpDnEnd) == INPUT_OFF)) && ((bitRead(flagBitForInput,bitForward ) == INPUT_ON)))
  {
     testingInPut();
  }  
  offJackUpDown(); */
  
}

void onJackDown()
{
  //Serial.println("Jack Down");
  if(buttonStateJackUpDnHome == INPUT_ON) 
  {
    offJackUpDown();
  }
  else
  {
    digitalWrite( pinQ1_0_JACK_UP, OFF_OUTPUT);
    digitalWrite(pinQ1_1_JACK_DN,ON_OUTPUT);
  } 
}

void offJackUpDown()
{
// Serial.println("Jack Up DownStopped");
 digitalWrite( pinQ1_0_JACK_UP, OFF_OUTPUT);
 digitalWrite(pinQ1_1_JACK_DN,OFF_OUTPUT);  
}

void onCrossOverForward()
{
 // Serial.println("Moving CrossOver Forward");
  if(buttonStateCrossOverEnd == INPUT_ON) 
  {
    offCrossOver();
  }
  else
  {
   // if( (buttonStateGantryAxel2Pos == HIGH) || (buttonStateGantryHomePos == HIGH))
    {
    digitalWrite(pinQ0_7_CROSS_REV, OFF_OUTPUT); 
    digitalWrite(pinQ0_6_CROSS_FWD, ON_OUTPUT);
    }
   /* if( (startFlag == 4 ) && (buttonStateGantryHomePos == HIGH))
    {
    digitalWrite(pinQ0_7_CROSS_REV, OFF_OUTPUT); 
    digitalWrite(pinQ0_6_CROSS_FWD, ON_OUTPUT);
    }*/
  //  else
    {
   //   offCrossOver();
    }
  }   
}

void offCrossOver()
{
 //Serial.println("CrossOver Stopped");
 digitalWrite(pinQ0_6_CROSS_FWD, OFF_OUTPUT); 
 digitalWrite(pinQ0_7_CROSS_REV, OFF_OUTPUT);    
}

void onCrossOverReverse()
{
  //Serial.println("Moving CrossOver Reverse"); 
  if(buttonStateCrossOverHome == INPUT_ON)
  {
    offCrossOver();
  }
  else
  {     
 if (( buttonStateJackUpDnHome == HIGH) && (buttonStateJackFwdRevHome == HIGH))
 {
   digitalWrite(pinQ0_6_CROSS_FWD, OFF_OUTPUT);  
   digitalWrite(pinQ0_7_CROSS_REV, ON_OUTPUT);
 }
 else
   {
    offCrossOver();
    }
  }
}
 
void testingOutPut()
{
 #if 0
 digitalWrite(pinQ0_1_SEW_SPEED_1, ON_OUTPUT);   // turn the LED on (HIGH is the voltage level)
 digitalWrite(pinQ0_2_SEW_SPEED_2, ON_OUTPUT);   // turn the LED on (HIGH is the voltage level)
 digitalWrite(pinQ0_4_SEW_FWD, ON_OUTPUT);   // turn the LED on (HIGH is the voltage level)
 digitalWrite(pinQ0_5_SEW_REV, ON_OUTPUT);   // turn the LED on (HIGH is the voltage level)
 digitalWrite(pinQ0_6_CROSS_FWD, ON_OUTPUT);
 digitalWrite(pinQ0_7_CROSS_REV, ON_OUTPUT);
 digitalWrite(pinQ1_0_JACK_UP, ON_OUTPUT);
 digitalWrite(pinQ1_1_JACK_DN, ON_OUTPUT);

 //digitalWrite(pinQ1_2_INDUCTION_MOTOR_FWD, ON_OUTPUT);
 //digitalWrite(pinQ1_3_INDUCTION_MOTOR_REV, ON_OUTPUT);
// digitalWrite(pinQ1_4_AC_ORIENTAL_MOTOR_CLCK, ON_OUTPUT);
 //digitalWrite(pinQ1_5_AC_ORIENTAL_MOTOR_ANTI_CLCK, ON_OUTPUT);
 digitalWrite(pinQ2_5_TOWER_LAMP_RED, ON_OUTPUT);
 digitalWrite(pinQ2_6_TOWER_LAMP_GREEN, ON_OUTPUT);
 digitalWrite(pinQ0_3_TOWER_LAMP_YELLOW, ON_OUTPUT);
 //digitalWrite(pinQ2_3_TOWER_LAMP_BUZZER, ON_OUTPUT);
 
 delay(1000);
 #endif 
 #if 1
 digitalWrite(pinQ0_1_SEW_SPEED_1, OFF_OUTPUT);   // turn the LED on (HIGH is the voltage level)
 digitalWrite(pinQ0_2_SEW_SPEED_2, OFF_OUTPUT);   // turn the LED on (HIGH is the voltage level)
 digitalWrite(pinQ0_4_SEW_FWD, OFF_OUTPUT);   // turn the LED on (HIGH is the voltage level)
 digitalWrite(pinQ0_5_SEW_REV, OFF_OUTPUT);   // turn the LED on (HIGH is the voltage level)
 digitalWrite(pinQ0_6_CROSS_FWD, OFF_OUTPUT);
 digitalWrite(pinQ0_7_CROSS_REV, OFF_OUTPUT);
 digitalWrite(pinQ1_0_JACK_UP, OFF_OUTPUT);
 digitalWrite(pinQ1_1_JACK_DN, OFF_OUTPUT);

 digitalWrite(pinQ1_2_INDUCTION_MOTOR_FWD, OFF_OUTPUT);
 digitalWrite(pinQ1_3_INDUCTION_MOTOR_REV, OFF_OUTPUT);
 digitalWrite(pinQ1_4_AC_ORIENTAL_MOTOR_CLCK, OFF_OUTPUT);
 digitalWrite(pinQ1_5_AC_ORIENTAL_MOTOR_ANTI_CLCK, OFF_OUTPUT);
 digitalWrite(pinQ2_5_TOWER_LAMP_RED, OFF_OUTPUT);
 digitalWrite(pinQ2_6_TOWER_LAMP_GREEN, OFF_OUTPUT);
// digitalWrite(pinQ0_3_TOWER_LAMP_YELLOW, OFF_OUTPUT);
 digitalWrite(pinQ2_3_TOWER_LAMP_BUZZER, OFF_OUTPUT); 
 delay(1000);
 #endif
 digitalWrite(pinQ2_5_TOWER_LAMP_RED, ON_OUTPUT);
 //digitalWrite(pinQ2_6_TOWER_LAMP_GREEN, ON_OUTPUT);
 //digitalWrite(pinQ0_3_TOWER_LAMP_YELLOW, ON_OUTPUT);
// digitalWrite(pinQ2_3_TOWER_LAMP_BUZZER, ON_OUTPUT);
 
 // wait for a second
 //digitalWrite(pinQ0_1_SEW_SPEED_1, OFF_OUTPUT);   // turn the LED on (HIGH is the voltage level)
 //digitalWrite(pinQ0_2_SEW_SPEED_2, OFF_OUTPUT);   // turn the LED on (HIGH is the voltage level)
 //digitalWrite(pinQ0_3_SEW_SPEED_3, OFF_OUTPUT);   // turn the LED on (HIGH is the voltage level)
 //digitalWrite(pinQ0_4_SEW_FWD, OFF_OUTPUT);   // turn the LED on (HIGH is the voltage level)
 //digitalWrite(pinQ0_5_SEW_REV, OFF_OUTPUT);   // turn the LED on (HIGH is the voltage level)
 //delay(1000);
}
void setup() {
  // put your setup code here, to run once:
  int b;
  initGpioOutPut();
  initiGPIOInPut();
  Serial1.begin(9600);
  Serial.begin(9600);
  RS232.begin(9600);
  #if RH
  Ethernet.begin(mac, ip);
  server.begin();
  server1.begin();
  #endif
 // RS485.begin(9600);
  testingOutPut();
  Serial.println(ver);
  Serial.println("Modified on ");
  Serial.println(compile_date);
  Serial.println("Testing I/O lines");
   
}

void loop() {
  // put your main code here, to run repeatedly:
 
//***************************************************************************************************  
//  timerValue = 100;
////for(int i=0; i< 10000; i++)
////{
////   pulseMistON();  
////      delayMicroseconds(timerValue);
////      pulseMistOFF();
////      delayMicroseconds(timerValue);
////  }
//MotorControlStopFrontSensor();
//delay(2000);
//MotorControlStopBackSensorforScaning();
////MotorControlStopBackSensor();
////MotorControlStopBackSensorRH();
//
//while(1);
//******************************************************************************************************
    /*while (1)
    {
    if (RS232.available())
    {
      char data_ser = RS232.read();
      RS232.println(data_ser);
      
    }
    }*/
    #if RH
      client = server.available();
      client1 = server1.available();
      readDataTcp();
    #endif
   // writeDataTcp();
    testingInPut();
    
    maintainceMode();
    autoMode();
    readMasterCommands();
//    computerControl();
    fineTuneILRValues();   
    //PrintUARTSerial(CommandString);  


}

void readDataTcp()
{
   
    // read bytes from the incoming client and write them back
    // to the same client connected to the server
   if (client.available())
   {
    wc = client.read();
    //Serial.write(wc);
    if (wc == 123)
    { 
      array_stringTcp[0] = wc;
      //Serial.write(array_stringTcp[0]);
      indexTcp = 1;
    }
    else if ((array_stringTcp[0] == 123) && (wc != 125) )  
     {
       array_stringTcp[indexTcp] = wc;
       //Serial.write(array_stringTcp[indexTcp]); 
       indexTcp = indexTcp + 1;
     }
    // if ((wc == '}'))
     //  Serial.println(wc);
     if ((array_stringTcp[0] == 123) && (wc == 125) )
     {
       array_stringTcp[indexTcp] = wc;
       indexTcp = 0;
       for ( indexTcp = 0;indexTcp < 3;indexTcp++)
         Serial.write(array_string[index]);
       DataReadFlag = 1;
       commadFlag = 1;
       indexTcp = 0;
     }  
 }
}

void writeDataTcp()
{
  unsigned char indexTcpWr;
  if (DataReadFlag == 1)
  {
    //client1.write("{ALLOK}");
          
   indexTcpWr = 0;
   while (array_stringTcp[indexTcpWr] != '}')
   {
     client1.write(array_stringTcp[indexTcpWr]);
     Serial.write(array_stringTcp[indexTcpWr]);
    // array_stringTcp[indexTcpWr] = 0;
     indexTcpWr = indexTcpWr +1;
   }
     client1.write('}');   
     //DataReadFlag = 0;      
  }    
}

void MotorControlStopBackSensor()
{
  unsigned long currentTime =  micros;
  dirMistFwd();
  noOfPulsesCount = 0;
  buttonStateRotSensorBack = digitalRead(pinI0_6_ROT_SENSOR_BACK);
  //Serial.println("In 8 Case");
  if (buttonStateRotSensorBack == LOW)
  {
    while (buttonStateRotSensorBack == LOW)
    {
      buttonStateRotSensorBack = digitalRead(pinI0_6_ROT_SENSOR_BACK);
      pulseMistON();  
      delayMicroseconds(timerValue);
      pulseMistOFF();
      delayMicroseconds(timerValue);
      noOfPulsesCount = noOfPulsesCount + 1;
      if (noOfPulsesCount > 110000)
      {
        //client1.write("{SB-FALT}");
        errorInSensorSerial("{SB-FALT}");
        
        break;
      } 
    }
    pulseMistOFF();
    if (sensorFault == 0)
    {
      pulseMistOFF();
        RS232.println("{MBCK}"); 
        Serial1.println("{MBCK}"); 
        Serial.println("{MBCK}");

        RS232.println("{MBCK}"); 
        Serial1.println("{MBCK}"); 
        Serial.println("{MBCK}");

        RS232.println("{MBCK}"); 
        Serial1.println("{MBCK}"); 
        Serial.println("{MBCK}");


        RS232.println("{MBCK}"); 
        Serial1.println("{MBCK}"); 
        Serial.println("{MBCK}");
        CommandString = "{MBCK}" ;
        //client1.write("{MBCK}");
    }    
  }
  else
  {
    pulseMistOFF();
      RS232.println("{MBCK}");
      Serial1.println("{MBCK}"); 
      Serial.println("{MBCK}");

      RS232.println("{MBCK}");
      Serial1.println("{MBCK}"); 
      Serial.println("{MBCK}");

      RS232.println("{MBCK}");
      Serial1.println("{MBCK}"); 
      Serial.println("{MBCK}");

      RS232.println("{MBCK}");
      Serial1.println("{MBCK}"); 
      Serial.println("{MBCK}");
      CommandString = "{MBCK}";
     // client1.write("{MBCK}");
  } 
}

void MotorControlStopBackSensorRH()
{
  unsigned long currentTime =  micros;
  dirMistFwd();
  noOfPulsesCount = 0;
  buttonStateRotSensorBack = digitalRead(pinI0_6_ROT_SENSOR_BACK); //digitalRead(pinI0_6_ROT_SENSOR_BACK);
  //Serial.println("In 8 Case");
  if (buttonStateRotSensorBack == LOW)
  {
    while (buttonStateRotSensorBack == LOW)
    {
      buttonStateRotSensorBack = digitalRead(pinI0_6_ROT_SENSOR_BACK); // digitalRead(pinI0_6_ROT_SENSOR_BACK);
      pulseMistON();  
      delayMicroseconds(timerValue);
      pulseMistOFF();
      delayMicroseconds(timerValue);
      noOfPulsesCount = noOfPulsesCount + 1;
      if (noOfPulsesCount > 110000)
      {
        #if RH
         client1.write("{SB-FALT}");
        #endif
        CommandString = "{SB-FALT}";
        Serial1.print(CommandString);
        RS232.println(CommandString);
        errorInSensorTCP("{SB-FALT}");
        
        break;
      } 
    }
    pulseMistOFF();
    if (sensorFault == 0)
    {
      pulseMistOFF();
        RS232.println("{MBCK}"); 
        Serial1.println("{MBCK}");
        Serial.println("{MBCK}");
        

        RS232.println("{MBCK}"); 
        Serial1.println("{MBCK}");
        Serial.println("{MBCK}");
        #if RH
        client1.write("{MBCK}");
        client1.write("{MBCK}");
        client1.write("{MBCK}");
        #endif
        RS232.println("{MBCK}"); 
        Serial1.println("{MBCK}");
        Serial.println("{MBCK}");
        
        CommandString = "{MBCK}";
    }    
  }
  else
  {
    pulseMistOFF();
      RS232.println("{MBCK}"); 
      Serial1.println("{MBCK}");
      Serial.println("{MBCK}");
      

            RS232.println("{MBCK}"); 
      Serial1.println("{MBCK}");
      Serial.println("{MBCK}");
#if RH
      client1.write("{MBCK}");
      client1.write("{MBCK}");
      client1.write("{MBCK}");
      client1.write("{MBCK}");
#endif
            RS232.println("{MBCK}"); 
      Serial1.println("{MBCK}");
      Serial.println("{MBCK}");
      

            RS232.println("{MBCK}"); 
      Serial1.println("{MBCK}");
      Serial.println("{MBCK}");
            CommandString =  "{MBCK}";
  } 
}


void MotorControlStopBackSensorforScaning()
{
  unsigned long currentTime =  micros;
  dirMistFwd();
  noOfPulsesCount = 0;
  buttonStateRotSensorBack = digitalRead(pinI0_6_ROT_SENSOR_BACK);
  if (buttonStateRotSensorBack == LOW)
  {
    while (buttonStateRotSensorBack == LOW)
    {
      buttonStateRotSensorBack = digitalRead(pinI0_6_ROT_SENSOR_BACK);
      pulseMistON();  
      delayMicroseconds(timerValue);
      pulseMistOFF();
      delayMicroseconds(timerValue);
      noOfPulsesCount = noOfPulsesCount + 1;
      if (noOfPulsesCount > 40000)
      {
      // client1.write("{SB-FALT}"); 
       errorInSensorSerial("{SB-FALT}");
       break;
      }
    }
    pulseMistOFF();
    if (sensorFault == 0)
    {
     noOfPulsesCount = 0;
     while (buttonStateRotSensorBack == HIGH)
     {
      buttonStateRotSensorBack = digitalRead(pinI0_6_ROT_SENSOR_BACK);
      pulseMistON();  
      delayMicroseconds(timerValue);
      pulseMistOFF();
      delayMicroseconds(timerValue);
    }
    pulseMistOFF();
     while (buttonStateRotSensorBack == LOW)
     {
      buttonStateRotSensorBack = digitalRead(pinI0_6_ROT_SENSOR_BACK);
      pulseMistON();  
      delayMicroseconds(timerValue);
      pulseMistOFF();
      delayMicroseconds(timerValue);
      noOfPulsesCount = noOfPulsesCount + 1;
      if (noOfPulsesCount > 25000)
      { 
         //client1.write("{SB-FALT}");
         errorInSensorSerial("{SB-FALT}");
         break;
      }
     }
     pulseMistOFF();
    if( sensorFault == 0)
    { 
      noOfPulsesCount = 0;
      BuzzerBeep();      
      pulseMistOFF();
      RS232.println("{MBCK}"); 
      Serial1.println("{MBCK}");
      Serial.println("{MBCK}");

      RS232.println("{MBCK}"); 
      Serial1.println("{MBCK}");
      Serial.println("{MBCK}");

      RS232.println("{MBCK}"); 
      Serial1.println("{MBCK}");
      Serial.println("{MBCK}");

      RS232.println("{MBCK}"); 
      Serial1.println("{MBCK}");
      Serial.println("{MBCK}"); 
      CommandString = "{MBCK}";                 
      //client1.write("{MBCK}");
    }
   }    
  }  
  else
  {
    pulseMistOFF();
    RS232.println("{MBCK}"); 
    Serial1.println("{MBCK}");
    Serial.println("{MBCK}");

    RS232.println("{MBCK}"); 
    Serial1.println("{MBCK}");
    Serial.println("{MBCK}");

    RS232.println("{MBCK}"); 
    Serial1.println("{MBCK}");
    Serial.println("{MBCK}");

    RS232.println("{MBCK}"); 
    Serial1.println("{MBCK}");
    Serial.println("{MBCK}");
    CommandString = "{MBCK}";
    //client1.write("{MBCK}");
  } 
}


void MotorControlStopBackSensorforScaningRH()
{
  unsigned long currentTime =  micros;
  dirMistFwd();
  noOfPulsesCount = 0;
  buttonStateRotSensorBack = digitalRead(pinI0_6_ROT_SENSOR_BACK); //digitalRead(pinI0_6_ROT_SENSOR_BACK);
  
  if (buttonStateRotSensorBack == LOW)
  {
    while (buttonStateRotSensorBack == LOW)
    {
      buttonStateRotSensorBack = digitalRead(pinI0_6_ROT_SENSOR_BACK); //digitalRead(pinI0_6_ROT_SENSOR_BACK);
      pulseMistON();  
      delayMicroseconds(timerValue);
      pulseMistOFF();
      delayMicroseconds(timerValue);
      noOfPulsesCount = noOfPulsesCount + 1;
      sensorFault = 0;
      if (noOfPulsesCount > 90000)
      {
       //client1.write("{SB-FALT}");  
       errorInSensorTCP("{SB-FALT}");
       break;
      } 
    }
    pulseMistOFF();        
    if (sensorFault == 0)
    {
     noOfPulsesCount = 0;
     buttonStateRotSensorBack = digitalRead(pinI0_6_ROT_SENSOR_BACK);
     timerValue = 50;
     while (buttonStateRotSensorBack == HIGH)
     {
      buttonStateRotSensorBack = digitalRead(pinI0_6_ROT_SENSOR_BACK); //digitalRead(pinI0_6_ROT_SENSOR_BACK);
      pulseMistON();  
      delayMicroseconds(timerValue);
      pulseMistOFF();
      delayMicroseconds(timerValue);
    }
     pulseMistOFF();
     buttonStateRotSensorBack = digitalRead(pinI0_6_ROT_SENSOR_BACK); 
     while (buttonStateRotSensorBack == LOW)
     {
      buttonStateRotSensorBack = digitalRead(pinI0_6_ROT_SENSOR_BACK); //digitalRead(pinI0_6_ROT_SENSOR_BACK);
      pulseMistON();  
      delayMicroseconds(timerValue);
      pulseMistOFF();
      delayMicroseconds(timerValue);
      noOfPulsesCount = noOfPulsesCount + 1;
      if (noOfPulsesCount > 90000)
      { 
        //client1.write("{SB-FALT}");
        errorInSensorTCP("{SB-FALT}");
         break;
      }
     }     
     pulseMistOFF();
    if( sensorFault == 0)
    { 
      noOfPulsesCount = 0;
     // BuzzerBeep();      
      pulseMistOFF();
        RS232.println("{MBCK}"); 
        RS232.println("{MBCK}");
        RS232.println("{MBCK}");
        RS232.println("{MBCK}");
        RS232.println("{MBCK}");
        RS232.println("{MBCK}");
        Serial1.println("{MBCK}");
        Serial1.println("{MBCK}");
        Serial1.println("{MBCK}");
        Serial1.println("{MBCK}");
        Serial.println("{MBCK}");
      #if RH 
        client1.write("{MBCK}");
        client1.write("{MBCK}");
        client1.write("{MBCK}");
        client1.write("{MBCK}");
        #endif
        CommandString = "{MBCK}";
    }
   }    
  }  
  else
  {
    pulseMistOFF();
        RS232.println("{MBCK}");
        RS232.println("{MBCK}");
        RS232.println("{MBCK}");
        RS232.println("{MBCK}");
        RS232.println("{MBCK}"); 
        Serial1.println("{MBCK}");
        Serial.println("{MBCK}");
        
        RS232.println("{MBCK}"); 
        Serial1.println("{MBCK}");
        Serial.println("{MBCK}");
        #if RH
        client1.write("{MBCK}");
        client1.write("{MBCK}");
        client1.write("{MBCK}");
        client1.write("{MBCK}");
        #endif
        
        RS232.println("{MBCK}");
        RS232.println("{MBCK}"); 
        Serial1.println("{MBCK}");
        Serial.println("{MBCK}");        
        
        RS232.println("{MBCK}");
        RS232.println("{MBCK}");
        RS232.println("{MBCK}"); 
        Serial1.println("{MBCK}");
        Serial.println("{MBCK}");
        
        CommandString = "{MBCK}";
  } 
}
/*
void MotorControlStopBackSensorforScaningRH()
{
  unsigned long currentTime =  micros;
  dirMistRev();
  noOfPulsesCount = 0;
  buttonStateRotSensorBack = digitalRead(pinI0_5_ROT_SENSOR_FRONT); //digitalRead(pinI0_6_ROT_SENSOR_BACK);
  if (buttonStateRotSensorBack == LOW)
  {
    while (buttonStateRotSensorBack == LOW)
    {
      buttonStateRotSensorBack = digitalRead(pinI0_5_ROT_SENSOR_FRONT); //digitalRead(pinI0_6_ROT_SENSOR_BACK);
      pulseMistON();  
      delayMicroseconds(timerValue);
      pulseMistOFF();
      delayMicroseconds(timerValue);
      noOfPulsesCount = noOfPulsesCount + 1;
      if (noOfPulsesCount > 40000)
      {
       errorInSensor("SB-FALT");
       break;
      }
    }
    if (sensorFault == 0)
    {
     noOfPulsesCount = 0;
     while (buttonStateRotSensorBack == HIGH)
     {
      buttonStateRotSensorBack = digitalRead(pinI0_5_ROT_SENSOR_FRONT); //digitalRead(pinI0_6_ROT_SENSOR_BACK);
      pulseMistON();  
      delayMicroseconds(timerValue);
      pulseMistOFF();
      delayMicroseconds(timerValue);
    }
    
     while (buttonStateRotSensorBack == LOW)
     {
      buttonStateRotSensorBack = digitalRead(pinI0_5_ROT_SENSOR_FRONT); //digitalRead(pinI0_6_ROT_SENSOR_BACK);
      pulseMistON();  
      delayMicroseconds(timerValue);
      pulseMistOFF();
      delayMicroseconds(timerValue);
      noOfPulsesCount = noOfPulsesCount + 1;
      if (noOfPulsesCount > 20000)
      { errorInSensor("SB-FALT");
         break;
      }
     }
    if( sensorFault == 0)
    { 
      noOfPulsesCount = 0;
      BuzzerBeep();      
      pulseMistOFF();

      for (int i = 0;i<10;i++)
       {
        RS232.println("MBCK"); 
        Serial.println("MBCK");
        }     
    }
   }    
  }  
  else
  {
    pulseMistOFF();
    for (int i = 0;i<10;i++)
    {
        RS232.println("MBCK"); 
        Serial.println("MBCK");
        }
  } 
}

*/
void checkPulse()
{
  timerValue = 50;
  buttonStateRotSensorBack = digitalRead(pinI0_6_ROT_SENSOR_BACK);
  if(buttonStateRotSensorBack == HIGH)
  {
    buttonStateRotSensorFront = digitalRead(pinI0_5_ROT_SENSOR_FRONT);
  if (buttonStateRotSensorFront == LOW)
  {
    while((buttonStateRotSensorFront == LOW) )//|| (noOfPulsesCount < 14351))
    {
      buttonStateRotSensorFront = digitalRead(pinI0_5_ROT_SENSOR_FRONT);
      pulseMistON();  
      delayMicroseconds(timerValue);
      pulseMistOFF();
      delayMicroseconds(timerValue);
      noOfPulsesCount = noOfPulsesCount + 1;            
    }
    pulseMistOFF();
    while(1)
    {
      RS232.println(noOfPulsesCount);
      Serial1.println(noOfPulsesCount);
      //RS232.println("") ; 
     }  
  }
}
}

void errorInSensorTCP(String Str1)
{
  unsigned countBeep = 0;
  while(1)
        {
          while ( (commadFlag == 1) && (array_stringTcp[1] !='2'))
          {
          offAllOuputs();
          offWheel();
          pulseMistOFF();
          //readMasterCommands();
           readDataTcp();
         // writeDataTcp();
          RS232.println(Str1);
          RS232.println(noOfPulsesCount);
          Serial1.println(Str1);
          Serial1.println(noOfPulsesCount);
          //client1.write(Str1);
        #if RH
           client1.write(noOfPulsesCount);
           #endif
         // RS485.println(Str1);
         // RS485.println(noOfPulsesCount);
          if ( countBeep < 10)
          {  countBeep = countBeep + 1;  
             BuzzerBeep();
          } 
          sensorFault = 0;
         }
          sensorFault = 1;
         break;
      }         
 }

void PrintUARTSerial(String Str1)
{  unsigned char i;
    if (Str1 != "")
    {
      Serial1.print(Str1);
      RS232.println(Str1);
      #if RH
      {
         for ( i = 0; Str1[i]!= '\0'; i++)
         client1.print(Str1[i]);        
        }
      #endif  
    }
}
void errorInSensorSerial(String Str1)
{
  unsigned countBeep = 0;
  while(1)
        {
          while ((array_string[1] !='2'))
          {
          offAllOuputs();
          offWheel();
          pulseMistOFF();
          readMasterCommands();
          if(array_string[1] =='2')
            break;
        //  readDataTcp();
         // writeDataTcp();
          RS232.println(Str1);
          RS232.println(noOfPulsesCount);
          Serial1.println(Str1);
          Serial1.println(noOfPulsesCount);
          //client1.write(Str1);
          //client1.write(noOfPulsesCount);
         // RS485.println(Str1);
         // RS485.println(noOfPulsesCount);
          if ( countBeep < 10)
          {  countBeep = countBeep + 1;  
             BuzzerBeep();
          } 
          sensorFault = 0;
         }
          sensorFault = 1;
         break;
      }         
 }
 
void MotorControlStopFrontSensor()
{
  unsigned long currentTime =  micros;
  unsigned char funCount = 0;
  
  dirMistRev();
  noOfPulsesCount = 0;
  buttonStateRotSensorFront = digitalRead(pinI0_5_ROT_SENSOR_FRONT);
  if (buttonStateRotSensorFront == LOW)
  {
    while (buttonStateRotSensorFront == LOW)
    {
      buttonStateRotSensorFront = digitalRead(pinI0_5_ROT_SENSOR_FRONT);
      pulseMistON();  
      delayMicroseconds(timerValue);
      pulseMistOFF();
      delayMicroseconds(timerValue);
      noOfPulsesCount = noOfPulsesCount + 1;
      if (noOfPulsesCount > 40000)
      {
        //client1.write("{S1-FALT}");
        errorInSensorSerial("{S1-FALT}");
        break;
      }        
    }
    pulseMistOFF();
    if (sensorFault == 0)   
    {         
     noOfPulsesCount = 0;
    //BuzzerBeep();
    while( buttonStateRotSensorFront == HIGH)
    {
      buttonStateRotSensorFront = digitalRead(pinI0_5_ROT_SENSOR_FRONT);
      pulseMistON();  
      delayMicroseconds(timerValue);
      pulseMistOFF();
      delayMicroseconds(timerValue);
      noOfPulsesCount = noOfPulsesCount + 1;
      /*if (noOfPulsesCount > 3000)
      {
        errorInSensor("S2-FALT");
        break;
      }*/
     } 
     pulseMistOFF();
     noOfPulsesCount = 0;
    }//BuzzerBeep();
    
    if (sensorFault == 0)   
    { 
    while (buttonStateRotSensorFront == LOW)
    {
      buttonStateRotSensorFront = digitalRead(pinI0_5_ROT_SENSOR_FRONT);
      pulseMistON();  
      delayMicroseconds(timerValue);
      pulseMistOFF();
      delayMicroseconds(timerValue);
      noOfPulsesCount = noOfPulsesCount + 1;
      if (noOfPulsesCount > 40000)  //20000
       {
         //client1.write("{S3-FALT}");
         errorInSensorSerial("{S3-FALT}");
         break;
       }       
    }
    }
    pulseMistOFF();
    if (sensorFault == 0)   
    {    
      noOfPulsesCount = 0;
      pulseMistOFF();
      BuzzerBeep();
        RS232.println("{MFNT}");
        RS232.println("{MFNT}");
        RS232.println("{MFNT}");
        RS232.println("{MFNT}");
        RS232.println("{MFNT}");
        RS232.println("{MFNT}");
        RS232.println("{MFNT}");
        RS232.println("{MFNT}");
        
        Serial1.println("{MFNT}"); 
        Serial.println("{MFNT}");

                RS232.println("{MFNT}");
        Serial1.println("{MFNT}"); 
        Serial.println("{MFNT}");

                RS232.println("{MFNT}");
        Serial1.println("{MFNT}"); 
        Serial.println("{MFNT}");

                RS232.println("{MFNT}");
        Serial1.println("{MFNT}"); 
        Serial.println("{MFNT}");
        CommandString = "{MBCK}";
        //client1.write("{MFNT}");
    }   
  }
  else
  {
    pulseMistOFF();
        RS232.println("{MFNT}"); 
        RS232.println("{MFNT}");
        RS232.println("{MFNT}");
        RS232.println("{MFNT}");
        RS232.println("{MFNT}");
        RS232.println("{MFNT}");
        RS232.println("{MFNT}");
        RS232.println("{MFNT}");
        
        Serial1.println("{MFNT}");
        Serial.println("{MFNT}");
        RS232.println("{MFNT}"); 
        Serial1.println("{MFNT}");
        Serial.println("{MFNT}");
        RS232.println("{MFNT}"); 
        Serial1.println("{MFNT}");
        Serial.println("{MFNT}");
        RS232.println("{MFNT}"); 
        Serial1.println("{MFNT}");
        Serial.println("{MFNT}");
        CommandString = "{MFNT}";
        
        //client1.write("{MFNT}");
  } 
}

void MotorControlStopFrontSensorRH()
{
  unsigned long currentTime =  micros;
  unsigned char funCount = 0;
  
  dirMistRev();
  noOfPulsesCount = 0;
  buttonStateRotSensorFront = digitalRead(pinI0_5_ROT_SENSOR_FRONT);  //pinI0_6_ROT_SENSOR_BACK
  if (buttonStateRotSensorFront == LOW)
  {
    while (buttonStateRotSensorFront == LOW)
    {
      buttonStateRotSensorFront = digitalRead(pinI0_5_ROT_SENSOR_FRONT);  //digitalRead(pinI0_5_ROT_SENSOR_FRONT);
      pulseMistON();  
      delayMicroseconds(timerValue);
      pulseMistOFF();
      delayMicroseconds(timerValue);
      noOfPulsesCount = noOfPulsesCount + 1;
      if (noOfPulsesCount > 90000)
      {
        #if RH
        client1.write("{S1-FALT}");
        #endif
        errorInSensorTCP("{S1-FALT}");
        //RS485.println("{S1-FALT}");
        break;
      }        
    }
    if (sensorFault == 0)   
    {         
     noOfPulsesCount = 0;
    //BuzzerBeep();
    while( buttonStateRotSensorFront == HIGH)
    {
      buttonStateRotSensorFront = digitalRead(pinI0_5_ROT_SENSOR_FRONT);  //digitalRead(pinI0_5_ROT_SENSOR_FRONT);
      pulseMistON();  
      delayMicroseconds(timerValue);
      pulseMistOFF();
      delayMicroseconds(timerValue);
      noOfPulsesCount = noOfPulsesCount + 1;
      /*if (noOfPulsesCount > 3000)
      {
        errorInSensor("S2-FALT");
        break;
      }*/
     } 
     noOfPulsesCount = 0;
    }//BuzzerBeep();
    
    if (sensorFault == 0)   
    { 
    while (buttonStateRotSensorFront == LOW)
    {
      buttonStateRotSensorFront = digitalRead(pinI0_5_ROT_SENSOR_FRONT);  //digitalRead(pinI0_5_ROT_SENSOR_FRONT);
      pulseMistON();  
      delayMicroseconds(timerValue);
      pulseMistOFF();
      delayMicroseconds(timerValue);
      noOfPulsesCount = noOfPulsesCount + 1;
      if (noOfPulsesCount > 90000)  //20000
       {
        #if RH
         client1.write("{S3-FALT}");
         #endif
         errorInSensorTCP("{S3-FALT}");
         //RS485.println("{S3-FALT}");
         break;
       }       
    }
    }
    pulseMistOFF();
    if (sensorFault == 0)   
    {    
      noOfPulsesCount = 0;
      pulseMistOFF();
      BuzzerBeep();
        RS232.println("{MFNT}"); 
        RS232.println("{MFNT}");
        RS232.println("{MFNT}");
        RS232.println("{MFNT}");
        RS232.println("{MFNT}");
        RS232.println("{MFNT}");
        RS232.println("{MFNT}");
        
        Serial1.println("{MFNT}");
        Serial.println("{MFNT}");
        
        
        RS232.println("{MFNT}"); 
        Serial1.println("{MFNT}");
        Serial.println("{MFNT}");
        #if RH 
        client1.write("{MFNT}");
        client1.write("{MFNT}");
        client1.write("{MFNT}");
        client1.write("{MFNT}");
        #endif  
        
        RS232.println("{MFNT}"); 
        Serial1.println("{MFNT}");
        Serial.println("{MFNT}");
        

        RS232.println("{MFNT}"); 
        Serial1.println("{MFNT}");
        Serial.println("{MFNT}");
        
        CommandString = "{MFNT}";
    }   
  }
  else
  {
    pulseMistOFF();
        RS232.println("{MFNT}");
        RS232.println("{MFNT}");
        RS232.println("{MFNT}");
        RS232.println("{MFNT}");
        RS232.println("{MFNT}");
        RS232.println("{MFNT}");
        RS232.println("{MFNT}");
        RS232.println("{MFNT}");
        RS232.println("{MFNT}");
        RS232.println("{MFNT}");
        
        Serial1.println("{MFNT}"); 
        Serial.println("{MFNT}");
        
        RS232.println("{MFNT}");
        Serial1.println("{MFNT}"); 
        Serial.println("{MFNT}");
        #if RH 
        client1.write("{MFNT}");
        client1.write("{MFNT}");
        #endif 
        CommandString = "{MFNT}"; 
        //RS485.println("{MFNT}");
  } 
}

void MotorControlStopFrontWheelRotation()
{
  
  unsigned long currentTime =  micros;
  unsigned char funCount = 0;
  noOfPulsesCount = 0;
  dirMistRev();
  
    buttonStateRotSensorFront = digitalRead(pinI0_5_ROT_SENSOR_FRONT);
    if (buttonStateRotSensorFront == LOW)
    {
      if (wheelRoationFlag == 1)
       {  wheelRoationFlag = 0;
      buttonStateRotSensorFront = digitalRead(pinI0_5_ROT_SENSOR_FRONT);
    while (buttonStateRotSensorFront == LOW)
    {
     // readMasterCommands();
     // if (array_string[1] !='C')
     //  break;
      buttonStateRotSensorFront = digitalRead(pinI0_5_ROT_SENSOR_FRONT);
      onWheelReverse();
      noOfPulsesCount = noOfPulsesCount + 1;
      sensorFault = 0;
      if (noOfPulsesCount > 250000)
      {
        //client1.write("{WT-FALT}");
        errorInSensorSerial("{WT-FALT}");
        break;
      }
            
    }
    delay(500);
    buttonStateRotSensorFront = digitalRead(pinI0_5_ROT_SENSOR_FRONT);
    if (sensorFault == 0)   
    {
    while( buttonStateRotSensorFront == HIGH)
    {
      //delay(50);
      //readMasterCommands();
      //if (array_string[1] !='C')
      // break;
      buttonStateRotSensorFront = digitalRead(pinI0_5_ROT_SENSOR_FRONT);
      onWheelReverse();
     } 
     //funCount = 1;
    //delay(50);
    buttonStateRotSensorFront = digitalRead(pinI0_5_ROT_SENSOR_FRONT);
    noOfPulsesCount = 0;
    while (buttonStateRotSensorFront == LOW)
    {
      // readMasterCommands();
      //if (array_string[1] !='C')
      // break;
       buttonStateRotSensorFront = digitalRead(pinI0_5_ROT_SENSOR_FRONT);
       onWheelReverse();
       sensorFault = 0;
       if (noOfPulsesCount > 100000)
       {
        //client1.write("{WT-FALT}");
        errorInSensorSerial("{WT-FALT}");
        break;
       }
    }    
   }
   if (sensorFault == 0)   
   {  
      //offWheel();
      //delay(100);
      for (int i = 0;i<20;i++)
      {      
        offWheel();
        RS232.println("{MFNT}"); 
        RS232.println("{MFNT}");
        RS232.println("{MFNT}");
        RS232.println("{MFNT}");
        RS232.println("{MFNT}");
        RS232.println("{MFNT}");
        RS232.println("{MFNT}");
        RS232.println("{MFNT}");
        Serial1.println("{MFNT}");
        Serial.println("{MFNT}");
        
        #if RH
        client1.write("{MFNT}");
        client1.write("{MFNT}");
        client1.write("{MFNT}");
        client1.write("{MFNT}");
        #endif
        
        RS232.println("{MFNT}"); 
        Serial1.println("{MFNT}");
        Serial.println("{MFNT}");
        
        
        
        RS232.println("{MFNT}"); 
        Serial1.println("{MFNT}");
        Serial.println("{MFNT}");
        
        RS232.println("{MFNT}"); 
        Serial1.println("{MFNT}");
        Serial.println("{MFNT}");
        
        CommandString = "{MFNT}";
      }
        delay(3000);
        readMasterCommands();
        //break;
        //offAllOuputs();
        
      //}      
   } 
  }  
  }
  else
  {
    offWheel();      
    RS232.println("{MFNT}"); 
    RS232.println("{MFNT}");
    RS232.println("{MFNT}");
    RS232.println("{MFNT}");
    RS232.println("{MFNT}");
    RS232.println("{MFNT}");
    RS232.println("{MFNT}");
    RS232.println("{MFNT}");
    Serial1.println("{MFNT}");
    Serial.println("{MFNT}");   
    RS232.println("{MFNT}"); 
    Serial1.println("{MFNT}");
    Serial.println("{MFNT}");   
    RS232.println("{MFNT}"); 
    Serial1.println("{MFNT}");
    Serial.println("{MFNT}");   
    RS232.println("{MFNT}"); 
    Serial1.println("{MFNT}");
    Serial.println("{MFNT}");
    CommandString = "{MFNT}";               
    //client1.write("{MFNT}");   
  } 
}

void MotorControlStopFrontWheelRotationRH()
{
  unsigned long currentTime =  micros;
  unsigned char funCount = 0;
  noOfPulsesCount = 0;
  //dirMistRev();
  buttonStateRotSensorFront = digitalRead(pinI0_5_ROT_SENSOR_FRONT);
  if (buttonStateRotSensorFront == LOW)
  {
    while (buttonStateRotSensorFront == LOW)
    {
     // readMasterCommands();
    //  if (array_string[1] !='L')  
    //   break;
      buttonStateRotSensorFront = digitalRead(pinI0_5_ROT_SENSOR_FRONT);
      onWheelForward();
      noOfPulsesCount = noOfPulsesCount + 1;
      sensorFault = 0;
      if (noOfPulsesCount > 250000)
      {
        #if RH
        client1.write("{WT-FALT}");
        #endif
        errorInSensorTCP("{WT-FALT}");
        break;
      }      
    }
    delay(500);
    buttonStateRotSensorFront = digitalRead(pinI0_5_ROT_SENSOR_FRONT);
    if (sensorFault == 0)   
    {
    while( buttonStateRotSensorFront == HIGH)
    {
      onWheelForward();
      buttonStateRotSensorFront = digitalRead(pinI0_5_ROT_SENSOR_FRONT);
      
     } 
     //funCount = 1;
     noOfPulsesCount = 0;
     buttonStateRotSensorFront = digitalRead(pinI0_5_ROT_SENSOR_FRONT);
    while (buttonStateRotSensorFront == LOW)
    {
      //readMasterCommands();
      //if (array_string[1] !='L')  
      // break;
      buttonStateRotSensorFront = digitalRead(pinI0_5_ROT_SENSOR_FRONT);
       onWheelForward();
       if (noOfPulsesCount > 100000)
       {
        #if RH
        client1.write("{WT-FALT}");
        #endif
        errorInSensorTCP("{WT-FALT}");
        break;
       }
    }
    offWheel();
    delay(10);        
   }
   if (sensorFault == 0)   
   {
      offWheel();
      for (int i = 0;i<10;i++)
      {
        RS232.println("{MFNT}");
        RS232.println("{MFNT}");
        RS232.println("{MFNT}");
        RS232.println("{MFNT}");
        RS232.println("{MFNT}");
        RS232.println("{MFNT}");
        RS232.println("{MFNT}");
        RS232.println("{MFNT}"); 
        Serial1.println("{MFNT}");
        Serial.println("{MFNT}");
               
        
        RS232.println("{MFNT}"); 
        Serial1.println("{MFNT}");
        Serial.println("{MFNT}");
        #if RH
        client1.write("{MFNT}");        
        client1.write("{MFNT}"); 
        client1.write("{MFNT}");
        client1.write("{MFNT}");
        #endif
        RS232.println("{MFNT}"); 
        Serial1.println("{MFNT}");
        Serial.println("{MFNT}");
                
        RS232.println("{MFNT}"); 
        Serial1.println("{MFNT}");
        Serial.println("{MFNT}");
        
        CommandString = "{MFNT}";
                                        
      }
      delay(3000);
      readMasterCommands();
    }   
  }
  else
  {
    offWheel();
      //for (int i = 0;i<10;i++)
       {
        RS232.println("{MFNT}");
        RS232.println("{MFNT}");
        RS232.println("{MFNT}");
        RS232.println("{MFNT}");
        RS232.println("{MFNT}");
        RS232.println("{MFNT}");
        RS232.println("{MFNT}");
        RS232.println("{MFNT}");
        RS232.println("{MFNT}");
        RS232.println("{MFNT}");
        RS232.println("{MFNT}");
         
        Serial1.println("{MFNT}");
        Serial.println("{MFNT}");
        
        RS232.println("{MFNT}"); 
        Serial1.println("{MFNT}");
        Serial.println("{MFNT}");
        
        #if RH
        client1.write("{MFNT}");
        client1.write("{MFNT}");
        client1.write("{MFNT}");
        client1.write("{MFNT}");
        #endif
        
        RS232.println("{MFNT}"); 
        Serial1.println("{MFNT}");
        Serial.println("{MFNT}");
        
        RS232.println("{MFNT}"); 
        Serial1.println("{MFNT}");
        Serial.println("{MFNT}");
        
        CommandString = "{MFNT}";                        
      }
   } 
  //} 
}
//}

void MotorControlScanFrontSensor()
{
  dirMistRev();  
  noOfPulsesCount = 0;
  RS232.println("{MFIS}");
  RS232.println("{MFIS}");
  RS232.println("{MFIS}");
  
  Serial1.println("{MFIS}");
  Serial1.println("{MFIS}");
  Serial1.println("{MFIS}");


  RS232.println("{MFIS}");
  RS232.println("{MFIS}");
  RS232.println("{MFIS}");
  
  Serial1.println("{MFIS}");
  Serial1.println("{MFIS}");
  Serial1.println("{MFIS}");

   RS232.println("{MFIS}");
  RS232.println("{MFIS}");
  RS232.println("{MFIS}");
  
  Serial1.println("{MFIS}");
  Serial1.println("{MFIS}");
  Serial1.println("{MFIS}");
  CommandString = "{MFIS}";
  for ( noOfPulsesCount = 0; noOfPulsesCount < 800; noOfPulsesCount++) //lft1
  {
      pulseMistON();  
      delayMicroseconds(timerValue);
      pulseMistOFF();
      delayMicroseconds(timerValue);
  }  
  pulseMistOFF(); 
  BuzzerBeep();
  noOfPulsesCount = 0;
  RS232.println("{MFIE}");
  RS232.println("{MFIE}");
  RS232.println("{MFIE}");

  Serial1.println("{MFIE}");
  Serial1.println("{MFIE}");
  Serial1.println("{MFIE}");

  RS232.println("{MFIE}");
  RS232.println("{MFIE}");
  RS232.println("{MFIE}");

  Serial1.println("{MFIE}");
  Serial1.println("{MFIE}");
  Serial1.println("{MFIE}");

  RS232.println("{MFIE}");
  RS232.println("{MFIE}");
  RS232.println("{MFIE}");

  Serial1.println("{MFIE}");
  Serial1.println("{MFIE}");
  Serial1.println("{MFIE}");

  RS232.println("{MFIE}");
  RS232.println("{MFIE}");
  RS232.println("{MFIE}");

  Serial1.println("{MFIE}");
  Serial1.println("{MFIE}");
  Serial1.println("{MFIE}");
  CommandString = "{MFIE}";
  //delay(500);
  //readMasterCommands();
  //delay(1000);
}

void MotorControlScanFrontSensor_Aug_21_tested()
{
  dirMistRev();  
  noOfPulsesCount = 0;
  //while ( (commadFlag == 1) && (array_string[1] !='2') && ((array_string[1] !='I') || (array_string[1] =='O') ) )
 // while ( ((array_string[1] !='2') && (array_string[1] !='9')) || ((array_stringTcp[1] !='2') && (array_stringTcp[1] !='9')) )
  while ((array_string[1] !='2') || (array_string[1] !='9') )
  { 
    readMasterCommands();
    if (array_string[1] == '2')
       break;
    if (array_string[1] !='9')
      break;  
    pulseMistON();  
    delayMicroseconds(timerValue);
    pulseMistOFF();
    delayMicroseconds(timerValue);
//    readMasterCommands();
  //  readDataTcp();
    noOfPulsesCount = noOfPulsesCount + 1;
    if (noOfPulsesCount > 3000)
    {
        //client1.write("{FL-ILR}"); 
        errorInSensorSerial("{FL-ILR}");
        break;
    }    
  }
  pulseMistOFF(); 
  BuzzerBeep();
  noOfPulsesCount = 0;
  
}

void MotorControlScanFrontSensorRH()
{
    dirMistRev();  
    noOfPulsesCount = 0;
    //client1.write("{MFIS}");
    //client1.write("{MFIS}");
    # if RH
    client1.write("{MFIS}");
    client1.write("{MFIS}");
    client1.write("{MFIS}");
    client1.write("{MFIS}");
    client1.write("{MFIS}");
    #endif
    CommandString = "{MFIS}";
    Serial.println("{MFIS}");
    Serial1.println("{MFIS}");
    Serial1.println("{MFIS}");
    Serial1.println("{MFIS}");
    Serial1.println("{MFIS}");
   
    RS232.println("{MFIS}");
    RS232.println("{MFIS}");
    RS232.println("{MFIS}");
    RS232.println("{MFIS}");
    RS232.println("{MFIS}");
    RS232.println("{MFIS}");
    for (noOfPulsesCount = 0;noOfPulsesCount<170;noOfPulsesCount++) //ryt1
    {
      pulseMistON();  
      delayMicroseconds(timerValue);
      pulseMistOFF();
      delayMicroseconds(timerValue);
    }
    #if RH
    client1.write("{MFIE}");
    client1.write("{MFIE}");
    client1.write("{MFIE}");
    client1.write("{MFIE}");
    client1.write("{MFIE}");
    client1.write("{MFIE}");
    #endif 
     CommandString= "{MFIE}";
    Serial1.println("{MFIE}"); 
    Serial1.println("{MFIE}");
    Serial1.println("{MFIE}");
    Serial1.println("{MFIE}");
    Serial1.println("{MFIE}"); 
    Serial1.println("{MFIE}");
    Serial1.println("{MFIE}");
    Serial1.println("{MFIE}");
    RS232.println("{MFIE}");
    RS232.println("{MFIE}");
    RS232.println("{MFIE}");
    RS232.println("{MFIE}");
    RS232.println("{MFIE}");
    RS232.println("{MFIE}");
    RS232.println("{MFIE}");
    RS232.println("{MFIE}");
    //client1.write("{MFIE}");
    //client1.write("{MFIE}");
    //BuzzerBeep();
    noOfPulsesCount = 0;
    pulseMistOFF();    
}

void MotorControlScanFrontSensorRH_Aug_26()
{
  dirMistRev();  
  noOfPulsesCount = 0;
 // while ((array_string[1] !='2') && ((array_string[1] !='I') || (array_string[1] =='O') ) )
 // while ( (commadFlag == 1) && (array_string[1] !='2'))
  //while ((array_string[1] !='2') || (array_stringTcp[1] =='I'))
  while ((array_stringTcp[1] =='I'))
  { 
    //readMasterCommands();
    pulseMistON();  
    delayMicroseconds(timerValue);
    pulseMistOFF();
    delayMicroseconds(timerValue);
    
    readDataTcp();
    noOfPulsesCount = noOfPulsesCount + 1;
    if (noOfPulsesCount > 9000)
    {
        #if RH
        client1.write("{FL-ILR}");
        #endif 
        errorInSensorTCP("{FL-ILR}");
        break;
    }    
  } 
  BuzzerBeep();
  noOfPulsesCount = 0;
  pulseMistOFF();
}

void MotorControlScanBackSensor()
{
  dirMistFwd();
  noOfPulsesCount = 0;
  RS232.println("{MBIS}");
  RS232.println("{MBIS}");
  RS232.println("{MBIS}");
  RS232.println("{MBIS}");
  RS232.println("{MBIS}");
  RS232.println("{MBIS}");
  RS232.println("{MBIS}");
  RS232.println("{MBIS}");
  RS232.println("{MBIS}");
  
  Serial1.println("{MBIS}");
  Serial1.println("{MBIS}");
  Serial1.println("{MBIS}");
  Serial1.println("{MBIS}");
  Serial1.println("{MBIS}");
  Serial1.println("{MBIS}");
  Serial1.println("{MBIS}");
  Serial1.println("{MBIS}");
  Serial1.println("{MBIS}");
  Serial1.println("{MBIS}");
  Serial1.println("{MBIS}");
  Serial1.println("{MBIS}");

  Serial1.println("{MBIS}");
  Serial1.println("{MBIS}");
  Serial1.println("{MBIS}");

  CommandString = "{MBIS}";

  for(noOfPulsesCount = 0;noOfPulsesCount < 800;noOfPulsesCount++)//lft2
  {
    pulseMistON();  
    delayMicroseconds(timerValue);
    pulseMistOFF();
    delayMicroseconds(timerValue);
  }
  pulseMistOFF(); 
  BuzzerBeep();
  noOfPulsesCount = 0;
  RS232.println("{MBIE}");
  RS232.println("{MBIE}");
  RS232.println("{MBIE}");
  RS232.println("{MBIE}");
  RS232.println("{MBIE}");
  RS232.println("{MBIE}");
  RS232.println("{MBIE}");
  RS232.println("{MBIE}");
  RS232.println("{MBIE}");


  Serial1.println("{MBIE}");
  Serial1.println("{MBIE}");
  Serial1.println("{MBIE}");
  Serial1.println("{MBIE}");
  Serial1.println("{MBIE}");
  Serial1.println("{MBIE}");
  Serial1.println("{MBIE}");
  Serial1.println("{MBIE}");
  Serial1.println("{MBIE}");
  Serial1.println("{MBIE}");
  Serial1.println("{MBIE}");
  Serial1.println("{MBIE}");
  CommandString = "{MBIE}";      
}

void MotorControlScanBackSensor_Aug_21()
{
  dirMistFwd();
  noOfPulsesCount = 0;
 // while ((array_string[1] !='2'))
  //while ( ((array_string[1] !='2') && (array_string[1] !='B'))|| ((array_stringTcp[1] !='2') && (array_stringTcp[1] !='B')))
  //while ( (commadFlag == 1) && (array_string[1] !='2'))
  while ((array_string[1] !='2') || (array_string[1] !='B') )
  {     
    readMasterCommands();
    if(array_string[1] =='2')
     break;
    if(array_string[1] !='B')
     break; 
    
    pulseMistON();  
    delayMicroseconds(timerValue);
    pulseMistOFF();
    delayMicroseconds(timerValue);
    readMasterCommands();
  //  readDataTcp();
    noOfPulsesCount = noOfPulsesCount + 1;
    if (noOfPulsesCount > 3000)
    {
        //client1.write("{FL-ILR}");
        errorInSensorSerial("{FL-ILR}");
        break;       
    }
  } 
  BuzzerBeep();
  noOfPulsesCount = 0;
  pulseMistOFF();
}

void MotorControlScanBackSensorRH()
{
  dirMistFwd();
  noOfPulsesCount = 0;
  #if RH
  client1.write("{MBIS}");
  client1.write("{MBIS}");
  client1.write("{MBIS}");
  client1.write("{MBIS}");
  client1.write("{MBIS}");
  client1.write("{MBIS}");
  client1.write("{MBIS}");
  #endif
  
  CommandString = "{MBIS}";
  Serial1.println(CommandString);
  Serial1.println(CommandString);
  Serial1.println(CommandString);
  Serial1.println(CommandString);
  Serial1.println(CommandString);
  Serial1.println(CommandString);
  Serial1.println(CommandString); 
  
  RS232.println(CommandString);
  RS232.println(CommandString);
  RS232.println(CommandString);
  RS232.println(CommandString);
  RS232.println(CommandString);
  RS232.println(CommandString);
  RS232.println(CommandString); 
 
 // client1.write("{MBIS}");
 // client1.write("{MBIS}");
  for (noOfPulsesCount = 0;noOfPulsesCount < 170;noOfPulsesCount++) //ryt2
  {
      pulseMistON();  
      delayMicroseconds(timerValue);
      pulseMistOFF();
      delayMicroseconds(timerValue);
  } 
  BuzzerBeep();
  noOfPulsesCount = 0;
  pulseMistOFF();
  #if RH
  client1.write("{MBIE}");
  #endif
  CommandString = "{MBIE}";
  Serial1.println(CommandString);
  Serial1.println(CommandString);
  Serial1.println(CommandString);
  Serial1.println(CommandString);
  Serial1.println(CommandString);
  Serial1.println(CommandString);
  RS232.println(CommandString);
  RS232.println(CommandString);
  RS232.println(CommandString);
  RS232.println(CommandString);
  RS232.println(CommandString);
  RS232.println(CommandString);
  RS232.println(CommandString);
  
  //client1.write("{MBIE}");
  //client1.write("{MBIE}");  
}

void MotorControlScanBackSensorRH_Aug_26()
{
  dirMistFwd();
  noOfPulsesCount = 0;
  //array_string[1] = '0';
  while (((array_string[1] !='2') && ((array_string[1] =='K') ||(array_string[1] =='Q'))) || ((array_stringTcp[1] !='2') && ((array_stringTcp[1] =='K') ||(array_stringTcp[1] =='Q'))) )
  
  //while ( (commadFlag == 1) && (array_string[1] !='2') && (array_string[1] =='K') )
  //while ((array_string[1] !='2'))
  {    
    pulseMistON();  
    delayMicroseconds(timerValue);
    pulseMistOFF();
    delayMicroseconds(timerValue);
   // readMasterCommands();
    readDataTcp();    
    noOfPulsesCount = noOfPulsesCount + 1;
    if (noOfPulsesCount > 9000)
    {
       #if RH
        client1.write("{FL-ILR}");
        #endif
        errorInSensorTCP("{FL-ILR}");
        break;       
    }
  } 
  BuzzerBeep();
  noOfPulsesCount = 0;
  pulseMistOFF();
}
void correctionSystemJack()
{
  while ( (commadFlag == 1) && (array_string[1] !='2'))
  {  
    readMasterCommands();
    onJackForwardForCorrection();       
  } 
  offAllOuputs();
  BuzzerBeep();
}

void alignToSensor()
{
  dirMistRev();
  noOfPulsesCount = 0;
  timerValue = 1000;
  while ( (commadFlag == 1) && (array_string[1] !='2'))
  { 
    
    pulseMistON();  
    delayMicroseconds(timerValue);
    pulseMistOFF();
    delayMicroseconds(timerValue);
    noOfPulsesCount = noOfPulsesCount + 1;
    if (noOfPulsesCount > 500)
    {
      #if RH
        client1.write("{FL-ILR}");
        #endif
        errorInSensorTCP("{FL-ILR}");
        break;       
    }
  } 
  BuzzerBeep();
  noOfPulsesCount = 0;
  pulseMistOFF(); 
  offAllOuputs();
  //BuzzerBeep();  
}

void fineTuneILRValues()
{
  unsigned char swState = 0;
  int count_i = 0;
  testingInPut();
  if(commadFlag == 1)
  {
   //if (buttonStatePhotoSensor  == LOW) //changed on 8 jun 2021
   if (buttonStatePhotoSensor  == HIGH)
   { 
    sensorFault = 0;
    noOfPulsesCount = 0;
    
    if (swState == 0 )
    {
      swState = array_string[1];
    }
    if (swState == 0 )
    {
      swState = array_stringTcp[1];
      array_stringTcp[0] = 0;
      array_stringTcp[1] = 0;
      array_stringTcp[2] = 0;
      commadFlag = 0;
      DataReadFlag = 0;
      Serial.println("TCp Data");
      Serial.println(swState);
    }

    if(swState !=PrvswState)
    { 
      PrvswState = swState;
     switch (swState)
    {
        case '1':
              offWheel();
              speedMistHS();        
              onMistForward();      
        break;
         
        case '2': 
              pulseMistOFF();
              offMist();
              offWheel();
              AlignILRFormeasurment = 1;
              jackReverseFlag = 0;
              BuzzerBeep();
              CommandString = "";
              PrvswState = 0;
        break;
        
        case '3':
              speedMistHS();        
              onMistReverse();      
        break;
         
        case '4':               
              onWheelReverse();
        break;
       
        case '5': 
              onWheelForward();
        break; 

        case '6':
              AlignILRFormeasurment = 1;
        break;
        
        case '7':
              offWheel();
              timerValue = timval;//50;
              MotorControlStopFrontSensor();
        break;
        
        case '8':
              offWheel();
              timerValue  = timval;//50;
              MotorControlStopBackSensor();
        break;
        
        case '9':
              offWheel();
              //timerValue  = 2500;
              timerValue  = 5000;
              MotorControlScanFrontSensor();
        break;

        case 'A':
                offWheel();
                timerValue  = timval;//50;
                MotorControlStopBackSensorforScaning();
        break;

        case 'B':
              offWheel();  
              //timerValue  = 2500;
              timerValue  = 5000;
              MotorControlScanBackSensor();
              wheelRoationFlag = 1;
        break;
        case 'C':
              
              MotorControlStopFrontWheelRotation();
              jackReverseFlag = 0;
              break;

        case 'D':
              checkPulse();
              break;     
       case 'E':
              correctionSystemJack();
              break;    
       case 'F':
             alignToSensor();
             break;        

       case 'G':
              offWheel();
              timerValue = timval;//50;
              MotorControlStopBackSensorRH();
              break;
        
       case 'H':
              timerValue  = timval;//50;
              offWheel();
              MotorControlStopFrontSensorRH();
              break;
       case 'I':
              timerValue  = 3000;
              offWheel();              
              MotorControlScanFrontSensorRH();
              break;
       case 'J':
              timerValue  = timval;//50;
              offWheel();
              MotorControlStopBackSensorforScaningRH();
              //MotorControlStopBackSensorRH();
              break;
       case 'K': 
              timerValue  = 3000;
              offWheel();              
              MotorControlScanBackSensorRH();
              jackReverseFlag = 0;
              break; 
       case 'L': 
             if (jackReverseFlag == 0)
             {
              jackReverseFlag = 1;
              count_i = 0;
              while( count_i < 2)
              {
              
               onJackReverse();
               count_i++;
               delay(2000);
              } 
             }            
             offJack();
             for (int i = 0;i< 5;i++)
             {
                RS232.println("{JFNT}");
                Serial1.println("{JFNT}");              
                Serial.println("{JFNT}");
                #if RH
                  client1.write("{JFNT}");
                #endif
                CommandString = "{JFNT}";
             }
            // MotorControlStopFrontWheelRotationRH();
             break;
       case 'M': 
             timerValue  = timval;//50;
             offWheel();
             MotorControlStopBackSensorRH();
             break;
       case 'N': 
             timerValue  = timval;//50;
             offWheel();
             MotorControlStopFrontSensorRH();
             break;
       case 'O': 
             timerValue  = 3000;
             offWheel();              
             MotorControlScanFrontSensorRH();
             break;                  
       case 'P': 
             timerValue  = timval;//50;
             offWheel();
             MotorControlStopBackSensorforScaningRH();
             break;
       case 'Q':
             timerValue  = 3000;
             offWheel();              
             MotorControlScanBackSensorRH();
             break;             
       case 'R':
             MotorControlStopFrontWheelRotationRH();
             break;  
       case 'S':       
             if (jackReverseFlag == 0)
             {  jackReverseFlag = 1;
                buttonStateJackFwdRevEnd = digitalRead(pinI1_2_JACK_FWDREV_END);
               while( buttonStateJackFwdRevEnd == INPUT_OFF)
               {
                buttonStateJackFwdRevEnd = digitalRead(pinI1_2_JACK_FWDREV_END);
                onJackForward();
               }
             }             
             offJack();
             for (int i = 0;i< 5;i++)
             {
                RS232.println("{JSTP}");
                Serial1.println("{JSTP}");
                Serial.println("{JSTP}");
                #if RH
                client1.write("{JSTP}");
                #endif
                CommandString = "{JSTP}";
             }
             break;  
        case 'Z':              
              while(array_stringTcp[1] != '2')
              {
                Serial.println("Correction");
                readMasterCommands();
                onJackForward();
                ui16counterJackFwd = ui16counterJackFwd + 1;
                if((array_stringTcp[1] == '2'))
                {
                  offJack();
                  ui16counterJackFwd = 0;
                  break;  
                }                
              }
              ui16counterJackFwd = 0;
              offJack();
              BuzzerBeep();
              for (int i = 0;i< 5;i++)
              {
                RS232.println("{JCRR}");
                Serial1.println("{JCRR}");
                Serial.println("{JCRR}");
                #if RH
                client1.write("{JCRR}");
                #endif
                CommandString = "{JCRR}";
             }                
        break;
                     
        default:
            break;    
   }
   }
  }
    commadFlag = 0;
    DataReadFlag = 0;
  }
}


void readMasterCommands()
{  
  String commandString;  
  if(Serial1.available())
 {   
//   TXRXFlag = 0; 
   //TCPIP_data = RS232.read();
   TCPIP_data = Serial1.read();
   if (TCPIP_data == '{')
   { 
      array_string[0] = TCPIP_data;
      array_stringTcp[0] = TCPIP_data;
      index = 1;
   }
   else if ((array_string[0] == '{') && (TCPIP_data != '}') && (index <= 2) )  
   {
       array_string[index] = TCPIP_data;
       array_stringTcp[index] = TCPIP_data;
       index = index + 1;
   }
   if ((array_string[0] == '{') && (TCPIP_data == '}') )
   {
       array_string[index] = TCPIP_data;
       array_stringTcp[index] = TCPIP_data;
       commadFlag = 1;
       DataReadFlag = 1;      
       for ( forIndex = 0;forIndex <=index; forIndex++)
        {  
           Serial.write(array_string[forIndex]);
        }
       index = 0;
       
       //RS232.print("FSTP");    
   }
   
   //Serial.print(d);  
 }
 readMasterCommandsRS232();
 /*
 if(RS485.available())
 {
   TCPIP_data = RS485.read();
   if (TCPIP_data == '{')
   { 
      array_string[0] = TCPIP_data;
      index = 1;
   }
   else if ((array_string[0] == '{') && (TCPIP_data != '}') )  
   {
       array_string[index] = TCPIP_data;
       index = index + 1;
   }
   if ((array_string[0] == '{') && (TCPIP_data == '}') )
   {
       array_string[index] = TCPIP_data;
       commadFlag = 1;      
       for ( forIndex = 0;forIndex <=index; forIndex++)
        {  
           Serial.write(array_string[forIndex]);
           //RS485.write(array_string[forIndex]);
        }
       index = 0;
       
       //RS232.print("FSTP");    
   }   
   //Serial.print(d);  
 } */  
 
}

void readMasterCommandsRS232()
{  
  String commandString;  
  if(RS232.available())
 {   
//   TXRXFlag = 0; 
   TCPIP_data = RS232.read();
   //TCPIP_data = Serial1.read();
   if (TCPIP_data == '{')
   { 
      array_string[0] = TCPIP_data;
      array_stringTcp[0] = TCPIP_data;
      index = 1;
   }
   else if ((array_string[0] == '{') && (TCPIP_data != '}') && (index <= 2) )  
   {
       array_string[index] = TCPIP_data;
       array_stringTcp[index] = TCPIP_data;
       index = index + 1;
   }
   if ((array_string[0] == '{') && (TCPIP_data == '}') )
   {
       array_string[index] = TCPIP_data;
       array_stringTcp[index] = TCPIP_data;
       commadFlag = 1;
       DataReadFlag = 1;      
       for ( forIndex = 0;forIndex <=index; forIndex++)
        {  
           Serial.write(array_string[forIndex]);
        }
       index = 0;
       
       //RS232.print("FSTP");    
   }
   
   //Serial.print(d);  
 }
 /*
 if(RS485.available())
 {
   TCPIP_data = RS485.read();
   if (TCPIP_data == '{')
   { 
      array_string[0] = TCPIP_data;
      index = 1;
   }
   else if ((array_string[0] == '{') && (TCPIP_data != '}') )  
   {
       array_string[index] = TCPIP_data;
       index = index + 1;
   }
   if ((array_string[0] == '{') && (TCPIP_data == '}') )
   {
       array_string[index] = TCPIP_data;
       commadFlag = 1;      
       for ( forIndex = 0;forIndex <=index; forIndex++)
        {  
           Serial.write(array_string[forIndex]);
           //RS485.write(array_string[forIndex]);
        }
       index = 0;
       
       //RS232.print("FSTP");    
   }   
   //Serial.print(d);  
 } */  
}
void stopFrontILR()
{
     //front direction working   
      if((fzAxis <= 0.95) && (fRotationalAngle < 0.00))
      {      
        delay(100);
        offMist();
//        readILRValues();
        //reachedFront = 1;
        //RS232.println("FSTP");
        Serial.print("Reached  ");
        Serial.println(fzAxis);
        if (fzAxis == 0.95)
        {
          reachedFront = 1;
          RS232.println("FSTP");
          Serial1.println("FSTP");
                    RS232.println("FSTP");
          Serial1.println("FSTP");
                    RS232.println("FSTP");
          Serial1.println("FSTP");
        }
      // delay(1000);
      }
      else
      {      
        if((fzAxis > 0.95) )
        {
          if ( (reachedFront == 0))
          {
            speedMistMS();
            //speedMistLS();
            onMistForward();
            // onMistReverse();
            reachedFront = 0;
          }
        }      
     }  
}

void stopbackILR()
{
  if((fzAxis >= 2.95) && (fRotationalAngle < 320.00))
      {
        offMist();
        reachedBack = 1;
        RS232.println("BSTP");
        Serial1.println("BSTP");
        //readILRValues();
      // delay(1000);
      }
      else
      {      
        speedMistMS();
        //speedMistLS();
      //  onMistForward();
        onMistReverse();
      //  reachedFront = 0;      
     }  
}
/*
void serialEvent() {
  while (Serial1.available()) {
    
   // char inChar = (char)Serial.read(); Serial.print(inChar); //Output Original Data, use this code 
  
    Re_buf[counter]=(unsigned char)Serial1.read();
    if(counter==0&&Re_buf[0]!=0x55) return;      //第0号数据不是帧头              
    counter++;       
    if(counter==11)             //接收到11个数据
    {    
       counter=0;               //重新赋值，准备下一帧数据的接收 
       sign=1;
    }      
  }
}

*/
void angleWithPulse()
{
//serialEvent(); 
  if(sign)
  {  
     sign=0;
     if(Re_buf[0]==0x55)      //检查帧头
     {  
  switch(Re_buf [1])
  {
  case 0x51:
    a[0] = (short(Re_buf [3]<<8| Re_buf [2]))/32768.0*16;
    a[1] = (short(Re_buf [5]<<8| Re_buf [4]))/32768.0*16;
    a[2] = (short(Re_buf [7]<<8| Re_buf [6]))/32768.0*16;
    fzAxis = a[2];
   // T = (short(Re_buf [9]<<8| Re_buf [8]))/340.0+36.25;*/ 
    break;
  case 0x52:
    /*w[0] = (short(Re_buf [3]<<8| Re_buf [2]))/32768.0*2000;
    w[1] = (short(Re_buf [5]<<8| Re_buf [4]))/32768.0*2000;
    w[2] = (short(Re_buf [7]<<8| Re_buf [6]))/32768.0*2000;
    T = (short(Re_buf [9]<<8| Re_buf [8]))/340.0+36.25;*/
    break;
  case 0x53:
    angle[0] = (short(Re_buf [3]<<8| Re_buf [2]))/32768.0*180;
    angle[1] = (short(Re_buf [5]<<8 | Re_buf [4])) /32768.0*180;
    angle[2] = (short(Re_buf [7]<<8| Re_buf [6]))/32768.0*180;
    angle[2] = (short(Re_buf [7]<<8| Re_buf [6]))/32768.0*180;
  //  T = (short(Re_buf [9]<<8| Re_buf [8]))/340.0+36.25;
    angle[1] = (angle[1] * 3.14) ;
    fRotationalAngle = angle[1];
    Serial.print("a:");
              //  client.write("a:");
              //  Serial.print(a[0]);Serial.print(" ");
               // Serial.print(a[1]);Serial.print(" ");
                //Serial.print(a[2]);Serial.print(" ");
                Serial.print(fzAxis);Serial.print(" ");                
                //client.write(a[2]);
                
               // Serial.print("w:");
               // Serial.print(w[0]);Serial.print(" ");
               // Serial.print(w[1]);Serial.print(" ");
               // Serial.print(a[2]);Serial.print(" ");
                 Serial.print("angle:");
                 // client.write("angle:");
          //      Serial.print(angle[0]);Serial.print(" ");
               Serial.println(fRotationalAngle);Serial.print(" ");
                //client.write(fRotationalAngle);
      //          Serial.println(angle[2]);Serial.print(" ");
               // Serial.print("T:");
               // Serial.println(T);
                break;
      } 
    }
  }
  
   if((fzAxis > 0.26) &&  ( fzAxis < 1.94) && (fRotationalAngle > 0))
   {
     Serial.println("Third Qudarant");
     mov90DegreeReverse();
   }
   
   if((fzAxis > 1.90) &&  ( fzAxis < 2.96) && (fRotationalAngle > 50))
   {
     ReachedFlag = 0;
     mov90DegreeReverse();
     Serial.println("Fourth Qudarant");
   }
   
   if((fzAxis > 2.01) &&  ( fzAxis < 2.96) && (fRotationalAngle < 0))
   {
     Serial.println("Second Qudarant");
   }
    
   if((fzAxis > 0.94) &&  ( fzAxis < 2.01) && (fRotationalAngle < 0))
   {
     Serial.println("First Qudarant");
   }    
}

void mov90DegreeReverse()
{
  if (ReachedFlag == 0)
  {
   for (int i = 0; i< 2500; i++)
   {
     dirMistFwd();
     pulseMistON();
     delayMicroseconds(100);
     pulseMistOFF();
     delayMicroseconds(100);    
   }
   ReachedFlag = 1;
   pulseMistOFF();
 }
  // while(1);  
}
void angleCalculation()
{
  //serialEvent(); 
  if(sign)
  {  
     sign=0;
     if(Re_buf[0]==0x55)      //检查帧头
     {  
  switch(Re_buf [1])
  {
  case 0x51:
    a[0] = (short(Re_buf [3]<<8| Re_buf [2]))/32768.0*16;
    a[1] = (short(Re_buf [5]<<8| Re_buf [4]))/32768.0*16;
    a[2] = (short(Re_buf [7]<<8| Re_buf [6]))/32768.0*16;
    fzAxis = a[2];
   // T = (short(Re_buf [9]<<8| Re_buf [8]))/340.0+36.25;*/ 
    break;
  case 0x52:
    /*w[0] = (short(Re_buf [3]<<8| Re_buf [2]))/32768.0*2000;
    w[1] = (short(Re_buf [5]<<8| Re_buf [4]))/32768.0*2000;
    w[2] = (short(Re_buf [7]<<8| Re_buf [6]))/32768.0*2000;
    T = (short(Re_buf [9]<<8| Re_buf [8]))/340.0+36.25;*/
    break;
  case 0x53:
    angle[0] = (short(Re_buf [3]<<8| Re_buf [2]))/32768.0*180;
    angle[1] = (short(Re_buf [5]<<8 | Re_buf [4])) /32768.0*180;
    angle[2] = (short(Re_buf [7]<<8| Re_buf [6]))/32768.0*180;

   // float accZval =  a[2];
   // float accXangle = a[0];
   // float accYangle = a[1];
      
    angle[1] = (angle[1] * 3.14) ;
    fRotationalAngle = angle[1];

    if(buttonStateForwardButton == INPUT_ON)
    {
       MoveFront = 1;
       MoveBack = 0;
    } 
    if(buttonStateReverseButton == INPUT_ON) 
    {
       MoveBack = 1;
       MoveFront = 0;
    }
    if (MoveFront == 1)
    {
      stopFrontILR();
      //RS232.print("FSTP");
            
    }
    if (MoveBack == 1)
    {
       stopbackILR();
       //RS232.print("BSTP");
       
    }
/*
   if(accZval < 0)//360 degrees
  {
    if(accXangle < 0)
    {
      accXangle = -180-accXangle;
    }else
    {
      accXangle = 180-accXangle;
    }
    if(accYangle < 0)
    {
      accYangle = -180-accYangle;
    }else
    {
      accYangle = 180-accYangle;
    }
  }
  Serial.print("AccX Angle = ");
  Serial.print(accXangle);
  
  Serial.print("AccY Angle = ");
  Serial.print(accYangle);

  Serial.print("AccZ Angle = ");
  Serial.print(accZval);
   */
   /*     
     //front direction working   
      if((fzAxis < 0.94) && (fRotationalAngle < 0.00))
      {
        offMist();
        reachedFront = 1;
      // delay(1000);
      }
      else
      {      
        speedMistMS();
        //speedMistLS();
        onMistForward();
       // onMistReverse();
        reachedFront = 0;      
     }
  */
    /*  
     //front direction working   
      if((fzAxis > 2.80) && (fRotationalAngle < 0.00))
      {
        offMist();
        reachedBack = 1;
      // delay(1000);
      }
      else
      {      
        speedMistMS();
        //speedMistLS();
        onMistForward();
       // onMistReverse();
        reachedFront = 0;      
     }
 */
  
/*
     if ((fzAxis < 0.94) && (fRotationalAngle < 0.00))
     {
      if ((fRotationalAngle  <= -5.00 ) && (reachedFront == 1))
       {
        if ((fzAxis <= 0.94) && (fRotationalAngle >= 2.00))
        {
          offMist();
          reachedFront = 2;
         // reachedFront = 2;
         // reachedFront = 0;
        }
        speedMistLS();
        //onMistForward();
        onMistReverse();
        }
      }            
    //}
  */   
 
 /*  
   // Move forward direction from top to Front position
   //if((fRotationalAngle <= -98.00) && (fzAxis <= 1.99))
   if((fRotationalAngle <= -98.00) && (fzAxis <= 1.99) && (fRotationalAngle < 0.00))
   {
     onMistForward();
     
     if((fRotationalAngle <= -3.00) && (fzAxis < 0.94))
     {
       offMist();
     }    
   }

// Move Revsers Direction from Bottom to Front Position 
  // if((fRotationalAngle <= 98.00) && (fzAxis <= 1.99))
   if((fRotationalAngle <= 98.00) && (fzAxis <= 1.99) && (fRotationalAngle > 0.00))
   {
     onMistReverse();
     
     if((fRotationalAngle <= 3.00) && (fzAxis < 0.94))
     {
       offMist();
     }    
   }
 */  
/*
    if(( fRotationalAngle > 1.0) && (fRotationalAngle < 6))
    {
       offMist(); // Back Reflector 
    }
    
    if(( fRotationalAngle > 1.0) && (fRotationalAngle < 6))
    {
       offMist(); // Front Refelectro 
    }

    
     
    if ( fRotationalAngle < -6.0) 
    {  
      Serial.println("In High");
      onMistForward();
    }
  
    if ( fRotationalAngle > -4.5) 
    {
       Serial.println("In Low");
      offMist();
    } 
    
  */  
  //  angle[2] = (short(Re_buf [7]<<8| Re_bfRotationalAngleuf [6]))/32768.0*180;
  //  T = (short(Re_buf [9]<<8| Re_buf [8]))/340.0+36.25;
                Serial.print("a:");
 
              //  client.write("a:");
              //  Serial.print(a[0]);Serial.print(" ");
               // Serial.print(a[1]);Serial.print(" ");
                Serial.print(a[2]);Serial.print(" ");
                //client.write(a[2]);
                
               // Serial.print("w:");
               // Serial.print(w[0]);Serial.print(" ");
               // Serial.print(w[1]);Serial.print(" ");
               // Serial.print(a[2]);Serial.print(" ");
                 Serial.print("angle:");
                 // client.write("angle:");
          //      Serial.print(angle[0]);Serial.print(" ");
               Serial.println(fRotationalAngle);Serial.print(" ");
                //client.write(fRotationalAngle);
      //          Serial.println(angle[2]);Serial.print(" ");
               // Serial.print("T:");
               // Serial.println(T);
                break;
  } 
    }
  } 
}
/*
void EthernetRead()
{
  
  unsigned char i,charbuf[10];
  // if an incoming client connects, there will be bytes available to read:
 // client = server.available();
  if (client.available()) {
    // read bytes from the incoming client and write them back
    // to the same client connected to the server
     TCPIP_data = client.read();
     client.write('{');
     dtostrf(a[2],6,3,charbuf);
   // fc.f = a[2];
    //fc.f = a[2];
    //mystring = String(a[2]);
    //gcvt(a[2], 6, buf);
    for (i = 0; i< sizeof(float);i++) 
     client.write(charbuf[i]);
   // client.write(fRotationalAngle);
     client.write('}');
    //client.write(TCPIP_data);
    if (TCPIP_data == '{')
    { 
      array_string[0] = TCPIP_data;
      index = 1;
    }
    else if ((array_string[0] == '{') && (TCPIP_data != '}') )  
     {
       array_string[index] = TCPIP_data;
       index = index + 1;
     }
     if ((array_string[0] == '{') && (TCPIP_data == '}') )
     {
       array_string[index] = TCPIP_data;      
       for ( forIndex = 0;forIndex <=index; forIndex++)
           Serial.write(array_string[forIndex]);
       index = 0;
     }
  }
}*/
