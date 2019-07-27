#include "DualVNH5019MotorShield.h"
/***********************************************************************
 * Name: Christian Yap / Ryan Michal
 * Date: July 26th, 2019
 * Notes: Testing Version 1
 * Program listens to commands coming over RF
 *   A will Turn motor clockwise
 *   B will turn motor counter clockwise
 *   C will expand actuator
 *   D will close actuator 
 *   
 * ---------------------------------------------------------------------
 * WIRING: Dual VNH5019 DC MOTORS 
 * NO WIRING NEEDED WHEN IT IS ATTACHED ON THE ARDUINO.
 * M1CS   -   A0(54)  - Motor 1 current sense output
 * M2CS   -   A1(55)  - Motor 2 current sense output
 * M1EN   -   GPIO(12)- Motor 1 enable input/fault output
 * M2EN   -   GPIO(6) - Motor 2 enable input/fault output
 * M1INA  -   GPIO(2) - Motor 1 Direction Input A
 * M1INB  -   GPIO(4) - Motor 1 Direction Input B
 * M2INA  -   GPIO(7) - Motor 2 Direction Input A
 * M2INB  -   GPIO(8) - Motor 2 Direction Input B
 * M1PWM  -   PWM(9)  - Motor 1 speed input
 * M2PWM  -   PWM(10) - Motor 2 speed input
 * VIN    -   POWER SUPPLY 12 VOLTS
 * GND    -   POWER SUPPLY GND
 * 
 * ---------------------------------------------------------------------
 * WIRING: SINGLE VNH5019 LINEAR ACTUATOR
 * VDD    -   5V Logic Power
 * GND1   -   GND
 * INA    -   GPIO(Pin 16)
 * INB    -   GPIO(Pin 17)
 * PWM    -   GPIO(Pin 3)
 * CS     -   ANALOG 2 (Pin 46) * OPTIONAL
 * VDD    -   5V
 * GND    -   Arduino GND
 * OUTA   -   Linear Actuator +
 * OUTB   -   Linear Actuator -
 * GND    -   Power Supply 12V GND
 * VIN    -   Power Supply 12V +
 * ----------------------------------------------------------------------
 * 
 * WIRING: RF Communication
 * RF_A   -   GPIO(PIN 22) 
 * RF_B   -   GPIO(PIN 23)
 * RF_C   -   GPIO(PIN 24)    
 * RF_D   -   GPIO(PIN 25)
 * VDD    -   Power Supply 12 Volts
 * GND    -   Power Supply GND
 * ----------------------------------------------------------------------
 * 
 * WIRING: Solenoid LEFT (Use IRF MOSFET Board)
 * SIG LEFT   - PWM(44)
 * GND        - Arduino GND (Next to the VCC input)
 * V+         - To Solenoid Load +(Wire direction doesn't matter)
 * V-         - To Solenoid Load -(Wire direction doesn't matter)
 * Vin        - Power Supply 12V
 * GND        - Power Supply GND
 * 
 * ----------------------------------------------------------------------
 * 
 * WIRING: Solenoid RIGHT (Use IRF MOSFET Board)
 * SIG RIGHT  - PWM(46)
 * GND        - Arduino GND (Next to the VCC input)
 * V+         - To Solenoid Load +(Wire direction doesn't matter)
 * V-         - To Solenoid Load -(Wire direction doesn't matter)
 * Vin        - Power Supply 12V
 * GND        - Power Supply GND
 * 
 * ----------------------------------------------------------------------
 * WIRING: SENSORS - LITTELFUSE SENSORS
 * Bottom Left Sensor   - Analog 10 (64)
 * Bottom Right Sensor  - Analog 11 (65)
 * Middle Left Sensor   - Analog 12 (66)
 * Middle Right Sensor  - Analog 13 (67)
 * Top Left Sensor      - Analog 14 (68)
 * Top Right Sensor*    - Analog 15 (69)
 * GND (BLACK WIRE) ------------------------------------------------->ARDUINO GND
 * DATA (BLUE WIRE) -----------1k or 10k resistor  ------v----------->5V ARDUINO
 *                                                  ARDUINO INPUT     ^
 * RED WIRE-----------------------------------------------------------|
 * 
 * ----------------------------------------------------------------------
 * 
 * WIRING: HC04 Bluetooth (if needed)
 * HC-05 RX   - RX(0)
 * HC-05 TX   - TX(1)
***********************************************************************/
DualVNH5019MotorShield md;

//STATE Definitions:
#define REST        0
#define STATE_A     2
#define STATE_B     3
#define STATE_C     4
#define STATE_D     5
#define UNLOCK      6
int state =         REST;

//Pin Declarations - RF PINS:
int RF_A = 22;
int RF_B = 23;
int RF_C = 24;
int RF_D = 25;
int rf_input_position = 0;
int rf_combination[4] = {RF_A, RF_C, RF_A, RF_B};
int rf_last_input;

unsigned long lock_timer_old = 0;
unsigned long lock_timer_new = 0;
unsigned long lock_debouncer = 0;

//Pin Declarations - Actuator Pins
int act_inA = 16;
int act_inB = 17;
int act_PWM = 3;
int act_CS  = 56;

//Pin Declarations - Left Sensors
int bottomLeftSensor = 64;
int middleLeftSensor = 66;
int topLeftSensor = 68;

//Pin Declarations - Right Sensors
int bottomRightSensor = 65;
int middleRightSensor = 67;
int topRightSensor = 69;

//Sensor Statuses:
int statusTopLeftSensor = 0;
int statusMiddleLeftSensor = 0;
int statusBottomLeftSensor = 0;
int statusTopRightSensor = 0;
int statusMiddleRightSensor = 0;
int statusBottomRightSensor = 0;

//Pin Declarations - Left Solenoid
int solenoidLeft = 46;   

//Pin Declarations - Right Solenoid
int solenoidRight = 44;

//BEGIN THE GRIND:
void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);

  //Initialize Remote:
  //initRemoteRF();

  //Initialize DC Motors:
  //initDCMotors();

  //Initialize Linear Actuator:
  //initLinearAct();

  //Initialize Left Sensors:
  initSolenoidLeft();

  //Initialize Right Sensors:
  initSolenoidRight();

  //Initialize Left Solenoid:
  initSolenoidLeft();

  //Initialize Right Solenoid:
  initSolenoidRight();


  state = REST;
  lock_timer_old = millis();
}
/**********************************************************************
 * MAIN LOOP
***********************************************************************/
void loop() {


  //UNLOCKED STATE:
  if(rf_unlock()) 
  {
    // perform open/close as needed
    Serial.println("unlocked");
    delay(5000);
    state = UNLOCK;
    return;
  }
  //LOCKED STATE:
  else 
  {
    state = REST;
    Serial.println("REST");
  }  
  delay(250);

}

//START RF REMOTE FUNCTIONALITY
/**********************************************************************
 * FUNCTION: Initialize LEFT and RIGHT DC Motors
***********************************************************************/
void initRemoteRF()
{
  pinMode(RF_A, INPUT);
  pinMode(RF_B, INPUT);
  pinMode(RF_C, INPUT);
  pinMode(RF_D, INPUT);
}

/**********************************************************************
 * UNLOCK SYSTEM STATE:
***********************************************************************/
int rf_unlock() 
{
  int remote_input = read_remote();
  lock_timer_new = millis();

  /* check time of last input; is this the same sequence? */
  if(lock_timer_new - lock_timer_old > 5000) 
  {
    // waited to long between button clicks restart combination input
    // note overflow will occur every 50 days for timer. on the very unlikely case that
    // that rf is used at this exact moment it will likely fail. 
    Serial.println("timed out new sequence");
    lock_timer_old = millis();
    rf_input_position = 0;     
  } 
  
  /* is this the correct input for this sequence? */
  if( (remote_input > 0) && (remote_input != rf_last_input) && (lock_timer_new - lock_timer_old > 500) ) 
  {
    lock_timer_old = millis();
    Serial.println("click");
  
    switch(rf_input_position) 
    {
      // switch between input_positions
      case 0:
        if(remote_input == rf_combination[0]) 
        {
          // you have the first button right!
          rf_input_position++;
          Serial.println("case 1");
        }
        else 
        {
          rf_input_position = 0;
          Serial.println("case 0");
        }
      break;
        
      case 1:
        if(remote_input == rf_combination[1]) 
        {
            rf_input_position++;
          Serial.println("case 2");
        } 
        else 
        {
          rf_input_position = 0;
          Serial.println("case 0");
        }
      break;
        
      case 2:
        if(remote_input == rf_combination[2]) 
        {
            rf_input_position++;
          Serial.println("case 3");
        } 
        else 
        {
          rf_input_position = 0;
          Serial.println("case 0");
        }
      break;
        
      case 3:
        if(remote_input == rf_combination[3])
        {
            rf_input_position = 0;
            Serial.println("case unlock");
            return 1;
        } 
        else 
        {
            rf_input_position = 0;
            Serial.println("case 0");
        }
      break; 
    }
  }
  
  rf_last_input = remote_input;
  return 0;
}

/**********************************************************************
 * FUNCTION: READ IN REMOTE SIGNAL INPUT
***********************************************************************/
int read_remote() 
{
    //do this by reading the register value?  
    // will be easier for logic. 
    int A_on = digitalRead(RF_A);
    int B_on = digitalRead(RF_B);
    int C_on = digitalRead(RF_C);
    int D_on = digitalRead(RF_D);
    
    if (A_on && !B_on && !C_on && !D_on)  
    {
        return RF_A;
    } 
    else if (B_on && !A_on && !C_on && !D_on) 
    {
        return RF_B;
    } 
    else if (C_on && !A_on && !B_on && !D_on) 
    {
        return RF_C;
    } 
    else if (D_on && !A_on && !C_on && !B_on) 
    {
        return RF_D;
    } 
    else  
    {
        return 0;                 //no input
    }
}


//END RF REMOTE FUNCTIONALITY
//START DC MOTOR RELATED FUNCTIONALITY
/**********************************************************************
 * FUNCTION: Initialize LEFT and RIGHT DC Motors
***********************************************************************/
void initDCMotors()
{
    md.init();
}

/**********************************************************************
 * FUNCTION: ROTATE DC Motors One Way
 * TODO: Need to find out which direction makes tarp roll up and down
***********************************************************************/
void rotateMotorsDirectionOne(int speedInput)
{
     md.setM1Speed(speedInput);
     md.setM2Speed(speedInput);
}

void rotateMotorsDirectionTwo(input speedInput)
{

    md.setM1Speed(-speedInput);
    md.setM2Speed(-speedInput);
  
}

void stopMotors()
{

  
}


//END DC MOTOR RELATED FUNCTIONALITY
//START LINEAR ACTUATOR FUNCTIONALITY
/**********************************************************************
 * FUNCTION: Initialize Linear Actuator
***********************************************************************/
void initLinearAct()
{
  pinMode(act_inA,OUTPUT);
  pinMode(act_inB,OUTPUT);
  pinMode(act_PWM,OUTPUT);

}

/**********************************************************************
 * FUNCTION: Set the Direction of the Linear Actuator
 * Parameters: int - 0 - OFF ; 1 - FORWARD ? ; 2 - BACKWARDS ; 3 - ??
***********************************************************************/
void setDir(int d)
{
  switch(d) 
  {
    case 0: // off?
      digitalWrite(act_inA,LOW);
      digitalWrite(act_inB,LOW);
      break;
    case 1: // forward
      digitalWrite(act_inA,HIGH);
      digitalWrite(act_inB,LOW);
      break;
    case 2:  // backward
      digitalWrite(act_inA,LOW);
      digitalWrite(act_inB,HIGH);
      break;
    case 3:  // locked?
      digitalWrite(act_inA,HIGH);
      digitalWrite(act_inB,HIGH);
      break;
    default:
      digitalWrite(act_inA,LOW);
      digitalWrite(act_inB,LOW);
      break;
  }
}

/**********************************************************************
 * FUNCTION: SLOWLY SLIDE TEH ACTUATOR FORWARD/REVERSE WITH PWM @PIN3
 * TODO: NEED TO READ REMOTE INPUT IN THE FOR LOOP FOR MANUAL OVERRIDE
***********************************************************************/
void slide(int d) {
  for(int v=0;v<256;++v) {
    analogWrite(act_PWM,v);
    delay(d);
  }
  for(int v=255;v>=0;--v) {
    analogWrite(act_PWM,v);
    delay(d);
  }
}

//END LINEAR ACTUATOR FUNCTIONALITY
//START SENSOR FUNCTIONALITY
/**********************************************************************
 * FUNCTION: Initialize Left sensors
***********************************************************************/
void initLeftSensors()
{
  //Pin Declarations - Left Sensors
  pinMode(topLeftSensor, INPUT);
  pinMode(middleLeftSensor, INPUT);
  pinMode(bottomLeftSensor, INPUT);

}

/**********************************************************************
 * FUNCTION: initialize right sensors
***********************************************************************/
void initRightSensors()
{
  //Pin Declarations - Right Sensors
  pinMode(topRightSensor, INPUT);
  pinMode(middleRightSensor, INPUT);
  pinMode(bottomRightSensor, INPUT);
}

/**********************************************************************
 * FUNCTION: Checks each status of the sensor
***********************************************************************/
void updateSensorValues()
{
  statusTopLeftSensor     = digitalRead(topLeftSensor);
  statusMiddleLeftSensor  = digitalRead(middleLeftSensor);
  statusBottomLeftSensor  = digitalRead(bottomLeftSensor);
  
  statusTopRightSensor    = digitalRead(topRightSensor);
  statusMiddleRightSensor = digitalRead(middleRightSensor);
  statusBottomRightSensor = digitalRead(bottomRightSensor);
}

//END SENSOR FUNCTIONALTIY
//START SOLENOID FUNCTIONALITY
/**********************************************************************
 * FUNCTION: Initialize the left solenoid, separated to be able to test
 * ach side individually.
***********************************************************************/
void initSolenoidLeft()
{
  pinMode(solenoidLeft,OUTPUT);// define left as output
}

/**********************************************************************
 * FUNCTION: Initialize the right solenoid
***********************************************************************/
void initSolenoidRight()
{
  pinMode(solenoidRight,OUTPUT);// define left as output
}

/**********************************************************************
 * FUNCTION: Apply current to unlock left solenoid
***********************************************************************/
void unlockLeftSolenoid()
{
  digitalWrite(solenoidLeft,HIGH); // turn the MOSFET Switch ON
}

/**********************************************************************
 * FUNCTION: Apply current to unlock right solenoid
***********************************************************************/
void unlockRightSolenoid()
{
  digitalWrite(solenoidRight,HIGH); // turn the MOSFET Switch ON
}

/**********************************************************************
 * FUNCTION: Apply current to both solenoids to UNLOCK
***********************************************************************/
void unlockBothSolenoid()
{
  digitalWrite(solenoidLeft,HIGH); // turn the MOSFET Switch ON
  digitalWrite(solenoidRight,HIGH); // turn the MOSFET Switch ON
}

/**********************************************************************
 * FUNCTION: Lock both solenoids, turns off current application.
***********************************************************************/
void lockBothSolenoid()
{
  digitalWrite(solenoidLeft,LOW); // Turn the MOSFET Switch OFf
  digitalWrite(solenoidRight,LOW); // Turn the MOSFET Switch OFF
}

//END SOLENOID FUNCTIONALITY
