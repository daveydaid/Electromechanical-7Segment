/*#####################################################################################################
#
# Title:    7_Seg_Electromechanical_Clock_TMC2208
# Version:  3.6b
# Auth:     DMcD
# Date:     01/02/2022
#
# References:
#
# TODO: Make shiftValues_n & numbernPositions values into 2D arrays; then can get rid of large switches
#
#####################################################################################################*/

/*#####################################################################################################
# PROJECT INCLUDES
#####################################################################################################*/
#include <TMCStepper.h>             // For BIGTREETECH-TMC2208-V3.0
#include <TonePlayer.h>             // For playing melody on piezo buzzer
#include <ShiftRegister74HC595.h>   // For NXP/TI Shift Register 74HC595
#include <AccelStepper.h>           // For stepper control
#include <MultiStepper.h>           // For simultaneous stepper movements
#include <DS3231.h>                 // For RTC module (time keeping)
#include <Wire.h>                   // For communicating with DS3231
#include "7seglist.h"               // Contains all possible motor positions 
#include "words.h"                  // Contains all possible words
#include "notes.h"                  // Contains all possible words
#include <math.h>

/*#####################################################################################################
# PREPROCESSOR DIRECTIVES
#####################################################################################################*/
#define DEBUG_MODE  1

#define ENABLE      0
#define DIRECTION   1

#define BLACK_BUTTON_1      3     // SET_TIME Button (Interrupt Enabled) - Active LOW
#define BLACK_BUTTON_2      2     // SET_TIME Button (Interrupt Enabled) - Active LOW 
#define WHITE_BUTTON_1      16    // UP_COUNTER Button - Active LOW
#define WHITE_BUTTON_2      17    // xxxxxxxx Button (unassigned for now) - Active LOW
#define RED_BUTTON          18    // GENERATE_WORD Button (Interrupt Enabled) - Active LOW

#define BUZZER_PIN          46    // Configure as signal pin for the piezo buzzer, must be specific pin based on lib

#define STEP_PIN_0          13    // M0 (A) STEP line - Pin 2 of each BIGTREETECH-TMC2208-V3.0  (MINUTES_UNITS)
#define STEP_PIN_1          12    // M1 (B) STEP line - Pin 2 of each BIGTREETECH-TMC2208-V3.0  (MINUTES_UNITS)
#define STEP_PIN_2          11    // M2 (C) STEP line - Pin 2 of each BIGTREETECH-TMC2208-V3.0  (MINUTES_UNITS)
#define STEP_PIN_3          10    // M3 (D) STEP line - Pin 2 of each BIGTREETECH-TMC2208-V3.0  (MINUTES_UNITS)
#define STEP_PIN_4          9     // M4 (E) STEP line - Pin 2 of each BIGTREETECH-TMC2208-V3.0  (MINUTES_UNITS)
#define STEP_PIN_5          8     // M5 (F) STEP line - Pin 2 of each BIGTREETECH-TMC2208-V3.0  (MINUTES_UNITS)
#define STEP_PIN_6          7     // M6 (G) STEP line - Pin 2 of each BIGTREETECH-TMC2208-V3.0  (MINUTES_UNITS)
#define GROUP0_SHIFT_DATA   6     // DATA line - Pin X of 74CH595
#define GROUP0_SHIFT_CLOCK  5     // CLOCK line - Pin X of 74CH595
#define GROUP0_SHIFT_LATCH  4     // LATCH line - Pin X of 74CH595
#define NUMBER_0            0     // SIMPLE DEFINE FOR HELPING CODE READABILITY THROUGHOUT
#define GROUP_0             0     // ONLY GROUP_0

#define STEP_PIN_7          22    // M0 (A) STEP line - Pin 2 of each BIGTREETECH-TMC2208-V3.0  (MINUTES_TENS)
#define STEP_PIN_8          24    // M1 (B) STEP line - Pin 2 of each BIGTREETECH-TMC2208-V3.0  (MINUTES_TENS)
#define STEP_PIN_9          26    // M2 (C) STEP line - Pin 2 of each BIGTREETECH-TMC2208-V3.0  (MINUTES_TENS)
#define STEP_PIN_10         28    // M3 (D) STEP line - Pin 2 of each BIGTREETECH-TMC2208-V3.0  (MINUTES_TENS)
#define STEP_PIN_11         30    // M4 (E) STEP line - Pin 2 of each BIGTREETECH-TMC2208-V3.0  (MINUTES_TENS)
#define STEP_PIN_12         32    // M5 (F) STEP line - Pin 2 of each BIGTREETECH-TMC2208-V3.0  (MINUTES_TENS)
#define STEP_PIN_13         34    // M6 (G) STEP line - Pin 2 of each BIGTREETECH-TMC2208-V3.0  (MINUTES_TENS)
#define GROUP1_SHIFT_DATA   36    // DATA line - Pin X of 74CH595
#define GROUP1_SHIFT_CLOCK  38    // CLOCK line - Pin X of 74CH595
#define GROUP1_SHIFT_LATCH  40    // LATCH line - Pin X of 74CH595
#define NUMBER_1            1     // SIMPLE DEFINE FOR HELPING CODE READABILITY THROUGHOUT
#define GROUP_1             1     // INCLUDES GROUP_0 AND GROUP_1

#define STEP_PIN_14         23    // M0 (A) STEP line - Pin 2 of each BIGTREETECH-TMC2208-V3.0  (HOURS_UNITS)
#define STEP_PIN_15         25    // M1 (B) STEP line - Pin 2 of each BIGTREETECH-TMC2208-V3.0  (HOURS_UNITS)
#define STEP_PIN_16         27    // M2 (C) STEP line - Pin 2 of each BIGTREETECH-TMC2208-V3.0  (HOURS_UNITS)
#define STEP_PIN_17         29    // M3 (D) STEP line - Pin 2 of each BIGTREETECH-TMC2208-V3.0  (HOURS_UNITS)
#define STEP_PIN_18         31    // M4 (E) STEP line - Pin 2 of each BIGTREETECH-TMC2208-V3.0  (HOURS_UNITS)
#define STEP_PIN_19         33    // M5 (F) STEP line - Pin 2 of each BIGTREETECH-TMC2208-V3.0  (HOURS_UNITS)
#define STEP_PIN_20         35    // M6 (G) STEP line - Pin 2 of each BIGTREETECH-TMC2208-V3.0  (HOURS_UNITS)
#define GROUP2_SHIFT_DATA   37    // DATA line - Pin X of 74CH595
#define GROUP2_SHIFT_CLOCK  39    // CLOCK line - Pin X of 74CH595
#define GROUP2_SHIFT_LATCH  41    // LATCH line - Pin X of 74CH595
#define NUMBER_2            2     // SIMPLE DEFINE FOR HELPING CODE READABILITY THROUGHOUT
#define GROUP_2             2     // INCLUDES GROUP_0, GROUP_1 AND GROUP_2

#define STEP_PIN_21         42    // M0 (A) STEP line - Pin 2 of each BIGTREETECH-TMC2208-V3.0  (HOURS_TENS)
#define STEP_PIN_22         44    // M1 (B) STEP line - Pin 2 of each BIGTREETECH-TMC2208-V3.0  (HOURS_TENS)
#define STEP_PIN_23         48    // M2 (C) STEP line - Pin 2 of each BIGTREETECH-TMC2208-V3.0  (HOURS_TENS)
#define STEP_PIN_24         50    // M3 (D) STEP line - Pin 2 of each BIGTREETECH-TMC2208-V3.0  (HOURS_TENS)
#define STEP_PIN_25         52    // M4 (E) STEP line - Pin 2 of each BIGTREETECH-TMC2208-V3.0  (HOURS_TENS)
#define STEP_PIN_26         53    // M5 (F) STEP line - Pin 2 of each BIGTREETECH-TMC2208-V3.0  (HOURS_TENS)
#define STEP_PIN_27         51    // M6 (G) STEP line - Pin 2 of each BIGTREETECH-TMC2208-V3.0  (HOURS_TENS)
#define GROUP3_SHIFT_DATA   45    // DATA line - Pin X of 74CH595
#define GROUP3_SHIFT_CLOCK  47    // CLOCK line - Pin X of 74CH595
#define GROUP3_SHIFT_LATCH  49    // LATCH line - Pin X of 74CH595
#define NUMBER_3            3     // SIMPLE DEFINE FOR HELPING CODE READABILITY THROUGHOUT
#define GROUP_ALL           3     // INCLUDES ALL GROUPS; GROUP_0, GROUP_1, GROUP_2 AND GROUP_3

#define ALL_NUMBERS         4

#define SERIAL_PORT   Serial3 // TMC2208 HardwareSerial Port

#define R_SENSE 0.11f // Match to your driver
                     // SilentStepStick series use 0.11

#define QUARTER_TURN_VALUE 1028 // 2056 for 16 microsteps // 1028 for 8 microsteps

/*#####################################################################################################
# GLOBAL VARIABLES
#####################################################################################################*/                     
constexpr uint32_t steps_per_mm = 30; // *** TODO: REVIEW THIS VALUE *** was 80 by default

uint8_t loop_flag = 0; // Temp flag for looping routines
// Temp flags for looping routines
uint8_t timeLoop_flag = 0, home_flag = 0, top_flag = 0, numbers_flag = 0, numbersBackwards_flag = 0, words_flag = 0;
// Temp flags for looping routines
uint8_t letters_flag = 0, choose_number_flag = 0, alert_flag = 0, countdown_flag = 0, set_time_buttons_flag = 0, set_TIMER_buttons_flag = 0;

uint8_t shiftValues_0[] = { B00000000, B00000000 }; // Default pin setup for shift regs MINUTES_UNITS
uint8_t shiftENStore_0 = { B00000000 };
uint8_t shiftValues_1[] = { B00000000, B00000000 }; // Default pin setup for shift regs MINUTES_TENS
uint8_t shiftENStore_1 = { B00000000 };
uint8_t shiftValues_2[] = { B00000000, B00000000 }; // Default pin setup for shift regs HOURS_UNITS
uint8_t shiftENStore_2 = { B00000000 };
uint8_t shiftValues_3[] = { B00000000, B00000000 }; // Default pin setup for shift regs HOURS_TENS
uint8_t shiftENStore_3 = { B00000000 };

long number0Positions[8] = {0,0,0,0,0,0,0,666}; // Array of first 7 (0-6) desired stepper positions (666 used as delimiter)
long number1Positions[8] = {0,0,0,0,0,0,0,666}; // Array of second 7 (7-13) desired stepper positions (666 used as delimiter)
long number2Positions[8] = {0,0,0,0,0,0,0,666}; // Array of third 7 (14-20) desired stepper positions (666 used as delimiter)
long number3Positions[8] = {0,0,0,0,0,0,0,666}; // Array of fourth 7 (21-27) desired stepper positions (666 used as delimiter)

TonePlayer piezoBuzzer (TCCR5A, TCCR5B, OCR5AH, OCR5AL, TCNT5H, TCNT5L);  // pin D46

uint16_t secondCounter = 0;
uint16_t time_left = 0;
uint8_t timerMinutes = 0;

// notes of the moledy followed by the duration.
// a 4 means a quarter note, 8 an eighteenth , 16 sixteenth, so on
// !!negative numbers are used to represent dotted notes,
// so -4 means a dotted quarter note, that is, a quarter plus an eighteenth!!
uint16_t melody[9] = {
  NOTE_C7, 16, NOTE_A6, 16, NOTE_B6, 16, NOTE_C7, 8, REST
};
// ^ THIS NEED TO BE GLOBAL. NO IDEA WHY. (Better now that I set the size?)

/*#####################################################################################################
# OBJECT DECLARATIONS
#####################################################################################################*/
// Create a global shift register object to be used for ENABLE and DIR pins for all motors
// parameters: <number of shift registers> (data pin, clock pin, latch pin)
ShiftRegister74HC595<2> MinUnit(GROUP0_SHIFT_DATA, GROUP0_SHIFT_CLOCK, GROUP0_SHIFT_LATCH);  // *****MINUTES_UNITS*****
ShiftRegister74HC595<2> MinTens(GROUP1_SHIFT_DATA, GROUP1_SHIFT_CLOCK, GROUP1_SHIFT_LATCH);  // *****MINUTES_TENS*****
ShiftRegister74HC595<2> HourUnit(GROUP2_SHIFT_DATA, GROUP2_SHIFT_CLOCK, GROUP2_SHIFT_LATCH); // *****HOURS_UNITS*****
ShiftRegister74HC595<2> HourTens(GROUP3_SHIFT_DATA, GROUP3_SHIFT_CLOCK, GROUP3_SHIFT_LATCH); // *****HOURS_TENS*****

// Common UART driver for all connected chips:
TMC2208Stepper driver = TMC2208Stepper(&SERIAL_PORT, R_SENSE); // Hardware Serial3

// Create the instances for each stepper driver *****(MINUTES_UNITS)*****
AccelStepper stepper0 = AccelStepper(stepper0.DRIVER, STEP_PIN_0);
AccelStepper stepper1 = AccelStepper(stepper1.DRIVER, STEP_PIN_1);
AccelStepper stepper2 = AccelStepper(stepper2.DRIVER, STEP_PIN_2);
AccelStepper stepper3 = AccelStepper(stepper3.DRIVER, STEP_PIN_3);
AccelStepper stepper4 = AccelStepper(stepper4.DRIVER, STEP_PIN_4);
AccelStepper stepper5 = AccelStepper(stepper5.DRIVER, STEP_PIN_5);
AccelStepper stepper6 = AccelStepper(stepper6.DRIVER, STEP_PIN_6);

// Create the instances for each stepper driver *****(MINUTES_TENS)*****
AccelStepper stepper7 = AccelStepper(stepper7.DRIVER, STEP_PIN_7);
AccelStepper stepper8 = AccelStepper(stepper8.DRIVER, STEP_PIN_8);
AccelStepper stepper9 = AccelStepper(stepper9.DRIVER, STEP_PIN_9);
AccelStepper stepper10 = AccelStepper(stepper10.DRIVER, STEP_PIN_10);
AccelStepper stepper11 = AccelStepper(stepper11.DRIVER, STEP_PIN_11);
AccelStepper stepper12 = AccelStepper(stepper12.DRIVER, STEP_PIN_12);
AccelStepper stepper13 = AccelStepper(stepper13.DRIVER, STEP_PIN_13);

// Create the instances for each stepper driver *****(HOURS_UNITS)*****
AccelStepper stepper14 = AccelStepper(stepper14.DRIVER, STEP_PIN_14);
AccelStepper stepper15 = AccelStepper(stepper15.DRIVER, STEP_PIN_15);
AccelStepper stepper16 = AccelStepper(stepper16.DRIVER, STEP_PIN_16);
AccelStepper stepper17 = AccelStepper(stepper17.DRIVER, STEP_PIN_17);
AccelStepper stepper18 = AccelStepper(stepper18.DRIVER, STEP_PIN_18);
AccelStepper stepper19 = AccelStepper(stepper19.DRIVER, STEP_PIN_19);
AccelStepper stepper20 = AccelStepper(stepper20.DRIVER, STEP_PIN_20);

// Create the instances for each stepper driver *****(HOURS_TENS)*****
AccelStepper stepper21 = AccelStepper(stepper21.DRIVER, STEP_PIN_21);
AccelStepper stepper22 = AccelStepper(stepper22.DRIVER, STEP_PIN_22);
AccelStepper stepper23 = AccelStepper(stepper23.DRIVER, STEP_PIN_23);
AccelStepper stepper24 = AccelStepper(stepper24.DRIVER, STEP_PIN_24);
AccelStepper stepper25 = AccelStepper(stepper25.DRIVER, STEP_PIN_25);
AccelStepper stepper26 = AccelStepper(stepper26.DRIVER, STEP_PIN_26);
AccelStepper stepper27 = AccelStepper(stepper27.DRIVER, STEP_PIN_27);

// We will handle 7 steppers as a group with MultiStepper (Max is 10)
MultiStepper group0Steppers;    // nn:nY
MultiStepper group1Steppers;    // nn:YY
MultiStepper group2Steppers;    // nY:YY
MultiStepper allSteppers;       // YY:YY

// Declare an instance of our RTC module for time-keeping purposes
DS3231 rtcClock;
bool h12Flag, pmFlag;

/*#####################################################################################################
# FUNCTIONS
#####################################################################################################*/
/*#####################################################################################################
# Function:     updatePositions
#
# Description:  Updates the shift registers after calculating the new positions and motor movements. 
#
# Inputs:       uint8_t a[8] - array containing the desired motor positions to display number or letter  
#               uint8_t NUMBER - which number's motor poisitons should be updated?     
#
# Retuns:       n/a 
#
# TODO:         See top note about arrays
#####################################################################################################*/
void updatePositions(uint8_t a[8], uint8_t NUMBER) {  
  uint8_t i;
  uint8_t b;

  switch (NUMBER)
    {
      case 0: // NUMBER_0 (nn:nY) *******************************************************
      {
//        Serial.println("UPDATING POSITIONS OF NUMBER_0"); 
    
        // Reset DIR signals to 0
        shiftValues_0[DIRECTION] = {B10000000};
        b = 6;

        number0Positions[0] = 666;
        // Determine which direction the motors should turn and update shiftreg[1]
        for (i=1;i<8;i++)
        {
          if (a[i] == 1)
          {
            number0Positions[i] = QUARTER_TURN_VALUE;    // Set value
            shiftValues_0[DIRECTION] |= 1UL << b; // Set direction
          }
          else
          {
            number0Positions[i] = 0;           // Set value
            shiftValues_0[DIRECTION] &= ~(1UL << b);  // Set direction
          }
          b--;
        }

        // Determine which motors should be powered in order to change their current positions....
        // 1. XNOR the current state with our new positions (i.e. inverse XOR)
        shiftValues_0[ENABLE] = getXNOR((uint8_t)shiftValues_0[DIRECTION], (uint8_t)shiftENStore_0);
      
        // 2. 'shiftValues_0[0]' now contains '0' for each motor that requires a change in position, '1' for each motor that doesn't.
      
        // 3. Always ensure to store the current position state
        shiftENStore_0 = shiftValues_0[DIRECTION];
 
        break;
      }
      case 1: // NUMBER_1 (nn:Yn) *******************************************************
      {
//        Serial.println("UPDATING POSITIONS OF NUMBER_1");        
        // Reset DIR signals to 0
        shiftValues_1[DIRECTION] = {B10000000};
        b = 6;

        number1Positions[0] = 666;
        // Determine which direction the motors should turn and update shiftreg[1]
        for (i=1;i<8;i++)
        {
          if (a[i] == 1)
          {
            number1Positions[i] = QUARTER_TURN_VALUE;    // Set value
            shiftValues_1[DIRECTION] |= 1UL << b; // Set direction
          }
          else
          {
            number1Positions[i] = 0;           // Set value
            shiftValues_1[DIRECTION] &= ~(1UL << b);  // Set direction
          }
          b--;
        }

        // Determine which motors should be powered in order to change their current positions....
        // 1. XNOR the current state with our new positions (i.e. inverse XOR)
        shiftValues_1[ENABLE] = getXNOR((uint8_t)shiftValues_1[DIRECTION], (uint8_t)shiftENStore_1);
      
        // 2. 'shiftValues_1[0]' now contains '0' for each motor that requires a change in position, '1' for each motor that doesn't.
      
        // 3. Always ensure to store the current position state
        shiftENStore_1 = shiftValues_1[DIRECTION];
 
        break;
      }
      case 2: // NUMBER_2 (nY:nn) *******************************************************
      {
//        Serial.println("UPDATING POSITIONS OF NUMBER_2");        
        // Reset DIR signals to 0
        shiftValues_2[DIRECTION] = {B10000000};
        b = 6;

        number2Positions[0] = 666;
        // Determine which direction the motors should turn and update shiftreg[1]
        for (i=1;i<8;i++)
        {
          if (a[i] == 1)
          {
            number2Positions[i] = QUARTER_TURN_VALUE;    // Set value
            shiftValues_2[DIRECTION] |= 1UL << b; // Set direction
          }
          else
          {
            number2Positions[i] = 0;           // Set value
            shiftValues_2[DIRECTION] &= ~(1UL << b);  // Set direction
          }
          b--;
        }

        // Determine which motors should be powered in order to change their current positions....
        // 1. XNOR the current state with our new positions (i.e. inverse XOR)
        shiftValues_2[ENABLE] = getXNOR((uint8_t)shiftValues_2[DIRECTION], (uint8_t)shiftENStore_2);
      
        // 2. 'shiftValues_2[0]' now contains '0' for each motor that requires a change in position, '1' for each motor that doesn't.
      
        // 3. Always ensure to store the current position state
        shiftENStore_2 = shiftValues_2[DIRECTION];
  
        break;
      }
      case 3: // NUMBER_3 (Yn:nn) *******************************************************
      {
//        Serial.println("UPDATING POSITIONS OF NUMBER_3");        
        // Reset DIR signals to 0
        shiftValues_3[DIRECTION] = {B10000000};
        b = 6;

        number3Positions[0] = 666;
        // Determine which direction the motors should turn and update shiftreg[1]
        for (i=1;i<8;i++)
        {
          if (a[i] == 1)
          {
            number3Positions[i] = QUARTER_TURN_VALUE;    // Set value
            shiftValues_3[DIRECTION] |= 1UL << b; // Set direction
          }
          else
          {
            number3Positions[i] = 0;           // Set value
            shiftValues_3[DIRECTION] &= ~(1UL << b);  // Set direction
          }
          b--;
        }

        // Determine which motors should be powered in order to change their current positions....
        // 1. XNOR the current state with our new positions (i.e. inverse XOR)
        shiftValues_3[ENABLE] = getXNOR((uint8_t)shiftValues_3[DIRECTION], (uint8_t)shiftENStore_3);
      
        // 2. 'shiftValues_3[0]' now contains '0' for each motor that requires a change in position, '1' for each motor that doesn't.
      
        // 3. Always ensure to store the current position state
        shiftENStore_3 = shiftValues_3[DIRECTION];
  
        break;
      }
      case 4: // ALL_NUMBERS (YY:YY) *******************************************************
      {
//        Serial.println("UPDATING POSITIONS OF ALL_NUMBERS");        
        // Reset DIR signals to 0
        shiftValues_0[DIRECTION] = {B10000000};
        shiftValues_1[DIRECTION] = {B10000000};
        shiftValues_2[DIRECTION] = {B10000000};
        shiftValues_3[DIRECTION] = {B10000000};                        
        b = 6;

        number0Positions[0] = 666;
        number1Positions[1] = 666;
        number2Positions[2] = 666;
        number3Positions[3] = 666;                  
        // Determine which direction the motors should turn and update shiftreg[1]
        for (i=1;i<8;i++)
        {
          if (a[i] == 1)
          {
            number0Positions[i] = QUARTER_TURN_VALUE;    // Set value
            shiftValues_0[DIRECTION] |= 1UL << b; // Set direction
            
            number1Positions[i] = QUARTER_TURN_VALUE;    // Set value
            shiftValues_1[DIRECTION] |= 1UL << b; // Set direction
            
            number2Positions[i] = QUARTER_TURN_VALUE;    // Set value
            shiftValues_2[DIRECTION] |= 1UL << b; // Set direction
            
            number3Positions[i] = QUARTER_TURN_VALUE;    // Set value
            shiftValues_3[DIRECTION] |= 1UL << b; // Set direction                                    
          }
          else
          {
            number0Positions[i] = 0;           // Set value
            shiftValues_0[DIRECTION] &= ~(1UL << b);  // Set direction
            
            number1Positions[i] = 0;           // Set value
            shiftValues_1[DIRECTION] &= ~(1UL << b);  // Set direction
            
            number2Positions[i] = 0;           // Set value
            shiftValues_2[DIRECTION] &= ~(1UL << b);  // Set direction
            
            number3Positions[i] = 0;           // Set value
            shiftValues_3[DIRECTION] &= ~(1UL << b);  // Set direction                                    
          }
          b--;
        }

        // Determine which motors should be powered in order to change their current positions....
        // 1. XNOR the current state with our new positions (i.e. inverse XOR)
        shiftValues_0[ENABLE] = getXNOR((uint8_t)shiftValues_0[DIRECTION], (uint8_t)shiftENStore_0);
        shiftValues_1[ENABLE] = getXNOR((uint8_t)shiftValues_1[DIRECTION], (uint8_t)shiftENStore_1);
        shiftValues_2[ENABLE] = getXNOR((uint8_t)shiftValues_2[DIRECTION], (uint8_t)shiftENStore_2);
        shiftValues_3[ENABLE] = getXNOR((uint8_t)shiftValues_3[DIRECTION], (uint8_t)shiftENStore_3);
                              
        // 2. 'shiftValues_n[0]' now contains '0' for each motor that requires a change in position, '1' for each motor that doesn't.
      
        // 3. Always ensure to store the current position state
        shiftENStore_0 = shiftValues_0[DIRECTION];
        shiftENStore_1 = shiftValues_1[DIRECTION];
        shiftENStore_2 = shiftValues_2[DIRECTION];
        shiftENStore_3 = shiftValues_3[DIRECTION];                        

        break;
      }          
      default:
      {
        Serial.println("ERORR - SWITCH CHOICE NOT WITHIN LIMITS");        
        delay(100);
      }
    }
}

/*#####################################################################################################
# Function:     updateMotors
#
# Description:  Moves the motors of chosen group to the positions held in the global arrays
#
# Inputs:       uint8_t GROUP - which numebr group should be updated/moved? See definitions at top
#
# Retuns:       n/a 
#
#####################################################################################################*/
void updateMotors(uint8_t GROUP){
  uint8_t i;

  switch (GROUP)
    {
      case 0: // GROUP_0 (nn:nY) *******************************************************
      {
//        Serial.println("UPDATING G0 xx:xY");    

        // Send update to the ENABLE and DIRECTION shift register outputs
        MinUnit.setAll(shiftValues_0);

        // Construct correct group array from the individual arrays
        long newPositions[6];
        
        for(i = 0 ; i < 7 ; ++i )
        {
          newPositions[i] = number0Positions[i+1];  // Add positions from NUMBER_0 only
        }

        // Send new positions to steppers lib
        group0Steppers.moveTo(newPositions);
        
        // Move all motors (Blocks until all are in position)
        group0Steppers.runSpeedToPosition();
      
        // TURN OFF all motors
        shiftValues_0[ENABLE] = {B11111111};
        MinUnit.setAll(shiftValues_0);
  
        break;
      }
      case 1: // GROUP_1 (nn:YY) *******************************************************
      {
//        Serial.println("UPDATING G1 xx:YY");         

        // Send update to the ENABLE and DIRECTION shift register outputs
        MinUnit.setAll(shiftValues_0);
        MinTens.setAll(shiftValues_1);

        // Construct correct group array from the individual arrays
        long group1Positions[13]; // Combined array of first 14 desired stepper positions (number0Positions & number1Positions)

        for(i = 0 ; i < 7 ; ++i)
        {
          group1Positions[i] = number0Positions[i+1];     // Add positions from NUMBER_0
        }
        for(i = 0 ; i < 7 ; ++i)
        {
          group1Positions[i+7] = number1Positions[i+1];   // Add positions from NUMBER_1
        }            

        // Send new positions to steppers lib
        group1Steppers.moveTo(group1Positions);
        
        // Move all motors (Blocks until all are in position)
        group1Steppers.runSpeedToPosition();
      
        // TURN OFF all motors
        shiftValues_0[ENABLE] = {B11111111};
        shiftValues_1[ENABLE] = {B11111111};
        MinUnit.setAll(shiftValues_0);
        MinTens.setAll(shiftValues_1);        
  
        break;
      }   
      case 2: // GROUP_2 (nY:YY) *******************************************************
      {
//        Serial.println("UPDATING G2 xY:YY");         
               
        // Send update to the ENABLE and DIRECTION shift register outputs
        MinUnit.setAll(shiftValues_0);
        MinTens.setAll(shiftValues_1);
        HourUnit.setAll(shiftValues_2);

        // Construct correct group array from the individual arrays
        long group2Positions[20]; // Combined array of first 21 desired stepper positions (number0Positions, number1Positions & number2Positions)

        for(i = 0 ; i < 7 ; ++i)
        {
          group2Positions[i] = number0Positions[i+1];     // Add positions from NUMBER_0
        }
        for(i = 0 ; i < 7 ; ++i)
        {
          group2Positions[i+7] = number1Positions[i+1];   // Add positions from NUMBER_1
        }
        for(i = 0 ; i < 7 ; ++i)
        {
          group2Positions[i+14] = number2Positions[i+1];   // Add positions from NUMBER_2
        }                 
        
        // Send new positions to steppers lib
        group2Steppers.moveTo(group2Positions);
        
        // Move all motors (Blocks until all are in position)
        group2Steppers.runSpeedToPosition();
      
        // TURN OFF all motors
        shiftValues_0[ENABLE] = {B11111111};
        shiftValues_1[ENABLE] = {B11111111};
        shiftValues_2[ENABLE] = {B11111111};
        MinUnit.setAll(shiftValues_0);
        MinTens.setAll(shiftValues_1);
        HourUnit.setAll(shiftValues_2);         
  
        break;
      }   
      case 3: // GROUP_ALL (YY:YY) *****************************************************
      {
//        Serial.println("UPDATING G3 YY:YY");          
        
        // Send update to the ENABLE and DIRECTION shift register outputs
        MinUnit.setAll(shiftValues_0);
        MinTens.setAll(shiftValues_1);
        HourUnit.setAll(shiftValues_2);
        HourTens.setAll(shiftValues_3);

        // Construct correct group array from the individual arrays
        long allPositions[27]; // Array of ALL 28 desired stepper positions (number0Positions, number1Positions, number2Positions & number3Positions)

        for(i = 0 ; i < 7 ; ++i)
        {
          allPositions[i] = number0Positions[i+1];     // Add positions from NUMBER_0
        }
        for(i = 0 ; i < 7 ; ++i)
        {
          allPositions[i+7] = number1Positions[i+1];   // Add positions from NUMBER_1
        }
        for(i = 0 ; i < 7 ; ++i)
        {
          allPositions[i+14] = number2Positions[i+1];   // Add positions from NUMBER_2
        }
        for(i = 0 ; i < 7 ; ++i)
        {
          allPositions[i+21] = number3Positions[i+1];   // Add positions from NUMBER_3
        }

        // Send new positions to steppers lib
        allSteppers.moveTo(allPositions);
        
        // Move all motors (Blocks until all are in position)
        allSteppers.runSpeedToPosition();
      
        // TURN OFF all motors
        shiftValues_0[ENABLE] = {B11111111};
        shiftValues_1[ENABLE] = {B11111111};
        shiftValues_2[ENABLE] = {B11111111};
        shiftValues_3[ENABLE] = {B11111111};
        MinUnit.setAll(shiftValues_0);
        MinTens.setAll(shiftValues_1);
        HourUnit.setAll(shiftValues_2);
        HourTens.setAll(shiftValues_3);
                    
        break;
      }         
      default:
      {
        Serial.println("ERORR - SWITCH CHOICE NOT WITHIN LIMITS");        
        delay(100);
      }
    }
}

/*#####################################################################################################
# Function:     homeMotors
#
# Description:  Bring all arms to the bottom position (no number, should be "blank")
#               We should regard this as the default position of the arms from power up
#
# Inputs:       n/a
#
# Retuns:       n/a
#####################################################################################################*/
void homeMotors() {
  Serial.println("   HOMING");

  updatePositions(defaultHome, ALL_NUMBERS);

  updateMotors(GROUP_ALL);  

  Serial.println("         END OF HOMING");  
}

/*#####################################################################################################
# Function:     topMotors
#
# Description:  Bring all arms to the top position (effectively make the number 8)
#
# Inputs:       n/a
#
# Retuns:       n/a
#####################################################################################################*/
void topMotors() {
  Serial.println("BRINGING ALL TO TOP POSITION");

  updatePositions(defaultTop, ALL_NUMBERS);

  updateMotors(GROUP_ALL);
}

/*#####################################################################################################
# Function:     runWords
#
# Description:  Generate a random 4-letter word, splits into 4 separate letters and sends to display
#
# Inputs:       n/a
#
# Retuns:       n/a
#####################################################################################################*/
void runWords() {
  // Feedback noise for button press (here because can't be in ISR)
  piezoBuzzer.tone(3000); delay(75); // play tone
  piezoBuzzer.noTone(); // stop playing
  
  int arrayCount = (int)(sizeof(myWords)/2);  // sizeof(x) returns the size of the variable. 
                                              // "myWords" is a pointer to a String object, and thus is 2 bytes 
                                              // (as pointers on the Arduino are 2 bytes)
                                              // So we divide by 2 for total array size

  long randomValue = random(0, arrayCount);

  Serial.print("Array size: ");
  Serial.println(arrayCount);

  Serial.print("Random value: ");
  Serial.println(randomValue);

  Serial.print("Random 4-letter word: ");
  Serial.println(myWords[randomValue]);  

  // Send 1st letter to correct display (far left letter)
  char firstChar = myWords[randomValue][0];
  uint8_t firstLetter = convertLetterToNumber(firstChar);
  updatePositions(letterValues[firstLetter], NUMBER_3);

  // Send 2nd letter to correct display (far left letter)
  char secondChar = myWords[randomValue][1];  
  uint8_t secondLetter = convertLetterToNumber(secondChar);
  updatePositions(letterValues[secondLetter], NUMBER_2);  

  // Send 3rd letter to correct display (far left letter)
  char thirdChar = myWords[randomValue][2];  
  uint8_t thirdLetter = convertLetterToNumber(thirdChar);
  updatePositions(letterValues[thirdLetter], NUMBER_1);  

  // Send 4th letter to correct display (far left letter)
  char fourthChar = myWords[randomValue][3];  
  uint8_t fourthLetter = convertLetterToNumber(fourthChar);
  updatePositions(letterValues[fourthLetter], NUMBER_0);

  updateMotors(GROUP_ALL);  
  
  words_flag = 0;
}

/*#####################################################################################################
# Function:     convertLetterToNumberbers
#
# Description:  Convert a single letter to the corrosponding letterValues array value for sending to display 
#
# Inputs:       char letter - letter to be converted to number for array purposes
#
# Retuns:       uint8_t number - to be used for array when sending value to be displayed
#####################################################################################################*/
uint8_t convertLetterToNumber(char letter) {
  uint8_t number = 0;

  switch (letter)
  {
     case 'A':
     { number = 0; break;}
     case 'B':
     { number = 1; break;}
     case 'C':
     { number = 2; break;}
     case 'D':
     { number = 3; break;}
     case 'E':
     { number = 4; break;}
     case 'F':
     { number = 5; break;}
     case 'G':
     { number = 6; break;}
     case 'H':
     { number = 7; break;}
     case 'I':
     { number = 8; break;}
     case 'J':
     { number = 9; break;}
     case 'L':
     { number = 10; break;}
     case 'N':
     { number = 11; break;}
     case 'O':
     { number = 12; break;}
     case 'P':
     { number = 13; break;}
     case 'Q':
     { number = 14; break;}
     case 'R':
     { number = 15; break;}
     case 'S':
     { number = 16; break;}
     case 'T':
     { number = 17; break;}
     case 'U':
     { number = 18; break;}
     case 'Y':
     { number = 19; break;}
     default:
     {
        Serial.println("ERORR - LETTER NOT POSSIBLE");        
        delay(100);
      }
  }
  return number;
}

/*#####################################################################################################
# Function:     runNumbers
#
# Description:  Go through and display each number from 0 - 9
#
# Inputs:       n/a
#
# Retuns:       n/a
#####################################################################################################*/
void runNumbers() {
  static uint8_t j = 0;
  
  Serial.println("CYCLING NUMBERS");

  while (j < 10)
  {    
    updatePositions(numberValues[j], ALL_NUMBERS);
                
    updateMotors(GROUP_ALL);
    delay(250);
  
    j++;

    if (j == 10)
    {
     Serial.println("...FINISHED LOOP");
    }
  }
  j = 0;
}

/*#####################################################################################################
# Function:     runNumbersBackwards
#
# Description:  Go through and display each number from 9 - 0
#
# Inputs:       n/a
#
# Retuns:       n/a
#####################################################################################################*/
void runNumbersBackwards() {
  static int j = 9;

  Serial.println("CYCLING NUMBERS BACKWARDS");

  while (j >= 0)
  {
    updatePositions(numberValues[j], ALL_NUMBERS);

    updateMotors(GROUP_ALL);
    delay(100);
  
    j--;

    if (j < 0)
    {
     Serial.println("...FINISHED LOOP");
    }
  }
  j = 9;
}

/*#####################################################################################################
# Function:     runLetters
#
# Description:  Go through and display each possible letter from A to Z (exl. K, M, V, W, X, Z)  
#               See the words.h file
#
# Inputs:       n/a
#
# Retuns:       n/a
#####################################################################################################*/
void runLetters() {
  static uint8_t j = 0;

  Serial.println("CYCLING LETTERS");

  while (j < 20)
  {
    updatePositions(letterValues[j], ALL_NUMBERS);
                
    updateMotors(GROUP_ALL);
    delay(250);
  
    j++;

    if (j == 20)
    {
     Serial.println("...FINISHED LOOP");
    }
  }
  j = 0; 
}

/*#####################################################################################################
# Function:     chooseNumber
#
# Description:  Allow user to choose via terminal which specific number to display from 0 - 9
#
# Inputs:       n/a
#
# Retuns:       n/a
#####################################################################################################*/
void chooseNumber() {
  int dummyread = -2;
  
  Serial.println("CHOOSE SPECIFIC NUMBER...");

  while (Serial.available() < 1) // Wait for input
  {
    delay(100);
  }

  while (Serial.available() > 0)
  {
    dummyread = (Serial.read() - 48); // '0' char is 48

    Serial.print("dummyread: ");
    Serial.println(dummyread);

    if (dummyread > (-1))
    {
      break;
    }
  }
  serialFlush(); // Clear serial to avoid lingering inputs

  if ((dummyread >= 0) && (dummyread <= 9))
  {
    updatePositions(numberValues[dummyread], NUMBER_0);
    updateMotors(GROUP_0);
    dummyread = -2;
  }
  else
  {
    Serial.println("ERORR - CHOICE NOT WITHIN LIMITS");
    delay(100);
  }
}
  
/*#####################################################################################################
# Function:     getXNOR
#
# Description:  Return the bitwise XOR operation of two binary numbers
#
# Inputs:       uint8_t x, uint8_t y
#
# Retuns:       uint8_t - the XNOR output
#####################################################################################################*/
uint8_t getXNOR(uint8_t x, uint8_t y) {
  return ~((x | y) & (~x | ~y));
}

/*#####################################################################################################
# Function:     displayNumber
#
# Description:  Displays specific number to display from 0 - 9
#
# Inputs:       uint8_t num (the number to be displayed)
#
# Retuns:       n/a
#####################################################################################################*/
void displayNumber(uint8_t num) {
  if ((num >= 0) && (num <= 9))
  {
    updatePositions(numberValues[num], NUMBER_0);
    updateMotors(GROUP_0);
  }
  else
  {
    Serial.println("ERORR - CHOICE NOT WITHIN LIMITS");
    delay(100);
  }
}

/*#####################################################################################################
# Function:     printCurrentTime
#
# Description:  Read from our DS3231 RTC module and print the current values
#
# Inputs:       n/a
#
# Retuns:       n/a
#####################################################################################################*/
void printCurrentTime(){
  Serial.println("Here is the current time...");
  
  Serial.print(rtcClock.getDate(), DEC);
  Serial.print("-");
  Serial.print(rtcClock.getMonth(h12Flag), DEC);
  Serial.print("-");    
  Serial.print(rtcClock.getYear(), DEC);
  Serial.print(" ");

  Serial.print(rtcClock.getHour(h12Flag, pmFlag), DEC); //24-hr
  Serial.print(":");
  Serial.print(rtcClock.getMinute(), DEC);
  Serial.print(":");
  Serial.println(rtcClock.getSecond(), DEC);

  delay(100);
}

/*#####################################################################################################
# Function:     displayTime
#
# Description:  Check and send latest time to the display
#
# Inputs:       n/a
#
# Retuns:       n/a
#####################################################################################################*/
void displayTime(){
  static uint8_t hoursStored = 0;
  static uint8_t minutesStored= 0;

  uint8_t hours = 0;
  uint8_t hourUnits = 0;
  uint8_t hourTens = 0; 
      
  uint8_t minutes = 0;
  uint8_t minutesUnits = 0;
  uint8_t minutesTens = 0;   

  hours = rtcClock.getHour(h12Flag, pmFlag);
  minutes = rtcClock.getMinute();

  if (minutes == minutesStored) // No change since last call
  {
    delay(500); // Do nothing
  }
  else
  {
    delay(100);

    // Get minute UNITS **************************************
    minutesUnits = getLastDigit(minutes);
     
    // Get minute TENS ***************************************
    if (minutes >= 10)
    {
      minutesTens = minutes;
      while (minutesTens >= 10)
        minutesTens /= 10;  
    }
    else
    {
      minutesTens = 0;
    }

    // Get hour UNITS ************************************
    hourUnits = getLastDigit(hours);
  
    // Get hour TENS *************************************
    if (hours >= 10)
    {
      hourTens = hours;
      while (hourTens >= 10)
        hourTens /= 10;     
    }
    else
    {
      hourTens = 0;
    }

    // Send updates to the display
    updatePositions(numberValues[minutesUnits], NUMBER_0);
    updatePositions(numberValues[minutesTens], NUMBER_1);
    updatePositions(numberValues[hourUnits], NUMBER_2);
    updatePositions(numberValues[hourTens], NUMBER_3);       

    updateMotors(GROUP_ALL);      

    // Update the stored values
    hoursStored = hours; 
    minutesStored = minutes;
  }
}

  uint8_t storedSecond;
  uint8_t currentSecond = 0;
  uint16_t totalElapsedSeconds = 0;
  uint16_t liveSeconds = 0;
    
  uint8_t secsUnits = 0;
  uint8_t secsTens = 0;
  uint8_t minsUnits = 0;
  uint8_t minsTens = 0;  

  uint16_t totalTimerSeconds = 0;
  uint16_t SECONDS = 60;

/*#####################################################################################################
# Function:     runTimerCountdown
#
# Description:  Run a countdown from n number of seconds, change display every second, then play alert
#
# Inputs:       uint8_t minutes - Total number of minutes input from user button to be used as timer
#
# Retuns:       n/a
#
#####################################################################################################*/
void runTimerCountdown(){
  storedSecond = 0;
  currentSecond = 0;
  totalElapsedSeconds = 0;
  liveSeconds = 0;
      
  secsUnits = 0;
  secsTens = 0;
  minsUnits = 0;
  minsTens = 0;  
  
  totalTimerSeconds = 0;
  SECONDS = 60;

  Serial.println("RUNNING A COUNTDOWN...");

  // Avoiding 0 seconds as a starter... it just gets messy
  while(rtcClock.getSecond() == 0){ delay(100); }
  
  storedSecond = rtcClock.getSecond();

  totalTimerSeconds = timerMinutes * 60;
  totalElapsedSeconds = storedSecond; // We may start on random number, so store it for subtraction
  secondCounter = storedSecond;       // Also use it as starting point for our compare value
  liveSeconds = 60;

  // Create simple second counter from RTC
  while ((totalElapsedSeconds-storedSecond) <= totalTimerSeconds)  // Set simple timer for 2 minutes
  {
    time_left = totalTimerSeconds-(totalElapsedSeconds-storedSecond);   // update timer
    liveSeconds = SECONDS-(totalElapsedSeconds-storedSecond);

    if (liveSeconds == 0)
    {
      liveSeconds = 60;
      SECONDS = 60 + (totalElapsedSeconds-storedSecond);
    }

    // Extract the different digits...    
    if ((time_left % 60) == 0)
    {
      secsUnits = 0;
      secsTens = 0;
    }
    else
    {
      // Seconds_Units
      secsUnits = liveSeconds % 10;
      // Seconds_Tens
      if (liveSeconds >= 10) secsTens = (liveSeconds / (10)) % 10;
      else secsTens = 0;
    }

    // Minutes_Units
    if (time_left >= 60)
    {
      Serial.println("YES TIME IS MORE THAN 60 ");
      minsUnits = uint8_t(floor(time_left/60)) % 10;

      Serial.print("minsUnits(*** 1 ***): ");
      Serial.println(minsUnits); 

      while (minsUnits >= 10)
      {
        Serial.print("minsUnits(*** 1 ***): ");
        Serial.println(minsUnits);

        minsUnits -= 10;

        Serial.print("FIXED? minsUnits: ");
        Serial.println(minsUnits); 
      } // ensuring only 1 digit is stored
    }
    else {minsUnits = 0;}
    
    
    // Minutes_Tens
    if (time_left >= 600){minsTens = uint8_t(floor(time_left/600)) % 10;}
    else {minsTens = 0;}
    if (minsTens >= 10){minsTens %= 10;} // ensuring only 1 digit is stored        

    currentSecond = (rtcClock.getSecond());

    if (1)    // *** FOR DEBUG ONLY ***
    {
      Serial.print("Real seconds: ");
      Serial.println(rtcClock.getSecond(), DEC);
  
      Serial.print("secondCounter: ");
      Serial.println(secondCounter, DEC);   
  
      Serial.print("currentSecond: ");
      Serial.println(currentSecond, DEC); 

      Serial.print("liveSeconds: ");
      Serial.println(liveSeconds, DEC);    

      Serial.print("time_left: ");
      Serial.println(time_left, DEC);        

      Serial.print("secsUnits: ");
      Serial.println(secsUnits, DEC); 
      Serial.print("secsTens: ");
      Serial.println(secsTens, DEC);
      Serial.print("minsUnits: ");
      Serial.println(minsUnits, DEC); 
      Serial.print("minsTens: ");
      Serial.println(minsTens, DEC);                   
    } 

    // Check if the RTC seconds have looped back
//    if((rtcClock.getSecond()==0) && (secondCounter > 0))
    if(secondCounter > currentSecond)
    {
      secondCounter = 0; 
      totalElapsedSeconds++;      
    }
    // Otherwise, just check for an increase in seconds and update counters
    else
    {
      if (currentSecond > secondCounter) 
      {
        secondCounter++;
        totalElapsedSeconds++;
      }
    }

    if (1)    // *** FOR DEBUG ONLY ***
    {
      Serial.println("Sec     Overflow");
      Serial.print(totalElapsedSeconds-storedSecond);Serial.print("       ");
    }
  
    delay(100);

    // Update numbers on display with new timer value
    updatePositions(numberValues[secsUnits], NUMBER_0);
    updatePositions(numberValues[secsTens], NUMBER_1);   
    updatePositions(numberValues[minsUnits], NUMBER_2);
    updatePositions(numberValues[minsTens], NUMBER_3);        

    updateMotors(GROUP_ALL);    // TEMP

  }

  Serial.println("ALARM HIT!!!");
  runAlert();
  delay(1000);
  runAlert();
  delay(1000);
  runAlert();
  
  //timeLoop_flag = 1; // Ensure to turn time mode back ON (default)

  timerMinutes = 0;
  
  storedSecond = 0;
  currentSecond = 0;
  totalElapsedSeconds = 0;
  liveSeconds = 0;
    
  secsUnits = 0;
  secsTens = 0;
  minsUnits = 0;
  minsTens = 0;
     
  totalTimerSeconds = 0;
  SECONDS = 60;

  delay(100);
}

/*#####################################################################################################
# Function:     serialFlush
#
# Description:  Simple routine for clearing active serial data
#
# Inputs:       n/a
#
# Retuns:       n/a
#####################################################################################################*/
void serialFlush(){
  while(Serial.available() > 0) {
    char t = Serial.read();
  }
}

/*#####################################################################################################
# Function:     getFirstDigit
#
# Inputs:       uint8_t number
#
# Retuns:       uint8_t number (returns same modified variable)
#####################################################################################################*/
uint8_t getFirstDigit(uint8_t number){
//    while (number >= 10)
//    {
                number = (number - (number % 10)) / 10;
//    }

    return number;
}

/*#####################################################################################################
# Function:     getLastDigit
#
# Inputs:       uint8_t number
#
# Retuns:       uint8_t number (returns same modified variable)
#####################################################################################################*/
uint8_t getLastDigit(uint8_t number){
    return (number % 10);           // WHY DID THIS STOP WORKING PREVIOUSLY? Fixed by adding math.h ???
//    if (number >= 10)
//    {
//      number = number - (number/10) * 10;
//    }
//    else
//    {
//      number = number;
//    }
}

/*#####################################################################################################
# Function:     blackOneButtonISR
#
# Inputs:       n/a
#
# Retuns:       n/a
#####################################################################################################*/
void blackOneButtonISR(){
  long debouncing_time = 100; //Debouncing Time in Milliseconds
  static unsigned long last_micros = 0;
  
  if((long)(micros() - last_micros) >= debouncing_time * 1000) 
  {
    set_time_buttons_flag = 1;
    last_micros = micros();
  }
}

/*#####################################################################################################
# Function:     blackTwoButtonISR
#
# Inputs:       n/a
#
# Retuns:       n/a
#####################################################################################################*/
void blackTwoButtonISR(){
  long debouncing_time = 100; //Debouncing Time in Milliseconds
  static unsigned long last_micros = 0;
  
  if((long)(micros() - last_micros) >= debouncing_time * 1000)
  {
    set_TIMER_buttons_flag = 1;
    last_micros = micros();
  }
}

/*#####################################################################################################
# Function:     redButtonISR
#
# Inputs:       n/a
#
# Retuns:       n/a
#####################################################################################################*/
void redButtonISR(){
  long debouncing_time = 100; //Debouncing Time in Milliseconds
  static unsigned long last_micros = 0;
  
  if((long)(micros() - last_micros) >= debouncing_time * 1000) 
  {
    words_flag = 1;
    last_micros = micros();
  }
}

/*#####################################################################################################
# Function:     getTimeSetButtonInputs
#
# Description:  Uses two buttons. WHITE_1 incremenets, BLACK_1 sets the hrs, then the mins
#
# Inputs:       n/a
#
# Retuns:       n/a
#####################################################################################################*/
void getTimeSetButtonInputs(){
  dettachAllInterrupts();

  uint8_t whitePresses_0 = 0, whitePresses_1 = 0;

  piezoBuzzer.tone(3000); delay(75); // play tone
  piezoBuzzer.noTone(); // stop playing
  
  Serial.println("ENTER HOURS");
  delay(500); // Note: delay() relies on interrupts, hence we detach our interrupts inside our ISRs
              //       ESPECIALLY in this case, where I'm using same interrupt button as a non-interrupt button (lack of IO)
  
  while(digitalRead(BLACK_BUTTON_1) == HIGH)  // first loop until hrs confirmed
  {
    if(digitalRead(WHITE_BUTTON_1) == LOW)
    {
      piezoBuzzer.tone(2000); delay(75); // play tone
      piezoBuzzer.noTone(); // stop playing

      whitePresses_0++;
      delay(100);
    }
  }
  piezoBuzzer.tone(3000); delay(75); // play tone
  piezoBuzzer.noTone(); // stop playing

  Serial.println("ENTER MINUTES");
  delay(500);
  
  while(digitalRead(BLACK_BUTTON_1) == HIGH)  // second loop until mins confirmed 
  {
    if(digitalRead(WHITE_BUTTON_1) == LOW)
    {
      piezoBuzzer.tone(2000); delay(75); // play tone
      piezoBuzzer.noTone(); // stop playing
              
      whitePresses_1++;
      delay(100);        
    }
  } 
  piezoBuzzer.tone(3000); delay(75); // play tone
  piezoBuzzer.noTone(); // stop playing
  
  Serial.println("Hrs     Mins");
  Serial.print(whitePresses_0);Serial.print("       ");Serial.println(whitePresses_1);
  delay(500);

  set_time_buttons_flag = 0;

  // Update the RTC registers with the new settings
  rtcClock.setHour(whitePresses_0);
  rtcClock.setMinute(whitePresses_1);
  rtcClock.setSecond(00);

  REattachAllInterrupts();
}

/*#####################################################################################################
# Function:     dettachAllInterrupts
#
# Inputs:       n/a
#
# Retuns:       n/a
#####################################################################################################*/
void dettachAllInterrupts(){
  detachInterrupt(digitalPinToInterrupt(BLACK_BUTTON_1));
  detachInterrupt(digitalPinToInterrupt(BLACK_BUTTON_2));
  detachInterrupt(digitalPinToInterrupt(RED_BUTTON));  
}

/*#####################################################################################################
# Function:     REattachAllInterrupts
#
# Inputs:       n/a
#
# Retuns:       n/a
#####################################################################################################*/
void REattachAllInterrupts(){
  attachInterrupt(digitalPinToInterrupt(BLACK_BUTTON_1), blackOneButtonISR, LOW);
  attachInterrupt(digitalPinToInterrupt(BLACK_BUTTON_2), blackTwoButtonISR, LOW);  
  attachInterrupt(digitalPinToInterrupt(RED_BUTTON), redButtonISR, LOW); 
}

/*#####################################################################################################
# Function:     getTimerButtonInputs
#
# Description:  Uses two buttons. WHITE_1 incremenets, BLACK_1 sets the mins
#
# Inputs:       n/a
#
# Retuns:       n/a
#####################################################################################################*/
void getTimerButtonInputs(){
  dettachAllInterrupts();

  timeLoop_flag = 0; // Ensure to turn time mode OFF

  uint8_t whitePresses_0 = 0;

  piezoBuzzer.tone(3000); delay(75); // play tone
  piezoBuzzer.noTone(); // stop playing

  Serial.println("ENTER MINUTES");
  delay(500); // Note: delay() relies on interrupts, hence we detach our interrupts inside our ISRs
              //       ESPECIALLY in this case, where I'm using same interrupt button as a non-interrupt button (lack of IO)
  
  while(digitalRead(BLACK_BUTTON_2) == HIGH)  // loop until minutes confirmed
  {
    if(digitalRead(WHITE_BUTTON_1) == LOW)
    {
      piezoBuzzer.tone(2000); delay(75); // play tone
      piezoBuzzer.noTone(); // stop playing

      whitePresses_0++;
      delay(100);
    }
  }
  piezoBuzzer.tone(3000); delay(75); // play tone
  piezoBuzzer.noTone(); // stop playing
  
  Serial.print("Minutes: ");
  Serial.println(whitePresses_0);
  delay(500);

  // Update global timer minutes request with new input
  timerMinutes = whitePresses_0;
  // Run timer function once we leave here
  countdown_flag = 1;

  set_TIMER_buttons_flag = 0;

  REattachAllInterrupts();
}

/*#####################################################################################################
# Function:     runAlert
#
# Description:  Takes an array of notes and durations, playing them together as a melody
#
# Inputs:       n/a - But note the global 'melody' array at the top 
#
# Retuns:       n/a
#####################################################################################################*/
void runAlert() {
  Serial.println("PLAYING ALERT! ... ");

  // change this to make the song slower or faster
  int tempo = 150;

  // there are two parts of each note (pitch and duration), so for each note there are four bytes
  int notes = 0;
  
  // this calculates the duration of a whole note in ms
  int wholenote = 0;
  
  int divider = 0, noteDuration = 0; 

  wholenote = (60000 * 4) / tempo;
  notes = sizeof(melody) / sizeof(melody[0]) / 2;
 
  // Iterate over the notes of the melody
  // The array is twice the number of notes (i.e. notes + durations)
  for (int thisNote = 0; thisNote < notes * 2; thisNote = thisNote + 2) 
  {
    // calculates the duration of each note
    divider = melody[thisNote + 1];
    if (divider > 0)
    {
      // regular note, just proceed
      noteDuration = (wholenote) / divider;
    } 
    else if (divider < 0) 
    {
      // dotted notes are represented with negative durations!!
      noteDuration = (wholenote) / abs(divider);
      noteDuration *= 1.5; // increases the duration in half for dotted notes
    }
   
    piezoBuzzer.tone (melody[thisNote]); // play tone
    delay (noteDuration);

    piezoBuzzer.noTone (); // stop playing
  }
  alert_flag = 0;
}

/*#####################################################################################################
#####################################        SETUP        ############################################
#####################################################################################################*/
void setup() {

  if (DEBUG_MODE)
  {
    Serial.begin(9600);
    delay(100);
  //  while(!Serial);
    Serial.println("......Starting Application......");
  }

  // Start the I2C interface for our RTC module
  Wire.begin();

  //TEMP / TODO: Program the date & time
  rtcClock.setClockMode(false);  // set to 24h 
  rtcClock.setYear(22);
  rtcClock.setMonth(01);
  rtcClock.setDate(31);
  rtcClock.setDoW(0);
  rtcClock.setHour(19);
  rtcClock.setMinute(38);
  rtcClock.setSecond(00);

  // if analog input pin 0 is unconnected, random analog
  // noise will cause the call to randomSeed() to generate
  // different seed numbers each time the sketch runs.
  // randomSeed() will then shuffle the random function.
  randomSeed(analogRead(0));

  // Configure signal pin to the piezo buzzer
  pinMode (BUZZER_PIN, OUTPUT);  // Note that only certain pins can be used
  digitalWrite(BUZZER_PIN, LOW);

//-------- Configure the UART setup --------------------
  SERIAL_PORT.begin(57600);      // HW UART drivers

  //UART TX is sent to all connected chips, so this setup applies to all
  driver.begin();               // Initiate pins and registeries
  driver.toff(5);               // Enables driver in software
  driver.rms_current(175);      // Set stepper current to 175mA
  driver.pwm_autoscale(1);
  driver.microsteps(8);         // 8 seems to be lowest we can go before the motors struggle to move the arms!

  driver.en_spreadCycle(false); // Enable spreadCycle (false = ON / silence)

  SERIAL_PORT.end();  // Turn UART connection OFF. Control will now done by the STEP pin for each motor

//-------------- Configure each stepper driver --------------------
  stepper0.setMaxSpeed(50*steps_per_mm); // 100mm/s @ 80 steps/mm
  stepper0.setAcceleration(1000*steps_per_mm); // 2000mm/s^2
  stepper0.setPinsInverted(false, false, true);
  stepper0.enableOutputs();
  stepper0.setCurrentPosition(0);

  stepper1.setMaxSpeed(50*steps_per_mm); // 100mm/s @ 80 steps/mm
  stepper1.setAcceleration(1000*steps_per_mm); // 2000mm/s^2
  stepper1.setPinsInverted(false, false, true);
  stepper1.enableOutputs();
  stepper1.setCurrentPosition(0);

  stepper2.setMaxSpeed(50*steps_per_mm); // 100mm/s @ 80 steps/mm
  stepper2.setAcceleration(1000*steps_per_mm); // 2000mm/s^2
  stepper2.setPinsInverted(false, false, true);
  stepper2.enableOutputs();
  stepper2.setCurrentPosition(0);

  stepper3.setMaxSpeed(50*steps_per_mm); // 100mm/s @ 80 steps/mm
  stepper3.setAcceleration(1000*steps_per_mm); // 2000mm/s^2
  stepper3.setPinsInverted(false, false, true);
  stepper3.enableOutputs();
  stepper3.setCurrentPosition(0);

  stepper4.setMaxSpeed(50*steps_per_mm); // 100mm/s @ 80 steps/mm
  stepper4.setAcceleration(1000*steps_per_mm); // 2000mm/s^2
  stepper4.setPinsInverted(false, false, true);
  stepper4.enableOutputs();
  stepper4.setCurrentPosition(0);

  stepper5.setMaxSpeed(50*steps_per_mm); // 100mm/s @ 80 steps/mm
  stepper5.setAcceleration(1000*steps_per_mm); // 2000mm/s^2
  stepper5.setPinsInverted(false, false, true);
  stepper5.enableOutputs();
  stepper5.setCurrentPosition(0);

  stepper6.setMaxSpeed(50*steps_per_mm); // 100mm/s @ 80 steps/mm
  stepper6.setAcceleration(1000*steps_per_mm); // 2000mm/s^2
  stepper6.setPinsInverted(false, false, true);
  stepper6.enableOutputs();
  stepper6.setCurrentPosition(0);  

  //**************************************************************

  stepper7.setMaxSpeed(50*steps_per_mm); // 100mm/s @ 80 steps/mm
  stepper7.setAcceleration(1000*steps_per_mm); // 2000mm/s^2
  stepper7.setPinsInverted(false, false, true);
  stepper7.enableOutputs();
  stepper7.setCurrentPosition(0);

  stepper8.setMaxSpeed(50*steps_per_mm); // 100mm/s @ 80 steps/mm
  stepper8.setAcceleration(1000*steps_per_mm); // 2000mm/s^2
  stepper8.setPinsInverted(false, false, true);
  stepper8.enableOutputs();
  stepper8.setCurrentPosition(0);

  stepper9.setMaxSpeed(50*steps_per_mm); // 100mm/s @ 80 steps/mm
  stepper9.setAcceleration(1000*steps_per_mm); // 2000mm/s^2
  stepper9.setPinsInverted(false, false, true);
  stepper9.enableOutputs();
  stepper9.setCurrentPosition(0);

  stepper10.setMaxSpeed(50*steps_per_mm); // 100mm/s @ 80 steps/mm
  stepper10.setAcceleration(1000*steps_per_mm); // 2000mm/s^2
  stepper10.setPinsInverted(false, false, true);
  stepper10.enableOutputs();
  stepper10.setCurrentPosition(0);

  stepper11.setMaxSpeed(50*steps_per_mm); // 100mm/s @ 80 steps/mm
  stepper11.setAcceleration(1000*steps_per_mm); // 2000mm/s^2
  stepper11.setPinsInverted(false, false, true);
  stepper11.enableOutputs();
  stepper11.setCurrentPosition(0);

  stepper12.setMaxSpeed(50*steps_per_mm); // 100mm/s @ 80 steps/mm
  stepper12.setAcceleration(1000*steps_per_mm); // 2000mm/s^2
  stepper12.setPinsInverted(false, false, true);
  stepper12.enableOutputs();
  stepper12.setCurrentPosition(0);

  stepper13.setMaxSpeed(50*steps_per_mm); // 100mm/s @ 80 steps/mm
  stepper13.setAcceleration(1000*steps_per_mm); // 2000mm/s^2
  stepper13.setPinsInverted(false, false, true);
  stepper13.enableOutputs();
  stepper13.setCurrentPosition(0);  

  //**************************************************************   

  stepper14.setMaxSpeed(50*steps_per_mm); // 100mm/s @ 80 steps/mm
  stepper14.setAcceleration(1000*steps_per_mm); // 2000mm/s^2
  stepper14.setPinsInverted(false, false, true);
  stepper14.enableOutputs();
  stepper14.setCurrentPosition(0);

  stepper15.setMaxSpeed(50*steps_per_mm); // 100mm/s @ 80 steps/mm
  stepper15.setAcceleration(1000*steps_per_mm); // 2000mm/s^2
  stepper15.setPinsInverted(false, false, true);
  stepper15.enableOutputs();
  stepper15.setCurrentPosition(0);

  stepper16.setMaxSpeed(50*steps_per_mm); // 100mm/s @ 80 steps/mm
  stepper16.setAcceleration(1000*steps_per_mm); // 2000mm/s^2
  stepper16.setPinsInverted(false, false, true);
  stepper16.enableOutputs();
  stepper16.setCurrentPosition(0);

  stepper17.setMaxSpeed(50*steps_per_mm); // 100mm/s @ 80 steps/mm
  stepper17.setAcceleration(1000*steps_per_mm); // 2000mm/s^2
  stepper17.setPinsInverted(false, false, true);
  stepper17.enableOutputs();
  stepper17.setCurrentPosition(0);

  stepper18.setMaxSpeed(50*steps_per_mm); // 100mm/s @ 80 steps/mm
  stepper18.setAcceleration(1000*steps_per_mm); // 2000mm/s^2
  stepper18.setPinsInverted(false, false, true);
  stepper18.enableOutputs();
  stepper18.setCurrentPosition(0);

  stepper19.setMaxSpeed(50*steps_per_mm); // 100mm/s @ 80 steps/mm
  stepper19.setAcceleration(1000*steps_per_mm); // 2000mm/s^2
  stepper19.setPinsInverted(false, false, true);
  stepper19.enableOutputs();
  stepper19.setCurrentPosition(0);

  stepper20.setMaxSpeed(50*steps_per_mm); // 100mm/s @ 80 steps/mm
  stepper20.setAcceleration(1000*steps_per_mm); // 2000mm/s^2
  stepper20.setPinsInverted(false, false, true);
  stepper20.enableOutputs();
  stepper20.setCurrentPosition(0);

  //**************************************************************   

  stepper21.setMaxSpeed(50*steps_per_mm); // 100mm/s @ 80 steps/mm
  stepper21.setAcceleration(1000*steps_per_mm); // 2000mm/s^2
  stepper21.setPinsInverted(false, false, true);
  stepper21.enableOutputs();
  stepper21.setCurrentPosition(0);

  stepper22.setMaxSpeed(50*steps_per_mm); // 100mm/s @ 80 steps/mm
  stepper22.setAcceleration(1000*steps_per_mm); // 2000mm/s^2
  stepper22.setPinsInverted(false, false, true);
  stepper22.enableOutputs();
  stepper22.setCurrentPosition(0);

  stepper23.setMaxSpeed(50*steps_per_mm); // 100mm/s @ 80 steps/mm
  stepper23.setAcceleration(1000*steps_per_mm); // 2000mm/s^2
  stepper23.setPinsInverted(false, false, true);
  stepper23.enableOutputs();
  stepper23.setCurrentPosition(0);

  stepper24.setMaxSpeed(50*steps_per_mm); // 100mm/s @ 80 steps/mm
  stepper24.setAcceleration(1000*steps_per_mm); // 2000mm/s^2
  stepper24.setPinsInverted(false, false, true);
  stepper24.enableOutputs();
  stepper24.setCurrentPosition(0);

  stepper25.setMaxSpeed(50*steps_per_mm); // 100mm/s @ 80 steps/mm
  stepper25.setAcceleration(1000*steps_per_mm); // 2000mm/s^2
  stepper25.setPinsInverted(false, false, true);
  stepper25.enableOutputs();
  stepper25.setCurrentPosition(0);

  stepper26.setMaxSpeed(50*steps_per_mm); // 100mm/s @ 80 steps/mm
  stepper26.setAcceleration(1000*steps_per_mm); // 2000mm/s^2
  stepper26.setPinsInverted(false, false, true);
  stepper26.enableOutputs();
  stepper26.setCurrentPosition(0);

  stepper27.setMaxSpeed(50*steps_per_mm); // 100mm/s @ 80 steps/mm
  stepper27.setAcceleration(1000*steps_per_mm); // 2000mm/s^2
  stepper27.setPinsInverted(false, false, true);
  stepper27.enableOutputs();
  stepper27.setCurrentPosition(0); 

  // Now pass them to MultiStepper to manage
  // Combine all steppers required to move FIRST number (NUMBER_0 only)
  group0Steppers.addStepper(stepper0);      
  group0Steppers.addStepper(stepper1);
  group0Steppers.addStepper(stepper2);
  group0Steppers.addStepper(stepper3);
  group0Steppers.addStepper(stepper4);
  group0Steppers.addStepper(stepper5);
  group0Steppers.addStepper(stepper6);

  // Combine all steppers required to move FIRST GROUP (NUMBER_0 and NUMBER_1)
  group1Steppers.addStepper(stepper0);      
  group1Steppers.addStepper(stepper1);
  group1Steppers.addStepper(stepper2);
  group1Steppers.addStepper(stepper3);
  group1Steppers.addStepper(stepper4);
  group1Steppers.addStepper(stepper5);
  group1Steppers.addStepper(stepper6); 
  group1Steppers.addStepper(stepper7);  
  group1Steppers.addStepper(stepper8);
  group1Steppers.addStepper(stepper9);
  group1Steppers.addStepper(stepper10);
  group1Steppers.addStepper(stepper11);
  group1Steppers.addStepper(stepper12);
  group1Steppers.addStepper(stepper13);

  // Combine all steppers required to move SECOND GROUP (NUMBER_0, NUMBER_1 and NUMBER_2)
  group2Steppers.addStepper(stepper0);      
  group2Steppers.addStepper(stepper1);
  group2Steppers.addStepper(stepper2);
  group2Steppers.addStepper(stepper3);
  group2Steppers.addStepper(stepper4);
  group2Steppers.addStepper(stepper5);
  group2Steppers.addStepper(stepper6); 
  group2Steppers.addStepper(stepper7);  
  group2Steppers.addStepper(stepper8);
  group2Steppers.addStepper(stepper9);
  group2Steppers.addStepper(stepper10);
  group2Steppers.addStepper(stepper11);
  group2Steppers.addStepper(stepper12);
  group2Steppers.addStepper(stepper13);
  group2Steppers.addStepper(stepper14);
  group2Steppers.addStepper(stepper15);
  group2Steppers.addStepper(stepper16);
  group2Steppers.addStepper(stepper17);
  group2Steppers.addStepper(stepper18);
  group2Steppers.addStepper(stepper19);
  group2Steppers.addStepper(stepper20);

  // Combine all steppers required to move THIRD GROUP (ALL NUMBERS)
  allSteppers.addStepper(stepper0);      
  allSteppers.addStepper(stepper1);
  allSteppers.addStepper(stepper2);
  allSteppers.addStepper(stepper3);
  allSteppers.addStepper(stepper4);
  allSteppers.addStepper(stepper5);
  allSteppers.addStepper(stepper6);
  allSteppers.addStepper(stepper7);
  allSteppers.addStepper(stepper8);
  allSteppers.addStepper(stepper9);
  allSteppers.addStepper(stepper10);
  allSteppers.addStepper(stepper11);
  allSteppers.addStepper(stepper12);
  allSteppers.addStepper(stepper13);
  allSteppers.addStepper(stepper14);
  allSteppers.addStepper(stepper15);
  allSteppers.addStepper(stepper16);
  allSteppers.addStepper(stepper17);
  allSteppers.addStepper(stepper18);
  allSteppers.addStepper(stepper19);
  allSteppers.addStepper(stepper20);
  allSteppers.addStepper(stepper21);
  allSteppers.addStepper(stepper22);
  allSteppers.addStepper(stepper23);
  allSteppers.addStepper(stepper24);
  allSteppers.addStepper(stepper25);
  allSteppers.addStepper(stepper26);
  allSteppers.addStepper(stepper27);  

  // Configure the buttons
  pinMode(BLACK_BUTTON_1, INPUT_PULLUP);
  pinMode(BLACK_BUTTON_2, INPUT_PULLUP);
  pinMode(WHITE_BUTTON_1, INPUT_PULLUP);
  pinMode(WHITE_BUTTON_2, INPUT_PULLUP);
  pinMode(RED_BUTTON, INPUT_PULLUP);

  // Attach the necessary pin interrupts
  REattachAllInterrupts();
}

/*#####################################################################################################
###################################        MAIN LOOP        ##########################################
#####################################################################################################*/
void loop() {
  uint8_t i = 0;
  uint8_t b = 0;
  static uint8_t once = 0;
  
  // 74HC595N: CHANGED LIBRARY TO LSBFIRST, therefore;
  // (Q0,    Q1,   Q2,   Q3,   Q4,   Q5,   Q6,   Q7)   (Q0,    Q1,    Q2,    Q3,    Q4,    Q5,    Q6,    Q7)
  // (SPARE, EN_0, EN_1, EN_2, EN_3, EN_4, EN_5, EN_7) (SPARE, DIR_0, DIR_1, DIR_2, DIR_3, DIR_4, DIR_5, DIR_6)  
  
  // Remember that for EN_x; 1 = OFF, 0 = ON

  if (once == 0)
  {
    long positions[27]; // Array of 7 desired stepper positions
    Serial.println("RUNNING INITIAL SET TO 0");
      // Set all pins at once - TURN ON
    shiftValues_0[0] = {B10000000};
    shiftValues_0[1] = {B01111111}; 
    MinUnit.setAll(shiftValues_0);
    MinTens.setAll(shiftValues_0);
    HourUnit.setAll(shiftValues_0);
    HourTens.setAll(shiftValues_0);        
  
    for (i=0;i<28;i++)
    {
      positions[i] = 0;
    }
      
    allSteppers.moveTo(positions);
    allSteppers.runSpeedToPosition(); // Blocks until all are in position
  
    // Set all pins at once - TURN OFF (saving power)
    shiftValues_0[0] = {B11111111}; // Note: don't need to update shiftValues_0[1], DIR doesn't matter
    MinUnit.setAll(shiftValues_0);
    MinTens.setAll(shiftValues_0);
    HourUnit.setAll(shiftValues_0);
    HourTens.setAll(shiftValues_0);         
    
    delay(500);
  
    once++;
  }

  /////////////////////////////////////////////////////////////////////////
  // CHOICES BELOW HERE ///////////////////////////////////////////////////
  /////////////////////////////////////////////////////////////////////////
  
  if (home_flag) //h
  {
    homeMotors();
    home_flag = 0;
  }

  if (top_flag) //u
  {
    topMotors();
    top_flag = 0;    
  }  

  if (numbers_flag) //n
  {
    runNumbers();
    numbers_flag = 0;    
  } 

  if (numbersBackwards_flag) //b
  {
    runNumbersBackwards();
    numbersBackwards_flag = 0;    
  }    

  if (letters_flag) //l
  {
    runLetters();
    letters_flag = 0;    
  }

  if (choose_number_flag) //c
  {
    chooseNumber();
    choose_number_flag = 0;    
  }  

  if (alert_flag) //a
  {
    runAlert();
    alert_flag = 0;    
  } 

  if (countdown_flag) //d
  {
    runTimerCountdown();
    timerMinutes = 0;
    countdown_flag = 0;    
  } 

  if (words_flag) //a
  {
    runWords();
    words_flag = 0;    
  }   

  if (set_time_buttons_flag) //set by ISR
  {
    getTimeSetButtonInputs();
    set_time_buttons_flag = 0;    
  }

  if (set_TIMER_buttons_flag) //set by ISR
  {
    getTimerButtonInputs();
    set_TIMER_buttons_flag = 0;    
  }      

  if (loop_flag) //f
  {
    runNumbers();
  }

  if (timeLoop_flag)//z
  {
//    Serial.println("RUNNING displayTime()");  
    displayTime();
  }

  delay(100);
}

void serialEvent() {
  long positions[13]; // Array of 7 desired stepper positions
  uint8_t i = 0;
  while (Serial.available()) {
    char inChar = (char)Serial.read();
    serialFlush();
    switch (inChar)
    {
      case 'h':
      {
        home_flag = 1;
        break;
      }
      case 'u':
      {
        top_flag = 1;
        break;
      } 
      case 'c':
      {
        choose_number_flag = 1;
        break;
      }                        
      case 'n':
      {
        numbers_flag = 1;
        break;
      }    
      case 'b':
      {
        numbersBackwards_flag = 1;
        break;
      }         
      case 'l':
      {
        letters_flag = 1;
        break;
      }      
      case 'w':
      {
        words_flag = 1;       
        break;
      }   
      case 'f':
      {
        loop_flag = !loop_flag; 
        Serial.print("loop_flag value changed to: ");
        Serial.println(loop_flag);        
        break;
      }   
      case 't':
      {
        printCurrentTime();      
        break;
      }
      case 'z':
      {
        timeLoop_flag = !timeLoop_flag; 
        Serial.print("timeLoop_flag value changed to: ");
        Serial.println(timeLoop_flag);        
        break;
      }
      case 'a':
      {
        alert_flag = 1;
        break;
      }     
      case 'd':
      {
        countdown_flag = 1;
        break;
      }
      default:
      {
        Serial.println("ERORR - SWITCH CHOICE NOT WITHIN LIMITS");        
        delay(100);
      }
    }
    serialFlush();
  }
}
