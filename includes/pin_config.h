/**************************************************************************
 * FILE NAME: pin_config.h                                                *
 * DESCRIPTION: Contains all #defines for the pin config of Mega2560      *
 *                                                                        *
 * VERSION: 1 (02/03/2022)                                                *
 *************************************************************************/

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
