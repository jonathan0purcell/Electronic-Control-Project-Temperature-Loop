/*  Process Control Operations: Temperature Loop
 *  
 *  Through the use of various pushbuttons, the user can control the temperature
 *  loop with the aid of the arduino. It operates on one of four modes:
 *  
 *    1. Manual Control
 *    2. On/Off Control
 *    3. P Control
 *    4. PI Control
 *    
 *  Each mode will control the CO, to get the PV to the SP, with the use of
 *  other control operation variables (e.g., SP, hysteresis, Kp, Ti)
 *  
 *  The wiring, schematic and simulation for this project can be found at:
 *  https://www.tinkercad.com/things/jNcjeUPqnLh 
 *  
 *  Create 15 May, 2022
 *  by Jonathan Purcell
 *  
 *  This code, amoung others, can be found in the GitHub repository:
 *  https://github.com/jonathan0purcell/Electronic-Control-Project-Temperature-Loop
*/

// Library to use the I2C LCD functions
#include <LiquidCrystal_I2C.h>

// Defining the input and output pins
#define IN_BTN 13                   // Button to switch modes (0=manual, 1=on/off, 2=p, 3=pi)
#define IN_CO_SP_DEC 12             // Button to decrease the CO (when mode=1) or SP value (when mode=1 or 2 or 3)
#define IN_CO_SP_INC 8              // Button to increase the CO (when mode=1) or SP value (when mode=1 or 2 or 3)
#define IN_HYS_KP_DEC 7             // Button to decrease the Hysteresis (when mode=1) or Kp (when mode=2 or 3)
#define IN_HYS_KP_INC 4             // Button to increase the hysteresis (when mode=1) or Kp (when mode=2 or 3)
#define IN_KI_INC 2                 // Button to increase the Ki (when mode=3)
#define IN_KI_DEC 3                 // Button to decrease the Ki (when mode=3)
#define IN_KI_SCREEN 6              // Button to change the Ki mode screen
#define SENSOR A0                   // The LDR Sensor (Connected to analog pin)
#define ACTUATOR 10                 // The LED (Connected to digital PWM pin)

// Creating the LCD object using the I2C library
LiquidCrystal_I2C lcd(0x3F, 16, 2);

// Process control variables
int pv = 0;                         // The process variable of the process (LDR value)                    Range = 0 to 1023
int co = 0;                         // The controller output of the process (Brightness of LED)           Range = 0 to 255
int setpoint = 0;                   // The setpoint of the process (Brightness of the LED)                Range = 0 to 255
int hysteresis = 5;                // The hystersis of the on/off process
int error = 0;                      // The error of the p process
float kp = 0.01;                     // The p gain of the p(i) process (Float since can be a decimal value)
float ki = 0.01;                     // The i gain of the pi process (float since it can be decimal value)
int integralAction = 0;             // The accumulation of the error in pi
int mode = 0;                       // The currently selected control mode (0=Manual, 1=On/Off, 2=P)
bool kpModeSwitchScreens = false;

// Process control change variables (How much to change each variable when a pushbutton is pressed)
const int CO_CHANGE = 5;            // How much to change the controller output by when button is pressed
const int SP_CHANGE = 10;           // How much to change the setpoint by when button is pressed
const int HYSTERESIS_CHANGE = 1;    // How much to change the hysteresis when button is pressed
const float KP_CHANGE = 0.01;        // How much to change the p gain when button is pressed
const float KI_CHANGE = 0.01;

// To read button at intervals without interruption (Without using the delay function)
unsigned long previousMillis = 0;   // Will store last time the pushbutton state was read
const long interval = 100;          // Interval at which to read the input from buttons (in ms)

void setup() {
  // Begin the serial communication with a baud rate of 9600 bps
  Serial.begin(9600);

  // Initialization the LCD I2C properties
  lcd.init();                       // Starting the LCD
  lcd.clear();                      // Clearing the screen
  lcd.backlight();                  // Turning on the backlight

  // Set the LED PWM pin as output
  pinMode(ACTUATOR, OUTPUT);

  // Set the pushbuttons as inputs
  pinMode(IN_BTN, INPUT);
  pinMode(IN_CO_SP_DEC, INPUT);
  pinMode(IN_CO_SP_INC, INPUT);
  pinMode(IN_HYS_KP_DEC, INPUT);
  pinMode(IN_HYS_KP_INC, INPUT);
  pinMode(IN_KI_INC, INPUT);
  pinMode(IN_KI_DEC, INPUT);
  pinMode(IN_KI_SCREEN, INPUT);
}

void handleInputs() {
  /* Uses the the inputs/pushbuttons to set the process control variables
   *    Button 1: Controls the currently selected mode (0=manual, 1=on/off, 2=p)
   *    Button 2: Controls the decrement of CO (when mode=0) and SP (when mode=1 or 2)
   *    Button 3: Controls the increment of CO (when mode=0) and SP (when mode=1 or 2)
   *    Button 4: Controls the decrement of hysteresis (when mode=1) and Kp (when mode=2)
   *    Button 5: Controls the increment of hysteresis (when mode=1) and Kp (when mode=2) */
  
  // The current time
  unsigned long currentMillis = millis();

  // Read pushbutton state if the current time minus the last time
  // the pushbutton state was read is greater than the interval
  if (currentMillis - previousMillis >= interval) {
    // Set the previous time the pushbutton state was read
    // as the current time
    previousMillis = currentMillis;
    
    // [PB1] Change the process control mode if the first pushbutton is pressed
    // The mode can either be 0, 1, 2 or 3. If it goes higher it will go to 0
    if (digitalRead(IN_BTN) and mode < 3) {
      lcd.clear();
      mode += 1;
    } else if (digitalRead(IN_BTN)) {
      lcd.clear();
      mode = 0;
    }

    // [PB2] If the second pushbutton is pressed
    if (digitalRead(IN_CO_SP_DEC) and mode==0) {
      // Decrease CO if the mode is on manual (0)
      co -= CO_CHANGE;
    } else if (digitalRead(IN_CO_SP_DEC) and mode>=1) {
      // Decrease SP if the mode is on on/off (1) or P (2)
      setpoint -= SP_CHANGE;
    }
  
    // [PB3] If the third pushbutton is pressed
    if (digitalRead(IN_CO_SP_INC) and mode==0) {
      // Decrease CO if the mode is on manual (0)
      co += CO_CHANGE;
    } else if (digitalRead(IN_CO_SP_INC) and mode>=1) {
      // Decrease SP if the mode is on on/off (1) or P (2)
      setpoint += SP_CHANGE;
    }

    // [PB4] If the fourth button is pressed
    if (digitalRead(IN_HYS_KP_DEC) and mode==1) {
      // Decrease the hysteresis if the mode is on/off (1)
      hysteresis -= HYSTERESIS_CHANGE;
    } else if (digitalRead(IN_HYS_KP_DEC) and (mode==2 or mode==3)) {
      // Decrease the Kp if the mode is on p (2)
      kp -= KP_CHANGE;
    }

    // [PB5] If the fifth button is pressed
    if (digitalRead(IN_HYS_KP_INC) and mode==1) {
      // Increase the hysteresis if the mode is on/off (1)
      hysteresis += HYSTERESIS_CHANGE;
    } else if (digitalRead(IN_HYS_KP_INC) and (mode==2 or mode==3)) {
      // Increase the Kp if the mode is on p (2)
      kp += KP_CHANGE;
    }

    // [PB6] If the sixth button is pressed
    if (digitalRead(IN_KI_INC) and mode==3) {
      // Increase the Ki if the mode is pi (3)
      ki += KI_CHANGE;
      Serial.println("Test1");
    }

    // [PB7] If the seventh button is pressed
    if (digitalRead(IN_KI_DEC) and mode==3) {
      // Increase the Ki if the mode is pi (3)
      ki -= KI_CHANGE;
      Serial.println("Test2");
    }

    // [PB8] If the eight button is pressed
    if (digitalRead(IN_KI_SCREEN) and mode==3) {
      Serial.println("Button pressed"); 
      kpModeSwitchScreens = !kpModeSwitchScreens;
      lcd.clear();
    }
  }
}

void handleLCD() {
  /* Handles the clearning, printing and formatting of the LCD screen */

  // Resetting the cursor and clearing the screen
  lcd.setCursor(0, 0);

  // Print PV and CO in first row since all control strategies use it 
  // (Won't need to paste 3 times)
  lcd.print("PV:");
  lcd.print(pv);
  lcd.print(", CO:");
  lcd.print(co);

  // Set the cursor to the second row (or else would just paste it twice)
  lcd.setCursor(0,1);
 
  switch(mode) {
    case 0:
      // Manual Control 
      // (Already printed PV and CO above)
      break;
    case 1:
      // On/off Control
      lcd.print("SP:");
      lcd.print(setpoint);
      lcd.print(", Hys:");
      lcd.print(hysteresis);
      break;
    case 2:
      // P Control
      lcd.print("SP:");
      lcd.print(setpoint);
      lcd.print(", Kp:");
      lcd.print(kp);
      break;
    case 3:
      // PI Control
      if (kpModeSwitchScreens) {
        lcd.print("SP:");
        lcd.print(setpoint);
      } else {
        lcd.print("Kp:");
        lcd.print(kp, 2);
        lcd.print(", Ki:");
        lcd.print(ki, 2);
      }
      break;
  }
}

void handleSerialPrint() {
  // Regardless of the mode, they all print CO and PV  
  Serial.print("CO = ");
  Serial.print(co);
  Serial.print("\t PV = ");
  Serial.print(pv);

  // If mode is 1 (on/off) or 2 (p control)
  if (mode != 0) {
    Serial.print("\t SP = ");
    Serial.print(setpoint);
  }

  // If the mode is 1 (on/off) print the hysteresis
  if (mode == 1) {
    Serial.print("\t Hys = ");
    Serial.print(hysteresis);
  }

  // If the mode is 2 (p control) or 3 (pi control) print the Kp
  if (mode == 2 or mode == 3) {
    Serial.print("\t Kp = ");
    Serial.print(kp);
  }

  // If the mode is 3 (pi control) print the Ki
  if (mode == 3) {
    Serial.print("\t Ki = ");
    Serial.print(ki);
  }

  // If the mode is 2 (p control) or 3 (pi control), print the error
  if (mode == 2 or mode == 3) {
    Serial.print("\t Error = ");
    Serial.print(error);
  }

  // Print a new line to seperate the lines
  Serial.println();
}

/* Control Strategies Functions */

void manualControl() {
  /* In manual control, the CO% is controlled, there's no SP, and it is open loop */
}

void onOffControl() {
  // In on/off control, the output is either fully on (if pv less than sp-hys) 
  // or off (if pv is greater than sp+hys)
  if (pv <= (setpoint-hysteresis)) {
    co = 255;
  } else if (pv >= (setpoint+hysteresis)) {
    co = 0;
  }
}

void pControl() {
  error = setpoint - pv;                      // The error is the setpoint minus the process variable
  co = kp * error;                            // The controller output

  // Make sure the co doesn't go out of the limits
  if (co > 255) {
    co = 255;
  } else if (co < 0) {
    co = 0;
  }
}

void piControl() {
  error = setpoint - pv;                      // The error is the setpoint minus the process variable
  integralAction = (integralAction + error);  // The accumulation of the error in pi
  co = (kp * error) + (integralAction * ki);  // The controller output

  // Make sure the co doesn't go out of the limits
  if (co > 255) {
    co = 255;
  } else if (co < 0) {
    co = 0;
  }
}

void loop() {
  // Handle the input/pushbuttons (To control CO, SP, Hysterisus and Kp depending on mode)
  handleInputs();

  // Since all functions update PV, may as well put it here to avoid repeating
  // Read the analog value of the LDR sensor and put it in PV
  pv = analogRead(SENSOR);

  // Each mode has a corresponding function to control/calculate the CO 
  switch (mode) {
    case 0:
      // Function doesn't actually do anything
      // since the handleInputs() functions controls the CO
      manualControl();
      break;
    case 1:
      // Changes the CO based on PV
      onOffControl();
      break;
    case 2:
      // Changes the CO based on error
      pControl();
      break;
    case 3:
      // Changes CO based on
      piControl();
      break;
  }

  // Write the co value (modified by the above functions) to the LED
  // Putting this here so I don't have to repeat it in every function
  analogWrite(ACTUATOR, co);

  // Handles clearing, printing and formatting of the serial monitor
  // No point in repeating Serial.print for the same thing in every function
  handleSerialPrint();

  // Handles the clearing, printing, and formatting of the variables on the LCD
  // Function is called after since you want to update the process varibles beforehand
  handleLCD();
}
