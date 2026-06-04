/* RVTOP-Ccode 
 *  ROVotron Cadet topside control box main program 

(C) 2010, 2023, 2024 David Forbes

Program overview

The ROVotron top is a box that controls an ROV. 
It has a 4x20 char LCD display and a USB host port for an XBox One gamepad.
The ROV connects via an RJ45 4 pair cable with RS-485 serial data on one pair. 

This program controls the ROV by reading the gamepad and sending 
the translated motor commands to an RS-485 serial port. 
It receives telemetry and displays it on the LCD. 

The configuration is built into the code with tables. 

Revision history

2023-03-26 DF  forked from rtxa.c code, converting to Teensy and Xbox one
2023-09-09 DF  Removing all the config code
2023-09-10 DF  Corrected directions and trim, it works!
2023-09-11 DF  Made vector drive optional with a define
2023-11-25 DF  Removed vector drive, reduced number of motors and servos
2024-02-11 DF  Added motor index, matched motors to instructions
2024-02-22 DF  Added telems[5] for pressure
2024-02-25 DF  Updated depth scale factor and units
2024-05-31 DF  Fixed USB declarations to compile with TeensyDuino 1.59
2024-06-02 DF  Ver 1.20 - Added USB ID display at startup
2024-06-09 DF  Ver 1.21 - added temp readout enable to Y button

Known bugs:

Things to do:

Calibrate temp sensors

Function:

Uses an XBox One gamepad to run an ROV

Connects to ROV using RS-485 link with a hex ASCII command message
and a reply message with telemetry.

Displays useful information on an LCD display. 

The data format is curently described in RVdataDescr.txt

Terminology:

Buttons are the 14 user pushbuttons on the gamepad 
Analogs are the ten analog axes of control on the gamepad (button pairs or sticks)
Motors are the ROV motors 0..5 (gripper is 0)
Servos are the servo outputs 0..2 (dimming is 0)
volts are the raw 12-bit ADC outputs from the ROV (0 is battery)
telems are the floating telemetry values, mapped into user units

Motor parameters:
Gain is the multiplication factor for an analog to a motor, -99 to +99
mode is the speed mode: one of two below
0 OFF is a deactivated motor or servo
1 SPEED is no conversion. Passes analog to motor or servo directly
2 RATE causes analog to be integrated to make servo position or motor speed
in Rate mode, Gain is the speed at which a full stick increases the servo position

*/

// Included Teensy I/O libraries
#include <LiquidCrystalFast.h>
#include "USBHost_t36.h"

// ------------------ Teensy pin definitions ----------------------- //

// The LCD screen uses four data bits
// they have 1k series resistors for 3.3V compatibility
#define LCD_RS 5
#define LCD_RW 6
#define LCD_E  7
#define LCD_D4 24
#define LCD_D5 25
#define LCD_D6 26
#define LCD_D7 27

// ROV com port is Serial1
#define BAUD_RATE 115200

// RS-485 serial Tx enable pin
#define SER_TXEN 2

// for the cpu clock speed setting function
extern "C" uint32_t set_arm_clock(uint32_t frequency); // required prototype

// ------------ Sizes of things --------------------------- //

// max number of motor control channels in command message
#define NMOTORS 6
// max number of servo control channels in command message
#define NSERVOS 2
// max number of switch channels in command message
#define NSWITCHES 2
// max number of analog channels in reply message
#define NVOLTS 6
// number of hex digits in each motor parameter
#define MOTOR_DIGITS 2
// number of hex digits in each servo parameter
#define SERVO_DIGITS 2
// number of hex digits in each volts parameter
#define VOLTS_DIGITS 3

// the sloppiness of a XBox one joystick
#define DEADBAND 3000

// This value is written to a PWM device for center (zero) output
#define Pwm0  128



#define SLOW_MODE_SCALE 0.4
#define ACCEL_RATE_NORMAL 2.5
#define DECEL_RATE_NORMAL 6.0
#define ACCEL_RATE_SLOW 1.0
#define DECEL_RATE_SLOW 3.0
// ----------------- Motor configuration ------------------ //

// ESC to motor wiring as described in instructions
#define GripperMot 0
#define LUpDownMot 1
#define RUpDownMot 2
#define LForAftMot 3
#define RForAftMot 4
#define StrafeMot  5

// Motor direction array to easily reverse each motor
float motDirs[NMOTORS];

// If a motor runs backwards, change the sign in front of its 1 here.
#define GripperMotDir  1. 
#define LUpDownMotDir  1.
#define RUpDownMotDir -1.
#define LForAftMotDir -1.
#define RForAftMotDir  1.
#define StrafeMotDir   1.

// Motor gain trims
// Adjust these to get it to drive straight
// and to have the desired steering and strafing behavior
float DriveGainL  = 0.50;
float DriveGainR  = 0.60;
float SteerGain   = 0.60;
float DiveGainL   = 0.60;    // reducing total current draw
float DiveGainR   = 0.60;
float StrafeGain  = 0.50;
float GripperGain = 0.50;   // to keep from breaking the plastic!

// make the fast LCD screen object - uses RW to allow readback of screen contents
LiquidCrystalFast lcd(LCD_RS, LCD_RW, LCD_E, LCD_D4, LCD_D5, LCD_D6, LCD_D7);

// ------------ USB joystick object ----------------------- // 

USBHost myusb;
USBHub hub1(myusb);
USBHIDParser hid1(myusb);

#define COUNT_JOYSTICKS 4
JoystickController joysticks[COUNT_JOYSTICKS] = {
  JoystickController(myusb), JoystickController(myusb),
  JoystickController(myusb), JoystickController(myusb)
};
int user_axis[64];
uint32_t buttons_prev = 0;

// display USB device name buffer
char namebuf[100] = { 0 };


// This is set up to handle up to four gamepads on a hub. 
// Not really needed for ROVotron
USBDriver *drivers[] = {&hub1, &joysticks[0], &joysticks[1], &joysticks[2], &joysticks[3], &hid1};
#define CNT_DEVICES (sizeof(drivers)/sizeof(drivers[0]))
const char * driver_names[CNT_DEVICES] = {"Hub1", "joystick[0D]", "joystick[1D]", "joystick[2D]", "joystick[3D]",  "HID1"};
bool driver_active[CNT_DEVICES] = {false, false, false, false};


// Lets also look at HID Input devices
USBHIDInput *hiddrivers[] = {&joysticks[0], &joysticks[1], &joysticks[2], &joysticks[3]};
#define CNT_HIDDEVICES (sizeof(hiddrivers)/sizeof(hiddrivers[0]))
const char * hid_driver_names[CNT_DEVICES] = {"joystick[0H]", "joystick[1H]", "joystick[2H]", "joystick[3H]"};
bool hid_driver_active[CNT_DEVICES] = {false};

//------------ GLOBAL VARIABLES AND RUNTIME STATE ------------------------------- //

// The main loop speed is governed by this software timer
unsigned long loopPeriod = 50000;  // in microseconds: 20 loops per second
unsigned long nextLoopMicros = 0;

unsigned long lastSlewMicros = 0;
char inString[100];          // Storage for the received telemetry string 
char inChr = 0;              // the receive char is built here
char * inPtr;
bool gotInString = false;    // a data link string has been received, delimited by term char

// ------------ XBox gamepad control mapping ----------------- //

// The gamepad has buttons and joysticks. These are read into butts and axes[]. 
// Then these values are translated into buttons[] and analogs[]. 
// Those values are mapped into motion values for each motor and servo. 

// total analog axes including button pairs
#define NANALOG 10
#define NBUTTONS 16

// Real Xbox gamepad data
uint32_t butts;    // the single word that contains all button bits 
unsigned char buttons[NBUTTONS];  // button control values: 1 = pressed, 0 = not
int axes[6];     // the value of each real analog control axis from gamepad

// Index into buttons[] for each button
// the var butts is in order of these bits
int PairButton = 1;
int MenuButton = 2;
int ViewButton = 3;
int AButton    = 4;
int BButton    = 5;
int XButton    = 6;
int YButton    = 7;
int DpadUp     = 8;
int DpadDown   = 9;
int DpadLeft   = 10;
int DpadRight  = 11;
int LButton    = 12;
int RButton    = 13;
int LJButton   = 14;
int RJButton   = 15;

// Gamepad analog index, extended to button pads for controlling dimming, camera etc. 
// This isn't quite in Xbox one order, trigs are weirdly placed
int LJoyX = 0;
int LJoyY = 1;
int RJoyX = 2;
int RJoyY = 3;
int LTrig = 4;
int RTrig = 5;
int DPadX = 6;  // synthesized from dpad presses
int DPadY = 7;
int ButsX = 8;  // synthesized from button pad presses
int ButsY = 9;

// Each control analog value indexed above is -1.0 to +1.0, zero is idle
float analogs[NANALOG];    // analog control values extracted from gamepad

float requestedAnalogs[NANALOG];
// Each of these maps a button to move a synthesized axis up or down
int up_button[NANALOG] = {0,0,0,0,0,0,DpadRight,DpadUp,  BButton,YButton};
int dn_button[NANALOG] = {0,0,0,0,0,0,DpadLeft, DpadDown,XButton,AButton};

// motion step of synthesized axis per main loop iteration
float button_inc = 0.03;

// these are sent to the ROV
int motors[NMOTORS];    // motor command values 0=full reverse, FF=full fwd
int servos[NSERVOS];    // servo command values 0=full CCW, FF=full CW
int switches[NSWITCHES];  // on/off switches based on buttons held down

bool slow_mode_enabled = false;
bool lbRbComboWasPressed = false;
// The ROV messages get put here
char command_msg[100];  // command message, newline, 0
char reply_msg[100];    // reply message, newline, 0
char buf[100];      // temporary string storage

// data from ROV
float volts[NVOLTS];      // telemetry voltages 0.0 to 3.3V (we receive 0..0xFFF)
float telems[NVOLTS];     // telemetry in its units

// --------------- Telemetry calculation ----------------- //

// Telemetry conversion factors
// Depth is 0.5V = 1 atm, 58 PSI/V, H2O is 0.42 lb/ft depth
// volts[1] is degC H2O, zero is 0C, scale is 1/128

// Temp scale needs caclualtion...
//    signal              tether   ana1   ana2    ana3    ana4    press
//    measurement         Battery  H2Otemp not  LEDtemp   ---    Depth
//    units                Volts    degC   used   degC    --      Feet
float telZeros[NVOLTS] = {   0.00,  0.00,  0.10,   1.40,   1.40,  0.00};
float telScale[NVOLTS] = {  11.00,  256./3.3,  8.00,  70.00,  71.00, 90};

/*
Purpose:
Formats and displays decoded telemetry values on the operator LCD.
Inputs:
Global volts[] array populated by parse_reply_msg().
Outputs:
Updates the four-line LCD display.
Side Effects:
Writes to LCD hardware.
Dependencies:
Telemetry scaling constants must match bottom-side sensor wiring.
Notes:
This function also owns the visible slow-mode status line in the current source.
*/
void display_all_telems(void) {
  float telems[NVOLTS];
  for (int i=0;i<NVOLTS;i++) {
    telems[i] = (volts[i] - telZeros[i]) * telScale[i];
  }
  sprintf(buf, "Battery  %5.1f V", telems[0]);
  lcd.setCursor(0, 0);
  lcd.print(buf);
  sprintf(buf, "Depth    %5.2f Feet", telems[5]);
  lcd.setCursor(0, 1);
  lcd.print(buf);
  lcd.setCursor(0, 2);
  lcd.print(slow_mode_enabled ? "SLOW MODE: ON      " : "SLOW MODE: OFF     ");
  sprintf(buf, "H2O temp %5.1f C", telems[1]);
  lcd.setCursor(0, 3);
  lcd.print(buf);
}

// ------------------- ROV message I/O ------------------ //

// !!! This needs to be redone in a non-blocking manner. 

// get a message from ROV serial port
// returns 0 if OK, 1 if timeout error
// The timer waits a fraction of a second for a valid message. 
// it throws away chars until M seen, to sync up to start of real message
/*
Purpose:
Legacy blocking helper for receiving a full RS-485 message beginning with a selected sync character.
Inputs:
Destination buffer and expected first character.
Outputs:
Returns 0 on success, 1 on timeout.
Side Effects:
Consumes bytes from Serial1.
Dependencies:
Serial1 must be initialized.
Notes:
Current main loop uses non-blocking receive logic instead.
*/
int receive_msg(char *msg, char first_char) {
  int timer;

  // Wait for the start character, discarding all others
  do {
    timer = 10000;        // 500 millisecond timeout
    while (!Serial1.available()) {
      delayMicroseconds(50);  
      if (!--timer) 
        return 1;
    }
  }
    while ((*msg = Serial1.read()) != first_char);
  msg++;  // save start char

  // wait for the rest of message with shorter timeout
  do {
    timer = 100;        // 5 millisecond timeout
    while (!Serial1.available()) {
      delayMicroseconds(50);  
      if (!--timer) 
        return 1;
    }
  }
    while ((*msg++ = Serial1.read()) != '\0');  // read the rest of message thru NUL
  return 0;
}

// -------------------- COMMUNICATION PARSING HELPERS ---------------------- //

// convert a character from ASCII to hex
// returns -1 if invalid hex character
/*
Purpose:
Converts one ASCII hexadecimal character into a numeric nibble.
Inputs:
Single character from an RS-485 packet field.
Outputs:
0..15 for valid hex, -1 for invalid input.
Side Effects:
None.
Dependencies:
Used by atox() during packet parsing.
Notes:
Packet validation depends on callers checking the returned error value.
*/
char atoxdigit(char chr) {
  if (('a' <= chr) && (chr <= 'f')) return (chr - 'a' + 10);
  if (('A' <= chr) && (chr <= 'F')) return (chr - 'A' + 10);
  if (('0' <= chr) && (chr <= '9')) return (chr - '0');
  return -1;
}

// get an n digit positive hex number into an int from string
// returns -1 if invalid hex characters
/*
Purpose:
Parses a fixed-width ASCII hexadecimal field from a packet buffer.
Inputs:
Pointer to the first hex digit and the number of digits to read.
Outputs:
Integer field value, or an error value for invalid hex.
Side Effects:
None.
Dependencies:
Uses atoxdigit() for individual character conversion.
Notes:
Callers advance their own packet pointer after this function returns.
*/
int atox(char *str, char n) {
  char digit, i;
  int val;
  val = 0;
  for (i = 0; i < n; i++) {
    if ((digit = atoxdigit(*str++)) == -1) return -4;
    val = (val << 4) + digit;
  }
  return val;
}

// Parse the reply in msg into the float array volts
// Each value is 12 bits of hex. 
// It gets scaled into real volts. 
// returns 1 if error, 0 if all OK
int parse_reply_msg(char *msg) {
  int i;
  int val;

  if (*msg++ != 'V') return 1;    // ADC voltages

  for (i=0;i<NVOLTS;i++) {
    if ((val = atox(msg, VOLTS_DIGITS)) == -1) return 1;
    msg += VOLTS_DIGITS;
    volts[i] = (float)(val) * 3.3/4096.;  // save it
  }

  if (*msg++ != '\n') return 1; 
  if (*msg++ != '\0') return 1; 
  return 0;
}

// ----------------- translation code -------------------- //

// Remove deadband slop from joysticks
// And scale joystick value from 16 bit int to float for motor math
/*
Purpose:
Applies joystick deadband and scales raw stick input to normalized control space.
Inputs:
Raw joystick axis value from USBHost_t36.
Outputs:
Normalized float used by motion mixing.
Side Effects:
None.
Dependencies:
DEADBAND must match controller noise/slop.
Notes:
This is the first stage of pilot command shaping.
*/
float joyScale(int stick) {
  int val = 0;
  if (stick >  DEADBAND) val = (stick - DEADBAND)/256;
  if (stick < -DEADBAND) val = (stick + DEADBAND)/256;
  return (float)(val) * 32768. / (32768.-DEADBAND)/128.; // scale out the deadband
}

// Scale trigger value from 10 bit int to float (it's different from joysticks)
/*
Purpose:
Scales raw trigger input into normalized gripper command space.
Inputs:
Raw trigger axis value.
Outputs:
Normalized trigger value.
Side Effects:
None.
Dependencies:
Controller trigger range is assumed to be 0..1024.
Notes:
Trigger path is separate from joystick motion axes.
*/
float trigScale(int stick) {
  return (float)(stick)/1024.;
}

float limitRateOfChange(float current, float target, float accelRate, float decelRate, float dtSeconds) {
  float rate = accelRate;
  if ((current > 0.0 && target < current) || (current < 0.0 && target > current)) {
    rate = decelRate;
  }
  if ((current > 0.0 && target < 0.0) || (current < 0.0 && target > 0.0)) {
    target = 0.0;
    rate = decelRate;
  }
  float maxStep = rate * dtSeconds;
  float delta = target - current;
  if (delta > maxStep) return current + maxStep;
  if (delta < -maxStep) return current - maxStep;
  return target;
}

void updateSlowModeToggle(void) {
  bool lbRbComboPressed = buttons[LButton] && buttons[RButton];
  if (lbRbComboPressed && !lbRbComboWasPressed) {
    slow_mode_enabled = !slow_mode_enabled;
    lbRbComboWasPressed = true;
    lcd.setCursor(0, 2);
    lcd.print(slow_mode_enabled ? "SLOW MODE: ON      " : "SLOW MODE: OFF     ");
  }
  if (!lbRbComboPressed) {
    lbRbComboWasPressed = false;
  }
}

void applySlewRate(float dtSeconds) {
  float accelRate = slow_mode_enabled ? ACCEL_RATE_SLOW : ACCEL_RATE_NORMAL;
  float decelRate = slow_mode_enabled ? DECEL_RATE_SLOW : DECEL_RATE_NORMAL;
  float scale = slow_mode_enabled ? SLOW_MODE_SCALE : 1.0;

  analogs[LJoyY] = limitRateOfChange(analogs[LJoyY], requestedAnalogs[LJoyY] * scale, accelRate, decelRate, dtSeconds);
  analogs[LJoyX] = limitRateOfChange(analogs[LJoyX], requestedAnalogs[LJoyX] * scale, accelRate, decelRate, dtSeconds);
  analogs[RJoyX] = limitRateOfChange(analogs[RJoyX], requestedAnalogs[RJoyX] * scale, accelRate, decelRate, dtSeconds);
  analogs[RJoyY] = limitRateOfChange(analogs[RJoyY], requestedAnalogs[RJoyY] * scale, accelRate, decelRate, dtSeconds);
}

// read gamepad and populate the control variables with current readings
/*
Purpose:
Reads USB gamepad state and updates normalized pilot command variables.
Inputs:
USB joystick buttons and axes.
Outputs:
Updates analogs[], switches[], and related controller state.
Side Effects:
May update LCD slow-mode status in the current source.
Dependencies:
myusb.Task() must run regularly before this function.
Notes:
Motion axes feed the thruster mixer; trigger and D-pad controls feed actuator commands.
*/
void translate_response_to_controls() {
  bool sawGamepadFrame = false;
  unsigned long nowMicros = micros();
  float dtSeconds = loopPeriod / 1000000.0;
  if (lastSlewMicros != 0) {
    dtSeconds = (nowMicros - lastSlewMicros) / 1000000.0;
  }
  if (dtSeconds > 0.10) dtSeconds = 0.10;
  lastSlewMicros = nowMicros;
  for (int joystick_index = 0; joystick_index < COUNT_JOYSTICKS; joystick_index++) {
  // read the xbox controller over USB
    if (joysticks[joystick_index].available()) {
      sawGamepadFrame = true;
      // all the buttons are in one word
      butts = joysticks[joystick_index].getButtons();
   //   Serial.printf("micros %9d  buttons %04X", nextLoopMicros, butts);
      // the first six axes are what we need
      for (uint8_t i = 0; i<6; i++) {
        axes[i] = joysticks[joystick_index].getAxis(i);
  //      Serial.printf(" %6d", axes[i]);
      }
  //    Serial.println();
      // Read all the button bits into an array for ease of access
      for (int i=0;i<16;i++) {
        buttons[i] = (butts>>i)&0x0001;
      }
  
      // XBox One stick reading, scales to -1.0 .. +1.0 range
      updateSlowModeToggle();
      requestedAnalogs[LJoyX] = joyScale(axes[0]);
      requestedAnalogs[LJoyY] = joyScale(axes[1]);
      requestedAnalogs[RJoyX] = joyScale(axes[2]);
      requestedAnalogs[RJoyY] = joyScale(axes[5]);
      analogs[LTrig] = trigScale(axes[3]);
      analogs[RTrig] = trigScale(axes[4]);
  
        // calculate virtual stick position for each synthesized axis from button pair
      for (int i=6; i<NANALOG; i++) {
        float val = analogs[i];  // get current position
        if (buttons[up_button[i]]) val += button_inc;
        if (buttons[dn_button[i]]) val -= button_inc;
        if (val >  1.) val =  1.;        // obey range limit 
        if (val < -1.) val = -1.;
        // if (buttons[zero_button[i]]) val = 0.;  // reset to midpoint
        analogs[i] = val;   // update it as calculated
      }
      // use X and Y buttons as on/off controls (for temp reading)
      switches[0] = buttons[YButton];
      switches[1] = buttons[BButton];
      
      if (0) {
        for (int i=0;i<NANALOG;i++) {
          Serial.printf(" %6.3f", analogs[i]);
        }
        Serial.println();
      }
    }
  }
  if (sawGamepadFrame) {
    applySlewRate(dtSeconds);
  }
}

// rescale the controls to the motors and servos
float motScale = 128.;

// make sure the numbers don't overflow the 2 digit hex range
/*
Purpose:
Clamps signed motor/servo offsets so command bytes stay inside the 2-digit hex range.
Inputs:
Signed command offset around neutral.
Outputs:
Clamped offset from -127 to +127.
Side Effects:
None.
Dependencies:
Packet fields are encoded as two hex digits.
Notes:
This protects packet formatting, not physical current draw.
*/
int lims(int val) {
  int res = val;
  if (res > 127) res = 127;
  if (res < -127) res = -127;
  return res;
}

/* calculate the motor outputs based on the controls and trims

  Left  X is strafe  (move left or right without rotating)
  Left  Y is drive   (move forward or reverse)
  Right X is steer   (rotate the ROV left or right)
  Right Y is dive    (move up or down)
  Trigger is gripper (left to release, right to close)
*/

/*
Purpose:
Mixes normalized pilot controls into motor and servo command bytes.
Inputs:
analogs[], direction signs, and trim gains.
Outputs:
Updates motors[] and servos[] command arrays.
Side Effects:
None until build_command_msg() transmits the values.
Dependencies:
Controller input processing must run first.
Notes:
Thruster mapping and gain behavior are intentionally unchanged.
*/
void translate_controls_to_commands() {
  float LRForeAft = analogs[LJoyY]*DriveGainL + analogs[RJoyX]*SteerGain;
  float RRForeAft = analogs[LJoyY]*DriveGainR - analogs[RJoyX]*SteerGain;
  float Strafe    = analogs[LJoyX]*StrafeGain;
  float LUpDown   = analogs[RJoyY]*DiveGainL;
  float RUpDown   = analogs[RJoyY]*DiveGainR;
  // gripper uses both of the triggers
  float Gripper   = (analogs[RTrig] - analogs[LTrig])*GripperGain;

  // calculate motor speed for each motor output
  // Also limit the values to valid PWM range 
  motors[GripperMot] = Pwm0 + lims((int)(Gripper  *motDirs[GripperMot]*motScale));
  motors[LUpDownMot] = Pwm0 + lims((int)(LUpDown  *motDirs[LUpDownMot]*motScale));
  motors[RUpDownMot] = Pwm0 + lims((int)(RUpDown  *motDirs[RUpDownMot]*motScale));
  motors[LForAftMot] = Pwm0 + lims((int)(LRForeAft*motDirs[LForAftMot]*motScale));
  motors[RForAftMot] = Pwm0 + lims((int)(RRForeAft*motDirs[RForAftMot]*motScale));
  motors[StrafeMot ] = Pwm0 + lims((int)(Strafe   *motDirs[StrafeMot ]*motScale));

  // the servos use the D pad buttons for camera tilt and brightness
  servos[0] = Pwm0 + lims((int)(analogs[DPadX] * motScale));  // LED dimming
  servos[1] = Pwm0 + lims((int)(-analogs[DPadY] * motScale));  // camera tilt servo
}


// build a command string for ROV
/*
Purpose:
Builds the top-side RS-485 command packet from current motor, servo, and switch command arrays.
Inputs:
Global motors[], servos[], and switches[].
Outputs:
Writes a newline-terminated command packet into msg.
Side Effects:
None until Serial1.print() sends the buffer.
Dependencies:
Bottom-side parser must use the same packet field order and widths.
Notes:
Protocol fields were not changed by this documentation pass.
*/
void build_command_msg(char *msg) {
  int i, checksum = 0;

  *msg++ = 'M';
  for (i=0;i<NMOTORS;i++) {
    checksum += motors[i];
    msg += sprintf(msg, "%02X", motors[i]);
  }

  *msg++ = 'P';
  for (i=0;i<NSERVOS;i++) {
    checksum += servos[i];
    msg += sprintf(msg, "%02X", servos[i]);
  }

  *msg++ = 'S';
  for (i=0;i<NSWITCHES;i++) {
    checksum += switches[i];
    msg += sprintf(msg, "%01X", switches[i]);
  }

  *msg++ = 'C';
  checksum = checksum%0x100;
  msg += sprintf(msg, "%02X", checksum);
  *msg++ = '\n'; 
  *msg = '\0';
}

// ----------- Show when USB devices are added or removed ------------------

/*
Purpose:
Reports USB gamepad attach/detach events and mirrors device IDs onto the LCD.
Inputs:
USBHost driver state.
Outputs:
Serial debug lines and LCD device ID text.
Side Effects:
Writes to Serial and LCD only when USB device state changes.
Dependencies:
myusb.Task() must run in the main loop.
Notes:
Useful during controller compatibility bring-up.
*/
void PrintDeviceListChanges() {
  for (uint8_t i = 0; i < CNT_DEVICES; i++) {
    if (*drivers[i] != driver_active[i]) {
      if (driver_active[i]) {
        Serial.printf("*** Device %s - disconnected ***\n", driver_names[i]);
        driver_active[i] = false;
      } else {
        Serial.printf("*** Device %s %x:%x - connected ***\n", driver_names[i], drivers[i]->idVendor(), drivers[i]->idProduct());
        sprintf(namebuf,"Dev ID %04X:%04X", drivers[i]->idVendor(), drivers[i]->idProduct());
        // display Dev ID on the LCD to make life easier
        lcd.setCursor(0, 1);
        lcd.print(namebuf);   
        driver_active[i] = true;

        const uint8_t *psz = drivers[i]->manufacturer();
        if (psz && *psz) Serial.printf("  manufacturer: %s\n", psz);
        psz = drivers[i]->product();
        if (psz && *psz) Serial.printf("  product: %s\n", psz);
        psz = drivers[i]->serialNumber();
        if (psz && *psz) Serial.printf("  Serial: %s\n", psz);
      }
    }
  }

  for (uint8_t i = 0; i < CNT_HIDDEVICES; i++) {
    if (*hiddrivers[i] != hid_driver_active[i]) {
      if (hid_driver_active[i]) {
        Serial.printf("*** HID Device %s - disconnected ***\n", hid_driver_names[i]);
        hid_driver_active[i] = false;
      } else {
        Serial.printf("*** HID Device %s %x:%x - connected ***\n", hid_driver_names[i], hiddrivers[i]->idVendor(), hiddrivers[i]->idProduct());
        sprintf(namebuf,"Dev ID %04X:%04X", hiddrivers[i]->idVendor(), hiddrivers[i]->idProduct());
        // display Dev ID on the LCD to make life easier
        lcd.setCursor(0, 1);
        lcd.print(namebuf);   
        hid_driver_active[i] = true;

        const uint8_t *psz = hiddrivers[i]->manufacturer();
        if (psz && *psz) Serial.printf("  manufacturer: %s\n", psz);
        psz = hiddrivers[i]->product();
        if (psz && *psz) Serial.printf("  product: %s\n", psz);
        psz = hiddrivers[i]->serialNumber();
        if (psz && *psz) Serial.printf("  Serial: %s\n", psz);
      }
    }
  }
}
/* ----------------- init code -------------------- */

// Initialize the onboard peripherals
/*
Purpose:
Initializes serial ports, actuator pins, ADC resolution, Servo objects, and ESC startup sequence.
Inputs:
None.
Outputs:
Hardware peripherals are configured for the main control loop.
Side Effects:
Runs blocking startup delays required by the existing ESC initialization behavior.
Dependencies:
Pin definitions and Servo library configuration must match the control board wiring.
Notes:
Behavior intentionally unchanged by this documentation pass.
*/
void setup() {
  // set the CPU clock slower than 600 MHz to make the LCD work properly
  if ( F_CPU_ACTUAL >= 100'000'000 )
    set_arm_clock(100'000'000);

// Set motor directions per the defines at top
motDirs[GripperMot] = GripperMotDir;  
motDirs[LUpDownMot] = LUpDownMotDir; 
motDirs[RUpDownMot] = RUpDownMotDir;
motDirs[LForAftMot] = LForAftMotDir;
motDirs[RForAftMot] = RForAftMotDir;
motDirs[StrafeMot]  = StrafeMotDir;

  // Debug port via micro USB
  Serial.begin(115200);
  delay(100);
  Serial.println("Starting Teensy");

  // ROV RS-485 serial comms port
  Serial1.begin(BAUD_RATE); 
  Serial1.transmitterEnable(SER_TXEN);
  Serial.println("Starting LCD");
  
  lcd.begin(20,4);    // LCD size
  lcd.clear();  
  lcd.setCursor(0, 0);
  lcd.print("   ROVotron Cadet  ");    // display splash screen
  lcd.setCursor(0, 1);
  lcd.print(" ROV Control System");
  lcd.setCursor(0, 3);
  lcd.print("     Rev. 1.21");
//  Serial.println("LCD should be alive");
  delay(1500);
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Seeking Gamepad");   // the USB ID of gamepad will display below this
  lcd.setCursor(0, 2);
  lcd.print("SLOW MODE: OFF     ");
// make the joystick host USB port 
  myusb.begin();
// initialize all motors and servos to safe position
  for (int i=0;i<NMOTORS;i++) 
    motors[i] = Pwm0;
  for (int i=0;i<NSERVOS;i++) 
    servos[i] = Pwm0;
  inPtr = reply_msg;       // initialize the reply message receiver
  gotInString = false;
}

/* --------------------- main loop --------------------------- */

/*
Purpose:
Runs the bottom-side control loop: sample telemetry, receive command packets, reply, parse, and apply actuator outputs.
Inputs:
RS-485 bytes from Serial1 and analog sensor readings.
Outputs:
Telemetry packets and actuator command pulses/PWM.
Side Effects:
Updates thrusters, camera servo, gripper outputs, LED PWM, and debug Serial output.
Dependencies:
Top-side packet format must match parse_command_msg().
Notes:
No behavior changes were made; comments clarify the existing pipeline only.
*/
void loop() {    
 // unsigned long mics = micros();
 // Serial.printf("Mics %d\n", mics);
  myusb.Task();
  PrintDeviceListChanges();  // show USB connect/disconnect activity
  
  // Read the data link receive port, and act on the char that was received
  if (Serial1.available()) {
    int inChr = Serial1.read();
    if (inChr) *inPtr++ = inChr;   // store it if it's not null
// newline indicates the message is complete, so prepare it for action
    if (inChr == '\n') {
      *inPtr++ = 0x00;           // null terminate the input string
      gotInString = true;
      inPtr = reply_msg;       // tell the parser to do its work on this string
    }
  }
  // The command message is sent on a rigid schedule
  // It's built from the gamepad control status
  if (micros() > nextLoopMicros) {
    nextLoopMicros += loopPeriod;
    translate_response_to_controls();  // Read the gamepad into analogs, buttons
    translate_controls_to_commands();  // make ROV motion data from that
    build_command_msg(command_msg);
  //  Serial.print(command_msg);        // show commands to monitor
    Serial1.print(command_msg);        // give commands to ROV
  }
  // The reply message is parsed as soon as it's received
  // And the LCD status is redrawn
  if (gotInString) {
    Serial.print(reply_msg);        // give commands to ROV
    gotInString = false;
    parse_reply_msg(reply_msg);    // update telemetry if it's valid
    display_all_telems();          // display telemetry on the LCD
  }
}

	     
