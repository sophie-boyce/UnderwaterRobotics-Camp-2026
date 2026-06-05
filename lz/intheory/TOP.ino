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
#define NSERVOS 3
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



#define SLOW_MODE_SCALE 0.4          // Multiplies motion axes in slow mode for precision driving.
#define ACCEL_RATE_NORMAL 2.5       // Normal-mode max joystick ramp-up rate, normalized units/sec.
#define DECEL_RATE_NORMAL 6.0       // Normal-mode ramp-down rate for quick stopping.
#define ACCEL_RATE_SLOW 1.0         // Slow-mode max joystick ramp-up rate.
#define DECEL_RATE_SLOW 3.0         // Slow-mode ramp-down rate; still faster than acceleration.
#define VERTICAL_DEADBAND 0.05      // Released vertical-stick threshold in normalized units.
#define DEPTH_HOLD_ACTIVATION_DELAY_MS 500 // Delay after stick release before capturing target depth.
#define DEPTH_HOLD_DEADBAND 0.15  // Feet of allowed depth error before PID correction begins.
#define MAX_VERTICAL_PID_OUTPUT 0.35 // Maximum automatic vertical command from depth hold.
#define DEPTH_KP 0.45               // Proportional correction for depth error in feet.
#define DEPTH_KI 0.02               // Integral correction for persistent depth error.
#define DEPTH_KD 0.10               // Derivative damping for depth-rate changes.
#define DEPTH_UNITS_SCALE 1.0  // Depth telemetry is already scaled to feet by telScale[5].
#define DEPTH_FILTER_ALPHA 0.10     // Low-pass filter weight for new depth samples.
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
bool depthHoldEnabled = false;
bool verticalStickReleased = false;
bool depthTelemetryValid = false;
unsigned long verticalReleaseMillis = 0;
float currentDepthMeters = 0.0;  // Current depth in feet; name retained to avoid wider code churn.
float filteredDepthMeters = 0.0; // Filtered depth in feet for display and PID.
float targetDepthMeters = 0.0;   // Captured hold target depth in feet.
float depthIntegral = 0.0;
float previousDepthError = 0.0;
float depthPidOutput = 0.0;
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
Displays operator mode state on LCD line 0.
Inputs:
Global slow_mode_enabled and depthHoldEnabled flags.
Outputs:
Writes slow-mode and depth-hold state to the LCD.
Side Effects:
Updates the operator display only.
Dependencies:
LCD must be initialized in setup().
Notes:
Keeps mode state visible without changing command packets or control timing.
*/
void display_mode_status(void) {
  sprintf(buf, "SLOW:%3s HOLD:%3s", slow_mode_enabled ? "ON" : "OFF", depthHoldEnabled ? "ON" : "OFF");
  lcd.setCursor(0, 0);
  lcd.print(buf);
  lcd.print("   ");
}

/*
Purpose:
Formats telemetry and depth-hold status for the 4-line operator LCD.
Inputs:
volts[] telemetry values and filtered depth-hold state.
Outputs:
Line 0: slow/hold state; line 1: battery; line 2: depth in feet; line 3: PID error/status.
Side Effects:
Writes to LCD hardware.
Dependencies:
parse_reply_msg() must update volts[] and filtered depth before display refresh.
Notes:
Depth and PID error are shown in feet.
*/
void display_all_telems(void) {
  float telems[NVOLTS];
  for (int i=0;i<NVOLTS;i++) {
    telems[i] = (volts[i] - telZeros[i]) * telScale[i];
  }
  display_mode_status();
  sprintf(buf, "Battery: %5.1fV", telems[0]);
  lcd.setCursor(0, 1);
  lcd.print(buf);
  lcd.print("    ");
  sprintf(buf, "Depth:%6.2f ft", filteredDepthMeters);
  lcd.setCursor(0, 2);
  lcd.print(buf);
  lcd.print("    ");
  sprintf(buf, "PID:%+5.2f ft %3s", targetDepthMeters - filteredDepthMeters, depthHoldEnabled ? "ON" : "OFF");
  lcd.setCursor(0, 3);
  lcd.print(buf);
  lcd.print("   ");
}

// ------------------- ROV message I/O ------------------ //

// Legacy helper below is blocking; the active main loop uses non-blocking Serial1 reads.

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
  currentDepthMeters = (volts[5] - telZeros[5]) * telScale[5] * DEPTH_UNITS_SCALE;
  if (!depthTelemetryValid) {
    filteredDepthMeters = currentDepthMeters;
  } else {
    filteredDepthMeters = filteredDepthMeters * (1.0 - DEPTH_FILTER_ALPHA) + currentDepthMeters * DEPTH_FILTER_ALPHA;
  }
  depthTelemetryValid = true;
  return 0;
}

// ----------------- CONTROLLER INPUT, SLEW LIMITING, AND DEPTH HOLD -------------------- //

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

// D-pad: Xbox uses buttons[8..11]; PS4/HID often use butts bits 16..19 or hat axis 9.
void readDpadDirections(bool &up, bool &down, bool &left, bool &right) {
  up = buttons[DpadUp] || (butts & 0x10000);
  down = buttons[DpadDown] || (butts & 0x40000);
  left = buttons[DpadLeft] || (butts & 0x80000);
  right = buttons[DpadRight] || (butts & 0x20000);

  for (int ji = 0; ji < COUNT_JOYSTICKS; ji++) {
    if (!(bool)joysticks[ji]) continue;
    int hat = joysticks[ji].getAxis(9);
    if (hat >= 0 && hat <= 7) {
      up |= (hat == 0 || hat == 1 || hat == 7);
      right |= (hat == 1 || hat == 2 || hat == 3);
      down |= (hat == 3 || hat == 4 || hat == 5);
      left |= (hat == 5 || hat == 6 || hat == 7);
    }
    const int HAT_THRESH = 8000;
    int ax6 = joysticks[ji].getAxis(6);
    int ax7 = joysticks[ji].getAxis(7);
    if (ax6 > HAT_THRESH) right = true;
    else if (ax6 < -HAT_THRESH) left = true;
    if (ax7 > HAT_THRESH) down = true;
    else if (ax7 < -HAT_THRESH) up = true;
  }
}

void updateVirtualPadAxes(void) {
  bool dUp, dDown, dLeft, dRight;
  readDpadDirections(dUp, dDown, dLeft, dRight);

  float val = analogs[DPadX];
  if (dRight) val += button_inc;
  if (dLeft) val -= button_inc;
  if (val > 1.0f) val = 1.0f;
  if (val < -1.0f) val = -1.0f;
  analogs[DPadX] = val;

  val = analogs[DPadY];
  if (dUp) val += button_inc;
  if (dDown) val -= button_inc;
  if (val > 1.0f) val = 1.0f;
  if (val < -1.0f) val = -1.0f;
  analogs[DPadY] = val;

  for (int i = ButsX; i < NANALOG; i++) {
    val = analogs[i];
    if (buttons[up_button[i]]) val += button_inc;
    if (buttons[dn_button[i]]) val -= button_inc;
    if (val > 1.0f) val = 1.0f;
    if (val < -1.0f) val = -1.0f;
    analogs[i] = val;
  }
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
    display_mode_status();
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

/*
Purpose:
Limits a floating-point value to a safe numeric range.
Inputs:
Value to clamp, minimum allowed value, maximum allowed value.
Outputs:
Clamped floating-point value.
Side Effects:
None.
Dependencies:
Used by depth-hold PID output limiting.
Notes:
Keeps PID output bounded before it is applied to vertical motion command.
*/
float constrainFloat(float val, float minVal, float maxVal) {
  if (val < minVal) return minVal;
  if (val > maxVal) return maxVal;
  return val;
}

/*
Purpose:
Reports whether USBHost currently sees a joystick/HID controller connection.
Inputs:
USB driver active-state arrays maintained by PrintDeviceListChanges().
Outputs:
True when a controller-like USB device is active.
Side Effects:
None.
Dependencies:
myusb.Task() and PrintDeviceListChanges() must run regularly.
Notes:
Depth hold is disabled when the controller is not connected.
*/
bool joystickConnected(void) {
  return driver_active[1] || driver_active[2] || driver_active[3] || driver_active[4] ||
         hid_driver_active[0] || hid_driver_active[1] || hid_driver_active[2] || hid_driver_active[3];
}

/*
Purpose:
Clears accumulated PID state for depth hold.
Inputs:
None.
Outputs:
Resets integral, previous error, and current PID output globals.
Side Effects:
Depth-hold controller restarts without stale accumulated correction.
Dependencies:
Called whenever manual pilot control takes priority or a new target is captured.
Notes:
Prevents integral windup from fighting the pilot after mode changes.
*/
void resetDepthHoldPid(void) {
  depthIntegral = 0.0;
  previousDepthError = 0.0;
  depthPidOutput = 0.0;
}

/*
Purpose:
Immediately exits depth hold and returns vertical control to the pilot.
Inputs:
None.
Outputs:
Clears hold-enable state and release tracking.
Side Effects:
Updates LCD mode status when hold was active.
Dependencies:
Manual joystick movement and controller disconnect both call this path.
Notes:
This function is the manual-override path for depth hold assist.
*/
void disableDepthHold(void) {
  if (depthHoldEnabled) {
    depthHoldEnabled = false;
    display_mode_status();
  }
  verticalStickReleased = false;
  resetDepthHoldPid();
}

/*
Purpose:
Runs the depth-hold assist state machine and PID correction for vertical motion.
Inputs:
Elapsed loop time in seconds and filtered depth telemetry in feet.
Outputs:
May replace analogs[RJoyY] with a bounded PID vertical command.
Side Effects:
Captures target depth, updates hold state, accumulates PID terms, and refreshes LCD status.
Dependencies:
Valid depth telemetry, connected controller, and released vertical stick.
Notes:
Pilot vertical stick movement always disables hold immediately; no packet format changes occur.
*/
void updateDepthHoldAssist(float dtSeconds) {
  // Do not allow automatic vertical output without a live controller and valid depth data.
  if (!joystickConnected() || !depthTelemetryValid) {
    disableDepthHold();
    return;
  }

  // Pilot stick movement has priority over assist mode and disables hold immediately.
  if (abs(requestedAnalogs[RJoyY]) > VERTICAL_DEADBAND) {
    disableDepthHold();
    return;
  }

  // First loop after stick release starts the activation timer but does not capture depth yet.
  if (!verticalStickReleased) {
    verticalStickReleased = true;
    verticalReleaseMillis = millis();
    resetDepthHoldPid();
    return;
  }

  // After the delay, capture the current filtered depth as the hold target.
  if (!depthHoldEnabled) {
    if (millis() - verticalReleaseMillis < DEPTH_HOLD_ACTIVATION_DELAY_MS) return;
    targetDepthMeters = filteredDepthMeters;
    depthHoldEnabled = true;
    resetDepthHoldPid();
    display_mode_status();
  }

  // Signed error preserves whether the ROV is above or below the target depth.
  float error = targetDepthMeters - filteredDepthMeters;
  if (abs(error) < DEPTH_HOLD_DEADBAND) error = 0.0;
  depthIntegral += error * dtSeconds;
  float derivative = 0.0;
  if (dtSeconds > 0.0) derivative = (error - previousDepthError) / dtSeconds;
  depthPidOutput = error * DEPTH_KP + depthIntegral * DEPTH_KI + derivative * DEPTH_KD;
  // Limit automatic vertical command so depth hold cannot demand full thrust.
  depthPidOutput = constrainFloat(depthPidOutput, -MAX_VERTICAL_PID_OUTPUT, MAX_VERTICAL_PID_OUTPUT);
  previousDepthError = error;
  analogs[RJoyY] = depthPidOutput;
}
// Read gamepad input, update pilot command state, and apply assist-mode command shaping.
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
  // read the xbox controller over USB (use connected, not available(), so held D-pad ramps every loop)
    if ((bool)joysticks[joystick_index]) {
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
  
      // Convert raw Xbox stick axes into requested normalized commands; motion axes are slewed later.
      updateSlowModeToggle();
      requestedAnalogs[LJoyX] = joyScale(axes[0]);
      requestedAnalogs[LJoyY] = joyScale(axes[1]);
      requestedAnalogs[RJoyX] = joyScale(axes[2]);
      requestedAnalogs[RJoyY] = joyScale(axes[5]);
      analogs[LTrig] = trigScale(axes[3]);
      analogs[RTrig] = trigScale(axes[4]);
  
      // D-pad -> LED (DPadX) and camera tilt (DPadY); face buttons -> ButsX/ButsY
      updateVirtualPadAxes();
      // Preserve existing switch packet behavior for the Y/B button channels.
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
  updateDepthHoldAssist(dtSeconds);
}

// Convert shaped controller commands into motor and servo command bytes.
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
  servos[2] = Pwm0 + lims((int)(Gripper  *motDirs[GripperMot]*motScale));
}


// Build the RS-485 command packet without changing the existing protocol field order.
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
  lcd.print("SLOW:OFF HOLD:OFF  ");
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
Runs the top-side operator control loop.
Inputs:
USB controller state and RS-485 telemetry bytes from the ROV.
Outputs:
RS-485 command packets, Serial debug messages, and LCD telemetry/status updates.
Side Effects:
Updates command arrays, transmits commands over Serial1, and refreshes LCD when telemetry arrives.
Dependencies:
myusb.Task(), PrintDeviceListChanges(), command generation, and telemetry parsing must run regularly.
Notes:
Manual control, slow mode, slew limiting, and depth hold are evaluated before packet generation.
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

	     
