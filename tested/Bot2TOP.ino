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
#include <math.h>
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
#define TOP_DEBUG_SERIAL 0

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

// This value is written to a PWM device for center (zero) output
#define Pwm0  128



#define SLOW_MODE_SCALE 0.4
#define ACCEL_RATE_NORMAL 2.5
#define DECEL_RATE_NORMAL 6.0
#define ACCEL_RATE_SLOW 1.0
#define DECEL_RATE_SLOW 3.0
// --- Deadzones / deadbands (see docs below) ---
#define DEADBAND 3000              // Gamepad stick raw units (~9% travel) before axis responds
#define VERTICAL_DEADBAND 0.05f    // Right-stick Y: hold disables above 5% command
#define DEPTH_HOLD_DEADBAND 0.35f  // PID ignores |error| below this (feet)
#define DEPTH_HOLD_ERROR_BLEND 0.20f // Soft ramp above deadband (feet) — avoids hard on/off twitch
#define DEPTH_RATE_DEADBAND 0.08f  // Ignore tiny depth-rate noise for D-term (ft/s)
#define DEPTH_HOLD_ACTIVATION_DELAY_MS 750
#define MAX_VERTICAL_PID_OUTPUT 0.18f
#define DEPTH_KP 0.20f
#define DEPTH_KI 0.005f
#define DEPTH_KD 0.14f
#define DEPTH_VOLT_FILTER_ALPHA 0.12f  // EMA on pressure voltage before feet conversion
#define DEPTH_FILTER_ALPHA 0.07f       // EMA on depth feet (after conversion)
#define DEPTH_RATE_FILTER_ALPHA 0.25f  // EMA on depth rate for D-term
#define DEPTH_PID_SLEW_RATE 1.0f       // Max hold-output change per second
#define TELEMETRY_TIMEOUT_MS 1000
// Depth sensor: feet = (pressureVolts - DEPTH_SURFACE_VOLT) * DEPTH_FEET_PER_VOLT
// Calibrate DEPTH_SURFACE_VOLT at the surface; tune DEPTH_FEET_PER_VOLT at a known depth.
#define DEPTH_SURFACE_VOLT 0.0f
#define DEPTH_FEET_PER_VOLT 90.0f
#define DEPTH_DISPLAY_OFFSET_FT 42.7f   // LCD zero at surface (was -1.3 ft with 44.0)
#define DEPTH_PID_OUTPUT_SIGN (-1.0f)
#define DEPTH_INTEGRAL_MAX 1.0f   // Anti-windup clamp for depth-hold integral term.
#define DPAD_AXIS_STEP 0.03f
#define CAMERA_DPAD_STEP 0.12f
#define CAMERA_DPAD_SNAP 1
#define DEPTH_PRESSURE_V_MIN 0.05f
#define DEPTH_PRESSURE_V_MAX 3.25f
#define DEPTH_FEET_MIN -2.0f
#define DEPTH_FEET_MAX 250.0f
#define CAMERA_TILT_ENABLED 0
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
float DriveGainL  = 0.60;
float DriveGainR  = 0.60;
float SteerGain   = -0.60;   // must match this ROV wiring (was +0.60 — broke turning)
float DiveGainL   = 0.60;
float DiveGainR   = 0.60;
float StrafeGain  = 0.58;
#define DRIVE_PITCH_TRIM (-0.12f)
#define GRIPPER_TRIGGER_ON 0.18f   // ignore trigger noise below this
#define GRIPPER_STEP 0.07f         // move/sec factor per loop (~20 Hz) at full trigger

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

//------------ Global Variables ------------------------------- //

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
// D-pad maps to DPadX/DPadY via readDpadDirections(); face buttons use up_button/dn_button below.
int up_button[NANALOG] = {0,0,0,0,0,0,0,0,  BButton,YButton};
int dn_button[NANALOG] = {0,0,0,0,0,0,0,0,XButton,AButton};

// motion step of synthesized axis per main loop iteration
float button_inc = DPAD_AXIS_STEP;

static int activeGamepadIndex = -1;
static float gripperCmd = 0.0f;
static float depthHoldOutput = 0.0f;

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
unsigned long lastTelemetryMillis = 0;
float currentDepthFeet = 0.0;
float filteredDepthFeet = 0.0;
float targetDepthFeet = 0.0;
float previousDepthFeet = 0.0;
float filteredPressureVolts = 0.0;
float filteredDepthRate = 0.0;
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

float depthFeetFromPressureVolts(float pressureVolts) {
  return (pressureVolts - DEPTH_SURFACE_VOLT) * DEPTH_FEET_PER_VOLT;
}

float applyDepthErrorDeadband(float errorFeet) {
  float magnitude = fabsf(errorFeet);
  if (magnitude <= DEPTH_HOLD_DEADBAND) return 0.0f;
  if (magnitude >= DEPTH_HOLD_DEADBAND + DEPTH_HOLD_ERROR_BLEND) return errorFeet;
  float blend = (magnitude - DEPTH_HOLD_DEADBAND) / DEPTH_HOLD_ERROR_BLEND;
  return (errorFeet >= 0.0f ? 1.0f : -1.0f) * blend * magnitude;
}

void display_slow_mode_line(void) {
  sprintf(buf, "Slow Mode:%3s     ", slow_mode_enabled ? "ON" : "OFF");
  lcd.setCursor(0, 0);
  lcd.print(buf);
}

void display_pid_hold_line(void) {
  sprintf(buf, "PID:%+5.2f ft %3s", targetDepthFeet - filteredDepthFeet,
          depthHoldEnabled ? "ON" : "OFF");
  lcd.setCursor(0, 3);
  lcd.print(buf);
  lcd.print("   ");
}

void display_all_telems(void) {
  float telems[NVOLTS];
  float depthDisplayFt;
  for (int i=0;i<NVOLTS;i++) {
    telems[i] = (volts[i] - telZeros[i]) * telScale[i];
  }
  depthDisplayFt = filteredDepthFeet - DEPTH_DISPLAY_OFFSET_FT;
  display_slow_mode_line();
  sprintf(buf, "Battery  %5.1f V", telems[0]);
  lcd.setCursor(0, 1);
  lcd.print(buf);
  lcd.print("   ");
  sprintf(buf, "Depth    %5.2f Feet", depthDisplayFt);
  lcd.setCursor(0, 2);
  lcd.print(buf);
  lcd.print("   ");
  display_pid_hold_line();
}

// ------------------- ROV message I/O ------------------ //

// !!! This needs to be redone in a non-blocking manner. 

// get a message from ROV serial port
// returns 0 if OK, 1 if timeout error
// The timer waits a fraction of a second for a valid message. 
// it throws away chars until M seen, to sync up to start of real message
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

// -------------------- Parsing routines ---------------------- //

// convert a character from ASCII to hex
// returns -1 if invalid hex character
char atoxdigit(char chr) {
  if (('a' <= chr) && (chr <= 'f')) return (chr - 'a' + 10);
  if (('A' <= chr) && (chr <= 'F')) return (chr - 'A' + 10);
  if (('0' <= chr) && (chr <= '9')) return (chr - '0');
  return -1;
}

// get an n digit positive hex number into an int from string
// returns -1 if invalid hex characters
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

bool depthTelemetrySampleValid(float depthFeet, float pressureVolts) {
  if (pressureVolts < DEPTH_PRESSURE_V_MIN || pressureVolts > DEPTH_PRESSURE_V_MAX) return false;
  if (depthFeet < DEPTH_FEET_MIN || depthFeet > DEPTH_FEET_MAX) return false;
  return true;
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
    if ((val = atox(msg, VOLTS_DIGITS)) < 0) return 1;
    msg += VOLTS_DIGITS;
    volts[i] = (float)(val) * 3.3/4096.;  // save it
  }

  if (*msg++ != '\n') return 1; 
  if (*msg++ != '\0') return 1; 
  currentDepthFeet = depthFeetFromPressureVolts(volts[5]);
  if (depthTelemetrySampleValid(currentDepthFeet, volts[5])) {
    if (!depthTelemetryValid) {
      filteredPressureVolts = volts[5];
      filteredDepthFeet = currentDepthFeet;
      previousDepthFeet = currentDepthFeet;
      filteredDepthRate = 0.0f;
    } else {
      filteredPressureVolts = filteredPressureVolts * (1.0f - DEPTH_VOLT_FILTER_ALPHA)
                            + volts[5] * DEPTH_VOLT_FILTER_ALPHA;
      float depthFromFilteredV = depthFeetFromPressureVolts(filteredPressureVolts);
      filteredDepthFeet = filteredDepthFeet * (1.0f - DEPTH_FILTER_ALPHA)
                        + depthFromFilteredV * DEPTH_FILTER_ALPHA;
    }
    depthTelemetryValid = true;
    lastTelemetryMillis = millis();
#if TOP_DEBUG_SERIAL
    Serial.printf("DEPTH V=%.3f ft=%.2f filt=%.2f hold=%d\n",
                  volts[5], currentDepthFeet, filteredDepthFeet, depthHoldEnabled ? 1 : 0);
#endif
  } else {
    depthTelemetryValid = false;
  }
  return 0;
}

// ----------------- translation code -------------------- //

// Remove deadband slop from joysticks
// And scale joystick value from 16 bit int to float for motor math
float joyScale(int stick) {
  int val = 0;
  if (stick >  DEADBAND) val = (stick - DEADBAND)/256;
  if (stick < -DEADBAND) val = (stick + DEADBAND)/256;
  return (float)(val) * 32768. / (32768.-DEADBAND)/128.; // scale out the deadband
}

// Scale trigger value from 10 bit int to float (it's different from joysticks)
float trigScale(int stick) {
  return (float)(stick)/1024.;
}

void readDpadDirections(bool &up, bool &down, bool &left, bool &right) {
  up = buttons[DpadUp] || (butts & 0x10000);
  down = buttons[DpadDown] || (butts & 0x40000);
  left = buttons[DpadLeft] || (butts & 0x80000);
  right = buttons[DpadRight] || (butts & 0x20000);
  if (activeGamepadIndex >= 0 && (bool)joysticks[activeGamepadIndex]) {
    int hat = joysticks[activeGamepadIndex].getAxis(9);
    if (hat >= 0 && hat <= 7) {
      up |= (hat == 0 || hat == 1 || hat == 7);
      right |= (hat == 1 || hat == 2 || hat == 3);
      down |= (hat == 3 || hat == 4 || hat == 5);
      left |= (hat == 5 || hat == 6 || hat == 7);
    }
  }
}

// DPadX -> servos[0] LED; DPadY -> servos[1] camera tilt (via translate_controls_to_commands).
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
#if CAMERA_TILT_ENABLED
#if CAMERA_DPAD_SNAP
  if (dUp) val = 1.0f;
  else if (dDown) val = -1.0f;
#else
  if (dUp) val += CAMERA_DPAD_STEP;
  if (dDown) val -= CAMERA_DPAD_STEP;
  if (val > 1.0f) val = 1.0f;
  if (val < -1.0f) val = -1.0f;
#endif
#endif
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

int findActiveGamepadIndex(void) {
  for (int i = 0; i < COUNT_JOYSTICKS; i++) {
    if ((bool)joysticks[i] && joysticks[i].available()) return i;
  }
  for (int i = 0; i < COUNT_JOYSTICKS; i++) {
    if ((bool)joysticks[i]) return i;
  }
  return -1;
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
    display_slow_mode_line();
  }
  if (!lbRbComboPressed) {
    lbRbComboWasPressed = false;
  }
}

float constrainFloat(float val, float minVal, float maxVal) {
  if (val < minVal) return minVal;
  if (val > maxVal) return maxVal;
  return val;
}

bool joystickConnected(void) {
  return driver_active[1] || driver_active[2] || driver_active[3] || driver_active[4] ||
         hid_driver_active[0] || hid_driver_active[1] || hid_driver_active[2] || hid_driver_active[3];
}

void resetDepthHoldPid(void) {
  depthIntegral = 0.0;
  previousDepthError = 0.0;
  depthPidOutput = 0.0;
  depthHoldOutput = 0.0;
  filteredDepthRate = 0.0f;
}

void disableDepthHold(void) {
  if (depthHoldEnabled) {
    depthHoldEnabled = false;
    display_pid_hold_line();
  }
  verticalStickReleased = false;
  resetDepthHoldPid();
}

void updateDepthHoldAssist(float dtSeconds) {
  float accelRate = slow_mode_enabled ? ACCEL_RATE_SLOW : ACCEL_RATE_NORMAL;
  float decelRate = slow_mode_enabled ? DECEL_RATE_SLOW : DECEL_RATE_NORMAL;

  if (depthTelemetryValid && (millis() - lastTelemetryMillis > TELEMETRY_TIMEOUT_MS)) {
    depthTelemetryValid = false;
  }

  if (!joystickConnected() || !depthTelemetryValid) {
    disableDepthHold();
    return;
  }

  if (!depthTelemetrySampleValid(filteredDepthFeet, volts[5])) {
    disableDepthHold();
    return;
  }

  if (fabsf(requestedAnalogs[RJoyY]) > VERTICAL_DEADBAND) {
    disableDepthHold();
    return;
  }

  if (!verticalStickReleased) {
    verticalStickReleased = true;
    verticalReleaseMillis = millis();
    resetDepthHoldPid();
    return;
  }

  if (!depthHoldEnabled) {
    if (millis() - verticalReleaseMillis < DEPTH_HOLD_ACTIVATION_DELAY_MS) {
      analogs[RJoyY] = limitRateOfChange(analogs[RJoyY], 0.0f, accelRate, decelRate, dtSeconds);
      resetDepthHoldPid();
      return;
    }
    targetDepthFeet = filteredDepthFeet;
    depthHoldEnabled = true;
    resetDepthHoldPid();
    depthHoldOutput = analogs[RJoyY];
    previousDepthFeet = filteredDepthFeet;
    display_pid_hold_line();
  }

  float error = targetDepthFeet - filteredDepthFeet;
  float errorForPid = applyDepthErrorDeadband(error);

  float depthRate = 0.0f;
  if (dtSeconds > 0.0f) depthRate = (filteredDepthFeet - previousDepthFeet) / dtSeconds;
  previousDepthFeet = filteredDepthFeet;
  if (fabsf(depthRate) < DEPTH_RATE_DEADBAND) depthRate = 0.0f;
  filteredDepthRate = filteredDepthRate * (1.0f - DEPTH_RATE_FILTER_ALPHA)
                    + depthRate * DEPTH_RATE_FILTER_ALPHA;

  if (fabsf(error) > DEPTH_HOLD_DEADBAND) {
    depthIntegral += errorForPid * dtSeconds;
    depthIntegral = constrainFloat(depthIntegral, -DEPTH_INTEGRAL_MAX, DEPTH_INTEGRAL_MAX);
  }

  depthPidOutput = errorForPid * DEPTH_KP + depthIntegral * DEPTH_KI - filteredDepthRate * DEPTH_KD;
  depthPidOutput *= DEPTH_PID_OUTPUT_SIGN;
  float pidUnclamped = depthPidOutput;
  depthPidOutput = constrainFloat(depthPidOutput, -MAX_VERTICAL_PID_OUTPUT, MAX_VERTICAL_PID_OUTPUT);

  if (pidUnclamped > MAX_VERTICAL_PID_OUTPUT && errorForPid > 0.0f) {
    depthIntegral -= errorForPid * dtSeconds;
  } else if (pidUnclamped < -MAX_VERTICAL_PID_OUTPUT && errorForPid < 0.0f) {
    depthIntegral -= errorForPid * dtSeconds;
  }

  previousDepthError = errorForPid;
  depthHoldOutput = limitRateOfChange(depthHoldOutput, depthPidOutput,
                                      DEPTH_PID_SLEW_RATE, DEPTH_PID_SLEW_RATE, dtSeconds);
  analogs[RJoyY] = depthHoldOutput;
}

void applySlewRate(float dtSeconds) {
  float accelRate = slow_mode_enabled ? ACCEL_RATE_SLOW : ACCEL_RATE_NORMAL;
  float decelRate = slow_mode_enabled ? DECEL_RATE_SLOW : DECEL_RATE_NORMAL;
  float scale = slow_mode_enabled ? SLOW_MODE_SCALE : 1.0;

  analogs[LJoyY] = limitRateOfChange(analogs[LJoyY], requestedAnalogs[LJoyY] * scale, accelRate, decelRate, dtSeconds);
  analogs[LJoyX] = limitRateOfChange(analogs[LJoyX], requestedAnalogs[LJoyX] * scale, accelRate, decelRate, dtSeconds);
  analogs[RJoyX] = limitRateOfChange(analogs[RJoyX], requestedAnalogs[RJoyX] * scale, accelRate, decelRate, dtSeconds);
  if (!depthHoldEnabled) {
    analogs[RJoyY] = limitRateOfChange(analogs[RJoyY], requestedAnalogs[RJoyY] * scale, accelRate, decelRate, dtSeconds);
  }
}

// read gamepad and populate the control variables with current readings
void translate_response_to_controls() {
  unsigned long nowMicros = micros();
  float dtSeconds = loopPeriod / 1000000.0;
  if (lastSlewMicros != 0) {
    dtSeconds = (nowMicros - lastSlewMicros) / 1000000.0;
  }
  if (dtSeconds > 0.10) dtSeconds = 0.10;
  lastSlewMicros = nowMicros;
  activeGamepadIndex = -1;
  for (int joystick_index = 0; joystick_index < COUNT_JOYSTICKS; joystick_index++) {
    if (!(bool)joysticks[joystick_index] || !joysticks[joystick_index].available()) continue;
    activeGamepadIndex = joystick_index;
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
  
      updateVirtualPadAxes();
      // use X and Y buttons as on/off controls (for temp reading)
      switches[0] = buttons[YButton];
      switches[1] = buttons[BButton];
      
      if (0) {
        for (int i=0;i<NANALOG;i++) {
          Serial.printf(" %6.3f", analogs[i]);
        }
        Serial.println();
      }
    break;
  }
  if (joystickConnected()) {
    applySlewRate(dtSeconds);
  }
  updateDepthHoldAssist(dtSeconds);
}

// rescale the controls to the motors and servos
float motScale = 128.;

// make sure the numbers don't overflow the 2 digit hex range
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

void translate_controls_to_commands() {
  float LRForeAft = analogs[LJoyY]*DriveGainL + analogs[RJoyX]*SteerGain;
  float RRForeAft = analogs[LJoyY]*DriveGainR - analogs[RJoyX]*SteerGain;
  float Strafe    = analogs[LJoyX]*StrafeGain;
  float pitchTrim = analogs[LJoyY] * DRIVE_PITCH_TRIM;
  float LUpDown   = analogs[RJoyY]*DiveGainL + pitchTrim;
  float RUpDown   = analogs[RJoyY]*DiveGainR + pitchTrim;
  // Claw: RT/LT step open/close while held; position latched on release (no snap to center).
  if (analogs[RTrig] > GRIPPER_TRIGGER_ON) {
    gripperCmd += GRIPPER_STEP * analogs[RTrig];
  } else if (analogs[LTrig] > GRIPPER_TRIGGER_ON) {
    gripperCmd -= GRIPPER_STEP * analogs[LTrig];
  }
  if (gripperCmd > 1.0f) gripperCmd = 1.0f;
  if (gripperCmd < -1.0f) gripperCmd = -1.0f;
  motors[GripperMot] = Pwm0 + lims((int)(gripperCmd * motDirs[GripperMot] * 127.0f));
  motors[LUpDownMot] = Pwm0 + lims((int)(LUpDown  *motDirs[LUpDownMot]*motScale));
  motors[RUpDownMot] = Pwm0 + lims((int)(RUpDown  *motDirs[RUpDownMot]*motScale));
  motors[LForAftMot] = Pwm0 + lims((int)(LRForeAft*motDirs[LForAftMot]*motScale));
  motors[RForAftMot] = Pwm0 + lims((int)(RRForeAft*motDirs[RForAftMot]*motScale));
  motors[StrafeMot ] = Pwm0 + lims((int)(Strafe   *motDirs[StrafeMot ]*motScale));

  // servos[0]=LED dim; servos[1]=camera (disabled when CAMERA_TILT_ENABLED 0)
  servos[0] = Pwm0 + lims((int)((analogs[DPadX]) * motScale));  // LED dimming
#if CAMERA_TILT_ENABLED
  servos[1] = Pwm0 + lims((int)(-(analogs[DPadY]) * motScale));  // camera tilt
#else
  servos[1] = Pwm0;
#endif
}


// build a command string for ROV
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
  lcd.print("Slow Mode:OFF     ");
// make the joystick host USB port 
  myusb.begin();
// initialize all motors and servos to safe position
  for (int i=0;i<NMOTORS;i++) 
    motors[i] = Pwm0;
  for (int i=0;i<NSERVOS;i++) 
    servos[i] = Pwm0;
  for (int i = 0; i < NANALOG; i++) {
    analogs[i] = 0.0f;
    requestedAnalogs[i] = 0.0f;
  }
  inPtr = reply_msg;
  gotInString = false;
  nextLoopMicros = micros();
  Serial.println("TOP ready");
}

/* --------------------- main loop --------------------------- */

void loop() {    
 // unsigned long mics = micros();
 // Serial.printf("Mics %d\n", mics);
  myusb.Task();
  PrintDeviceListChanges();  // show USB connect/disconnect activity
  
  // Read the data link receive port, and act on the char that was received
  if (Serial1.available()) {
    int inChr = Serial1.read();
    if (inChr) {
      if (inPtr < reply_msg + sizeof(reply_msg) - 2) {
        *inPtr++ = inChr;
      } else {
        inPtr = reply_msg;
      }
    }
    if (inChr == '\n') {
      *inPtr++ = 0x00;
      gotInString = true;
      inPtr = reply_msg;
    }
  }
  // Parse telemetry before PID so depth hold uses the freshest sample.
  if (gotInString) {
#if TOP_DEBUG_SERIAL
    Serial.print(reply_msg);
#endif
    gotInString = false;
    parse_reply_msg(reply_msg);
    display_all_telems();
  }
  // The command message is sent on a rigid schedule
  if (micros() > nextLoopMicros) {
    nextLoopMicros += loopPeriod;
    translate_response_to_controls();
    translate_controls_to_commands();
    build_command_msg(command_msg);
    Serial1.print(command_msg);
#if TOP_DEBUG_SERIAL
    Serial.println(command_msg);
#endif
  }
}

	     
