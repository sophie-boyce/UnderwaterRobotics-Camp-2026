/*==============================================================================
 * Top_Code_Poseidon_2025 — ROVotron topside (production)
 *
 * Xbox gamepad → thruster mix → RS-485 ASCII command @ 20 Hz
 * Telemetry V... reply → LCD + USB debug print
 *
 * Packet: M(6 motors) P(2 servos) C(checksum) \\n
 * (C) David Forbes — revision notes in file header below
 *==============================================================================*/

/*
 * RVTOP-Ccode / Poseidon 2025 — historical header
 * Rev 1.21 — Y button tied to switch channel for temp readout enable
 */

//==============================================================================
// 1. INCLUDES
//==============================================================================

#include <LiquidCrystalFast.h>
#include "USBHost_t36.h"

extern "C" uint32_t set_arm_clock(uint32_t frequency);

//==============================================================================
// 2. PIN AND HARDWARE CONSTANTS
//==============================================================================

#define LCD_RS 5
#define LCD_RW 6
#define LCD_E  7
#define LCD_D4 24
#define LCD_D5 25
#define LCD_D6 26
#define LCD_D7 27

#define BAUD_RATE 115200
#define SER_TXEN 2

#define CPU_CLOCK_LIMIT_HZ 100000000UL
#define USB_BOOT_DELAY_MS 100
#define LCD_SPLASH_DELAY_MS 1500

//==============================================================================
// 3. PROTOCOL AND CONTROL CONSTANTS
//==============================================================================

#define NMOTORS 6
#define NSERVOS 2
#define NSWITCHES 2
#define NVOLTS 6
#define MOTOR_DIGITS 2
#define SERVO_DIGITS 2
#define VOLTS_DIGITS 3

#define DEADBAND 3000
#define Pwm0 128

#define MOTOR_CMD_LIMIT 127
#define MOTOR_CMD_SCALE 128.0f
#define JOY_SCALE_DIVISOR 256
#define TRIGGER_SCALE_DIVISOR 1024
#define JOY_FULL_SCALE 32768.0f
#define ADC_COUNTS_MAX 4096.0f
#define ADC_VOLTS_FULLSCALE 3.3f

#define LOOP_PERIOD_US 50000UL
#define DPAD_AXIS_STEP 0.03f

#define FORWARD_SLEW_RATE 2.5f
#define TURN_SLEW_RATE 3.0f
#define VERTICAL_SLEW_RATE 2.0f
#define STRAFE_SLEW_RATE 2.5f

#define RECEIVE_SYNC_TIMEOUT_LOOPS 10000
#define RECEIVE_BODY_TIMEOUT_LOOPS 100
#define RECEIVE_POLL_DELAY_US 50

#define MSG_BUFFER_LEN 100

//==============================================================================
// 4. THRUSTER INDEX AND TRIM (competition tuning — change only after pool test)
//==============================================================================

#define GripperMot 0
#define LUpDownMot 1
#define RUpDownMot 2
#define LForAftMot 3
#define RForAftMot 4
#define StrafeMot 5

#define GripperMotDir 1.
#define LUpDownMotDir 1.
#define RUpDownMotDir -1.
#define LForAftMotDir -1.
#define RForAftMotDir 1.
#define StrafeMotDir 1.

float motDirs[NMOTORS];

float DriveGainL = 0.50;
float DriveGainR = 0.60;
float SteerGain = 0.60;
float DiveGainL = 0.60;
float DiveGainR = 0.60;
float StrafeGain = 0.50;
float GripperGain = 0.50;

//==============================================================================
// 5. PERIPHERALS — LCD AND USB HOST
//==============================================================================

LiquidCrystalFast lcd(LCD_RS, LCD_RW, LCD_E, LCD_D4, LCD_D5, LCD_D6, LCD_D7);

USBHost myusb;
USBHub hub1(myusb);
USBHIDParser hid1(myusb);

#define COUNT_JOYSTICKS 4
JoystickController joysticks[COUNT_JOYSTICKS] = {
  JoystickController(myusb), JoystickController(myusb),
  JoystickController(myusb), JoystickController(myusb)
};

USBDriver *drivers[] = {&hub1, &joysticks[0], &joysticks[1], &joysticks[2], &joysticks[3], &hid1};
#define CNT_DEVICES (sizeof(drivers) / sizeof(drivers[0]))
const char *driver_names[CNT_DEVICES] = {"Hub1", "joystick[0D]", "joystick[1D]", "joystick[2D]", "joystick[3D]", "HID1"};
bool driver_active[CNT_DEVICES] = {false, false, false, false};

USBHIDInput *hiddrivers[] = {&joysticks[0], &joysticks[1], &joysticks[2], &joysticks[3]};
#define CNT_HIDDEVICES (sizeof(hiddrivers) / sizeof(hiddrivers[0]))
const char *hid_driver_names[CNT_DEVICES] = {"joystick[0H]", "joystick[1H]", "joystick[2H]", "joystick[3H]"};
bool hid_driver_active[CNT_DEVICES] = {false};

char namebuf[MSG_BUFFER_LEN] = {0};
bool gamepadConnected = false;
bool gamepadReady = false;

// DEPRECATED — retained for reference; not used in current production build.
int user_axis[64];
uint32_t buttons_prev = 0;

//==============================================================================
// 6. GLOBAL STATE — TIMING AND TELEMETRY RX
//==============================================================================

unsigned long loopPeriod = LOOP_PERIOD_US;
unsigned long nextLoopMicros = 0;
unsigned long nextDebugMicros = 0;
unsigned long lastSlewMicros = 0;

char inString[MSG_BUFFER_LEN];
char inChr = 0;
char *inPtr;
bool gotInString = false;

//==============================================================================
// 7. GLOBAL STATE — GAMEPAD AND NORMALIZED CONTROLS
//==============================================================================

#define NANALOG 10
#define NBUTTONS 16

uint32_t butts;
unsigned char buttons[NBUTTONS];
int axes[6];

int PairButton = 1, MenuButton = 2, ViewButton = 3, AButton = 4, BButton = 5;
int XButton = 6, YButton = 7, DpadUp = 8, DpadDown = 9, DpadLeft = 10, DpadRight = 11;
int LButton = 12, RButton = 13, LJButton = 14, RJButton = 15;

int LJoyX = 0, LJoyY = 1, RJoyX = 2, RJoyY = 3, LTrig = 4, RTrig = 5;
int DPadX = 6, DPadY = 7, ButsX = 8, ButsY = 9;

float analogs[NANALOG];
float requestedAnalogs[NANALOG];
int up_button[NANALOG] = {0, 0, 0, 0, 0, 0, DpadRight, DpadUp, BButton, YButton};
int dn_button[NANALOG] = {0, 0, 0, 0, 0, 0, DpadLeft, DpadDown, XButton, AButton};
float button_inc = DPAD_AXIS_STEP;

//==============================================================================
// 8. GLOBAL STATE — COMMAND BYTES AND TELEMETRY
//==============================================================================

int motors[NMOTORS];
int servos[NSERVOS];
int switches[NSWITCHES];

char command_msg[MSG_BUFFER_LEN];
char reply_msg[MSG_BUFFER_LEN];
char buf[MSG_BUFFER_LEN];

float volts[NVOLTS];
float telems[NVOLTS];

float telZeros[NVOLTS] = {0.00, 0.00, 0.10, 1.40, 1.40, 0.00};
float telScale[NVOLTS] = {11.00, 256. / 3.3, 8.00, 70.00, 71.00, 90};

float motScale = MOTOR_CMD_SCALE;

//==============================================================================
// 9. DEBUG OUTPUT
//==============================================================================

void print_controller_debug(void) {
  Serial.printf(
    "LX:%6.3f LY:%6.3f RX:%6.3f RY:%6.3f LT:%5.3f RT:%5.3f BTN:%04lX "
    "M:%02X %02X %02X %02X %02X %02X P:%02X %02X\n",
    analogs[LJoyX], analogs[LJoyY], analogs[RJoyX], analogs[RJoyY],
    analogs[LTrig], analogs[RTrig], butts,
    motors[0], motors[1], motors[2], motors[3], motors[4], motors[5],
    servos[0], servos[1]
  );
}

//==============================================================================
// 10. LCD — TELEMETRY DISPLAY
//==============================================================================

/*
 * Purpose:
 *   Show battery, depth, LED temp, and water temp on the 4x20 operator LCD.
 * Inputs:
 *   volts[] (scaled via telZeros/telScale), buf[], lcd.
 * Outputs:
 *   LCD lines 0–3 updated.
 * Dependencies:
 *   parse_reply_msg() must have filled volts[] from the latest V... packet.
 * Notes:
 *   Depth uses telem index 5 (pressure/depth channel). H2O temp uses index 1.
 */
void display_all_telems(void) {
  float telems[NVOLTS];
  for (int i = 0; i < NVOLTS; i++) {
    telems[i] = (volts[i] - telZeros[i]) * telScale[i];
  }
  sprintf(buf, "Battery  %5.1f V", telems[0]);
  lcd.setCursor(0, 0);
  lcd.print(buf);
  sprintf(buf, "Depth    %5.2f Feet", telems[5]);
  lcd.setCursor(0, 1);
  lcd.print(buf);
  sprintf(buf, "LED temp %5.1f C", telems[3]);
  lcd.setCursor(0, 2);
  lcd.print(buf);
  sprintf(buf, "H2O temp %5.1f C", telems[1]);
  lcd.setCursor(0, 3);
  lcd.print(buf);
}

//==============================================================================
// 10. COMMUNICATION — LEGACY BLOCKING RX (UNUSED)
//==============================================================================

// DEPRECATED — NOT USED IN CURRENT BUILD
// Main loop assembles reply_msg on Serial1 non-blocking; this blocking helper remains for reference.

/*
 * Purpose:
 *   (Legacy) Block until one RS-485 message is read starting with first_char.
 * Inputs:
 *   msg buffer, first_char sync byte (e.g. 'V').
 * Outputs:
 *   0 success, 1 timeout.
 * Dependencies:
 *   Serial1, RECEIVE_*_TIMEOUT_LOOPS constants.
 * Notes:
 *   Not called from loop(); do not use in production without retesting timing.
 */
int receive_msg(char *msg, char first_char) {
  int timer;
  do {
    timer = RECEIVE_SYNC_TIMEOUT_LOOPS;
    while (!Serial1.available()) {
      delayMicroseconds(RECEIVE_POLL_DELAY_US);
      if (!--timer) return 1;
    }
  } while ((*msg = Serial1.read()) != first_char);
  msg++;
  do {
    timer = RECEIVE_BODY_TIMEOUT_LOOPS;
    while (!Serial1.available()) {
      delayMicroseconds(RECEIVE_POLL_DELAY_US);
      if (!--timer) return 1;
    }
  } while ((*msg++ = Serial1.read()) != '\0');
  return 0;
}

//==============================================================================
// 11. COMMUNICATION — HEX PARSING AND TELEMETRY DECODE
//==============================================================================

/*
 * Purpose:
 *   Convert one ASCII hex digit to 0–15.
 * Inputs:
 *   chr — hex character.
 * Outputs:
 *   nibble value, or -1 if invalid.
 */
char atoxdigit(char chr) {
  if (('a' <= chr) && (chr <= 'f')) return (chr - 'a' + 10);
  if (('A' <= chr) && (chr <= 'F')) return (chr - 'A' + 10);
  if (('0' <= chr) && (chr <= '9')) return (chr - '0');
  return -1;
}

/*
 * Purpose:
 *   Parse n consecutive hex digits from a string.
 * Inputs:
 *   str, digit count n.
 * Outputs:
 *   Integer value, or -4 if any digit invalid.
 */
int atox(char *str, char n) {
  char digit, i;
  int val = 0;
  for (i = 0; i < n; i++) {
    if ((digit = atoxdigit(*str++)) == -1) return -4;
    val = (val << 4) + digit;
  }
  return val;
}

/*
 * Purpose:
 *   Parse bottom-side V... telemetry line into volts[] (0.0–3.3 V per channel).
 * Inputs:
 *   msg — null-terminated buffer beginning with 'V'.
 * Outputs:
 *   0 OK, 1 error; updates volts[].
 * Dependencies:
 *   RS-485 reply from bottomCODEEE.
 * Notes:
 *   Error check uses (val == -1); atox returns -4 on bad hex (legacy behavior preserved).
 */
int parse_reply_msg(char *msg) {
  int i, val;
  if (*msg++ != 'V') return 1;
  for (i = 0; i < NVOLTS; i++) {
    if ((val = atox(msg, VOLTS_DIGITS)) == -1) return 1;
    msg += VOLTS_DIGITS;
    volts[i] = (float)(val) * ADC_VOLTS_FULLSCALE / ADC_COUNTS_MAX;
  }
  if (*msg++ != '\n') return 1;
  if (*msg++ != '\0') return 1;
  return 0;
}

//==============================================================================
// 12. CONTROLLER INPUT — GAMEPAD TO NORMALIZED ANALOGS
//==============================================================================

/*
 * Purpose:
 *   Apply joystick deadband and scale to ±1.0 for thruster mixing math.
 * Inputs:
 *   stick — raw 16-bit axis from USBHost.
 * Outputs:
 *   Normalized float command contribution.
 */
float joyScale(int stick) {
  int val = 0;
  if (stick > DEADBAND) val = (stick - DEADBAND) / JOY_SCALE_DIVISOR;
  if (stick < -DEADBAND) val = (stick + DEADBAND) / JOY_SCALE_DIVISOR;
  return (float)(val) * JOY_FULL_SCALE / (JOY_FULL_SCALE - DEADBAND) / MOTOR_CMD_SCALE;
}

/*
 * Purpose:
 *   Scale trigger axis (0..1023) to 0..1 float for gripper mix.
 * Inputs:
 *   stick — raw trigger value.
 * Outputs:
 *   Normalized trigger value.
 */
float trigScale(int stick) {
  return (float)(stick) / TRIGGER_SCALE_DIVISOR;
}

/*
 * Purpose:
 *   Limit how quickly one normalized command can move toward a target.
 * Inputs:
 *   current, target — normalized axis values; ratePerSecond — max units/sec; dtSeconds.
 * Outputs:
 *   Rate-limited command value.
 */
float limitRateOfChange(float current, float target, float ratePerSecond, float dtSeconds) {
  float maxStep = ratePerSecond * dtSeconds;
  float delta = target - current;
  if (delta > maxStep) return current + maxStep;
  if (delta < -maxStep) return current - maxStep;
  return target;
}

/*
 * Purpose:
 *   Slew-limit pilot motion axes before thruster mixing.
 * Notes:
 *   Camera tilt, LED brightness, and gripper remain direct for task precision.
 */
void applySlewRate(float dtSeconds) {
  analogs[LJoyY] = limitRateOfChange(analogs[LJoyY], requestedAnalogs[LJoyY], FORWARD_SLEW_RATE, dtSeconds);
  analogs[RJoyX] = limitRateOfChange(analogs[RJoyX], requestedAnalogs[RJoyX], TURN_SLEW_RATE, dtSeconds);
  analogs[RJoyY] = limitRateOfChange(analogs[RJoyY], requestedAnalogs[RJoyY], VERTICAL_SLEW_RATE, dtSeconds);
  analogs[LJoyX] = limitRateOfChange(analogs[LJoyX], requestedAnalogs[LJoyX], STRAFE_SLEW_RATE, dtSeconds);
}

/*
 * Purpose:
 *   Read Xbox controller USB data into analogs[] and switches[].
 * Inputs:
 *   joysticks[], up_button/dn_button tables, button_inc.
 * Outputs:
 *   analogs[] in ±1.0; switches[0]=Y, switches[1]=B retained for local state.
 * Dependencies:
 *   USBHost Task() in loop().
 * Notes:
 *   D-pad synthesized axes drive LED (DPadX) and camera tilt (DPadY) after mixing stage.
 */
void translate_response_to_controls() {
  bool sawGamepadFrame = false;
  unsigned long nowMicros = micros();
  float dtSeconds = loopPeriod / 1000000.0f;
  if (lastSlewMicros != 0) {
    dtSeconds = (nowMicros - lastSlewMicros) / 1000000.0f;
  }
  if (dtSeconds > 0.10f) dtSeconds = 0.10f;
  lastSlewMicros = nowMicros;

  for (int joystick_index = 0; joystick_index < COUNT_JOYSTICKS; joystick_index++) {
    if (joysticks[joystick_index].available()) {
      sawGamepadFrame = true;
      butts = joysticks[joystick_index].getButtons();
      for (uint8_t i = 0; i < 6; i++) {
        axes[i] = joysticks[joystick_index].getAxis(i);
      }
      for (int i = 0; i < 16; i++) {
        buttons[i] = (butts >> i) & 0x0001;
      }
      requestedAnalogs[LJoyX] = joyScale(axes[0]);
      requestedAnalogs[LJoyY] = joyScale(axes[1]);
      requestedAnalogs[RJoyX] = joyScale(axes[2]);
      requestedAnalogs[RJoyY] = joyScale(axes[5]);
      analogs[LTrig] = trigScale(axes[3]);
      analogs[RTrig] = trigScale(axes[4]);
      for (int i = 6; i < NANALOG; i++) {
        float val = analogs[i];
        if (buttons[up_button[i]]) val += button_inc;
        if (buttons[dn_button[i]]) val -= button_inc;
        if (val > 1.) val = 1.;
        if (val < -1.) val = -1.;
        analogs[i] = val;
      }
      switches[0] = buttons[YButton];
      switches[1] = buttons[BButton];
    }
  }
  if (sawGamepadFrame) {
    applySlewRate(dtSeconds);
  }
  if (sawGamepadFrame && !gamepadReady) {
    gamepadReady = true;
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Gamepad Ready");
    lcd.setCursor(0, 1);
    lcd.print(namebuf);
    Serial.println("Gamepad Ready");
  }
}

//==============================================================================
// 13. COMMAND GENERATION — THRUSTER MIXING AND PACKET BUILD
//==============================================================================

/*
 * Purpose:
 *   Limit per-motor offset so two-digit hex field cannot overflow.
 * Inputs:
 *   val — signed offset from Pwm0 neutral.
 * Outputs:
 *   Clamped val in ±MOTOR_CMD_LIMIT.
 */
int lims(int val) {
  int res = val;
  if (res > MOTOR_CMD_LIMIT) res = MOTOR_CMD_LIMIT;
  if (res < -MOTOR_CMD_LIMIT) res = -MOTOR_CMD_LIMIT;
  return res;
}

/*
 * Purpose:
 *   Mix normalized sticks into motor[] and servo[] command bytes (128 = stop).
 * Inputs:
 *   analogs[], trim gains, motDirs[], GripperGain.
 * Outputs:
 *   motors[], servos[] — consumed by build_command_msg().
 * Dependencies:
 *   translate_response_to_controls() must run first each frame.
 * Notes:
 *   Left Y + right X = differential fore/aft; left X = strafe; right Y = heave;
 *   triggers = gripper. D-pad axes → LED brightness and camera tilt servos.
 */
void translate_controls_to_commands() {
  float LRForeAft = analogs[LJoyY] * DriveGainL + analogs[RJoyX] * SteerGain;
  float RRForeAft = analogs[LJoyY] * DriveGainR - analogs[RJoyX] * SteerGain;
  float Strafe = analogs[LJoyX] * StrafeGain;
  float LUpDown = analogs[RJoyY] * DiveGainL;
  float RUpDown = analogs[RJoyY] * DiveGainR;
  float Gripper = (analogs[RTrig] - analogs[LTrig]) * GripperGain;

  motors[GripperMot] = Pwm0 + lims((int)(Gripper * motDirs[GripperMot] * motScale));
  motors[LUpDownMot] = Pwm0 + lims((int)(LUpDown * motDirs[LUpDownMot] * motScale));
  motors[RUpDownMot] = Pwm0 + lims((int)(RUpDown * motDirs[RUpDownMot] * motScale));
  motors[LForAftMot] = Pwm0 + lims((int)(LRForeAft * motDirs[LForAftMot] * motScale));
  motors[RForAftMot] = Pwm0 + lims((int)(RRForeAft * motDirs[RForAftMot] * motScale));
  motors[StrafeMot] = Pwm0 + lims((int)(Strafe * motDirs[StrafeMot] * motScale));

  servos[0] = Pwm0 + lims((int)(analogs[DPadX] * motScale));
  servos[1] = Pwm0 + lims((int)(-analogs[DPadY] * motScale));
}

/*
 * Purpose:
 *   Build RS-485 ASCII command: M + P + C + newline.
 * Inputs:
 *   motors[], servos[].
 * Outputs:
 *   msg buffer; checksum is low 8 bits of sum of all payload bytes.
 * Dependencies:
 *   Serial1.print in loop() sends this to bottom Teensy.
 * Notes:
 *   Matches current bottomCODEEE parser.
 */
void build_command_msg(char *msg) {
  int i, checksum = 0;
  *msg++ = 'M';
  for (i = 0; i < NMOTORS; i++) {
    checksum += motors[i];
    msg += sprintf(msg, "%02X", motors[i]);
  }
  *msg++ = 'P';
  for (i = 0; i < NSERVOS; i++) {
    checksum += servos[i];
    msg += sprintf(msg, "%02X", servos[i]);
  }
  *msg++ = 'C';
  checksum = checksum % 0x100;
  msg += sprintf(msg, "%02X", checksum);
  *msg++ = '\n';
  *msg = '\0';
}

//==============================================================================
// 14. USB DEVICE ENUMERATION
//==============================================================================

/*
 * Purpose:
 *   Log USB connect/disconnect on Serial and show gamepad USB ID on LCD line 2.
 * Inputs:
 *   USBHost driver_active[] state.
 * Outputs:
 *   Serial.printf lines; optional LCD Dev ID string.
 */
void PrintDeviceListChanges() {
  for (uint8_t i = 0; i < CNT_DEVICES; i++) {
    if (*drivers[i] != driver_active[i]) {
      if (driver_active[i]) {
        Serial.printf("*** Device %s - disconnected ***\n", driver_names[i]);
        driver_active[i] = false;
      } else {
        Serial.printf("*** Device %s %x:%x - connected ***\n", driver_names[i],
          drivers[i]->idVendor(), drivers[i]->idProduct());
        sprintf(namebuf, "Dev ID %04X:%04X", drivers[i]->idVendor(), drivers[i]->idProduct());
        if (drivers[i] == &joysticks[0] || drivers[i] == &joysticks[1] ||
            drivers[i] == &joysticks[2] || drivers[i] == &joysticks[3]) {
          gamepadConnected = true;
          lcd.clear();
          lcd.setCursor(0, 0);
          lcd.print("Gamepad Connected");
        }
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
        Serial.printf("*** HID Device %s %x:%x - connected ***\n", hid_driver_names[i],
          hiddrivers[i]->idVendor(), hiddrivers[i]->idProduct());
        sprintf(namebuf, "Dev ID %04X:%04X", hiddrivers[i]->idVendor(), hiddrivers[i]->idProduct());
        gamepadConnected = true;
        lcd.clear();
        lcd.setCursor(0, 0);
        lcd.print("Gamepad Connected");
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

//==============================================================================
// 15. SETUP
//==============================================================================

void setup() {
  if (F_CPU_ACTUAL >= CPU_CLOCK_LIMIT_HZ) {
    set_arm_clock(CPU_CLOCK_LIMIT_HZ);
  }

  motDirs[GripperMot] = GripperMotDir;
  motDirs[LUpDownMot] = LUpDownMotDir;
  motDirs[RUpDownMot] = RUpDownMotDir;
  motDirs[LForAftMot] = LForAftMotDir;
  motDirs[RForAftMot] = RForAftMotDir;
  motDirs[StrafeMot] = StrafeMotDir;

  Serial.begin(BAUD_RATE);
  delay(USB_BOOT_DELAY_MS);
  Serial.println("Starting Teensy");

  Serial1.begin(BAUD_RATE);
  Serial1.transmitterEnable(SER_TXEN);
  Serial.println("Starting LCD");

  lcd.begin(20, 4);
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("   ROVotron Cadet  ");
  lcd.setCursor(0, 1);
  lcd.print(" ROV Control System");
  lcd.setCursor(0, 3);
  lcd.print("     Rev. 1.21");
  delay(LCD_SPLASH_DELAY_MS);
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Seeking Gamepad");

  myusb.begin();

  for (int i = 0; i < NMOTORS; i++) motors[i] = Pwm0;
  for (int i = 0; i < NSERVOS; i++) servos[i] = Pwm0;
  inPtr = reply_msg;
  gotInString = false;
}

//==============================================================================
// 16. MAIN LOOP
//==============================================================================

void loop() {
  myusb.Task();
  PrintDeviceListChanges();

  if (Serial1.available()) {
    int inChr = Serial1.read();
    if (inChr) *inPtr++ = inChr;
    if (inChr == '\n') {
      *inPtr++ = 0x00;
      gotInString = true;
      inPtr = reply_msg;
    }
  }

  if (micros() > nextLoopMicros) {
    nextLoopMicros += loopPeriod;
    translate_response_to_controls();
    translate_controls_to_commands();
    build_command_msg(command_msg);
    Serial1.print(command_msg);
    if (gamepadReady && micros() > nextDebugMicros) {
      nextDebugMicros = micros() + 250000UL;
      print_controller_debug();
    }
  }

  if (gotInString) {
    Serial.print(reply_msg);
    gotInString = false;
    parse_reply_msg(reply_msg);
    display_all_telems();
  }
}
