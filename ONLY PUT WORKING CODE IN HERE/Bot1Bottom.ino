/*==============================================================================
 * bottomCODEEE — ROVotron vehicle controller (production)
 *
 * RS-485 command in → thrusters, gripper, camera, LED
 * Sensor ADC → V... telemetry reply out
 *
 * (C) David Forbes — see revision history in comments below
 *==============================================================================*/

//==============================================================================
// 1. INCLUDES
//==============================================================================

#include <Servo.h>

//==============================================================================
// 2. PROTOCOL AND TIMING CONSTANTS
//==============================================================================

#define NMOTORS 6
#define NSERVOS 2
#define NVOLTS 6

#define SERIAL_BAUD 115200
#define COMMAND_MSG_MAX (sizeof(command_msg) - 2)

#define PWM_CMD_CENTER 128
#define MOTOR_US_CENTER 1500
#define MOTOR_US_ARM 1600
#define SERVO_US_CENTER 1500
#define MOTOR_SCALE_DIVISOR 256
#define MOTOR_SCALE_SPAN_US 1000
#define SERVO_SCALE_SPAN_US 1600

#define ESC_INIT_DELAY_MS 7000
#define ESC_ARM_PULSE_MS 1000
#define GRIP_BRAKE_PWM 1

#define ADC_SMOOTH_RANGE 100
#define ADC_SMOOTH_WEIGHT 80

//==============================================================================
// 3. PIN DEFINITIONS
//==============================================================================

#define TXE_PIN 2

#define MOTOR1_PIN 3
#define MOTOR2_PIN 4
#define MOTOR3_PIN 5
#define MOTOR4_PIN 6
#define MOTOR5_PIN 7
#define GRIP1_PIN 9
#define GRIP2_PIN 10
#define SERVO1_PIN 21
// SERVO 1 is camera
// #define SERVO1_PIN 17 on robot 2

#define DIM_PIN 22

#define SDA0_PIN 18
#define SCL0_PIN 19
#define MOSI_PIN 11
#define MISO_PIN 12
#define SCK_PIN 13

#define VBATT_PIN A9
#define ANALOG1_PIN A3
#define ANALOG2_PIN A2
#define ANALOG3_PIN A1
#define ANALOG4_PIN A0
#define PRESSURE_PIN A6

//==============================================================================
// 4. GLOBAL STATE — COMMANDS, TELEMETRY, SERIAL BUFFERS
//==============================================================================

int motors[NMOTORS];
int servos[NSERVOS];
int volts[NVOLTS];

char command_msg[300];
char reply_msg[300];
char buf[300];

Servo motor1;
Servo motor2;
Servo motor3;
Servo motor4;
Servo motor5;
Servo servo1;

char *inPtr;
bool gotInString = false;

int grip, grip1, grip2 = 0;

int msg_count = 0;

// DEPRECATED — retained for reference; not used in current production build.
float depthMeters;
char inString[100];
char inChr = 0;

//==============================================================================
// 5. SENSOR PROCESSING
//==============================================================================

/*
 * Purpose:
 *   Sample all telemetry ADC channels and apply exponential smoothing.
 * Inputs:
 *   Analog pins VBATT, ANALOG1–4, PRESSURE; prior volts[].
 * Outputs:
 *   volts[] holds 12-bit counts (0–4095) for V... telemetry packet.
 * Dependencies:
 *   analogReadResolution(12) in setup().
 * Notes:
 *   Called every loop(); smoothing reduces tether noise on LCD depth/battery.
 */
void read_all_adcs(void) {
  volts[0] = (volts[0] * ADC_SMOOTH_WEIGHT + analogRead(VBATT_PIN) * (ADC_SMOOTH_RANGE - ADC_SMOOTH_WEIGHT)) / ADC_SMOOTH_RANGE;
  volts[1] = (volts[1] * ADC_SMOOTH_WEIGHT + analogRead(ANALOG1_PIN) * (ADC_SMOOTH_RANGE - ADC_SMOOTH_WEIGHT)) / ADC_SMOOTH_RANGE;
  volts[2] = (volts[2] * ADC_SMOOTH_WEIGHT + analogRead(ANALOG2_PIN) * (ADC_SMOOTH_RANGE - ADC_SMOOTH_WEIGHT)) / ADC_SMOOTH_RANGE;
  volts[3] = (volts[3] * ADC_SMOOTH_WEIGHT + analogRead(ANALOG3_PIN) * (ADC_SMOOTH_RANGE - ADC_SMOOTH_WEIGHT)) / ADC_SMOOTH_RANGE;
  volts[4] = (volts[4] * ADC_SMOOTH_WEIGHT + analogRead(ANALOG4_PIN) * (ADC_SMOOTH_RANGE - ADC_SMOOTH_WEIGHT)) / ADC_SMOOTH_RANGE;
  volts[5] = (volts[5] * ADC_SMOOTH_WEIGHT + analogRead(PRESSURE_PIN) * (ADC_SMOOTH_RANGE - ADC_SMOOTH_WEIGHT)) / ADC_SMOOTH_RANGE;
}

//==============================================================================
// 6. COMMUNICATION — COMMAND PARSING AND TELEMETRY TX
//==============================================================================

/*
 * Purpose:
 *   Convert one ASCII hex digit to 0–15.
 * Inputs:
 *   chr — hex character.
 * Outputs:
 *   Nibble value, or -1 if invalid.
 */
char atoxdigit(char chr) {
  if (('a' <= chr) && (chr <= 'f')) return (chr - 'a' + 10);
  if (('A' <= chr) && (chr <= 'F')) return (chr - 'A' + 10);
  if (('0' <= chr) && (chr <= '9')) return (chr - '0');
  return -1;
}

/*
 * Purpose:
 *   Parse n hex digits from command string.
 * Inputs:
 *   str, digit count n.
 * Outputs:
 *   Integer value, or -4 if invalid digit.
 */
int atox(char *str, char n) {
  char digit;
  int i, val = 0;
  for (i = 0; i < n; i++) {
    if ((digit = atoxdigit(*str++)) == -1) return -4;
    val = (val << 4) + digit;
  }
  return val;
}

/*
 * Purpose:
 *   Validate and decode M...P...C... command from topside (production parser).
 * Inputs:
 *   msg — null-terminated RS-485 line; mots[], sers[] output arrays.
 * Outputs:
 *   0 if OK, 1 if format/checksum field error.
 * Dependencies:
 *   Top_Code_Poseidon sends M + P + S + C; this build expects M + P + C only.
 * Notes:
 *   Checksum compare is commented out (legacy). Invalid packets skip actuator update.
 *   Top switch section (S) is not parsed here — must match paired firmware on bench.
 */
int parse_command_msg(char *msg, int *mots, int *sers) {
  int i, checksum = 0, val;
  if (*msg++ != 'M') return 1;
  for (i = 0; i < NMOTORS; i++) {
    if ((val = atox(msg, 2)) == -1) return 1;
    msg += 2;
    checksum += mots[i] = val;
  }
  if (*msg++ != 'P') return 1;
  for (i = 0; i < NSERVOS; i++) {
    if ((val = atox(msg, 2)) == -1) return 1;
    msg += 2;
    checksum += sers[i] = val;
  }
  if (*msg++ != 'C') return 1;
  if ((val = atox(msg, 2)) == -1) return 1;
  msg += 2;
  //	if (checksum != (unsigned char) val) return 1;
  if (*msg++ != '\n') return 1;
  if (*msg++ != '\0') return 1;
  return 0;
}

/*
 * Purpose:
 *   Build V... telemetry line from smoothed ADC counts for topside LCD.
 * Inputs:
 *   volts[] — six 12-bit channel values.
 * Outputs:
 *   msg buffer, newline terminated.
 * Dependencies:
 *   Serial1.print in loop() after each command line received.
 */
void build_reply_msg(char *msg) {
  int i;
  *msg++ = 'V';
  for (i = 0; i < NVOLTS; i++) {
    msg += sprintf(msg, "%03X", volts[i]);
  }
  *msg++ = '\n';
  *msg = '\0';
}

//==============================================================================
// 7. ACTUATOR CONTROL — PWM SCALING AND OUTPUTS
//==============================================================================

/*
 * Purpose:
 *   Map thruster command byte (128 neutral) to ESC pulse width in microseconds.
 * Inputs:
 *   cmd 0..255 from packet motor field.
 * Outputs:
 *   Microseconds for Servo.writeMicroseconds on thruster ESCs.
 */
int motor_scale(int cmd) {
  return (((cmd - PWM_CMD_CENTER) * MOTOR_SCALE_SPAN_US) / MOTOR_SCALE_DIVISOR) + MOTOR_US_CENTER;
}

/*
 * Purpose:
 *   Map camera tilt command byte to servo pulse width.
 * Inputs:
 *   cmd 0..255 from packet servo field [1].
 * Outputs:
 *   Microseconds for camera servo.
 */
int servo_scale(int cmd) {
  return (((cmd - PWM_CMD_CENTER) * SERVO_SCALE_SPAN_US) / MOTOR_SCALE_DIVISOR) + SERVO_US_CENTER;
}

/*
 * Purpose:
 *   BlueRobotics ESC arming sequence — brief forward pulse after long neutral hold.
 * Inputs:
 *   None.
 * Outputs:
 *   All five thruster ESCs driven to MOTOR_US_ARM.
 * Notes:
 *   Called once during setup() only.
 */
void start_all_motors(void) {
  motor1.writeMicroseconds(MOTOR_US_ARM);
  motor2.writeMicroseconds(MOTOR_US_ARM);
  motor3.writeMicroseconds(MOTOR_US_ARM);
  motor4.writeMicroseconds(MOTOR_US_ARM);
  motor5.writeMicroseconds(MOTOR_US_ARM);
}

/*
 * Purpose:
 *   Set all thruster ESCs to neutral (1500 µs).
 * Inputs:
 *   None.
 * Outputs:
 *   Five ESC neutral pulses.
 */
void stop_all_motors(void) {
  motor1.writeMicroseconds(MOTOR_US_CENTER);
  motor2.writeMicroseconds(MOTOR_US_CENTER);
  motor3.writeMicroseconds(MOTOR_US_CENTER);
  motor4.writeMicroseconds(MOTOR_US_CENTER);
  motor5.writeMicroseconds(MOTOR_US_CENTER);
}

//==============================================================================
// 8. SETUP
//==============================================================================

void setup() {
  Serial.begin(SERIAL_BAUD);
  Serial1.begin(SERIAL_BAUD);
  Serial1.transmitterEnable(TXE_PIN);
  pinMode(GRIP1_PIN, OUTPUT);
  pinMode(GRIP2_PIN, OUTPUT);
  pinMode(DIM_PIN, OUTPUT);
  analogReadResolution(12);

  motor1.attach(MOTOR1_PIN);
  motor2.attach(MOTOR2_PIN);
  motor3.attach(MOTOR3_PIN);
  motor4.attach(MOTOR4_PIN);
  motor5.attach(MOTOR5_PIN);
  servo1.attach(SERVO1_PIN);

  stop_all_motors();
  delay(ESC_INIT_DELAY_MS);
  start_all_motors();
  delay(ESC_ARM_PULSE_MS);
  stop_all_motors();
  inPtr = command_msg;
  gotInString = false;
}

//==============================================================================
// 9. MAIN LOOP
//==============================================================================

void loop() {
  char st = 0;

  read_all_adcs();

  if (Serial1.available()) {
    int inChr = Serial1.read();
    if (inChr) *inPtr++ = inChr;
    if (inChr == '\n') {
      *inPtr++ = 0x00;
      gotInString = true;
      inPtr = command_msg;
    }
  }

  if (gotInString) {
    Serial.println(command_msg);
    gotInString = false;

    build_reply_msg(reply_msg);
    Serial.println(reply_msg);
    st = 0;
    if (st) {
      stop_all_motors();
    } else {
      Serial1.print(reply_msg);
      Serial.println(msg_count);
      msg_count++;

      st = parse_command_msg(command_msg, motors, servos);

      if (!st) {
        motor1.writeMicroseconds(motor_scale(motors[1]));
        motor2.writeMicroseconds(motor_scale(motors[2]));
        motor3.writeMicroseconds(motor_scale(motors[3]));
        motor4.writeMicroseconds(motor_scale(motors[4]));
        motor5.writeMicroseconds(motor_scale(motors[5]));
        servo1.writeMicroseconds(servo_scale(servos[1]));
        grip = motors[0] - PWM_CMD_CENTER;
        if (grip < 0) {
          grip2 = GRIP_BRAKE_PWM + grip * 2;
          grip1 = GRIP_BRAKE_PWM;
        } else {
          grip1 = GRIP_BRAKE_PWM - grip * 2;
          grip2 = GRIP_BRAKE_PWM;
        }
        analogWrite(GRIP1_PIN, grip1);
        analogWrite(GRIP2_PIN, grip2);
        analogWrite(DIM_PIN, servos[0]);
      }
    }
  }
}
