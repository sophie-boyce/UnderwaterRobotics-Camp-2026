
/* 
Rovotron Cadet RVCBOT-D control receiver program

Controls ROV motors, switches, servos using RS-485 link with a 
hex ASCII command message and a reply message with telemetry.

Runs on a Teensy 4.0 board

Revision history

2023-04-04 DF  Teensy version, making it simpler to use Servo library
2023-04-09 DF  ready to try running, needs I2C code
2023-04-16 DF  Reversing motors, commenting out I2C code for now
2023-04-30 DF  Adding exponential control to motors
2023-05-07 DF  Not doing exponnential, instead adding 7 seconds delay at startup per Bluerobotics
2023-06-04 DF  Changing servo 4 to analogWrite becasue we need faster PWM to reduce LED flicker
2023-06-09 DF  Adding the MS5837 I2C pressure sensor code
2023-06-10 DF  Disabled depth sensor, as it's not behaving in top end

2023-07-31 DF  Added pin defines for RVCBOT-A pinout, Teensy 4.0 MPU, 6 motors
2023-08-01 DF  Swapped motor5 and servo2 for strafing motor
2023-08-16 DF  Checking camera servo and LED 
2023-08-26 DF  RVCBOT-B version has different pinout
2023-08-29 DF  Added ADC reads
2023-09-09 DF  Changing to talk to RVTOP-A board, different motor count etc.
2023-11-19 DF  RVCBOT-C version has only 6 motors, tilt, dim
2024-02-22 DF  Adding pressure to telem message

Functions:
 6 PWM motor outputs
 2 PWM servo outputs

Gripper on motor 0 (controlled by board 4 motor A)
 It makes two PWM outputs range 0..FF, 
 but grip1 is outward, grip2 is inward. 
 So grip1 is 0 while grip2 is pulsing. 
 
Telemetry sensors:

voltage 0 Battery voltage
analog 1  Depth analog
analog 2  --
analog 2  LED temp
analog 4  H2O temp
pressure  Depth 

The values transmitted are the raw ADC values.
0.00V = 0x000
3.30V = 0xFFF (4095.)

Things to do:

3. Add better comm. error handling
4. Devise a way to test error handling
*/

//------------ INCLUDES AND LIBRARIES ------------------------------- //

// Generic Arduino PWM servo
//We may select a dfferent driver if this sucks
#include <Servo.h> 

float depthMeters;  // should be depth in meters 

// ------------ Sizes of things --------------- //

// these things each start at 0 for the special case
#define NMOTORS 6		// max number of motor control channels
#define NSERVOS 2   	// match top-side P section field count
#define NSWITCHES 2		// match top-side S section field count
#define NVOLTS 6		// max number of analog channels sent


// Teensy pin definitions ------------------ //

// Serial1 is pins 0,1 for Host
#define TXE_PIN      2

// Motors and Servos
#define MOTOR1_PIN   3
#define MOTOR2_PIN   4
#define MOTOR3_PIN   5
#define MOTOR4_PIN   6
#define MOTOR5_PIN   7
#define GRIP1_PIN    9
#define GRIP2_PIN   10
#define SERVO1_PIN  17 // used to be 21
// #define SERVO2_PIN  21
#define DIM_PIN     22

// I2C bus
#define SDA0_PIN    18
#define SCL0_PIN    19

// SPI is reserved
#define MOSI_PIN    11
#define MISO_PIN    12
#define SCK_PIN     13

#define VBATT_PIN   A9 
#define ANALOG1_PIN A3
#define ANALOG2_PIN A2
#define ANALOG3_PIN A1
#define ANALOG4_PIN A0
#define PRESSURE_PIN  A6

#define Pwm0 128
#define BOTTOM_DEBUG_SERIAL 1

//------------ GLOBAL VARIABLES AND RUNTIME STATE ------------------------------- //

// these are received from the surface
int motors[NMOTORS];		// motor command values
int servos[NSERVOS];		// servo command values
int volts[NVOLTS];					// ADC results

int msg_count = 0;   // for debugging message rate


// The ROV messages get put here
char command_msg[300];	// command message, newline, 0
char reply_msg[300];		// reply message, newline, 0
char buf[300];			// temporary string storage

// Define all the PWM motor and servo objects. They're attached to pins in setup()
Servo motor1;  
Servo motor2;  
Servo motor3;  
Servo motor4;  
Servo motor5;  

Servo servo1;
// Servo servo2;

char inString[100];          // where the received data link string goes
char inChr = 0;              // the receive char is built here
char * inPtr;
bool gotInString = false;    // a data link string has been received, delimited by term char

int grip, grip1, grip2 = 0;
bool escStartupDone = false;
unsigned long lastGoodPacketMs = 0;
int lastParseStatus = -1;

// ---------------------- SENSOR READINGS --------------------------- //

const int smoothRange  = 100;      // 0 for no smoothing, smoothRange-1 for a lot
const int smooth    = 80;      // 0 for no smoothing, 9 for a lot of smoothing

// read all ADCs and store in volts[]
/*
Purpose:
Samples all bottom-side analog telemetry channels and stores smoothed raw ADC counts.
Inputs:
Physical analog pins for battery, auxiliary sensors, temperature, and pressure.
Outputs:
Updates the global volts[] telemetry array.
Side Effects:
Performs analogRead() calls every control loop pass.
Dependencies:
analogReadResolution(12) must be configured during setup().
Notes:
This is sensor packaging only; it does not directly affect actuator outputs.
*/
void read_all_adcs(void) {
  volts[0] = (volts[0]*smooth+analogRead(VBATT_PIN)*(smoothRange-smooth))/smoothRange;  
  volts[1] = (volts[1]*smooth+analogRead(ANALOG1_PIN)*(smoothRange-smooth))/smoothRange;  
  volts[2] = (volts[2]*smooth+analogRead(ANALOG2_PIN)*(smoothRange-smooth))/smoothRange;  
  volts[3] = (volts[3]*smooth+analogRead(ANALOG3_PIN)*(smoothRange-smooth))/smoothRange;  
  volts[4] = (volts[4]*smooth+analogRead(ANALOG4_PIN)*(smoothRange-smooth))/smoothRange;  
  volts[5] = (volts[5]*smooth+analogRead(PRESSURE_PIN)*(smoothRange-smooth))/smoothRange;  
  return;
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
	char digit;
	int i, val;
	val = 0;
	for (i = 0; i < n; i++) {
		if ((digit = atoxdigit(*str++)) == -1) return -4;
		val = (val << 4) + digit;
	}
	return val;
}

// -------------------- COMMAND PACKET PARSING ------------------------ //

// Parse the commands in msg into the arrays mots, sws, sers
// returns 1 if error, 0 if all OK
/*
Purpose:
Validates and decodes one top-side command packet into motor and servo command arrays.
Inputs:
Null-terminated command buffer received over RS-485.
Outputs:
Updates caller-provided motor and servo arrays; returns 0 when accepted.
Side Effects:
None directly; actuator writes happen later only if parsing succeeds.
Dependencies:
Packet format and field count must match the top-side transmitter.
Notes:
Checksum comparison is currently present but disabled in the existing code.
*/
int parse_command_msg(char *msg, int *mots, int *sers) {
	int i, checksum = 0;
	int val;

	if (*msg++ != 'M') return 1;

	for (i=0;i<NMOTORS;i++) {
		if ((val = atox(msg, 2)) < 0) return 1;
		msg += 2;
		checksum += mots[i] = val;
	}

	if (*msg++ != 'P') return 1;
	for (i=0;i<NSERVOS;i++) {
		if ((val = atox(msg, 2)) < 0) return 1;
		msg += 2;
		checksum += sers[i] =  val;
	}

	if (*msg++ != 'S') return 1;
	for (i=0;i<NSWITCHES;i++) {
		if ((val = atox(msg, 1)) < 0) return 1;
		msg += 1;
		checksum += val;
	}

	if (*msg++ != 'C') return 1;
	if ((val = atox(msg, 2)) < 0) return 1;
	msg += 2;
//	if (checksum != (unsigned char) val) return 1;
	if (*msg++ != '\n') return 1; 
	if (*msg++ != '\0') return 1; 
	return 0;
}

// -------------------- TELEMETRY PACKAGING ------------------------ //

// build a reply string from ROV
/*
Purpose:
Builds the bottom-side telemetry response packet from raw ADC values.
Inputs:
Global volts[] array populated by read_all_adcs().
Outputs:
Writes a newline-terminated V... telemetry string into msg.
Side Effects:
None beyond writing the supplied buffer.
Dependencies:
Top-side parser expects the same number and width of telemetry fields.
Notes:
No telemetry checksum is included in the existing protocol.
*/
void build_reply_msg(char *msg) {
	int i;

	*msg++ = 'V';		// for volts
	for (i=0;i<NVOLTS;i++) {
		msg += sprintf(msg, "%03X", volts[i]);
	}

	// no checksum - not needed for safety??!?!?
  *msg++ = '\n'; 
	*msg = '\0';
}

// scale the motor command from the old 0..FF to 1100..1900 microseconds
//
// microseconds is 128 > 1500
//                 255 > 1100
//                  0  > 1900
//

// scale the motor command from the old 0..FF to 1000..2000 microseconds
/*
Purpose:
Maps an 8-bit command byte into ESC servo-pulse microseconds.
Inputs:
Command byte where 128 is neutral.
Outputs:
Pulse width suitable for Servo.writeMicroseconds().
Side Effects:
None.
Dependencies:
ESCs must be attached to Servo objects before use.
Notes:
This preserves the existing 1000..2000 us command span.
*/
int motor_scale(int cmd) {
  return (((cmd - 128) * 1000) / 256) + 1500;
} 

// scale the servo command from the old 0..FF to 700..2300 microseconds
/*
Purpose:
Maps an 8-bit command byte into camera-servo pulse microseconds.
Inputs:
Command byte where 128 is center.
Outputs:
Pulse width suitable for Servo.writeMicroseconds().
Side Effects:
None.
Dependencies:
Servo object must be attached during setup().
Notes:
Uses the existing wider 700..2300 us servo span.
*/
int servo_scale(int cmd) {
  return (((cmd - 128) * 1600) / 256) + 1500;
} 


// start all motors by setting speeds to slighty forward
/*
Purpose:
Applies the existing ESC wake/arming pulse to all thruster channels.
Inputs:
None.
Outputs:
Writes the arming pulse to each thruster Servo object.
Side Effects:
Thrusters/ESCs receive a brief non-neutral command during startup.
Dependencies:
Servo objects must already be attached to motor pins.
Notes:
Called only during setup as part of the existing BlueRobotics ESC sequence.
*/
void start_all_motors(void) {
  motor1.writeMicroseconds(1600);  
  motor2.writeMicroseconds(1600);  
  motor3.writeMicroseconds(1600);  
  motor4.writeMicroseconds(1600);  
  motor5.writeMicroseconds(1600);  
}

// stop all motors by setting speeds to 'off'
/*
Purpose:
Commands all thruster ESC outputs to neutral/off.
Inputs:
None.
Outputs:
Writes the neutral pulse to each thruster Servo object.
Side Effects:
Immediately changes thruster output commands.
Dependencies:
Servo objects must be attached.
Notes:
Used during startup and intended safety fallback paths.
*/
void stop_all_motors(void) {
  motor1.writeMicroseconds(1500);  
  motor2.writeMicroseconds(1500);  
  motor3.writeMicroseconds(1500);  
  motor4.writeMicroseconds(1500);  
  motor5.writeMicroseconds(1500);  
}

void runEscStartupSequence(void) {
  if (escStartupDone) return;
  stop_all_motors();
  delay(7000);
  start_all_motors();
  delay(1000);
  stop_all_motors();
  escStartupDone = true;
#if BOTTOM_DEBUG_SERIAL
  Serial.println("ESC startup done");
#endif
}

void applyActuatorCommands(void) {
  motor1.writeMicroseconds(motor_scale(motors[1]));
  motor2.writeMicroseconds(motor_scale(motors[2]));
  motor3.writeMicroseconds(motor_scale(motors[3]));
  motor4.writeMicroseconds(motor_scale(motors[4]));
  motor5.writeMicroseconds(motor_scale(motors[5]));
  // servo2.writeMicroseconds(servo_scale(servos[2]));
  grip = motors[0] - 128;
  if (grip < 0) {
    grip2 = 1 + grip * 2;
    grip1 = 1;
  } else {
    grip1 = 1 - grip * 2;
    grip2 = 1;
  }
  // analogWrite(GRIP1_PIN, grip1);
  // analogWrite(GRIP2_PIN, grip2);
  analogWrite(DIM_PIN, servos[0]);
  analogWrite(SERVO1_PIN, servos[1]);
}

/* ----------------- init code -------------------- */

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
  Serial.begin(115200);            // debug monitor port
  Serial1.begin(115200);             // tether port
  Serial1.transmitterEnable(TXE_PIN);   // RS-485 Tx enable  
  pinMode(GRIP1_PIN, OUTPUT);        // gripper pins are always driven
  pinMode(GRIP2_PIN, OUTPUT);        // gripper pins are always driven
  pinMode(DIM_PIN, OUTPUT);        // the LED PWM pin is always driven
  analogReadResolution(12);        // use all the bits of the ADC

  // attaches the servo on pin to the servo object. These are named for the 
  // motors and servos in the old code. 
  // Note that motor 9,10 and servo 5,6 are ignored. 
  motor1.attach(MOTOR1_PIN);  
  motor2.attach(MOTOR2_PIN);  
  motor3.attach(MOTOR3_PIN);  
  motor4.attach(MOTOR4_PIN);  
  motor5.attach(MOTOR5_PIN);  
  servo1.attach(SERVO1_PIN);  
  // servo2.attach(SERVO2_PIN);

  for (int i = 0; i < NMOTORS; i++) motors[i] = Pwm0;
  for (int i = 0; i < NSERVOS; i++) servos[i] = Pwm0;

  runEscStartupSequence();
  inPtr = command_msg;
  gotInString = false;
#if BOTTOM_DEBUG_SERIAL
  Serial.println("TRASHBACK ready");
#endif
}

/*
Main operating loop:

Receive ROV command message  
Translate telemetry data to surface
Send ROV commands to servos
Build ROV status message with telemetry data
*/

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
	char st = 0;
  
  read_all_adcs();      // get the volts info frequenctly to average it

  // Read the data link receive port, and act on the char that was received
  if (Serial1.available()) {
    int inChr = Serial1.read();
    if (inChr) {
      if (inPtr < command_msg + sizeof(command_msg) - 2) {
        *inPtr++ = inChr;
      } else {
        inPtr = command_msg;
      }
    }
    if (inChr == '\n') {
      *inPtr++ = 0x00;
      gotInString = true;
      inPtr = command_msg;
    }
  }
	if (gotInString) {
    gotInString = false;
    build_reply_msg(reply_msg);
    Serial1.print(reply_msg);

    st = parse_command_msg(command_msg, motors, servos);
    lastParseStatus = st;
    if (!st) {
      applyActuatorCommands();
      lastGoodPacketMs = millis();
      msg_count++;
#if BOTTOM_DEBUG_SERIAL
      if ((msg_count % 20) == 0) {
        Serial.printf("OK #%d M1=%02X drv=%02X\n", msg_count, motors[1], motors[3]);
      }
#endif
    }
#if BOTTOM_DEBUG_SERIAL
    else {
      Serial.print("PARSE FAIL: ");
      Serial.println(command_msg);
    }
#endif
	}
}
