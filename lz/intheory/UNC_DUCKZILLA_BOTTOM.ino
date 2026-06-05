
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

//------------ INCLUDES ------------------------------- //

// Generic Arduino PWM servo
//We may select a dfferent driver if this sucks
#include <Servo.h> 

float depthMeters;  // should be depth in meters 

// ------------ Sizes of things --------------- //

// these things each start at 0 for the special case
#define NMOTORS 6		// max number of motor control channels
#define NSERVOS 3		// max number of servo control channels
#define NSWITCHES 2		// must match top-side S section in command packet
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
#define SERVO1_PIN  21
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


//------------ Global Variables ------------------------------- //

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

char inString[100];          // where the received data link string goes
char inChr = 0;              // the receive char is built here
char * inPtr;
bool gotInString = false;    // a data link string has been received, delimited by term char

int grip, grip1, grip2 = 0;   // for gripper direction calculation

// ---------------------- ADC read --------------------------- //

const int smoothRange  = 100;      // 0 for no smoothing, smoothRange-1 for a lot
const int smooth    = 80;      // 0 for no smoothing, 9 for a lot of smoothing

// read all ADCs and store in volts[]
void read_all_adcs(void) {
  volts[0] = (volts[0]*smooth+analogRead(VBATT_PIN)*(smoothRange-smooth))/smoothRange;  
  volts[1] = (volts[1]*smooth+analogRead(ANALOG1_PIN)*(smoothRange-smooth))/smoothRange;  
  volts[2] = (volts[2]*smooth+analogRead(ANALOG2_PIN)*(smoothRange-smooth))/smoothRange;  
  volts[3] = (volts[3]*smooth+analogRead(ANALOG3_PIN)*(smoothRange-smooth))/smoothRange;  
  volts[4] = (volts[4]*smooth+analogRead(ANALOG4_PIN)*(smoothRange-smooth))/smoothRange;  
  volts[5] = (volts[5]*smooth+analogRead(PRESSURE_PIN)*(smoothRange-smooth))/smoothRange;  
  return;
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
	char digit;
	int i, val;
	val = 0;
	for (i = 0; i < n; i++) {
		if ((digit = atoxdigit(*str++)) == -1) return -4;
		val = (val << 4) + digit;
	}
	return val;
}

// -------------------- Command parser ------------------------ //

// Parse the commands in msg into the arrays mots, sws, sers
// returns 1 if error, 0 if all OK
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

	// Top-side (retrytrash) sends M + P + S + C; skip switch nibbles before checksum.
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

// -------------------- Build a reply ------------------------ //

// build a reply string from ROV
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
int motor_scale(int cmd) {
  return (((cmd - 128) * 1000) / 256) + 1500;
} 

// scale the servo command from the old 0..FF to 700..2300 microseconds
int servo_scale(int cmd) {
  return (((cmd - 128) * 1600) / 256) + 1500;
} 


// start all motors by setting speeds to slighty forward
void start_all_motors(void) {
  motor1.writeMicroseconds(1600);  
  motor2.writeMicroseconds(1600);  
  motor3.writeMicroseconds(1600);  
  motor4.writeMicroseconds(1600);  
  motor5.writeMicroseconds(1600);  
}

// stop all motors by setting speeds to 'off'
void stop_all_motors(void) {
  motor1.writeMicroseconds(1500);  
  motor2.writeMicroseconds(1500);  
  motor3.writeMicroseconds(1500);  
  motor4.writeMicroseconds(1500);  
  motor5.writeMicroseconds(1500);  
}

/* ----------------- init code -------------------- */

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

  stop_all_motors();         // initialize to zero speed
  delay(7000);               // give the ESCs time to initialize
  start_all_motors();        // Give all motors a speed to get them turned on
  delay(1000);               // for just a moment
  stop_all_motors();         // then put back to zero speed
  inPtr = command_msg;       // tell the parser to do its work on this string
  gotInString = false;
}

/*
Main operating loop:

Receive ROV command message  
Translate telemetry data to surface
Send ROV commands to servos
Build ROV status message with telemetry data
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
    if (inChr == '\n') { // boring old newline terminator
      *inPtr++ = 0x00;           // null terminate the input string
      gotInString = true;
      inPtr = command_msg;       // tell the parser to do its work on this string
    }
  }
	if (gotInString) {
    Serial.println(command_msg);
    gotInString = false;
    
		build_reply_msg(reply_msg);	// code it up
    Serial.println(reply_msg);
		st = 0;   // !!! Assume message was good
		if (st) {
			stop_all_motors();		 // timeout - shut down
		}
		else {		// got a message - is it good?
  //    Serial.print(reply_msg);   // send analog stuff back
      Serial1.print(reply_msg);   // send analog stuff back
      Serial.println(msg_count);
      msg_count++;
      
			st = parse_command_msg(command_msg, motors, servos);
  //    Serial.printf("st=%d  M=%02X S=%02X\n", st, motors[3], servos[0]);
      
			if (!st) {			// only use message if valid
        motor1.writeMicroseconds(motor_scale(motors[1]));  
        motor2.writeMicroseconds(motor_scale(motors[2]));  
        motor3.writeMicroseconds(motor_scale(motors[3]));  
        motor4.writeMicroseconds(motor_scale(motors[4]));  
        motor5.writeMicroseconds(motor_scale(motors[5]));  
        servo1.writeMicroseconds(servo_scale(servos[1]));  
        grip = motors[0] - 128;
        if (grip < 0) {
          grip2 = 1+grip*2;    // I think this is correct, brakes when both = 1
          grip1 = 1;
        }
        else {
          grip1 = 1-grip*2;
          grip2 = 1;
        }
        analogWrite(GRIP1_PIN, grip1);
        analogWrite(GRIP2_PIN, grip2);   // pwm but direction dependent
        analogWrite(DIM_PIN, servos[0]);
			}
		}
	}	
}
