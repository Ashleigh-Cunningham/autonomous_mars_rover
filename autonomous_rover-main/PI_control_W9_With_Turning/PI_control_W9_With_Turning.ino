//################### INITIALIZATION ##########################
#include <Arduino_LSM6DS3.h>

//#################GYRO##################
float omega_x, omega_y, omega_z;  //angular rates from gyro degrees/s
// Variables to store accelerations [g's]
float a_x, a_y, a_z;
// Variables to store sample rates from sensor [Hz]
float a_f, g_f;
//###########################################

int kp = 150;
int ki = 30;
double kd = 30;
double vdesired = 0;      //desired V for PID Calculations //should be less than 0.9
double omegadesired = 0;  //desired w for PID Calculations //has worked with omega 2 and vdesired of 0.5

int EA = 6;
int I1 = 7;
int I2 = 4;

int EB = 10;
int I3 = 12;
int I4 = 13;

const double ELL = 0.2775;
const byte SIGNAL_A = 2;
const byte SIGNAL_B = 3;
const byte SIGNAL_C = 9;
const byte SIGNAL_D = 8;
//serial input constants
const byte numChars = 32;
char receivedChars[numChars];
char tempChars[numChars]; 
//serial input variables
char messageFromPC[numChars] = {0};
int integerFromPC = 0;
float floatFromPC = 0.0;
boolean newData = false;
// Encoder ticks per (motor) revolution (TPR)
const int TPR = 3000;

// Wheel radius [m]
const double RHO = 0.0625;

// Counter to keep track of encoder ticks [integer]
volatile long encoder_ticks = 0;
volatile long encoder_ticks_right = 0;
//variable to store estimated angular rate of left wheel [rad/s]
double omega_L = 0.0;
double omega_R = 0.0;
byte u = 0;
// Sampling interval for measurements in milliseconds
const int T = 100;

// Counter to keep track of the last number of ticks [integer]
long encoder_ticks_last = 0;
long t_now = 0;
long t_last = 0;

long encoder_ticks_last_right = 0;
long t_now_right = 0;
long t_last_right = 0;

double e_int = 0;
double e_intR = 0;

//previous loop veriables:
  double vL;
  double vR;
  double vLd;
  double vRd;
  double errLeft, errRight;
  double uL, uR;

// This function is called when SIGNAL_A goes HIGH
void decodeEncoderTicks() {
  if (digitalRead(SIGNAL_B) == LOW) {
    // SIGNAL_A leads SIGNAL_B, so count one way
    encoder_ticks--;
  } else {
    // SIGNAL_B leads SIGNAL_A, so count the other way
    encoder_ticks++;
  }
}
void decodeEncoderTicksRight() {
  if (digitalRead(SIGNAL_D) == LOW) {
    // SIGNAL_A leads SIGNAL_B, so count one way
    encoder_ticks_right--;
  } else {
    // SIGNAL_B leads SIGNAL_A, so count the other way
    encoder_ticks_right++;
  }
}

void setup() {

  Serial.begin(115200);
  // put your setup code here, to run once:
  pinMode(EA, OUTPUT);
  pinMode(I1, OUTPUT);
  pinMode(I2, OUTPUT);
  pinMode(EB, OUTPUT);
  pinMode(I3, OUTPUT);
  pinMode(I4, OUTPUT);

  pinMode(SIGNAL_A, INPUT);
  pinMode(SIGNAL_B, INPUT);
  pinMode(SIGNAL_C, INPUT);
  pinMode(SIGNAL_D, INPUT);


  // Every time SIGNAL_A goes HIGH, this is a pulse
  attachInterrupt(digitalPinToInterrupt(SIGNAL_A), decodeEncoderTicks, RISING);
  attachInterrupt(digitalPinToInterrupt(SIGNAL_C), decodeEncoderTicksRight, RISING);
  // Check that the board is initialized
  if (!IMU.begin()) {
    // Print an error message if the IMU is not ready
    Serial.print("Failed to initialize IMU :(");
    Serial.print("\n");
    while (1) {
      delay(10);
    }
  }
  g_f = IMU.gyroscopeSampleRate();
  // Print a message
  //Serial.print("Program initialized.");
  //Serial.print("\n");
}
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void loop() {
  // Get the elapsed time [ms]
  t_now = millis();
  t_now_right = millis();

  recvWithStartEndMarkers();
    if (newData == true) {
        strcpy(tempChars, receivedChars);
            // this temporary copy is necessary to protect the original data
            //   because strtok() used in parseData() replaces the commas with \0
        parseData();
        newData = false;
    }



//calculate omegaL and vL 
  if (t_now - t_last >= T) {
    // Estimate the rotational speed [rad/s]
    omega_L = 2.0 * PI * ((double)encoder_ticks / (double)TPR) * 1000.0 / (double)(t_now - t_last);
    vL = RHO * omega_L;
    t_last = t_now;
    // Reset the encoder ticks counter
    encoder_ticks = 0;
  }

//calculate omegaR and vR
  if (t_now_right - t_last_right >= T) {
    // Estimate the rotational speed [rad/s]
    omega_R = 2.0 * PI * ((double)encoder_ticks_right / (double)TPR) * 1000.0 / (double)(t_now_right - t_last_right);
    vR = -omega_R * RHO;
    // Record the current time [ms]
    t_last_right = t_now_right;
    // Reset the encoder ticks counter
    encoder_ticks_right = 0;
  }

  if (IMU.gyroscopeAvailable()) {
    IMU.readGyroscope(omega_x, omega_y, omega_z);
  }

  //compute desired V's
  vLd = vdesired - ((ELL/2)*omegadesired);
  vRd = vdesired + ((ELL/2)*omegadesired);

  //compute errors
  errLeft = vLd - vL;
  errRight = vRd - vR;

  //compute u left/right
  uL = short_PI_controller(errLeft, kp, &e_int, ki);  //kP we shall pick/tune
  uR = short_PI_controller(errRight, kp, &e_intR, ki);

  //drive forward
  drive(uL, uR);
  //drive(255, 255);
  
    // Serial.print("  uL:");
    // Serial.print(uL);
    // Serial.print("  uR:");
    // Serial.print(uR);
  Serial.print(compute_vehicle_speed(vR,vL));
  Serial.print(",");
  Serial.print(compute_vehicle_rate(vR, vL));
  Serial.print("\n");
  delay(50);   
}
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//FUNCTIONS//
short PI_controller(double e_now, double k_P, double* e_int, double k_I) {
  short u;
  u = (short)(k_P * e_now + k_I * *e_int);
  // Serial.print("  u:");
  // Serial.print(u);
  if (u > 255) {
    u = 255;
  } else if (u < -255) {
    u = -255;
  } else
  {
    *e_int += e_now;
  }

  return u;
}

//PID_controller(double e_now, double k_P, double* e_int, double k_I, double k_D){


}

void recvWithStartEndMarkers() {
    static boolean recvInProgress = false;
    static byte ndx = 0;
    char startMarker = '<';
    char endMarker = '>';
    char rc;

    while (Serial.available() > 0 && newData == false) {
        rc = Serial.read();

        if (recvInProgress == true) {
            if (rc != endMarker) {
                receivedChars[ndx] = rc;
                ndx++;
                if (ndx >= numChars) {
                    ndx = numChars - 1;
                }
            }
            else {
                receivedChars[ndx] = '\0'; // terminate the string
                recvInProgress = false;
                ndx = 0;
                newData = true;
            }
        }

        else if (rc == startMarker) {
            recvInProgress = true;
        }
    }
}

void parseData() {      // split the data into its parts

    char * strtokIndx; // this is used by strtok() as an index

    strtokIndx = strtok(tempChars,",");      // get the first part - the string
    vdesired = atof(strtokIndx); // copy it to messageFromPC
 
    strtokIndx = strtok(NULL, ","); // this continues where the previous call left off
    omegadesired = atof(strtokIndx);     // convert this part to an integer

      // convert this part to a float

}



// Compute vehicle speed [m/s]
double compute_vehicle_speed (double v_L, double v_R) {
  double v;
  v = 0.5 * (v_L + v_R);
  return v;
}
// Compute vehicle turning rate [rad/s]
double compute_vehicle_rate(double v_L, double v_R) {
  double omega;
  omega = 1.0 / ELL * (v_R - v_L);
  return omega;
}

void drive(int uLeft, int uRight) {
  // A variable for the motor PWM command [0-255]
  if ((uLeft >= 0) && (uRight >= 0)) {  //forward direction setting
    
    digitalWrite(I1, HIGH);
    digitalWrite(I2, LOW);
    digitalWrite(I3, LOW);
    digitalWrite(I4, HIGH);
    // PWM command to the motor driver
    analogWrite(EA, uRight);
    analogWrite(EB, uLeft);
  }
  else if ((uLeft < 0) && (uRight < 0)) {  //Backwards
    

    digitalWrite(I1, LOW);
    digitalWrite(I2, HIGH);
    digitalWrite(I3, HIGH);
    digitalWrite(I4, LOW);
    // PWM command to the motor driver
    analogWrite(EA, -1 * uRight);
    analogWrite(EB, -1 * uLeft);
  }
  else if ((uLeft < 0) && (uRight > 0)) {
    
    digitalWrite(I1, HIGH);
    digitalWrite(I2, LOW);
    digitalWrite(I3, HIGH);
    digitalWrite(I4, LOW);
    // PWM command to the motor driver
    analogWrite(EA, uRight);
    analogWrite(EB, -1 * uLeft);
  }
  else if ((uLeft > 0) && (uRight < 0)) {
    
    digitalWrite(I1, LOW);
    digitalWrite(I2, HIGH);
    digitalWrite(I3, LOW);
    digitalWrite(I4, HIGH);
    // PWM command to the motor driver
    analogWrite(EA, -1*uRight);
    analogWrite(EB, uLeft);
  }
}
