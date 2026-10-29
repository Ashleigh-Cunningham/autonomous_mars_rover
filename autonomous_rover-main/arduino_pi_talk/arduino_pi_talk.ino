
const byte SIGNAL_A = 2;
const byte SIGNAL_B = 3;
const byte SIGNAL_C = 9;
const byte SIGNAL_D = 8;
volatile long encoder_ticks = 0;
volatile long encoder_ticks_right = 0;
long encoder_ticks_last = 0;
const int TPR = 3000;

double omega_L = 0.0;
double omega_R = 0.0;


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
  Serial.begin(9600);

  pinMode(SIGNAL_A, INPUT);
  pinMode(SIGNAL_B, INPUT);
  pinMode(SIGNAL_C, INPUT);
  pinMode(SIGNAL_D, INPUT); 


  attachInterrupt(digitalPinToInterrupt(SIGNAL_A), decodeEncoderTicks, RISING);
  attachInterrupt(digitalPinToInterrupt(SIGNAL_C), decodeEncoderTicksRight, RISING);
}


void loop() {
  if (Serial.available() > 0) {
    /*String data = Serial.readStringUntil('\n');
    Serial.print("You sent me: ");
    Serial.println(data);
  }
}*/
  
    // Estimate the rotational speed [rad/s]
    omega_L = 2.0 * PI * ((double)encoder_ticks / (double)TPR);
    
    
    // Reset the encoder ticks counter
    encoder_ticks = 0;
  

//calculate omegaR and vR
  
    // Estimate the rotational speed [rad/s]
    omega_R = 2.0 * PI * ((double)encoder_ticks_right / (double)TPR);
    
    // Record the current time [ms]
    
    // Reset the encoder ticks counter
    encoder_ticks_right = 0;
  
    
  }
  Serial.print(omega_L);
  Serial.print(",");
  Serial.print(omega_R);
  Serial.print("\n");
}
