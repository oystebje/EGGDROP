#include <AVR_RTC.h>



//*************************************
// DC Motor PID position control example
// By Øystein Bjelland, IIR, NTNU
// Based on this example: https://curiores.com/dc-motor-control/ 
//**************************************

#include <util/atomic.h>

//**************************************

#define ENCA 2      //Encoder pinA
#define ENCB 3      //Encoder pinB
#define PWM 11       //motor PWM pin
#define IN2 6       //motor controller pin2
#define IN1 7       //motor controller pin1

volatile int posi = 0; // position variable. https://www.arduino.cc/reference/en/language/variables/variable-scope-qualifiers/volatile/ 
long prevT = 0;
float eprev = 0;
float eintegral = 0;

//********************************************

void setup() {

  Serial.begin (9600);

  // ENCODER
  pinMode (ENCA, INPUT);
  pinMode (ENCB, INPUT);
  attachInterrupt(digitalPinToInterrupt(ENCA), readEncoder, RISING); //https://www.arduino.cc/reference/en/language/functions/external-interrupts/attachinterrupt/

  // DC MOTOR
  pinMode(PWM, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);

  Serial.println("target pos");
}

//************************************************

void loop() {

  // Set target position (2048 * X rounds)
  int targ = 2048*5;

  //PID constants
  float kp = 1.0;
  float kd = 0.01;
  float ki = 0.0;

  //time diference
  long currT = micros();
  float deltaT = ((float) (currT - prevT))/(1.0e6);
  prevT = currT;

  int pos = 0;
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE){
    pos = posi;
  }

  //Error calculation
  int e = pos - targ;

  //PID Calculation
  float dedt = (e-eprev)/(deltaT);  //Derivative
  eintegral = eintegral + e*deltaT; //Integral
  float u = kp*e + kd*dedt + ki*eintegral;  //Control signal

  //Motor power
  float pwr = fabs(u);  //fabs == floating point, absolute value
    if(pwr > 70){
      pwr = 70; //Capping
    }

  //Motor direction
  int dir = 1;
  if(u<0){
    dir = -1; //if the control signal is negative, we flip the motor direction
  }

  //Send signal to motor
  setMotor(dir,pwr,PWM,IN1,IN2);

  //Store previous error
  eprev = e;

//  Serial.print(targ);
//  Serial.print(" ");
  Serial.println(pos);
//  Serial.print(" ");
//  Serial.print(u);
//  Serial.print(" ");
//  Serial.println(pwr);
  
}

//******************************************
//FUNCTIONS FOR MOTOR AND ENCODER

//MOTOR
void setMotor(int dir, int pwmVal, int pwm, int in1, int in2){
  analogWrite(pwm,pwmVal);
  if(dir == 1){
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
  }
  else if(dir == -1){
    digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH);
  }
  else{
    digitalWrite(in1, LOW);
    digitalWrite(in2, LOW);
  }
}


//ENCODER
void readEncoder(){
  int b = digitalRead(ENCB);
  if(b > 0){
    posi++;
  }
  else{
    posi--;
  }
}
