#define ldrPin1 13
#define ldrPin2 12
#define ldrPin3 14
#define ldrPin4 27
#define referenceLdr 26

#define MY_dirPin 25
#define MY_stepPin 33
#define MX_dirPin 27
#define MX_stepPin 14
#define stepsPerRevolution 200*32
#define thresholdError 0.12

int dest[5];
void setup() {
  Serial.begin(9600);
  SetMotorPins();

}

void loop() {
  readLDRData();
  findMotorPosition();
  delayMicroseconds(1);
  

}
void readLDRData()
{
 int potValue = 0;
 int ldrValue1 = 0; 
 int ldrValue2 = 0;
 int ldrValue3 = 0;
 int ldrValue4 = 0;
 potValue = map(analogRead(referenceLdr),0,4095,0,100);
 ldrValue1 = map(analogRead(ldrPin1),0,4095,0,100);
 ldrValue2 = map(analogRead(ldrPin2),0,4095,0,100);
 ldrValue3 = map(analogRead(ldrPin3),0,4095,0,100);
 ldrValue4 = map(analogRead(ldrPin4),0,4095,0,100);

 
 dest[4] = potValue;
 dest[0] = ldrValue1;
 dest[1] = ldrValue2;
 dest[2] = ldrValue3-14;
 dest[3] = ldrValue4; 

// for(int i=0;i<5;i++){
//  Serial.print(dest[i]);
//  Serial.print("\t");
//  }
//  Serial.println();

}

void SetMotorPosition(){

}

void SetMotorPins(){
  pinMode(MX_stepPin, OUTPUT);
  pinMode(MX_dirPin, OUTPUT);
  pinMode(MY_stepPin, OUTPUT);
  pinMode(MY_dirPin, OUTPUT);
}

void findMotorPosition() {
  int ldr_values[5];
  int refLDR = 0;
  int UL = 0;
  int UR = 0;
  int DL = 0;
  int DR = 0; 
  refLDR = dest[4];
  UL = dest[0];
  UR = dest[1];
  DL = dest[2];
  DR = dest[3];
  refLDR = (refLDR+UL+UR+DL+DR)/5;
 float thresholdLDR = thresholdError*refLDR;
 float left = (UL + DL) / 2;
 float rigth = (UR + DR) / 2;
 float up = (UL + UR) / 2;
 float down = (DL + DR) / 2;
 int LeftRightABS = abs(left-rigth);
 int UpDownABS = abs(up-down);
 
// Serial.print("Reference LDR -> ");
// Serial.println(refLDR);
// Serial.print("Left -> ");
// Serial.println(left);
// Serial.print("Right -> ");
// Serial.println(rigth);
// Serial.print("Up -> ");
// Serial.println(up);
// Serial.print("Down -> ");
// Serial.println(down);
 
 if (up > down && UpDownABS >= thresholdLDR ) {
  Serial.println("Motor Y ClockWise");
  MY_CW();

 }
 else if (up <= down && UpDownABS >= thresholdLDR) {
  Serial.println("Motor Y Counter ClockWise");
  MY_CCW();
 }
 else if (left > rigth && LeftRightABS >= thresholdLDR) {
  Serial.println("Motor X ClockWise");
  MX_CW();
 }
else if (left <= rigth && LeftRightABS >= thresholdLDR) 
{
  Serial.println("Motor X Counter ClockWise");
  MX_CCW();

 }

 else {
  Serial.println("Stop the Motor!");
  return; //Exit from the algorithm.
 }
if(UpDownABS <= thresholdLDR && LeftRightABS <= thresholdLDR) {
  Serial.println("Stop the Motor!");
  Serial.println("Stop the Motor!");
  return; 
  }
  return;
  } 
void YMotor_rotate(){
  for (int i = 0; i < stepsPerRevolution / 1024; i++) {
    digitalWrite(MY_stepPin, HIGH);
    delayMicroseconds(2000);
    digitalWrite(MY_stepPin, LOW);
    delayMicroseconds(1000);
  }
 }
 void XMotor_rotate(){
  for (int i = 0; i < stepsPerRevolution / 1024; i++) {
    digitalWrite(MX_stepPin, HIGH);
    delayMicroseconds(2000);
    digitalWrite(MX_stepPin, LOW);
    delayMicroseconds(1000);
  }
 }
 void MX_CW(){
  digitalWrite(MX_dirPin, HIGH);
  XMotor_rotate();
  }
  void MX_CCW(){
  digitalWrite(MX_dirPin, LOW);
  XMotor_rotate();
  }
  void MY_CW(){
  digitalWrite(MY_dirPin, HIGH);
  YMotor_rotate();
  }
  void MY_CCW(){
  digitalWrite(MY_dirPin, LOW);
  YMotor_rotate();
  }
