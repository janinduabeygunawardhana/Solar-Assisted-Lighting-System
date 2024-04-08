#define ldrPin1 13
#define ldrPin2 12
#define ldrPin3 14
#define ldrPin4 27
#define referenceLdr 26

#define MY_dirPin 12
#define MY_stepPin 13
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
//  for(int i=0;i<5;i++){
//    Serial.print(dest[i]);
//    Serial.print(" ");
//  }
//  Serial.println();
  delay(500);

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
// WRITE_PERI_REG(SENS_SAR_READ_CTRL2_REG, reg_b);
// SET_PERI_REG_MASK(SENS_SAR_READ_CTRL2_REG, 
//SENS_SAR2_DATA_INV);
 ldrValue4 = map(analogRead(ldrPin4),0,4095,0,100);

 Serial.print(ldrValue1);
 Serial.print("\t");
 Serial.print(ldrValue2);
 Serial.print("\t");
 Serial.print(ldrValue3);
 Serial.print("\t");
 Serial.print(ldrValue4);
 Serial.print("\t");
 Serial.println(potValue);
 
 dest[5] = potValue;
 dest[0] = ldrValue1;
 dest[1] = ldrValue2;
 dest[2] = ldrValue3;
 dest[3] = ldrValue4; 
 //return dest;
}

void SerMotorPosition(){
  int ldr_values[5];
  int refLDR = 0;
  int UL = 0;
  int UR = 0;
  int DL = 0;
  int DR = 0;
}

void SetMotorPins(){
 pinMode(MX_stepPin, OUTPUT);
 pinMode(MX_dirPin, OUTPUT);
 pinMode(MY_stepPin, OUTPUT);
 pinMode(MY_dirPin, OUTPUT);
}

int findMotorPosition(int *dest) {
 refLDR = dest[0];
 UL = dest[1];
 UR = dest[2];
 DL = dest[3];
 DR = dest[4];
 refLDR = (refLDR+UL+UR+DL+DR)/5;
 float thresholdLDR = thresholdError*refLDR;
 float left = (UL + DL) / 2;
 float rigth = (UR + DR) / 2;
 float up = (UL + UR) / 2;
 float down = (DL + DR) / 2;
 int LeftRightABS = abs(left-rigth);
 int UpDownABS = abs(up-down);
 
 Serial.print("Reference LDR -> ");
 Serial.println(refLDR);
 Serial.print("Left -> ");
 Serial.println(left);
 Serial.print("Right -> ");
 Serial.println(rigth);
 Serial.print("Up -> ");
 Serial.println(up);
 Serial.print("Down -> ");
 Serial.println(down);
 if (left > rigth && LeftRightABS >= thresholdLDR) {
 Serial.println("Motor X ClockWise");
 digitalWrite(MX_dirPin, HIGH); digitalWrite(MY_dirPin, 
HIGH);
 // Spin the stepper motor 1 revolution slowly:
 for (int i = 0; i < stepsPerRevolution / 1024; i++) {
 // These four lines result in 1 step:
 digitalWrite(MX_stepPin, HIGH); 
digitalWrite(MY_stepPin, HIGH);
 delayMicroseconds(3000);
 digitalWrite(MX_stepPin, LOW); 
digitalWrite(MY_stepPin, LOW);
 delayMicroseconds(3000);
 }
 }
else if (left <= rigth && LeftRightABS >= thresholdLDR) 
{
 Serial.println("Motor X Counter ClockWise");
 digitalWrite(MX_dirPin, LOW); digitalWrite(MY_dirPin, 
LOW);
 // Spin the stepper motor 1 revolution slowly:
 for (int i = 0; i < stepsPerRevolution / 1024; i++) {
 // These four lines result in 1 step:
 digitalWrite(MX_stepPin, HIGH); 
digitalWrite(MY_stepPin, HIGH);
 delayMicroseconds(3000);
 digitalWrite(MX_stepPin, LOW); 
digitalWrite(MY_stepPin, LOW);
 delayMicroseconds(3000);
 }
 }
 else if (up > down && UpDownABS >= thresholdLDR ) {
 Serial.println("Motor Y ClockWise");
 digitalWrite(MY_dirPin, HIGH);
 // Spin the stepper motor 1 revolution slowly:
 for (int i = 0; i < stepsPerRevolution / 1024; i++) {
 // These four lines result in 1 step:
 digitalWrite(MY_stepPin, HIGH);
 delayMicroseconds(3000);
 digitalWrite(MY_stepPin, LOW);
 delayMicroseconds(3000);
 }
 }
 else if (up <= down && UpDownABS >= thresholdLDR) {
 Serial.println("Motor Y Counter ClockWise");
 digitalWrite(MY_dirPin, LOW);

 for (int i = 0; i < stepsPerRevolution / 1024; i++) {

 digitalWrite(MY_stepPin, HIGH);
 delayMicroseconds(3000);
 digitalWrite(MY_stepPin, LOW);
 delayMicroseconds(3000);
 } }
 else {
 Serial.println("Stop the Motor!");
 return 1; //Exit from the algorithm.
 }
if(UpDownABS <= thresholdLDR && LeftRightABS <= 
thresholdLDR) {
 Serial.println("Stop the Motor!");
 Serial.println("Stop the Motor!");
 return 1; 
 }
 return 0;
 } 
MotorPositionCalculation MotorPositionCalculation;
