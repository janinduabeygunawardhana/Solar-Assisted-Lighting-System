#define ldrPin1 A0
#define ldrPin2 A1
#define ldrPin3 A2
#define ldrPin4 A3
#define referenceLdr A4

int dest[5];
void setup() {
  Serial.begin(9600);
  // put your setup code here, to run once:

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
 potValue = map(analogRead(referenceLdr),0,1023,0,100);
 ldrValue1 = map(analogRead(ldrPin1),0,1023,0,100);
 ldrValue2 = map(analogRead(ldrPin2),0,1023,0,100);
 ldrValue3 = map(analogRead(ldrPin3),0,1023,0,100);
// WRITE_PERI_REG(SENS_SAR_READ_CTRL2_REG, reg_b);
// SET_PERI_REG_MASK(SENS_SAR_READ_CTRL2_REG, 
//SENS_SAR2_DATA_INV);
 ldrValue4 = map(analogRead(ldrPin4),0,1023,0,100);

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
