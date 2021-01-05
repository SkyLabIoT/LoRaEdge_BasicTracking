#include <Wire.h>
void setup() {
  // put your setup code here, to run once:
Serial.begin(9600);
pinMode (LEDR , OUTPUT);
pinMode (LEDG , OUTPUT);
pinMode (LEDB , OUTPUT);
digitalWrite(LEDR , HIGH);
digitalWrite(LEDB , HIGH);
digitalWrite(LEDG , HIGH);
}

void loop() {
  // put your main code here, to run repeatedly:
Serial.println("test");
delay(1000);
digitalWrite(LEDR , LOW);
delay(1000);
digitalWrite(LEDR , HIGH);
digitalWrite(LEDB , LOW);
delay(1000);
digitalWrite(LEDB , HIGH);
digitalWrite(LEDG , LOW);
delay(1000);
digitalWrite(LEDG , HIGH);
LSM303_sleep();
}

void LSM303_sleep(){
  Wire.begin();
  //Wire.beginTransmission(0b00111100);
  //Wire.write(byte(0x60));
  //Wire.write(byte(0b00010011));
  //Wire.endTransmission();
  Wire.beginTransmission(byte(0b0011001));
  Wire.write(byte(0x20));
  Wire.write(byte(0b00001000));
  Wire.endTransmission();
  Wire.end();
}
