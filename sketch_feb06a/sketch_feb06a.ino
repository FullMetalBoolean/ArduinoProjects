//Author::   JReuwer
//Date::  2/6/2019
//Version::   1.0
//Lesson::  2

void setup() {
  Serial.begin(9600);
  Serial.println("Serial monitor setup done");
}

void loop() {
  Serial.println("Beginning the loop");
  delay(2000);
  Serial.println("Middle of the loop");
  delay(2000);
  Serial.println("End of the loop");
}
