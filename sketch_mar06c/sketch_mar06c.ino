void setup() {
  Serial.begin(9600); //initalize
}

void loop() {
  int A = 5, B = 7, C = 9;
  if(A < C){
    Serial.println("A is less than C");
    Serial.println("A = " + A);
    Serial.println("C = " + C);
    A++;
  } else if(B < C) {
      Serial.println("B is less than C");
      Serial.println("B = " + B);
      Serial.println("C = " + C);
      B++;
  } else {
    Serial.println("Neither A nor B is less than C");
    Serial.println("A = " + A);
    Serial.println("B = " + B);
    Serial.println("C = " + C);
  }

}
