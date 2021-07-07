int numA = 5, numB = 7, numC = 9;
void setup() {
  Serial.begin(9600); //initalize
  Serial.println("Serial monitor setup done");
}

void loop() {
  if(numA < numC){
    Serial.println("A is less than C");
    Serial.print("A = ");
    Serial.println(numA);
    Serial.print("C = ");
    Serial.println(numC);
    ++ numA;
  } else if( (numB != numC) || (numB < numC)) {
      Serial.println("B is less than C");
      Serial.print("B = "); 
      Serial.println(numB);
      Serial.print("C = "); 
      Serial.println(numC);
      ++ numB;
  } else {
    Serial.println("Neither A nor B is less than C");
    Serial.print("A = "); 
    Serial.println(numA);
    Serial.print("B = "); 
    Serial.println(numB);
    Serial.print("C = ");
    Serial.println(numC);
  }
}
