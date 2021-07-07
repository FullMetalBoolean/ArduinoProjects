float numberCounter = 0; //initialize number counter holder
void setup() {
  Serial.begin(9600); //begin
  Serial.println("Serial monitor setup done \n"); //setup complete
}

void loop() {
  Serial.println(numberCounter); //print current iteration of numberCounter
  numberCounter++; //increment counter 
  delay(2500); //delay 2500 milli seconds
}
