void setup() {
  Serial.begin(9600);
  Serial.println("Serial monitor setup done");
  int score1 = 2;
  int score2 = 7;
  int score3 = 9;
  float averageScore;
  
  Serial.println(score1);
  Serial.println(score2);
  Serial.println(score3);
  
  averageScore = ((score1 + score2 +score3) / 3);
  Serial.println(averageScore);
}

void loop() {
  // put your main code here, to run repeatedly:

}
