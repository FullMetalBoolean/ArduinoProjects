void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600); 
  Serial.println("Serial monitor setup done");
  int tempFahrenheit = 92; //given temperature in prompt
  int tempCelsius = (tempFahrenheit - 32) * 5/9; //formula to conver F to C, uses modulus becuase most weather reports dont include degrees and thier decimals 
  Serial.print(tempFahrenheit); //print lines to output the conversion to text
  Serial.print( " Degrees Fahrenheit equates to ");
  Serial.print(tempCelsius);
  Serial.print(" Celsisus!");
}

void loop() {
  // put your main code here, to run repeatedly:

}
