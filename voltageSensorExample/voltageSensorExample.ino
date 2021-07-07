/*

OSEPP Voltage sensor Example 

This module is based on the principle of the resistor divider design, enabling the terminal interface 
input voltage reduced five times,  analog input voltage up to 5V, then the voltage detection module 
can not be greater than the input voltage of 5V × 5 = 25V (3.3V if used system, the input voltage can not
exceed 3.3Vx5 = 16.5V). Because  AVR chips used in 10 AD, so this module's analog resolution 0.00489V (5V/1023),
so the input voltage detection module detects a minimum voltage of 0.00489V × 5 = 0.02445V.

*/

int voltageVal;
int calcVoltageVal;
int LED1 = 13;
 
void setup ()
{
  pinMode (LED1, OUTPUT);
  Serial.begin (9600);
}
void loop ()
{
  float temp;
  voltageVal = analogRead (0); //read the value from voltage sensor.
  temp = voltageVal/4.092; //resolution value to store
  voltageVal = (int) temp ;
 calcVoltageVal = ((voltageVal% 100) / 10); //calculate stepped down voltage
    Serial.println ("Voltage: ");
  Serial.println (calcVoltageVal); //print out voltage readings.
  delay (1000);
}
