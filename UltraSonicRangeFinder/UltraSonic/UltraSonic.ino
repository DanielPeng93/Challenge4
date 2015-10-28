void setup() {
  // initialize serial communications
  Serial.begin(9600); 
}

void loop() {
   int sensor,inches,cm, x;
  
  // read the analog output of the EZ1 from analog input 0
  sensor = analogRead(3);
  
  // convert the sensor reading to inches
  inches = sensor / 2;

  //convert inches to centimeters
  cm = inches*2.54;
  
  // print out the decimal result
  Serial.print(cm,DEC);
  
  // print out a graphic representation of the result
  Serial.print(" ");
  for (x=0;x<(inches/5);x++)
  {
    Serial.print(".");
  }
  Serial.println("|");

  // pause before taking the next reading
  delay(100); 
}
