float sensorValue, cm;

void setup() {
  Serial.begin(9600);
  pinMode(A1, INPUT);
}

void loop() {
  sensorValue = analogRead(A1);
  cm = 10650.08 * pow(sensorValue, -0.935)-10;
  Serial.print("cm:");
  Serial.println(cm);
  delay(1000);
}
