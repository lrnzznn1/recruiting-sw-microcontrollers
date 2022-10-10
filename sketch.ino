const byte pinled1 = 13;
const byte pinled2 = 12;
const byte pinInputSensor = 15;
const float BETA = 3950;
void setup() {
  //Display setup
Serial.begin(115200);

  //Led setup
  pinMode(pinled1,OUTPUT);
  pinMode(pinled2, OUTPUT);
  
  //Input setup

}

void loop() {
  
  digitalWrite(pinled1, HIGH);
  Serial.println("HIGH");
  delay(500);
  digitalWrite(pinled1,LOW);
  Serial.println("LOW");
  delay(500);
  
  float analogValue = analogRead(pinInputSensor);
  Serial.print("Temperature: ");
  Serial.print(analogValue);
  Serial.println(" ℃");
  delay(1000);
  
  
}
