String input="";
void setup() {
  Serial.begin(115200);
  pinMode(13, OUTPUT);

}

void loop() {
  if(Serial.available()>0){
    input=Serial.readStringUntil('\r');
    digitalWrite(13, HIGH);
    Serial.println("r");  
    delay(500);  
  }
  digitalWrite(13,LOW);
}
