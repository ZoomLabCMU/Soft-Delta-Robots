void setup()
{
  Serial.begin(115200);
  delay(2000);
  Serial1.begin(115200);
  delay(2000);
  while (!Serial1)
    delay(10); // will pause Zero, Leonardo, etc until serial console opens
  Serial1.println("AT+GMR\r\n");
}
 
void loop(){
  if(Serial1.available()){
    while(Serial1.available()){
      char c = Serial1.read(); // read the next character.
      Serial.write(c);
    }
  }
  if(Serial.available()) // check if connection through Serial Monitor from computer is available
  {
    // the following delay is required because otherwise the arduino will read the first letter of the command but not the rest
    // In other words without the delay if you use AT+RST, for example, the Arduino will read the letter A send it, then read the rest and send it
    // but we want to send everything at the same time.
    delay(200); 
    
    String command="";
    
    while(Serial.available()) // read the command character by character
    {
      command+=(char)Serial.read();
    }
    Serial.println(command);
    Serial1.println(command); // send the read character to the Esp module
  }
  delay(1000);
  
}
