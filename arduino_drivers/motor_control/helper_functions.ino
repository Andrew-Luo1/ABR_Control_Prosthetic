const int potPinM3 = A0;
const int potPinM4 = A1;


void printVoltages(){
  Serial.print(analogRead(potPinM3));
  Serial.print(" ");
  Serial.println(analogRead(potPinM4));
}

void voltageToAngle(int num_prints){
  float m = 180.0/(M3Closed-M3Open);
  float b = m*M3Open;
  float curAngle = analogRead(potPinM3);
  float angle1 = m*curAngle - b;
  //Full DEBUG: 
//  Serial.print(analogRead(potPinM3));
//  Serial.print(" | ");
//  Serial.print(m);
//  Serial.print(" | ");
//  Serial.print(b);
//  Serial.print(" | ");
//  Serial.println(angle);

  //Repeat for index. 
  m = 180.0/(M4Closed-M4Open);
  b = m*M4Open;
  curAngle = analogRead(potPinM4);
  float angle2 = m*curAngle - b;
  for(int i = 0; i<num_prints; i++){
    Serial.print(angle1);
    Serial.print(" ");
    Serial.print(angle2);
    if (curShield=="thumb_index"){
      Serial.print(" ");
      Serial.println("1");
    }
    else{
      Serial.print(" ");
      Serial.println("2");
    }
  }
}
