//############################ SERIAL COMM FUNCTIONS #######################################3
void updateTrajectory(){
  traj_iter = 0;
  go = true;
}

void printIPAddr(){
  while (Serial1.available() > 0){
    char ch = Serial1.read();
    Serial.print(ch);
  }
}

void add_new_rc(byte rc,bool ifend){
  if(!ifend){
    marker_count++;
  }else{
    marker_count = 0;
  }
  input_cmd[ndx] = rc;
  ndx++;
  if (ndx >= numChars) {
    ndx = numChars - 1;
  }
}

void recvWithStartEndMarkers() {
  byte rc;
  while (Serial1.available() > 0 && newData == false) {
    rc = Serial1.read();
//    Serial.println(rc);
    if (recvInProgress == true) {
      if (rc == endMarker) {
        add_new_rc(rc, false);
      } 
      else if(rc == confMarker){
        add_new_rc(rc, false);
        if(marker_count == 3){
          marker_count = 0;
          input_cmd[ndx] = '\0'; // terminate the string
          recvInProgress = false;
          newData = true;
          ndxx = ndx - 3;
          Serial.println(ndx);
          ndx = 0;
        }
      } 
      else {
        add_new_rc(rc, true);
      }
    }

    else if (rc == startMarker) {
      marker_count++;
    }
    else if (rc == confMarker) {
      marker_count++;
      if(marker_count == 3){
        marker_count = 0;
        recvInProgress = true; 
      }
    }
    else{
      marker_count = 0;
      ndx = 0;
    }
  }
}

//
//void sendJointPositions(){
//  pb_ostream_t ostream = pb_ostream_from_buffer(input_cmd, sizeof(input_cmd));
//  message.id = MY_ID;
//  for(int i = 0; i < NUM_MOTORS; i++){
//    motor_val[i] = adcs[i]->readADC_SingleEnded(channels[i]); 
//    message.trajectory[i] = motor_val[i] * 0.00006; // 100mm / 1650
////    Serial.print(message.joint_pos[i]);
//  }
//  if (pb_encode(&ostream, DeltaMessage_fields, &message)){
//    Serial1.print("AT+CIPSEND=0,");Serial1.println(ostream.bytes_written +8);
//    delay(10);
//    printIPAddr();
//    delay(10);
//    Serial1.write("~~~");
//    Serial1.write(input_cmd, ostream.bytes_written);
//    Serial1.write("~~~\r\n");
//  }
//}

String data = "";
bool sent_done = false;

bool send_done_signal(){
  if(is_movement_done){
    data = "A\r\n";
  }else{
    data = "B\r\n";
  }
  Serial.println(data);Serial.println(data.length());
  Serial1.print("AT+CIPSEND=0,");Serial1.println(data.length());
  delay(10);
  Serial1.print(data);
  Serial1.println("AT+CIPCLOSE=0");
//  skip_trajectory = true;

  for(int i=0; i < sizeof(input_cmd); i++){
    input_cmd[i] = (uint8_t)0;
  }
  ndx = 0;
  sent_done = true;
  return true;
}


bool decodeNanopbData(){
  pb_istream_t istream = pb_istream_from_buffer(input_cmd, ndxx);
  bool ret = pb_decode(&istream, DeltaMessage_fields, &message);
  if (message.id == MY_ID){
    ret = true;
    skip_trajectory = false;
//    Serial.println("HAKUNA");Serial.println(message.request_done_state);
    if (message.request_done_state){
//      Serial.println("MATATA");
//      if(is_movement_done){
//        data = "A\r\n";
//      }else{
//        data = "B\r\n";
//      }
//      Serial.println(data);Serial.println(data.length());
//      Serial1.print("AT+CIPSEND=0,");Serial1.println(data.length());
//      delay(10);
//      Serial1.print(data);
//      Serial1.println("AT+CIPCLOSE=0");
//      skip_trajectory = true;
    }
    else if (message.reset){
      for (int i=0; i<NUM_MOTORS; i++){
        new_joint_positions[i] = 0.05;
      }
    }
    else{
      for(int i=0; i<20; i++){
        for(int j=0; j<12; j++){
          trajectory[i][j] = message.trajectory[i*12 + j];
//          Serial.print(message.trajectory[i*12 + j],4);Serial.print(" ");
        }
//        Serial.println();
      }
      for(int i=0; i < sizeof(input_cmd); i++){
        input_cmd[i] = (uint8_t)0;
      }
    }
    ndx = 0;
  }
  else{
    ndx = 0;
    Serial.println("FAILED HAKUNA");
    ret = false;
  }
  return ret;
}
