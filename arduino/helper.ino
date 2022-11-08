
//float p = 300.0;
//float i_pid = 1.2;
//float d = 5;
//float p = 550;
//float i_pid = 0;
//float d = 15;
float p = 450.0;
float i_pid = 0;
float d = 0.2;

float position_threshold = 0.00045;
//############################# READ / WRITE JOINT POSITIONS #######################################3
void printJtPos(){
  for(int i = 0; i < NUM_MOTORS; i++)
  {
    Serial.print(new_joint_positions[i]);
  }
  Serial.println();
}

void executeTrajectory(){
  if(traj_iter < 20 && go){
    sent_done = false;
//    Serial.println(ndx);
    for(int i = 0; i < NUM_MOTORS; i++)
    {
      new_joint_positions[i] = trajectory[traj_iter][i];
    }
    writeJointPositions();
    traj_iter += 1; 
  }else if(traj_iter >= 20 && !sent_done){
    Serial.print(traj_iter);Serial.println(" No. of iters done! ");
    send_done_signal();
  }
}

void readJointPositions(){
  for(int i = 0; i < NUM_MOTORS; i++){
    motor_val[i] = adcs[i]->readADC_SingleEnded(channels[i]); 
    joint_positions[i] = motor_val[i] * 0.00006; // 100mm / 1650
  }
}

void writeJointPositions(){
//  printJtPos();
  bool reached_point = false;
  last_arduino_time = millis();
  is_movement_done = false;
  exec_time = millis()/1000;
  while (!reached_point){
    current_arduino_time = millis();
    time_elapsed = float(current_arduino_time - last_arduino_time)/1000;
//    last_arduino_time = current_arduino_time;
    readJointPositions();
    reached_point = true;
    for(int i = 0; i < NUM_MOTORS; i++){
      joint_errors[i] = joint_positions[i] - new_joint_positions[i];
      
//      float pid = p * joint_errors[i] + i_pid * total_joint_errors[i] + d * (joint_errors[i] - last_joint_err/ors[i]/time_elapsed);
      float pid = p * joint_errors[i] + i_pid * total_joint_errors[i] + d * (joint_errors[i] - last_joint_errors[i])/time_elapsed;
//      Serial.print(d * (joint_errors[i] - last_joint_errors[i])/time_elapsed);

//##########################################################################
      if (joint_errors[i] > position_threshold){
        reached_point &= false;
        int motor_speed = (int)(min(max(0.0, pid), 1.0) * 255.0); //change it to use duration input
        motors[i]->setSpeed(motor_speed);
        motors[i]->run(BACKWARD);
        total_joint_errors[i] += joint_errors[i];
      }else if (joint_errors[i] < -position_threshold){
        reached_point &= false;
        int motor_speed = (int)(min(max(-1.0, pid), 0.0) * -255.0);
        motors[i]->setSpeed(motor_speed);
        motors[i]->run(FORWARD);
        total_joint_errors[i] += joint_errors[i];
      }else{
        last_joint_errors[i] = 0;
        reached_point &= true;
        motors[i]->setSpeed(0);
        motors[i]->run(RELEASE);
        total_joint_errors[i] = 0.0;
      }

//      ###################################################
//      int motor_speed = (int)(min(max(-255.0, pid* 255.0), 255.0));
//      motors[i]->setSpeed(abs(motor_speed));
//      total_joint_errors[i] += joint_errors[i];
//      last_joint_errors[i] = joint_errors[i];
//
//      if(abs(joint_errors[i]) < position_threshold){
//        last_joint_errors[i] = 0;
//        reached_point &= true;
//        motors[i]->setSpeed(0);
//        motors[i]->run(RELEASE);
//        total_joint_errors[i] = 0.0;
//      }
//      else if(motor_speed>0){
//        reached_point &= false;
//        motors[i]->run(BACKWARD);
//      }
//      else{
//        reached_point &= false;
//        motors[i]->run(FORWARD);
//      }
//      ###########################################
    }
    
    if ((millis()/1000 - exec_time) > 3){
      for(int i = 0; i < 12; i++){
        last_joint_errors[i] = 0;
        reached_point &= true;
        motors[i]->setSpeed(0);
        motors[i]->run(RELEASE);
        total_joint_errors[i] = 0.0;
      }
    }
  }
  for(int i = 0; i < 12; i++){
      motors[i]->setSpeed(0);
      motors[i]->run(RELEASE);
      total_joint_errors[i] = 0.0;
  }
  is_movement_done = true;
   
//  Serial.println("Moved to New Position");
}

//########################### STOP OR RESET FUNCTIONS ######################################3
void resetJoints(){
  for(int i = 0; i < NUM_MOTORS; i++)
  {
    new_joint_positions[i] = 0.0;
  }
  writeJointPositions();
}

void stop(){
  // Turn off all motors
  for(int i = 0; i < NUM_MOTORS; i++)
  {
    motors[i]->run(RELEASE);
  }
}
