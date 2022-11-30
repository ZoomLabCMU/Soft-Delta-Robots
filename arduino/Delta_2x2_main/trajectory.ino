void executeWaypoints(){
  float vmax = 25.0; //mm/s
  float amax = 25.0; //mm/s2
  float xf[NUM_MOTORS];
  
  if(traj_iter < 20 && go){
    sent_done = false;
    for(int i = 0; i < NUM_MOTORS; i++)
    {
      xf[i] = trajectory[traj_iter][i];
    }
    controller.ramp2pos(xf, vmax, amax);
    traj_iter++;
 
  }else if(traj_iter >= 20 && !sent_done){
    Serial.print(traj_iter);Serial.println(" No. of iters done! ");
    send_done_signal();
  }
}

void updateTrajectory(){
  traj_iter = 0;
  go = true;
}

void reset() {
  controller.reset_joints();
}
