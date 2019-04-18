void IRsetup(){
  pinMode(LEFT_IR, INPUT);
  pinMode(RIGHT_IR, INPUT);
}

void infraRed() {
  LEFT_OBSTACLE = analogRead(LEFT_IR);
  RIGHT_OBSTACLE = analogRead(RIGHT_IR);
  
  if (LEFT_OBSTACLE < 500 || RIGHT_OBSTACLE < 500) {
    stop_while_forward = true;
  } else {
    stop_while_forward = false;
  }
}
