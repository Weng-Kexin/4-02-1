void IRsetup(){
  pinMode(LEFT_IR, INPUT);
  pinMode(RIGHT_IR, INPUT);
}

void infraRed() {
  LEFT_OBSTACLE = analogRead(LEFT_IR);
  RIGHT_OBSTACLE = analogRead(RIGHT_IR);

//  Serial.print("LEFT_OBSTACLE : ");
//  Serial.println(LEFT_OBSTACLE);
//  Serial.print("RIGHT_OBSTACLE : ");
//  Serial.println(RIGHT_OBSTACLE);
  
  if (LEFT_OBSTACLE < 500 || RIGHT_OBSTACLE < 500) {
//    Serial.println("HI HARRY");
    stop_while_forward = true;
  } else {
//    Serial.println("BYE HARRY");
    stop_while_forward = false;
  }
}
