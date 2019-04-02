void ultrasonicsetup{
  pinMode(A0, OUTPUT);
  pinMode(A1, INPUT);
}

void ultrasonicloop() {
  currTime = micros();
  digitalWrite(A0, HIGH);
  if (currTime - lastTime > THRESHOLD) {
    digitalWrite(A0, LOW);
    ultraTime = pulseIn(A1, HIGH);
    distance = (ultraTime / 2 * 343) / 10000;
    lastTime = currTime;
    //Serial.println(distance);
  }
}
