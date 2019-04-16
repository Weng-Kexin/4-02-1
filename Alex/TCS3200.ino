#define S0 7
#define S1 8
#define S2 9
#define S3 12
#define sensorOut 13
#define GREEN 1
#define RED 0
#define WHITE 2
int frequencyR = 0;
int frequencyG = 0;
int frequencyB = 0;

void coloursetup() {
  pinMode(S0, OUTPUT);
  pinMode(S1, OUTPUT);
  pinMode(S2, OUTPUT);
  pinMode(S3, OUTPUT);
  pinMode(sensorOut, INPUT);

  // Setting frequency-scaling to 20%
  digitalWrite(S0, HIGH);
  digitalWrite(S1, LOW);
}

void findColour() {
  digitalWrite(S2, LOW); //turn on blue photodiode
  digitalWrite(S3, HIGH);

  double three = 3;
  while (three--) {
    int frequency = 0;
    frequency = pulseIn(sensorOut, LOW);

    frequencyB += map(frequency, 20, 230, 255, 0);
//    frequencyB += frequency;
  }
  frequencyB = frequencyB / 3.0;
//  Serial.print("BLUE: "); Serial.println(frequencyB);

  delay(100);
  // Setting red filtered photodiodes to be read
  digitalWrite(S2, LOW);
  digitalWrite(S3, LOW);

  // Reading the output frequency
  three = 3;
  while (three--) {
    int frequency = 0;
    frequency = pulseIn(sensorOut, LOW);

    //Remaping the value of the frequency to the RGB Model of 0 to 255
    frequencyR += map(frequency, 20, 230, 255, 0);
//    frequencyR += frequency;
  }
  frequencyR = frequencyR / 3.0;
//  Serial.print("RED: "); Serial.println(frequencyR);

  // Setting Green filtered photodiodes to be read
  digitalWrite(S2, HIGH);
  digitalWrite(S3, HIGH);

  delay(100);

  // Reading the output frequency
  three = 3;
  while (three--) {
    int frequency = 0;
    frequency = pulseIn(sensorOut, LOW);

    //Remaping the value of the frequency to the RGB Model of 0 to 255
    frequencyG += map(frequency, 20, 300, 255, 0);
//   frequencyG += frequency;
  }
  frequencyG = frequencyG / 3.0;
//  Serial.print("GREEN: "); Serial.println(frequencyG);

  if (frequencyG > 250 && frequencyR > 250 && frequencyB > 250) {
//    Serial.println("WHITE");
    sendColour(WHITE);
  } else if (frequencyG > frequencyR){
//    Serial.println("GREEN");
    sendColour(GREEN);
  }
  else
  {
//    Serial.println("RED");
    sendColour(RED);
  }
}
