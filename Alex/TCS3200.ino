
#define S0 7
#define S1 8
#define S2 9
#define S3 12
#define sensorOut 13
#define GREEN 1
#define RED 0

int frequencyR = 0;
int frequencyG = 0;

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
  // Setting red filtered photodiodes to be read
  digitalWrite(S2, LOW);
  digitalWrite(S3, LOW);

  // Reading the output frequency
  int three = 10;
  while (three--) {
    int frequency = 0;
    frequency = pulseIn(sensorOut, HIGH);

    //Remaping the value of the frequency to the RGB Model of 0 to 255
    frequencyR += map(frequency, 25, 72, 255, 0);
  }
  frequencyR = frequencyR / 10;

  // Setting Green filtered photodiodes to be read
  digitalWrite(S2, HIGH);
  digitalWrite(S3, HIGH);

  // Reading the output frequency
  three = 10;
  while (three--) {
    int frequency = 0;
    frequency = pulseIn(sensorOut, HIGH);

    //Remaping the value of the frequency to the RGB Model of 0 to 255
    frequencyG += map(frequency, 30, 90, 255, 0);
  }
  frequencyG = frequencyG / 10;

  if (frequencyG > frequencyR) {
    //Serial.println("GREEN");
    sendColour(GREEN);
  } else {
    //Serial.println("RED");
    sendColour(RED);
  }
}
