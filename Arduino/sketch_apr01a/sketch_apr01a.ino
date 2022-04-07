int counter = 0;
int counter2 = 0;

void isr() {
  counter++;
}

void isr_2() {
  counter2++;
}


void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  attachInterrupt(digitalPinToInterrupt(21), isr, RISING);
  attachInterrupt(digitalPinToInterrupt(20), isr_2, RISING);
  pinMode(8, INPUT);
  pinMode(9, INPUT);
}

void loop() {
  // put your main code here, to run repeatedly:
  Serial.print(counter);
  Serial.print("//////");
  Serial.print(counter2);
  Serial.print("//////");
  Serial.print(digitalRead(9));
  Serial.print("//////");
  Serial.println(digitalRead(8));
}
