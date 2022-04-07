//2.0 was first successful run using basic ideas for the capstone project
//2.1 is testing with 2 Arduinos, printing 2 encoder values. Max baudrate
//3.0 is with esp and Arduino Mega. Max baudrate that works (so far). Loop time is 4-5ms
//Decrements and sends 4 pulse values
//Currently 4.0 is the sameas 3.0
//5.0 implements pulse value resets. Resets after printing (send a 1 to esp)

//Pin Definitions
#define RX 26
#define TX 25
#define FL_A 21
#define FL_B 22
#define FR_A 23
#define FR_B 24

//Arduino comms variables
const char startOfNumberDelimiter = '<';
const char endOfNumberDelimiter = '>';
int pulsesFL_A = 0;
int pulsesFR_A = 0;
int dirFL = 0;
int dirFR = 0;

void isr_FLA() {
  pulsesFL_A++;
  if (digitalRead(FL_B) == LOW) {
    dirFL = 1;
  }
  else {
    dirFL = 2;
  }
}

void isr_FRA() {
  pulsesFR_A++;
  if (digitalRead(FR_B) == LOW) {
    dirFR = 1;
  }
  else {
    dirFR = 2;
  }
}


void setup() {
  // put your setup code here, to run once:
  Serial1.begin(57600, SERIAL_8N1, RX, TX);
  attachInterrupt(digitalPinToInterrupt(FL_A), isr_FLA, RISING);
  attachInterrupt(digitalPinToInterrupt(FR_A), isr_FRA, RISING);
  pinMode(FL_B, INPUT);
  pinMode(FR_B, INPUT);

}

void processNumber (const long n) {
  //Serial.println(n);
  if (n == 49) {
    Serial1.print(startOfNumberDelimiter);
    Serial1.print(dirFL);
    Serial1.print(endOfNumberDelimiter);

    Serial1.print(startOfNumberDelimiter);
    Serial1.print(pulsesFL_A);
    Serial1.print(endOfNumberDelimiter);

    Serial1.print(startOfNumberDelimiter);
    Serial1.print(dirFR);
    Serial1.print(endOfNumberDelimiter);

    Serial1.print(startOfNumberDelimiter);
    Serial1.print(pulsesFR_A);
    Serial1.print(endOfNumberDelimiter);
    Serial1.println();

    pulsesFL_A = 0;
    pulsesFR_A = 0;
  }
}  // end of processNumber

void loop() {
  if (Serial1.available()) {
    static long receivedNumber = 0;

    byte c = Serial1.read ();
    //Serial.println(c);
    switch (c)
    {

      case 62://endOfNumberDelimiter:
        processNumber (receivedNumber);

      // fall through to start a new number
      case 60://startOfNumberDelimiter:
        receivedNumber = 0;
        break;

      case 49 ... 50://'1' ... '2':
        receivedNumber = c;
        break;

    } // end of switch
  }
}
