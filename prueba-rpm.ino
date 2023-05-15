#define motorSTEPpin   33    // output senal para pulsar el driver y mover el motor
#define motorDIRpin    31    // output senal para la direccion del motor
#define potAnalogPin   A15

volatile int counterPrev, counter = 0; //This variable will increase or decrease depending on the rotation of encoder

#define encoderA 18
#define encoderB 19

#define pulsesPerRevolution 600  // pulsos del encoder. 


void setup() {
    Serial.begin(9600);

    pinMode (motorSTEPpin, OUTPUT);
    pinMode (motorDIRpin, OUTPUT);

    pinMode(encoderA, INPUT_PULLUP); 
    pinMode(encoderB, INPUT_PULLUP);

    //Setting up interrupt
    //A rising pulse from encodenren activated ai0().
    attachInterrupt(digitalPinToInterrupt(encoderA), ai0, RISING);
    //B rising pulse from encodenren activated ai1().
    attachInterrupt(digitalPinToInterrupt(encoderB), ai1, RISING);
}

void loop() {
    int potReading = analogRead(potAnalogPin);
    int runSpeed = map(potReading, 0, 1023, 50, 500); 
    
    move_motorRun(runSpeed, HIGH, '5');
}

void move_motorRun(int velocidad, boolean dir, char Page)  // page = 5 for run mode
{
    digitalWrite(motorDIRpin, dir);        // set Direction
  
    if (Page =='5')   
    {
        digitalWrite(motorSTEPpin,HIGH);  
        delayMicroseconds(velocidad);       // pulse the motor
        digitalWrite(motorSTEPpin,LOW);
        delayMicroseconds(velocidad);            // wait betweeen pulses
    }  
    
    return;  
}

void ai0() {
  // ai0 is activated if encoderA is going from LOW to HIGH
  // Check encoderB to determine the direction
  if (digitalRead(encoderB)==LOW) {
    counter++;
  } else {
    counter--;
  }
}
   

void ai1() {
  // ai0 is activated if encoderB is going from LOW to HIGH
  // Check with encoderA to determine the direction
  if (digitalRead(encoderA)==LOW) {
    counter--;
  } else {
    counter++;
  }
}