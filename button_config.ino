/* FSR testing sketch. 
 
Connect one end of FSR to 5V, the other end to Analog 0.
Then connect one end of a 10K resistor from Analog 0 to ground
Connect LED from pin 11 through a resistor to ground 
 
For more information see www.ladyada.net/learn/sensors/fsr.html */
#define BUTTON_PIN 4
int fsrAnalogPin1 = 0;
int fsrAnalogPin2 = 1;
int fsrReading1;
int fsrReading2;           // the analog reading from the FSR resistor divider
#define stepPin 7
#define dirPin 8
int modifyable_delay=500;
const char open = 31;
const char close = 32;
bool pressed= false;
bool check = false;
bool bpressed_both=false;
void setup(void) {
  Serial.begin(115200);   // We'll send debugging information via the Serial monitor
  pinMode(stepPin, OUTPUT);
  pinMode(dirPin, OUTPUT);
  // Serial.begin(115200);
  pinMode(open, INPUT_PULLUP);
  pinMode(close, INPUT_PULLUP);
}
 
void loop(void) {  
  // byte buttonState = digitalRead(open);
  bool buttonopenstate = digitalRead(open);
  bool buttonclosestate = digitalRead(close);

  
  if (buttonopenstate == pressed  ) 
  {
    stepper_clockwise();
    while(digitalRead(open)==pressed);
  }

  if (buttonclosestate == pressed) 
  {
    check = true;
  }

  if (check == true){
    run();
    if (buttonopenstate == pressed) 
  {
    check= false;
  }

  }
}



void run(){
    stepper_anticlockwise();
    fsrReading1 = analogRead(fsrAnalogPin1);
    Serial.print("Analog reading1 = ");
    Serial.println(fsrReading1);
    fsrReading2 = analogRead(fsrAnalogPin2);
    Serial.print("Analog reading2 = ");
    Serial.println(fsrReading2);
    modifyable_delay=2000+((fsrReading1+fsrReading2)*1000);
    if ((fsrReading2>=120) || (fsrReading1>=120))
    {
      stepper_stop();
      check = false;
    }

    else
    {
      digitalWrite(dirPin, HIGH);
      // Spin motor slowly
      for(int x = 0; x <10; x++)
      {
        digitalWrite(stepPin, HIGH);
        delayMicroseconds(modifyable_delay);
        digitalWrite(stepPin, LOW);
        delayMicroseconds(modifyable_delay);
      }
    }
}


void stepper_clockwise()
{
  digitalWrite(dirPin, HIGH);
  // Spin motor slowly
  for(int x = 0; x < 200; x++)
  {
    digitalWrite(stepPin, HIGH);
    delayMicroseconds(2000);
    digitalWrite(stepPin, LOW);
    delayMicroseconds(2000);
  }
}

void stepper_anticlockwise()
{
  digitalWrite(dirPin, LOW);

  // Spin motor slowly
  for(int x = 0; x < 200; x++)
  {
    digitalWrite(stepPin, HIGH);
    delayMicroseconds(2000);
    digitalWrite(stepPin, LOW);
    delayMicroseconds(2000);
  }
}

void stepper_stop()
{
  digitalWrite(dirPin, LOW);

  // Spin motor slowly
  for(int x = 0; x < 200; x++)
  {
    digitalWrite(stepPin, LOW);
    delayMicroseconds(2000);
    digitalWrite(stepPin, LOW);
    delayMicroseconds(2000);
  }
}