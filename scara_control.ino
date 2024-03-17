#include <AccelStepper.h>
#include <MultiStepper.h>

#define trigPin 13// Define the trig pin
// Define the echo pin
#define relayPin A5 // Define the relay control pin
int x = 0;
bool loopExecuted = false;

#define limitSwitch1 11
#define limitSwitch2 10
#define limitSwitch3 9
#define limitSwitch4 13
#define limitSwitch5 12
int y = 2;

char c;




int data;
// Define the stepper motor and the pins that is connected to
AccelStepper stepper1(1, 2, 5); // (Typeof driver: with 2 pins, STEP, DIR)
AccelStepper stepper2(1, 3, 6);
AccelStepper stepper3(1, 4, 7);


MultiStepper steppersControl;  // Create instance of MultiStepper

long gotoposition[3]; // An array to store the target positions for each stepper motor

unsigned long startTime;

void setup() {
   Serial.begin(115200);

   startTime = millis();

   
  stepper1.setMaxSpeed(500);
   pinMode(limitSwitch1, INPUT_PULLUP);
   pinMode(limitSwitch2, INPUT_PULLUP);
   pinMode(limitSwitch3, INPUT_PULLUP);
   pinMode(limitSwitch4, INPUT_PULLUP);
   pinMode(limitSwitch5, INPUT_PULLUP);
   
  pinMode(relayPin, OUTPUT);
  digitalWrite(relayPin, HIGH); // Initialize the relay as off

  // Set the maximum speed and acceleration for each motor
  stepper1.setMaxSpeed(600); // Adjust to your motor's specs
  stepper1.setAcceleration(600);
stepper2.setMaxSpeed(600); // Adjust to your motor's specs
  stepper2.setAcceleration(600);

    stepper3.setMaxSpeed(1000);
    stepper3.setAcceleration(800);

    int x = 0;

  // Adding the 3 steppers to the steppersControl instance for multi stepper control
  steppersControl.addStepper(stepper1);
  steppersControl.addStepper(stepper2);
  steppersControl.addStepper(stepper3);
  float duration, distance;
//  
//  // Trigger the ultrasonic sensor
//  
 

while(digitalRead(limitSwitch3) != 1) {
  stepper3.setSpeed(800);
  stepper3.runSpeed();
  stepper3.setCurrentPosition(0);
     
}
while (digitalRead(limitSwitch1) != 1) {
  stepper1.setSpeed(-300);
  stepper1.runSpeed();
  stepper1.setCurrentPosition(0);
 
  
    
}

while (digitalRead(limitSwitch2) != 1) {
  stepper2.setSpeed(500);
  stepper2.runSpeed();
  stepper2.setCurrentPosition(0);
     
}



delay(1000);





}



void loop() {
if(x == 0){
   stepper1.moveTo(1200); // Adjust the desired distance
  stepper2.moveTo(-100);
  while (stepper1.run() || stepper2.run()) {}
    Serial.write('A');

 unsigned long startTime = millis();
  while (Serial.available() == 0 && millis() - startTime < 1000); // Wait for up to 1 second
  if (Serial.available() > 0 && Serial.read() == '1') {
    // Perform a function
    nesh();

    // Send 'B' serially
    Serial.write('B');
    x += 1;
  }

  // Clear the serial buffer
  while (Serial.available() > 0) {
    Serial.read();
  }
}
delay(1000);

if( x == 1){
stepper1.moveTo(1200); // Adjust the desired distance
  stepper2.moveTo(-500);
  while (stepper1.run() || stepper2.run()) {}
    Serial.write('A');

    
  while (Serial.available() == 0 && millis() - startTime < 1000); // Wait for up to 1 second
  if (Serial.available() > 0 && Serial.read() == '1') {
    // Perform a function
    nesh();

    // Send 'B' serially
    Serial.write('B');
  }

  // Clear the serial buffer
  while (Serial.available() > 0) {
    Serial.read();
    x += 1;
  }
}


    
}

void pickPlace(){
  Serial.write('A');
   if(Serial.available() > 0){
  char c = Serial.read();
  delay(1500);
  
  if(c == '1'){
    nesh();
    Serial.write('B');
  }
}

}




void wayPoints() {

//set1

   // Move forward along the first side of the square
  stepper1.moveTo(1200); // Adjust the desired distance
  stepper2.moveTo(-100);
  while (stepper1.run() || stepper2.run()) {}
  pickPlace();


while(digitalRead(limitSwitch3) != 1) {
  stepper3.setSpeed(800);
  stepper3.runSpeed();
  stepper3.setCurrentPosition(0);
     
}
while (digitalRead(limitSwitch1) != 1) {
  stepper1.setSpeed(-300);
  stepper1.runSpeed();
  stepper1.setCurrentPosition(0);
 
  
    
}

while (digitalRead(limitSwitch2) != 1) {
  stepper2.setSpeed(500);
  stepper2.runSpeed();
  stepper2.setCurrentPosition(0);
     
}






}


void nesh(){


  stepper1.stop();
  stepper2.stop();
  digitalWrite(relayPin, LOW);
  int state1 = digitalRead(limitSwitch4);
  int state2 = digitalRead(limitSwitch5);
 while(digitalRead(limitSwitch5) != 1) {
  stepper3.setSpeed(-800);
  stepper3.runSpeed();
 
}

while(digitalRead(limitSwitch3) != 1) {
  stepper3.setSpeed(1000);
  stepper3.runSpeed();
 
     
}

// Set the maximum speed and acceleration for each motor

  stepper1.moveTo(1500);
  stepper2.moveTo(-1100);
  while (stepper1.run() || stepper2.run()) {}


digitalWrite(relayPin, HIGH);

}
