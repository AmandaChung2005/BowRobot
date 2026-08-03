/*   
 *   Control a stepper motor for impact hammer measurements with a PCB 086E80. 
 *
 *   Useful resources:
 *   https://howtomechatronics.com/tutorials/arduino/stepper-motors-and-arduino-the-ultimate-guide/
 *   
 *   Mark Rau - mrau@ccrma.stanford.edu
 *   June, 2024
 */

#include <AccelStepper.h>
const int buttonPin = 7; // button pin for the hammer trigger
const int potentiometerPin = A0; // potentiometer pin for the pullback amount adjustment
const int currentControlPin = 9; // current control of the stepper motor driver

// Define the stepper motor and the pins that is connected to
AccelStepper stepper1(1, 2, 5); // (Type of driver: with 2 pins, STEP, DIR)

int buttonState = 0; // State of the trigger button

int potentiometerVal = 0;

void setup() {
  // Set maximum speed value for the stepper
  stepper1.setMaxSpeed(20000);
  stepper1.setAcceleration(1000000);
  stepper1.setCurrentPosition(0);
  Serial.begin(115200); // open the serial port at 115200 bps:

  analogWrite(currentControlPin,round(255.0*0.2/5.0));
  
}

void loop() {

  // Initial button state
  buttonState = digitalRead(buttonPin);

  // Read the potentiometer value to set the pullback amount
  potentiometerVal = analogRead(potentiometerPin);

  
  // If the button is pressed, trigger the hammer
  if (buttonState == HIGH){
    analogWrite(currentControlPin,round(255.0*1.8/5.0)); // Set the hammer current higher during the impact
    delay(500);
    stepper1.setCurrentPosition(0);
    stepper1.setSpeed(8000); // Speed of the initial hammer pullback
    
    // stepper1.setSpeed(2000);

    stepper1.moveTo(int(200.0*float(potentiometerVal)/1023.0)); // Set the potentiometer pullback amount
    stepper1.runSpeedToPosition();

    while ((stepper1.distanceToGo() > 0)) {
      stepper1.runSpeedToPosition();
      
    }

    stepper1.setSpeed(18000); // Sped of the hammer strike 
    stepper1.moveTo(-20);

    while ((stepper1.distanceToGo() < 0)) {
      stepper1.runSpeedToPosition();
    }

  
    stepper1.moveTo(0);

    while ((stepper1.distanceToGo() >0 )) {
      stepper1.runSpeedToPosition();
    }



    buttonState = LOW; // Reset the button state to low adter the impact is done
    analogWrite(currentControlPin,round(255.0*0.2/5.0)); // Set the hammer current low between impacts so the motor doesn't heat up but still stays in the same place

  }

  // Serial Command Functionality for hammer
  if (Serial.available() > 0) {
    // read the incoming byte:
    char commandChar = Serial.read();
    int commandInt = Serial.parseInt();

    


    if (commandChar == 's' && commandInt == 1) {
      // If the commandChar == 's', impact the system with the hammer. Note, this is a simple setup where the hammer is suspended like a pendulum and the stepper motor just moves it back to strat a free-fall excitation.
      analogWrite(currentControlPin,round(255.0*1.8/5.0)); // Set the hammer current higher during the impact
      delay(500);
      stepper1.setCurrentPosition(0);
      stepper1.setSpeed(8000); // Speed of the initial hammer pullback
      
      // stepper1.setSpeed(2000);

      stepper1.moveTo(int(200.0*float(potentiometerVal)/1023.0)); // Set the potentiometer pullback amount
      stepper1.runSpeedToPosition();

      while ((stepper1.distanceToGo() > 0)) {
        stepper1.runSpeedToPosition();
        
      }

      stepper1.setSpeed(18000); // Sped of the hammer strike 
      stepper1.moveTo(-20);

      while ((stepper1.distanceToGo() < 0)) {
        stepper1.runSpeedToPosition();
      }

    
      stepper1.moveTo(0);

      while ((stepper1.distanceToGo() >0 )) {
        stepper1.runSpeedToPosition();
      }



      buttonState = LOW; // Reset the button state to low adter the impact is done
      analogWrite(currentControlPin,round(255.0*0.2/5.0)); // Set the hammer current low between impacts so the motor doesn't heat up but still stays in the same place

      delay(1);
    }

  delay(1);
  }  






  
}