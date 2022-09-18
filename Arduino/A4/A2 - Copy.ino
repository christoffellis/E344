#include "BluetoothSerial.h"

const int ledPin = 23;
const int freq = 16;
const int ledChannel = 0;
const int resolution = 16;

#define Trigger1 2  // pin 2 will flash for debug only
#define LED 

// Check if Bluetooth configs are enabled
#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error Bluetooth is not enabled! Please run `make menuconfig` to and enable it
#endif

// Bluetooth Serial object
BluetoothSerial SerialBT;

// Handle received and sent commands
String command = "";
char incomingChar;

// Timer: auxiliar variables
unsigned long previousMillis = 0;    // Stores last time temperature was published
const long interval = 10000;         // interval at which to publish sensor readings


void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200); // uart setup
  pinMode(Trigger1,OUTPUT); //set up pin 2 as output
  ledcSetup(ledChannel, freq, resolution); //set pwn freq and resulution
  ledcAttachPin(ledPin, ledChannel); //select pwm outpit pin and channel
  ledcWrite(ledChannel, 11); // set dutycycle to 11/65536 (10u out of 62.5ms)

  SerialBT.begin("ESP32");
  Serial.println("The device started, now you can pair it with bluetooth!");
}


void loop() {
  // This is just for debug. LED flashes and write to uart.
  delay(500);
  digitalWrite(Trigger1,HIGH);
  delay(500);
  digitalWrite(Trigger1,LOW);
  //Serial.println("Heart-beat");

  // Controller controls the wheels and reports back the measurements
  Controller();
  
  // Read incoming bluetooth signals
  if (SerialBT.available()){
    while (1) {
      char incomingChar = SerialBT.read();
      if (incomingChar != '\n'){
        command += String(incomingChar);
      }
      else{
        if (command != "") {
          SerialBT.println("Command is '" + command + "'");
          Serial.println("Command is '" + command + "'");
          handleCommand(command);
        }
        command = "";
        break;
      }
    }

  }
}

int rightWheelSpeed = 500;
int leftWheelSpeed = 500;
int wheelBias = 0;

void handleCommand(String cmd){
  command = cmd;

  if (cmd == "left") {
    wheelBias = 50;
  } else 
  if (cmd == "right") {
    wheelBias = -50;
  } else
  if (cmd == "staight") {
    wheelBias = 0;
  }
  Serial.println("Set bias to:");
}
void Controller() {
  int speedDifference = getRightWheelSpeed() - getLeftWheelSpeed()- wheelBias;
  
  if (speedDifference > 0)
  {
    changeRightWheelSpeed(speedDifference);
  }
  else if (speedDifference < 0) {
    changeLeftWheelSpeed(speedDifference);
  }
    Serial.println("====================");
    Serial.println("Right Wheel Speed: ");
    Serial.println(rightWheelSpeed);
    Serial.println("Left Wheel Speed: ");
    Serial.println(leftWheelSpeed);
}

void changeRightWheelSpeed(int difference) {
  rightWheelSpeed += difference;
}

void changeLeftWheelSpeed(int difference) {
  leftWheelSpeed += difference;
}

int getRightWheelSpeed() {
  return rightWheelSpeed;
}

int getLeftWheelSpeed() {
  return leftWheelSpeed;
}
