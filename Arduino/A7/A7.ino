
#include "BluetoothSerial.h"

#define DEBUG //comment out to remove debug

#define studentID  123
#define passKey 111
#define c_timeout 5000 //ms


/* ***************** */
// protocol definitions
/* ***************** */
#define cmd_set 0
#define cmd_get 1

//get commands
#define prot_get_rightWheelCurrent 0
#define prot_get_rightRange 1
#define prot_get_rightWheelNibble 2 //just information back to operator

#define prot_get_leftWheelCurrent 11
#define prot_get_leftRange 12
#define prot_get_leftWheelNibble 13 //just information back to operator

#define prot_get_batteryVoltage 20
#define prot_get_accellerometer 30

#define prot_get_everything 40

//set commands lengths
#define prot_set_rightWheelSpeed 50
#define prot_set_leftWheelSpeed 51
#define prot_set_bothWheelSpeed 52

//message lengths
#define prot_len_rightWheelCurrent 1*2  //two bytes, one number 
#define prot_len_rightWheelNibble 1  //two bytes, one number 
#define prot_len_getEverything  18//RWCur=2,RWRange=2, RWnibble=1, LWCur=2, LWRange=2,LWNibble=1, battery=2, accel=6
/* ***************** */

/* ***************** */
//pinout definitions
/* ***************** */
#define apin_rightWheelCurrent 27 //
#define apin_rightWheelRange 26 // Done
#define apin_leftWheelCurrent 25 //Done
#define apin_leftWheelRange 19 //Done
#define apin_batteryVoltage 2 //Done
#define apin_accelerometerX 17 //Done - Not ADC pin
#define apin_accelerometerY 5 //Done - Not ADC pin

#define apin_DAC_1 34 //Done
#define apin_DAC_2 35 //Done
#define apin_DAC_4 32 //Done
#define apin_DAC_8 33 //Done

#define trigger_pin 23 //Done
#define pwm_pin 14 //Done

#define trigger_freq 16
#define trigger_chan 0
#define trigger_resolution 16

#define pwm_freq 1000
#define pwm_chan 2
#define pwm_resolution 8

/* ***************** */

#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error Bluetooth is not enabled! Please run `make menuconfig` to and enable it
#endif

enum rxState_t {s_idle,s_pass,s_len, s_receiving};
enum cmdState_t {s_waiting, s_received, s_response};

BluetoothSerial SerialBT;
char rxBuffer[10];
char txBuffer[20];
int cntRx = 0; //receive counter
//int cntPrc = 0;  //processing pointer
int receivedByte;
int returnValue;  //
int lastPrintTime = 0;


unsigned long timeoutStart = 0;
char packetLen = 0;
rxState_t  rxState = s_idle;
cmdState_t cmdState = s_waiting;

//wheelspeeds - four-bit nibbles
char rightWheelSpeed = 100;
char leftWheelSpeed = 100;

int leftWheelRNG = 0;
 
//function prototypes
void rxStateMachine(int receivedByte);
void cmdStateMachine();
void debugPrint(char* debugStr1, char* valPtr);

//setup
void setup() {
  Serial.begin(115200);
  SerialBT.begin("Chris Car"); //Bluetooth device name
  Serial.println("The device started, now you can pair it with bluetooth!");

  // Set up DAC pins
  pinMode(apin_DAC_1, OUTPUT);
  pinMode(apin_DAC_2, OUTPUT);
  pinMode(apin_DAC_4, OUTPUT);
  pinMode(apin_DAC_8, OUTPUT);

  // Set up Trigger pin
  ledcSetup(trigger_chan, trigger_freq, trigger_resolution); //set pwn freq and resulution
  ledcAttachPin(trigger_pin, trigger_chan); //select pwm outpit pin and channel
  ledcWrite(trigger_chan, 11); // set dutycycle to 11/65536 (10u out of 62.5ms)

  //pinMode(pwm_pin, OUTPUT);
  //digitalWrite(pwm_pin, HIGH);
  // Set up PWM pin
  ledcSetup(pwm_chan, pwm_freq, pwm_resolution); //set pwn freq and resulution
  ledcAttachPin(pwm_pin, pwm_chan); //select pwm outpit pin and channel

  attachInterrupt(apin_leftWheelRange, setPinTime, CHANGE);
}

void loop() {
  if (SerialBT.available()) { 
      debugPrint("I have received a byte!",NULL); 
      rxStateMachine(SerialBT.read());
  }
  cmdStateMachine();
  outputDigitalWheelControl(leftWheelRNG/1000.0, leftWheelSpeed);
  // wheelCurrentControl();
  
  //rx timeout check
  if (rxState != s_idle)
    if ((millis() - timeoutStart) > c_timeout)  {
    debugPrint("Timeout on receive! Resetting to idle",NULL);
    rxState = s_idle;
  }
  if (millis() > lastPrintTime + 5000) {
    debugPrint("Lub-dub", NULL);
    lastPrintTime=millis();
  } 
}

void cmdStateMachine()
{   
  char debugchar;
  if(cmdState == s_received){
     debugPrint("Command received: ", &rxBuffer[0]);
    switch(rxBuffer[0]) { //evaluate type of message get/set
      case cmd_set: {
          debugPrint("Set command received: ",&rxBuffer[2]); 
          switch(rxBuffer[1]) {
            case prot_set_rightWheelSpeed: 
              rightWheelSpeed = (rxBuffer[2])&0xff;
              debugPrint("Right wheel speed now: ",&rightWheelSpeed);
              outputAnalogueWheelControl(rightWheelSpeed);
              break;
            case prot_set_leftWheelSpeed:
              leftWheelSpeed = (rxBuffer[2])&0xff;
              debugPrint("Left wheel speed now: ",&leftWheelSpeed);
              outputDigitalWheelControl(leftWheelRNG/1000.0, leftWheelSpeed);
              break;
            case prot_set_bothWheelSpeed:
              rightWheelSpeed = (rxBuffer[2])&0xff;
              leftWheelSpeed = (rxBuffer[2])&0xff;
              debugPrint("Both wheel speed now: ",&leftWheelSpeed);
              outputDigitalWheelControl(leftWheelRNG/1000.0, leftWheelSpeed);
              outputAnalogueWheelControl(rightWheelSpeed);
          }   
      } break;  
      case cmd_get: {
          debugPrint("Get command received, sending response ",&rxBuffer[1]); 
          txBuffer[0] = rxBuffer[0]; //identify response message
          debugPrint("Buffer: ", &rxBuffer[1]);
          switch(rxBuffer[1]) {
            case prot_get_rightWheelCurrent: 
                txBuffer[1] = prot_len_rightWheelCurrent; //length of response
                returnValue = analogRead(apin_rightWheelCurrent); 
                txBuffer[2] = (char)(returnValue&0xff);
                txBuffer[3] = (char)((returnValue>>8)&0xff);
              break;
            case prot_get_rightWheelNibble: 
                txBuffer[1] = prot_len_rightWheelNibble; //length of response
                returnValue = rightWheelSpeed; 
                txBuffer[2] = returnValue;
              break;
              
            case prot_get_everything: 
                txBuffer[1] = prot_len_getEverything; 
                
                returnValue =  getRightWheelCurrent();
                txBuffer[2] = (char)(returnValue&0xff);
                txBuffer[3] = (char)((returnValue>>8)&0xff);
                
                returnValue = (int) (analogRead(apin_rightWheelRange) / 4095.0 * 3300.0); 
                txBuffer[4] = (char)(returnValue&0xff);
                txBuffer[5] = (char)((returnValue>>8)&0xff);
                txBuffer[6] = rightWheelSpeed;

                returnValue = getLeftWheelCurrent();
                txBuffer[7] = (char)(returnValue&0xff);
                txBuffer[8] = (char)((returnValue>>8)&0xff);
                leftWheelRNG = leftWheelRange() * 10000;
                returnValue = leftWheelRNG; 
                txBuffer[9] = (char)(returnValue&0xff);
                txBuffer[10] = (char)((returnValue>>8)&0xff);
                txBuffer[11] = leftWheelSpeed;
                
                returnValue = (int) (analogRead(apin_batteryVoltage) / 4095.0 * 3300.0); 
                txBuffer[12] = (char)(returnValue&0xff);
                txBuffer[13] = (char)((returnValue>>8)&0xff);
                
                returnValue = analogRead(apin_accelerometerX); 
                txBuffer[14] = (char)(returnValue&0xff);
                txBuffer[15] = (char)((returnValue>>8)&0xff);
                
                returnValue = analogRead(apin_accelerometerY); 
                txBuffer[16] = (char)(returnValue&0xff);
                txBuffer[17] = (char)((returnValue>>8)&0xff);
              break;    
          } 
          //send the packet
          for (int i=0; i<txBuffer[1]+2; i++){
            SerialBT.write(txBuffer[i]);
          }      
      } break;   
    } 
  }
  cmdState = s_waiting;
}

void rxStateMachine(int receivedByte)
{
      switch (rxState) {
        case s_idle: {
              if (receivedByte==studentID) {
                debugPrint("I have received student ID", NULL); 
                rxState = s_pass;
                timeoutStart = millis(); 
              }
        } break;
        case s_pass: {
              if (receivedByte==passKey){
                debugPrint("I have received the passkey", NULL);
                rxState = s_len;  
              }
        } break;
        case s_len: {
              packetLen = receivedByte;
              debugPrint("I have received the length ", &packetLen);
              rxState = s_receiving;
              cntRx = 0;
        } break;
        case s_receiving: { 
            rxBuffer[cntRx++] = receivedByte;
            debugPrint("Receiving real data my bru!",NULL);
          }        
          if (cntRx >= packetLen) 
          {
            debugPrint("I have received the whole packet", NULL);
            cmdState = s_received;
            rxState = s_idle;
          } break;
      }
}


int riseTime = 0;
int prevRiseTime = 0;

int fallTime = 0;
int prevFallTime = 0;

void setPinTime() 
{
  if (digitalRead(apin_leftWheelRange))
  {
    prevRiseTime = riseTime;
    riseTime = micros();
  }
  else
  {
    prevFallTime = fallTime;
    fallTime = micros();
  }
}

float leftWheelRange()
{
  if (riseTime > fallTime)
  {
      return (fallTime - prevRiseTime) / 62500.0;
  }
  else
  {
    return (fallTime - riseTime) / 62500.0;
  }
}

void wheelControl(uint8_t leftWheelSpeed, float duty, uint8_t rightWheelSpeed)
{
  outputAnalogueWheelControl(leftWheelSpeed);
  outputDigitalWheelControl(duty, rightWheelSpeed);
}

int getRightWheelCurrent()
{
 return (int) (analogRead(apin_rightWheelCurrent) / 4095.0 * 3300.0 * (100.0/70.0)); 
}

int getLeftWheelCurrent()
{
 return (int) (analogRead(apin_leftWheelCurrent) / 4095.0 * 3300.0);  //(130.0/150.0));  
}

void wheelCurrentControl()
{
  if (leftWheelSpeed == rightWheelSpeed)
  {
    float RWC = getRightWheelCurrent();
    float LWC = getLeftWheelCurrent();
    Serial.println(RWC);
    Serial.println(LWC);
    if (RWC > LWC)
    {
      rightWheelSpeed = (int) rightWheelSpeed * (LWC/RWC); 
    }
    else if (RWC < LWC)
    {
      leftWheelSpeed = (int) leftWheelSpeed * (RWC/LWC); 
    }
  }
}

void outputAnalogueWheelControl(uint8_t value)
{
  int val1 = value / 128;
  int val2 = (value % 128) / 64;
  int val3 = (value % 196) / 32;
  int val4 = (value % 228) / 16;

  Serial.println("DAC1:");
  Serial.println(val1);
  
  Serial.println("DAC2:");
  Serial.println(val2);
  
  Serial.println("DAC4:");
  Serial.println(val3);
  
  Serial.println("DAC8:");
  Serial.println(val4);

  digitalWrite(apin_DAC_1, val1);
  digitalWrite(apin_DAC_2, val2);
  digitalWrite(apin_DAC_4, val3);
  digitalWrite(apin_DAC_8, val4);

}

void outputDigitalWheelControl(float duty, uint8_t rightWheelSpeed)
{
  ledcWrite(pwm_chan, (int)((rightWheelSpeed/100.0 * 256) * 1 * duty));
}

void debugPrint(char* debugStr1, char* valPtr){
  int val = 0;
  #ifdef DEBUG            
    if (valPtr == NULL) {
      Serial.println(debugStr1);
    }
    else {
      val = *valPtr;
      Serial.print(debugStr1);
      Serial.println(val);
    }
  #endif
}
