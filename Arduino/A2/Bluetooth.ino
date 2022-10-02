
#include "BluetoothSerial.h"

#define DEBUG //comment out to remove debug

#define studentID  123
#define passKey 111
#define c_timeout 2000 //ms


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

//set commands lengths
#define prot_set_rightWheelSpeed 50
#define prot_set_leftWheelSpeed 51

//message lengths
#define prot_len_rightWheelCurrent 1*2  //two bytes, one number 
#define prot_len_rightWheelNibble 1  //two bytes, one number 
#define prot_len_getEverything  18//RWCur=2,RWRange=2, RWnibble=1, LWCur=2, LWRange=2,LWNibble=1, battery=2, accel=6
/* ***************** */

/* ***************** */
//pinout definitions
/* ***************** */
#define apin_rightWheelCurrent 34 //change to match your pinout
#define apin_rightWheelRange 34 //change to match your pinout
#define apin_leftWheelCurrent 34 //change to match your pinout
#define apin_leftWheelRange 34 //change to match your pinout
#define apin_batteryVoltage 34 //change to match your pinout
#define apin_accelerometerX 34 //change to match your pinout
#define apin_accelerometerY 34 //change to match your pinout
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
char rightWheelSpeed = 0;
char leftWheelSpeed =0;


//function prototypes
void rxStateMachine(int receivedByte);
void cmdStateMachine();
void debugPrint(char* debugStr1, char* valPtr);

//setup
void setup() {
  Serial.begin(115200);
  SerialBT.begin("ThinusSeKar"); //Bluetooth device name
  Serial.println("The device started, now you can pair it with bluetooth!");
}

void loop() {
  if (SerialBT.available()) { 
      debugPrint("I have received a byte!",NULL); 
      rxStateMachine(SerialBT.read());
  }
  cmdStateMachine();
  
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
            case prot_set_rightWheelSpeed: rightWheelSpeed = (rxBuffer[2])&0xff; debugPrint("Wheel speed now: ",&rightWheelSpeed); break;
            case prot_set_leftWheelSpeed: leftWheelSpeed++; break;
          }   
      } break;  
      case cmd_get: {
          debugPrint("Get command received, sending response ",&rxBuffer[1]); 
          txBuffer[0] = rxBuffer[0]; //identify response message
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
              
            case prot_len_getEverything: // you can populate this with your own. 
                txBuffer[1] = prot_len_getEverything; //length of response
                returnValue = analogRead(apin_rightWheelCurrent); 
                txBuffer[2] = (char)(returnValue&0xff);
                txBuffer[3] = (char)((returnValue>>8)&0xff);
                returnValue = analogRead(apin_rightWheelRange); 
                txBuffer[4] = (char)(returnValue&0xff);
                txBuffer[5] = (char)((returnValue>>8)&0xff);
                txBuffer[6] = rightWheelSpeed;
                //here you build the rest of your packet
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
              debugPrint("I have received the length", &packetLen);
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
