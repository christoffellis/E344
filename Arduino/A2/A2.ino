const int triggerPin = 23;
const int freq = 16;
const int triggerChannel = 0;
const int resolution = 16;

const int pwmPin = 14;
const int pwmFreq = 5000;
const int pwmChannel = 2;
const int pwmResolution = 8;

const int DIPin1 = 2;
const int DIPin2 = 0;
const int DIPin4 = 4;
const int DIPin8 = 16;

const int leftSensorPin = 19;
#define trigger 2  // pin 2 will flash for debug only


void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200); // uart setup
  while(!Serial);
  Serial.println("START UP");
  pinMode(triggerPin, OUTPUT);

  pinMode(DIPin1, INPUT);
  pinMode(DIPin2, INPUT);
  pinMode(DIPin4, INPUT);
  pinMode(DIPin8, INPUT);
  
  //ledcSetup(triggerChannel, freq, resolution); //set pwn freq and resulution
  //ledcAttachPin(triggerPin, triggerChannel); //select pwm outpit pin and channel
  //ledcWrite(triggerChannel, 11); // set dutycycle to 11/65536 (10u out of 62.5ms)

  ledcSetup(pwmChannel, pwmFreq, pwmResolution); //set pwn freq and resulution
  ledcAttachPin(pwmPin, pwmChannel); //select pwm outpit pin and channel
}

int timeSinceSound = 0;
void loop() {
  
  
  int now = micros();
  // Serial.println(now - timeSinceSound);
  if (now - timeSinceSound > 62000)
  {
    timeSinceSound = now;
    digitalWrite(triggerPin, HIGH);
    delayMicroseconds(10);
    digitalWrite(triggerPin, LOW);
    int ontime = pulseIn(leftSensorPin, HIGH, 61000);
    Serial.println(ontime);
    float duty = ontime / 620000.0 * 100.0;
    if (duty < 0.01 || duty > 1) {
      duty = 1;
    }
    Serial.println("Duty:");
    Serial.println(duty);
    int dip = readDIP();
    Serial.println("DIP:");
    Serial.println(dip);
    setPWMDuty(duty, dip);
  }
}

int startTime = 0;

void setPWMDuty(float duty, int dip) {
  ledcWrite(pwmChannel, (int)((256-dip) * 1 * duty)); // set dutycycle to 11/65536 (10u out of 62.5ms)
}

uint8_t readDIP() {
  int val = 0;
  if (digitalRead(DIPin1))
  {
    val += 16;
  }
  if (digitalRead(DIPin2)) 
  {
    val += 32;
  }
  if (digitalRead(DIPin4))
  {
    val += 64;
  }
  if (digitalRead(DIPin8)) 
  {
    val += 128;
  }
  return val;
}
