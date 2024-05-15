#include <Servo.h>
class _Servo {
private:
  Servo sv;

  short pin;
  int pos = 0;
public:
  _Servo (short p) {
    pin = p;
    sv.attach(pin);
  }

  void opened (int p) {
    for (pos = 0; pos <= p; pos++) {  
      // in steps of 1 degree
      sv.write(pos);              
      delay(15);
    }
  }

  void closed () {
    for (pos = pos; pos >= 0; pos--) {  
      // in steps of 1 degree
      sv.write(pos);         
      delay(15);
    }
  }
};

/**********************************************************************************************************/
// Flame sensor
class FlameSensor {
private:
  uint8_t pin;
public:
  FlameSensor (uint8_t p) {
    pin = p;
    pinMode(pin, INPUT);
  }

  bool isFire () {
    int sensorReading = analogRead(pin);
    // MIN = 0, MAX = 1024
      int range = map(sensorReading, 0, 1024, 0, 3);

    if (range) {
      return true;
    }

    return false;
  }
}; 

/**********************************************************************************************************/
// Gas Sensor
class GasSensor {
private:
  short analogPin, digitalPin;
  int sensorValue;
  int digitalValue;
public:
  GasSensor (uint8_t ap, short dp) {
    analogPin = ap;
    digitalPin = dp;
    pinMode(analogPin, INPUT);
    pinMode(digitalPin, INPUT);
  }

  bool isPolluted () {
    sensorValue = analogRead(analogPin); // read analog input pin 0
    digitalValue = digitalRead(digitalPin);

    if (sensorValue > 400) {
      return true;
    }

    return false;
  } 
};

/**********************************************************************************************************/
// Temp and Humidity
#include <dht.h>

class TempAndHumidity {
private:
  short pin;
  dht DHT;
public:
  TempAndHumidity (short p) {
    pin = p;
    pinMode(pin, INPUT);
  }

  float humidity () {
    int readData = DHT.read11(pin);
    return DHT.humidity;
  }

  float temperature () {
    int readData = DHT.read11(pin);
    return DHT.temperature;
  }
};

/**********************************************************************************************************/
// Light Sensor
class LightSensor {
private:
  short pin;
public:
  LightSensor (short p) {
    pin = p;
    pinMode(pin, INPUT);
  }

  bool isDark () {
    if(digitalRead(pin) == LOW)
      return true;

    return false;
  }
};

/**********************************************************************************************************/
// Water sensor
class WaterSensor {
private:
  short power_pin;
  uint8_t signal_pin;

  int threshold, value = 0, level = 0;
public:
  WaterSensor (short p, uint8_t s, int t) {
    power_pin = p;
    signal_pin = s;
    threshold = t;

    pinMode(power_pin, OUTPUT);
    digitalWrite(power_pin, LOW); // turn the sensor OFF
  }

  bool isWaterLeakage () {
    digitalWrite(power_pin, HIGH);  // turn the sensor ON
    delay(10);                      // wait 10 milliseconds
    value = analogRead(signal_pin); // read the analog value from sensor
    digitalWrite(power_pin, LOW);   // turn the sensor OFF

    if (value > threshold)
      return true;

    return false;
  }

  int waterLevel () {
    digitalWrite(power_pin, HIGH);  // turn the sensor ON
    delay(10);                      // wait 10 milliseconds
    value = analogRead(signal_pin); // read the analog value from sensor
    digitalWrite(power_pin, LOW);   // turn the sensor OFF

    // SENSOR_MIN = 0, SENSOR_MAX = 521
    level = map(value, 0, 521, 0, 4); // 4 levels

    return level;
  }
};

/**********************************************************************************************************/
// RFID Module
#include <Wire.h>
#include <PN532.h>
#include <PN532_I2C.h>
#include <NfcAdapter.h>

PN532_I2C pn532_i2c(Wire);
NfcAdapter nfc = NfcAdapter(pn532_i2c);

String tagId = "None";
byte nuidPICC[4];

bool isNFC() {
  delay(300);
  if (!nfc.tagPresent()) {
    return false;  
  }
  
  short total = 0;
  for (short i = 0; i < 5; i++) {
    if (nfc.tagPresent()) {
     NfcTag tag = nfc.read();
     tag.print();
     tagId = tag.getUidString();
    }

    delay (100);
    
    if (tagId == "D3 61 6A 0B") {
     total++; 
    }
  }

  return (total > 4 ? 1 : 0);
}

/**********************************************************************************************************/
// Keypad
#include <Keypad.h>

const byte ROWS = 4; 
const byte COLS = 4; 

char hexaKeys[ROWS][COLS] = {
  {'1', '2', '3', 'A'},
  {'4', '5', '6', 'B'},
  {'7', '8', '9', 'C'},
  {'*', '0', '#', 'D'}
};

byte rowPins[ROWS] = {52, 50, 48, 46}; 
byte colPins[COLS] = {44, 42, 40, 38}; 

Keypad controlPad = Keypad(makeKeymap(hexaKeys), rowPins, colPins, ROWS, COLS);

/**********************************************************************************************************/
// Buzzer
class buzzer {
  private:
    short pin;
  public:
    buzzer (short p) {
      pin = p;
      pinMode(pin, OUTPUT);
    }

    void normal () {
      tone(pin, 1023);
      delay(100);
      noTone(pin);
    }

    void notNormal () {
      tone(pin, 400);
      delay(100);
      noTone(pin);
    }

    void siren () {
      digitalWrite (pin, HIGH);
      delay (100);
        
      digitalWrite (pin, LOW);
      delay (100); 
    }
};

/**********************************************************************************************************/
// Ultrasonic
class Ultrasonic {
  private:
    short trigPin, echoPin;
  public:
    Ultrasonic (short t, short e) {
      trigPin = t, echoPin = e;
      pinMode(trigPin, OUTPUT); // Sets the trigPin as an Output
      pinMode(echoPin, INPUT); // Sets the echoPin as an Input
    }
  
    int distance () {
      int dis, dur;
      // Clears the trigPin
      digitalWrite(trigPin, LOW);
      delayMicroseconds(2);
  
      // Sets the trigPin on HIGH state for 10 micro seconds
      digitalWrite(trigPin, HIGH);
      delayMicroseconds(10);
      digitalWrite(trigPin, LOW);
  
      // Reads the echoPin, returns the sound wave travel time in microseconds
      dur = pulseIn(echoPin, HIGH);
  
      // Calculating the distance
      dis= dur*0.034/2;
  
      return dis;
    }
  
    bool isEnemy (int d) {
      if (this -> distance() <= d)
        return true;
  
      return false;
    }
};

/**********************************************************************************************************/
buzzer buzzer1(30);
TempAndHumidity th1(32);
FlameSensor fs1(A12);
Ultrasonic us1(9, 8), us2(11, 10);
GasSensor gs1(A14, 24);
#define FAN_PIN 26;
_Servo garage(6);
#define GARAGE_LED 28;

/**********************************************************************************************************/
// Password
short numberOfTries = 3;
String password = "1234";
bool isPass () {
  while (numberOfTries) {
    String input = "";
 
    while (true) {
      char keyPress = controlPad.getKey();

      if (keyPress == '*') {
        buzzer1.normal();
        return false;
      }
      
      if (keyPress != '#') {
        if (keyPress) {
          buzzer1.normal();
        } 
 
        input += String(keyPress);

        continue;
      }

      break;
    }
    
    if (input == password) {
      return true;
    }
  
    if (input != password) {
      numberOfTries--;
      buzzer1.notNormal();
    }
  }

  while (true) {
    buzzer1.siren();  
  }

  return false;
}

/**********************************************************************************************************/
void setup() {
  Serial.begin(115200);
  Serial.println("System initialized");
}

void loop() {
  Serial.println(th1.humidity());
}
