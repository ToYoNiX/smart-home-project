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

byte rowPins[ROWS] = {12, 11, 10, 9}; 
byte colPins[COLS] = {8, 7, 6, 5}; 

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
buzzer buzzer_1(4);

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
        buzzer_1.normal();
        return false;
      }
      
      if (keyPress != '#') {
        if (keyPress) {
          buzzer_1.normal();
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
      buzzer_1.notNormal();
    }
  }

  while (true) {
    buzzer_1.siren();  
  }

  return false;
}

/**********************************************************************************************************/
void setup() {
  Serial.begin(115200);
  Serial.println("System initialized");
  nfc.begin();

  while (true) {
    if (isPass() || isNFC()) {
      buzzer_1.siren();
      break;
    }
  }
}

void loop() {
}
