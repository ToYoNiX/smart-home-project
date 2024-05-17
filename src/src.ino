// Display
#include <Wire.h> 
#include <LiquidCrystal_I2C.h>

LiquidCrystal_I2C lcd(0x27,20,4);  // set the LCD address to 0x27 for a 16 chars and 2 line display

/**********************************************************************************************************/
#include <Servo.h>
void servo_move(Servo sv, int p) {
  for (int pos = 0; pos <= p; pos++) {  
    // in steps of 1 degree
    sv.write(pos);              
    delay(15);
  }
}

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
      delay(50);
      noTone(pin);
    }

    void notNormal () {
      tone(pin, 200);
      delay(50);
      noTone(pin);
    }

    void siren () {
      digitalWrite (pin, HIGH);
      delay (50);

      digitalWrite (pin, LOW);
      delay (50); 
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
buzzer buzzer1(41);
TempAndHumidity th1(32);
FlameSensor fs1(A15);
Ultrasonic us1(26, 24), us2(53, 51);
GasSensor gs1(A14, 22);
WaterSensor wt(7, A13 ,400);
LightSensor lt(30);
#define FAN_PIN 34;
#define GARAGE_LED 47;
Servo gate, window_r, window_l, door_r, door_l, garage;

/**********************************************************************************************************/
// Password
short numberOfTries = 3;
String password = "1234";
bool isPass () {
  while (numberOfTries) {
    String input = "";

    int c = 0;
    while (true) {
      char keyPress = controlPad.getKey();

      if (keyPress != '#') {
        if (keyPress) {
          buzzer1.normal();
          lcd.setCursor(c, 1);
          lcd.print("*");
          c++;
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
  nfc.begin();
  lcd.init();
  lcd.backlight();

  Serial.println("System initialized");

  //window_r.attach(3);
  //window_l.attach(2);
  door_r.attach(5);
  door_l.attach(6);
  gate.attach(4); 
  garage.attach(9);
  servo_move(gate,0);

  servo_move(door_l,0);
  servo_move(door_r,90);
  servo_move(garage,360);

  lcd.setCursor(0, 0);
  lcd.print("ENTER PASSWORD: ");
  if(isPass()) {
    buzzer1.siren();
    servo_move(gate,90);
    servo_move(garage,0);
  }
  lcd.init();
  lcd.print("Gate Opened!");

  delay(5000);
  servo_move(gate,0);

  lcd.init();
  lcd.print("Gate Closed!");

  bool isNFC_PASS = false;
  while (true) {
    if (isNFC()) {
      isNFC_PASS = true;
      break;
    }
  }
  if (isNFC_PASS) {
    buzzer1.siren();
    servo_move(door_l,90);
    servo_move(door_r,0);
  }

  pinMode(47, OUTPUT); 
  pinMode(49, OUTPUT); 
  pinMode(45, OUTPUT); 
  pinMode(43, OUTPUT);   
  pinMode(34, OUTPUT);  
  pinMode(35, OUTPUT); 
  pinMode(33, OUTPUT);

  lcd.init();
  lcd.print("Welcome home <3");
  delay(5000);
  servo_move(door_l,0);
  servo_move(door_r,90);;
}

void loop() {

  if (fs1.isFire()) {
    lcd.init();
    while (true) {
      digitalWrite(35, HIGH);
      digitalWrite(33, HIGH);
      buzzer1.siren();
      lcd.setCursor(0, 0);
      lcd.print("!! FIRE !!");
      delay(50);
      digitalWrite(35, LOW);
      digitalWrite(33, LOW);
      delay(50);

      if (!fs1.isFire()) {
        lcd.setCursor(0, 0);
        lcd.print("FIRE IS GONE!");
        delay(500);
        lcd.init();
        break;
      }
    }
  }
   
  Serial.println("Humid:" + String(th1.humidity()));
  Serial.println("Temp:" + String(th1.temperature()));
  Serial.println("Flame:" + String(fs1.isFire()));
  Serial.println("Gas:" + String(gs1.isPolluted()));
  Serial.println("ultra1:" + String(us1.distance()));
  Serial.println("ultra2:" + String(us2.distance()));
  Serial.println("water:" + String(wt.isWaterLeakage()));
  Serial.println("light:" + String(lt.isDark ())); 
  lcd.init();
  lcd.setCursor(0, 0);
  lcd.print("Humidity:" + String(int(th1.humidity())) + "%");
  lcd.setCursor(0, 1);
  lcd.print("Temp:" + String(int(th1.temperature())) + "C, Rain:" + String(wt.isWaterLeakage()));




  if(gs1.isPolluted()){
    digitalWrite(34, HIGH);
    Serial.println("Fan is on.");
  }      
  else {
    digitalWrite(34, LOW);
  }

  if(! lt.isDark ()) {
    digitalWrite(35, HIGH);
    digitalWrite(33, HIGH);
    Serial.println("Outside light is on.");}
  else {
    digitalWrite(35, LOW);
    digitalWrite(33, LOW);
  }

  if(us2.distance()<=6) {
    digitalWrite(45, HIGH);
    Serial.println("Salon light is on.");
  } 
  else {
    digitalWrite(45, LOW);
  }

  if(us1.distance() <= 8) {
    digitalWrite(43, HIGH);
    Serial.println("Room light is on.");
  }
  else {
    digitalWrite(43, LOW);
  }

  if(wt.isWaterLeakage()) {
    digitalWrite(47, HIGH);
    Serial.println("Garage light is on.");
    delay(5000);
  } 
}
