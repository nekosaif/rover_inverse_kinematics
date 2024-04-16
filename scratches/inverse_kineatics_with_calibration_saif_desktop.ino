#include <EEPROM.h>


#define ACT1_UP_PIN 3
#define ACT1_DOWN_PIN 2
#define ACT1_POT_PIN A3
#define ACT2_UP_PIN 5
#define ACT2_DOWN_PIN 4
#define ACT2_POT_PIN A4
#define BASE_RIGHT_PIN 6
#define BASE_LEFT_PIN 7
#define BASE_POT_PIN A5
#define CALIBRATION_PIN 8
#define BASE_CALIBRATION_PIN 9

#define ACT1_POT_MIN 0
#define ACT1_POT_MAX 1
#define ACT2_POT_MIN 2
#define ACT2_POT_MAX 3
#define BASE_POT_MIN 4
#define BASE_POT_MAX 5


void pinSetup();
bool calibrationBlink();
void getPOTValuesFromEEPROM();
void writePOTValuesToEEPROM();
void calibratePOT();
void moveAct1Up();
void moveAct1Down();
void moveAct2Up();
void moveAct2Down();
void moveBaseRight();
void moveBaseLeft();
void stopAct1();
void stopAct2();
void stopBase();
void move(int armNumber, int potValue);
void moveAct1MIN();
void moveAct2MIN();
void moveBaseMIN();
void moveAct1MAX();
void moveAct2MAX();
void moveBaseMAX();


const int EEPROM_ADDR = 0;
const int CALIBRATION_BLINK_TIME = 3;
const int STALL_TIME = 1;
const int POT_OFFSET = 5;


const int ACT1_MIN_ANGLE = 0;
const int ACT1_MAX_ANGLE = 98;
const int ACT2_MIN_ANGLE = 35;
const int ACT2_MAX_ANGLE = 134;
const int BASE_MIN_ANGLE = 0;
const int BASE_MAX_ANGLE = 90;



int POT_VALUES[6] = {-1, -1, -1, -1, -1, -1};


void setup()
{
  Serial.begin(9600);
  
  pinSetup();

  if (!calibrationBlink())
  {
    getPOTValuesFromEEPROM();
  }
}


void loop()
{
  if (Serial.available() > 0)
  {
    String commandStr = Serial.readString();
    int armNumber = commandStr.substring(0, 1).toInt();
    int potValue = commandStr.substring(1).toInt();
    move(armNumber, potValue);
  }
}


void pinSetup()
{
  pinMode(LED_BUILTIN, OUTPUT);
  
  pinMode(ACT1_UP_PIN, OUTPUT);
  pinMode(ACT1_DOWN_PIN, OUTPUT);
  pinMode(ACT1_POT_PIN, INPUT);
  pinMode(ACT2_UP_PIN, OUTPUT);
  pinMode(ACT2_DOWN_PIN, OUTPUT);
  pinMode(ACT2_POT_PIN, INPUT);
  pinMode(BASE_RIGHT_PIN, OUTPUT);
  pinMode(BASE_LEFT_PIN, OUTPUT);
  pinMode(BASE_POT_PIN, INPUT);

  pinMode(CALIBRATION_PIN, INPUT_PULLUP);
  pinMode(BASE_CALIBRATION_PIN, INPUT_PULLUP);

  digitalWrite(LED_BUILTIN, LOW);
  digitalWrite(ACT1_UP_PIN, HIGH);
  digitalWrite(ACT1_DOWN_PIN, HIGH);
  digitalWrite(ACT2_UP_PIN, HIGH);
  digitalWrite(ACT2_DOWN_PIN, HIGH);
  digitalWrite(BASE_RIGHT_PIN, HIGH);
  digitalWrite(BASE_LEFT_PIN, HIGH);
}


bool calibrationBlink()
{
  for (int i = 0; i < CALIBRATION_BLINK_TIME * 2; ++i)
  {
    digitalWrite(LED_BUILTIN, HIGH);
    delay(250);
    if (!digitalRead(CALIBRATION_PIN))
    {
      for (int k = 0; k < 20; ++k)
      {
        digitalWrite(LED_BUILTIN, HIGH);
        delay(50);
        digitalWrite(LED_BUILTIN, LOW);
        delay(50);
      }
      calibratePOT();
      return true;
    }
    digitalWrite(LED_BUILTIN, LOW);
    delay(250);
  }
  return false;
}


void getPOTValuesFromEEPROM()
{
  for (int i = 0, j = EEPROM_ADDR; i < 6; ++i, ++j)
  {
    POT_VALUES[i] = EEPROM.read(j);
    Serial.println(POT_VALUES[i]);
  }
}


void writePOTValuesToEEPROM()
{
  for (int i = 0, j = EEPROM_ADDR; i < 6; ++i, ++j)
  {
    EEPROM.update(j, POT_VALUES[i]);
  }
}


void calibratePOT()
{
  Serial.println("I am moveAct2MAX");
  moveAct2MAX();
  delay(1000);
  Serial.println("I am moveAct1MAX");
  moveAct1MAX();
  delay(1000);
  Serial.println("I am moveAct1MIN");
  moveAct1MIN();
  delay(1000);
  Serial.println("I am moveAct2MIN");
  moveAct2MIN();
  delay(1000);
  for (int i = 0; i < 5; ++i)
  {
    digitalWrite(LED_BUILTIN, HIGH);
    delay(500);
    digitalWrite(LED_BUILTIN, LOW);
    delay(500);
  }
  while (digitalRead(CALIBRATION_PIN)) {} //safty feature so that saif does not break Base -- NSA
  for (int i = 0; i < 4; ++i)
  {
    digitalWrite(LED_BUILTIN, HIGH);
    delay(125);
    digitalWrite(LED_BUILTIN, LOW);
    delay(125);
  }
  Serial.println("I am moveBaseMIN");
  moveBaseMIN();
  delay(1000);
  Serial.println("I am moveBaseMAX");
  moveBaseMAX();
  delay(1000);
  String potValueStr = "";
  for (int i = 0; i < 6; ++i)
  {
    potValueStr += ((String)POT_VALUES[i] + " ");
  }
  Serial.println(potValueStr);
  writePOTValuesToEEPROM();
  Serial.println("I am complete calibration");
} 

void moveAct1Up()
{
  digitalWrite(ACT1_UP_PIN, LOW);
}

void moveAct1Down()
{
  digitalWrite(ACT1_DOWN_PIN, LOW);
}

void moveAct2Up()
{
  digitalWrite(ACT2_UP_PIN, LOW);
}

void moveAct2Down()
{
  digitalWrite(ACT2_DOWN_PIN, LOW);
}

void moveBaseRight()
{
  digitalWrite(BASE_RIGHT_PIN, LOW);
}

void moveBaseLeft()
{
  digitalWrite(BASE_LEFT_PIN, LOW);
}

void stopAct1()
{
  digitalWrite(ACT1_UP_PIN, HIGH);
  digitalWrite(ACT1_DOWN_PIN, HIGH);
}

void stopAct2()
{
  digitalWrite(ACT2_UP_PIN, HIGH);
  digitalWrite(ACT2_DOWN_PIN, HIGH);
}

void stopBase()
{
  digitalWrite(BASE_RIGHT_PIN, HIGH);
  digitalWrite(BASE_LEFT_PIN, HIGH);
}


void move(int armNumber, int potValue)
{
  int armUpPin;
  int armDownPin;
  int armPotPin;
  int currentPotValue;

  if (armNumber == 1)
  {
    armUpPin = ACT1_UP_PIN;
    armDownPin = ACT1_DOWN_PIN;
    armPotPin = ACT1_POT_PIN;
    potValue = map(potValue, ACT1_MIN_ANGLE, ACT1_MAX_ANGLE, POT_VALUES[1], POT_VALUES[0]);
    Serial.print("POt1; "); Serial.println(potValue);
  }
  else if (armNumber == 2)
  {
    armUpPin = ACT2_DOWN_PIN;
    armDownPin = ACT2_UP_PIN;
    armPotPin = ACT2_POT_PIN;
    potValue = map(potValue, ACT2_MIN_ANGLE, ACT2_MAX_ANGLE, POT_VALUES[2], POT_VALUES[3]);
    Serial.print("POt2; "); Serial.println(potValue);
  }
  else if (armNumber == 3)
  {
    armUpPin = BASE_RIGHT_PIN;
    armDownPin = BASE_LEFT_PIN;
    armPotPin = BASE_POT_PIN;
    potValue = map(potValue, BASE_MIN_ANGLE, BASE_MAX_ANGLE, POT_VALUES[4], POT_VALUES[5]);
    Serial.print("Base; "); Serial.println(potValue);
  }

  while (true)
  {
    currentPotValue = map(analogRead(armPotPin), 0, 1023, 0, 255);
    if (currentPotValue < potValue - POT_OFFSET)
    {
      digitalWrite(armDownPin, LOW);
      delay(100);
      digitalWrite(armDownPin, HIGH);
    }
    else if (currentPotValue > potValue + POT_OFFSET)
    {
      digitalWrite(armUpPin, LOW);
      delay(100);
      digitalWrite(armUpPin, HIGH);
    }
    else
    {
      digitalWrite(armUpPin, HIGH);
      digitalWrite(armDownPin, HIGH);
      break;
    }
  }
}


void moveAct1MIN()
{
  int act1PotValue = map(analogRead(ACT1_POT_PIN), 0, 1023, 0, 255);
  unsigned long act1PotValueUpdateTime = millis();
  moveAct1Up();
  while (true)
  {
    if (millis() > act1PotValueUpdateTime + STALL_TIME * 1000 || !digitalRead(CALIBRATION_PIN))
    {
      if (abs(act1PotValue - map(analogRead(ACT1_POT_PIN), 0, 1023, 0, 255)) < 3)
      {
        stopAct1();
        POT_VALUES[ACT1_POT_MIN] = act1PotValue;
        return;
      }
    }
    if (act1PotValue != map(analogRead(ACT1_POT_PIN), 0, 1023, 0, 255))
    {
      act1PotValueUpdateTime = millis();
      act1PotValue = map(analogRead(ACT1_POT_PIN), 0, 1023, 0, 255);
    }
  }
}

void moveAct2MIN()
{
  int act2PotValue = map(analogRead(ACT2_POT_PIN), 0, 1023, 0, 255);
  unsigned long act2PotValueUpdateTime = millis();
  moveAct2Down();
  while (true)
  {
    if (millis() > act2PotValueUpdateTime + STALL_TIME * 1000 || !digitalRead(CALIBRATION_PIN))
    {
      if (abs(act2PotValue - map(analogRead(ACT2_POT_PIN), 0, 1023, 0, 255) < 3))
      {
        stopAct2();
        POT_VALUES[ACT2_POT_MIN] = act2PotValue;
        return;
      }
    }
    if (act2PotValue != map(analogRead(ACT2_POT_PIN), 0, 1023, 0, 255))
    {
      act2PotValueUpdateTime = millis();
      act2PotValue = map(analogRead(ACT2_POT_PIN), 0, 1023, 0, 255);
    }
  }
}

void moveBaseMIN()
{
  int basePotValue = map(analogRead(BASE_POT_PIN), 0, 1023, 0, 255);
  moveBaseRight();
  int moveBaseDirection = 0;
  unsigned long moveBaseDiectionUpdateTime = millis();
  while (true)
  {
    if (!digitalRead(CALIBRATION_PIN))
    {
      stopBase();
      POT_VALUES[BASE_POT_MIN] = basePotValue;
      for (int i = 0; i < 3; ++i)
      {
        digitalWrite(LED_BUILTIN, HIGH);
        delay(500);
        digitalWrite(LED_BUILTIN, LOW);
        delay(500);
      }
      return;
    }
    basePotValue == map(analogRead(BASE_POT_PIN), 0, 1023, 0, 255);
    if (!digitalRead(BASE_CALIBRATION_PIN) && moveBaseDiectionUpdateTime + 1000 < millis())
    {
      if (moveBaseDirection == 0)
      {
        stopBase();
        moveBaseDirection = 1;
        moveBaseDiectionUpdateTime = millis();
      }
      else if (moveBaseDirection == 1)
      {
        stopBase();
        moveBaseLeft();
        moveBaseDirection = 2;
        moveBaseDiectionUpdateTime = millis();
      }
      else if (moveBaseDirection == 2)
      {
        stopBase();
        moveBaseRight();
        moveBaseDirection = 0;
        moveBaseDiectionUpdateTime = millis();
      }
    }
  }
}


void moveAct1MAX()
{
  int act1PotValue = map(analogRead(ACT1_POT_PIN), 0, 1023, 0, 255);
  unsigned long act1PotValueUpdateTime = millis();
  moveAct1Down();
  while (true)
  {
    if (millis() > act1PotValueUpdateTime + STALL_TIME * 1000 || !digitalRead(CALIBRATION_PIN))
    {
      if (abs(act1PotValue - map(analogRead(ACT1_POT_PIN), 0, 1023, 0, 255)) < 3)
      {
        stopAct1();
        POT_VALUES[ACT1_POT_MAX] = act1PotValue;
        return;
      }
    }
    act1PotValue = map(analogRead(ACT1_POT_PIN), 0, 1023, 0, 255);
    if (act1PotValue != map(analogRead(ACT1_POT_PIN), 0, 1023, 0, 255))
    {
      act1PotValueUpdateTime = millis();
      act1PotValue = map(analogRead(ACT1_POT_PIN), 0, 1023, 0, 255);
    }
  }
}

void moveAct2MAX()
{
  int act2PotValue = map(analogRead(ACT2_POT_PIN), 0, 1023, 0, 255);
  unsigned long act2PotValueUpdateTime = millis();
  moveAct2Up();
  while (true)
  {
    if (millis() > act2PotValueUpdateTime + STALL_TIME * 1000 || !digitalRead(CALIBRATION_PIN))
    {
      if (abs(act2PotValue - map(analogRead(ACT2_POT_PIN), 0, 1023, 0, 255)) < 3)
      {
        stopAct2();
        POT_VALUES[ACT2_POT_MAX] = act2PotValue;
        return;
      }
    }
    if (act2PotValue != map(analogRead(ACT2_POT_PIN), 0, 1023, 0, 255))
    {
      act2PotValueUpdateTime = millis();
      act2PotValue = map(analogRead(ACT2_POT_PIN), 0, 1023, 0, 255);
    }
  }
}

void moveBaseMAX()
{
  int basePotValue = map(analogRead(BASE_POT_PIN), 0, 1023, 0, 255);
  moveBaseLeft();
  int moveBaseDirection = 2;
  unsigned long moveBaseDiectionUpdateTime = millis();
  while (true)
  {
    if (!digitalRead(CALIBRATION_PIN))
    {
      stopBase();
      POT_VALUES[BASE_POT_MAX] = basePotValue;
      for (int i = 0; i < 3; ++i)
      {
        digitalWrite(LED_BUILTIN, HIGH);
        delay(500);
        digitalWrite(LED_BUILTIN, LOW);
        delay(500);
      }
      return;
    }
    basePotValue == map(analogRead(BASE_POT_PIN), 0, 1023, 0, 255);
    if (!digitalRead(BASE_CALIBRATION_PIN) && moveBaseDiectionUpdateTime + 1000 < millis())
    {
      if (moveBaseDirection == 0)
      {
        stopBase();
        moveBaseDirection = 1;
        moveBaseDiectionUpdateTime = millis();
      }
      else if (moveBaseDirection == 1)
      {
        stopBase();
        moveBaseRight();
        moveBaseDirection = 2;
        moveBaseDiectionUpdateTime = millis();
      }
      else if (moveBaseDirection == 2)
      {
        stopBase();
        moveBaseLeft();
        moveBaseDirection = 0;
        moveBaseDiectionUpdateTime = millis();
      }
    }
  }
}
