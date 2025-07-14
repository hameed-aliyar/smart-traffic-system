#include <SoftwareSerial.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>

const int redLED = 2;
const int yellowLED = 3;
const int greenLED = 4;
const int buttonPin = 5;
const int pedRedLED = 6;
const int pedGreenLED = 7;
const int trigJaywalk = 11;
const int echoJaywalk = 12;
const int alertLED = 13;
const int moisturePin = A0;
const int buzzerPin = A1;

SoftwareSerial bluetooth(9, 8);
LiquidCrystal_I2C lcd(0x27, 16, 2);

enum TrafficState {
  TRAFFIC_GREEN,
  TRAFFIC_YELLOW,
  TRAFFIC_RED
};

int trafficLevel = 1;
bool pedestrianRequested = false;
bool buttonPressed = false;
unsigned long stateStartTime = 0;
unsigned long lastDistanceCheck = 0;
long previousDistance = 100;
TrafficState currentState = TRAFFIC_GREEN;

unsigned long lastRainCheck = 0;
unsigned long lastRainBlink = 0;
bool rainAlertState = false;
bool isRaining = false;

const int YELLOW_TIME = 3;
const int PED_WALK_TIME = 15;
const int MIN_GREEN_TIME = 5;
const int MIN_RED_TIME = 3;

void setup() {
  pinMode(redLED, OUTPUT);
  pinMode(yellowLED, OUTPUT);
  pinMode(greenLED, OUTPUT);
  pinMode(pedRedLED, OUTPUT);
  pinMode(pedGreenLED, OUTPUT);
  pinMode(buttonPin, INPUT_PULLUP);
  pinMode(trigJaywalk, OUTPUT);
  pinMode(echoJaywalk, INPUT);
  pinMode(alertLED, OUTPUT);
  pinMode(buzzerPin, OUTPUT);

  Serial.begin(9600);
  bluetooth.begin(9600);
  lcd.init();
  lcd.backlight();

  displayMessage("Smart Traffic", "System Ready");
  delay(2000);

  setTrafficState(TRAFFIC_GREEN);
  stateStartTime = millis();

  Serial.println("Traffic Light System Started");
  bluetooth.println("Traffic Light System Started");
}

void loop() {
  // Handle Bluetooth Traffic Level Input
  if (Serial.available()) {
    char input = Serial.read();
    if (input >= '1' && input <= '3') {
      trafficLevel = input - '0';
      Serial.print("Traffic Level set to: ");
      Serial.println(trafficLevel);
      bluetooth.print(" Traffic Level: ");
      bluetooth.println(trafficLevel);
    }
  }

  // Handle Pedestrian Button
  static unsigned long lastButtonTime = 0;
  if (digitalRead(buttonPin) == LOW && millis() - lastButtonTime > 500) {
    if (!buttonPressed && !pedestrianRequested) {
      if (currentState == TRAFFIC_GREEN && (millis() - stateStartTime) >= (MIN_GREEN_TIME * 1000)) {
        pedestrianRequested = true;
        buttonPressed = true;
        bluetooth.println(" Pedestrian crossing requested");
        Serial.println("Pedestrian button pressed");
      } else if (currentState != TRAFFIC_GREEN) {
        bluetooth.println("Please wait - crossing will be available soon");
      } else {
        bluetooth.println("Please wait - minimum green time not reached");
      }
    }
    lastButtonTime = millis();
  }

  if (digitalRead(buttonPin) == HIGH) {
    buttonPressed = false;
  }

  // Check jaywalking conditions
  if (millis() - lastDistanceCheck > 1000) {
    checkSafetyConditions();
    lastDistanceCheck = millis();
  }

  // Check rain condition
  if (millis() - lastRainCheck > 2000) {
    checkRainCondition();
    lastRainCheck = millis();
  }

  // Blink alert LED in rain
  if (isRaining && millis() - lastRainBlink > 500) {
    rainAlertState = !rainAlertState;
    if (!digitalRead(alertLED)) {
      digitalWrite(alertLED, rainAlertState ? HIGH : LOW);
    }
    lastRainBlink = millis();
  }

  // Handle traffic light state machine
  manageTrafficLights();
}

void manageTrafficLights() {
  unsigned long currentTime = millis();
  unsigned long elapsedTime = (currentTime - stateStartTime) / 1000;

  switch (currentState) {
    case TRAFFIC_GREEN: {
      int greenDuration = getGreenTime();
      int timeLeft = greenDuration - elapsedTime;
      updateLCDCountdown("Green Light", timeLeft);

      if (elapsedTime >= greenDuration || 
          (pedestrianRequested && elapsedTime >= MIN_GREEN_TIME)) {
        setTrafficState(TRAFFIC_YELLOW);
        stateStartTime = currentTime;
      }
      break;
    }

    case TRAFFIC_YELLOW: {
      int timeLeft = YELLOW_TIME - elapsedTime;
      updateLCDCountdown("Yellow Light", timeLeft);

      if (elapsedTime >= YELLOW_TIME) {
        setTrafficState(TRAFFIC_RED);
        stateStartTime = currentTime;
      }
      break;
    }

    case TRAFFIC_RED: {
      int redDuration = pedestrianRequested ? PED_WALK_TIME : MIN_RED_TIME;
      int timeLeft = redDuration - elapsedTime;

      if (pedestrianRequested) {
        updateLCDCountdown("Walk Signal", timeLeft);
      } else {
        updateLCDCountdown("Red Light", timeLeft);
      }

      if (elapsedTime >= redDuration) {
        setTrafficState(TRAFFIC_GREEN);
        stateStartTime = currentTime;
        pedestrianRequested = false;
      }
      break;
    }
  }
}

void setTrafficState(TrafficState newState) {
  currentState = newState;

  digitalWrite(redLED, LOW);
  digitalWrite(yellowLED, LOW);
  digitalWrite(greenLED, LOW);

  switch (newState) {
    case TRAFFIC_GREEN:
      digitalWrite(greenLED, HIGH);
      digitalWrite(pedRedLED, HIGH);
      digitalWrite(pedGreenLED, LOW);
      bluetooth.println("Traffic: GREEN | Pedestrians: STOP");
      Serial.println("State: Traffic GREEN, Pedestrian RED");
      break;

    case TRAFFIC_YELLOW:
      digitalWrite(yellowLED, HIGH);
      digitalWrite(pedRedLED, HIGH);
      digitalWrite(pedGreenLED, LOW);
      bluetooth.println("Traffic: YELLOW | Pedestrians: STOP");
      Serial.println("State: Traffic YELLOW, Pedestrian RED");
      break;

    case TRAFFIC_RED:
      digitalWrite(redLED, HIGH);
      digitalWrite(pedRedLED, LOW);
      digitalWrite(pedGreenLED, HIGH);
      if (pedestrianRequested) {
        bluetooth.println("Traffic: RED | Pedestrians: WALK (Button Pressed)");
        Serial.println("State: Traffic RED ON, Pedestrian GREEN ON (Button Pressed)");
      } else {
        bluetooth.println("Traffic: RED | Pedestrians: WALK (Auto Cycle)");
        Serial.println("State: Traffic RED ON, Pedestrian GREEN ON (Auto Cycle)");
      }
      break;
  }

  Serial.print("LEDs - Traffic R:");
  Serial.print(digitalRead(redLED));
  Serial.print(" Y:");
  Serial.print(digitalRead(yellowLED));
  Serial.print(" G:");
  Serial.print(digitalRead(greenLED));
  Serial.print(" | Ped R:");
  Serial.print(digitalRead(pedRedLED));
  Serial.print(" G:");
  Serial.println(digitalRead(pedGreenLED));
}

int getGreenTime() {
  switch (trafficLevel) {
    case 1: return 15;
    case 2: return 25;
    case 3: return 35;
    default: return 15;
  }
}

void checkSafetyConditions() {
  long currentDistance = getDistance(trigJaywalk, echoJaywalk);
  bool hazardDetected = false;

  if (currentDistance > 0 && currentDistance < 50 &&
      abs(currentDistance - previousDistance) > 10) {
    hazardDetected = true;
    bluetooth.println(" WARNING: Pedestrian movement detected in traffic area!");
  }

  if (currentDistance > 0 && currentDistance < 4) {
    hazardDetected = true;
    digitalWrite(buzzerPin, HIGH);
    bluetooth.println(" COLLISION ALERT: Object too close!");
    Serial.println("COLLISION ALERT: Distance < 4cm");
  } else {
    digitalWrite(buzzerPin, LOW);
  }

  if (!isRaining) {
    digitalWrite(alertLED, hazardDetected ? HIGH : LOW);
  } else if (hazardDetected) {
    digitalWrite(alertLED, HIGH);
  }

  previousDistance = currentDistance;
}

void checkRainCondition() {
  int moistureValue = analogRead(moisturePin);

  if (moistureValue < 600) {
    if (!isRaining) {
      isRaining = true;
      digitalWrite(alertLED, HIGH);
      bluetooth.println(" RAIN DETECTED: Caution - wet road conditions");
      bluetooth.print(" Moisture level: ");
      bluetooth.println(moistureValue);
      Serial.println("Rain detected - Alert LED ON");
    }
  } else {
    if (isRaining) {
      isRaining = false;
      digitalWrite(alertLED, LOW);
      bluetooth.println(" Rain stopped - road conditions normal");
      bluetooth.print(" Moisture level: ");
      bluetooth.println(moistureValue);
      Serial.println("Rain stopped");
    }
  }
}

long getDistance(int trig, int echo) {
  digitalWrite(trig, LOW);
  delayMicroseconds(2);
  digitalWrite(trig, HIGH);
  delayMicroseconds(10);
  digitalWrite(trig, LOW);

  long duration = pulseIn(echo, HIGH, 30000);
  if (duration == 0) return -1;

  return duration * 0.034 / 2;
}

void updateLCDCountdown(String state, int timeLeft) {
  static String lastState = "";
  static int lastTime = -1;

  if (state != lastState || timeLeft != lastTime) {
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print(state);
    lcd.setCursor(0, 1);

    if (timeLeft > 0) {
      lcd.print("Time left: " + String(timeLeft) + "s");
    } else {
      lcd.print("Changing...");
    }

    lastState = state;
    lastTime = timeLeft;
  }
}

void displayMessage(String line1, String line2) {
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print(line1);
  lcd.setCursor(0, 1);
  lcd.print(line2);
}
