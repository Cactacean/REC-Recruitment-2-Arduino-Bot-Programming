// ===========================================================
// 5-SENSOR LINE FOLLOWER (Non-blocking, strict toggle, 90° turns,
//                       servo with 10s cooldown, frequent checks)
// ===========================================================
#include <Servo.h>
Servo myServo;

// ---------- Timers & states ----------
unsigned long nowMillis = 0;

unsigned long moveUntil = 0;        // time until which currentAction is held
int currentAction = 0;              // 0=stop,1=forward,2=slightL,3=slightR,4=hardL,5=hardR,7=turnR90,8=turnL90
int lastAction = 0;                 // last non-zero movement action

// ---------- Servo cooldown & phases (non-blocking) ----------
unsigned long lastServoTrigger = 0;
const unsigned long SERVO_COOLDOWN = 10000UL; // 10 seconds cooldown
bool servoActive = false;
int servoPhase = 0;                 // 0=idle, 1=pre-wait, 2=rotated, 3=returned
unsigned long servoPhaseUntil = 0;

// ---------- Pins ----------
int EN12 = 5;   // Left motor PWM
int IN1  = 6;
int IN2  = 7;

int EN34 = 11;  // Right motor PWM
int IN3  = 9;
int IN4  = 10;

int s1 = A0;  // leftmost
int s2 = A1;
int s3 = A2;  // center
int s4 = A3;
int s5 = A4;  // rightmost

int buttonPin = 4;

// ---------- Button debounce ----------
bool isRunning = false;
int lastRawButtonState = HIGH;
int stableButtonState = HIGH;
unsigned long lastButtonDebounce = 0;
const unsigned long BUTTON_DEBOUNCE_MS = 50;

// ---------- Tuning (change these to tune behavior) ----------
int baseSpeed   = 200;              // base forward PWM
int rightBoost  = 90;               // boost to right motor (your right is weaker)
int turnOffset  = 80;               // how strong slight turns are
// durations (ms) — short so the loop re-evaluates frequently
const unsigned long DUR_FORWARD  = 60UL;
const unsigned long DUR_SLIGHT   = 80UL;
const unsigned long DUR_HARD     = 180UL;
const unsigned long DUR_TURN90   = 380UL; // how long to hold full-power 90° turn

// ===========================================================
// Setup
// ===========================================================
void setup() {
  pinMode(EN12, OUTPUT); pinMode(IN1, OUTPUT); pinMode(IN2, OUTPUT);
  pinMode(EN34, OUTPUT); pinMode(IN3, OUTPUT); pinMode(IN4, OUTPUT);

  pinMode(s1, INPUT); pinMode(s2, INPUT); pinMode(s3, INPUT);
  pinMode(s4, INPUT); pinMode(s5, INPUT);

  pinMode(buttonPin, INPUT_PULLUP);

  myServo.attach(8);       // servo signal on pin 8
  myServo.write(90);       // center
  lastServoTrigger = 0;
  servoActive = false;
  servoPhase = 0;

  Serial.begin(9600);
  stopMotors();
}

// ===========================================================
// Non-blocking servo state machine
// Phases:
// 1 = pre-wait (pause before moving)  e.g. 500ms
// 2 = move to drop angle (write 180), wait ~400ms
// 3 = return to center (write 90), wait ~400ms then finish
// ===========================================================
void updateServoState() {
  if (!servoActive) return;

  nowMillis = millis();
  if (nowMillis < servoPhaseUntil) return;

  if (servoPhase == 1) {
    // rotate clockwise (drop)
    myServo.write(180);
    servoPhase = 2;
    servoPhaseUntil = nowMillis + 400UL;
  }
  else if (servoPhase == 2) {
    // return to center
    myServo.write(90);
    servoPhase = 3;
    servoPhaseUntil = nowMillis + 400UL;
  }
  else if (servoPhase == 3) {
    // finished
    servoActive = false;
    servoPhase = 0;
    lastServoTrigger = nowMillis;
    // resume forward immediately with a short momentum
    currentAction = 1;
    moveUntil = nowMillis + DUR_FORWARD;
    runCurrentAction();
  }
}

// Trigger servo action non-blocking (if cooldown passed)
void triggerServoNonBlocking() {
  nowMillis = millis();
  if (servoActive) return; // already running
  if ((nowMillis - lastServoTrigger) < SERVO_COOLDOWN) return; // still cooling down

  // start servo sequence
  stopMotors();
  servoActive = true;
  servoPhase = 1;
  servoPhaseUntil = nowMillis + 500UL; // pre-wait 500ms
}

// ===========================================================
// Motor helpers (no delays)
// ===========================================================
void moveForward() {
  digitalWrite(IN1, LOW); digitalWrite(IN2, HIGH);   // left forward
  digitalWrite(IN3, LOW); digitalWrite(IN4, HIGH);   // right forward
  analogWrite(EN12, constrain(baseSpeed, 0, 255));
  analogWrite(EN34, constrain(baseSpeed + rightBoost, 0, 255));
}

void slightLeft() {
  digitalWrite(IN1, LOW); digitalWrite(IN2, HIGH);   // left forward (reduced)
  digitalWrite(IN3, LOW); digitalWrite(IN4, HIGH);   // right forward
  analogWrite(EN12, constrain(baseSpeed - turnOffset, 0, 255));
  analogWrite(EN34, constrain(baseSpeed + rightBoost, 0, 255));
}

void slightRight() {
  digitalWrite(IN1, LOW); digitalWrite(IN2, HIGH);
  digitalWrite(IN3, LOW); digitalWrite(IN4, HIGH);
  analogWrite(EN12, constrain(baseSpeed, 0, 255));
  analogWrite(EN34, constrain(baseSpeed + rightBoost - turnOffset, 0, 255));
}

void hardLeft() {
  // left forward stronger, right braked
  digitalWrite(IN1, LOW); digitalWrite(IN2, HIGH);
  digitalWrite(IN3, HIGH); digitalWrite(IN4, HIGH);
  analogWrite(EN12, constrain(baseSpeed, 0, 255));
  analogWrite(EN34, 0);
}

void hardRight() {
  // left braked, right forward
  digitalWrite(IN1, HIGH); digitalWrite(IN2, HIGH);
  digitalWrite(IN3, LOW); digitalWrite(IN4, HIGH);
  analogWrite(EN12, 0);
  analogWrite(EN34, constrain(baseSpeed + rightBoost, 0, 255));
}

void turnRight90Start() {
  // full-power pivot for right 90: left forward, right forward (opposite dirs can be used if needed)
  digitalWrite(IN1, HIGH); digitalWrite(IN2, LOW);   // left backward
  digitalWrite(IN3, LOW);  digitalWrite(IN4, HIGH); // right forward
  analogWrite(EN12, 255);
  analogWrite(EN34, 255);
}

void turnLeft90Start() {
  digitalWrite(IN1, LOW);  digitalWrite(IN2, HIGH); // left forward
  digitalWrite(IN3, HIGH); digitalWrite(IN4, LOW);  // right backward
  analogWrite(EN12, 255);
  analogWrite(EN34, 255);
}

void stopMotors() {
  // active brake
  digitalWrite(IN1, HIGH); digitalWrite(IN2, HIGH);
  digitalWrite(IN3, HIGH); digitalWrite(IN4, HIGH);
  analogWrite(EN12, 0);
  analogWrite(EN34, 0);
}

// Execute currentAction (non-blocking - motors set, durations handled by moveUntil)
void runCurrentAction() {
  switch (currentAction) {
    case 1: moveForward();  break;
    case 2: slightLeft();   break;
    case 3: slightRight();  break;
    case 4: hardLeft();     break;
    case 5: hardRight();    break;
    case 7: turnRight90Start(); break;
    case 8: turnLeft90Start();  break;
    default: stopMotors();  break;
  }
}

// ===========================================================
// Button handling (strict edge detection + debounce)
// returns true once per physical press
// ===========================================================
bool readButtonPressed() {
  bool pressed = false;
  int reading = digitalRead(buttonPin); // HIGH when not pressed (pull-up)

  if (reading != lastRawButtonState) {
    lastButtonDebounce = millis();
    lastRawButtonState = reading;
  } else {
    if ((millis() - lastButtonDebounce) > BUTTON_DEBOUNCE_MS) {
      if (reading != stableButtonState) {
        stableButtonState = reading;
        if (stableButtonState == LOW) { // falling edge (pressed)
          pressed = true;
        }
      }
    }
  }
  return pressed;
}

// ===========================================================
// Main loop - non-blocking, frequent checks
// ===========================================================
void loop() {
  nowMillis = millis();

  // 1) update button (strict)
  if (readButtonPressed()) {
    isRunning = !isRunning;
    if (!isRunning) {
      stopMotors();
    } else {
      // small safety pause to avoid immediate accidental re-press
      // (non-blocking: just record time and ignore)
      // not required but recommended: give motors a tiny moment
      moveUntil = nowMillis + 50;
    }
  }

  // always process servo state machine first (so servo phases progress even if stopped?)
  updateServoState();

  if (!isRunning) return;

  // 2) read sensors (HIGH=white for many sensors; treat >500 analog as black)
  bool b1 = analogRead(s1) > 500;
  bool b2 = analogRead(s2) > 500;
  bool b3 = analogRead(s3) > 500;
  bool b4 = analogRead(s4) > 500;
  bool b5 = analogRead(s5) > 500;

  // 3) If currently inside a timed action window, keep executing that action.
  //    This allows frequent re-evaluation because durations are short.
  if (nowMillis < moveUntil) {
    runCurrentAction();
    return;
  }

  // 4) DECISION LOGIC (turns prioritized)
  // First: servo trigger (all black) — non-blocking and respects cooldown
  if (b1 && b2 && b3 && b4 && b5) {
    triggerServoNonBlocking();
    // servoActive started; while servoActive we'll keep motors stopped until servo finishes
    currentAction = 0;
    moveUntil = nowMillis; // no movement window
    runCurrentAction();
    return;
  }

  // 90° detection (patterns explained):
  // RIGHT 90: left-heavy triple (b1,b2,b3) while right side clear
  if (b1 && b2 && b3 && !b4 && !b5) {
    currentAction = 7;                  // turn right 90
    moveUntil = nowMillis + DUR_TURN90;
    runCurrentAction();
    return;
  }
  // 90° detection (patterns explained):
  // RIGHT 90: left-heavy triple (b1,b2,b3) while right side clear
  if (b1 && b2 && !b3 && !b4 && !b5) {
    currentAction = 7;                  // turn right 90
    moveUntil = nowMillis + DUR_TURN90;
    runCurrentAction();
    return;
  }
  // LEFT 90: right-heavy triple (b3,b4,b5) while left side clear
  if (!b1 && !b2 && b3 && b4 && b5) {
    currentAction = 8;                  // turn left 90
    moveUntil = nowMillis + DUR_TURN90;
    runCurrentAction();
    return;
  }
  if (!b1 && !b2 && !b3 && b4 && b5) {
    currentAction = 8;                  // turn left 90
    moveUntil = nowMillis + DUR_TURN90;
    runCurrentAction();
    return;
  }
  // Priority: hard turns override slight/forward
  if (b1) {
    currentAction = 4;                  // hard left
    moveUntil = nowMillis + DUR_HARD;
    lastAction = currentAction;
    runCurrentAction();
    return;
  }
  if (b5) {
    currentAction = 5;                  // hard right
    moveUntil = nowMillis + DUR_HARD;
    lastAction = currentAction;
    runCurrentAction();
    return;
  }

  // slight corrections
  if (b2) {
    currentAction = 2;                  // slight left
    moveUntil = nowMillis + DUR_SLIGHT;
    lastAction = currentAction;
    runCurrentAction();
    return;
  }
  if (b4) {
    currentAction = 3;                  // slight right
    moveUntil = nowMillis + DUR_SLIGHT;
    lastAction = currentAction;
    runCurrentAction();
    return;
  }

  // forward only if center sees the line
  if (b3) {
    currentAction = 1;
    moveUntil = nowMillis + DUR_FORWARD;
    lastAction = currentAction;
    runCurrentAction();
    return;
  }

  // no sensor sees black → continue lastAction for short retry or stop if none
  if (lastAction != 0) {
    currentAction = lastAction;
    // keep turning in the last direction briefly to search
    moveUntil = nowMillis + DUR_HARD; // try a harder sweep
    runCurrentAction();
    return;
  } else {
    currentAction = 0;
    moveUntil = nowMillis;
    stopMotors();
    return;
  }
}
