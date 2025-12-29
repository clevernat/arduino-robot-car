// ==========================================
//   ROBOT CAR - SMART SCANNING SYSTEM
//   Version: 6.0
//   Features: Manual, Auto, Follow, Line Tracking
//   NEW: 360° Intelligent Path Planning
// ==========================================

#include <IRremote.hpp>  // Library for IR remote control
#include <Servo.h>       // Library for servo motor control

// ==========================================
//   MOTOR PINS - L298N Motor Driver
// ==========================================
// LEFT MOTOR (Motor A)
const int A_1A = 5;  // Left motor forward (PWM pin)
const int A_1B = 3;  // Left motor reverse (PWM pin)

// RIGHT MOTOR (Motor B)
const int B_1A = 9;  // Right motor forward (PWM pin)
const int B_1B = 6;  // Right motor reverse (PWM pin)

// ==========================================
//   SERVO MOTOR - For Ultrasonic Scanner
// ==========================================
const int SERVO_PIN = 11;         // Servo control pin
const int SERVO_CENTER = 90;      // Center position (degrees)
const int SERVO_RIGHT = 20;       // Right scan position (degrees)
const int SERVO_LEFT = 160;       // Left scan position (degrees)
const int SERVO_BACK = 180;       // Back scan position (degrees)
const int SERVO_STEP = 2;         // Degrees to move per step (smaller = smoother)
const int SERVO_DELAY = 15;       // Milliseconds between steps (higher = slower)

// ==========================================
//   IR REMOTE RECEIVER
// ==========================================
const int IR_RECEIVE_PIN = 12;    // IR receiver signal pin

// ==========================================
//   IR OBSTACLE DETECTION SENSORS
// ==========================================
// These sensors detect obstacles close to the robot
// Output: LOW (0) = Obstacle detected, HIGH (1) = Clear
const int IR_LEFT_PIN = 8;        // Left obstacle sensor
const int IR_RIGHT_PIN = 10;      // Right obstacle sensor

// ==========================================
//   ULTRASONIC DISTANCE SENSOR (HC-SR04)
// ==========================================
// Measures distance to objects (2cm - 400cm range)
const int TRIG_PIN = 7;           // Trigger pin (output)
const int ECHO_PIN = 4;           // Echo pin (input)

// ==========================================
//   LINE TRACKER SENSOR
// ==========================================
// Detects black line on white surface
// Output: LOW (0) = Black line, HIGH (1) = White surface
const int LINE_TRACKER_PIN = 2;   // Line sensor signal pin

// ==========================================
//   REMOTE CONTROL COMMANDS
// ==========================================
// Button codes from your IR remote
const uint8_t CMD_FORWARD = 24;   // UP arrow - Move forward
const uint8_t CMD_BACKWARD = 82;  // DOWN arrow - Move backward
const uint8_t CMD_LEFT = 8;       // LEFT arrow - Turn left
const uint8_t CMD_RIGHT = 90;     // RIGHT arrow - Turn right
const uint8_t CMD_MANUAL = 7;     // Button 7 - Manual mode
const uint8_t CMD_AUTO = 25;      // Button 25 - Auto mode (obstacle avoidance)
const uint8_t CMD_FOLLOW = 22;    // Button 22 - Follow mode (chase object)
const uint8_t CMD_LINE = 70;      // Button 70 - Line following mode
const uint8_t CMD_RESET = 69;     // Button 69 - Stop/Reset

// ==========================================
//   ROBOT MODES
// ==========================================
const int MODE_MANUAL = 0;        // Manual control with remote
const int MODE_AUTO = 1;          // Autonomous obstacle avoidance
const int MODE_FOLLOW = 2;        // Follow detected objects
const int MODE_LINE = 3;          // Follow black line on floor
int currentMode = MODE_MANUAL;    // Start in manual mode

// ==========================================
//   AUTO MODE STATES
// ==========================================
// State machine for autonomous navigation
const int STATE_MOVING = 0;       // Moving forward, scanning
const int STATE_SCANNING = 1;     // 360° scanning for best path
const int STATE_DECIDING = 2;     // Analyzing scan results
const int STATE_MANEUVERING = 3;  // Executing chosen maneuver
int autoState = STATE_MOVING;     // Current auto mode state

// ==========================================
//   MOTOR SPEED SETTINGS
// ==========================================
const int MOTOR_SPEED = 255;      // Maximum motor speed (0-255)
int leftMotorTrim = 15;           // Left motor calibration (+/- adjustment)
int rightMotorTrim = 0;           // Right motor calibration (+/- adjustment)
int leftMotorSpeed = 255;         // Calculated left motor speed
int rightMotorSpeed = 255;        // Calculated right motor speed

// ==========================================
//   LINE FOLLOWING SETTINGS
// ==========================================
const int LINE_SPEED = 200;       // Speed when following line (0-255)
const int LINE_TURN_SPEED = 180;  // Speed when turning to find line (0-255)

// ==========================================
//   FOLLOW MODE SETTINGS
// ==========================================
// Robot will follow objects within this distance
const int FOLLOW_MAX_DIST = 50;   // Maximum follow distance (cm)

// ==========================================
//   OBSTACLE AVOIDANCE SETTINGS
// ==========================================
const int DIST_DANGER = 25;       // Danger threshold in AUTO mode (cm)
const int DIST_SAFE = 40;         // Safe distance threshold (cm)
const int DIST_IDEAL = 80;        // Ideal clear distance (cm)

// ==========================================
//   SCANNING SYSTEM
// ==========================================
// Scan positions and their angles
const int SCAN_POSITIONS = 5;
const int SCAN_FRONT = 0;
const int SCAN_RIGHT = 1;
const int SCAN_LEFT = 2;
const int SCAN_BACK = 3;
const int SCAN_RIGHT45 = 4;

// Servo angles for each scan position
const int scanAngles[SCAN_POSITIONS] = {
  90,   // Front (center)
  20,   // Right
  160,  // Left
  180,  // Back
  45    // Right 45° (optional extra scan)
};

// Distance readings for each position
int scanDistances[SCAN_POSITIONS] = {100, 100, 100, 100, 100};

// Current scan progress
int currentScanPos = 0;
bool scanComplete = false;
unsigned long scanStepStartTime = 0;
const unsigned long SCAN_STEP_DELAY = 300;  // Wait time at each position (ms)

// ==========================================
//   PATH DECISION VARIABLES
// ==========================================
int chosenDirection = SCAN_FRONT;  // Best direction to move
int maneuverPhase = 0;              // Current phase of maneuver
unsigned long maneuverStartTime = 0;

// ==========================================
//   SERVO SCANNING VARIABLES
// ==========================================
Servo headServo;                  // Servo object
int currentServoAngle = SERVO_CENTER;  // Current servo position
int targetServoAngle = SERVO_CENTER;   // Target servo position
unsigned long lastServoMove = 0;       // Last servo movement time

// Quick scanning for normal operation
int scanDir = 0;                  // 0=center, 1=right, 2=left
unsigned long lastScanTime = 0;   // Last scan time
const unsigned long SCAN_WAIT = 500;   // Wait time at each scan position (ms)

// Distance readings from ultrasonic sensor (quick scan)
int distCenter = 100;             // Distance at center (cm)
int distRight = 100;              // Distance at right (cm)
int distLeft = 100;               // Distance at left (cm)

// ==========================================
//   IR OBSTACLE SENSOR FLAGS
// ==========================================
bool irLeft = false;              // true = obstacle on left
bool irRight = false;             // true = obstacle on right

// ==========================================
//   MANUAL MODE VARIABLES
// ==========================================
// Tracks button presses for hold-to-move functionality
uint8_t activeManualCmd = 0;           // Current active movement command
uint8_t lastReceivedCmd = 0;           // Last received command (for repeats)
unsigned long lastCmdTime = 0;         // Last command time
const unsigned long CMD_TIMEOUT = 200; // Button release timeout (ms)

// ==========================================
//   LINE FOLLOWING VARIABLES
// ==========================================
int lastLineTurnDir = 1;               // Last turn direction (1=right, -1=left)
unsigned long lineOffTime = 0;         // Time when line was lost
const unsigned long LINE_LOST_MS = 300; // Time before stopping if line lost (ms)

// ==========================================
//   STATISTICS & MONITORING
// ==========================================
unsigned long startTime = 0;           // Robot start time
int obstacleCount = 0;                 // Number of obstacles avoided
int smartScanCount = 0;                // Number of smart scans performed

// ==========================================
//   SETUP - RUNS ONCE AT STARTUP
// ==========================================
void setup() {
  // Initialize serial communication for debugging
  Serial.begin(115200);
  
  // ===== CONFIGURE MOTOR PINS =====
  pinMode(A_1A, OUTPUT);  // Left motor forward
  pinMode(A_1B, OUTPUT);  // Left motor reverse
  pinMode(B_1A, OUTPUT);  // Right motor forward
  pinMode(B_1B, OUTPUT);  // Right motor reverse
  
  // ===== CONFIGURE SENSOR PINS =====
  pinMode(IR_LEFT_PIN, INPUT);       // Left obstacle sensor (input)
  pinMode(IR_RIGHT_PIN, INPUT);      // Right obstacle sensor (input)
  pinMode(LINE_TRACKER_PIN, INPUT);  // Line tracker sensor (input)
  pinMode(TRIG_PIN, OUTPUT);         // Ultrasonic trigger (output)
  pinMode(ECHO_PIN, INPUT);          // Ultrasonic echo (input)
  
  // ===== INITIALIZE SERVO =====
  headServo.attach(SERVO_PIN);       // Attach servo to pin
  headServo.write(SERVO_CENTER);     // Center the servo
  
  // ===== INITIALIZE IR RECEIVER =====
  IrReceiver.begin(IR_RECEIVE_PIN, ENABLE_LED_FEEDBACK);  // Start IR receiver with LED feedback
  
  // ===== CALCULATE MOTOR SPEEDS =====
  updateMotorSpeeds();  // Apply trim adjustments
  
  // ===== STOP MOTORS =====
  stopMove();  // Ensure motors are off at startup
  
  // ===== PRINT STARTUP INFORMATION =====
  Serial.println(F("\n=== ROBOT CONTROL v6.0 - SMART SCANNING ===\n"));
  Serial.println(F("✨ NEW: 360° Intelligent Path Planning!"));
  Serial.println(F("\nMODES:"));
  Serial.println(F("  7 = MANUAL"));
  Serial.println(F("  25 = AUTO (smart obstacle avoidance)"));
  Serial.println(F("  22 = FOLLOW (chase object)"));
  Serial.println(F("  70 = LINE FOLLOW ⬛"));
  Serial.println(F("  69 = STOP/RESET"));
  Serial.println(F("\nMANUAL: 24=FWD 82=BACK 8=LEFT 90=RIGHT"));
  Serial.print(F("\nMotor Trim: L="));
  Serial.print(leftMotorTrim);
  Serial.print(F(" R="));
  Serial.println(rightMotorTrim);
  Serial.println(F("\nAUTO MODE: Scans Front→Right→Left→Back"));
  Serial.println(F("           Chooses best path automatically!\n"));
  Serial.println(F("Ready!\n"));
  
  // Record startup time
  startTime = millis();
}

// ==========================================
//   MAIN LOOP - RUNS CONTINUOUSLY
// ==========================================
void loop() {
  // ===== PROCESS IR REMOTE SIGNALS =====
  if (IrReceiver.decode()) {  // Check if IR signal received
    uint8_t cmd = IrReceiver.decodedIRData.command;  // Get command code
    bool isRepeat = (IrReceiver.decodedIRData.flags & IRDATA_FLAGS_IS_REPEAT);  // Check if button held
    
    if (cmd != 0) {  // Valid command received
      lastCmdTime = millis();  // Update last command time
      
      if (!isRepeat) {
        // NEW button press
        lastReceivedCmd = cmd;
        handleNewCommand(cmd);  // Process new command
      } else {
        // REPEAT signal (button still held)
        if (currentMode == MODE_MANUAL && isMovementCommand(lastReceivedCmd)) {
          activeManualCmd = lastReceivedCmd;  // Keep movement active
        }
      }
    }
    
    IrReceiver.resume();  // Ready for next signal
  }
  
  // ===== EXECUTE CURRENT MODE =====
  if (currentMode == MODE_MANUAL) {
    runManual();      // Manual control mode
  } else if (currentMode == MODE_AUTO) {
    runAutoSmart();   // Smart autonomous obstacle avoidance
  } else if (currentMode == MODE_FOLLOW) {
    runFollow();      // Follow detected objects
  } else if (currentMode == MODE_LINE) {
    runLineFollow();  // Follow black line
  }
  
  // ===== UPDATE SERVO POSITION =====
  updateServo();  // Smooth servo movement
}

// ==========================================
//   SMART AUTO MODE - 360° SCANNING
// ==========================================
// Enhanced autonomous navigation with intelligent path planning
void runAutoSmart() {
  // ===== READ IR OBSTACLE SENSORS =====
  irLeft = (digitalRead(IR_LEFT_PIN) == LOW);
  irRight = (digitalRead(IR_RIGHT_PIN) == LOW);
  
  // ===== STATE MACHINE =====
  switch (autoState) {
    case STATE_MOVING:
      runAutoMoving();
      break;
      
    case STATE_SCANNING:
      runAutoScanning();
      break;
      
    case STATE_DECIDING:
      runAutoDeciding();
      break;
      
    case STATE_MANEUVERING:
      runAutoManeuvering();
      break;
  }
}

// ==========================================
//   STATE: MOVING - Navigate Forward
// ==========================================
void runAutoMoving() {
  // Check for immediate IR obstacles
  if (irLeft || irRight) {
    Serial.println(F("\n⚠️ IR OBSTACLE DETECTED!"));
    initiateSmartScan();
    return;
  }
  
  // ===== QUICK FRONT SCANNING =====
  if (millis() - lastScanTime >= SCAN_WAIT) {
    if (scanDir == 0) {
      // Scan center
      distCenter = getDist();
      Serial.print(F("Front: "));
      Serial.print(distCenter);
      Serial.println(F("cm"));
      
      // Check for danger ahead
      if (distCenter < DIST_DANGER) {
        Serial.println(F("\n⚠️ OBSTACLE AHEAD!"));
        initiateSmartScan();
        return;
      }
      
      // Quick side scans
      targetServoAngle = SERVO_RIGHT;
      scanDir = 1;
      lastScanTime = millis();
    }
    else if (scanDir == 1) {
      // Scan right
      distRight = getDist();
      targetServoAngle = SERVO_LEFT;
      scanDir = 2;
      lastScanTime = millis();
    }
    else {
      // Scan left
      distLeft = getDist();
      targetServoAngle = SERVO_CENTER;
      scanDir = 0;
      lastScanTime = millis();
    }
  }
  
  // Keep moving forward
  moveForward();
}

// ==========================================
//   INITIATE SMART SCAN
// ==========================================
void initiateSmartScan() {
  Serial.println(F("\n🔍 STARTING 360° SMART SCAN..."));
  smartScanCount++;
  obstacleCount++;
  
  // Stop and prepare for scanning
  stopMove();
  
  // Reset scan variables
  currentScanPos = 0;
  scanComplete = false;
  for (int i = 0; i < SCAN_POSITIONS; i++) {
    scanDistances[i] = 0;
  }
  
  // Set first scan position
  targetServoAngle = scanAngles[SCAN_FRONT];
  scanStepStartTime = millis();
  
  // Change to scanning state
  autoState = STATE_SCANNING;
}

// ==========================================
//   STATE: SCANNING - Measure All Directions
// ==========================================
void runAutoScanning() {
  // Wait for servo to reach target position
  if (abs(currentServoAngle - targetServoAngle) > 5) {
    return;  // Still moving servo
  }
  
  // Wait at position before measuring
  if (millis() - scanStepStartTime < SCAN_STEP_DELAY) {
    return;  // Wait for stabilization
  }
  
  // ===== MEASURE CURRENT POSITION =====
  scanDistances[currentScanPos] = getDist();
  
  // Print scan result
  Serial.print(F("  "));
  switch(currentScanPos) {
    case SCAN_FRONT:
      Serial.print(F("⬆️ FRONT: "));
      break;
    case SCAN_RIGHT:
      Serial.print(F("➡️ RIGHT: "));
      break;
    case SCAN_LEFT:
      Serial.print(F("⬅️ LEFT: "));
      break;
    case SCAN_BACK:
      Serial.print(F("⬇️ BACK: "));
      break;
    case SCAN_RIGHT45:
      Serial.print(F("↗️ RIGHT-45: "));
      break;
  }
  Serial.print(scanDistances[currentScanPos]);
  Serial.println(F("cm"));
  
  // ===== MOVE TO NEXT POSITION =====
  currentScanPos++;
  
  if (currentScanPos >= SCAN_POSITIONS) {
    // ===== SCAN COMPLETE =====
    scanComplete = true;
    Serial.println(F("\n✅ SCAN COMPLETE!"));
    autoState = STATE_DECIDING;
  } else {
    // Move to next scan position
    targetServoAngle = scanAngles[currentScanPos];
    scanStepStartTime = millis();
  }
}

// ==========================================
//   STATE: DECIDING - Choose Best Path
// ==========================================
void runAutoDeciding() {
  Serial.println(F("\n🧠 ANALYZING PATHS..."));
  
  // ===== EVALUATE EACH DIRECTION =====
  int bestDistance = -1;
  int bestDirection = SCAN_BACK;  // Default to backing up
  
  for (int i = 0; i < SCAN_POSITIONS; i++) {
    int distance = scanDistances[i];
    
    // Skip back direction unless it's the only option
    if (i == SCAN_BACK && bestDistance > 0) {
      continue;
    }
    
    // Find direction with most space
    if (distance > bestDistance) {
      bestDistance = distance;
      bestDirection = i;
    }
  }
  
  // ===== PRINT DECISION =====
  Serial.print(F("🎯 BEST PATH: "));
  switch(bestDirection) {
    case SCAN_FRONT:
      Serial.print(F("⬆️ FRONT"));
      break;
    case SCAN_RIGHT:
      Serial.print(F("➡️ RIGHT"));
      break;
    case SCAN_LEFT:
      Serial.print(F("⬅️ LEFT"));
      break;
    case SCAN_BACK:
      Serial.print(F("⬇️ BACK"));
      break;
    case SCAN_RIGHT45:
      Serial.print(F("↗️ RIGHT-45"));
      break;
  }
  Serial.print(F(" ("));
  Serial.print(bestDistance);
  Serial.println(F("cm)"));
  
  // ===== CHECK IF PATH IS SAFE =====
  if (bestDistance < DIST_DANGER && bestDirection != SCAN_BACK) {
    Serial.println(F("⚠️ All paths blocked! Going BACK!"));
    bestDirection = SCAN_BACK;
  }
  
  // Store decision
  chosenDirection = bestDirection;
  
  // Reset servo to center for maneuvering
  targetServoAngle = SERVO_CENTER;
  
  // Start maneuver
  maneuverPhase = 0;
  maneuverStartTime = millis();
  autoState = STATE_MANEUVERING;
}

// ==========================================
//   STATE: MANEUVERING - Execute Chosen Path
// ==========================================
void runAutoManeuvering() {
  unsigned long elapsed = millis() - maneuverStartTime;
  
  switch(chosenDirection) {
    case SCAN_FRONT:
      // ===== FRONT IS CLEAR - GO FORWARD =====
      if (elapsed < 500) {
        moveForward();
      } else {
        resumeNormalOperation();
      }
      break;
      
    case SCAN_RIGHT:
      // ===== TURN RIGHT AND GO =====
      if (elapsed < 400) {
        // Phase 1: Back up slightly
        moveBackward();
      } else if (elapsed < 1000) {
        // Phase 2: Turn right
        moveRight();
      } else if (elapsed < 1500) {
        // Phase 3: Move forward
        moveForward();
      } else {
        resumeNormalOperation();
      }
      break;
      
    case SCAN_LEFT:
      // ===== TURN LEFT AND GO =====
      if (elapsed < 400) {
        // Phase 1: Back up slightly
        moveBackward();
      } else if (elapsed < 1000) {
        // Phase 2: Turn left
        moveLeft();
      } else if (elapsed < 1500) {
        // Phase 3: Move forward
        moveForward();
      } else {
        resumeNormalOperation();
      }
      break;
      
    case SCAN_BACK:
      // ===== GO BACKWARD AND TURN =====
      if (elapsed < 800) {
        // Phase 1: Back up
        moveBackward();
        Serial.println(F("⬇️ BACKING UP..."));
      } else if (elapsed < 1500) {
        // Phase 2: Turn around (180°)
        moveRight();
        Serial.println(F("🔄 TURNING AROUND..."));
      } else if (elapsed < 2000) {
        // Phase 3: Move forward in new direction
        moveForward();
      } else {
        resumeNormalOperation();
      }
      break;
      
    case SCAN_RIGHT45:
      // ===== SLIGHT RIGHT TURN =====
      if (elapsed < 300) {
        moveBackward();
      } else if (elapsed < 700) {
        moveRight();
      } else if (elapsed < 1200) {
        moveForward();
      } else {
        resumeNormalOperation();
      }
      break;
  }
}

// ==========================================
//   RESUME NORMAL OPERATION
// ==========================================
void resumeNormalOperation() {
  Serial.println(F("✅ Maneuver complete - Resuming navigation\n"));
  
  // Reset to moving state
  autoState = STATE_MOVING;
  scanDir = 0;
  lastScanTime = millis();
  distCenter = distLeft = distRight = 100;
  
  // Continue moving forward
  moveForward();
}

// ==========================================
//   LINE FOLLOWING MODE
// ==========================================
// Follows a black line on white surface
void runLineFollow() {
  // Read line sensor (LOW = black line, HIGH = white surface)
  int lineValue = digitalRead(LINE_TRACKER_PIN);
  
  if (lineValue == LOW) {
    // ===== ON BLACK LINE - GO FORWARD =====
    lineOffTime = 0;  // Reset lost timer
    
    // Move forward at line following speed
    analogWrite(A_1A, LINE_SPEED);
    analogWrite(A_1B, 0);
    analogWrite(B_1A, LINE_SPEED);
    analogWrite(B_1B, 0);
  }
  else {
    // ===== OFF LINE - ON WHITE SURFACE =====
    
    // Start lost timer if not already started
    if (lineOffTime == 0) {
      lineOffTime = millis();
    }
    
    // Check if line lost for too long
    if (millis() - lineOffTime > LINE_LOST_MS) {
      // Line completely lost - STOP
      stopMove();
    } else {
      // Turn to find line (use last successful turn direction)
      if (lastLineTurnDir > 0) {
        // Turn RIGHT to find line
        analogWrite(A_1A, LINE_TURN_SPEED);
        analogWrite(A_1B, 0);
        analogWrite(B_1A, 0);
        analogWrite(B_1B, LINE_TURN_SPEED);
      } else {
        // Turn LEFT to find line
        analogWrite(A_1A, 0);
        analogWrite(A_1B, LINE_TURN_SPEED);
        analogWrite(B_1A, LINE_TURN_SPEED);
        analogWrite(B_1B, 0);
      }
    }
  }
  
  delay(50);  // Small delay for stability
}

// ==========================================
//   FOLLOW MODE - CHASE OBJECTS
// ==========================================
// Uses all three sensors for robust object following
void runFollow() {
  // ===== READ ALL SENSORS =====
  irLeft = (digitalRead(IR_LEFT_PIN) == LOW);   // Left obstacle detected?
  irRight = (digitalRead(IR_RIGHT_PIN) == LOW); // Right obstacle detected?
  int dist = getDist();                          // Get distance from ultrasonic
  
  // ===== CHECK IF ANY SENSOR DETECTS OBJECT =====
  bool ultrasonicDetects = (dist < FOLLOW_MAX_DIST && dist > 0);  // Object within range?
  bool irDetects = (irLeft || irRight);                            // IR sensors detect?
  
  if (ultrasonicDetects || irDetects) {
    // ===== OBJECT DETECTED - FOLLOW IT! =====
    
    // Determine direction based on IR sensors
    if (irLeft && irRight) {
      // ===== BOTH IR SENSORS DETECT - GO STRAIGHT =====
      analogWrite(A_1A, 255);  // Left motor full speed
      analogWrite(A_1B, 0);
      analogWrite(B_1A, 255);  // Right motor full speed
      analogWrite(B_1B, 0);
    }
    else if (irLeft && !irRight) {
      // ===== ONLY LEFT IR DETECTS - TURN LEFT WHILE FOLLOWING =====
      analogWrite(A_1A, 180);  // Left motor slower
      analogWrite(A_1B, 0);
      analogWrite(B_1A, 255);  // Right motor faster
      analogWrite(B_1B, 0);
    }
    else if (irRight && !irLeft) {
      // ===== ONLY RIGHT IR DETECTS - TURN RIGHT WHILE FOLLOWING =====
      analogWrite(A_1A, 255);  // Left motor faster
      analogWrite(A_1B, 0);
      analogWrite(B_1A, 180);  // Right motor slower
      analogWrite(B_1B, 0);
    }
    else if (ultrasonicDetects) {
      // ===== ONLY ULTRASONIC DETECTS - GO STRAIGHT =====
      analogWrite(A_1A, 255);
      analogWrite(A_1B, 0);
      analogWrite(B_1A, 255);
      analogWrite(B_1B, 0);
    }
  }
  else {
    // ===== NO OBJECT DETECTED - STOP AND WAIT =====
    stopMove();
  }
  
  delay(50);  // Small delay for stability
}

// ==========================================
//   MANUAL MODE - REMOTE CONTROL
// ==========================================
// Hold buttons to move, release to stop
void runManual() {
  // Check if button released (no command for CMD_TIMEOUT ms)
  if (millis() - lastCmdTime > CMD_TIMEOUT) {
    if (activeManualCmd != 0) {
      // Button released - stop robot
      activeManualCmd = 0;
      stopMove();
    }
  } else {
    // Button still held - execute movement
    executeMovement(activeManualCmd);
  }
}

// ==========================================
//   CHECK IF COMMAND IS A MOVEMENT
// ==========================================
bool isMovementCommand(uint8_t cmd) {
  return (cmd == CMD_FORWARD || cmd == CMD_BACKWARD || 
          cmd == CMD_LEFT || cmd == CMD_RIGHT);
}

// ==========================================
//   HANDLE NEW REMOTE COMMAND
// ==========================================
void handleNewCommand(uint8_t cmd) {
  // ===== MODE SWITCHING COMMANDS =====
  if (cmd == CMD_MANUAL) {
    switchToManual();
    return;
  }
  
  if (cmd == CMD_AUTO) {
    switchToAuto();
    return;
  }
  
  if (cmd == CMD_FOLLOW) {
    switchToFollow();
    return;
  }
  
  if (cmd == CMD_LINE) {
    switchToLineFollow();
    return;
  }
  
  if (cmd == CMD_RESET) {
    resetRobot();
    return;
  }
  
  // ===== MANUAL MOVEMENT COMMANDS =====
  if (currentMode == MODE_MANUAL && isMovementCommand(cmd)) {
    activeManualCmd = cmd;  // Store command for continuous execution
    executeMovement(cmd);  // Start movement
  }
}

// ==========================================
//   EXECUTE MOVEMENT COMMAND
// ==========================================
void executeMovement(uint8_t cmd) {
  if (cmd == 0) {
    stopMove();
    return;
  }
  
  switch(cmd) {
    case CMD_FORWARD:
      moveForward();
      break;
    case CMD_BACKWARD:
      moveBackward();
      break;
    case CMD_LEFT:
      moveLeft();
      break;
    case CMD_RIGHT:
      moveRight();
      break;
    default:
      stopMove();
      break;
  }
}

// ==========================================
//   MODE SWITCHING FUNCTIONS
// ==========================================

// Switch to MANUAL mode (remote control)
void switchToManual() {
  currentMode = MODE_MANUAL;
  activeManualCmd = 0;
  lastReceivedCmd = 0;
  stopMove();
  targetServoAngle = SERVO_CENTER;  // Center servo
  Serial.println(F("\n[MANUAL MODE]"));
}

// Switch to AUTO mode (obstacle avoidance)
void switchToAuto() {
  currentMode = MODE_AUTO;
  activeManualCmd = 0;
  lastReceivedCmd = 0;
  autoState = STATE_MOVING;         // Start in moving state
  targetServoAngle = SERVO_CENTER;  // Center servo
  scanDir = 0;                       // Reset scan direction
  lastScanTime = millis();
  distCenter = distLeft = distRight = 100;  // Reset distances
  Serial.println(F("\n[SMART AUTO MODE] 🧠"));
  Serial.println(F("360° Scanning Enabled!"));
  moveForward();  // Start moving
}

// Switch to FOLLOW mode (chase objects)
void switchToFollow() {
  currentMode = MODE_FOLLOW;
  activeManualCmd = 0;
  lastReceivedCmd = 0;
  stopMove();
  targetServoAngle = SERVO_CENTER;  // Center servo
  Serial.println(F("\n[FOLLOW MODE]"));
}

// Switch to LINE FOLLOWING mode
void switchToLineFollow() {
  currentMode = MODE_LINE;
  activeManualCmd = 0;
  lastReceivedCmd = 0;
  stopMove();
  targetServoAngle = SERVO_CENTER;  // Center servo
  lineOffTime = 0;                   // Reset line lost timer
  lastLineTurnDir = 1;               // Default turn right
  Serial.println(F("\n[LINE FOLLOWING MODE] ⬛"));
}

// RESET robot to manual mode
void resetRobot() {
  stopMove();
  activeManualCmd = 0;
  lastReceivedCmd = 0;
  targetServoAngle = SERVO_CENTER;  // Center servo
  
  Serial.println(F("\n=== STOPPED ==="));
  Serial.print(F("Smart Scans: "));
  Serial.println(smartScanCount);
  Serial.print(F("Obstacles: "));
  Serial.println(obstacleCount);
  Serial.println();
  
  // Return to manual mode
  currentMode = MODE_MANUAL;
  autoState = STATE_MOVING;
  obstacleCount = 0;
  smartScanCount = 0;
  startTime = millis();
  Serial.println(F("[MANUAL MODE]"));
}

// ==========================================
//   SERVO CONTROL - SMOOTH MOVEMENT
// ==========================================
// Moves servo smoothly to target angle
void updateServo() {
  if (millis() - lastServoMove >= SERVO_DELAY) {
    if (currentServoAngle < targetServoAngle) {
      // Move servo right (increase angle)
      currentServoAngle += SERVO_STEP;
      if (currentServoAngle > targetServoAngle) {
        currentServoAngle = targetServoAngle;  // Don't overshoot
      }
      headServo.write(currentServoAngle);
      lastServoMove = millis();
    }
    else if (currentServoAngle > targetServoAngle) {
      // Move servo left (decrease angle)
      currentServoAngle -= SERVO_STEP;
      if (currentServoAngle < targetServoAngle) {
        currentServoAngle = targetServoAngle;  // Don't overshoot
      }
      headServo.write(currentServoAngle);
      lastServoMove = millis();
    }
  }
}

// ==========================================
//   ULTRASONIC DISTANCE MEASUREMENT
// ==========================================
// Returns distance in centimeters (0-400cm range)
int getDist() {
  // Send 10us pulse to trigger pin
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);
  
  // Read echo pulse duration (timeout after 30ms)
  long duration = pulseIn(ECHO_PIN, HIGH, 30000);
  
  // Check for timeout (no echo)
  if (duration == 0) {
    return 200;  // Return large value if no echo
  }
  
  // Calculate distance in cm
  int distance = (duration * 343) / 20000;
  return distance;
}

// ==========================================
//   MOTOR SPEED CALCULATION
// ==========================================
// Applies trim adjustments to motor speeds
void updateMotorSpeeds() {
  // Add trim to base speed and constrain to 0-255
  leftMotorSpeed = constrain(MOTOR_SPEED + leftMotorTrim, 0, 255);
  rightMotorSpeed = constrain(MOTOR_SPEED + rightMotorTrim, 0, 255);
}

// ==========================================
//   MOTOR CONTROL FUNCTIONS
// ==========================================

// Move FORWARD at calibrated speed
void moveForward() {
  analogWrite(A_1A, leftMotorSpeed);   // Left motor forward
  analogWrite(A_1B, 0);                // Left motor reverse OFF
  analogWrite(B_1A, rightMotorSpeed);  // Right motor forward
  analogWrite(B_1B, 0);                // Right motor reverse OFF
}

// Move BACKWARD at calibrated speed
void moveBackward() {
  analogWrite(A_1A, 0);                // Left motor forward OFF
  analogWrite(A_1B, leftMotorSpeed);   // Left motor reverse
  analogWrite(B_1A, 0);                // Right motor forward OFF
  analogWrite(B_1B, rightMotorSpeed);  // Right motor reverse
}

// Turn LEFT (left motor reverse, right motor forward)
void moveLeft() {
  analogWrite(A_1A, 0);                // Left motor forward OFF
  analogWrite(A_1B, leftMotorSpeed);   // Left motor reverse
  analogWrite(B_1A, rightMotorSpeed);  // Right motor forward
  analogWrite(B_1B, 0);                // Right motor reverse OFF
}

// Turn RIGHT (left motor forward, right motor reverse)
void moveRight() {
  analogWrite(A_1A, leftMotorSpeed);   // Left motor forward
  analogWrite(A_1B, 0);                // Left motor reverse OFF
  analogWrite(B_1A, 0);                // Right motor forward OFF
  analogWrite(B_1B, rightMotorSpeed);  // Right motor reverse
}

// STOP all motors
void stopMove() {
  analogWrite(A_1A, 0);  // Left motor forward OFF
  analogWrite(A_1B, 0);  // Left motor reverse OFF
  analogWrite(B_1A, 0);  // Right motor forward OFF
  analogWrite(B_1B, 0);  // Right motor reverse OFF
}

// ==========================================
//   END OF CODE
// ==========================================