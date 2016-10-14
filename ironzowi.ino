#include <avr/pgmspace.h>
#include <Wire.h>
#include "Adafruit_PWMServoDriver.h"

// S0 - Right Shoulder
// S1 - Right Shoulder
// S2 - Right Shoulder
// S3 - Right Shoulder
// S4 - Left Shoulder
// S5 - Right Shoulder
// S6 - Left Arm
// S7 - Right Arm


Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();
// you can also call it with a different address you want
//Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x41);

// Depending on your servo make, the pulse width min and max may vary, you 
// want these to be as small/large as possible without hitting the hard stop
// for max range. You'll have to tweak them as necessary to match the servos you
// have!
#define SERVOMIN  120 // this is the 'minimum' pulse length count (out of 4096)
#define SERVOMAX  600 // this is the 'maximum' pulse length count (out of 4096)
#define SERVOMID  SERVOMIN + ((SERVOMAX - SERVOMIN) >> 1)

#define TESTMIN 320
#define TESTMAX 410

// our servo # counter
uint8_t servonum = 0;

// servo state
uint16_t servo_state[16][5] = {{SERVOMID, SERVOMIN, SERVOMAX, 0, 180}, {SERVOMID, SERVOMIN, SERVOMAX, 180, 0},
                               {SERVOMID, SERVOMIN, SERVOMAX, 0, 180}, {SERVOMID, SERVOMIN, SERVOMAX, 180, 0},
                               {SERVOMID, SERVOMIN, SERVOMAX, 0, 180}, {SERVOMID, SERVOMIN, SERVOMAX, 180, 0},
                               {SERVOMID, SERVOMIN, SERVOMAX, 0, 180}, {SERVOMID, SERVOMIN, SERVOMAX, 180, 0},
                               {SERVOMID, SERVOMIN, SERVOMAX, 0, 180}, {SERVOMID, SERVOMIN, SERVOMAX, 180, 0},
                               {SERVOMID, SERVOMIN, SERVOMAX, 0, 180}, {SERVOMID, SERVOMIN, SERVOMAX, 180, 0},
                               {SERVOMID, SERVOMIN, SERVOMAX, 0, 180}, {SERVOMID, SERVOMIN, SERVOMAX, 180, 0},
                               {SERVOMID, SERVOMIN, SERVOMAX, 0, 180}, {SERVOMID, SERVOMIN, SERVOMAX, 180, 0}};


int sqn_reset[9][3] = {{0, 330, 0}, {1, 330, 0}, {2, 340, 0}, {3, 350, 0}, {4, 375, 0}, {5, 375, 0}, {6, 150, 0}, {7, 600, 0}, {-1, -1, -1}};
int sqn_standup[3][3] = {{0, 600, 0}, {1, 120, 0}, {-1, -1, -1}};
int sqn_waveleft[10][3] = {{6, 150, 0}, {4, 150, 400}, {6, 210, 400}, {6, 150, 400}, {6, 210, 400}, {6, 150, 400}, {6, 210, 400}, {6, 150, 400}, {4, 375, 0}, {-1, -1, -1}};

int sqn_waveright[10][3] = {{7, 560, 0}, {5, 600, 400}, {7, 530, 400}, {7, 600, 400}, {7, 530, 400}, {7, 600, 400}, {7, 530, 400}, {7, 600, 400}, {5, 350, 0}, {-1, -1, -1}};

int sqn_walk[37][3] = {{0, 330, 0}, {1, 330, 0}, {2, 340, 0}, {3, 350, 0}, {6, 150, 0}, {7, 600, 1000},       // reset
                       {0, 270, 0}, {1, 270, 1000}, {4, 600, 0}, {5, 600, 500}, {2, 400, 0}, {3, 400, 500},   // shift left + push right forward
                       {0, 400, 0}, {1, 400, 1000}, {4, 150, 0}, {5, 150, 500}, {2, 270, 0}, {3, 270, 1000},  // shift right + push left forward
                       {0, 270, 0}, {1, 270, 1000}, {4, 600, 0}, {5, 600, 500}, {2, 400, 0}, {3, 400, 1000},  // shift left + push right forward
                       {0, 400, 0}, {1, 400, 1000}, {4, 150, 0}, {5, 150, 500}, {2, 270, 0}, {3, 270, 1000},  // shift right + push left forward
                       {0, 330, 0}, {1, 330, 0}, {2, 340, 0}, {3, 350, 0}, {4, 375, 0}, {5, 375, 0},          // reset
                       {-1, -1, -1}};

int sqn_starjump[25][3] = {{0, 330, 0}, {1, 330, 0}, {2, 340, 0}, {3, 350, 0}, {4, 375, 0}, {5, 375, 0}, {6, 150, 0}, {7, 600, 0},  // reset
                                 {0, 600, 0}, {1, 120, 0}, {6, 375, 0}, {7, 375, 1500},
                                 {0, 330, 0}, {1, 330, 0}, {6, 150, 0}, {7, 600, 1500},
                                 {0, 600, 0}, {1, 120, 0}, {6, 375, 0}, {7, 375, 1500},
                                 {0, 330, 0}, {1, 330, 0}, {6, 150, 0}, {7, 600, 0},
                                 {-1, -1, -1}};
 
int (*sqn_active)[3] = sqn_reset;
int sqn_idx = 0;
unsigned long sqn_next = 0;
bool do_sqn = true;

unsigned long heartbeatNext = 0;
bool heartbeatState = false;

void setup() {
  Serial.begin(9600);
  Serial.println("16 channel Servo test!");

  pwm.begin();  
  pwm.setPWMFreq(60);  // Analog servos run at ~60 Hz updates

  pinMode(5, OUTPUT);
  digitalWrite(5, LOW);
  
  pinMode(13, OUTPUT);
  

  
  yield();
}

// you can use this function if you'd like to set the pulse length in seconds
// e.g. setServoPulse(0, 0.001) is a ~1 millisecond pulse width. its not precise!
void setServoPulse(uint8_t n, double pulse) {
  double pulselength;
  
  pulselength = 1000000;   // 1,000,000 us per second
  pulselength /= 60;       // 60 Hz
  Serial.print(pulselength); Serial.println(" us per period"); 
  pulselength /= 4096;     // 12 bits of resolution
  Serial.print(pulselength); Serial.println(" us per bit"); 
  pulse *= 1000;
  pulse /= pulselength;
  Serial.println(pulse);
  pwm.setPWM(n, 0, pulse);
}



void loop() {
  // read commands
  readCommand();   
  
  
  // heartbeat
  unsigned long now = millis();

  if (now > heartbeatNext) {
    heartbeatState = !heartbeatState;
    digitalWrite(13, heartbeatState ? HIGH : LOW);
    heartbeatNext = now + 500;
  }
    
  while (do_sqn && now > sqn_next) {
    now = millis();
    setServoPulse(sqn_active[sqn_idx][0], sqn_active[sqn_idx][1]);
    sqn_next = now + sqn_active[sqn_idx][2];
    sqn_idx++;
 
    // check for stop sequence
    if (sqn_active[sqn_idx][0] == -1) {
      do_sqn = false;
    }
    
    yield();
  }
  
  // set servo positions
  for (uint8_t s=0; s<16; s++) {
    pwm.setPWM(s, 0, servo_state[s][0]);
  }
}

int readline(int readch, char *buffer, int len) {
  static int pos = 0;
  int rpos;

  if (readch > 0) {
    switch (readch) {
      case '\n': // Ignore new-lines
        break;
      case '\r': // Return on CR
        rpos = pos;
        pos = 0;  // Reset position index ready for next time
        return rpos;
      default:
        if (pos < len-1) {
          buffer[pos++] = readch;
          buffer[pos] = 0;
        }
    }
  }
  
  // No end of line has been found, so return -1.
  return -1;
}

int readCommand() {
  static char buffer[80];
  
  // process commands
  if (Serial.available() > 0) {
    if (readline(Serial.read(), buffer, 80) > 0) {
      Serial.print("You entered: >");
      Serial.print(buffer);
      Serial.println("<");
      
      processCommand(buffer);
    } 
  }
}

int processCommand(String command) {
  int servo_id;
  int pulse_length;
  
  // dump stats
  if (command == "d" || command == "dump") {
    Serial.println("--------------------------------");
   
    char out[80];
    long angle;
    
    for (uint8_t s=0; s<16; s++) {
      angle = map((long)servo_state[s][0], (long)servo_state[s][1], (long)servo_state[s][2], (long)servo_state[s][3], (long)servo_state[s][4]);
      angle = (servo_state[servo_id][3] == 180) ? 180 - angle : angle;
      snprintf(out, 80, "Servo %02d: %03d (%03d) -- %03d (%03d) -- %03d (%03d)", s, servo_state[s][1], servo_state[s][3], servo_state[s][0], (uint16_t)angle, servo_state[s][2], servo_state[s][4]);
      Serial.println(out);
    }
    
    Serial.println("--------------------------------");
  }
  // save
  else if (command == "w" || command == "write") {
    Serial.print("{");
    char out[80];
    for (uint8_t s=0; s<16; s++) {
      snprintf(out, 80, "{%03d,%03d,%03d,%03d,%03d}", servo_state[s][1], servo_state[s][0], servo_state[s][2], servo_state[s][3], servo_state[s][4]);
      Serial.print(out);
      Serial.println((s<15) ? "," : "}");
    }
    
    Serial.println("--------------------------------");
  }
  // set character "s#,a#"
  else if (sscanf(command.c_str(), "s%d,a%d", &servo_id, &pulse_length) == 2) {
    if (setServoAngle(servo_id, pulse_length) == 0) {
      Serial.print(command); Serial.println("... OK");
    }
  }
  // set character "s#,p#"
  else if (sscanf(command.c_str(), "s%d,p%d", &servo_id, &pulse_length) == 2) {
    if (setServoPulse(servo_id, pulse_length) == 0) {
      Serial.print(command); Serial.println("... OK");
    }
  }
  // set character "s#,min#"
  else if (sscanf(command.c_str(), "s%d,min%d", &servo_id, &pulse_length) == 2) {
    if (setServoPulse(servo_id, pulse_length) == 0) {
      Serial.print(command); Serial.println("... OK");
    }
  }
  else if (command == "reset") {
    sqn_active = sqn_reset;
    sqn_idx = 0;
    do_sqn = true;
  }
  else if (command == "standup") {
    sqn_active = sqn_standup;
    sqn_idx = 0;
    do_sqn = true;
  }
  else if (command == "starjump") {
    sqn_active = sqn_starjump;
    sqn_idx = 0;
    do_sqn = true;
  }
  else if (command == "walk") {
    sqn_active = sqn_walk;
    sqn_idx = 0;
    do_sqn = true;
  }
  else if (command == "waveleft") {
    sqn_active = sqn_waveleft;
    sqn_idx = 0;
    do_sqn = true;
  }
  else if (command == "waveright") {
    sqn_active = sqn_waveright;
    sqn_idx = 0;
    do_sqn = true;
  }
  // unknown
  else {
    Serial.print("Unknown command: "); Serial.println(command); 
  }  
}

int setServoPulse(int servo_id, int pulse_length) {
    if (servo_id < 0 || servo_id >= 16) {
       Serial.println("Invalid servo ID");
       return -1;
    }
    
    if (pulse_length < servo_state[servo_id][1] || pulse_length > servo_state[servo_id][2]) {
      Serial.println("Invalid Pulse Length"); 
      return -1;
    }
    
    servo_state[servo_id][0] = pulse_length;
    
    return 0;
}

int setServoAngle(int servo_id, int angle) {
    if (servo_id < 0 || servo_id >= 16) {
       Serial.println("Invalid servo ID");
       return -1;
    }
    
    /*
    if (angle < servo_state[servo_id][1] || pulse_length > servo_state[servo_id][2]) {
      Serial.println("Invalid Pulse Length"); 
      return -1;
    }
    */
    
    long pulse_length;
    
    if ((servo_state[3] - servo_state[4]) > 0) {
      angle = (int)servo_state[3] - angle;
      pulse_length = map((long)angle, 0, 180, (long)servo_state[1], (long)servo_state[2]);
    }
    else {
      pulse_length = map((long)angle, 0, 180, (long)servo_state[1], (long)servo_state[2]);    
    }
     
    servo_state[servo_id][0] = (uint16_t)pulse_length;
    
    return 0;
}

