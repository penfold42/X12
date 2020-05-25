/*
X12 library example
Use a serial console to control an X12.017 style micro stepper
*/

#include <X12.h>

// X12 chip pin wiring
#define pinStep   5   // step pin
#define pinDir    4   // direction pin
#define pinReset  3   // reset pin

// how many steps to full scale?
#define STEPS (315*12)

// how many letter shortcuts? 
#define LETTERS ('i'-'a')

void setup() {
  Serial.begin(115200);
  // wait for 32u4 leonardo serial port to wake up
  while (!Serial);  

  Serial.println(); Serial.println();
  Serial.println F("X12 micro stepper serial example");
  X12begin(STEPS, pinStep, pinDir, pinReset);

  Calibrate();
  printHelp();
}

void Calibrate() {
  Serial.print F("Calibrating...");
  Serial.print F(" Full...");  X12full();
  Serial.print F(" Zero...");  X12zero();
  Serial.print F(" Full...");  X12full();
  Serial.print F(" Zero...");  X12setPosition(0);
  Serial.println();
}

void printHelp() {
  char temp[80];
  Serial.println();
  Serial.print F("Directly specify a step count from 0 to "); Serial.println(STEPS);
  Serial.println F("Or use letters to set needle to presets:");
  for (int i=0; i <= LETTERS; i++) {
    int val = (long)STEPS * i / LETTERS;
    snprintf (temp, 79, "%3c: %5d    ", 'a'+i, val);
    Serial.print(temp);
    if ( (i%3 == 2) || (i == LETTERS) )
      Serial.println();  
  }
  Serial.println F("Other options:");
  Serial.println F("  H: this help");
  Serial.println F("  C: Calibrate        R: Random");
  Serial.println F("  -: CCW 100 steps    +: CW 100 steps");
  Serial.println F("  [: CCW 303 steps    ]: CW 303 steps");
}

// set stepper position and print message
void setPositionPrint(int pos) {
  X12setPosition(pos);
  Serial.print F("Moving to "); Serial.println(pos);
}

int target=0;     // Used to accumulate the target value from number chars
int numchar=0;    // how many number chars have been typed?



void loop() {
  // spin waiting for character
  while (!Serial.available());
  char c = Serial.read();
  // letters for quick jumps
  if (( c >= 'a') && (c <= 'a'+LETTERS)) {
    int pos = ((long)STEPS * (c - 'a') / LETTERS);
    setPositionPrint(pos);
    target = numchar = 0;
  }
  // digits for specify step count
  else if ((c>='0') && (c<='9')) {
    target *= 10;
    target += c-'0';
    numchar++;
  }

  else if ((c=='\r') || (c=='\n')) {
    if (numchar) {
      setPositionPrint(target);
    }
    target = numchar = 0;

  } else { // singe character command
    target = numchar = 0;
    int i;
    switch (c) {
      case 'R':
        Serial.println F("Random mode, any key to exit");
        while (1) {
          int pos = (random(0,STEPS));
          X12setPosition(pos);
          delay(random(50,200));
          if (Serial.available()) {
            char c = Serial.read();
            if ((c!='\r') && (c!='\n'))
              break; // a character not new line, so exit random
          }
        }
        break;
  
      case 'H':
        printHelp();
        break;
  
      case 'C':
        Calibrate();
        break;
  
      case '+':
        i = X12getPosition()+100;
        setPositionPrint(i);
        break;
        
      case '_':
      case '-':
        i = X12getPosition()-100;
        if (i<0) i=0;
        setPositionPrint(i);
        break;

      case ']':
        i = X12getPosition()+303;
        setPositionPrint(i);
        break;

      case '[':
        i = X12getPosition()-303;
        if (i<0) i=0;
        setPositionPrint(i);
        break;

      default:
        break;
    } // switch(c)
  }
}
