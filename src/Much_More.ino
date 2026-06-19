#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include <SD.h>
#include <SerialFlash.h>
#include <MIDI.h>
#include <USBHost_t36.h>
#include "MidiCC.h"
#include "Constants.h"
#include "Parameters.h"
#include "PatchMgr.h"
#include "Button.h"
#include "HWControls.h"
#include "EepromMgr.h"
#include "Settings.h"
#include <map>  // Include the map library

std::map<int, int> voiceAssignment;

#define PARAMETER 0      //The main page for displaying the current patch and control (parameter) changes
#define RECALL 1         //Patches list
#define SAVE 2           //Save patch page
#define REINITIALISE 3   // Reinitialise message
#define PATCH 4          // Show current patch bypassing PARAMETER
#define PATCHNAMING 5    // Patch naming page
#define DELETE 6         //Delete patch page
#define DELETEMSG 7      //Delete patch message page
#define SETTINGS 8       //Settings page
#define SETTINGSVALUE 9  //Settings page
#define PERFORMANCE_RECALL 10
#define PERFORMANCE_SAVE 11
#define PERFORMANCE_EDIT 12
#define PERFORMANCE_NAMING 13
#define PERFORMANCE_DELETE 14
#define PERFORMANCE_DELETEMSG 15

unsigned int state = PARAMETER;

enum PlayMode {
  WHOLE = 0,
  DUAL = 1,
  SPLIT = 2
};

struct Performance {
  int performanceNo;
  int upperPatchNo;
  int lowerPatchNo;
  String name;
  PlayMode mode;  // ← Back to enum type!
};

#include "ST7735Display.h"

boolean cardStatus = false;

struct VoiceAndNote {
  int note;
  int velocity;
  unsigned long timeOn;
  bool sustained;  // Sustain flag
  bool keyDown;
  double noteFreq;  // Note frequency
  int position;
  bool noteOn;
};

struct VoiceAndNote voices[NO_OF_VOICES] = {
  { -1, -1, 0, false, false, 0, -1, false },
  { -1, -1, 0, false, false, 0, -1, false },
  { -1, -1, 0, false, false, 0, -1, false },
  { -1, -1, 0, false, false, 0, -1, false },
  { -1, -1, 0, false, false, 0, -1, false },
  { -1, -1, 0, false, false, 0, -1, false },
  { -1, -1, 0, false, false, 0, -1, false },
  { -1, -1, 0, false, false, 0, -1, false },
  { -1, -1, 0, false, false, 0, -1, false },
  { -1, -1, 0, false, false, 0, -1, false },
  { -1, -1, 0, false, false, 0, -1, false },
  { -1, -1, 0, false, false, 0, -1, false }
};

// Tracks exactly which note each voice currently plays
int voiceToNoteLower[6] = { -1, -1, -1, -1, -1, -1 };
int voiceToNoteUpper[6] = { -1, -1, -1, -1, -1, -1 };


boolean voiceOn[NO_OF_VOICES] = { false, false, false, false, false, false, false, false, false, false, false, false };
int prevNote = 0;  //Initialised to middle value
bool notes[128] = { 0 }, initial_loop = 1;
int8_t noteOrder[40] = { 0 }, orderIndx = { 0 };

bool notesWhole[128], notesLower[128], notesUpper[128];
byte noteOrderWhole[40], noteOrderLower[40], noteOrderUpper[40];
int orderIndxWhole = 0, orderIndxLower = 0, orderIndxUpper = 0;

int voiceAssignmentLower[128];
int voiceAssignmentUpper[128];

CircularBuffer<Performance, PERFORMANCES_LIMIT> performances;
Performance currentPerformance;


//USB HOST MIDI Class Compliant
USBHost myusb;
USBHub hub1(myusb);
USBHub hub2(myusb);
MIDIDevice midi1(myusb);


//MIDI 5 Pin DIN
MIDI_CREATE_INSTANCE(HardwareSerial, Serial1, MIDI);   // main MIDI in and out
MIDI_CREATE_INSTANCE(HardwareSerial, Serial6, MIDI6);  // MIDI out to display (not connected)
MIDI_CREATE_INSTANCE(HardwareSerial, Serial7, MIDI7);  // MIDI out to lower board
MIDI_CREATE_INSTANCE(HardwareSerial, Serial8, MIDI8);  // MIDI out to upper board

int count = 0;  //For MIDI Clk Sync
int DelayForSH3 = 50;
int patchNo = 0;
int patchNoU = 0;
int patchNoL = 0;
int voiceToReturn = -1;                 //Initialise
unsigned long earliestTime = millis();  //For voice allocation - initialise to now
unsigned long buttonDebounce = 0;

void pollAllMCPs();

void initRotaryEncoders();

void initButtons();

int getEncoderSpeed(int id);

void setup() {

  chordHoldActive = false;
  chordHoldWaitingForNotes = false;
  chordHoldCount = 0;

  SPI.begin();
  Wire.begin();            // Join the I2C bus as Master
  Wire.setClock(400000);   // Set I2C speed to 400 kHz
  Wire1.begin();           // Join the I2C bus as Master
  Wire1.setClock(400000);  // Set I2C speed to 400 kHz

  mcp1.begin(0, Wire);
  delay(10);
  mcp2.begin(1, Wire);
  delay(10);
  mcp3.begin(2, Wire);
  delay(10);
  mcp4.begin(3, Wire);
  delay(10);
  mcp5.begin(4, Wire);
  delay(10);
  mcp6.begin(5, Wire);
  delay(10);
  mcp7.begin(6, Wire);
  delay(10);
  mcp8.begin(7, Wire);
  delay(10);
  mcp9.begin(0, Wire1);
  delay(10);
  mcp10.begin(1, Wire1);
  delay(10);
  mcp11.begin(2, Wire1);
  delay(10);
  mcp12.begin(3, Wire1);
  delay(10);
  mcp13.begin(4, Wire1);
  delay(10);
  mcp14.begin(5, Wire1);

  //groupEncoders();
  initRotaryEncoders();
  initButtons();

  setupDisplay();
  setUpSettings();
  setupHardware();


  for (int i = 0; i < 128; i++) {
    voiceAssignmentLower[i] = -1;
    voiceAssignmentUpper[i] = -1;
  }

  for (int i = 0; i < 6; i++) {
    voiceToNoteLower[i] = -1;
    voiceToNoteUpper[i] = -1;
  }


  cardStatus = SD.begin(BUILTIN_SDCARD);
  if (cardStatus) {
    Serial.println("SD card is connected");
    loadPatches();
    if (patches.size() == 0) {
      //save an initialised patch to SD card
      savePatch("1", INITPATCH);
      loadPatches();
    }
    loadPerformances();
    if (performances.size() == 0 && patches.size() > 0) {
      Performance defaultPerf = {
        1,
        patches.first().patchNo,
        patches.first().patchNo,
        "Default"
      };
      performances.push(defaultPerf);
      savePerformance("perf001", defaultPerf);
      loadPerformances();  // reload to ensure it's in the buffer
    }
  } else {
    Serial.println("SD card is not connected or unusable");
    reinitialiseToPanel();
    showPatchPage("No SD", "conn'd / usable", "", "");
  }

  //Read MIDI Channel from EEPROM
  midiChannel = getMIDIChannel();
  Serial.println("MIDI Ch:" + String(midiChannel) + " (0 is Omni On)");

  //USB HOST MIDI Class Compliant
  delay(400);  //Wait to turn on USB Host
  myusb.begin();
  midi1.setHandleControlChange(editControlChange);
  midi1.setHandleNoteOff(myNoteOff);
  midi1.setHandleNoteOn(myNoteOn);
  midi1.setHandlePitchChange(DinHandlePitchBend);
  midi1.setHandleAfterTouch(myAfterTouch);
  Serial.println("USB HOST MIDI Class Compliant Listening");

  //USB Client MIDI
  usbMIDI.setHandleControlChange(editControlChange);
  usbMIDI.setHandleProgramChange(myProgramChange);
  usbMIDI.setHandleAfterTouchChannel(myAfterTouch);
  usbMIDI.setHandlePitchChange(DinHandlePitchBend);
  usbMIDI.setHandleNoteOn(myNoteOn);
  usbMIDI.setHandleNoteOff(myNoteOff);
  Serial.println("USB Client MIDI Listening");

  //MIDI 5 Pin DIN
  MIDI.begin();
  MIDI.setHandleControlChange(editControlChange);
  MIDI.setHandleProgramChange(myProgramChange);
  MIDI.setHandleAfterTouchChannel(myAfterTouch);
  MIDI.setHandlePitchBend(DinHandlePitchBend);
  MIDI.setHandleNoteOn(myNoteOn);
  MIDI.setHandleNoteOff(myNoteOff);
  MIDI.turnThruOn(midi::Thru::Mode::Off);
  Serial.println("MIDI In DIN Listening");

  MIDI8.begin();
  MIDI8.turnThruOn(midi::Thru::Mode::Off);

  MIDI7.begin();
  MIDI7.turnThruOn(midi::Thru::Mode::Off);

  MIDI6.begin();
  MIDI6.turnThruOn(midi::Thru::Mode::Off);

  //Read Aftertouch from EEPROM, this can be set individually by each patch.
  upperData[P_AfterTouchDest] = getAfterTouchU();
  lowerData[P_AfterTouchDest] = getAfterTouchL();

  splitPoint = getSplitPoint();
  splitPoint = (splitPoint + 36);

  splitTrans = getSplitTrans();
  setTranspose(splitTrans);

  //Read Encoder Direction from EEPROM
  encCW = getEncoderDir();

  // Read the encoders accelerate
  accelerate = getEncoderAccelerate();

  // read in halfstep setting
  halfStepMode = getEncoderStepMode();

  //setupDisplay();
  delay(500);


  for (int i = 0; i < 6; i++) {
    int noteon = 60;
    MIDI7.sendNoteOn(noteon, 64, i +1);
    delay(1);
    MIDI7.sendNoteOff(noteon, 64, i +1);
    delay(5);
    MIDI8.sendNoteOn(noteon, 64, i +1);
    delay(1);
    MIDI8.sendNoteOff(noteon, 64, i +1);
  }
  delay(200);

  patchNoU = 1;
  patchNoL = 1;
  upperSW = false;
  lowerSW = true;
  updatekeyboardMode(0);
  updateplayMode(0);
  recallPatch(patchNoL);  //Load first patch
  refreshScreen();

  // midiCCOut79(CC_FV1_BANK_0, 127); // eeprom 1
  // midiCCOut79(CC_FV1_BANK_1, 127); // eeprom 2 
  // midiCCOut79(CC_FV1_BANK_2, 127); // eeprom 3 
  // midiCCOut79(CC_FV1_EFFECT_0, 0);
  // midiCCOut79(CC_FV1_EFFECT_1, 0);
  // midiCCOut79(CC_FV1_EFFECT_2, 0);
  // midiCCOut79(CC_FV1_INTERNAL, 0); // internal-external

}

void pollAllMCPs() {

  for (int j = 0; j < numMCPs; j++) {
    uint16_t gpioAB = allMCPs[j]->readGPIOAB();
    for (int i = 0; i < numEncoders; i++) {
      if (rotaryEncoders[i].getMCP() == allMCPs[j])
        rotaryEncoders[i].feedInput(gpioAB);
    }

    for (auto &button : allButtons) {
      if (button->getMcp() == allMCPs[j]) {
        button->feedInput(gpioAB);
      }
    }
  }
}

void initRotaryEncoders() {
  for (auto &rotaryEncoder : rotaryEncoders) {
    rotaryEncoder.init();
  }
}

void initButtons() {
  for (auto &button : allButtons) {
    button->begin();
  }
}

int getEncoderSpeed(int id) {
  if (id < 1 || id > numEncoders) return 1;

  unsigned long now = millis();
  unsigned long dt = now - lastTransition[id];

  // Linear acceleration mapping
  float minMult = 1.0f;
  float maxMult = 10.0f;
  float minDt = 30.0f;   // Fastest spins
  float maxDt = 350.0f;  // Slowest for any acceleration

  float mult;
  if (dt < minDt)
    mult = maxMult;
  else if (dt > maxDt)
    mult = minMult;
  else
    mult = maxMult - (maxMult - minMult) * ((dt - minDt) / (maxDt - minDt));

  // Optional: smooth multiplier for less jumpy response
  float alpha = 0.5f;  // 0.0 = no smoothing, 1.0 = max smoothing
  lastSpeed[id] = alpha * mult + (1.0f - alpha) * lastSpeed[id];

  lastTransition[id] = now;
  return (int)(lastSpeed[id] + 0.5f);
}

void RotaryEncoderChanged(bool clockwise, int id) {

  if (!accelerate) {
    speed = 1;
  } else {
    speed = getEncoderSpeed(id);
  }

  if (!clockwise) {
    speed = -speed;
  }

  // Serial.print("Encode ID ");
  // Serial.println(id);

  switch (id) {

    case 1:
      lowerData[P_arpRate] = (lowerData[P_arpRate] + speed);
      lowerData[P_arpRate] = constrain(lowerData[P_arpRate], 0, 127);
      arpRatestr = LFOTEMPO[lowerData[P_arpRate]];
      updatearpRate(1);
      break;

    case 2:
      if (upperSW) {
        upperData[P_glideTime] = (upperData[P_glideTime] + speed);
        upperData[P_glideTime] = constrain(upperData[P_glideTime], 0, 127);
        glideTimestr = LINEAR[upperData[P_glideTime]];
      } else {
        lowerData[P_glideTime] = (lowerData[P_glideTime] + speed);
        lowerData[P_glideTime] = constrain(lowerData[P_glideTime], 0, 127);
        glideTimestr = LINEAR[lowerData[P_glideTime]];
        if (wholemode) {
          upperData[P_glideTime] = lowerData[P_glideTime];
        }
      }

      updateglideTime(1);
      break;

    case 3:
      if (upperSW) {
        upperData[P_osc1PWM] = (upperData[P_osc1PWM] + speed);
        upperData[P_osc1PWM] = constrain(upperData[P_osc1PWM], 0, 127);
        osc1PWMstr = upperData[P_osc1PWM];
      } else {
        lowerData[P_osc1PWM] = (lowerData[P_osc1PWM] + speed);
        lowerData[P_osc1PWM] = constrain(lowerData[P_osc1PWM], 0, 127);
        osc1PWMstr = lowerData[P_osc1PWM];
        if (wholemode) {
          upperData[P_osc1PWM] = lowerData[P_osc1PWM];
        }
      }

      updateosc1PWM(1);
      break;

    case 4:
      if (upperSW) {
        upperData[P_osc1envPWM] = (upperData[P_osc1envPWM] + speed);
        upperData[P_osc1envPWM] = constrain(upperData[P_osc1envPWM], 0, 127);
        osc1PWMstr = upperData[P_osc1envPWM];
      } else {
        lowerData[P_osc1envPWM] = (lowerData[P_osc1envPWM] + speed);
        lowerData[P_osc1envPWM] = constrain(lowerData[P_osc1envPWM], 0, 127);
        osc1PWMstr = lowerData[P_osc1envPWM];
        if (wholemode) {
          upperData[P_osc1envPWM] = lowerData[P_osc1envPWM];
        }
      }

      updateosc1envPWM(1);
      break;

    case 5:
      if (upperSW) {
        upperData[P_osc1sawDetune] = (upperData[P_osc1sawDetune] + speed);
        upperData[P_osc1sawDetune] = constrain(upperData[P_osc1sawDetune], 0, 127);
        osc1sawDetunestr = upperData[P_osc1sawDetune];
      } else {
        lowerData[P_osc1sawDetune] = (lowerData[P_osc1sawDetune] + speed);
        lowerData[P_osc1sawDetune] = constrain(lowerData[P_osc1sawDetune], 0, 127);
        osc1sawDetunestr = lowerData[P_osc1sawDetune];
        if (wholemode) {
          upperData[P_osc1sawDetune] = lowerData[P_osc1sawDetune];
        }
      }

      updateosc1sawDetune(1);
      break;

    case 6:
      if (upperSW) {
        upperData[P_osc1PW] = (upperData[P_osc1PW] + speed);
        upperData[P_osc1PW] = constrain(upperData[P_osc1PW], 0, 127);
        osc1PWstr = PULSEWIDTH[upperData[P_osc1PW]];
      } else {
        lowerData[P_osc1PW] = (lowerData[P_osc1PW] + speed);
        lowerData[P_osc1PW] = constrain(lowerData[P_osc1PW], 0, 127);
        osc1PWstr = PULSEWIDTH[lowerData[P_osc1PW]];
        if (wholemode) {
          upperData[P_osc1PW] = lowerData[P_osc1PW];
        }
      }

      updateosc1PW(1);
      break;

    case 7:
      if (upperSW) {
        upperData[P_osc1SawLevel] = (upperData[P_osc1SawLevel] + speed);
        upperData[P_osc1SawLevel] = constrain(upperData[P_osc1SawLevel], 0, 127);
        osc1SawLevelstr = upperData[P_osc1SawLevel];
      } else {
        lowerData[P_osc1SawLevel] = (lowerData[P_osc1SawLevel] + speed);
        lowerData[P_osc1SawLevel] = constrain(lowerData[P_osc1SawLevel], 0, 127);
        osc1SawLevelstr = lowerData[P_osc1SawLevel];
        if (wholemode) {
          upperData[P_osc1SawLevel] = lowerData[P_osc1SawLevel];
        }
      }

      updateOsc1SawLevel(1);
      break;

    case 8:
      if (upperSW) {
        upperData[P_osc1PulseLevel] = (upperData[P_osc1PulseLevel] + speed);
        upperData[P_osc1PulseLevel] = constrain(upperData[P_osc1PulseLevel], 0, 127);
        osc1PulseLevelstr = upperData[P_osc1PulseLevel];
      } else {
        lowerData[P_osc1PulseLevel] = (lowerData[P_osc1PulseLevel] + speed);
        lowerData[P_osc1PulseLevel] = constrain(lowerData[P_osc1PulseLevel], 0, 127);
        osc1PulseLevelstr = lowerData[P_osc1PulseLevel];
        if (wholemode) {
          upperData[P_osc1PulseLevel] = lowerData[P_osc1PulseLevel];
        }
      }

      updateOsc1PulseLevel(1);
      break;

    case 9:
      if (upperSW) {
        upperData[P_osc1SubLevel] = (upperData[P_osc1SubLevel] + speed);
        upperData[P_osc1SubLevel] = constrain(upperData[P_osc1SubLevel], 0, 127);
        osc1SubLevelstr = upperData[P_osc1SubLevel];
      } else {
        lowerData[P_osc1SubLevel] = (lowerData[P_osc1SubLevel] + speed);
        lowerData[P_osc1SubLevel] = constrain(lowerData[P_osc1SubLevel], 0, 127);
        osc1SubLevelstr = lowerData[P_osc1SubLevel];
        if (wholemode) {
          upperData[P_osc1SubLevel] = lowerData[P_osc1SubLevel];
        }
      }

      updateOsc1SubLevel(1);
      break;

    case 10:
      if (upperSW) {
        upperData[P_osc1sawCount] = (upperData[P_osc1sawCount] + speed);
        upperData[P_osc1sawCount] = constrain(upperData[P_osc1sawCount], 0, 127);
        osc1sawCountstr = map(upperData[P_osc1sawCount], 0, 127, 1, 5);
      } else {
        lowerData[P_osc1sawCount] = (lowerData[P_osc1sawCount] + speed);
        lowerData[P_osc1sawCount] = constrain(lowerData[P_osc1sawCount], 0, 127);
        osc1sawCountstr = map(lowerData[P_osc1sawCount], 0, 127, 1, 5);
        ;
        if (wholemode) {
          upperData[P_osc1sawCount] = lowerData[P_osc1sawCount];
        }
      }

      updateosc1sawCount(1);
      break;

    case 11:
      if (upperSW) {
        upperData[P_osc2PW] = (upperData[P_osc2PW] + speed);
        upperData[P_osc2PW] = constrain(upperData[P_osc2PW], 0, 127);
        osc2PWstr = PULSEWIDTH[upperData[P_osc2PW]];
      } else {
        lowerData[P_osc2PW] = (lowerData[P_osc2PW] + speed);
        lowerData[P_osc2PW] = constrain(lowerData[P_osc2PW], 0, 127);
        osc2PWstr = PULSEWIDTH[lowerData[P_osc2PW]];
        if (wholemode) {
          upperData[P_osc2PW] = lowerData[P_osc2PW];
        }
      }

      updateosc2PW(1);
      break;

    case 12:
      if (upperSW) {
        upperData[P_osc2SawLevel] = (upperData[P_osc2SawLevel] + speed);
        upperData[P_osc2SawLevel] = constrain(upperData[P_osc2SawLevel], 0, 127);
        osc2SawLevelstr = upperData[P_osc2SawLevel];
      } else {
        lowerData[P_osc2SawLevel] = (lowerData[P_osc2SawLevel] + speed);
        lowerData[P_osc2SawLevel] = constrain(lowerData[P_osc2SawLevel], 0, 127);
        osc2SawLevelstr = lowerData[P_osc2SawLevel];
        if (wholemode) {
          upperData[P_osc2SawLevel] = lowerData[P_osc2SawLevel];
        }
      }

      updateOsc2SawLevel(1);
      break;

    case 13:
      if (upperSW) {
        upperData[P_osc2PulseLevel] = (upperData[P_osc2PulseLevel] + speed);
        upperData[P_osc2PulseLevel] = constrain(upperData[P_osc2PulseLevel], 0, 127);
        osc2PulseLevelstr = upperData[P_osc2PulseLevel];
      } else {
        lowerData[P_osc2PulseLevel] = (lowerData[P_osc2PulseLevel] + speed);
        lowerData[P_osc2PulseLevel] = constrain(lowerData[P_osc2PulseLevel], 0, 127);
        osc2PulseLevelstr = lowerData[P_osc2PulseLevel];
        if (wholemode) {
          upperData[P_osc2PulseLevel] = lowerData[P_osc2PulseLevel];
        }
      }

      updateOsc2PulseLevel(1);
      break;

    case 14:
      if (!clockwise) {
        speed = -1;
      } else {
        speed = +1;
      }
      if (upperSW) {
        upperData[P_osc2Interval] = (upperData[P_osc2Interval] + speed);
        upperData[P_osc2Interval] = constrain(upperData[P_osc2Interval], 0, 12);
        osc2Intervalstr = upperData[P_osc2Interval];
      } else {
        lowerData[P_osc2Interval] = (lowerData[P_osc2Interval] + speed);
        lowerData[P_osc2Interval] = constrain(lowerData[P_osc2Interval], 0, 12);
        osc2Intervalstr = lowerData[P_osc2Interval];
        if (wholemode) {
          upperData[P_osc2Interval] = lowerData[P_osc2Interval];
        }
      }

      updateosc2Interval(1);
      break;

    case 15:
      if (upperSW) {
        upperData[P_osc2PWM] = (upperData[P_osc2PWM] + speed);
        upperData[P_osc2PWM] = constrain(upperData[P_osc2PWM], 0, 127);
        osc2PWMstr = upperData[P_osc2PWM];
      } else {
        lowerData[P_osc2PWM] = (lowerData[P_osc2PWM] + speed);
        lowerData[P_osc2PWM] = constrain(lowerData[P_osc2PWM], 0, 127);
        osc2PWMstr = lowerData[P_osc2PWM];
        if (wholemode) {
          upperData[P_osc2PWM] = lowerData[P_osc2PWM];
        }
      }

      updateosc2PWM(1);
      break;

    case 16:
      if (upperSW) {
        upperData[P_osc2envPWM] = (upperData[P_osc2envPWM] + speed);
        upperData[P_osc2envPWM] = constrain(upperData[P_osc2envPWM], 0, 127);
        osc2PWMstr = upperData[P_osc2envPWM];
      } else {
        lowerData[P_osc2envPWM] = (lowerData[P_osc2envPWM] + speed);
        lowerData[P_osc2envPWM] = constrain(lowerData[P_osc2envPWM], 0, 127);
        osc2PWMstr = lowerData[P_osc2envPWM];
        if (wholemode) {
          upperData[P_osc2envPWM] = lowerData[P_osc2envPWM];
        }
      }

      updateosc2envPWM(1);
      break;

    case 17:
      if (upperSW) {
        upperData[P_osc2Detune] = (upperData[P_osc2Detune] + speed);
        upperData[P_osc2Detune] = constrain(upperData[P_osc2Detune], 0, 127);
        osc2Detunestr = upperData[P_osc2Detune];
      } else {
        lowerData[P_osc2Detune] = (lowerData[P_osc2Detune] + speed);
        lowerData[P_osc2Detune] = constrain(lowerData[P_osc2Detune], 0, 127);
        osc2Detunestr = lowerData[P_osc2Detune];
        if (wholemode) {
          upperData[P_osc2Detune] = lowerData[P_osc2Detune];
        }
      }

      updateosc2Detune(1);
      break;

    case 18:
      if (upperSW) {
        upperData[P_filterLevel1] = (upperData[P_filterLevel1] + speed);
        upperData[P_filterLevel1] = constrain(upperData[P_filterLevel1], 0, 127);
        filterLevel1str = upperData[P_filterLevel1];
      } else {
        lowerData[P_filterLevel1] = (lowerData[P_filterLevel1] + speed);
        lowerData[P_filterLevel1] = constrain(lowerData[P_filterLevel1], 0, 127);
        filterLevel1str = lowerData[P_filterLevel1];
        if (wholemode) {
          upperData[P_filterLevel1] = lowerData[P_filterLevel1];
        }
      }

      updatefilterLevel1(1);
      break;

    case 19:
      if (upperSW) {
        upperData[P_filterCutoff] = (upperData[P_filterCutoff] + speed);
        upperData[P_filterCutoff] = constrain(upperData[P_filterCutoff], 0, 127);
        filterCutoffstr = FILTERCUTOFF[upperData[P_filterCutoff]];
      } else {
        lowerData[P_filterCutoff] = (lowerData[P_filterCutoff] + speed);
        lowerData[P_filterCutoff] = constrain(lowerData[P_filterCutoff], 0, 127);
        filterCutoffstr = FILTERCUTOFF[lowerData[P_filterCutoff]];
        if (wholemode) {
          upperData[P_filterCutoff] = lowerData[P_filterCutoff];
        }
      }

      updateFilterCutoff(1);
      break;

    case 20:
      if (upperSW) {
        upperData[P_filterRes] = (upperData[P_filterRes] + speed);
        upperData[P_filterRes] = constrain(upperData[P_filterRes], 0, 127);
        filterResstr = upperData[P_filterRes];
      } else {
        lowerData[P_filterRes] = (lowerData[P_filterRes] + speed);
        lowerData[P_filterRes] = constrain(lowerData[P_filterRes], 0, 127);
        filterResstr = lowerData[P_filterRes];
        if (wholemode) {
          upperData[P_filterRes] = lowerData[P_filterRes];
        }
      }

      updatefilterRes(1);
      break;

    case 21:
      if (upperSW) {
        upperData[P_filterLevel2] = (upperData[P_filterLevel2] + speed);
        upperData[P_filterLevel2] = constrain(upperData[P_filterLevel2], 0, 127);
        filterLevel2str = upperData[P_filterLevel2];
      } else {
        lowerData[P_filterLevel2] = (lowerData[P_filterLevel2] + speed);
        lowerData[P_filterLevel2] = constrain(lowerData[P_filterLevel2], 0, 127);
        filterLevel2str = lowerData[P_filterLevel2];
        if (wholemode) {
          upperData[P_filterLevel2] = lowerData[P_filterLevel2];
        }
      }

      updatefilterLevel2(1);
      break;

    case 22:
      if (upperSW) {
        upperData[P_keytrack] = (upperData[P_keytrack] + speed);
        upperData[P_keytrack] = constrain(upperData[P_keytrack], 0, 127);
        keytrackstr = upperData[P_keytrack];
      } else {
        lowerData[P_keytrack] = (lowerData[P_keytrack] + speed);
        lowerData[P_keytrack] = constrain(lowerData[P_keytrack], 0, 127);
        keytrackstr = lowerData[P_keytrack];
        if (wholemode) {
          upperData[P_keytrack] = lowerData[P_keytrack];
        }
      }

      updatekeytrack(1);
      break;

    case 23:
      if (upperSW) {
        upperData[P_filterEGlevel] = (upperData[P_filterEGlevel] + speed);
        upperData[P_filterEGlevel] = constrain(upperData[P_filterEGlevel], 0, 127);
        filterEGlevelstr = upperData[P_filterEGlevel];
      } else {
        lowerData[P_filterEGlevel] = (lowerData[P_filterEGlevel] + speed);
        lowerData[P_filterEGlevel] = constrain(lowerData[P_filterEGlevel], 0, 127);
        filterEGlevelstr = lowerData[P_filterEGlevel];
        if (wholemode) {
          upperData[P_filterEGlevel] = lowerData[P_filterEGlevel];
        }
      }

      updatefilterEGlevel(1);
      break;

    case 24:
      if (upperSW) {
        upperData[P_filterLFO] = (upperData[P_filterLFO] + speed);
        upperData[P_filterLFO] = constrain(upperData[P_filterLFO], 0, 127);
        filterLFOstr = upperData[P_filterLFO];
      } else {
        lowerData[P_filterLFO] = (lowerData[P_filterLFO] + speed);
        lowerData[P_filterLFO] = constrain(lowerData[P_filterLFO], 0, 127);
        filterLFOstr = lowerData[P_filterLFO];
        if (wholemode) {
          upperData[P_filterLFO] = lowerData[P_filterLFO];
        }
      }

      updatefilterLFO(1);
      break;

    case 25:
      if (upperSW) {
        upperData[P_noiseLevel] = (upperData[P_noiseLevel] + speed);
        upperData[P_noiseLevel] = constrain(upperData[P_noiseLevel], 0, 127);
        noiseLevelstr = upperData[P_noiseLevel];
      } else {
        lowerData[P_noiseLevel] = (lowerData[P_noiseLevel] + speed);
        lowerData[P_noiseLevel] = constrain(lowerData[P_noiseLevel], 0, 127);
        noiseLevelstr = lowerData[P_noiseLevel];
        if (wholemode) {
          upperData[P_noiseLevel] = lowerData[P_noiseLevel];
        }
      }

      updatenoiseLevel(1);
      break;

    case 26:
      if (upperSW) {
        upperData[P_osc2TriangleLevel] = (upperData[P_osc2TriangleLevel] + speed);
        upperData[P_osc2TriangleLevel] = constrain(upperData[P_osc2TriangleLevel], 0, 127);
        osc2TriangleLevelstr = upperData[P_osc2TriangleLevel];
      } else {
        lowerData[P_osc2TriangleLevel] = (lowerData[P_osc2TriangleLevel] + speed);
        lowerData[P_osc2TriangleLevel] = constrain(lowerData[P_osc2TriangleLevel], 0, 127);
        osc2TriangleLevelstr = lowerData[P_osc2TriangleLevel];
        if (wholemode) {
          upperData[P_osc2TriangleLevel] = lowerData[P_osc2TriangleLevel];
        }
      }

      updateOsc2TriangleLevel(1);
      break;

    case 27:
      if (upperSW) {
        upperData[P_pitchAttack] = (upperData[P_pitchAttack] + speed);
        upperData[P_pitchAttack] = constrain(upperData[P_filterAttack], 0, 127);
        pitchAttackstr = ENVTIMES[upperData[P_pitchAttack]];
      } else {
        lowerData[P_pitchAttack] = (lowerData[P_pitchAttack] + speed);
        lowerData[P_pitchAttack] = constrain(lowerData[P_pitchAttack], 0, 127);
        pitchAttackstr = ENVTIMES[lowerData[P_pitchAttack]];
        if (wholemode) {
          upperData[P_pitchAttack] = lowerData[P_pitchAttack];
        }
      }

      updatepitchAttack(1);
      break;

    case 28:
      if (upperSW) {
        upperData[P_pitchDecay] = (upperData[P_pitchDecay] + speed);
        upperData[P_pitchDecay] = constrain(upperData[P_pitchDecay], 0, 127);
        pitchDecaystr = ENVTIMES[upperData[P_pitchDecay]];
      } else {
        lowerData[P_pitchDecay] = (lowerData[P_pitchDecay] + speed);
        lowerData[P_pitchDecay] = constrain(lowerData[P_pitchDecay], 0, 127);
        pitchDecaystr = ENVTIMES[lowerData[P_pitchDecay]];
        if (wholemode) {
          upperData[P_pitchDecay] = lowerData[P_pitchDecay];
        }
      }

      updatepitchDecay(1);
      break;

    case 29:
      if (upperSW) {
        upperData[P_pitchSustain] = (upperData[P_pitchSustain] + speed);
        upperData[P_pitchSustain] = constrain(upperData[P_pitchSustain], 0, 127);
        pitchSustainstr = LINEAR_FILTERMIXERSTR[upperData[P_pitchSustain]];
      } else {
        lowerData[P_pitchSustain] = (lowerData[P_pitchSustain] + speed);
        lowerData[P_pitchSustain] = constrain(lowerData[P_pitchSustain], 0, 127);
        pitchSustainstr = LINEAR_FILTERMIXERSTR[lowerData[P_pitchSustain]];
        if (wholemode) {
          upperData[P_pitchSustain] = lowerData[P_pitchSustain];
        }
      }

      updatepitchSustain(1);
      break;

    case 30:
      if (upperSW) {
        upperData[P_pitchRelease] = (upperData[P_pitchRelease] + speed);
        upperData[P_pitchRelease] = constrain(upperData[P_pitchRelease], 0, 127);
        pitchReleasestr = ENVTIMES[upperData[P_pitchRelease]];
      } else {
        lowerData[P_pitchRelease] = (lowerData[P_pitchRelease] + speed);
        lowerData[P_pitchRelease] = constrain(lowerData[P_pitchRelease], 0, 127);
        pitchReleasestr = ENVTIMES[lowerData[P_pitchRelease]];
        if (wholemode) {
          upperData[P_pitchRelease] = lowerData[P_pitchRelease];
        }
      }

      updatepitchRelease(1);
      break;

    case 31:
      if (upperSW) {
        upperData[P_ampAttack] = (upperData[P_ampAttack] + speed);
        upperData[P_ampAttack] = constrain(upperData[P_ampAttack], 0, 127);
        ampAttackstr = ENVTIMES[upperData[P_ampAttack]];
      } else {
        lowerData[P_ampAttack] = (lowerData[P_ampAttack] + speed);
        lowerData[P_ampAttack] = constrain(lowerData[P_ampAttack], 0, 127);
        ampAttackstr = ENVTIMES[lowerData[P_ampAttack]];
        if (wholemode) {
          upperData[P_ampAttack] = lowerData[P_ampAttack];
        }
      }

      updateampAttack(1);
      break;

    case 32:
      if (upperSW) {
        upperData[P_ampDecay] = (upperData[P_ampDecay] + speed);
        upperData[P_ampDecay] = constrain(upperData[P_ampDecay], 0, 127);
        ampDecaystr = ENVTIMES[upperData[P_ampDecay]];
      } else {
        lowerData[P_ampDecay] = (lowerData[P_ampDecay] + speed);
        lowerData[P_ampDecay] = constrain(lowerData[P_ampDecay], 0, 127);
        ampDecaystr = ENVTIMES[lowerData[P_ampDecay]];
        if (wholemode) {
          upperData[P_ampDecay] = lowerData[P_ampDecay];
        }
      }

      updateampDecay(1);
      break;

    case 33:
      if (upperSW) {
        upperData[P_filterRelease] = (upperData[P_filterRelease] + speed);
        upperData[P_filterRelease] = constrain(upperData[P_filterRelease], 0, 127);
        filterReleasestr = ENVTIMES[upperData[P_filterRelease]];
      } else {
        lowerData[P_filterRelease] = (lowerData[P_filterRelease] + speed);
        lowerData[P_filterRelease] = constrain(lowerData[P_filterRelease], 0, 127);
        filterReleasestr = ENVTIMES[lowerData[P_filterRelease]];
        if (wholemode) {
          upperData[P_filterRelease] = lowerData[P_filterRelease];
        }
      }

      updatefilterRelease(1);
      break;

    case 34:
      if (upperSW) {
        upperData[P_filterSustain] = (upperData[P_filterSustain] + speed);
        upperData[P_filterSustain] = constrain(upperData[P_filterSustain], 0, 127);
        filterSustainstr = LINEAR_FILTERMIXERSTR[upperData[P_filterSustain]];
      } else {
        lowerData[P_filterSustain] = (lowerData[P_filterSustain] + speed);
        lowerData[P_filterSustain] = constrain(lowerData[P_filterSustain], 0, 127);
        filterSustainstr = LINEAR_FILTERMIXERSTR[lowerData[P_filterSustain]];
        if (wholemode) {
          upperData[P_filterSustain] = lowerData[P_filterSustain];
        }
      }

      updatefilterSustain(1);
      break;

    case 35:
      if (upperSW) {
        upperData[P_filterDecay] = (upperData[P_filterDecay] + speed);
        upperData[P_filterDecay] = constrain(upperData[P_filterDecay], 0, 127);
        filterDecaystr = ENVTIMES[upperData[P_filterDecay]];
      } else {
        lowerData[P_filterDecay] = (lowerData[P_filterDecay] + speed);
        lowerData[P_filterDecay] = constrain(lowerData[P_filterDecay], 0, 127);
        filterDecaystr = ENVTIMES[lowerData[P_filterDecay]];
        if (wholemode) {
          upperData[P_filterDecay] = lowerData[P_filterDecay];
        }
      }

      updatefilterDecay(1);
      break;

    case 36:
      if (upperSW) {
        upperData[P_filterAttack] = (upperData[P_filterAttack] + speed);
        upperData[P_filterAttack] = constrain(upperData[P_filterAttack], 0, 127);
        filterAttackstr = ENVTIMES[upperData[P_filterAttack]];
      } else {
        lowerData[P_filterAttack] = (lowerData[P_filterAttack] + speed);
        lowerData[P_filterAttack] = constrain(lowerData[P_filterAttack], 0, 127);
        filterAttackstr = ENVTIMES[lowerData[P_filterAttack]];
        if (wholemode) {
          upperData[P_filterAttack] = lowerData[P_filterAttack];
        }
      }

      updatefilterAttack(1);
      break;

    case 37:
      if (upperSW) {
        upperData[P_ampSustain] = (upperData[P_ampSustain] + speed);
        upperData[P_ampSustain] = constrain(upperData[P_ampSustain], 0, 127);
        ampSustainstr = LINEAR_FILTERMIXERSTR[upperData[P_ampSustain]];
      } else {
        lowerData[P_ampSustain] = (lowerData[P_ampSustain] + speed);
        lowerData[P_ampSustain] = constrain(lowerData[P_ampSustain], 0, 127);
        ampSustainstr = LINEAR_FILTERMIXERSTR[lowerData[P_ampSustain]];
        if (wholemode) {
          upperData[P_ampSustain] = lowerData[P_ampSustain];
        }
      }

      updateampSustain(1);
      break;

    case 38:
      if (upperSW) {
        upperData[P_ampRelease] = (upperData[P_ampRelease] + speed);
        upperData[P_ampRelease] = constrain(upperData[P_ampRelease], 0, 127);
        ampReleasestr = ENVTIMES[upperData[P_ampRelease]];
      } else {
        lowerData[P_ampRelease] = (lowerData[P_ampRelease] + speed);
        lowerData[P_ampRelease] = constrain(lowerData[P_ampRelease], 0, 127);
        ampReleasestr = ENVTIMES[lowerData[P_ampRelease]];
        if (wholemode) {
          upperData[P_ampRelease] = lowerData[P_ampRelease];
        }
      }

      updateampRelease(1);
      break;

    case 39:
      if (upperSW) {
        upperData[P_ATDepth] = (upperData[P_ATDepth] + speed);
        upperData[P_ATDepth] = constrain(upperData[P_ATDepth], 0, 127);
        ATDepthstr = upperData[P_ATDepth];
      } else {
        lowerData[P_ATDepth] = (lowerData[P_ATDepth] + speed);
        lowerData[P_ATDepth] = constrain(lowerData[P_ATDepth], 0, 127);
        ATDepthstr = lowerData[P_ATDepth];
        if (wholemode) {
          upperData[P_ATDepth] = lowerData[P_ATDepth];
        }
      }

      updateATDepth(1);
      break;

    case 40:
      if (upperSW) {
        upperData[P_fmDepth] = (upperData[P_fmDepth] + speed);
        upperData[P_fmDepth] = constrain(upperData[P_fmDepth], 0, 127);
        fmDepthstr = upperData[P_fmDepth];
      } else {
        lowerData[P_fmDepth] = (lowerData[P_fmDepth] + speed);
        lowerData[P_fmDepth] = constrain(lowerData[P_fmDepth], 0, 127);
        fmDepthstr = lowerData[P_fmDepth];
        if (wholemode) {
          upperData[P_fmDepth] = lowerData[P_fmDepth];
        }
      }

      updatefmDepth(1);
      break;

    case 41:
      if (!clockwise) {
        speed = -1;
      } else {
        speed = +1;
      }
      if (upperSW) {
        upperData[P_PitchBendLevel] = (upperData[P_PitchBendLevel] + speed);
        upperData[P_PitchBendLevel] = constrain(upperData[P_PitchBendLevel], 0, 12);
        PitchBendLevelstr = upperData[P_PitchBendLevel];
      } else {
        lowerData[P_PitchBendLevel] = (lowerData[P_PitchBendLevel] + speed);
        lowerData[P_PitchBendLevel] = constrain(lowerData[P_PitchBendLevel], 0, 12);
        PitchBendLevelstr = lowerData[P_PitchBendLevel];
        if (wholemode) {
          upperData[P_PitchBendLevel] = lowerData[P_PitchBendLevel];
        }
      }

      updatePitchBendDepth(1);
      break;

    case 42:
      if (upperSW) {
        upperData[P_amDepth] = (upperData[P_amDepth] + speed);
        upperData[P_amDepth] = constrain(upperData[P_amDepth], 0, 127);
        amDepthstr = upperData[P_amDepth];
      } else {
        lowerData[P_amDepth] = (lowerData[P_amDepth] + speed);
        lowerData[P_amDepth] = constrain(lowerData[P_amDepth], 0, 127);
        amDepthstr = lowerData[P_amDepth];
        if (wholemode) {
          upperData[P_amDepth] = lowerData[P_amDepth];
        }
      }

      updateamDepth(1);
      break;

    case 43:
      if (upperSW) {
        upperData[P_modWheelDepth] = (upperData[P_modWheelDepth] + speed);
        upperData[P_modWheelDepth] = constrain(upperData[P_modWheelDepth], 0, 127);
        modWheelDepthstr = upperData[P_modWheelDepth];
      } else {
        lowerData[P_modWheelDepth] = (lowerData[P_modWheelDepth] + speed);
        lowerData[P_modWheelDepth] = constrain(lowerData[P_modWheelDepth], 0, 127);
        modWheelDepthstr = lowerData[P_modWheelDepth];
        if (wholemode) {
          upperData[P_modWheelDepth] = lowerData[P_modWheelDepth];
        }
      }

      updatemodWheelDepth(1);
      break;

    case 44:
      if (upperSW) {
        upperData[P_volumeControl] = (upperData[P_volumeControl] + speed);
        upperData[P_volumeControl] = constrain(upperData[P_volumeControl], 0, 127);
        volumeControlstr = upperData[P_volumeControl];
      } else {
        lowerData[P_volumeControl] = (lowerData[P_volumeControl] + speed);
        lowerData[P_volumeControl] = constrain(lowerData[P_volumeControl], 0, 127);
        volumeControlstr = lowerData[P_volumeControl];
        if (wholemode) {
          upperData[P_volumeControl] = lowerData[P_volumeControl];
        }
      }

      updatevolumeControl(1);
      break;

    case 45:
      if (upperSW) {
        upperData[P_LFO1Rate] = (upperData[P_LFO1Rate] + speed);
        upperData[P_LFO1Rate] = constrain(upperData[P_LFO1Rate], 0, 127);
        LFO1Ratestr = LFOTEMPO[upperData[P_LFO1Rate]];
      } else {
        lowerData[P_LFO1Rate] = (lowerData[P_LFO1Rate] + speed);
        lowerData[P_LFO1Rate] = constrain(lowerData[P_LFO1Rate], 0, 127);
        LFO1Ratestr = LFOTEMPO[lowerData[P_LFO1Rate]];
        if (wholemode) {
          upperData[P_LFO1Rate] = lowerData[P_LFO1Rate];
        }
      }

      updateLFO1Rate(1);
      break;

    case 46:
      if (upperSW) {
        upperData[P_LFO2Rate] = (upperData[P_LFO2Rate] + speed);
        upperData[P_LFO2Rate] = constrain(upperData[P_LFO2Rate], 0, 127);
        LFO2Ratestr = LFOTEMPO[upperData[P_LFO2Rate]];
      } else {
        lowerData[P_LFO2Rate] = (lowerData[P_LFO2Rate] + speed);
        lowerData[P_LFO2Rate] = constrain(lowerData[P_LFO2Rate], 0, 127);
        LFO2Ratestr = LFOTEMPO[lowerData[P_LFO2Rate]];
        if (wholemode) {
          upperData[P_LFO2Rate] = lowerData[P_LFO2Rate];
        }
      }

      updateLFO2Rate(1);
      break;

    case 47:
      if (upperSW) {
        upperData[P_LFO3Rate] = (upperData[P_LFO3Rate] + speed);
        upperData[P_LFO3Rate] = constrain(upperData[P_LFO3Rate], 0, 127);
        LFO3Ratestr = LFOTEMPO[upperData[P_LFO3Rate]];
      } else {
        lowerData[P_LFO3Rate] = (lowerData[P_LFO3Rate] + speed);
        lowerData[P_LFO3Rate] = constrain(lowerData[P_LFO3Rate], 0, 127);
        LFO3Ratestr = LFOTEMPO[lowerData[P_LFO3Rate]];
        if (wholemode) {
          upperData[P_LFO3Rate] = lowerData[P_LFO3Rate];
        }
      }

      updateLFO3Rate(1);
      break;

    case 48:
      if (upperSW) {
        upperData[P_LFO3Delay] = (upperData[P_LFO3Delay] + speed);
        upperData[P_LFO3Delay] = constrain(upperData[P_LFO3Delay], 0, 127);
        LFO3Delaystr = upperData[P_LFO3Delay];
      } else {
        lowerData[P_LFO3Delay] = (lowerData[P_LFO3Delay] + speed);
        lowerData[P_LFO3Delay] = constrain(lowerData[P_LFO3Delay], 0, 127);
        LFO3Delaystr = lowerData[P_LFO3Delay];
        if (wholemode) {
          upperData[P_LFO3Delay] = lowerData[P_LFO3Delay];
        }
      }

      updateLFO3Delay(1);
      break;

    case 49:
      if (!clockwise) {
        speed = -1;
      } else {
        speed = +1;
      }
      if (upperSW) {
        upperData[P_LFO3Waveform] = (upperData[P_LFO3Waveform] + speed);
        upperData[P_LFO3Waveform] = constrain(upperData[P_LFO3Waveform], 0, 15);
        LFO3Waveformstr = upperData[P_LFO3Waveform];
      } else {
        lowerData[P_LFO3Waveform] = (lowerData[P_LFO3Waveform] + speed);
        lowerData[P_LFO3Waveform] = constrain(lowerData[P_LFO3Waveform], 0, 15);
        LFO3Waveformstr = lowerData[P_LFO3Waveform];
        if (wholemode) {
          upperData[P_LFO3Waveform] = lowerData[P_LFO3Waveform];
        }
      }

      updateLFO3Waveform(1);
      break;

    case 50:
      if (upperSW) {
        upperData[P_LFO1Slope] = (upperData[P_LFO1Slope] + speed);
        upperData[P_LFO1Slope] = constrain(upperData[P_LFO1Slope], 0, 127);
        LFO1Slopestr = upperData[P_LFO1Slope];
      } else {
        lowerData[P_LFO1Slope] = (lowerData[P_LFO1Slope] + speed);
        lowerData[P_LFO1Slope] = constrain(lowerData[P_LFO1Slope], 0, 127);
        LFO1Slopestr = lowerData[P_LFO1Slope];
        if (wholemode) {
          upperData[P_LFO1Slope] = lowerData[P_LFO1Slope];
        }
      }

      updateLFO1Slope(1);
      break;

    case 51:
      if (upperSW) {
        upperData[P_LFO1Delay] = (upperData[P_LFO1Delay] + speed);
        upperData[P_LFO1Delay] = constrain(upperData[P_LFO1Delay], 0, 127);
        LFO1Delaystr = upperData[P_LFO1Delay];
      } else {
        lowerData[P_LFO1Delay] = (lowerData[P_LFO1Delay] + speed);
        lowerData[P_LFO1Delay] = constrain(lowerData[P_LFO1Delay], 0, 127);
        LFO1Delaystr = lowerData[P_LFO1Delay];
        if (wholemode) {
          upperData[P_LFO1Delay] = lowerData[P_LFO1Delay];
        }
      }

      updateLFO1Delay(1);
      break;

    case 52:
      if (upperSW) {
        upperData[P_effectPot2] = (upperData[P_effectPot2] + speed);
        upperData[P_effectPot2] = constrain(upperData[P_effectPot2], 0, 127);
        effectPot2str = upperData[P_effectPot2];
      } else {
        lowerData[P_effectPot2] = (lowerData[P_effectPot2] + speed);
        lowerData[P_effectPot2] = constrain(lowerData[P_effectPot2], 0, 127);
        effectPot2str = lowerData[P_effectPot2];
        if (wholemode) {
          upperData[P_effectPot2] = lowerData[P_effectPot2];
        }
      }

      updateeffectPot2(1);
      break;

    case 53:
      if (upperSW) {
        upperData[P_effectPot1] = (upperData[P_effectPot1] + speed);
        upperData[P_effectPot1] = constrain(upperData[P_effectPot1], 0, 127);
        effectPot1str = upperData[P_effectPot1];
      } else {
        lowerData[P_effectPot1] = (lowerData[P_effectPot1] + speed);
        lowerData[P_effectPot1] = constrain(lowerData[P_effectPot1], 0, 127);
        effectPot1str = lowerData[P_effectPot1];
        if (wholemode) {
          upperData[P_effectPot1] = lowerData[P_effectPot1];
        }
      }

      updateeffectPot1(1);
      break;

    case 54:
      if (upperSW) {
        upperData[P_effectPot3] = (upperData[P_effectPot3] + speed);
        upperData[P_effectPot3] = constrain(upperData[P_effectPot3], 0, 127);
        effectPot3str = upperData[P_effectPot3];
      } else {
        lowerData[P_effectPot3] = (lowerData[P_effectPot3] + speed);
        lowerData[P_effectPot3] = constrain(lowerData[P_effectPot3], 0, 127);
        effectPot3str = lowerData[P_effectPot3];
        if (wholemode) {
          upperData[P_effectPot3] = lowerData[P_effectPot3];
        }
      }

      updateeffectPot3(1);
      break;

    case 55:
      if (upperSW) {
        upperData[P_effectsMix] = (upperData[P_effectsMix] + speed);
        upperData[P_effectsMix] = constrain(upperData[P_effectsMix], 0, 127);
        effectsMixstr = LINEARCENTREZERO[upperData[P_effectsMix]];
      } else {
        lowerData[P_effectsMix] = (lowerData[P_effectsMix] + speed);
        lowerData[P_effectsMix] = constrain(lowerData[P_effectsMix], 0, 127);
        effectsMixstr = LINEARCENTREZERO[lowerData[P_effectsMix]];
        if (wholemode) {
          upperData[P_effectsMix] = lowerData[P_effectsMix];
        }
      }

      updateeffectsMix(1);
      break;

    case 56:
      if (upperSW) {
        upperData[P_osc2envDepth] = (upperData[P_osc2envDepth] + speed);
        upperData[P_osc2envDepth] = constrain(upperData[P_osc2envDepth], 0, 127);
        osc2envDepthstr = upperData[P_osc2envDepth];
      } else {
        lowerData[P_osc2envDepth] = (lowerData[P_osc2envDepth] + speed);
        lowerData[P_osc2envDepth] = constrain(lowerData[P_osc2envDepth], 0, 127);
        osc2envDepthstr = lowerData[P_osc2envDepth];
        if (wholemode) {
          upperData[P_osc2envDepth] = lowerData[P_osc2envDepth];
        }
      }

      updateOsc2EnvDepth(1);
      break;
  }

  //rotaryEncoderChanged(id, clockwise, speed);
}

void mainButtonChanged(Button *btn, bool released) {

  switch (btn->id) {

    case LFO3_RETRIG_BUTTON:
      if (!released) {
        panelData[P_monoMulti] = !panelData[P_monoMulti];
        myControlChange(midiChannel, CCmonoMulti, panelData[P_monoMulti]);
      }
      break;

    case LFO1_RETRIG_BUTTON:
      if (!released) {
        panelData[P_lfo1retrig] = !panelData[P_lfo1retrig];
        myControlChange(midiChannel, CClfo1retrig, panelData[P_lfo1retrig]);
      }
      break;    

      // if (btnIndex == CHORD_HOLD_SW && btnType == ROX_PRESSED) {
      //   chordHoldSW = !chordHoldSW;
      //   myControlChange(midiChannel, CCchordHoldSW, chordHoldSW);
      // }

    case EFFECT_ROTARY_BUTTON:
      if (!released) {
        if (upperSW) {
          upperfootPedal = true;
        } else {
          lowerfootPedal = true;
        }
        updatefootSwitch();
      }
      break;

    case GLIDE_BUTTON:
      if (!released) {
        panelData[P_glideSW] = !panelData[P_glideSW];
        myControlChange(midiChannel, CCglideSW, panelData[P_glideSW]);
      }
      break;

    case LFO1_WAVE_BUTTON:
      if (!released) {
        panelData[P_LFO1Waveform] = panelData[P_LFO1Waveform] + 1;
        if (panelData[P_LFO1Waveform] > 2) {
          panelData[P_LFO1Waveform] = 0;
        }
        myControlChange(midiChannel, CCLFO1Waveform, panelData[P_LFO1Waveform]);
      }
      break;

    case LFO2_WAVE_BUTTON:
      if (!released) {
        panelData[P_LFO2Waveform] = panelData[P_LFO2Waveform] + 1;
        if (panelData[P_LFO2Waveform] > 2) {
          panelData[P_LFO2Waveform] = 0;
        }
        myControlChange(midiChannel, CCLFO2Waveform, panelData[P_LFO2Waveform]);
      }
      break;

    case AMP_ENV_GATE_BUTTON:
      if (!released) {
        panelData[P_vcaGate] = !panelData[P_vcaGate];
        myControlChange(midiChannel, CCvcaGate, panelData[P_vcaGate]);
      }
      break;

    case DCO1_OCT_BUTTON:
      if (!released) {
        panelData[P_osc1Range] = panelData[P_osc1Range] + 1;
        if (panelData[P_osc1Range] > 2) {
          panelData[P_osc1Range] = 0;
        }
        myControlChange(midiChannel, CCosc1Oct, panelData[P_osc1Range]);
      }
      break;

    case DCO2_OCT_BUTTON:
      if (!released) {
        panelData[P_osc2Range] = panelData[P_osc2Range] + 1;
        if (panelData[P_osc2Range] > 2) {
          panelData[P_osc2Range] = 0;
        }
        myControlChange(midiChannel, CCosc2Oct, panelData[P_osc2Range]);
      }
      break;

    case DCO_AT_BUTTON:
      if (!released) {
        panelData[P_dco_at_SW] = !panelData[P_dco_at_SW];
        myControlChange(midiChannel, CCdco_at_SW, panelData[P_dco_at_SW]);
      }
      break;

    case FILTER_AT_BUTTON:
      if (!released) {
        panelData[P_filter_at_SW] = !panelData[P_filter_at_SW];
        myControlChange(midiChannel, CCfilter_at_SW, panelData[P_filter_at_SW]);
      }
      break;

    case VCF_POLE_BUTTON:
      if (!released) {
        panelData[P_filterPoleSW] = !panelData[P_filterPoleSW];
        myControlChange(midiChannel, CCfilterPoleSW, panelData[P_filterPoleSW]);
      }
      break;

    case VCF_EG_INV_BUTTON:
      if (!released) {
        panelData[P_filterEGinv] = !panelData[P_filterEGinv];
        myControlChange(midiChannel, CCfilterEGinv, panelData[P_filterEGinv]);
      }
      break;

    case VCF_TYPE_BUTTON:
      if (!released) {
        panelData[P_filterType] = panelData[P_filterType] + 1;
        if (panelData[P_filterType] > 7) {
          panelData[P_filterType] = 0;
        }
        myControlChange(midiChannel, CCfilterType, panelData[P_filterType]);
      }
      break;

    case LFO3_MULT_BUTTON:
      if (!released) {
        panelData[P_lfoMultiplier] = panelData[P_lfoMultiplier] + 1;
        if (panelData[P_lfoMultiplier] > 3) {
          panelData[P_lfoMultiplier] = 0;
        }
        myControlChange(midiChannel, CClfoMult, panelData[P_lfoMultiplier]);
      }
      break;

    case VCF_VELOCITY_BUTTON:
      if (!released) {
        panelData[P_filterVel] = !panelData[P_filterVel];
        myControlChange(midiChannel, CCfilterVel, panelData[P_filterVel]);
      }
      break;

    case AMP_VELOCITY_BUTTON:
      if (!released) {
        panelData[P_vcaVel] = !panelData[P_vcaVel];
        myControlChange(midiChannel, CCvcaVel, panelData[P_vcaVel]);
      }
      break;

    case VCF_LOOP_BUTTON:
      if (!released) {
        panelData[P_filterLoop] = panelData[P_filterLoop] + 1;
        if (panelData[P_filterLoop] > 2) {
          panelData[P_filterLoop] = 0;
        }
        myControlChange(midiChannel, CCFilterLoop, panelData[P_filterLoop]);
      }
      break;

    case AMP_LOOP_BUTTON:
      if (!released) {
        panelData[P_vcaLoop] = panelData[P_vcaLoop] + 1;
        if (panelData[P_vcaLoop] > 2) {
          panelData[P_vcaLoop] = 0;
        }
        myControlChange(midiChannel, CCAmpLoop, panelData[P_vcaLoop]);
      }
      break;

    case EFFECT_NUM_BUTTON:
      if (!released) {
        panelData[P_effectNum] = panelData[P_effectNum] + 1;
        if (panelData[P_effectNum] > 7) {
          panelData[P_effectNum] = 0;
        }
        myControlChange(midiChannel, CCeffectNumSW, panelData[P_effectNum]);
      }
      break;

    case EFFECT_BANK_BUTTON:
      if (!released) {
        panelData[P_effectBank] = panelData[P_effectBank] + 1;
        if (panelData[P_effectBank] > 3) {
          panelData[P_effectBank] = 0;
        }
        myControlChange(midiChannel, CCeffectBankSW, panelData[P_effectBank]);
      }
      break;

    case VCF_LIN_LOG_BUTTON:
      if (!released) {
        panelData[P_filterLogLin] = !panelData[P_filterLogLin];
        myControlChange(midiChannel, CCfilterenvLinLogSW, panelData[P_filterLogLin]);
      }
      break;

    case AMP_LIN_LOG_BUTTON:
      if (!released) {
        panelData[P_ampLogLin] = !panelData[P_ampLogLin];
        myControlChange(midiChannel, CCampenvLinLogSW, panelData[P_ampLogLin]);
      }
      break;

    case NOISE_SRC_BUTTON:
      if (!released) {
        panelData[P_noiseSrc] = !panelData[P_noiseSrc];
        myControlChange(midiChannel, CCnoiseSrc, panelData[P_noiseSrc]);
      }
      break;

    case POLY1_BUTTON:
      if (!released) {
        panelData[P_keyboardMode] = 0;
        myControlChange(midiChannel, CCkeyboardMode, panelData[P_keyboardMode]);
      }
      break;

    case POLY2_BUTTON:
      if (!released) {
        panelData[P_keyboardMode] = 1;
        myControlChange(midiChannel, CCkeyboardMode, panelData[P_keyboardMode]);
      }
      break;

    case UNISON_BUTTON:
      if (!released) {
        panelData[P_keyboardMode] = 2;
        myControlChange(midiChannel, CCkeyboardMode, panelData[P_keyboardMode]);
      }
      break;

    case MONO_BUTTON:
      if (!released) {
        panelData[P_keyboardMode] = 3;
        myControlChange(midiChannel, CCkeyboardMode, panelData[P_keyboardMode]);
      }
      break;

    case MODE_BUTTON:
      if (!released) {
        playMode = playMode + 1;
        if (playMode > 2) {
          playMode = 0;
        }
        myControlChange(midiChannel, CCplayMode, playMode);
      }
      break;

    case PRIORITY_BUTTON:
      if (!released) {
        panelData[P_NotePriority] = panelData[P_NotePriority] + 1;
        if (panelData[P_NotePriority] > 2) {
          panelData[P_NotePriority] = 0;
        }
        myControlChange(midiChannel, CCNotePriority, panelData[P_NotePriority]);
      }
      break;

    case DCO2_SYNC_BUTTON:
      if (!released) {
        panelData[P_sync] = panelData[P_sync] + 1;
        if (panelData[P_sync] > 2) {
          panelData[P_sync] = 0;
        }
        myControlChange(midiChannel, CCsyncSW, panelData[P_sync]);
      }
      break;

    case VCF_KEYTRACK_BUTTON:
      if (!released) {
        panelData[P_keytrackSW] = !panelData[P_keytrackSW];
        myControlChange(midiChannel, CCkeyTrackSW, panelData[P_keytrackSW]);
      }
      break;

    case LOWER_BUTTON:
      if (!released) {
        lowerSW = true;
        upperSW = false;
        myControlChange(midiChannel, CClowerSW, lowerSW);
      }
      break;

    case UPPER_BUTTON:
      if (!released) {
        lowerSW = false;
        upperSW = true;
        myControlChange(midiChannel, CCupperSW, upperSW);
      }
      break;
  }
}

void recallPerformance(const Performance &perf) {
  currentPerformance = perf;
  playMode = perf.mode;

  switch (playMode) {
    case WHOLE:
      recallPatch(perf.lowerPatchNo);
      patchNo = perf.lowerPatchNo;
      refreshPatchDisplayFromState();
      break;
    case DUAL:
    case SPLIT:
      recallPatch(perf.upperPatchNo);
      recallPatch(perf.lowerPatchNo);
      patchNo = perf.lowerPatchNo;
      refreshPatchDisplayFromState();
      break;
  }
}

void refreshPatchDisplayFromState() {
  showPatchPage(
    currentPgmNumU,
    currentPatchNameU,
    currentPgmNumL,
    currentPatchNameL);
}

String getModeName(PlayMode mode) {
  switch (mode) {
    case WHOLE: return "Whole";
    case DUAL: return "Dual";
    case SPLIT: return "Split";
    default: return "-";
  }
}


void loadPerformances() {
  performances.clear();
  File dir = SD.open("/performances");

  if (!dir || !dir.isDirectory()) {
    Serial.println("/performances not found or is not a directory");
    return;
  }

  while (true) {
    File file = dir.openNextFile();
    if (!file) break;

    if (file.isDirectory()) {
      file.close();
      continue;
    }

    String dataLine = file.readStringUntil('\n');
    file.close();

    if (dataLine.length() > 0) {
      int comma1 = dataLine.indexOf(',');
      int comma2 = dataLine.indexOf(',', comma1 + 1);
      int comma3 = dataLine.indexOf(',', comma2 + 1);

      if (comma1 == -1 || comma2 == -1 || comma3 == -1) continue;

      int upper = dataLine.substring(0, comma1).toInt();
      int lower = dataLine.substring(comma1 + 1, comma2).toInt();
      String name = dataLine.substring(comma2 + 1, comma3);
      int mode = dataLine.substring(comma3 + 1).toInt();

      int perfNo = performances.size() + 1;
      performances.push({ perfNo, upper, lower, name, (PlayMode)mode });
    }
  }

  if (performances.size() == 0) {
    Performance defaultPerf = { 1, 1, 1, "Default", WHOLE };
    savePerformance("perf001", defaultPerf);
    loadPerformances();  // try again
  }
}

void savePerformance(const char *fileName, const Performance &perf) {
  String path = "/performances/" + String(fileName);

  if (SD.exists(path.c_str())) {
    SD.remove(path.c_str());
  }

  File file = SD.open(path.c_str(), FILE_WRITE);
  if (file) {
    file.print(perf.upperPatchNo);
    file.print(",");
    file.print(perf.lowerPatchNo);
    file.print(",");
    file.print(perf.name);
    file.print(",");
    file.println((int)perf.mode);  // Save playMode as an integer (0, 1, 2)
    file.close();
  } else {
    Serial.print("Failed to save performance: ");
    Serial.println(path);
  }
}

void editControlChange(byte channel, byte control, byte value) {
  int newvalue = value;
  myControlChange(channel, control, newvalue);
}

int mod(int a, int b) {
  int r = a % b;
  return r < 0 ? r + b : r;
}

void setTranspose(int splitTrans) {
  switch (splitTrans) {
    case 0:
      lowerTranspose = -24;
      oldsplitTrans = splitTrans;
      break;

    case 1:
      lowerTranspose = -12;
      oldsplitTrans = splitTrans;
      break;

    case 2:
      lowerTranspose = 0;
      oldsplitTrans = splitTrans;
      break;

    case 3:
      lowerTranspose = 12;
      oldsplitTrans = splitTrans;
      break;

    case 4:
      lowerTranspose = 24;
      oldsplitTrans = splitTrans;
      break;
  }
}

void LFODelayHandle() {
  // LFO Delay code
  getDelayTime();

  unsigned long currentMillisU = millis();
  if (upperData[P_monoMulti] && !upperData[P_LFODelayGo]) {
    if (oldnumberOfNotesU < numberOfNotesU) {
      previousMillisU = currentMillisU;
      oldnumberOfNotesU = numberOfNotesU;
    }
  }
  if (numberOfNotesU > 0) {
    if (currentMillisU - previousMillisU >= intervalU) {
      upperData[P_LFODelayGo] = 1;
    } else {
      upperData[P_LFODelayGo] = 0;
    }
  } else {
    upperData[P_LFODelayGo] = 1;
    previousMillisU = currentMillisU;  //reset timer so its ready for the next time
  }

  unsigned long currentMillisL = millis();
  if (lowerData[P_monoMulti] && !lowerData[P_LFODelayGo]) {
    if (oldnumberOfNotesL < numberOfNotesL) {
      previousMillisL = currentMillisL;
      oldnumberOfNotesL = numberOfNotesL;
    }
  }
  if (numberOfNotesL > 0) {
    if (currentMillisL - previousMillisL >= intervalL) {
      lowerData[P_LFODelayGo] = 1;
    } else {
      lowerData[P_LFODelayGo] = 0;
    }
  } else {
    lowerData[P_LFODelayGo] = 1;
    previousMillisL = currentMillisL;  //reset timer so its ready for the next time
  }
}

// Mono lower & uppper

void commandTopNoteLower() {
  int topNote = -1;
  for (int i = 0; i < 128; i++)
    if (notesLower[i]) topNote = i;

  if (topNote >= 0)
    assignVoice(topNote, noteVel, 0);
  else
    releaseVoice(noteMsg, 0);
}

void commandBottomNoteLower() {
  int bottomNote = -1;
  for (int i = 127; i >= 0; i--)
    if (notesLower[i]) bottomNote = i;

  if (bottomNote >= 0)
    assignVoice(bottomNote, noteVel, 0);
  else
    releaseVoice(noteMsg, 0);
}

void commandLastNoteLower() {
  for (int i = 0; i < 40; i++) {
    int8_t idx = noteOrderLower[mod(orderIndxLower - i, 40)];
    if (notesLower[idx]) {
      assignVoice(idx, noteVel, 0);
      return;
    }
  }
  releaseVoice(noteMsg, 0);
}

void commandTopNoteUpper() {
  int topNote = -1;
  for (int i = 0; i < 128; i++)
    if (notesUpper[i]) topNote = i;

  if (topNote >= 0)
    assignVoice(topNote, noteVel, 6);
  else
    releaseVoice(noteMsg, 6);
}

void commandBottomNoteUpper() {
  int bottomNote = -1;
  for (int i = 127; i >= 0; i--)
    if (notesUpper[i]) bottomNote = i;

  if (bottomNote >= 0)
    assignVoice(bottomNote, noteVel, 6);
  else
    releaseVoice(noteMsg, 6);
}

void commandLastNoteUpper() {
  for (int i = 0; i < 40; i++) {
    int8_t idx = noteOrderUpper[mod(orderIndxUpper - i, 40)];
    if (notesUpper[idx]) {
      assignVoice(idx, noteVel, 6);
      return;
    }
  }
  releaseVoice(noteMsg, 6);
}

// Unison lower and upper

void commandTopNoteUniLower() {
  int topNote = -1;
  for (int i = 0; i < 128; i++)
    if (notesLower[i]) topNote = i;

  if (topNote >= 0)
    for (int v = 0; v < 6; v++) assignVoice(topNote, noteVel, v);
  else
    for (int v = 0; v < 6; v++) releaseVoice(noteMsg, v);
}

void commandBottomNoteUniLower() {
  int bottomNote = -1;
  for (int i = 127; i >= 0; i--)
    if (notesLower[i]) bottomNote = i;

  if (bottomNote >= 0)
    for (int v = 0; v < 6; v++) assignVoice(bottomNote, noteVel, v);
  else
    for (int v = 0; v < 6; v++) releaseVoice(noteMsg, v);
}

void commandLastNoteUniLower() {
  for (int i = 0; i < 40; i++) {
    int8_t idx = noteOrderLower[mod(orderIndxLower - i, 40)];
    if (notesLower[idx]) {
      for (int v = 0; v < 6; v++) assignVoice(idx, noteVel, v);
      return;
    }
  }
  for (int v = 0; v < 6; v++) releaseVoice(noteMsg, v);
}

void commandTopNoteUniUpper() {
  int topNote = -1;
  for (int i = 0; i < 128; i++)
    if (notesUpper[i]) topNote = i;

  if (topNote >= 0)
    for (int v = 6; v < 12; v++) assignVoice(topNote, noteVel, v);
  else
    for (int v = 6; v < 12; v++) releaseVoice(noteMsg, v);
}

void commandBottomNoteUniUpper() {
  int bottomNote = -1;
  for (int i = 127; i >= 0; i--)
    if (notesUpper[i]) bottomNote = i;

  if (bottomNote >= 0)
    for (int v = 6; v < 12; v++) assignVoice(bottomNote, noteVel, v);
  else
    for (int v = 6; v < 12; v++) releaseVoice(noteMsg, v);
}

void commandLastNoteUniUpper() {
  for (int i = 0; i < 40; i++) {
    int8_t idx = noteOrderUpper[mod(orderIndxUpper - i, 40)];
    if (notesUpper[idx]) {
      for (int v = 6; v < 12; v++) assignVoice(idx, noteVel, v);
      return;
    }
  }
  for (int v = 6; v < 12; v++) releaseVoice(noteMsg, v);
}

void memorizeChordFromVoices() {
  uint8_t heldNotes[MAX_CHORD_NOTES];
  uint8_t count = 0;
  for (int i = 0; i < NO_OF_VOICES; ++i) {
    // Use .noteOn or voiceOn[] (either works)
    if (voices[i].note >= 0 && voices[i].noteOn) {
      bool already = false;
      for (int j = 0; j < count; ++j)
        if (heldNotes[j] == voices[i].note) already = true;
      if (!already && count < MAX_CHORD_NOTES)
        heldNotes[count++] = voices[i].note;
    }
  }
  if (count > 0) {
    // Sort
    for (int i = 0; i < count - 1; i++)
      for (int j = i + 1; j < count; j++)
        if (heldNotes[j] < heldNotes[i])
          std::swap(heldNotes[i], heldNotes[j]);
    chordHoldRoot = heldNotes[0];
    chordHoldCount = count;
    for (int i = 0; i < count; i++)
      chordHoldIntervals[i] = heldNotes[i] - chordHoldRoot;
    chordHoldActive = true;
    chordHoldWaitingForNotes = false;
    //Serial.print("Chord Hold: root ");
    //Serial.print(chordHoldRoot);
    //Serial.print(" intervals: ");
    //for (int i = 0; i < count; i++) Serial.print((int)chordHoldIntervals[i]), Serial.print(" ");
    //Serial.println();
  } else {
    chordHoldActive = true;
    chordHoldWaitingForNotes = false;
    chordHoldCount = 0;
    //Serial.println("Chord Hold: No chord detected, disarmed.");
  }
}

void onHoldButtonPressed() {
  chordHoldActive = true;
  chordHoldWaitingForNotes = true;
  chordHoldCount = 0;

  // --- New: if notes are already held, capture immediately ---
  bool anyActive = false;
  for (int i = 0; i < NO_OF_VOICES; ++i) {
    if (voices[i].note >= 0 && voices[i].noteOn) {
      anyActive = true;
      break;
    }
  }
  if (anyActive) {
    memorizeChordFromVoices();
    chordHoldWaitingForNotes = false;
    chordHoldCaptureWindowActive = false;
    //Serial.println("Chord Hold: Captured chord immediately.");
  } else {
    // No notes held: start waiting for a chord (timer capture window)
    chordHoldCaptureWindowActive = false;
    chordHoldStartTime = 0;
    //Serial.println("Chord Hold: ARMED, waiting for chord input.");
  }
}

void onHoldButtonReleased() {
  chordHoldActive = false;
  chordHoldWaitingForNotes = false;
  chordHoldCount = 0;
  chordHoldCaptureWindowActive = false;
  chordHoldStartTime = 0;
  //Serial.println("Chord Hold: OFF");
}

void myNoteOn(byte channel, byte note, byte velocity) {


  numberOfNotesU++;
  numberOfNotesL++;
  prevNote = note;

  // ---- CHORD HOLD FOR POLY1/POLY2 ----
  bool polyMode = (lowerData[P_keyboardMode] == 0 || lowerData[P_keyboardMode] == 1);
  bool chordHoldIsActive = chordHoldActive && polyMode && playMode == 0;

  // Chord Hold active: play transposed chord
  if (chordHoldIsActive && chordHoldCount > 0 && !chordHoldWaitingForNotes) {
    for (int i = 0; i < chordHoldCount; i++) {
      uint8_t chordNote = note + chordHoldIntervals[i];
      int voiceNum = (lowerData[P_keyboardMode] == 0) ? getVoiceNo(-1) - 1 : getVoiceNoPoly2(-1) - 1;
      assignVoice(chordNote, velocity, voiceNum);
      voiceAssignment[chordNote] = voiceNum;
      //Serial.print("NoteOn: ");
      //Serial.println(chordNote);
    }
    return;
  }
  // ---- END CHORD HOLD ----

  int voiceNum = -1;

  switch (playMode) {

    // WHOLE MODE (No changes needed if currently working)
    case 0:
      switch (lowerData[P_keyboardMode]) {
        case 0:
          voiceNum = getVoiceNo(-1) - 1;
          assignVoice(note, velocity, voiceNum);
          break;  // Poly1
        case 1:
          voiceNum = getVoiceNoPoly2(-1) - 1;
          assignVoice(note, velocity, voiceNum);
          break;                                             // Poly2
        case 2: commandMonoNoteOn(note, velocity); break;    // Mono
        case 3: commandUnisonNoteOn(note, velocity); break;  // Unison
      }
      voiceAssignment[note] = voiceNum;
      break;

    // DUAL MODE (Explicitly corrected, place this clearly here):
    case 1:
      {
        // Lower Split
        if (lowerData[P_keyboardMode] == 1) {  // Poly2 Lower
          int lowerVoice = getLowerSplitVoicePoly2(note);
          int oldNote = voiceToNoteLower[lowerVoice];
          if (oldNote >= 0) {
            releaseVoice(oldNote, lowerVoice);
            voiceAssignmentLower[oldNote] = -1;
          }
          assignVoice(note, velocity, lowerVoice);
          voiceAssignmentLower[note] = lowerVoice;
          voiceToNoteLower[lowerVoice] = note;
        } else if (lowerData[P_keyboardMode] == 0) {  // Poly1 Lower
          int lowerVoice = getLowerSplitVoice(note);
          assignVoice(note, velocity, lowerVoice);
          voiceAssignmentLower[note] = lowerVoice;
          voiceToNoteLower[lowerVoice] = note;
        } else if (lowerData[P_keyboardMode] == 2) {
          commandMonoNoteOnLower(note, velocity, lowerData[P_NotePriority]);
        } else if (lowerData[P_keyboardMode] == 3) {
          commandUnisonNoteOnLower(note, velocity, lowerData[P_NotePriority]);
        }

        // Upper Split
        if (upperData[P_keyboardMode] == 1) {  // Poly2 Upper
          int upperVoice = getUpperSplitVoicePoly2(note);
          int oldNote = voiceToNoteUpper[upperVoice - 6];
          if (oldNote >= 0) {
            releaseVoice(oldNote, upperVoice);
            voiceAssignmentUpper[oldNote] = -1;
          }
          assignVoice(note, velocity, upperVoice);
          voiceAssignmentUpper[note] = upperVoice;
          voiceToNoteUpper[upperVoice - 6] = note;
        } else if (upperData[P_keyboardMode] == 0) {  // Poly1 Upper
          int upperVoice = getUpperSplitVoice(note);
          assignVoice(note, velocity, upperVoice);
          voiceAssignmentUpper[note] = upperVoice;
          voiceToNoteUpper[upperVoice - 6] = note;
        } else if (upperData[P_keyboardMode] == 2) {
          commandMonoNoteOnUpper(note, velocity, upperData[P_NotePriority]);
        } else if (upperData[P_keyboardMode] == 3) {
          commandUnisonNoteOnUpper(note, velocity, upperData[P_NotePriority]);
        }
      }
      break;

      // SPLIT MODE (Also explicitly corrected, place here clearly):
    case 2:  // SPLIT MODE explicitly confirmed (note-on):
      if (note < splitPoint) {
        switch (lowerData[P_keyboardMode]) {
          case 0:
            voiceNum = getLowerSplitVoice(note);
            assignVoice(note, velocity, voiceNum);
            voiceAssignmentLower[note] = voiceNum;
            voiceToNoteLower[voiceNum] = note;
            break;
          case 1:
            voiceNum = getLowerSplitVoicePoly2(note);
            assignVoice(note, velocity, voiceNum);
            voiceAssignmentLower[note] = voiceNum;
            voiceToNoteLower[voiceNum] = note;
            break;
          case 2:
            commandMonoNoteOnLower(note, velocity, lowerData[P_NotePriority]);
            break;
          case 3:
            commandUnisonNoteOnLower(note, velocity, lowerData[P_NotePriority]);
            break;
        }
      } else {
        switch (upperData[P_keyboardMode]) {
          case 0:
            voiceNum = getUpperSplitVoice(note);
            assignVoice(note, velocity, voiceNum);
            voiceAssignmentUpper[note] = voiceNum;
            voiceToNoteUpper[voiceNum - 6] = note;
            break;
          case 1:
            voiceNum = getUpperSplitVoicePoly2(note);
            assignVoice(note, velocity, voiceNum);
            voiceAssignmentUpper[note] = voiceNum;
            voiceToNoteUpper[voiceNum - 6] = note;
            break;
          case 2:
            commandMonoNoteOnUpper(note, velocity, upperData[P_NotePriority]);
            break;
          case 3:
            commandUnisonNoteOnUpper(note, velocity, upperData[P_NotePriority]);
            break;
        }
      }
      break;
  }
  if (chordHoldActive && chordHoldWaitingForNotes) {
    if (!chordHoldCaptureWindowActive) {
      chordHoldCaptureWindowActive = true;
      chordHoldStartTime = millis();
      //Serial.println("Chord Hold: Capture window started.");
    }
    // Do NOT call memorizeChordFromVoices() here; let loop() do it after window ends
  }
}

void myNoteOff(byte channel, byte note, byte velocity) {


  numberOfNotesU--;
  numberOfNotesL--;

  // ---- CHORD HOLD FOR POLY1/POLY2 ----
  bool polyMode = (lowerData[P_keyboardMode] == 0 || lowerData[P_keyboardMode] == 1);
  bool chordHoldIsActive = chordHoldActive && polyMode && playMode == 0;

  if (chordHoldIsActive && chordHoldCount > 0) {
    for (int i = 0; i < chordHoldCount; i++) {
      uint8_t chordNote = note + chordHoldIntervals[i];
      int assignedVoice = voiceAssignment[chordNote];
      if (assignedVoice >= 0) {
        releaseVoice(chordNote, assignedVoice);
        voiceAssignment[chordNote] = -1;
        //Serial.print("NoteOff: ");
        //Serial.println(chordNote);
      }
    }
    return;
  }
  // ---- END CHORD HOLD ----

  int assignedVoice = voiceAssignment[note];

  switch (playMode) {

    // WHOLE MODE corrected explicitly
    case 0:
      switch (lowerData[P_keyboardMode]) {
        case 0:
          assignedVoice = getVoiceNo(note) - 1;
          releaseVoice(note, assignedVoice);
          break;
        case 1:
          assignedVoice = getVoiceNoPoly2(note) - 1;
          releaseVoice(note, assignedVoice);
          break;
        case 2: commandMonoNoteOff(note); break;
        case 3: commandUnisonNoteOff(note); break;
      }
      break;

      // DUAL MODE corrected explicitly
    case 1:  // DUAL MODE Poly2 fix explicitly (note-off):
      {
        // Lower Split
        if (lowerData[P_keyboardMode] == 2) commandMonoNoteOffLower(note);
        else if (lowerData[P_keyboardMode] == 3) commandUnisonNoteOffLower(note);
        else {
          int lowerVoice = voiceAssignmentLower[note];
          if (lowerVoice >= 0 && lowerVoice <= 5 && voiceToNoteLower[lowerVoice] == note) {
            releaseVoice(note, lowerVoice);
            voiceAssignmentLower[note] = -1;
            voiceToNoteLower[lowerVoice] = -1;
          }
        }

        // Upper Split
        if (upperData[P_keyboardMode] == 2) commandMonoNoteOffUpper(note);
        else if (upperData[P_keyboardMode] == 3) commandUnisonNoteOffUpper(note);
        else {
          int upperVoice = voiceAssignmentUpper[note];
          if (upperVoice >= 4 && upperVoice <= 11 && voiceToNoteUpper[upperVoice - 6] == note) {
            releaseVoice(note, upperVoice);
            voiceAssignmentUpper[note] = -1;
            voiceToNoteUpper[upperVoice - 6] = -1;
          }
        }
      }
      break;

      // SPLIT MODE corrected explicitly
    case 2:  // SPLIT MODE explicitly corrected (note-off):
      {
        if (note < splitPoint) {
          if (lowerData[P_keyboardMode] == 2) {
            commandMonoNoteOffLower(note);
          } else if (lowerData[P_keyboardMode] == 3) {
            commandUnisonNoteOffLower(note);
          } else {
            int lowerVoice = voiceAssignmentLower[note];
            if (lowerVoice >= 0 && lowerVoice <= 3 && voiceToNoteLower[lowerVoice] == note) {
              releaseVoice(note, lowerVoice);
              voiceAssignmentLower[note] = -1;
              voiceToNoteLower[lowerVoice] = -1;
            }
          }
        } else {
          if (upperData[P_keyboardMode] == 2) {
            commandMonoNoteOffUpper(note);
          } else if (upperData[P_keyboardMode] == 3) {
            commandUnisonNoteOffUpper(note);
          } else {
            int upperVoice = voiceAssignmentUpper[note];
            if (upperVoice >= 6 && upperVoice <= 11 && voiceToNoteUpper[upperVoice - 6] == note) {
              releaseVoice(note, upperVoice);
              voiceAssignmentUpper[note] = -1;
              voiceToNoteUpper[upperVoice - 6] = -1;
            }
          }
        }
      }
      break;
  }
}

void commandMonoNoteOn(byte note, byte velocity) {
  notesWhole[note] = true;
  noteMsg = note;
  noteVel = velocity;
  orderIndxWhole = (orderIndxWhole + 1) % 40;
  noteOrderWhole[orderIndxWhole] = note;

  if (lowerData[P_NotePriority] == 0) commandTopNoteWhole();
  else if (lowerData[P_NotePriority] == 1) commandBottomNoteWhole();
  else commandLastNoteWhole();
}

void commandMonoNoteOff(byte note) {
  notesWhole[note] = false;
  noteMsg = note;
  commandLastNoteWhole();
}

void commandTopNoteWhole() {
  int topNote = -1;
  for (int i = 0; i < 128; i++)
    if (notesWhole[i]) topNote = i;

  if (topNote >= 0) assignVoice(topNote, noteVel, 0);
  else releaseVoice(noteMsg, 0);
}

void commandBottomNoteWhole() {
  int bottomNote = -1;
  for (int i = 127; i >= 0; i--)
    if (notesWhole[i]) bottomNote = i;

  if (bottomNote >= 0) assignVoice(bottomNote, noteVel, 0);
  else releaseVoice(noteMsg, 0);
}

void commandLastNoteWhole() {
  for (int i = 0; i < 40; i++) {
    int8_t idx = noteOrderWhole[mod(orderIndxWhole - i, 40)];
    if (notesWhole[idx]) {
      assignVoice(idx, noteVel, 0);
      return;
    }
  }
  releaseVoice(noteMsg, 0);
}

void commandUnisonNoteOn(byte note, byte velocity) {
  notesWhole[note] = true;
  noteMsg = note;
  noteVel = velocity;
  orderIndxWhole = (orderIndxWhole + 1) % 40;
  noteOrderWhole[orderIndxWhole] = note;

  if (lowerData[P_NotePriority] == 0) commandTopNoteUniWhole();
  else if (lowerData[P_NotePriority] == 1) commandBottomNoteUniWhole();
  else commandLastNoteUniWhole();
}

void commandUnisonNoteOff(byte note) {
  notesWhole[note] = false;
  noteMsg = note;
  commandLastNoteUniWhole();
}

void commandTopNoteUniWhole() {
  int topNote = -1;
  for (int i = 0; i < 128; i++)
    if (notesWhole[i]) topNote = i;
  if (topNote >= 0)
    for (int v = 0; v < 12; v++) assignVoice(topNote, noteVel, v);
  else
    for (int v = 0; v < 12; v++) releaseVoice(noteMsg, v);
}

void commandBottomNoteUniWhole() {
  int bottomNote = -1;
  for (int i = 127; i >= 0; i--)
    if (notesWhole[i]) bottomNote = i;
  if (bottomNote >= 0)
    for (int v = 0; v < 12; v++) assignVoice(bottomNote, noteVel, v);
  else
    for (int v = 0; v < 12; v++) releaseVoice(noteMsg, v);
}

void commandLastNoteUniWhole() {
  for (int i = 0; i < 40; i++) {
    int8_t idx = noteOrderWhole[mod(orderIndxWhole - i, 40)];
    if (notesWhole[idx]) {
      for (int v = 0; v < 12; v++) assignVoice(idx, noteVel, v);
      return;
    }
  }
  for (int v = 0; v < 12; v++) releaseVoice(noteMsg, v);
}


void commandMonoNoteOnUpper(byte note, byte velocity, byte priority) {
  notesUpper[note] = true;
  noteMsg = note;
  noteVel = velocity;
  orderIndxUpper = (orderIndxUpper + 1) % 40;
  noteOrderUpper[orderIndxUpper] = note;
  if (priority == 0) commandTopNoteUpper();
  else if (priority == 1) commandBottomNoteUpper();
  else commandLastNoteUpper();
}

void commandMonoNoteOffUpper(byte note) {
  notesUpper[note] = false;
  noteMsg = note;
  commandLastNoteUpper();
}

void commandMonoNoteOnLower(byte note, byte velocity, byte priority) {
  notesLower[note] = true;
  noteMsg = note;
  noteVel = velocity;
  orderIndxLower = (orderIndxLower + 1) % 40;
  noteOrderLower[orderIndxLower] = note;

  if (priority == 0) commandTopNoteLower();
  else if (priority == 1) commandBottomNoteLower();
  else commandLastNoteLower();
}

void commandMonoNoteOffLower(byte note) {
  notesLower[note] = false;
  noteMsg = note;
  commandLastNoteLower();
}

void commandUnisonNoteOnUpper(byte note, byte velocity, byte priority) {
  notesUpper[note] = true;
  noteMsg = note;                                       // explicitly set here
  noteVel = velocity;                                   // explicitly set here
  if (priority == 0) commandTopNoteUniUpper();          // Highest priority
  else if (priority == 1) commandBottomNoteUniUpper();  // Lowest priority
  else commandLastNoteUniUpper();                       // Last note priority
}

void commandUnisonNoteOffUpper(byte note) {
  notesUpper[note] = false;
  noteMsg = note;  // explicitly set here
  commandLastNoteUniUpper();
}

void commandUnisonNoteOnLower(byte note, byte velocity, byte priority) {
  notesLower[note] = true;
  noteMsg = note;                                       // explicitly set here
  noteVel = velocity;                                   // explicitly set here
  if (priority == 0) commandTopNoteUniLower();          // Highest priority
  else if (priority == 1) commandBottomNoteUniLower();  // Lowest priority
  else commandLastNoteUniLower();                       // Last note priority
}

void commandUnisonNoteOffLower(byte note) {
  notesLower[note] = false;
  noteMsg = note;  // explicitly set here
  commandLastNoteUniLower();
}

int getUpperSplitVoice(byte note) {
  for (int i = 0; i < 6; i++) {
    int idx = 6 + (upperSplitVoicePointer + i) % 6;
    if (!voiceOn[idx]) {
      upperSplitVoicePointer = (idx + 1) % 6;
      return idx;
    }
  }
  // fallback oldest (poly2 style if no voice free)
  int oldest = 6;
  unsigned long oldestTime = voices[6].timeOn;
  for (int i = 7; i < 12; i++)
    if (voices[i].timeOn < oldestTime) {
      oldest = i;
      oldestTime = voices[i].timeOn;
    }
  upperSplitVoicePointer = ((oldest - 6) + 1) % 6;
  return oldest;
}

int getLowerSplitVoice(byte note) {
  for (int i = 0; i < 6; i++) {
    int idx = (lowerSplitVoicePointer + i) % 6;
    if (!voiceOn[idx]) {
      lowerSplitVoicePointer = (idx + 1) % 6;
      return idx;
    }
  }
  int oldest = 0;
  unsigned long oldestTime = voices[0].timeOn;
  for (int i = 1; i < 6; i++)
    if (voices[i].timeOn < oldestTime) {
      oldest = i;
      oldestTime = voices[i].timeOn;
    }
  lowerSplitVoicePointer = (oldest + 1) % 6;
  return oldest;
}

int getLowerSplitVoicePoly2(byte note) {
  for (int i = 0; i < 6; i++)
    if (!voiceOn[i]) return i;

  int oldest = 0;
  unsigned long oldestTime = voices[0].timeOn;

  for (int i = 1; i < 6; i++) {
    if (voices[i].timeOn < oldestTime) {
      oldest = i;
      oldestTime = voices[i].timeOn;
    }
  }
  return oldest;
}

int getUpperSplitVoicePoly2(byte note) {
  for (int i = 6; i < 12; i++)
    if (!voiceOn[i]) return i;

  int oldest = 6;
  unsigned long oldestTime = voices[6].timeOn;

  for (int i = 7; i < 12; i++) {
    if (voices[i].timeOn < oldestTime) {
      oldest = i;
      oldestTime = voices[i].timeOn;
    }
  }
  return oldest;
}


// Leave these functions as-is
void assignVoice(byte note, byte velocity, int voiceIdx) {
  if (voiceIdx >= 0 && voiceIdx < 12) {
    voices[voiceIdx].note = note;
    voices[voiceIdx].velocity = velocity;
    voices[voiceIdx].timeOn = millis();
    voices[voiceIdx].noteOn = true;  // <-- This enables chord hold!

    if (voiceIdx < 6) {
      MIDI7.sendNoteOn(note, velocity, voiceIdx + 1);   // lower board, voices 1-6
    } else {
      MIDI8.sendNoteOn(note, velocity, voiceIdx - 5);   // upper board, voices 1-6
    }

    voiceOn[voiceIdx] = true;
  }
}

void releaseVoice(byte note, int voiceIdx) {
  if (voiceIdx >= 0 && voiceIdx < 12 && voices[voiceIdx].note == note) {
    if (voiceIdx < 6) {
      MIDI7.sendNoteOn(note, 0, voiceIdx + 1);          // lower board, voices 1-6
    } else {
      MIDI8.sendNoteOn(note, 0, voiceIdx - 5);          // upper board, voices 1-6
    }
    voices[voiceIdx].note = -1;
    voices[voiceIdx].noteOn = false;
    voiceOn[voiceIdx] = false;

    if (voiceIdx < 6) {
      voiceAssignmentLower[note] = -1;
      voiceToNoteLower[voiceIdx] = -1;
    } else {
      voiceAssignmentUpper[note] = -1;
      voiceToNoteUpper[voiceIdx - 6] = -1;
    }
  }
}

int getVoiceNoPoly2(int note) {
  voiceToReturn = -1;       // Initialize to 'null'
  earliestTime = millis();  // Initialize to now

  if (note == -1) {
    // NoteOn() - Get the oldest free voice (recent voices may still be on the release stage)
    if (voices[lastUsedVoice].note == -1) {
      return lastUsedVoice + 1;
    }

    // If the last used voice is not free or doesn't exist, check if the first voice is free
    if (voices[0].note == -1) {
      return 1;
    }

    // Find the lowest available voice for the new note
    for (int i = 0; i < NO_OF_VOICES; i++) {
      if (voices[i].note == -1) {
        return i + 1;
      }
    }

    // If no voice is available, release the oldest note
    int oldestVoice = 0;
    for (int i = 1; i < NO_OF_VOICES; i++) {
      if (voices[i].timeOn < voices[oldestVoice].timeOn) {
        oldestVoice = i;
      }
    }
    return oldestVoice + 1;
  } else {
    // NoteOff() - Get the voice number from the note
    for (int i = 0; i < NO_OF_VOICES; i++) {
      if (voices[i].note == note) {
        return i + 1;
      }
    }
  }

  // Shouldn't get here, return voice 1
  return 1;
}


int getVoiceNo(int note) {
  voiceToReturn = -1;       //Initialise to 'null'
  earliestTime = millis();  //Initialise to now
  if (note == -1) {
    //NoteOn() - Get the oldest free voice (recent voices may be still on release stage)
    for (int i = 0; i < NO_OF_VOICES; i++) {
      if (voices[i].note == -1) {
        if (voices[i].timeOn < earliestTime) {
          earliestTime = voices[i].timeOn;
          voiceToReturn = i;
        }
      }
    }
    if (voiceToReturn == -1) {
      //No free voices, need to steal oldest sounding voice
      earliestTime = millis();  //Reinitialise
      for (int i = 0; i < NO_OF_VOICES; i++) {
        if (voices[i].timeOn < earliestTime) {
          earliestTime = voices[i].timeOn;
          voiceToReturn = i;
        }
      }
    }
    return voiceToReturn + 1;
  } else {
    //NoteOff() - Get voice number from note
    for (int i = 0; i < NO_OF_VOICES; i++) {
      if (voices[i].note == note) {
        return i + 1;
      }
    }
  }
  //Shouldn't get here, return voice 1
  return 1;
}

void DinHandlePitchBend(byte channel, int pitch) {
  if (wholemode) {
    MIDI7.sendPitchBend(pitch, 9);
    MIDI8.sendPitchBend(pitch, 9);
  }
  if (dualmode) {
    MIDI7.sendPitchBend(pitch, 9);
    MIDI8.sendPitchBend(pitch, 9);
  }
  if (splitmode) {
    MIDI7.sendPitchBend(pitch, 9);
    MIDI8.sendPitchBend(pitch, 9);
  }
}

void getDelayTime() {
  delaytimeL = (lowerData[P_LFO3Delay]);
  if (delaytimeL <= 0) {
    delaytimeL = 0.1;
  }
  intervalL = (delaytimeL * 100);

  delaytimeU = (upperData[P_LFO3Delay]);
  if (delaytimeU <= 0) {
    delaytimeU = 0.1;
  }
  intervalU = (delaytimeU * 100);
}

void allNotesOff() {
  midiCCOut79(WSallNotesOff, 127);
  midiCCOut89(WSallNotesOff, 127);
}

FLASHMEM void updateLFO2Rate(boolean announce) {

  if (announce) {
    showCurrentParameterPage("LFO2 Rate", String(LFO2Ratestr) + " Hz");
    startParameterDisplay();
  }
  if (upperSW) {
    midiCCOut89(CC_LFO2_RATE, upperData[P_LFO2Rate]);
    midiCCOut(CCLFO2Rate, upperData[P_LFO2Rate]);
    midiCCOut61(CCLFO2Rate, upperData[P_LFO2Rate]);
  } else {
    midiCCOut79(CC_LFO2_RATE, lowerData[P_LFO2Rate]);
    midiCCOut(CCLFO2Rate, lowerData[P_LFO2Rate]);
    midiCCOut61(CCLFO2Rate, lowerData[P_LFO2Rate]);
    if (wholemode) {
      midiCCOut89(CC_LFO2_RATE, upperData[P_LFO2Rate]);
    }
  }
}

FLASHMEM void updatefmDepth(boolean announce) {
  if (announce) {
    showCurrentParameterPage("FM Depth", int(fmDepthstr));
    startParameterDisplay();
  }
  if (upperSW) {
    midiCCOut89(CC_LFO1_FM_DEPTH, upperData[P_fmDepth]);
    midiCCOut(CCfmDepth, upperData[P_fmDepth]);
    midiCCOut61(CCfmDepth, upperData[P_fmDepth]);
  } else {
    midiCCOut79(CC_LFO1_FM_DEPTH, lowerData[P_fmDepth]);
    midiCCOut(CCfmDepth, lowerData[P_fmDepth]);
    midiCCOut61(CCfmDepth, lowerData[P_fmDepth]);
    if (wholemode) {
      midiCCOut89(CC_LFO1_FM_DEPTH, upperData[P_fmDepth]);
    }
  }
}

FLASHMEM void updateATDepth(boolean announce) {
  if (announce) {
    showCurrentParameterPage("AT Depth", int(ATDepthstr));
    startParameterDisplay();
  }
  if (upperSW) {
    midiCCOut89(CC_AT_FM_DEPTH, upperData[P_ATDepth]);
    midiCCOut(CCATDepth, upperData[P_ATDepth]);
    midiCCOut61(CCATDepth, upperData[P_ATDepth]);
  } else {
    midiCCOut79(CC_AT_FM_DEPTH, lowerData[P_ATDepth]);
    midiCCOut(CCATDepth, lowerData[P_ATDepth]);
    midiCCOut61(CCATDepth, lowerData[P_ATDepth]);
    if (wholemode) {
      midiCCOut89(CC_AT_FM_DEPTH, upperData[P_ATDepth]);
    }
  }
}

FLASHMEM void updateosc2PW(boolean announce) {
  if (announce) {
    showCurrentParameterPage("OSC2 PW", String(osc2PWstr) + " %");
    startParameterDisplay();
  }
  if (upperSW) {
    midiCCOut89(CC_DCO2_PULSE_WIDTH, upperData[P_osc2PW]);
    midiCCOut(CCosc2PW, upperData[P_osc2PW]);
    midiCCOut61(CCosc2PW, upperData[P_osc2PW]);
  } else {
    midiCCOut79(CC_DCO2_PULSE_WIDTH, lowerData[P_osc2PW]);
    midiCCOut(CCosc2PW, lowerData[P_osc2PW]);
    midiCCOut61(CCosc2PW, lowerData[P_osc2PW]);
    if (wholemode) {
      midiCCOut89(CC_DCO2_PULSE_WIDTH, upperData[P_osc2PW]);
    }
  }
}

FLASHMEM void updateosc2PWM(boolean announce) {
  if (announce) {
    showCurrentParameterPage("OSC2 LFO PWM", int(osc2PWMstr));
    startParameterDisplay();
  }
  if (upperSW) {
    midiCCOut89(CC_DCO2_LFO2_PWM, upperData[P_osc2PWM]);
    midiCCOut(CCosc2PWM, upperData[P_osc2PWM]);
    midiCCOut61(CCosc2PWM, upperData[P_osc2PWM]);
  } else {
    midiCCOut79(CC_DCO2_LFO2_PWM, lowerData[P_osc2PWM]);
    midiCCOut(CCosc2PWM, lowerData[P_osc2PWM]);
    midiCCOut61(CCosc2PWM, lowerData[P_osc2PWM]);
    if (wholemode) {
      midiCCOut89(CC_DCO2_LFO2_PWM, upperData[P_osc2PWM]);
    }
  }
}

FLASHMEM void updateosc1PW(boolean announce) {

  if (announce) {
    showCurrentParameterPage("OSC1 PW", String(osc1PWstr) + " %");
    startParameterDisplay();
  }
  if (upperSW) {
    midiCCOut89(CC_DCO1_PULSE_WIDTH, upperData[P_osc1PW]);
    midiCCOut(CCosc1PW, upperData[P_osc1PW]);
    midiCCOut61(CCosc1PW, upperData[P_osc1PW]);
  } else {
    midiCCOut79(CC_DCO1_PULSE_WIDTH, lowerData[P_osc1PW]);
    midiCCOut(CCosc1PW, lowerData[P_osc1PW]);
    midiCCOut61(CCosc1PW, lowerData[P_osc1PW]);
    if (wholemode) {
      midiCCOut89(CC_DCO1_PULSE_WIDTH, upperData[P_osc1PW]);
    }
  }
}

FLASHMEM void updateosc1PWM(boolean announce) {
  if (announce) {
    showCurrentParameterPage("OSC1 LFO PWM", int(osc1PWMstr));
    startParameterDisplay();
  }
  if (upperSW) {
    midiCCOut89(CC_DCO1_LFO2_PWM, upperData[P_osc1PWM]);
    midiCCOut(CCosc1PWM, upperData[P_osc1PWM]);
    midiCCOut61(CCosc1PWM, upperData[P_osc1PWM]);
  } else {
    midiCCOut79(CC_DCO1_LFO2_PWM, lowerData[P_osc1PWM]);
    midiCCOut(CCosc1PWM, lowerData[P_osc1PWM]);
    midiCCOut61(CCosc1PWM, lowerData[P_osc1PWM]);
    if (wholemode) {
      midiCCOut89(CC_DCO1_LFO2_PWM, upperData[P_osc1PWM]);
    }
  }
}

FLASHMEM void updateosc1envPWM(boolean announce) {
  if (announce) {
    showCurrentParameterPage("OSC1 ENV1 PWM", int(osc1PWMstr));
    startParameterDisplay();
  }
  if (upperSW) {
    midiCCOut89(CC_ENV_DCO1_PWM, upperData[P_osc1envPWM]);
    midiCCOut(CCosc1envPWM, upperData[P_osc1envPWM]);
    midiCCOut61(CCosc1envPWM, upperData[P_osc1envPWM]);
  } else {
    midiCCOut79(CC_ENV_DCO1_PWM, lowerData[P_osc1envPWM]);
    midiCCOut(CCosc1envPWM, lowerData[P_osc1envPWM]);
    midiCCOut61(CCosc1envPWM, lowerData[P_osc1envPWM]);
    if (wholemode) {
      midiCCOut89(CC_ENV_DCO1_PWM, upperData[P_osc1envPWM]);
    }
  }
}

FLASHMEM void updateosc2envPWM(boolean announce) {
  if (announce) {
    showCurrentParameterPage("OSC1 ENV2 PWM", int(osc2PWMstr));
    startParameterDisplay();
  }
  if (upperSW) {
    midiCCOut89(CC_ENV_DCO2_PWM, upperData[P_osc2envPWM]);
    midiCCOut(CCosc2envPWM, upperData[P_osc2envPWM]);
    midiCCOut61(CCosc2envPWM, upperData[P_osc2envPWM]);
  } else {
    midiCCOut79(CC_ENV_DCO2_PWM, lowerData[P_osc2envPWM]);
    midiCCOut(CCosc2envPWM, lowerData[P_osc2envPWM]);
    midiCCOut61(CCosc2envPWM, lowerData[P_osc2envPWM]);
    if (wholemode) {
      midiCCOut89(CC_ENV_DCO2_PWM, upperData[P_osc2envPWM]);
    }
  }
}

FLASHMEM void updateosc1Range(boolean announce) {
  if (upperSW) {
    panelData[P_osc1Range] = upperData[P_osc1Range];
    if (upperData[P_osc1Range] == 2) {
      if (announce) {
        showCurrentParameterPage("Osc1 Range", String("8"));
        startParameterDisplay();
      }
      midiCCOut(CCosc1Oct, 2);
      midiCCOut89(CC_DCO1_OCTAVE, 127);
      midiCCOut62(CCosc1Oct, 2);
      mcp5.digitalWrite(DCO1_OCT_LED_RED, LOW);
      mcp5.digitalWrite(DCO1_OCT_LED_GREEN, HIGH);
    } else if (upperData[P_osc1Range] == 1) {
      if (announce) {
        showCurrentParameterPage("Osc1 Range", String("16"));
        startParameterDisplay();
      }
      midiCCOut(CCosc1Oct, 1);
      midiCCOut89(CC_DCO1_OCTAVE, 64);
      midiCCOut62(CCosc1Oct, 1);
      mcp5.digitalWrite(DCO1_OCT_LED_RED, HIGH);
      mcp5.digitalWrite(DCO1_OCT_LED_GREEN, HIGH);
    } else {
      if (announce) {
        showCurrentParameterPage("Osc1 Range", String("32"));
        startParameterDisplay();
      }
      midiCCOut(CCosc1Oct, 0);
      midiCCOut89(CC_DCO1_OCTAVE, 0);
      midiCCOut62(CCosc1Oct, 0);
      mcp5.digitalWrite(DCO1_OCT_LED_RED, HIGH);
      mcp5.digitalWrite(DCO1_OCT_LED_GREEN, LOW);
    }
  } else {
    panelData[P_osc1Range] = lowerData[P_osc1Range];
    if (lowerData[P_osc1Range] == 2) {
      if (announce) {
        showCurrentParameterPage("Osc1 Range", String("8"));
        startParameterDisplay();
      }
      midiCCOut(CCosc1Oct, 2);
      midiCCOut79(CC_DCO1_OCTAVE, 127);
      midiCCOut62(CCosc1Oct, 2);
      if (wholemode) {
        midiCCOut89(CC_DCO1_OCTAVE, 127);
      }
      mcp5.digitalWrite(DCO1_OCT_LED_RED, LOW);
      mcp5.digitalWrite(DCO1_OCT_LED_GREEN, HIGH);
    } else if (lowerData[P_osc1Range] == 1) {
      if (announce) {
        showCurrentParameterPage("Osc1 Range", String("16"));
        startParameterDisplay();
      }
      midiCCOut(CCosc1Oct, 1);
      midiCCOut79(CC_DCO1_OCTAVE, 64);
      midiCCOut62(CCosc1Oct, 1);
      if (wholemode) {
        midiCCOut89(CC_DCO1_OCTAVE, 64);
      }
      mcp5.digitalWrite(DCO1_OCT_LED_RED, HIGH);
      mcp5.digitalWrite(DCO1_OCT_LED_GREEN, HIGH);
    } else {
      if (announce) {
        showCurrentParameterPage("Osc1 Range", String("32"));
        startParameterDisplay();
      }
      midiCCOut(CCosc1Oct, 0);
      midiCCOut79(CC_DCO1_OCTAVE, 0);
      midiCCOut62(CCosc1Oct, 0);
      if (wholemode) {
        midiCCOut89(CC_DCO1_OCTAVE, 0);
      }
      mcp5.digitalWrite(DCO1_OCT_LED_RED, HIGH);
      mcp5.digitalWrite(DCO1_OCT_LED_GREEN, LOW);
    }
  }
}

FLASHMEM void updateosc2Range(boolean announce) {
  if (upperSW) {
    panelData[P_osc2Range] = upperData[P_osc2Range];
    if (upperData[P_osc2Range] == 2) {
      if (announce) {
        showCurrentParameterPage("Osc2 Range", String("8"));
        startParameterDisplay();
      }
      midiCCOut89(CC_DCO2_OCTAVE, 127);
      midiCCOut62(CCosc2Oct, 2);
      midiCCOut(CCosc2Oct, 2);
      mcp7.digitalWrite(DCO2_OCT_LED_RED, LOW);
      mcp7.digitalWrite(DCO2_OCT_LED_GREEN, HIGH);
    } else if (upperData[P_osc2Range] == 1) {
      if (announce) {
        showCurrentParameterPage("Osc2 Range", String("16"));
        startParameterDisplay();
      }
      midiCCOut89(CC_DCO2_OCTAVE, 64);
      midiCCOut62(CCosc2Oct, 1);
      midiCCOut(CCosc2Oct, 1);
      mcp7.digitalWrite(DCO2_OCT_LED_RED, HIGH);
      mcp7.digitalWrite(DCO2_OCT_LED_GREEN, HIGH);
    } else {
      if (announce) {
        showCurrentParameterPage("Osc2 Range", String("32"));
        startParameterDisplay();
      }
      midiCCOut(CCosc2Oct, 0);
      midiCCOut89(CC_DCO2_OCTAVE, 0);
      midiCCOut62(CCosc2Oct, 0);
      mcp7.digitalWrite(DCO2_OCT_LED_RED, HIGH);
      mcp7.digitalWrite(DCO2_OCT_LED_GREEN, LOW);
    }
  } else {
    panelData[P_osc2Range] = lowerData[P_osc2Range];
    if (lowerData[P_osc2Range] == 2) {
      if (announce) {
        showCurrentParameterPage("Osc2 Range", String("8"));
        startParameterDisplay();
      }
      midiCCOut(CCosc2Oct, 2);
      midiCCOut79(CC_DCO2_OCTAVE, 127);
      midiCCOut62(CCosc2Oct, 2);
      if (wholemode) {
        midiCCOut89(CC_DCO2_OCTAVE, 127);
      }
      mcp7.digitalWrite(DCO2_OCT_LED_RED, LOW);
      mcp7.digitalWrite(DCO2_OCT_LED_GREEN, HIGH);
    } else if (lowerData[P_osc2Range] == 1) {
      if (announce) {
        showCurrentParameterPage("Osc2 Range", String("16"));
        startParameterDisplay();
      }
      midiCCOut(CCosc2Oct, 1);
      midiCCOut79(CC_DCO2_OCTAVE, 64);
      midiCCOut62(CCosc2Oct, 1);
      if (wholemode) {
        midiCCOut89(CC_DCO2_OCTAVE, 64);
      }
      mcp7.digitalWrite(DCO2_OCT_LED_RED, HIGH);
      mcp7.digitalWrite(DCO2_OCT_LED_GREEN, HIGH);
    } else {
      if (announce) {
        showCurrentParameterPage("Osc2 Range", String("32"));
        startParameterDisplay();
      }
      midiCCOut(CCosc2Oct, 0);
      midiCCOut79(CC_DCO2_OCTAVE, 0);
      midiCCOut62(CCosc2Oct, 0);
      if (wholemode) {
        midiCCOut89(CC_DCO2_OCTAVE, 0);
      }
      mcp7.digitalWrite(DCO2_OCT_LED_RED, HIGH);
      mcp7.digitalWrite(DCO2_OCT_LED_GREEN, LOW);
    }
  }
}

FLASHMEM void updateglideTime(boolean announce) {
  if (announce) {
    showCurrentParameterPage("Glide Time", String(glideTimestr * 10) + " Seconds");
    startParameterDisplay();
  }
  if (upperSW) {
    midiCCOut89(CC_PORTAMENTO_TIME, upperData[P_glideTime]);
    midiCCOut(CCglideTime, upperData[P_glideTime]);
    midiCCOut61(CCglideTime, upperData[P_glideTime]);
  } else {
    midiCCOut79(CC_PORTAMENTO_TIME, lowerData[P_glideTime]);
    midiCCOut(CCglideTime, lowerData[P_glideTime]);
    midiCCOut61(CCglideTime, lowerData[P_glideTime]);
    if (wholemode) {
      midiCCOut89(CC_PORTAMENTO_TIME, upperData[P_glideTime]);
    }
  }
}

FLASHMEM void updateosc2Detune(boolean announce) {
  if (announce) {
    showCurrentParameterPage("OSC2 Detune", String(osc2Detunestr));
    startParameterDisplay();
  }
  if (upperSW) {
    midiCCOut89(CC_DCO2_DETUNE, upperData[P_osc2Detune]);
    midiCCOut(CCosc2Detune, upperData[P_osc2Detune]);
    midiCCOut61(CCosc2Detune, upperData[P_osc2Detune]);
  } else {
    midiCCOut79(CC_DCO2_DETUNE, lowerData[P_osc2Detune]);
    midiCCOut(CCosc2Detune, lowerData[P_osc2Detune]);
    midiCCOut61(CCosc2Detune, lowerData[P_osc2Detune]);
    if (wholemode) {
      midiCCOut89(CC_DCO2_DETUNE, upperData[P_osc2Detune]);
    }
  }
}

FLASHMEM void updateosc2Interval(boolean announce) {
  if (announce) {
    showCurrentParameterPage("OSC2 Interval", String(osc2Intervalstr));
    startParameterDisplay();
  }
  if (upperSW) {
    midiCCOut89(CC_DCO2_INTERVAL, upperData[P_osc2Interval]);
    midiCCOut(CCosc2Interval, upperData[P_osc2Interval]);
    midiCCOut61(CCosc2Interval, upperData[P_osc2Interval]);
  } else {
    midiCCOut79(CC_DCO2_INTERVAL, lowerData[P_osc2Interval]);
    midiCCOut(CCosc2Interval, lowerData[P_osc2Interval]);
    midiCCOut61(CCosc2Interval, lowerData[P_osc2Interval]);
    if (wholemode) {
      midiCCOut89(CC_DCO2_INTERVAL, upperData[P_osc2Interval]);
    }
  }
}

FLASHMEM void updatenoiseLevel(boolean announce) {
  if (announce) {
    showCurrentParameterPage("Noise Level", String(noiseLevelstr));
    startParameterDisplay();
  }
  if (upperSW) {
    midiCCOut810(VB_NOISE_LEVEL, upperData[P_noiseLevel]);
    midiCCOut(CCnoiseLevel, upperData[P_noiseLevel]);
    midiCCOut61(CCnoiseLevel, upperData[P_noiseLevel]);
  } else {
    midiCCOut710(VB_NOISE_LEVEL, lowerData[P_noiseLevel]);
    midiCCOut(CCnoiseLevel, lowerData[P_noiseLevel]);
    midiCCOut61(CCnoiseLevel, lowerData[P_noiseLevel]);
    if (wholemode) {
      midiCCOut810(VB_NOISE_LEVEL, upperData[P_noiseLevel]);
    }
  }
}

FLASHMEM void updateOsc2SawLevel(boolean announce) {
  if (announce) {
    showCurrentParameterPage("OSC2 Saw", int(osc2SawLevelstr));
    startParameterDisplay();
  }
  if (upperSW) {
    midiCCOut89(CC_DCO2_SAW_LEVEL, upperData[P_osc2SawLevel]);
    midiCCOut(CCosc2SawLevel, upperData[P_osc2SawLevel]);
    midiCCOut61(CCosc2SawLevel, upperData[P_osc2SawLevel]);
  } else {
    midiCCOut79(CC_DCO2_SAW_LEVEL, lowerData[P_osc2SawLevel]);
    midiCCOut(CCosc2SawLevel, lowerData[P_osc2SawLevel]);
    midiCCOut61(CCosc2SawLevel, lowerData[P_osc2SawLevel]);
    if (wholemode) {
      midiCCOut89(CC_DCO2_SAW_LEVEL, upperData[P_osc2SawLevel]);
    }
  }
}

FLASHMEM void updateOsc1SawLevel(boolean announce) {
  if (announce) {
    showCurrentParameterPage("OSC1 Saw", int(osc1SawLevelstr));
    startParameterDisplay();
  }
  if (upperSW) {
    midiCCOut89(CC_DCO1_SAW_LEVEL, upperData[P_osc1SawLevel]);
    midiCCOut(CCosc1SawLevel, upperData[P_osc1SawLevel]);
    midiCCOut61(CCosc1SawLevel, upperData[P_osc1SawLevel]);
  } else {
    midiCCOut79(CC_DCO1_SAW_LEVEL, lowerData[P_osc1SawLevel]);
    midiCCOut(CCosc1SawLevel, lowerData[P_osc1SawLevel]);
    midiCCOut61(CCosc1SawLevel, lowerData[P_osc1SawLevel]);
    if (wholemode) {
      midiCCOut89(CC_DCO1_SAW_LEVEL, upperData[P_osc1SawLevel]);
    }
  }
}

FLASHMEM void updateOsc2PulseLevel(boolean announce) {
  if (announce) {
    showCurrentParameterPage("OSC2 Pulse", int(osc2PulseLevelstr));
    startParameterDisplay();
  }
  if (upperSW) {
    midiCCOut89(CC_DCO2_PULSE_LEVEL, upperData[P_osc2PulseLevel]);
    midiCCOut(CCosc2PulseLevel, upperData[P_osc2PulseLevel]);
    midiCCOut61(CCosc2PulseLevel, upperData[P_osc2PulseLevel]);
  } else {
    midiCCOut79(CC_DCO2_PULSE_LEVEL, lowerData[P_osc2PulseLevel]);
    midiCCOut(CCosc2PulseLevel, lowerData[P_osc2PulseLevel]);
    midiCCOut61(CCosc2PulseLevel, lowerData[P_osc2PulseLevel]);
    if (wholemode) {
      midiCCOut89(CC_DCO2_PULSE_LEVEL, upperData[P_osc2PulseLevel]);
    }
  }
}

FLASHMEM void updateOsc1PulseLevel(boolean announce) {
  if (announce) {
    showCurrentParameterPage("OSC1 Pulse", int(osc1PulseLevelstr));
    startParameterDisplay();
  }
  if (upperSW) {
    midiCCOut89(CC_DCO1_PULSE_LEVEL, upperData[P_osc1PulseLevel]);
    midiCCOut(CCosc1PulseLevel, upperData[P_osc1PulseLevel]);
    midiCCOut61(CCosc1PulseLevel, upperData[P_osc1PulseLevel]);
  } else {
    midiCCOut79(CC_DCO1_PULSE_LEVEL, lowerData[P_osc1PulseLevel]);
    midiCCOut(CCosc1PulseLevel, lowerData[P_osc1PulseLevel]);
    midiCCOut61(CCosc1PulseLevel, lowerData[P_osc1PulseLevel]);
    if (wholemode) {
      midiCCOut89(CC_DCO1_PULSE_LEVEL, upperData[P_osc1PulseLevel]);
    }
  }
}

FLASHMEM void updateOsc2TriangleLevel(boolean announce) {
  if (announce) {
    showCurrentParameterPage("OSC2 Triangle", int(osc2TriangleLevelstr));
    startParameterDisplay();
  }
  if (upperSW) {
    midiCCOut89(CC_DCO2_SUB_LEVEL, upperData[P_osc2TriangleLevel]);
    midiCCOut(CCosc2TriangleLevel, upperData[P_osc2TriangleLevel]);
    midiCCOut61(CCosc2TriangleLevel, upperData[P_osc2TriangleLevel]);
  } else {
    midiCCOut79(CC_DCO2_SUB_LEVEL, lowerData[P_osc2TriangleLevel]);
    midiCCOut(CCosc2TriangleLevel, lowerData[P_osc2TriangleLevel]);
    midiCCOut61(CCosc2TriangleLevel, lowerData[P_osc2TriangleLevel]);
    if (wholemode) {
      midiCCOut89(CC_DCO2_SUB_LEVEL, upperData[P_osc2TriangleLevel]);
    }
  }
}

FLASHMEM void updateOsc1SubLevel(boolean announce) {
  if (announce) {
    showCurrentParameterPage("OSC1 Sub", int(osc1SubLevelstr));
    startParameterDisplay();
  }
  if (upperSW) {
    midiCCOut89(CC_DCO1_TRI_LEVEL, upperData[P_osc1SubLevel]);
    midiCCOut(CCosc1SubLevel, upperData[P_osc1SubLevel]);
    midiCCOut61(CCosc1SubLevel, upperData[P_osc1SubLevel]);
  } else {
    midiCCOut79(CC_DCO1_TRI_LEVEL, lowerData[P_osc1SubLevel]);
    midiCCOut(CCosc1SubLevel, lowerData[P_osc1SubLevel]);
    midiCCOut61(CCosc1SubLevel, lowerData[P_osc1SubLevel]);
    if (wholemode) {
      midiCCOut89(CC_DCO1_TRI_LEVEL, upperData[P_osc1SubLevel]);
    }
  }
}

FLASHMEM void updateOsc2EnvDepth(boolean announce) {
  if (announce) {
    showCurrentParameterPage("OSC2 Pitch Env", int(osc2envDepthstr));
    startParameterDisplay();
  }
  if (upperSW) {
    midiCCOut89(CC_XMOD_DEPTH, upperData[P_osc2envDepth]);
    midiCCOut(CCosc2EnvDepth, upperData[P_osc2envDepth]);
    midiCCOut61(CCosc2EnvDepth, upperData[P_osc2envDepth]);
  } else {
    midiCCOut79(CC_XMOD_DEPTH, lowerData[P_osc2envDepth]);
    midiCCOut(CCosc2EnvDepth, lowerData[P_osc2envDepth]);
    midiCCOut61(CCosc2EnvDepth, lowerData[P_osc2envDepth]);
    if (wholemode) {
      midiCCOut89(CC_XMOD_DEPTH, upperData[P_osc2envDepth]);
    }
  }
}

FLASHMEM void updateamDepth(boolean announce) {
  if (announce) {
    showCurrentParameterPage("AM Depth", int(amDepthstr));
    startParameterDisplay();
  }
  if (upperSW) {
    midiCCOut810(VB_AMP_LFO3, upperData[P_amDepth]);
    midiCCOut(CCamDepth, upperData[P_amDepth]);
    midiCCOut61(CCamDepth, upperData[P_amDepth]);
  } else {
    midiCCOut710(VB_AMP_LFO3, lowerData[P_amDepth]);
    midiCCOut(CCamDepth, lowerData[P_amDepth]);
    midiCCOut61(CCamDepth, lowerData[P_amDepth]);
    if (wholemode) {
      midiCCOut810(VB_AMP_LFO3, upperData[P_amDepth]);
    }
  }
}

FLASHMEM void updateFilterCutoff(boolean announce) {
  if (announce) {
    showCurrentParameterPage("Cutoff", String(filterCutoffstr) + " Hz");
    startParameterDisplay();
  }
  if (upperSW) {
    midiCCOut810(VB_FILTER_CUTOFF, upperData[P_filterCutoff]);
    midiCCOut(CCfilterCutoff, upperData[P_filterCutoff]);
    midiCCOut61(CCfilterCutoff, upperData[P_filterCutoff]);
  } else {
    midiCCOut710(VB_FILTER_CUTOFF, lowerData[P_filterCutoff]);
    midiCCOut(CCfilterCutoff, lowerData[P_filterCutoff]);
    midiCCOut61(CCfilterCutoff, lowerData[P_filterCutoff]);
    if (wholemode) {
      midiCCOut810(VB_FILTER_CUTOFF, upperData[P_filterCutoff]);
    }
  }
}

FLASHMEM void updatefilterLFO(boolean announce) {
  if (announce) {
    showCurrentParameterPage("TM depth", int(filterLFOstr));
    startParameterDisplay();
  }
  if (upperSW) {
    midiCCOut810(VB_FILTER_LFO3, upperData[P_filterLFO]);
    midiCCOut(CCfilterLFO, upperData[P_filterLFO]);
    midiCCOut61(CCfilterLFO, upperData[P_filterLFO]);
  } else {
    midiCCOut710(VB_FILTER_LFO3, lowerData[P_filterLFO]);
    midiCCOut(CCfilterLFO, lowerData[P_filterLFO]);
    midiCCOut61(CCfilterLFO, lowerData[P_filterLFO]);
    if (wholemode) {
      midiCCOut810(VB_FILTER_LFO3, upperData[P_filterLFO]);
    }
  }
}

FLASHMEM void updatefilterRes(boolean announce) {
  if (announce) {
    showCurrentParameterPage("Resonance", int(filterResstr));
    startParameterDisplay();
  }
  if (upperSW) {
    midiCCOut810(VB_FILTER_RES, upperData[P_filterRes]);
    midiCCOut(CCfilterRes, upperData[P_filterRes]);
    midiCCOut61(CCfilterRes, upperData[P_filterRes]);
  } else {
    midiCCOut710(VB_FILTER_RES, lowerData[P_filterRes]);
    midiCCOut(CCfilterRes, lowerData[P_filterRes]);
    midiCCOut61(CCfilterRes, lowerData[P_filterRes]);
    if (wholemode) {
      midiCCOut810(VB_FILTER_RES, upperData[P_filterRes]);
    }
  }
}

FLASHMEM void updateFilterType(boolean announce) {
  if (upperSW) {
    switch (upperData[P_filterType]) {
      case 0:
        if (upperData[P_filterPoleSW] == 1) {
          if (announce) {
            showCurrentParameterPage("Filter Type", String("3P LowPass"));
            startParameterDisplay();
          }
        } else {
          if (announce) {
            showCurrentParameterPage("Filter Type", String("4P LowPass"));
            startParameterDisplay();
          }
        }
        midiCCOut62(CCfilterType, 0);
        midiCCOut(CCfilterType, 0);
        midiCCOut810(VB_FILTER_A, 0);
        midiCCOut810(VB_FILTER_B, 0);
        midiCCOut810(VB_FILTER_C, 0);
        break;

      case 1:
        if (upperData[P_filterPoleSW] == 1) {
          if (announce) {
            showCurrentParameterPage("Filter Type", String("1P LowPass"));
            startParameterDisplay();
          }
        } else {
          if (announce) {
            showCurrentParameterPage("Filter Type", String("2P LowPass"));
            startParameterDisplay();
          }
        }
        midiCCOut62(CCfilterType, 1);
        midiCCOut(CCfilterType, 1);
        midiCCOut810(VB_FILTER_A, 127);
        midiCCOut810(VB_FILTER_B, 0);
        midiCCOut810(VB_FILTER_C, 0);
        break;

      case 2:
        if (upperData[P_filterPoleSW] == 1) {
          if (announce) {
            showCurrentParameterPage("Filter Type", String("3P HP + 1P LP"));
            startParameterDisplay();
          }
        } else {
          if (announce) {
            showCurrentParameterPage("Filter Type", String("4P HighPass"));
            startParameterDisplay();
          }
        }
        midiCCOut62(CCfilterType, 2);
        midiCCOut(CCfilterType, 2);
        midiCCOut810(VB_FILTER_A, 0);
        midiCCOut810(VB_FILTER_B, 127);
        midiCCOut810(VB_FILTER_C, 0);
        break;

      case 3:
        if (upperData[P_filterPoleSW] == 1) {
          if (announce) {
            showCurrentParameterPage("Filter Type", String("1P HP + 1P LP"));
            startParameterDisplay();
          }
        } else {
          if (announce) {
            showCurrentParameterPage("Filter Type", String("2P HighPass"));
            startParameterDisplay();
          }
        }
        midiCCOut62(CCfilterType, 3);
        midiCCOut(CCfilterType, 3);
        midiCCOut810(VB_FILTER_A, 127);
        midiCCOut810(VB_FILTER_B, 127);
        midiCCOut810(VB_FILTER_C, 0);
        break;

      case 4:
        if (upperData[P_filterPoleSW] == 1) {
          if (announce) {
            showCurrentParameterPage("Filter Type", String("2P HP + 1P LP"));
            startParameterDisplay();
          }
        } else {
          if (announce) {
            showCurrentParameterPage("Filter Type", String("4P BandPass"));
            startParameterDisplay();
          }
        }
        midiCCOut62(CCfilterType, 4);
        midiCCOut(CCfilterType, 4);
        midiCCOut810(VB_FILTER_A, 0);
        midiCCOut810(VB_FILTER_B, 0);
        midiCCOut810(VB_FILTER_C, 127);
        break;

      case 5:
        if (upperData[P_filterPoleSW] == 1) {
          if (announce) {
            showCurrentParameterPage("Filter Type", String("2P BP + 1P LP"));
            startParameterDisplay();
          }
        } else {
          if (announce) {
            showCurrentParameterPage("Filter Type", String("2P BandPass"));
            startParameterDisplay();
          }
        }
        midiCCOut62(CCfilterType, 5);
        midiCCOut(CCfilterType, 5);
        midiCCOut810(VB_FILTER_A, 127);
        midiCCOut810(VB_FILTER_B, 0);
        midiCCOut810(VB_FILTER_C, 127);
        break;

      case 6:
        if (upperData[P_filterPoleSW] == 1) {
          if (announce) {
            showCurrentParameterPage("Filter Type", String("3P AP + 1P LP"));
            startParameterDisplay();
          }
        } else {
          if (announce) {
            showCurrentParameterPage("Filter Type", String("3P AllPass"));
            startParameterDisplay();
          }
        }
        midiCCOut62(CCfilterType, 6);
        midiCCOut(CCfilterType, 6);
        midiCCOut810(VB_FILTER_A, 0);
        midiCCOut810(VB_FILTER_B, 127);
        midiCCOut810(VB_FILTER_C, 127);
        break;

      case 7:
        if (upperData[P_filterPoleSW] == 1) {
          if (announce) {
            showCurrentParameterPage("Filter Type", String("2P Notch + LP"));
            startParameterDisplay();
          }
        } else {
          if (announce) {
            showCurrentParameterPage("Filter Type", String("Notch"));
            startParameterDisplay();
          }
        }
        midiCCOut62(CCfilterType, 7);
        midiCCOut(CCfilterType, 7);
        midiCCOut810(VB_FILTER_A, 127);
        midiCCOut810(VB_FILTER_B, 127);
        midiCCOut810(VB_FILTER_C, 127);
        break;
    }
  } else {
    switch (lowerData[P_filterType]) {
      case 0:
        if (lowerData[P_filterPoleSW] == 1) {
          if (announce) {
            showCurrentParameterPage("Filter Type", String("3P LowPass"));
            startParameterDisplay();
          }
        } else {
          if (announce) {
            showCurrentParameterPage("Filter Type", String("4P LowPass"));
            startParameterDisplay();
          }
        }
        midiCCOut62(CCfilterType, 0);
        midiCCOut(CCfilterType, 0);
        midiCCOut710(VB_FILTER_A, 0);
        midiCCOut710(VB_FILTER_B, 0);
        midiCCOut710(VB_FILTER_C, 0);
        if (wholemode) {
          midiCCOut810(VB_FILTER_A, 0);
          midiCCOut810(VB_FILTER_B, 0);
          midiCCOut810(VB_FILTER_C, 0);
        }
        break;

      case 1:
        if (lowerData[P_filterPoleSW] == 1) {
          if (announce) {
            showCurrentParameterPage("Filter Type", String("1P LowPass"));
            startParameterDisplay();
          }
        } else {
          if (announce) {
            showCurrentParameterPage("Filter Type", String("2P LowPass"));
            startParameterDisplay();
          }
        }
        midiCCOut62(CCfilterType, 1);
        midiCCOut(CCfilterType, 1);
        midiCCOut710(VB_FILTER_A, 127);
        midiCCOut710(VB_FILTER_B, 0);
        midiCCOut710(VB_FILTER_C, 0);
        if (wholemode) {
          midiCCOut810(VB_FILTER_A, 127);
          midiCCOut810(VB_FILTER_B, 0);
          midiCCOut810(VB_FILTER_C, 0);
        }
        break;

      case 2:
        if (lowerData[P_filterPoleSW] == 1) {
          if (announce) {
            showCurrentParameterPage("Filter Type", String("3P HP + 1P LP"));
            startParameterDisplay();
          }
        } else {
          if (announce) {
            showCurrentParameterPage("Filter Type", String("4P HighPass"));
            startParameterDisplay();
          }
        }
        midiCCOut62(CCfilterType, 2);
        midiCCOut(CCfilterType, 2);
        midiCCOut710(VB_FILTER_A, 0);
        midiCCOut710(VB_FILTER_B, 127);
        midiCCOut710(VB_FILTER_C, 0);
        if (wholemode) {
          midiCCOut810(VB_FILTER_A, 0);
          midiCCOut810(VB_FILTER_B, 127);
          midiCCOut810(VB_FILTER_C, 0);
        }
        break;

      case 3:
        if (lowerData[P_filterPoleSW] == 1) {
          if (announce) {
            showCurrentParameterPage("Filter Type", String("1P HP + 1P LP"));
            startParameterDisplay();
          }
        } else {
          if (announce) {
            showCurrentParameterPage("Filter Type", String("2P HighPass"));
            startParameterDisplay();
          }
        }
        midiCCOut62(CCfilterType, 3);
        midiCCOut(CCfilterType, 3);
        midiCCOut710(VB_FILTER_A, 127);
        midiCCOut710(VB_FILTER_B, 127);
        midiCCOut710(VB_FILTER_C, 0);
        if (wholemode) {
          midiCCOut810(VB_FILTER_A, 127);
          midiCCOut810(VB_FILTER_B, 127);
          midiCCOut810(VB_FILTER_C, 0);
        }
        break;

      case 4:
        if (lowerData[P_filterPoleSW] == 1) {
          if (announce) {
            showCurrentParameterPage("Filter Type", String("2P HP + 1P LP"));
            startParameterDisplay();
          }
        } else {
          if (announce) {
            showCurrentParameterPage("Filter Type", String("4P BandPass"));
            startParameterDisplay();
          }
        }
        midiCCOut62(CCfilterType, 4);
        midiCCOut(CCfilterType, 4);
        midiCCOut710(VB_FILTER_A, 0);
        midiCCOut710(VB_FILTER_B, 0);
        midiCCOut710(VB_FILTER_C, 127);
        if (wholemode) {
          midiCCOut810(VB_FILTER_A, 0);
          midiCCOut810(VB_FILTER_B, 0);
          midiCCOut810(VB_FILTER_C, 127);
        }
        break;

      case 5:
        if (lowerData[P_filterPoleSW] == 1) {
          if (announce) {
            showCurrentParameterPage("Filter Type", String("2P BP + 1P LP"));
            startParameterDisplay();
          }
        } else {
          if (announce) {
            showCurrentParameterPage("Filter Type", String("2P BandPass"));
            startParameterDisplay();
          }
        }
        midiCCOut62(CCfilterType, 5);
        midiCCOut(CCfilterType, 5);
        midiCCOut710(VB_FILTER_A, 127);
        midiCCOut710(VB_FILTER_B, 0);
        midiCCOut710(VB_FILTER_C, 127);
        if (wholemode) {
          midiCCOut810(VB_FILTER_A, 127);
          midiCCOut810(VB_FILTER_B, 0);
          midiCCOut810(VB_FILTER_C, 127);
        }
        break;


      case 6:
        if (lowerData[P_filterPoleSW] == 1) {
          if (announce) {
            showCurrentParameterPage("Filter Type", String("3P AP + 1P LP"));
            startParameterDisplay();
          }
        } else {
          if (announce) {
            showCurrentParameterPage("Filter Type", String("3P AllPass"));
            startParameterDisplay();
          }
        }
        midiCCOut62(CCfilterType, 6);
        midiCCOut(CCfilterType, 6);
        midiCCOut710(VB_FILTER_A, 0);
        midiCCOut710(VB_FILTER_B, 127);
        midiCCOut710(VB_FILTER_C, 127);
        if (wholemode) {
          midiCCOut810(VB_FILTER_A, 0);
          midiCCOut810(VB_FILTER_B, 127);
          midiCCOut810(VB_FILTER_C, 127);
        }
        break;

      case 7:
        if (lowerData[P_filterPoleSW] == 1) {
          if (announce) {
            showCurrentParameterPage("Filter Type", String("2P Notch + LP"));
            startParameterDisplay();
          }
        } else {
          if (announce) {
            showCurrentParameterPage("Filter Type", String("Notch"));
            startParameterDisplay();
          }
        }
        midiCCOut62(CCfilterType, 7);
        midiCCOut(CCfilterType, 7);
        midiCCOut710(VB_FILTER_A, 127);
        midiCCOut710(VB_FILTER_B, 127);
        midiCCOut710(VB_FILTER_C, 127);
        if (wholemode) {
          midiCCOut810(VB_FILTER_A, 127);
          midiCCOut810(VB_FILTER_B, 127);
          midiCCOut810(VB_FILTER_C, 127);
        }
        break;
    }
  }
}

FLASHMEM void updatefilterEGlevel(boolean announce) {
  if (announce) {
    showCurrentParameterPage("EG Depth", int(filterEGlevelstr));
    startParameterDisplay();
  }
  if (upperSW) {
    midiCCOut810(VB_EG_DEPTH, upperData[P_filterEGlevel]);
    midiCCOut(CCfilterEGlevel, upperData[P_filterEGlevel]);
    midiCCOut61(CCfilterEGlevel, upperData[P_filterEGlevel]);
  } else {
    midiCCOut710(VB_EG_DEPTH, lowerData[P_filterEGlevel]);
    midiCCOut(CCfilterEGlevel, lowerData[P_filterEGlevel]);
    midiCCOut61(CCfilterEGlevel, lowerData[P_filterEGlevel]);
    if (wholemode) {
      midiCCOut810(VB_EG_DEPTH, upperData[P_filterEGlevel]);
    }
  }
}

FLASHMEM void updatekeytrack(boolean announce) {
  if (announce) {
    showCurrentParameterPage("Keytrack", int(keytrackstr));
    startParameterDisplay();
  }
  if (upperSW) {
    midiCCOut89(CC_KEYTRACK_DEPTH, upperData[P_keytrack]);
    midiCCOut(CCkeyTrack, upperData[P_keytrack]);
    midiCCOut61(CCkeyTrack, upperData[P_keytrack]);
  } else {
    midiCCOut79(CC_KEYTRACK_DEPTH, lowerData[P_keytrack]);
    midiCCOut(CCkeyTrack, lowerData[P_keytrack]);
    midiCCOut61(CCkeyTrack, lowerData[P_keytrack]);
    if (wholemode) {
      midiCCOut89(CC_KEYTRACK_DEPTH, upperData[P_keytrack]);
    }
  }
}

FLASHMEM void updatearpRate(boolean announce) {

  if (announce) {
    showCurrentParameterPage("Arp Rate", String(arpRatestr) + " Hz");
    startParameterDisplay();
  }
  midiCCOut(CCLFO1Rate, lowerData[P_arpRate]);
  midiCCOut61(CCLFO1Rate, lowerData[P_arpRate]);
}

FLASHMEM void updateLFO1Rate(boolean announce) {

  if (announce) {
    showCurrentParameterPage("LFO1 Rate", String(LFO1Ratestr) + " Hz");
    startParameterDisplay();
  }
  if (upperSW) {
    midiCCOut89(CC_LFO1_RATE, upperData[P_LFO1Rate]);
    midiCCOut(CCLFO1Rate, upperData[P_LFO1Rate]);
    midiCCOut61(CCLFO1Rate, upperData[P_LFO1Rate]);
  } else {
    midiCCOut79(CC_LFO1_RATE, lowerData[P_LFO1Rate]);
    midiCCOut(CCLFO1Rate, lowerData[P_LFO1Rate]);
    midiCCOut61(CCLFO1Rate, lowerData[P_LFO1Rate]);
    if (wholemode) {
      midiCCOut89(CC_LFO1_RATE, upperData[P_LFO1Rate]);
    }
  }
}

FLASHMEM void updateLFO1Delay(boolean announce) {
  if (announce) {
    showCurrentParameterPage("LFO1 Delay", String(LFO1Delaystr));
    startParameterDisplay();
  }
  if (upperSW) {
    midiCCOut89(CC_LFO1_DELAY_TIME, upperData[P_LFO1Delay]);
    midiCCOut(CCLFO1Delay, upperData[P_LFO1Delay]);
    midiCCOut61(CCLFO1Delay, upperData[P_LFO1Delay]);
  } else {
    midiCCOut79(CC_LFO1_DELAY_TIME, lowerData[P_LFO1Delay]);
    midiCCOut(CCLFO1Delay, lowerData[P_LFO1Delay]);
    midiCCOut61(CCLFO1Delay, lowerData[P_LFO1Delay]);
    if (wholemode) {
      midiCCOut89(CC_LFO1_DELAY_TIME, upperData[P_LFO1Delay]);
    }
  }
}

FLASHMEM void updateLFO1Slope(boolean announce) {
  if (announce) {
    showCurrentParameterPage("LFO1 Slope", String(LFO1Slopestr));
    startParameterDisplay();
  }
  if (upperSW) {
    midiCCOut89(CC_LFO1_DELAY_RAMP, upperData[P_LFO1Slope]);
    midiCCOut(CCLFO1Slope, upperData[P_LFO1Slope]);
    midiCCOut61(CCLFO1Slope, upperData[P_LFO1Slope]);
  } else {
    midiCCOut79(CC_LFO1_DELAY_RAMP, lowerData[P_LFO1Slope]);
    midiCCOut(CCLFO1Slope, lowerData[P_LFO1Slope]);
    midiCCOut61(CCLFO1Slope, lowerData[P_LFO1Slope]);
    if (wholemode) {
      midiCCOut89(CC_LFO1_DELAY_RAMP, upperData[P_LFO1Slope]);
    }
  }
}

FLASHMEM void updateLFO3Rate(boolean announce) {

  if (announce) {
    showCurrentParameterPage("LFO3 Rate", String(LFO3Ratestr) + " Hz");
    startParameterDisplay();
  }
  if (upperSW) {
    midiCCOut810(VB_LFO3_RATE, upperData[P_LFO3Rate]);
    midiCCOut(CCLFO3Rate, upperData[P_LFO3Rate]);
    midiCCOut61(CCLFO3Rate, upperData[P_LFO3Rate]);
  } else {
    midiCCOut710(VB_LFO3_RATE, lowerData[P_LFO3Rate]);
    midiCCOut(CCLFO3Rate, lowerData[P_LFO3Rate]);
    midiCCOut61(CCLFO3Rate, lowerData[P_LFO3Rate]);
    if (wholemode) {
      midiCCOut810(VB_LFO3_RATE, upperData[P_LFO3Rate]);
    }
  }
}

FLASHMEM void updateLFO3Delay(boolean announce) {
  if (announce) {
    showCurrentParameterPage("LFO3 Delay", String(LFO3Delaystr));
    startParameterDisplay();
  }
  if (upperSW) {
    midiCCOut(CCLFO3Delay, upperData[P_LFO3Delay]);
    midiCCOut61(CCLFO3Delay, upperData[P_LFO3Delay]);
  } else {
    midiCCOut(CCLFO3Delay, lowerData[P_LFO3Delay]);
    midiCCOut61(CCLFO3Delay, lowerData[P_LFO3Delay]);
  }
}

FLASHMEM void updatemodWheelDepth(boolean announce) {
  if (announce) {
    showCurrentParameterPage("Mod Wheel Depth", String(modWheelDepthstr));
    startParameterDisplay();
  }
  if (upperSW) {
    midiCCOut89(CC_MW_FM_DEPTH, upperData[P_modWheelDepth]);
    midiCCOut(CCmodWheelDepth, upperData[P_modWheelDepth]);
    midiCCOut61(CCmodWheelDepth, upperData[P_modWheelDepth]);
  } else {
    midiCCOut79(CC_MW_FM_DEPTH, lowerData[P_modWheelDepth]);
    midiCCOut(CCmodWheelDepth, lowerData[P_modWheelDepth]);
    midiCCOut61(CCmodWheelDepth, lowerData[P_modWheelDepth]);
    if (wholemode) {
      midiCCOut89(CC_MW_FM_DEPTH, upperData[P_modWheelDepth]);
    }
  }
}

FLASHMEM void updatePitchBendDepth(boolean announce) {
  if (announce) {
    showCurrentParameterPage("Pitch Bend Depth", String(PitchBendLevelstr));
    startParameterDisplay();
  }
  if (upperSW) {
    midiCCOut89(CC_PITCHBEND_RANGE, upperData[P_PitchBendLevel]);
    midiCCOut61(CCPitchBend, upperData[P_PitchBendLevel]);
  } else {
    midiCCOut79(CC_PITCHBEND_RANGE, lowerData[P_PitchBendLevel]);
    midiCCOut61(CCPitchBend, lowerData[P_PitchBendLevel]);
    if (wholemode) {
      midiCCOut89(CC_PITCHBEND_RANGE, upperData[P_PitchBendLevel]);
    }
  }
}

FLASHMEM void updateeffectPot1(boolean announce) {
  if (announce) {
    showCurrentParameterPage("Effect Pot 1", String(effectPot1str));
    startParameterDisplay();
  }
  if (upperSW) {
    midiCCOut810(VB_EFFECT_POT1, upperData[P_effectPot1]);
    midiCCOut(CCeffectPot1, upperData[P_effectPot1]);
    midiCCOut61(CCeffectPot1, upperData[P_effectPot1]);
  } else {
    midiCCOut710(VB_EFFECT_POT1, lowerData[P_effectPot1]);
    midiCCOut(CCeffectPot1, lowerData[P_effectPot1]);
    midiCCOut61(CCeffectPot1, lowerData[P_effectPot1]);
    if (wholemode) {
      midiCCOut810(VB_EFFECT_POT1, upperData[P_effectPot1]);
    }
  }
}

FLASHMEM void updateeffectPot2(boolean announce) {
  if (announce) {
    showCurrentParameterPage("Effect Pot 2", String(effectPot2str));
    startParameterDisplay();
  }
  if (upperSW) {
    midiCCOut810(VB_EFFECT_POT2, upperData[P_effectPot2]);
    midiCCOut(CCeffectPot2, upperData[P_effectPot2]);
    midiCCOut61(CCeffectPot2, upperData[P_effectPot2]);
  } else {
    midiCCOut710(VB_EFFECT_POT2, lowerData[P_effectPot2]);
    midiCCOut(CCeffectPot2, lowerData[P_effectPot2]);
    midiCCOut61(CCeffectPot2, lowerData[P_effectPot2]);
    if (wholemode) {
      midiCCOut810(VB_EFFECT_POT2, upperData[P_effectPot2]);
    }
  }
}

FLASHMEM void updateeffectPot3(boolean announce) {
  if (announce) {
    showCurrentParameterPage("Effect Pot 3", String(effectPot3str));
    startParameterDisplay();
  }
  if (upperSW) {
    midiCCOut810(VB_EFFECT_POT3, upperData[P_effectPot3]);
    midiCCOut(CCeffectPot3, upperData[P_effectPot3]);
    midiCCOut61(CCeffectPot3, upperData[P_effectPot3]);
  } else {
    midiCCOut710(VB_EFFECT_POT3, lowerData[P_effectPot3]);
    midiCCOut(CCeffectPot3, lowerData[P_effectPot3]);
    midiCCOut61(CCeffectPot3, lowerData[P_effectPot3]);
    if (wholemode) {
      midiCCOut810(VB_EFFECT_POT3, upperData[P_effectPot3]);
    }
  }
}

FLASHMEM void updateeffectsMix(boolean announce) {
  if (announce) {
    showCurrentParameterPage("Effects Mix", String(effectsMixstr));
    startParameterDisplay();
  }
  if (upperSW) {
    midiCCOut810(VB_EFFECT_MIX, upperData[P_effectsMix]);
    midiCCOut(CCeffectsMix, upperData[P_effectsMix]);
    midiCCOut61(CCeffectsMix, upperData[P_effectsMix]);
  } else {
    midiCCOut710(VB_EFFECT_MIX, lowerData[P_effectsMix]);
    midiCCOut(CCeffectsMix, lowerData[P_effectsMix]);
    midiCCOut61(CCeffectsMix, lowerData[P_effectsMix]);
    if (wholemode) {
      midiCCOut810(VB_EFFECT_MIX, upperData[P_effectsMix]);
    }
  }
}

FLASHMEM void updateLFO1Waveform(boolean announce) {

  switch (panelData[P_LFO1Waveform]) {
    case 0:
      StratusLFOWaveform = "Triangle";
      midiCCOut62(CCLFO1Waveform, 0);
      mcp13.digitalWrite(LFO1_WAVE_LED_RED, HIGH);
      mcp13.digitalWrite(LFO1_WAVE_LED_GREEN, LOW);
      break;

    case 1:
      StratusLFOWaveform = "Square";
      midiCCOut62(CCLFO1Waveform, 1);
      mcp13.digitalWrite(LFO1_WAVE_LED_RED, HIGH);
      mcp13.digitalWrite(LFO1_WAVE_LED_GREEN, HIGH);
      break;

    case 2:
      StratusLFOWaveform = "Sawtooth";
      midiCCOut62(CCLFO1Waveform, 2);
      mcp13.digitalWrite(LFO1_WAVE_LED_RED, LOW);
      mcp13.digitalWrite(LFO1_WAVE_LED_GREEN, HIGH);
      break;
  }

  if (announce) {
    showCurrentParameterPage("LFO1 Waveform", StratusLFOWaveform);
    startParameterDisplay();
  }
  if (upperSW) {
    midiCCOut89(CC_LFO1_WAVEFORM, upperData[P_LFO1Waveform]);
  } else {
    midiCCOut79(CC_LFO1_WAVEFORM, lowerData[P_LFO1Waveform]);
    if (wholemode) {
      midiCCOut89(CC_LFO1_WAVEFORM, upperData[P_LFO1Waveform]);
    }
  }
}

FLASHMEM void updateLFO2Waveform(boolean announce) {

  switch (panelData[P_LFO2Waveform]) {
    case 0:
      StratusLFOWaveform = "Triangle";
      midiCCOut62(CCLFO2Waveform, 0);
      mcp13.digitalWrite(LFO2_WAVE_LED_RED, HIGH);
      mcp13.digitalWrite(LFO2_WAVE_LED_GREEN, LOW);
      break;

    case 1:
      StratusLFOWaveform = "Square";
      midiCCOut62(CCLFO2Waveform, 1);
      mcp13.digitalWrite(LFO2_WAVE_LED_RED, HIGH);
      mcp13.digitalWrite(LFO2_WAVE_LED_GREEN, HIGH);
      break;

    case 2:
      StratusLFOWaveform = "Sawtooth";
      midiCCOut62(CCLFO2Waveform, 2);
      mcp13.digitalWrite(LFO2_WAVE_LED_RED, LOW);
      mcp13.digitalWrite(LFO2_WAVE_LED_GREEN, HIGH);
      break;
  }

  if (announce) {
    showCurrentParameterPage("LFO2 Waveform", StratusLFOWaveform);
    startParameterDisplay();
  }
  if (upperSW) {
    midiCCOut89(CC_LFO2_WAVEFORM, upperData[P_LFO2Waveform]);
  } else {
    midiCCOut79(CC_LFO2_WAVEFORM, lowerData[P_LFO2Waveform]);
    if (wholemode) {
      midiCCOut89(CC_LFO2_WAVEFORM, upperData[P_LFO2Waveform]);
    }
  }
}

void updateLFO3Waveform(boolean announce) {

  if (upperSW) {
    panelData[P_LFO3Waveform] = upperData[P_LFO3Waveform];
  } else {
    panelData[P_LFO3Waveform] = lowerData[P_LFO3Waveform];
  }

  switch (panelData[P_LFO3Waveform]) {
    case 0:
      StratusLFOWaveform = "Sawtooth Up";
      LFOWaveCV = 1;
      panelData[P_lfoAlt] = 127;
      midiCCOut62(CCLFO3Waveform, 0);
      break;

    case 1:
      StratusLFOWaveform = "Sawtooth Down";
      LFOWaveCV = 20;
      panelData[P_lfoAlt] = 127;
      midiCCOut62(CCLFO3Waveform, 1);
      break;

    case 2:
      StratusLFOWaveform = "Squarewave";
      LFOWaveCV = 35;
      panelData[P_lfoAlt] = 127;
      midiCCOut62(CCLFO3Waveform, 2);
      break;

    case 3:
      StratusLFOWaveform = "Triangle";
      LFOWaveCV = 50;
      panelData[P_lfoAlt] = 127;
      midiCCOut62(CCLFO3Waveform, 3);
      break;

    case 4:
      StratusLFOWaveform = "Sinewave";
      LFOWaveCV = 74;
      panelData[P_lfoAlt] = 127;
      midiCCOut62(CCLFO3Waveform, 4);
      break;

    case 5:
      StratusLFOWaveform = "Sweeps";
      LFOWaveCV = 90;
      panelData[P_lfoAlt] = 127;
      midiCCOut62(CCLFO3Waveform, 5);
      break;

    case 6:
      StratusLFOWaveform = "Lumps";
      LFOWaveCV = 107;
      panelData[P_lfoAlt] = 127;
      midiCCOut62(CCLFO3Waveform, 6);
      break;

    case 7:
      StratusLFOWaveform = "Sample & Hold";
      LFOWaveCV = 122;
      panelData[P_lfoAlt] = 127;
      midiCCOut62(CCLFO3Waveform, 7);
      break;

    case 8:
      StratusLFOWaveform = "Saw +Oct";
      LFOWaveCV = 1;
      panelData[P_lfoAlt] = 0;
      midiCCOut62(CCLFO3Waveform, 8);
      break;

    case 9:
      StratusLFOWaveform = "Quad Saw";
      LFOWaveCV = 20;
      panelData[P_lfoAlt] = 0;
      midiCCOut62(CCLFO3Waveform, 9);
      break;

    case 10:
      StratusLFOWaveform = "Quad Pulse";
      LFOWaveCV = 35;
      panelData[P_lfoAlt] = 0;
      midiCCOut62(CCLFO3Waveform, 10);
      break;

    case 11:
      StratusLFOWaveform = "Tri Step";
      LFOWaveCV = 50;
      panelData[P_lfoAlt] = 0;
      midiCCOut62(CCLFO3Waveform, 11);
      break;

    case 12:
      StratusLFOWaveform = "Sine +Oct";
      LFOWaveCV = 74;
      panelData[P_lfoAlt] = 0;
      midiCCOut62(CCLFO3Waveform, 12);
      break;

    case 13:
      StratusLFOWaveform = "Sine +3rd";
      LFOWaveCV = 90;
      panelData[P_lfoAlt] = 0;
      midiCCOut62(CCLFO3Waveform, 13);
      break;

    case 14:
      StratusLFOWaveform = "Sine +4th";
      LFOWaveCV = 107;
      panelData[P_lfoAlt] = 0;
      midiCCOut62(CCLFO3Waveform, 14);
      break;

    case 15:
      StratusLFOWaveform = "Rand Slopes";
      LFOWaveCV = 122;
      panelData[P_lfoAlt] = 0;
      midiCCOut62(CCLFO3Waveform, 15);
      break;
  }
  if (announce) {
    showCurrentParameterPage("LFO3 Wave", StratusLFOWaveform);
    startParameterDisplay();
  }
  if (upperSW) {
    midiCCOut810(VB_LFO3_ALT, panelData[P_lfoAlt]);
    midiCCOut810(VB_LFO3_WAVE, LFOWaveCV);
  } else {
    midiCCOut710(VB_LFO3_ALT, panelData[P_lfoAlt]);
    midiCCOut710(VB_LFO3_WAVE, LFOWaveCV);
    if (wholemode) {
      midiCCOut810(VB_LFO3_ALT, panelData[P_lfoAlt]);
      midiCCOut810(VB_LFO3_WAVE, LFOWaveCV);
    }
  }
}

FLASHMEM void updatepitchAttack(boolean announce) {
  if (announce) {
    if (pitchAttackstr < 1000) {
      showCurrentParameterPage("Pitch Attack", String(int(pitchAttackstr)) + " ms", FILTER_ENV);
    } else {
      showCurrentParameterPage("Pitch Attack", String(pitchAttackstr * 0.001) + " s", FILTER_ENV);
    }
    startParameterDisplay();
  }
  if (upperSW) {
    midiCCOut89(CC_ENV_ATTACK, upperData[P_pitchAttack]);
    midiCCOut(CCpitchAttack, upperData[P_pitchAttack]);
    midiCCOut61(CCpitchAttack, upperData[P_pitchAttack]);
  } else {
    midiCCOut79(CC_ENV_ATTACK, lowerData[P_pitchAttack]);
    midiCCOut(CCpitchAttack, lowerData[P_pitchAttack]);
    midiCCOut61(CCpitchAttack, lowerData[P_pitchAttack]);
    if (wholemode) {
      midiCCOut89(CC_ENV_ATTACK, upperData[P_pitchAttack]);
    }
  }
}

FLASHMEM void updatepitchDecay(boolean announce) {
  if (announce) {
    if (pitchDecaystr < 1000) {
      showCurrentParameterPage("Pitch Decay", String(int(pitchDecaystr)) + " ms", FILTER_ENV);
    } else {
      showCurrentParameterPage("Pitch Decay", String(pitchDecaystr * 0.001) + " s", FILTER_ENV);
    }
    startParameterDisplay();
  }
  if (upperSW) {
    midiCCOut89(CC_ENV_DECAY, upperData[P_pitchDecay]);
    midiCCOut(CCpitchDecay, upperData[P_pitchDecay]);
    midiCCOut61(CCpitchDecay, upperData[P_pitchDecay]);
  } else {
    midiCCOut79(CC_ENV_DECAY, lowerData[P_pitchDecay]);
    midiCCOut(CCpitchDecay, lowerData[P_pitchDecay]);
    midiCCOut61(CCpitchDecay, lowerData[P_pitchDecay]);
    if (wholemode) {
      midiCCOut89(CC_ENV_DECAY, upperData[P_pitchDecay]);
    }
  }
}

FLASHMEM void updatepitchSustain(boolean announce) {
  if (announce) {
    showCurrentParameterPage("Pitch Sustain", String(pitchSustainstr), FILTER_ENV);
    startParameterDisplay();
  }
  if (upperSW) {
    midiCCOut89(CC_ENV_SUSTAIN, upperData[P_pitchSustain]);
    midiCCOut(CCpitchSustain, upperData[P_pitchSustain]);
    midiCCOut61(CCpitchSustain, upperData[P_pitchSustain]);
  } else {
    midiCCOut79(CC_ENV_SUSTAIN, lowerData[P_pitchSustain]);
    midiCCOut(CCpitchSustain, lowerData[P_pitchSustain]);
    midiCCOut61(CCpitchSustain, lowerData[P_pitchSustain]);
    if (wholemode) {
      midiCCOut89(CC_ENV_SUSTAIN, upperData[P_pitchSustain]);
    }
  }
}

FLASHMEM void updatepitchRelease(boolean announce) {
  if (announce) {
    if (pitchReleasestr < 1000) {
      showCurrentParameterPage("Pitch Release", String(int(pitchReleasestr)) + " ms", FILTER_ENV);
    } else {
      showCurrentParameterPage("Pitch Release", String(pitchReleasestr * 0.001) + " s", FILTER_ENV);
    }
    startParameterDisplay();
  }
  if (upperSW) {
    midiCCOut89(CC_ENV_RELEASE, upperData[P_pitchRelease]);
    midiCCOut(CCpitchRelease, upperData[P_pitchRelease]);
    midiCCOut61(CCpitchRelease, upperData[P_pitchRelease]);
  } else {
    midiCCOut79(CC_ENV_RELEASE, lowerData[P_pitchRelease]);
    midiCCOut(CCpitchRelease, lowerData[P_pitchRelease]);
    midiCCOut61(CCpitchRelease, lowerData[P_pitchRelease]);
    if (wholemode) {
      midiCCOut89(CC_ENV_RELEASE, upperData[P_pitchRelease]);
    }
  }
}

FLASHMEM void updatefilterAttack(boolean announce) {
  if (announce) {
    if (filterAttackstr < 1000) {
      showCurrentParameterPage("VCF Attack", String(int(filterAttackstr)) + " ms", FILTER_ENV);
    } else {
      showCurrentParameterPage("VCF Attack", String(filterAttackstr * 0.001) + " s", FILTER_ENV);
    }
    startParameterDisplay();
  }
  if (upperSW) {
    midiCCOut810(VB_VCF_ATTACK, upperData[P_filterAttack]);
    midiCCOut(CCfilterAttack, upperData[P_filterAttack]);
    midiCCOut61(CCfilterAttack, upperData[P_filterAttack]);
  } else {
    midiCCOut710(VB_VCF_ATTACK, lowerData[P_filterAttack]);
    midiCCOut(CCfilterAttack, lowerData[P_filterAttack]);
    midiCCOut61(CCfilterAttack, lowerData[P_filterAttack]);
    if (wholemode) {
      midiCCOut810(VB_VCF_ATTACK, upperData[P_filterAttack]);
    }
  }
}

FLASHMEM void updatefilterDecay(boolean announce) {
  if (announce) {
    if (filterDecaystr < 1000) {
      showCurrentParameterPage("VCF Decay", String(int(filterDecaystr)) + " ms", FILTER_ENV);
    } else {
      showCurrentParameterPage("VCF Decay", String(filterDecaystr * 0.001) + " s", FILTER_ENV);
    }
    startParameterDisplay();
  }
  if (upperSW) {
    midiCCOut810(VB_VCF_DECAY, upperData[P_filterDecay]);
    midiCCOut(CCfilterDecay, upperData[P_filterDecay]);
    midiCCOut61(CCfilterDecay, upperData[P_filterDecay]);
  } else {
    midiCCOut710(VB_VCF_DECAY, lowerData[P_filterDecay]);
    midiCCOut(CCfilterDecay, lowerData[P_filterDecay]);
    midiCCOut61(CCfilterDecay, lowerData[P_filterDecay]);
    if (wholemode) {
      midiCCOut810(VB_VCF_DECAY, upperData[P_filterDecay]);
    }
  }
}

FLASHMEM void updatefilterSustain(boolean announce) {
  if (announce) {
    showCurrentParameterPage("VCF Sustain", String(filterSustainstr), FILTER_ENV);
    startParameterDisplay();
  }
  if (upperSW) {
    midiCCOut810(VB_VCF_SUSTAIN, upperData[P_filterSustain]);
    midiCCOut(CCfilterSustain, upperData[P_filterSustain]);
    midiCCOut61(CCfilterSustain, upperData[P_filterSustain]);
  } else {
    midiCCOut710(VB_VCF_SUSTAIN, lowerData[P_filterSustain]);
    midiCCOut(CCfilterSustain, lowerData[P_filterSustain]);
    midiCCOut61(CCfilterSustain, lowerData[P_filterSustain]);
    if (wholemode) {
      midiCCOut810(VB_VCF_SUSTAIN, upperData[P_filterSustain]);
    }
  }
}

FLASHMEM void updatefilterRelease(boolean announce) {
  if (announce) {
    if (filterReleasestr < 1000) {
      showCurrentParameterPage("VCF Release", String(int(filterReleasestr)) + " ms", FILTER_ENV);
    } else {
      showCurrentParameterPage("VCF Release", String(filterReleasestr * 0.001) + " s", FILTER_ENV);
    }
    startParameterDisplay();
  }
  if (upperSW) {
    midiCCOut810(VB_VCF_RELEASE, upperData[P_filterRelease]);
    midiCCOut(CCfilterRelease, upperData[P_filterRelease]);
    midiCCOut61(CCfilterRelease, upperData[P_filterRelease]);
  } else {
    midiCCOut710(VB_VCF_RELEASE, lowerData[P_filterRelease]);
    midiCCOut(CCfilterRelease, lowerData[P_filterRelease]);
    midiCCOut61(CCfilterRelease, lowerData[P_filterRelease]);
    if (wholemode) {
      midiCCOut810(VB_VCF_RELEASE, upperData[P_filterRelease]);
    }
  }
}

FLASHMEM void updateampAttack(boolean announce) {
  if (announce) {
    if (ampAttackstr < 1000) {
      showCurrentParameterPage("VCA Attack", String(int(ampAttackstr)) + " ms", AMP_ENV);
    } else {
      showCurrentParameterPage("VCA Attack", String(ampAttackstr * 0.001) + " s", AMP_ENV);
    }
    startParameterDisplay();
  }
  if (upperSW) {
    midiCCOut810(VB_VCA_ATTACK, upperData[P_ampAttack]);
    midiCCOut(CCampAttack, upperData[P_ampAttack]);
    midiCCOut61(CCampAttack, upperData[P_ampAttack]);
    upperData[P_oldampAttack] = upperData[P_ampAttack];
  } else {
    midiCCOut710(VB_VCA_ATTACK, lowerData[P_ampAttack]);
    midiCCOut(CCampAttack, lowerData[P_ampAttack]);
    midiCCOut61(CCampAttack, lowerData[P_ampAttack]);
    lowerData[P_oldampAttack] = lowerData[P_ampAttack];
    if (wholemode) {
      midiCCOut810(VB_VCA_ATTACK, upperData[P_ampAttack]);
      upperData[P_oldampAttack] = lowerData[P_oldampAttack];
    }
  }
}

FLASHMEM void updateampDecay(boolean announce) {
  if (announce) {
    if (ampDecaystr < 1000) {
      showCurrentParameterPage("VCA Decay", String(int(ampDecaystr)) + " ms", AMP_ENV);
    } else {
      showCurrentParameterPage("VCA Decay", String(ampDecaystr * 0.001) + " s", AMP_ENV);
    }
    startParameterDisplay();
  }
  if (upperSW) {
    midiCCOut810(VB_VCA_DECAY, upperData[P_ampDecay]);
    midiCCOut(CCampDecay, upperData[P_ampDecay]);
    midiCCOut61(CCampDecay, upperData[P_ampDecay]);
    upperData[P_oldampDecay] = upperData[P_ampDecay];
  } else {
    midiCCOut710(VB_VCA_DECAY, lowerData[P_ampDecay]);
    midiCCOut(CCampDecay, lowerData[P_ampDecay]);
    midiCCOut61(CCampDecay, lowerData[P_ampDecay]);
    lowerData[P_oldampDecay] = lowerData[P_ampDecay];
    if (wholemode) {
      midiCCOut810(VB_VCA_DECAY, upperData[P_ampDecay]);
      upperData[P_oldampDecay] = lowerData[P_oldampDecay];
    }
  }
}

FLASHMEM void updateampSustain(boolean announce) {
  if (announce) {
    showCurrentParameterPage("VCA Sustain", String(ampSustainstr), AMP_ENV);
    startParameterDisplay();
  }
  if (upperSW) {
    midiCCOut810(VB_VCA_SUSTAIN, upperData[P_ampSustain]);
    midiCCOut(CCampSustain, upperData[P_ampSustain]);
    midiCCOut61(CCampSustain, upperData[P_ampSustain]);
    upperData[P_oldampSustain] = upperData[P_ampSustain];
  } else {
    midiCCOut710(VB_VCA_SUSTAIN, lowerData[P_ampSustain]);
    midiCCOut(CCampSustain, lowerData[P_ampSustain]);
    midiCCOut61(CCampSustain, lowerData[P_ampSustain]);
    lowerData[P_oldampSustain] = lowerData[P_ampSustain];
    if (wholemode) {
      midiCCOut810(VB_VCA_SUSTAIN, upperData[P_ampSustain]);
      upperData[P_oldampSustain] = lowerData[P_oldampSustain];
    }
  }
}

FLASHMEM void updateampRelease(boolean announce) {
  if (announce) {
    if (ampReleasestr < 1000) {
      showCurrentParameterPage("VCA Release", String(int(ampReleasestr)) + " ms", AMP_ENV);
    } else {
      showCurrentParameterPage("VCA Release", String(ampReleasestr * 0.001) + " s", AMP_ENV);
    }
    startParameterDisplay();
  }
  if (upperSW) {
    midiCCOut810(VB_VCA_RELEASE, upperData[P_ampRelease]);
    midiCCOut(CCampRelease, upperData[P_ampRelease]);
    midiCCOut61(CCampRelease, upperData[P_ampRelease]);
    upperData[P_oldampRelease] = upperData[P_ampRelease];
  } else {
    midiCCOut710(VB_VCA_RELEASE, lowerData[P_ampRelease]);
    midiCCOut(CCampRelease, lowerData[P_ampRelease]);
    midiCCOut61(CCampRelease, lowerData[P_ampRelease]);
    lowerData[P_oldampRelease] = lowerData[P_ampRelease];
    if (wholemode) {
      midiCCOut810(VB_VCA_RELEASE, upperData[P_ampRelease]);
      upperData[P_oldampRelease] = lowerData[P_oldampRelease];
    }
  }
}

FLASHMEM void updatevolumeControl(boolean announce) {
  if (announce) {
    showCurrentParameterPage("Volume", int(volumeControlstr));
    startParameterDisplay();
  }
  if (upperSW) {
    midiCCOut810(VB_VOLUME, upperData[P_volumeControl]);
    midiCCOut(CCvolumeControl, upperData[P_volumeControl]);
    midiCCOut61(CCvolumeControl, upperData[P_volumeControl]);
  } else {
    midiCCOut710(VB_VOLUME, lowerData[P_volumeControl]);
    midiCCOut(CCvolumeControl, lowerData[P_volumeControl]);
    midiCCOut61(CCvolumeControl, lowerData[P_volumeControl]);
    if (wholemode) {
      midiCCOut810(VB_VOLUME, upperData[P_volumeControl]);
    }
  }
}

FLASHMEM void updatefilterLevel1(boolean announce) {
  if (announce) {
    showCurrentParameterPage("Filter Level 1", int(filterLevel1str));
    startParameterDisplay();
  }
  if (upperSW) {
    midiCCOut810(VB_FILTER1_LEVEL, upperData[P_filterLevel1]);
    midiCCOut(CCfilterLevel1, upperData[P_filterLevel1]);
    midiCCOut61(CCfilterLevel1, upperData[P_filterLevel1]);
  } else {
    midiCCOut710(VB_FILTER1_LEVEL, lowerData[P_filterLevel1]);
    midiCCOut(CCfilterLevel1, lowerData[P_filterLevel1]);
    midiCCOut61(CCfilterLevel1, lowerData[P_filterLevel1]);
    if (wholemode) {
      midiCCOut810(VB_FILTER1_LEVEL, upperData[P_filterLevel1]);
    }
  }
}

FLASHMEM void updatefilterLevel2(boolean announce) {
  if (announce) {
    showCurrentParameterPage("Filter Level 2", int(filterLevel2str));
    startParameterDisplay();
  }
  if (upperSW) {
    midiCCOut810(VB_FILTER2_LEVEL, upperData[P_filterLevel2]);
    midiCCOut(CCfilterLevel2, upperData[P_filterLevel2]);
    midiCCOut61(CCfilterLevel2, upperData[P_filterLevel2]);
  } else {
    midiCCOut710(VB_FILTER2_LEVEL, lowerData[P_filterLevel2]);
    midiCCOut(CCfilterLevel2, lowerData[P_filterLevel2]);
    midiCCOut61(CCfilterLevel2, lowerData[P_filterLevel2]);
    if (wholemode) {
      midiCCOut810(VB_FILTER2_LEVEL, upperData[P_filterLevel2]);
    }
  }
}

FLASHMEM void updateosc1sawDetune(boolean announce) {
  if (announce) {
    showCurrentParameterPage("Osc1 Saw Detune", int(osc1sawDetunestr));
    startParameterDisplay();
  }
  if (upperSW) {
    midiCCOut89(CC_DCO1_SAW_DETUNE, upperData[P_osc1sawDetune]);
    midiCCOut(CCosc1sawDetune, upperData[P_osc1sawDetune]);
    midiCCOut61(CCosc1sawDetune, upperData[P_osc1sawDetune]);
  } else {
    midiCCOut79(CC_DCO1_SAW_DETUNE, lowerData[P_osc1sawDetune]);
    midiCCOut(CCosc1sawDetune, lowerData[P_osc1sawDetune]);
    midiCCOut61(CCosc1sawDetune, lowerData[P_osc1sawDetune]);
    if (wholemode) {
      midiCCOut89(CC_DCO1_SAW_DETUNE, upperData[P_osc1sawDetune]);
    }
  }
}

FLASHMEM void updateosc1sawCount(boolean announce) {
  if (announce) {
    showCurrentParameterPage("Osc1 Saw Count", int(osc1sawCountstr));
    startParameterDisplay();
  }
  if (upperSW) {
    midiCCOut89(CC_DCO1_SAW_COUNT, upperData[P_osc1sawCount]);
    midiCCOut(CCosc1sawCount, upperData[P_osc1sawCount]);
    midiCCOut61(CCosc1sawCount, upperData[P_osc1sawCount]);
  } else {
    midiCCOut79(CC_DCO1_SAW_COUNT, lowerData[P_osc1sawCount]);
    midiCCOut(CCosc1sawCount, lowerData[P_osc1sawCount]);
    midiCCOut61(CCosc1sawCount, lowerData[P_osc1sawCount]);
    if (wholemode) {
      midiCCOut89(CC_DCO1_SAW_COUNT, upperData[P_osc1sawCount]);
    }
  }
}

// ////////////////////////////////////////////////////////////////

void updatechordHoldSW(boolean announce) {
  if (upperSW) {
    if (chordHoldU == 0) {
      if (announce) {
        showCurrentParameterPage("Chord Hold", "Off");
      }
      midiCCOut(CCchordHoldSW, 0);
      midiCCOut62(CCchordHoldSW, 0);
      onHoldButtonReleased();
    } else {
      if (announce) {
        showCurrentParameterPage("Chord Hold", "On");
      }
      midiCCOut(CCchordHoldSW, 127);
      midiCCOut62(CCchordHoldSW, 127);
      onHoldButtonPressed();
    }
  } else {
    if (chordHoldL == 0) {
      if (announce) {
        showCurrentParameterPage("Chord Hold", "Off");
      }
      midiCCOut(CCchordHoldSW, 0);
      midiCCOut62(CCchordHoldSW, 0);
      onHoldButtonReleased();
    } else {
      if (announce) {
        showCurrentParameterPage("Chord Hold", "On");
      }
      midiCCOut(CCchordHoldSW, 127);
      midiCCOut62(CCchordHoldSW, 127);
      onHoldButtonPressed();
    }
  }
}


FLASHMEM void updateplayMode(boolean announce) {
  if (playMode == 0) {
    if (announce) {
      showCurrentParameterPage("Key Mode", "Whole");
      startParameterDisplay();
    }
    midiCCOut62(CCplayMode, 0);
    midiCCOut(CCplayMode, 0);
    //srp.writePin(UPPER_RELAY_2, HIGH);
    //srp.writePin(UPPER_RELAY_3, HIGH);
    mcp3.digitalWrite(MODE_LED_RED, HIGH);
    mcp4.digitalWrite(MODE_LED_GREEN, LOW);
    wholemode = true;
    dualmode = false;
    splitmode = false;
    lowerSW = true;
    upperSW = false;
    updatelowerSW(0);
    lowerParamsToDisplay();
    setAllButtons();

  } else if (playMode == 1) {
    if (announce) {
      showCurrentParameterPage("Key Mode", "Dual");
      startParameterDisplay();
    }
    midiCCOut62(CCplayMode, 1);
    midiCCOut(CCplayMode, 1);
    //srp.writePin(UPPER_RELAY_2, LOW);
    //srp.writePin(UPPER_RELAY_3, LOW);
    mcp3.digitalWrite(MODE_LED_RED, HIGH);
    mcp4.digitalWrite(MODE_LED_GREEN, HIGH);
    wholemode = false;
    dualmode = true;
    splitmode = false;
  } else if (playMode == 2) {
    if (announce) {
      showCurrentParameterPage("Key Mode", "Split");
      startParameterDisplay();
    }
    midiCCOut62(CCplayMode, 2);
    midiCCOut(CCplayMode, 2);
    //srp.writePin(UPPER_RELAY_2, LOW);
    //srp.writePin(UPPER_RELAY_3, LOW);
    mcp3.digitalWrite(MODE_LED_RED, LOW);
    mcp4.digitalWrite(MODE_LED_GREEN, HIGH);
    wholemode = false;
    dualmode = false;
    splitmode = true;
  }
}

FLASHMEM void updatekeyboardMode(boolean announce) {
  if (upperSW) {
    if (dualmode) {
      lowerData[P_keyboardMode] = upperData[P_keyboardMode];
    }
    if (upperData[P_keyboardMode] == 0) {
      if (announce) {
        showCurrentParameterPage("Keyboard Mode", "Poly 1");
        startParameterDisplay();
      }
      mcp3.digitalWrite(POLY1_LED, HIGH);
      mcp3.digitalWrite(POLY2_LED, LOW);
      mcp3.digitalWrite(UNISON_LED, LOW);
      mcp3.digitalWrite(MONO_LED, LOW);
      midiCCOut62(CCkeyboardMode, 0);
      midiCCOut(CCkeyboardMode, 0);
    } else if (upperData[P_keyboardMode] == 1) {
      if (announce) {
        showCurrentParameterPage("Keyboard Mode", "Poly 2");
        startParameterDisplay();
      }
      mcp3.digitalWrite(POLY1_LED, LOW);
      mcp3.digitalWrite(POLY2_LED, HIGH);
      mcp3.digitalWrite(UNISON_LED, LOW);
      mcp3.digitalWrite(MONO_LED, LOW);
      midiCCOut62(CCkeyboardMode, 1);
      midiCCOut(CCkeyboardMode, 1);
    } else if (upperData[P_keyboardMode] == 2) {
      if (announce) {
        showCurrentParameterPage("Keyboard Mode", "Mono");
        startParameterDisplay();
      }
      midiCCOut62(CCkeyboardMode, 2);
      midiCCOut(CCkeyboardMode, 2);
      mcp3.digitalWrite(POLY1_LED, LOW);
      mcp3.digitalWrite(POLY2_LED, LOW);
      mcp3.digitalWrite(UNISON_LED, LOW);
      mcp3.digitalWrite(MONO_LED, HIGH);
    } else if (upperData[P_keyboardMode] == 3) {
      if (announce) {
        showCurrentParameterPage("Keyboard Mode", "Unison");
        startParameterDisplay();
      }
      mcp3.digitalWrite(POLY1_LED, LOW);
      mcp3.digitalWrite(POLY2_LED, LOW);
      mcp3.digitalWrite(UNISON_LED, HIGH);
      mcp3.digitalWrite(MONO_LED, LOW);
      midiCCOut62(CCkeyboardMode, 3);
      midiCCOut(CCkeyboardMode, 3);
    }
  } else {
    if (dualmode) {
      upperData[P_keyboardMode] = lowerData[P_keyboardMode];
    }
    if (lowerData[P_keyboardMode] == 0) {
      if (announce) {
        showCurrentParameterPage("Keyboard Mode", "Poly 1");
        startParameterDisplay();
      }
      midiCCOut62(CCkeyboardMode, 0);
      midiCCOut(CCkeyboardMode, 0);
      mcp3.digitalWrite(POLY1_LED, HIGH);
      mcp3.digitalWrite(POLY2_LED, LOW);
      mcp3.digitalWrite(UNISON_LED, LOW);
      mcp3.digitalWrite(MONO_LED, LOW);
    } else if (lowerData[P_keyboardMode] == 1) {
      if (announce) {
        showCurrentParameterPage("Keyboard Mode", "Poly 2");
        startParameterDisplay();
      }
      mcp3.digitalWrite(POLY1_LED, LOW);
      mcp3.digitalWrite(POLY2_LED, HIGH);
      mcp3.digitalWrite(UNISON_LED, LOW);
      mcp3.digitalWrite(MONO_LED, LOW);
      midiCCOut62(CCkeyboardMode, 1);
      midiCCOut(CCkeyboardMode, 1);
    } else if (lowerData[P_keyboardMode] == 2) {
      if (announce) {
        showCurrentParameterPage("Keyboard Mode", "Mono");
        startParameterDisplay();
      }
      mcp3.digitalWrite(POLY1_LED, LOW);
      mcp3.digitalWrite(POLY2_LED, LOW);
      mcp3.digitalWrite(UNISON_LED, LOW);
      mcp3.digitalWrite(MONO_LED, HIGH);
      midiCCOut62(CCkeyboardMode, 2);
      midiCCOut(CCkeyboardMode, 2);
    } else if (lowerData[P_keyboardMode] == 3) {
      if (announce) {
        showCurrentParameterPage("Keyboard Mode", "Unison");
        startParameterDisplay();
      }
      mcp3.digitalWrite(POLY1_LED, LOW);
      mcp3.digitalWrite(POLY2_LED, LOW);
      mcp3.digitalWrite(UNISON_LED, HIGH);
      mcp3.digitalWrite(MONO_LED, LOW);
      midiCCOut62(CCkeyboardMode, 3);
      midiCCOut(CCkeyboardMode, 3);
    }
  }
}

FLASHMEM void updateeffectNumSW(boolean announce) {
  if (upperSW) {
    if (upperData[P_effectNum] == 0) {
      if (announce) {
        showCurrentParameterPage("Effect", "1");
        startParameterDisplay();
      }
      midiCCOut89(CC_FV1_EFFECT_0, 0);
      midiCCOut89(CC_FV1_EFFECT_1, 0);
      midiCCOut89(CC_FV1_EFFECT_2, 0);
      midiCCOut62(CCeffectNumSW, 0);
      midiCCOut(CCeffectNumSW, 0);

    } else if (upperData[P_effectNum] == 1) {
      if (announce) {
        showCurrentParameterPage("Effect", "2");
        startParameterDisplay();
      }
      midiCCOut89(CC_FV1_EFFECT_0, 127);
      midiCCOut89(CC_FV1_EFFECT_1, 0);
      midiCCOut89(CC_FV1_EFFECT_2, 0);
      midiCCOut62(CCeffectNumSW, 1);
      midiCCOut(CCeffectNumSW, 1);

    } else if (upperData[P_effectNum] == 2) {
      if (announce) {
        showCurrentParameterPage("Effect", "3");
        startParameterDisplay();
      }
      midiCCOut89(CC_FV1_EFFECT_0, 0);
      midiCCOut89(CC_FV1_EFFECT_1, 127);
      midiCCOut89(CC_FV1_EFFECT_2, 0);
      midiCCOut62(CCeffectNumSW, 2);
      midiCCOut(CCeffectNumSW, 2);

    } else if (upperData[P_effectNum] == 3) {
      if (announce) {
        showCurrentParameterPage("Effect", "4");
        startParameterDisplay();
      }
      midiCCOut89(CC_FV1_EFFECT_0, 127);
      midiCCOut89(CC_FV1_EFFECT_1, 127);
      midiCCOut89(CC_FV1_EFFECT_2, 0);
      midiCCOut62(CCeffectNumSW, 3);
      midiCCOut(CCeffectNumSW, 3);

    } else if (upperData[P_effectNum] == 4) {
      if (announce) {
        showCurrentParameterPage("Effect", "5");
        startParameterDisplay();
      }
      midiCCOut89(CC_FV1_EFFECT_0, 0);
      midiCCOut89(CC_FV1_EFFECT_1, 0);
      midiCCOut89(CC_FV1_EFFECT_2, 127);
      midiCCOut62(CCeffectNumSW, 4);
      midiCCOut(CCeffectNumSW, 4);

    } else if (upperData[P_effectNum] == 5) {
      if (announce) {
        showCurrentParameterPage("Effect", "6");
        startParameterDisplay();
      }
      midiCCOut89(CC_FV1_EFFECT_0, 127);
      midiCCOut89(CC_FV1_EFFECT_1, 0);
      midiCCOut89(CC_FV1_EFFECT_2, 127);
      midiCCOut62(CCeffectNumSW, 5);
      midiCCOut(CCeffectNumSW, 5);

    } else if (upperData[P_effectNum] == 6) {
      if (announce) {
        showCurrentParameterPage("Effect", "7");
        startParameterDisplay();
      }
      midiCCOut89(CC_FV1_EFFECT_0, 0);
      midiCCOut89(CC_FV1_EFFECT_1, 127);
      midiCCOut89(CC_FV1_EFFECT_2, 127);
      midiCCOut62(CCeffectNumSW, 6);
      midiCCOut(CCeffectNumSW, 6);

    } else if (upperData[P_effectNum] == 7) {
      if (announce) {
        showCurrentParameterPage("Effect", "8");
        startParameterDisplay();
      }
      midiCCOut89(CC_FV1_EFFECT_0, 127);
      midiCCOut89(CC_FV1_EFFECT_1, 127);
      midiCCOut89(CC_FV1_EFFECT_2, 127);
      midiCCOut62(CCeffectNumSW, 7);
      midiCCOut(CCeffectNumSW, 7);
    }

  } else {
    if (lowerData[P_effectNum] == 0) {
      if (announce) {
        showCurrentParameterPage("Effect", "1");
        startParameterDisplay();
      }
      midiCCOut79(CC_FV1_EFFECT_0, 0);
      midiCCOut79(CC_FV1_EFFECT_1, 0);
      midiCCOut79(CC_FV1_EFFECT_2, 0);
      if (wholemode) {
        midiCCOut89(CC_FV1_EFFECT_0, 0);
        midiCCOut89(CC_FV1_EFFECT_1, 0);
        midiCCOut89(CC_FV1_EFFECT_2, 0);
      }
      midiCCOut62(CCeffectNumSW, 0);
      midiCCOut(CCeffectNumSW, 0);

    } else if (lowerData[P_effectNum] == 1) {
      if (announce) {
        showCurrentParameterPage("Effect", "2");
        startParameterDisplay();
      }
      midiCCOut79(CC_FV1_EFFECT_0, 127);
      midiCCOut79(CC_FV1_EFFECT_1, 0);
      midiCCOut79(CC_FV1_EFFECT_2, 0);
      if (wholemode) {
        midiCCOut89(CC_FV1_EFFECT_0, 127);
        midiCCOut89(CC_FV1_EFFECT_1, 0);
        midiCCOut89(CC_FV1_EFFECT_2, 0);
      }
      midiCCOut62(CCeffectNumSW, 1);
      midiCCOut(CCeffectNumSW, 1);

    } else if (lowerData[P_effectNum] == 2) {
      if (announce) {
        showCurrentParameterPage("Effect", "3");
        startParameterDisplay();
      }
      midiCCOut79(CC_FV1_EFFECT_0, 0);
      midiCCOut79(CC_FV1_EFFECT_1, 127);
      midiCCOut79(CC_FV1_EFFECT_2, 0);
      if (wholemode) {
        midiCCOut89(CC_FV1_EFFECT_0, 0);
        midiCCOut89(CC_FV1_EFFECT_1, 127);
        midiCCOut89(CC_FV1_EFFECT_2, 0);
      }
      midiCCOut62(CCeffectNumSW, 2);
      midiCCOut(CCeffectNumSW, 2);

    } else if (lowerData[P_effectNum] == 3) {
      if (announce) {
        showCurrentParameterPage("Effect", "4");
        startParameterDisplay();
      }
      midiCCOut79(CC_FV1_EFFECT_0, 127);
      midiCCOut79(CC_FV1_EFFECT_1, 127);
      midiCCOut79(CC_FV1_EFFECT_2, 0);
      if (wholemode) {
        midiCCOut89(CC_FV1_EFFECT_0, 127);
        midiCCOut89(CC_FV1_EFFECT_1, 127);
        midiCCOut89(CC_FV1_EFFECT_2, 0);
      }
      midiCCOut62(CCeffectNumSW, 3);
      midiCCOut(CCeffectNumSW, 3);

    } else if (lowerData[P_effectNum] == 4) {
      if (announce) {
        showCurrentParameterPage("Effect", "5");
        startParameterDisplay();
      }
      midiCCOut79(CC_FV1_EFFECT_0, 0);
      midiCCOut79(CC_FV1_EFFECT_1, 0);
      midiCCOut79(CC_FV1_EFFECT_2, 127);
      if (wholemode) {
        midiCCOut89(CC_FV1_EFFECT_0, 0);
        midiCCOut89(CC_FV1_EFFECT_1, 0);
        midiCCOut89(CC_FV1_EFFECT_2, 127);
      }
      midiCCOut62(CCeffectNumSW, 4);
      midiCCOut(CCeffectNumSW, 4);

    } else if (lowerData[P_effectNum] == 5) {
      if (announce) {
        showCurrentParameterPage("Effect", "6");
        startParameterDisplay();
      }
      midiCCOut79(CC_FV1_EFFECT_0, 127);
      midiCCOut79(CC_FV1_EFFECT_1, 0);
      midiCCOut79(CC_FV1_EFFECT_2, 127);
      if (wholemode) {
        midiCCOut89(CC_FV1_EFFECT_0, 127);
        midiCCOut89(CC_FV1_EFFECT_1, 0);
        midiCCOut89(CC_FV1_EFFECT_2, 127);
      }
      midiCCOut62(CCeffectNumSW, 5);
      midiCCOut(CCeffectNumSW, 5);

    } else if (lowerData[P_effectNum] == 6) {
      if (announce) {
        showCurrentParameterPage("Effect", "7");
        startParameterDisplay();
      }
      midiCCOut79(CC_FV1_EFFECT_0, 0);
      midiCCOut79(CC_FV1_EFFECT_1, 127);
      midiCCOut79(CC_FV1_EFFECT_2, 127);
      if (wholemode) {
        midiCCOut89(CC_FV1_EFFECT_0, 0);
        midiCCOut89(CC_FV1_EFFECT_1, 127);
        midiCCOut89(CC_FV1_EFFECT_2, 127);
      }
      midiCCOut62(CCeffectNumSW, 6);
      midiCCOut(CCeffectNumSW, 6);

    } else if (lowerData[P_effectNum] == 7) {
      if (announce) {
        showCurrentParameterPage("Effect", "8");
        startParameterDisplay();
      }
      midiCCOut79(CC_FV1_EFFECT_0, 127);
      midiCCOut79(CC_FV1_EFFECT_1, 127);
      midiCCOut79(CC_FV1_EFFECT_2, 127);
      if (wholemode) {
        midiCCOut89(CC_FV1_EFFECT_0, 127);
        midiCCOut89(CC_FV1_EFFECT_1, 127);
        midiCCOut89(CC_FV1_EFFECT_2, 127);
      }
      midiCCOut62(CCeffectNumSW, 7);
      midiCCOut(CCeffectNumSW, 7);
    }
  }
}

FLASHMEM void updateeffectBankSW(boolean announce) {
  int bank = upperSW ? upperData[P_effectBank] : lowerData[P_effectBank];

  if (announce) {
    showCurrentParameterPage("Effects", "Bank " + String(bank + 1));
    startParameterDisplay();
  }

  if (upperSW) {
    // Step 1: Enter external mode
    midiCCOut89(CC_FV1_INTERNAL, 127);

    // Step 2: Reset all CS lines
    midiCCOut89(CC_FV1_BANK_0, 127);
    midiCCOut89(CC_FV1_BANK_1, 127);
    midiCCOut89(CC_FV1_BANK_2, 127);


    if (bank == 0) {
      // Internal ROM selected
      midiCCOut89(CC_FV1_INTERNAL, 0);

    } else {
      // Select only the chosen EEPROM
      if (bank == 1) {
        midiCCOut89(CC_FV1_BANK_0, 0);
      } else if (bank == 2) {
        midiCCOut89(CC_FV1_BANK_1, 0);
      } else if (bank == 3) {
        midiCCOut89(CC_FV1_BANK_2, 0);

        midiCCOut89(CC_FV1_INTERNAL, 0);
        delay(1);
        midiCCOut89(CC_FV1_INTERNAL, 127);
      }
    }
  } else {
    // Step 1: Enter external mode
    midiCCOut79(CC_FV1_INTERNAL, 127);

    // Step 2: Reset all CS lines
    midiCCOut79(CC_FV1_BANK_0, 127);
    midiCCOut79(CC_FV1_BANK_1, 127);
    midiCCOut79(CC_FV1_BANK_2, 127);
    if (wholemode) {
      midiCCOut89(CC_FV1_INTERNAL, 127);

      // Step 2: Reset all CS lines
      midiCCOut89(CC_FV1_BANK_0, 127);
      midiCCOut89(CC_FV1_BANK_1, 127);
      midiCCOut89(CC_FV1_BANK_2, 127);
    }

    if (bank == 0) {
      midiCCOut79(CC_FV1_INTERNAL, 0);
      if (wholemode) {
        midiCCOut89(CC_FV1_INTERNAL, 0);
      }
    } else {
      if (bank == 1) {
        midiCCOut79(CC_FV1_BANK_0, 0);
        if (wholemode) {
          midiCCOut89(CC_FV1_BANK_0, 0);
        }
      } else if (bank == 2) {
        midiCCOut79(CC_FV1_BANK_1, 0);
        if (wholemode) {
          midiCCOut89(CC_FV1_BANK_1, 0);
        }
      } else if (bank == 3) {
        midiCCOut79(CC_FV1_BANK_2, 0);
        if (wholemode) {
          midiCCOut89(CC_FV1_BANK_2, 0);
        }

      }
      midiCCOut79(CC_FV1_INTERNAL, 0);
      delay(1);
      midiCCOut79(CC_FV1_INTERNAL, 127);
      if (wholemode) {
        midiCCOut89(CC_FV1_INTERNAL, 0);
        delay(1);
        midiCCOut89(CC_FV1_INTERNAL, 127);
      }

    }

    // Send MIDI
    midiCCOut62(CCeffectBankSW, bank);
    midiCCOut(CCeffectBankSW, bank);
  }
}

FLASHMEM void updatelfoMultiplier(boolean announce) {
  if (upperSW) {
    if (upperData[P_lfoMultiplier] == 0) {
      if (announce) {
        showCurrentParameterPage("LFO Multiplier", "x0.5");
        startParameterDisplay();
      }
      midiCCOut89(VB_MULTIPLIER_BIT0, 0);
      midiCCOut89(VB_MULTIPLIER_BIT1, 0);
      midiCCOut89(VB_MULTIPLIER_BIT2, 0);
      midiCCOut62(CClfoMult, 0);
      midiCCOut(CClfoMult, 0);
      mcp13.digitalWrite(LFO3_MULT_LED_RED, LOW);
      mcp13.digitalWrite(LFO3_MULT_LED_GREEN, LOW);
    } else if (upperData[P_lfoMultiplier] == 1) {
      if (announce) {
        showCurrentParameterPage("LFO Multiplier", "x1.0");
        startParameterDisplay();
      }
      midiCCOut89(VB_MULTIPLIER_BIT0, 127);
      midiCCOut89(VB_MULTIPLIER_BIT1, 0);
      midiCCOut89(VB_MULTIPLIER_BIT2, 0);
      midiCCOut62(CClfoMult, 1);
      midiCCOut(CClfoMult, 1);
      mcp13.digitalWrite(LFO3_MULT_LED_RED, HIGH);
      mcp13.digitalWrite(LFO3_MULT_LED_GREEN, LOW);
    } else if (upperData[P_lfoMultiplier] == 2) {
      if (announce) {
        showCurrentParameterPage("LFO Multiplier", "x1.5");
        startParameterDisplay();
      }
      midiCCOut89(VB_MULTIPLIER_BIT0, 0);
      midiCCOut89(VB_MULTIPLIER_BIT1, 127);
      midiCCOut89(VB_MULTIPLIER_BIT2, 0);
      midiCCOut62(CClfoMult, 2);
      midiCCOut(CClfoMult, 2);
      mcp13.digitalWrite(LFO3_MULT_LED_RED, LOW);
      mcp13.digitalWrite(LFO3_MULT_LED_GREEN, HIGH);
    } else if (upperData[P_lfoMultiplier] == 3) {
      if (announce) {
        showCurrentParameterPage("LFO Multiplier", "x2.0");
        startParameterDisplay();
      }
      midiCCOut89(VB_MULTIPLIER_BIT0, 127);
      midiCCOut89(VB_MULTIPLIER_BIT1, 127);
      midiCCOut89(VB_MULTIPLIER_BIT2, 0);
      midiCCOut62(CClfoMult, 3);
      midiCCOut(CClfoMult, 3);
      mcp13.digitalWrite(LFO3_MULT_LED_RED, HIGH);
      mcp13.digitalWrite(LFO3_MULT_LED_GREEN, HIGH);
    }
  } else {
    if (lowerData[P_lfoMultiplier] == 0) {
      if (announce) {
        showCurrentParameterPage("LFO Multiplier", "x0.5");
        startParameterDisplay();
      }
      midiCCOut79(VB_MULTIPLIER_BIT0, 0);
      midiCCOut79(VB_MULTIPLIER_BIT1, 0);
      midiCCOut79(VB_MULTIPLIER_BIT2, 0);
      if (wholemode) {
        midiCCOut89(VB_MULTIPLIER_BIT0, 0);
        midiCCOut89(VB_MULTIPLIER_BIT1, 0);
        midiCCOut89(VB_MULTIPLIER_BIT2, 0);
      }
      midiCCOut62(CClfoMult, 0);
      midiCCOut(CClfoMult, 0);
      mcp13.digitalWrite(LFO3_MULT_LED_RED, LOW);
      mcp13.digitalWrite(LFO3_MULT_LED_GREEN, LOW);
    } else if (lowerData[P_lfoMultiplier] == 1) {
      if (announce) {
        showCurrentParameterPage("LFO Multiplier", "x1.0");
        startParameterDisplay();
      }
      midiCCOut79(VB_MULTIPLIER_BIT0, 127);
      midiCCOut79(VB_MULTIPLIER_BIT1, 0);
      midiCCOut79(VB_MULTIPLIER_BIT2, 0);
      if (wholemode) {
        midiCCOut89(VB_MULTIPLIER_BIT0, 127);
        midiCCOut89(VB_MULTIPLIER_BIT1, 0);
        midiCCOut89(VB_MULTIPLIER_BIT2, 0);
      }
      midiCCOut62(CClfoMult, 1);
      midiCCOut(CClfoMult, 1);
      mcp13.digitalWrite(LFO3_MULT_LED_RED, HIGH);
      mcp13.digitalWrite(LFO3_MULT_LED_GREEN, LOW);
    } else if (lowerData[P_lfoMultiplier] == 2) {
      if (announce) {
        showCurrentParameterPage("LFO Multiplier", "x1.5");
        startParameterDisplay();
      }
      midiCCOut79(VB_MULTIPLIER_BIT0, 0);
      midiCCOut79(VB_MULTIPLIER_BIT1, 127);
      midiCCOut79(VB_MULTIPLIER_BIT2, 0);
      if (wholemode) {
        midiCCOut89(VB_MULTIPLIER_BIT0, 0);
        midiCCOut89(VB_MULTIPLIER_BIT1, 127);
        midiCCOut89(VB_MULTIPLIER_BIT2, 0);
      }
      midiCCOut62(CClfoMult, 2);
      midiCCOut(CClfoMult, 2);
      mcp13.digitalWrite(LFO3_MULT_LED_RED, LOW);
      mcp13.digitalWrite(LFO3_MULT_LED_GREEN, HIGH);
    } else if (lowerData[P_lfoMultiplier] == 3) {
      if (announce) {
        showCurrentParameterPage("LFO Multiplier", "x2.0");
        startParameterDisplay();
      }
      midiCCOut79(VB_MULTIPLIER_BIT0, 127);
      midiCCOut79(VB_MULTIPLIER_BIT1, 127);
      midiCCOut79(VB_MULTIPLIER_BIT2, 0);
      if (wholemode) {
        midiCCOut89(VB_MULTIPLIER_BIT0, 127);
        midiCCOut89(VB_MULTIPLIER_BIT1, 127);
        midiCCOut89(VB_MULTIPLIER_BIT2, 0);
      }
      midiCCOut62(CClfoMult, 3);
      midiCCOut(CClfoMult, 3);
      mcp13.digitalWrite(LFO3_MULT_LED_RED, HIGH);
      mcp13.digitalWrite(LFO3_MULT_LED_GREEN, HIGH);
    }
  }
}

FLASHMEM void updateglideSW(boolean announce) {
  if (upperSW) {
    if (upperData[P_glideSW] == 0) {
      if (announce) {
        showCurrentParameterPage("Glide", "Off");
        startParameterDisplay();
      }
      midiCCOut89(CC_PORTAMENTO_SW, 0);
      midiCCOut62(CCglideSW, 0);
      mcp4.digitalWrite(GLIDE_LED_GREEN, LOW);
    } else {
      if (announce) {
        showCurrentParameterPage("Glide", "On");
        startParameterDisplay();
      }
      midiCCOut89(CC_PORTAMENTO_TIME, upperData[P_glideTime]);
      midiCCOut89(CC_PORTAMENTO_SW, 127);
      midiCCOut61(CCglideTime, upperData[P_glideTime]);
      midiCCOut62(CCglideSW, 1);
      mcp4.digitalWrite(GLIDE_LED_GREEN, HIGH);
    }
  } else {
    if (lowerData[P_glideSW] == 0) {
      if (announce) {
        showCurrentParameterPage("Glide", "Off");
        startParameterDisplay();
      }
      midiCCOut79(CC_PORTAMENTO_SW, 0);
      midiCCOut62(CCglideSW, 0);
      if (wholemode) {
        midiCCOut89(CC_PORTAMENTO_SW, 0);
        mcp4.digitalWrite(GLIDE_LED_GREEN, LOW);
      }
      mcp4.digitalWrite(GLIDE_LED_RED, LOW);
    } else {
      if (announce) {
        showCurrentParameterPage("Glide", "On");
        startParameterDisplay();
      }
      midiCCOut79(CC_PORTAMENTO_TIME, lowerData[P_glideTime]);
      midiCCOut79(CC_PORTAMENTO_SW, 127);

      midiCCOut61(CCglideTime, lowerData[P_glideTime]);
      midiCCOut62(CCglideSW, 1);
      mcp4.digitalWrite(GLIDE_LED_RED, HIGH);
      if (wholemode) {
        midiCCOut89(CC_PORTAMENTO_TIME, upperData[P_glideTime]);
        midiCCOut89(CC_PORTAMENTO_SW, 127);
        mcp4.digitalWrite(GLIDE_LED_GREEN, HIGH);
      }
    }
  }
}

FLASHMEM void updatefilterPoleSwitch(boolean announce) {
  if (upperSW) {
    if (upperData[P_filterPoleSW] == 1) {
      if (announce) {
        updateFilterType(1);
      }
      midiCCOut810(VB_FILTER_POLE, 127);
      midiCCOut(CCfilterPoleSW, 127);
      midiCCOut62(CCfilterPoleSW, 127);
      mcp8.digitalWrite(VCF_POLE_LED, HIGH);
    } else {
      if (announce) {
        updateFilterType(1);
      }
      midiCCOut810(VB_FILTER_POLE, 0);
      midiCCOut(CCfilterPoleSW, 0);
      midiCCOut62(CCfilterPoleSW, 0);
      mcp8.digitalWrite(VCF_POLE_LED, LOW);
    }
  } else {
    if (lowerData[P_filterPoleSW] == 1) {
      if (announce) {
        updateFilterType(1);
      }
      midiCCOut710(VB_FILTER_POLE, 127);
      if (wholemode) {
        midiCCOut810(VB_FILTER_POLE, 127);
      }
      midiCCOut(CCfilterPoleSW, 127);
      midiCCOut62(CCfilterPoleSW, 127);
      mcp8.digitalWrite(VCF_POLE_LED, HIGH);
    } else {
      if (announce) {
        updateFilterType(1);
      }
      midiCCOut710(VB_FILTER_POLE, 00);
      if (wholemode) {
        midiCCOut810(VB_FILTER_POLE, 0);
      }
      midiCCOut(CCfilterPoleSW, 0);
      midiCCOut62(CCfilterPoleSW, 0);
      mcp8.digitalWrite(VCF_POLE_LED, LOW);
    }
  }
}

FLASHMEM void updatefilterLoop(boolean announce) {
  if (upperSW) {
    switch (upperData[P_filterLoop]) {
      case 0:
        if (announce) {
          showCurrentParameterPage("VCF Key Loop", "Off");
          startParameterDisplay();
        }
        midiCCOut810(VB_FILTER_LOOP_BIT0, 0);
        midiCCOut810(VB_FILTER_LOOP_BIT1, 0);
        midiCCOut62(CCFilterLoop, 0);
        midiCCOut(CCFilterLoop, 0);
        mcp9.digitalWrite(VCF_LOOP_LED_RED, LOW);
        mcp10.digitalWrite(VCF_LOOP_LED_GREEN, LOW);
        break;

      case 1:
        if (announce) {
          showCurrentParameterPage("VCF LFO Loop", "Gated");
          startParameterDisplay();
        }
        midiCCOut810(VB_FILTER_LOOP_BIT0, 127);
        midiCCOut810(VB_FILTER_LOOP_BIT1, 0);
        midiCCOut62(CCFilterLoop, 1);
        midiCCOut(CCFilterLoop, 63);
        mcp9.digitalWrite(VCF_LOOP_LED_RED, HIGH);
        mcp10.digitalWrite(VCF_LOOP_LED_GREEN, LOW);
        break;

      case 2:
        if (announce) {
          showCurrentParameterPage("VCF Looping", "LFO");
          startParameterDisplay();
        }
        midiCCOut810(VB_FILTER_LOOP_BIT0, 0);
        midiCCOut810(VB_FILTER_LOOP_BIT1, 127);
        midiCCOut62(CCFilterLoop, 2);
        midiCCOut(CCFilterLoop, 127);
        mcp9.digitalWrite(VCF_LOOP_LED_RED, LOW);
        mcp10.digitalWrite(VCF_LOOP_LED_GREEN, HIGH);
        break;
    }
  } else {
    switch (lowerData[P_filterLoop]) {
      case 0:
        if (announce) {
          showCurrentParameterPage("VCF Key Loop", "Off");
          startParameterDisplay();
        }
        midiCCOut710(VB_FILTER_LOOP_BIT0, 0);
        midiCCOut710(VB_FILTER_LOOP_BIT1, 0);
        if (wholemode) {
          midiCCOut810(VB_FILTER_LOOP_BIT0, 0);
          midiCCOut810(VB_FILTER_LOOP_BIT1, 0);
        }
        midiCCOut62(CCFilterLoop, 0);
        midiCCOut(CCFilterLoop, 0);
        mcp9.digitalWrite(VCF_LOOP_LED_RED, LOW);
        mcp10.digitalWrite(VCF_LOOP_LED_GREEN, LOW);
        break;

      case 1:
        if (announce) {
          showCurrentParameterPage("VCF LFO Loop", "Gated");
          startParameterDisplay();
        }
        midiCCOut710(VB_FILTER_LOOP_BIT0, 127);
        midiCCOut710(VB_FILTER_LOOP_BIT1, 0);
        if (wholemode) {
          midiCCOut810(VB_FILTER_LOOP_BIT0, 127);
          midiCCOut810(VB_FILTER_LOOP_BIT1, 0);
        }
        midiCCOut62(CCFilterLoop, 1);
        midiCCOut(CCFilterLoop, 63);
        mcp9.digitalWrite(VCF_LOOP_LED_RED, HIGH);
        mcp10.digitalWrite(VCF_LOOP_LED_GREEN, LOW);
        break;

      case 2:
        if (announce) {
          showCurrentParameterPage("VCF Looping", "LFO");
          startParameterDisplay();
        }
        midiCCOut710(VB_FILTER_LOOP_BIT0, 0);
        midiCCOut710(VB_FILTER_LOOP_BIT1, 127);
        if (wholemode) {
          midiCCOut810(VB_FILTER_LOOP_BIT0, 0);
          midiCCOut810(VB_FILTER_LOOP_BIT1, 127);
        }
        midiCCOut62(CCFilterLoop, 2);
        midiCCOut(CCFilterLoop, 127);
        mcp9.digitalWrite(VCF_LOOP_LED_RED, LOW);
        mcp10.digitalWrite(VCF_LOOP_LED_GREEN, HIGH);
        break;
    }
  }
}

FLASHMEM void updatefilterEGinv(boolean announce) {
  if (upperSW) {
    if (upperData[P_filterEGinv] == 0) {
      if (announce) {
        showCurrentParameterPage("Filter Env", "Positive");
        startParameterDisplay();
      }
      midiCCOut810(VB_EG_INVERT, 0);
      midiCCOut(CCfilterEGinv, 0);
      midiCCOut62(CCfilterEGinv, 0);
      mcp8.digitalWrite(VCF_EG_INV_LED, LOW);
    } else {
      if (announce) {
        showCurrentParameterPage("Filter Env", "Negative");
        startParameterDisplay();
      }
      midiCCOut810(VB_EG_INVERT, 127);
      midiCCOut(CCfilterEGinv, 127);
      midiCCOut62(CCfilterEGinv, 127);
      mcp8.digitalWrite(VCF_EG_INV_LED, HIGH);
    }
  } else {
    if (lowerData[P_filterEGinv] == 0) {
      if (announce) {
        showCurrentParameterPage("Filter Env", "Positive");
        startParameterDisplay();
      }
      midiCCOut710(VB_EG_INVERT, 0);
      if (wholemode) {
        midiCCOut810(VB_EG_INVERT, 0);
      }
      midiCCOut(CCfilterEGinv, 0);
      midiCCOut62(CCfilterEGinv, 0);
      mcp8.digitalWrite(VCF_EG_INV_LED, LOW);

    } else {
      if (announce) {
        showCurrentParameterPage("Filter Env", "Negative");
        startParameterDisplay();
      }
      midiCCOut710(VB_EG_INVERT, 127);
      if (wholemode) {
        midiCCOut810(VB_EG_INVERT, 127);
      }
      midiCCOut(CCfilterEGinv, 127);
      midiCCOut62(CCfilterEGinv, 127);
      mcp8.digitalWrite(VCF_EG_INV_LED, HIGH);
    }
  }
}

FLASHMEM void updatekeyTrackSW(boolean announce) {
  if (upperSW) {
    if (!upperData[P_keytrackSW]) {
      if (announce) {
        showCurrentParameterPage("Keytrack", "Off");
        startParameterDisplay();
      }
      midiCCOut89(CC_KEYTRACK_SW, 0);
      midiCCOut(CCkeyTrackSW, 0);
      midiCCOut62(CCkeyTrackSW, 0);
      mcp8.digitalWrite(VCF_KEYTRACK_LED, LOW);
    } else {
      if (announce) {
        showCurrentParameterPage("Keytrack", "On");
        startParameterDisplay();
      }
      midiCCOut89(CC_KEYTRACK_SW, 127);
      midiCCOut(CCkeyTrackSW, 127);
      midiCCOut62(CCkeyTrackSW, 1);
      mcp8.digitalWrite(VCF_KEYTRACK_LED, HIGH);
    }
  } else {
    if (!lowerData[P_keytrackSW]) {
      if (announce) {
        showCurrentParameterPage("Keytrack", "Off");
        startParameterDisplay();
      }
      midiCCOut79(CC_KEYTRACK_SW, 0);
      midiCCOut(CCkeyTrackSW, 0);
      midiCCOut62(CCkeyTrackSW, 0);
      if (wholemode) {
        midiCCOut89(CC_KEYTRACK_SW, 0);
      }
      mcp8.digitalWrite(VCF_KEYTRACK_LED, LOW);
    } else {
      if (announce) {
        showCurrentParameterPage("Keytrack", "On");
        startParameterDisplay();
      }
      midiCCOut79(CC_KEYTRACK_SW, 127);
      midiCCOut(CCkeyTrackSW, 127);
      midiCCOut62(CCkeyTrackSW, 1);
      if (wholemode) {
        midiCCOut89(CC_KEYTRACK_SW, 127);
      }
      mcp8.digitalWrite(VCF_KEYTRACK_LED, HIGH);
    }
  }
}

FLASHMEM void updatesyncSW(boolean announce) {
  if (upperSW) {
    if (upperData[P_sync] == 0) {
      if (announce) {
        showCurrentParameterPage("Sync", "Off");
        startParameterDisplay();
      }
      midiCCOut89(CC_SYNC_MODE, 0);
      midiCCOut(CCsyncSW, 0);
      midiCCOut62(CCsyncSW, 0);
      mcp7.digitalWrite(DCO2_SYNC_LED_RED, LOW);
      mcp7.digitalWrite(DCO2_SYNC_LED_GREEN, LOW);
    }
    if (upperData[P_sync] == 1) {
      if (announce) {
        showCurrentParameterPage("Sync", "Soft");
        startParameterDisplay();
      }
      midiCCOut89(CC_SYNC_MODE, 64);
      midiCCOut(CCsyncSW, 64);
      midiCCOut62(CCsyncSW, 1);
      mcp7.digitalWrite(DCO2_SYNC_LED_RED, HIGH);
      mcp7.digitalWrite(DCO2_SYNC_LED_GREEN, LOW);
    }
    if (upperData[P_sync] == 2) {
      if (announce) {
        showCurrentParameterPage("Sync", "Hard");
        startParameterDisplay();
      }
      midiCCOut89(CC_SYNC_MODE, 127);
      midiCCOut(CCsyncSW, 127);
      midiCCOut62(CCsyncSW, 2);
      mcp7.digitalWrite(DCO2_SYNC_LED_RED, LOW);
      mcp7.digitalWrite(DCO2_SYNC_LED_GREEN, HIGH);
    }
  } else {
    if (lowerData[P_sync] == 0) {
      if (announce) {
        showCurrentParameterPage("Sync", "Off");
        startParameterDisplay();
      }
      midiCCOut79(CC_SYNC_MODE, 0);
      midiCCOut(CCsyncSW, 0);
      midiCCOut62(CCsyncSW, 0);
      if (wholemode) {
        midiCCOut89(CC_SYNC_MODE, 0);
      }
      mcp7.digitalWrite(DCO2_SYNC_LED_RED, LOW);
      mcp7.digitalWrite(DCO2_SYNC_LED_GREEN, LOW);
    }
    if (lowerData[P_sync] == 1) {
      if (announce) {
        showCurrentParameterPage("Sync", "Soft");
        startParameterDisplay();
      }
      midiCCOut79(CC_SYNC_MODE, 64);
      midiCCOut(CCsyncSW, 127);
      midiCCOut62(CCsyncSW, 1);
      if (wholemode) {
        midiCCOut89(CC_SYNC_MODE, 64);
      }
      mcp7.digitalWrite(DCO2_SYNC_LED_RED, HIGH);
      mcp7.digitalWrite(DCO2_SYNC_LED_GREEN, LOW);
    }
    if (lowerData[P_sync] == 2) {
      if (announce) {
        showCurrentParameterPage("Sync", "Hard");
        startParameterDisplay();
      }
      midiCCOut79(CC_SYNC_MODE, 127);
      midiCCOut(CCsyncSW, 127);
      midiCCOut62(CCsyncSW, 2);
      if (wholemode) {
        midiCCOut89(CC_SYNC_MODE, 127);
      }
      mcp7.digitalWrite(DCO2_SYNC_LED_RED, LOW);
      mcp7.digitalWrite(DCO2_SYNC_LED_GREEN, HIGH);
    }
  }
}

void updatefootSwitch() {

  if (upperSW) {
    if (upperData[P_effectPot3] < 63) {
      upperslowpot3 = upperData[P_effectPot3];
      upperfast = true;
      upperslow = false;
    }
    if (upperData[P_effectPot3] > 63) {
      upperfastpot3 = upperData[P_effectPot3];
      upperfast = false;
      upperslow = true;
    }
  } else {
    if (lowerData[P_effectPot3] < 63) {
      lowerslowpot3 = lowerData[P_effectPot3];
      lowerfast = true;
      lowerslow = false;
    }
    if (lowerData[P_effectPot3] > 63) {
      lowerfastpot3 = lowerData[P_effectPot3];
      lowerfast = false;
      lowerslow = true;
    }
    if (wholemode) {
      if (upperData[P_effectPot3] < 63) {
        upperslowpot3 = upperData[P_effectPot3];
        upperfast = true;
        upperslow = false;
      }
      if (upperData[P_effectPot3] > 63) {
        upperfastpot3 = upperData[P_effectPot3];
        upperfast = false;
        upperslow = true;
      }
    }
  }
}

void changeSpeed() {
  static unsigned long lastStep = 0;
  unsigned long now = millis();

  // Only allow a step every SPEED_STEP_INTERVAL_MS
  if (now - lastStep < SPEED_STEP_INTERVAL_MS) return;
  lastStep = now;

  // ---------- UPPER SECTION ----------
  if (upperfootPedal) {
    if (upperslow) {
      if (upperData[P_effectPot3] > upperslowpot3) {
        upperData[P_effectPot3] -= 1;
        if (upperData[P_effectPot3] < upperslowpot3)
          upperData[P_effectPot3] = upperslowpot3;
        // Send MIDI only if changed
        if (upperData[P_effectPot3] != upperLastSentPot3) {
          midiCCOut810(VB_EFFECT_POT3, upperData[P_effectPot3]);
          midiCCOut61(CCeffectPot3, upperData[P_effectPot3]);
          upperLastSentPot3 = upperData[P_effectPot3];
        }
      } else {
        // Arrived at destination
        upperfootPedal = false;
        upperslow = false;
      }
    } else if (upperfast) {
      if (upperData[P_effectPot3] < upperfastpot3) {
        upperData[P_effectPot3] += 1;
        if (upperData[P_effectPot3] > upperfastpot3)
          upperData[P_effectPot3] = upperfastpot3;
        // Send MIDI only if changed
        if (upperData[P_effectPot3] != upperLastSentPot3) {
          midiCCOut810(VB_EFFECT_POT3, upperData[P_effectPot3]);
          midiCCOut61(CCeffectPot3, upperData[P_effectPot3]);
          upperLastSentPot3 = upperData[P_effectPot3];
        }
      } else {
        upperfootPedal = false;
        upperfast = false;
      }
    }
  }

  // ---------- LOWER SECTION ----------
  if (lowerfootPedal) {
    if (lowerslow) {
      if (lowerData[P_effectPot3] > lowerslowpot3) {
        lowerData[P_effectPot3] -= 1;
        if (lowerData[P_effectPot3] < lowerslowpot3)
          lowerData[P_effectPot3] = lowerslowpot3;
        if (lowerData[P_effectPot3] != lowerLastSentPot3) {
          midiCCOut710(VB_EFFECT_POT3, lowerData[P_effectPot3]);
          midiCCOut61(CCeffectPot3, lowerData[P_effectPot3]);
          lowerLastSentPot3 = lowerData[P_effectPot3];
        }
      } else {
        lowerfootPedal = false;
        lowerslow = false;
      }
    } else if (lowerfast) {
      if (lowerData[P_effectPot3] < lowerfastpot3) {
        lowerData[P_effectPot3] += 1;
        if (lowerData[P_effectPot3] > lowerfastpot3)
          lowerData[P_effectPot3] = lowerfastpot3;
        if (lowerData[P_effectPot3] != lowerLastSentPot3) {
          midiCCOut710(VB_EFFECT_POT3, lowerData[P_effectPot3]);
          midiCCOut61(CCeffectPot3, lowerData[P_effectPot3]);
          lowerLastSentPot3 = lowerData[P_effectPot3];
        }
      } else {
        lowerfootPedal = false;
        lowerfast = false;
      }
    }
  }
}

FLASHMEM void updatefilterenvLogLin(boolean announce) {

  if (upperSW) {
    if (!upperData[P_filterLogLin]) {
      if (announce) {
        showCurrentParameterPage("Filter Env", "Linear");
        startParameterDisplay();
      }
      midiCCOut810(VB_FILTER_LIN_LOG, 0);
      midiCCOut(CCfilterenvLinLogSW, 0);
      midiCCOut62(CCfilterenvLinLogSW, 0);
      mcp10.digitalWrite(VCF_LIN_LOG_LED_RED, HIGH);
      mcp10.digitalWrite(VCF_LIN_LOG_LED_GREEN, LOW);
    } else {
      if (announce) {
        showCurrentParameterPage("Filter Env", "Log");
        startParameterDisplay();
      }
      midiCCOut810(VB_FILTER_LIN_LOG, 127);
      midiCCOut(CCfilterenvLinLogSW, 127);
      midiCCOut62(CCfilterenvLinLogSW, 1);
      mcp10.digitalWrite(VCF_LIN_LOG_LED_RED, LOW);
      mcp10.digitalWrite(VCF_LIN_LOG_LED_GREEN, HIGH);
    }
  } else {
    if (!lowerData[P_filterLogLin]) {
      if (announce) {
        showCurrentParameterPage("Filter Env", "Linear");
        startParameterDisplay();
      }
      midiCCOut710(VB_FILTER_LIN_LOG, 0);
      if (wholemode) {
        midiCCOut810(VB_FILTER_LIN_LOG, 0);
      }
      midiCCOut(CCfilterenvLinLogSW, 0);
      midiCCOut62(CCfilterenvLinLogSW, 0);
      mcp10.digitalWrite(VCF_LIN_LOG_LED_RED, HIGH);
      mcp10.digitalWrite(VCF_LIN_LOG_LED_GREEN, LOW);
    } else {
      if (announce) {
        showCurrentParameterPage("Filter Env", "Log");
        startParameterDisplay();
      }
      midiCCOut710(VB_FILTER_LIN_LOG, 127);
      if (wholemode) {
        midiCCOut810(VB_FILTER_LIN_LOG, 127);
      }
      midiCCOut(CCfilterenvLinLogSW, 127);
      midiCCOut62(CCfilterenvLinLogSW, 1);
      mcp10.digitalWrite(VCF_LIN_LOG_LED_RED, LOW);
      mcp10.digitalWrite(VCF_LIN_LOG_LED_GREEN, HIGH);
    }
  }
}

FLASHMEM void updateampenvLogLin(boolean announce) {
  if (upperSW) {
    if (!upperData[P_ampLogLin]) {
      if (announce) {
        showCurrentParameterPage("Amp Env", "Linear");
        startParameterDisplay();
      }
      midiCCOut810(VB_AMP_LIN_LOG, 0);
      midiCCOut(CCampenvLinLogSW, 0);
      midiCCOut62(CCampenvLinLogSW, 0);
      mcp12.digitalWrite(AMP_LIN_LOG_LED_RED, HIGH);
      mcp12.digitalWrite(AMP_LIN_LOG_LED_GREEN, LOW);
    } else {
      if (announce) {
        showCurrentParameterPage("Amp Env", "Log");
        startParameterDisplay();
      }
      midiCCOut810(VB_AMP_LIN_LOG, 127);
      midiCCOut(CCampenvLinLogSW, 127);
      midiCCOut62(CCampenvLinLogSW, 1);
      mcp12.digitalWrite(AMP_LIN_LOG_LED_RED, LOW);
      mcp12.digitalWrite(AMP_LIN_LOG_LED_GREEN, HIGH);
    }
  } else {
    if (!lowerData[P_ampLogLin]) {
      if (announce) {
        showCurrentParameterPage("Amp Env", "Linear");
        startParameterDisplay();
      }
      midiCCOut710(VB_AMP_LIN_LOG, 0);
      if (wholemode) {
        midiCCOut810(VB_AMP_LIN_LOG, 0);
      }
      midiCCOut(CCampenvLinLogSW, 0);
      midiCCOut62(CCampenvLinLogSW, 0);
      mcp12.digitalWrite(AMP_LIN_LOG_LED_RED, HIGH);
      mcp12.digitalWrite(AMP_LIN_LOG_LED_GREEN, LOW);
    } else {
      if (announce) {
        showCurrentParameterPage("Amp Env", "Log");
        startParameterDisplay();
      }
      midiCCOut710(VB_AMP_LIN_LOG, 127);
      if (wholemode) {
        midiCCOut810(VB_AMP_LIN_LOG, 127);
      }
      midiCCOut(CCampenvLinLogSW, 127);
      midiCCOut62(CCampenvLinLogSW, 1);
      mcp12.digitalWrite(AMP_LIN_LOG_LED_RED, LOW);
      mcp12.digitalWrite(AMP_LIN_LOG_LED_GREEN, HIGH);
    }
  }
}

FLASHMEM void updatenoiseSrc(boolean announce) {
  if (upperSW) {
    if (!upperData[P_noiseSrc]) {
      if (announce) {
        showCurrentParameterPage("Noise Source", "White");
        startParameterDisplay();
      }
      midiCCOut810(VB_NOISE_SOURCE, 0);
      midiCCOut(CCnoiseSrc, 0);
      midiCCOut62(CCnoiseSrc, 0);
      mcp6.digitalWrite(NOISE_SRC_LED_RED, HIGH);
      mcp6.digitalWrite(NOISE_SRC_LED_GREEN, LOW);
    } else {
      if (announce) {
        showCurrentParameterPage("Noise Source", "Pink");
        startParameterDisplay();
      }
      midiCCOut810(VB_NOISE_SOURCE, 127);
      midiCCOut(CCnoiseSrc, 127);
      midiCCOut62(CCnoiseSrc, 1);
      mcp6.digitalWrite(NOISE_SRC_LED_RED, LOW);
      mcp6.digitalWrite(NOISE_SRC_LED_GREEN, HIGH);
    }
  } else {
    if (!lowerData[P_noiseSrc]) {
      if (announce) {
        showCurrentParameterPage("Noise Source", "White");
        startParameterDisplay();
      }
      midiCCOut710(VB_NOISE_SOURCE, 0);
      if (wholemode) {
        midiCCOut810(VB_NOISE_SOURCE, 0);
      }
      midiCCOut(CCnoiseSrc, 0);
      midiCCOut62(CCnoiseSrc, 0);
      mcp6.digitalWrite(NOISE_SRC_LED_RED, HIGH);
      mcp6.digitalWrite(NOISE_SRC_LED_GREEN, LOW);
    } else {
      if (announce) {
        showCurrentParameterPage("Noise Source", "Pink");
        startParameterDisplay();
      }
      midiCCOut710(VB_NOISE_SOURCE, 127);
      if (wholemode) {
        midiCCOut810(VB_NOISE_SOURCE, 127);
      }
      midiCCOut(CCnoiseSrc, 127);
      midiCCOut62(CCnoiseSrc, 1);
      mcp6.digitalWrite(NOISE_SRC_LED_RED, LOW);
      mcp6.digitalWrite(NOISE_SRC_LED_GREEN, HIGH);
    }
  }
}

FLASHMEM void updatedco_at_SW(boolean announce) {
  if (upperSW) {
    if (upperData[P_dco_at_SW] == 0) {
      if (announce) {
        showCurrentParameterPage("DCO Aftertouch", "Off");
        startParameterDisplay();
      }
      midiCCOut89(CC_AT_FM_ENABLE, 0);
      midiCCOut62(CCdco_at_SW, 0);
      midiCCOut(CCdco_at_SW, 0);
      mcp5.digitalWrite(DCO_AT_LED, LOW);
    } else {
      if (announce) {
        showCurrentParameterPage("DCO Aftertouch", "On");
        startParameterDisplay();
      }
      midiCCOut89(CC_AT_FM_ENABLE, 127);
      midiCCOut62(CCdco_at_SW, 1);
      midiCCOut(CCdco_at_SW, 127);
      mcp5.digitalWrite(DCO_AT_LED, HIGH);
    }
  } else {
    if (lowerData[P_dco_at_SW] == 0) {
      if (announce) {
        showCurrentParameterPage("DCO Aftertouch", "Off");
        startParameterDisplay();
      }
      midiCCOut79(CC_AT_FM_ENABLE, 0);
      if (wholemode) {
        midiCCOut89(CC_AT_FM_ENABLE, 0);
      }
      midiCCOut62(CCdco_at_SW, 0);
      midiCCOut(CCdco_at_SW, 0);
      mcp5.digitalWrite(DCO_AT_LED, LOW);
    } else {
      if (announce) {
        showCurrentParameterPage("DCO Aftertouch", "On");
        startParameterDisplay();
      }
      midiCCOut79(CC_AT_FM_ENABLE, 127);
      if (wholemode) {
        midiCCOut89(CC_AT_FM_ENABLE, 127);
      }
      midiCCOut62(CCdco_at_SW, 1);
      midiCCOut(CCdco_at_SW, 127);
      mcp5.digitalWrite(DCO_AT_LED, HIGH);
    }
  }
}

FLASHMEM void updatefilter_at_SW(boolean announce) {
  if (upperSW) {
    if (upperData[P_filter_at_SW] == 0) {
      if (announce) {
        showCurrentParameterPage("Filter Aftertouch", "Off");
        startParameterDisplay();
      }
      midiCCOut89(CC_AT_FILTER_ENABLE, 0);
      midiCCOut62(CCfilter_at_SW, 0);
      midiCCOut(CCfilter_at_SW, 0);
      mcp5.digitalWrite(FILTER_AT_LED, LOW);
    } else {
      if (announce) {
        showCurrentParameterPage("Filter Aftertouch", "On");
        startParameterDisplay();
      }
      midiCCOut89(CC_AT_FILTER_ENABLE, 127);
      midiCCOut62(CCfilter_at_SW, 1);
      midiCCOut(CCfilter_at_SW, 127);
      mcp5.digitalWrite(FILTER_AT_LED, HIGH);
    }
  } else {
    if (lowerData[P_filter_at_SW] == 0) {
      if (announce) {
        showCurrentParameterPage("Filter Aftertouch", "Off");
        startParameterDisplay();
      }
      midiCCOut79(CC_AT_FILTER_ENABLE, 0);
      if (wholemode) {
        midiCCOut89(CC_AT_FILTER_ENABLE, 0);
      }
      midiCCOut62(CCfilter_at_SW, 0);
      midiCCOut(CCfilter_at_SW, 0);
      mcp5.digitalWrite(FILTER_AT_LED, LOW);
    } else {
      if (announce) {
        showCurrentParameterPage("Filter Aftertouch", "On");
        startParameterDisplay();
      }
      midiCCOut79(CC_AT_FILTER_ENABLE, 127);
      if (wholemode) {
        midiCCOut89(CC_AT_FILTER_ENABLE, 127);
      }
      midiCCOut62(CCfilter_at_SW, 1);
      midiCCOut(CCfilter_at_SW, 127);
      mcp5.digitalWrite(FILTER_AT_LED, HIGH);
    }
  }
}

FLASHMEM void updatefilterVel(boolean announce) {
  if (upperSW) {
    if (upperData[P_filterVel] == 0) {
      if (announce) {
        showCurrentParameterPage("VCF Velocity", "Off");
        startParameterDisplay();
      }
      midiCCOut810(VB_FILTER_VELOCITY, 0);
      midiCCOut62(CCfilterVel, 0);
      midiCCOut(CCfilterVel, 0);
      mcp9.digitalWrite(VCF_VELOCITY_LED, LOW);
    } else {
      if (announce) {
        showCurrentParameterPage("VCF Velocity", "On");
        startParameterDisplay();
      }
      midiCCOut810(VB_FILTER_VELOCITY, 127);
      midiCCOut62(CCfilterVel, 1);
      midiCCOut(CCfilterVel, 127);
      mcp9.digitalWrite(VCF_VELOCITY_LED, HIGH);
    }
  } else {
    if (lowerData[P_filterVel] == 0) {
      if (announce) {
        showCurrentParameterPage("VCF Velocity", "Off");
        startParameterDisplay();
      }
      midiCCOut710(VB_FILTER_VELOCITY, 0);
      if (wholemode) {
        midiCCOut810(VB_FILTER_VELOCITY, 0);
      }
      midiCCOut62(CCfilterVel, 0);
      midiCCOut(CCfilterVel, 0);
      mcp9.digitalWrite(VCF_VELOCITY_LED, LOW);
    } else {
      if (announce) {
        showCurrentParameterPage("VCF Velocity", "On");
        startParameterDisplay();
      }
      midiCCOut710(VB_FILTER_VELOCITY, 127);
      if (wholemode) {
        midiCCOut810(VB_FILTER_VELOCITY, 127);
      }
      midiCCOut62(CCfilterVel, 1);
      midiCCOut(CCfilterVel, 127);
      mcp9.digitalWrite(VCF_VELOCITY_LED, HIGH);
    }
  }
}

void updateNotePriority(boolean announce) {
  if (upperSW) {
    if (dualmode) {
      lowerData[P_NotePriority] = upperData[P_NotePriority];
    }
    switch (upperData[P_NotePriority]) {
      case 0:
        if (announce) {
          showCurrentParameterPage("Note Priority", "Top");
        }
        mcp4.digitalWrite(PRIORITY_LED_RED, LOW);
        mcp4.digitalWrite(PRIORITY_LED_GREEN, HIGH);
        midiCCOut62(CCNotePriority, 0);
        midiCCOut(CCNotePriority, 0);
        break;

      case 1:
        if (announce) {
          showCurrentParameterPage("Note Priority", "Bottom");
        }
        mcp4.digitalWrite(PRIORITY_LED_RED, HIGH);
        mcp4.digitalWrite(PRIORITY_LED_GREEN, LOW);
        midiCCOut62(CCNotePriority, 1);
        midiCCOut(CCNotePriority, 63);
        break;

      case 2:
        if (announce) {
          showCurrentParameterPage("Note Priority", "Last");
        }
        mcp4.digitalWrite(PRIORITY_LED_RED, HIGH);
        mcp4.digitalWrite(PRIORITY_LED_GREEN, HIGH);
        midiCCOut62(CCNotePriority, 2);
        midiCCOut(CCNotePriority, 127);
        break;
    }
  } else {
    if (dualmode) {
      upperData[P_NotePriority] = lowerData[P_NotePriority];
    }
    switch (lowerData[P_NotePriority]) {
      case 0:
        if (announce) {
          showCurrentParameterPage("Note Priority", "Top");
        }
        mcp4.digitalWrite(PRIORITY_LED_RED, LOW);
        mcp4.digitalWrite(PRIORITY_LED_GREEN, HIGH);
        midiCCOut61(CCNotePriority, 0);
        midiCCOut(CCNotePriority, 0);
        break;

      case 1:
        if (announce) {
          showCurrentParameterPage("Note Priority", "Bottom");
        }
        mcp4.digitalWrite(PRIORITY_LED_RED, HIGH);
        mcp4.digitalWrite(PRIORITY_LED_GREEN, LOW);
        midiCCOut61(CCNotePriority, 1);
        midiCCOut(CCNotePriority, 63);
        break;

      case 2:
        if (announce) {
          showCurrentParameterPage("Note Priority", "Last");
        }
        mcp4.digitalWrite(PRIORITY_LED_RED, HIGH);
        mcp4.digitalWrite(PRIORITY_LED_GREEN, HIGH);
        midiCCOut61(CCNotePriority, 2);
        midiCCOut(CCNotePriority, 127);
        break;
    }
  }
}

FLASHMEM void updatevcaLoop(boolean announce) {
  if (upperSW) {
    switch (upperData[P_vcaLoop]) {
      case 0:
        if (announce) {
          showCurrentParameterPage("VCA Loop", "Off");
          startParameterDisplay();
        }
        midiCCOut810(VB_AMP_LOOP_BIT0, 0);
        midiCCOut810(VB_AMP_LOOP_BIT1, 0);
        midiCCOut62(CCAmpLoop, 0);
        midiCCOut(CCAmpLoop, 0);
        mcp11.digitalWrite(AMP_LOOP_LED_RED, LOW);
        mcp12.digitalWrite(AMP_LOOP_LED_GREEN, LOW);
        break;

      case 1:
        if (announce) {
          showCurrentParameterPage("VCA Loop", "Gated");
          startParameterDisplay();
        }
        midiCCOut810(VB_AMP_LOOP_BIT0, 127);
        midiCCOut810(VB_AMP_LOOP_BIT1, 0);
        midiCCOut62(CCAmpLoop, 1);
        midiCCOut(CCAmpLoop, 63);
        mcp11.digitalWrite(AMP_LOOP_LED_RED, HIGH);
        mcp12.digitalWrite(AMP_LOOP_LED_GREEN, LOW);
        break;

      case 2:
        if (announce) {
          showCurrentParameterPage("VCA Loop", "LFO");
          startParameterDisplay();
        }
        midiCCOut810(VB_AMP_LOOP_BIT0, 0);
        midiCCOut810(VB_AMP_LOOP_BIT1, 127);
        midiCCOut62(CCAmpLoop, 2);
        midiCCOut(CCAmpLoop, 127);
        mcp11.digitalWrite(AMP_LOOP_LED_RED, LOW);
        mcp12.digitalWrite(AMP_LOOP_LED_GREEN, HIGH);
        break;
    }
  } else {
    switch (lowerData[P_vcaLoop]) {
      case 0:
        if (announce) {
          showCurrentParameterPage("VCA Loop", "Off");
          startParameterDisplay();
        }
        midiCCOut710(VB_AMP_LOOP_BIT0, 0);
        midiCCOut710(VB_AMP_LOOP_BIT1, 0);
        if (wholemode) {
          midiCCOut810(VB_AMP_LOOP_BIT0, 0);
          midiCCOut810(VB_AMP_LOOP_BIT1, 0);
        }
        midiCCOut62(CCAmpLoop, 0);
        midiCCOut(CCAmpLoop, 0);
        mcp11.digitalWrite(AMP_LOOP_LED_RED, LOW);
        mcp12.digitalWrite(AMP_LOOP_LED_GREEN, LOW);
        break;

      case 1:
        if (announce) {
          showCurrentParameterPage("VCA Loop", "Gated");
          startParameterDisplay();
        }
        midiCCOut710(VB_AMP_LOOP_BIT0, 127);
        midiCCOut710(VB_AMP_LOOP_BIT1, 0);
        if (wholemode) {
          midiCCOut810(VB_AMP_LOOP_BIT0, 127);
          midiCCOut810(VB_AMP_LOOP_BIT1, 0);
        }
        midiCCOut62(CCAmpLoop, 1);
        midiCCOut(CCAmpLoop, 63);
        mcp11.digitalWrite(AMP_LOOP_LED_RED, HIGH);
        mcp12.digitalWrite(AMP_LOOP_LED_GREEN, LOW);
        break;

      case 2:
        if (announce) {
          showCurrentParameterPage("VCA Loop", "LFO");
          startParameterDisplay();
        }
        midiCCOut710(VB_AMP_LOOP_BIT0, 0);
        midiCCOut710(VB_AMP_LOOP_BIT1, 127);
        if (wholemode) {
          midiCCOut810(VB_AMP_LOOP_BIT0, 0);
          midiCCOut810(VB_AMP_LOOP_BIT1, 127);
        }
        midiCCOut62(CCAmpLoop, 2);
        midiCCOut(CCAmpLoop, 127);
        mcp11.digitalWrite(AMP_LOOP_LED_RED, LOW);
        mcp12.digitalWrite(AMP_LOOP_LED_GREEN, HIGH);
        break;
    }
  }
}

FLASHMEM void updatevcaVel(boolean announce) {
  if (upperSW) {
    if (upperData[P_vcaVel] == 0) {
      if (announce) {
        showCurrentParameterPage("VCA Velocity", "Off");
        startParameterDisplay();
      }
      midiCCOut810(VB_AMP_VELOCITY, 0);
      midiCCOut62(CCvcaVel, 0);
      midiCCOut(CCvcaVel, 0);
      mcp11.digitalWrite(AMP_VELOCITY_LED, LOW);
    } else {
      if (announce) {
        showCurrentParameterPage("VCA Velocity", "On");
        startParameterDisplay();
      }
      midiCCOut810(VB_AMP_VELOCITY, 127);
      midiCCOut62(CCvcaVel, 1);
      midiCCOut(CCvcaVel, 127);
      mcp11.digitalWrite(AMP_VELOCITY_LED, HIGH);
    }
  } else {
    if (lowerData[P_vcaVel] == 0) {
      if (announce) {
        showCurrentParameterPage("VCA Velocity", "Off");
        startParameterDisplay();
      }
      midiCCOut710(VB_AMP_VELOCITY, 0);
      if (wholemode) {
        midiCCOut810(VB_AMP_VELOCITY, 0);
      }
      midiCCOut62(CCvcaVel, 0);
      midiCCOut(CCvcaVel, 0);
      mcp11.digitalWrite(AMP_VELOCITY_LED, LOW);
    } else {
      if (announce) {
        showCurrentParameterPage("VCA Velocity", "On");
        startParameterDisplay();
      }
      midiCCOut710(VB_AMP_VELOCITY, 127);
      if (wholemode) {
        midiCCOut810(VB_AMP_VELOCITY, 127);
      }
      midiCCOut62(CCvcaVel, 1);
      midiCCOut(CCvcaVel, 127);
      mcp11.digitalWrite(AMP_VELOCITY_LED, HIGH);
    }
  }
}

FLASHMEM void updatevcaGate(boolean announce) {
  if (upperSW) {
    if (!upperData[P_vcaGate]) {
      if (announce) {
        showCurrentParameterPage("VCA Gate", "Off");
        startParameterDisplay();
      }
      midiCCOut(CCvcaGate, 0);
      midiCCOut62(CCvcaGate, 0);
      upperData[P_ampAttack] = upperData[P_oldampAttack];
      upperData[P_ampDecay] = upperData[P_oldampDecay];
      upperData[P_ampSustain] = upperData[P_oldampSustain];
      upperData[P_ampRelease] = upperData[P_oldampRelease];
      midiCCOut810(VB_VCA_ATTACK, upperData[P_ampAttack]);
      midiCCOut810(VB_VCA_DECAY, upperData[P_ampDecay]);
      midiCCOut810(VB_VCA_SUSTAIN, upperData[P_ampSustain]);
      midiCCOut810(VB_VCA_RELEASE, upperData[P_ampRelease]);
      mcp13.digitalWrite(AMP_ENV_GATE_LED, LOW);
    } else {
      if (announce) {
        showCurrentParameterPage("VCA Gate", "On");
        startParameterDisplay();
      }
      midiCCOut(CCvcaGate, 127);
      midiCCOut62(CCvcaGate, 1);
      midiCCOut810(VB_VCA_ATTACK, 0);
      midiCCOut810(VB_VCA_DECAY, 0);
      midiCCOut810(VB_VCA_SUSTAIN, 127);
      midiCCOut810(VB_VCA_RELEASE, 0);
      mcp13.digitalWrite(AMP_ENV_GATE_LED, HIGH);
    }
  } else {
    if (!lowerData[P_vcaGate]) {
      if (announce) {
        showCurrentParameterPage("VCA Gate", "Off");
        startParameterDisplay();
      }
      midiCCOut(CCvcaGate, 0);
      midiCCOut62(CCvcaGate, 0);
      lowerData[P_ampAttack] = lowerData[P_oldampAttack];
      lowerData[P_ampDecay] = lowerData[P_oldampDecay];
      lowerData[P_ampSustain] = lowerData[P_oldampSustain];
      lowerData[P_ampRelease] = lowerData[P_oldampRelease];
      midiCCOut710(VB_VCA_ATTACK, upperData[P_ampAttack]);
      midiCCOut710(VB_VCA_DECAY, upperData[P_ampDecay]);
      midiCCOut710(VB_VCA_SUSTAIN, upperData[P_ampSustain]);
      midiCCOut710(VB_VCA_RELEASE, upperData[P_ampRelease]);
      mcp13.digitalWrite(AMP_ENV_GATE_LED, LOW);
      if (wholemode) {
        upperData[P_ampAttack] = upperData[P_oldampAttack];
        upperData[P_ampDecay] = upperData[P_oldampDecay];
        upperData[P_ampSustain] = upperData[P_oldampSustain];
        upperData[P_ampRelease] = upperData[P_oldampRelease];
        midiCCOut810(VB_VCA_ATTACK, upperData[P_ampAttack]);
        midiCCOut810(VB_VCA_DECAY, upperData[P_ampDecay]);
        midiCCOut810(VB_VCA_SUSTAIN, upperData[P_ampSustain]);
        midiCCOut810(VB_VCA_RELEASE, upperData[P_ampRelease]);
      }
    } else {
      if (announce) {
        showCurrentParameterPage("VCA Gate", "On");
        startParameterDisplay();
      }
      midiCCOut(CCvcaGate, 127);
      midiCCOut62(CCvcaGate, 1);
      midiCCOut710(VB_VCA_ATTACK, 0);
      midiCCOut710(VB_VCA_DECAY, 0);
      midiCCOut710(VB_VCA_SUSTAIN, 127);
      midiCCOut710(VB_VCA_RELEASE, 0);
      mcp13.digitalWrite(AMP_ENV_GATE_LED, HIGH);
      if (wholemode) {
        midiCCOut810(VB_VCA_ATTACK, 0);
        midiCCOut810(VB_VCA_DECAY, 0);
        midiCCOut810(VB_VCA_SUSTAIN, 127);
        midiCCOut810(VB_VCA_RELEASE, 0);
      }
    }
  }
}

FLASHMEM void updateupperSW(boolean announce) {
  if (!wholemode) {
    if (upperSW) {
      mcp3.digitalWrite(UPPER_LED, HIGH);
      mcp3.digitalWrite(LOWER_LED, LOW);
      midiCCOut62(CClowerSW, 0);
      midiCCOut62(CCupperSW, 1);
      upperParamsToDisplay();
      setAllButtons();
      //srp.writePin(UPPER_RELAY_1, HIGH);
    }
  }
}

FLASHMEM void updatelowerSW(boolean announce) {
  if (lowerSW) {
    mcp3.digitalWrite(UPPER_LED, LOW);
    mcp3.digitalWrite(LOWER_LED, HIGH);
    midiCCOut62(CCupperSW, 0);
    midiCCOut62(CClowerSW, 1);
    lowerParamsToDisplay();
    setAllButtons();
    //srp.writePin(UPPER_RELAY_1, LOW);
  }
}

FLASHMEM void updateMonoMulti(boolean announce) {
  if (upperSW) {
    if (!upperData[P_monoMulti]) {
      if (announce) {
        showCurrentParameterPage("LFO 3 Retrigger", "Off");
        startParameterDisplay();
      }
      midiCCOut(CCmonoMulti, 0);
      midiCCOut62(CCmonoMulti, 0);
      mcp13.digitalWrite(LFO3_RETRIG_LED, LOW);
    } else {
      if (announce) {
        showCurrentParameterPage("LFO 3 Retrigger", "On");
        startParameterDisplay();
      }
      midiCCOut(CCmonoMulti, 127);
      midiCCOut62(CCmonoMulti, 1);
      mcp13.digitalWrite(LFO3_RETRIG_LED, HIGH);
    }
  } else {
    if (!lowerData[P_monoMulti]) {
      if (announce) {
        showCurrentParameterPage("LFO 3 Retrigger", "Off");
        startParameterDisplay();
      }
      midiCCOut(CCmonoMulti, 0);
      midiCCOut62(CCmonoMulti, 0);
      if (wholemode) {
        upperData[P_monoMulti] = lowerData[P_monoMulti];
      }
      mcp13.digitalWrite(LFO3_RETRIG_LED, LOW);
    } else {
      if (announce) {
        showCurrentParameterPage("LFO 3 Retrigger", "On");
        startParameterDisplay();
      }
      midiCCOut(CCmonoMulti, 127);
      midiCCOut62(CCmonoMulti, 1);
      if (wholemode) {
        upperData[P_monoMulti] = lowerData[P_monoMulti];
      }
      mcp13.digitalWrite(LFO3_RETRIG_LED, HIGH);
    }
  }
}

FLASHMEM void updateLFO1retrig(boolean announce) {
  if (upperSW) {
    if (!upperData[P_lfo1retrig]) {
      if (announce) {
        showCurrentParameterPage("LFO 1 Retrigger", "Off");
        startParameterDisplay();
      }
      midiCCOut89(CC_LFO1_RETRIG, 0);
      midiCCOut(CClfo1retrig, 0);
      midiCCOut62(CClfo1retrig, 0);
      mcp14.digitalWrite(LFO1_RETRIG_LED, LOW);
    } else {
      if (announce) {
        showCurrentParameterPage("LFO 1 Retrigger", "On");
        startParameterDisplay();
      }
      midiCCOut89(CC_LFO1_RETRIG, 127);
      midiCCOut(CClfo1retrig, 127);
      midiCCOut62(CClfo1retrig, 1);
      mcp14.digitalWrite(LFO1_RETRIG_LED, HIGH);
    }
  } else {
    if (!lowerData[P_lfo1retrig]) {
      if (announce) {
        showCurrentParameterPage("LFO 1 Retrigger", "Off");
        startParameterDisplay();
      }
      midiCCOut79(CC_LFO1_RETRIG, 0);
      midiCCOut(CClfo1retrig, 0);
      midiCCOut62(CClfo1retrig, 0);
      if (wholemode) {
        midiCCOut89(CC_LFO1_RETRIG, 0);
      }
      mcp14.digitalWrite(LFO1_RETRIG_LED, LOW);
    } else {
      if (announce) {
        showCurrentParameterPage("LFO 1 Retrigger", "On");
        startParameterDisplay();
      }
      midiCCOut79(CC_LFO1_RETRIG, 127);
      midiCCOut(CClfo1retrig, 127);
      midiCCOut62(CClfo1retrig, 1);
      if (wholemode) {
        midiCCOut89(CC_LFO1_RETRIG, 127);
      }
      mcp14.digitalWrite(LFO1_RETRIG_LED, HIGH);
    }
  }
}

void startParameterDisplay() {
  refreshScreen();

  lastDisplayTriggerTime = millis();
  waitingToUpdate = true;
}

void updatePatchname() {
  refreshPatchDisplayFromState();
}

void myControlChange(byte channel, byte control, int value) {

  switch (control) {

    case CCglideTime:
      if (upperSW) {
        upperData[P_glideTime] = value;
      } else {
        lowerData[P_glideTime] = value;
        if (wholemode) {
          upperData[P_glideTime] = value;
        }
      }
      glideTimestr = LINEAR[value];
      updateglideTime(1);
      break;

    case CCLFO2Rate:
      if (upperSW) {
        upperData[P_LFO2Rate] = value;
      } else {
        lowerData[P_LFO2Rate] = value;
        if (wholemode) {
          upperData[P_LFO2Rate] = value;
        }
      }
      LFO2Ratestr = LFOTEMPO[value];  // for display
      updateLFO2Rate(1);
      break;

    case CCfmDepth:
      if (upperSW) {
        upperData[P_fmDepth] = value;
      } else {
        lowerData[P_fmDepth] = value;
        if (wholemode) {
          upperData[P_fmDepth] = value;
        }
      }
      fmDepthstr = value;
      updatefmDepth(1);
      break;

    case CCosc2PW:
      if (upperSW) {
        upperData[P_osc2PW] = value;
      } else {
        lowerData[P_osc2PW] = value;
        if (wholemode) {
          upperData[P_osc2PW] = value;
        }
      }
      osc2PWstr = PULSEWIDTH[value];
      updateosc2PW(1);
      break;

    case CCosc2PWM:
      if (upperSW) {
        upperData[P_osc2PWM] = value;
      } else {
        lowerData[P_osc2PWM] = value;
        if (wholemode) {
          upperData[P_osc2PWM] = value;
        }
      }
      osc2PWMstr = value;
      updateosc2PWM(1);
      break;

    case CCosc1PW:
      if (upperSW) {
        upperData[P_osc1PW] = value;
      } else {
        lowerData[P_osc1PW] = value;
        if (wholemode) {
          upperData[P_osc1PW] = value;
        }
      }
      osc1PWstr = PULSEWIDTH[value];
      updateosc1PW(1);
      break;

    case CCosc1PWM:
      if (upperSW) {
        upperData[P_osc1PWM] = value;
      } else {
        lowerData[P_osc1PWM] = value;
        if (wholemode) {
          upperData[P_osc1PWM] = value;
        }
      }
      osc1PWMstr = value;
      updateosc1PWM(1);
      break;

    case CCosc1envPWM:
      if (upperSW) {
        upperData[P_osc1envPWM] = value;
      } else {
        lowerData[P_osc1envPWM] = value;
        if (wholemode) {
          upperData[P_osc1envPWM] = value;
        }
      }
      osc1PWMstr = value;
      updateosc1envPWM(1);
      break;

    case CCosc2envPWM:
      if (upperSW) {
        upperData[P_osc2envPWM] = value;
      } else {
        lowerData[P_osc2envPWM] = value;
        if (wholemode) {
          upperData[P_osc2envPWM] = value;
        }
      }
      osc2PWMstr = value;
      updateosc2envPWM(1);
      break;

    case CCosc1Oct:
      if (upperSW) {
        upperData[P_osc1Range] = value;
      } else {
        lowerData[P_osc1Range] = value;
        if (wholemode) {
          upperData[P_osc1Range] = value;
        }
      }
      updateosc1Range(1);
      break;

    case CCosc2Oct:
      if (upperSW) {
        upperData[P_osc2Range] = value;
      } else {
        lowerData[P_osc2Range] = value;
        if (wholemode) {
          upperData[P_osc2Range] = value;
        }
      }
      updateosc2Range(1);
      break;

    case CCosc2Detune:
      if (upperSW) {
        upperData[P_osc2Detune] = value;
      } else {
        lowerData[P_osc2Detune] = value;
        if (wholemode) {
          upperData[P_osc2Detune] = value;
        }
      }
      osc2Detunestr = PULSEWIDTH[value];
      updateosc2Detune(1);
      break;

    case CCosc2Interval:
      if (upperSW) {
        upperData[P_osc2Interval] = value;
      } else {
        lowerData[P_osc2Interval] = value;
        if (wholemode) {
          upperData[P_osc2Interval] = value;
        }
      }
      osc2Intervalstr = value;
      updateosc2Interval(1);
      break;

    case CCATDepth:
      if (upperSW) {
        upperData[P_ATDepth] = value;
      } else {
        lowerData[P_ATDepth] = value;
        if (wholemode) {
          upperData[P_ATDepth] = value;
        }
      }
      ATDepthstr = value;
      updateATDepth(1);
      break;

    case CCnoiseLevel:
      if (upperSW) {
        upperData[P_noiseLevel] = value;
      } else {
        lowerData[P_noiseLevel] = value;
        if (wholemode) {
          upperData[P_noiseLevel] = value;
        }
      }
      noiseLevelstr = value;
      updatenoiseLevel(1);
      break;

    case CCosc2SawLevel:
      if (upperSW) {
        upperData[P_osc2SawLevel] = value;
      } else {
        lowerData[P_osc2SawLevel] = value;
        if (wholemode) {
          upperData[P_osc2SawLevel] = value;
        }
      }
      osc2SawLevelstr = value;  // for display
      updateOsc2SawLevel(1);
      break;

    case CCosc1SawLevel:
      if (upperSW) {
        upperData[P_osc1SawLevel] = value;
      } else {
        lowerData[P_osc1SawLevel] = value;
        if (wholemode) {
          upperData[P_osc1SawLevel] = value;
        }
      }
      osc1SawLevelstr = value;  // for display
      updateOsc1SawLevel(1);
      break;

    case CCosc2PulseLevel:
      if (upperSW) {
        upperData[P_osc2PulseLevel] = value;
      } else {
        lowerData[P_osc2PulseLevel] = value;
        if (wholemode) {
          upperData[P_osc2PulseLevel] = value;
        }
      }
      osc2PulseLevelstr = value;  // for display
      updateOsc2PulseLevel(1);
      break;

    case CCosc1PulseLevel:
      if (upperSW) {
        upperData[P_osc1PulseLevel] = value;
      } else {
        lowerData[P_osc1PulseLevel] = value;
        if (wholemode) {
          upperData[P_osc1PulseLevel] = value;
        }
      }
      osc1PulseLevelstr = value;  // for display
      updateOsc1PulseLevel(1);
      break;

    case CCosc2TriangleLevel:
      if (upperSW) {
        upperData[P_osc2TriangleLevel] = value;
      } else {
        lowerData[P_osc2TriangleLevel] = value;
        if (wholemode) {
          upperData[P_osc2TriangleLevel] = value;
        }
      }
      osc2TriangleLevelstr = value;  // for display
      updateOsc2TriangleLevel(1);
      break;

    case CCosc1SubLevel:
      if (upperSW) {
        upperData[P_osc1SubLevel] = value;
      } else {
        lowerData[P_osc1SubLevel] = value;
        if (wholemode) {
          upperData[P_osc1SubLevel] = value;
        }
      }
      osc1SubLevelstr = value;  // for display
      updateOsc1SubLevel(1);
      break;

    case CCosc2EnvDepth:
      if (upperSW) {
        upperData[P_osc2envDepth] = value;
      } else {
        lowerData[P_osc2envDepth] = value;
        if (wholemode) {
          upperData[P_osc2envDepth] = value;
        }
      }
      osc2envDepthstr = value;  // for display
      updateOsc2EnvDepth(1);
      break;

    case CCLFO1Delay:
      if (upperSW) {
        upperData[P_LFO1Delay] = value;
      } else {
        lowerData[P_LFO1Delay] = value;
        if (wholemode) {
          upperData[P_LFO1Delay] = value;
        }
      }
      LFO1Delaystr = value;  // for display
      updateLFO1Delay(1);
      break;

    case CCLFO3Delay:
      if (upperSW) {
        upperData[P_LFO3Delay] = value;
      } else {
        lowerData[P_LFO3Delay] = value;
        if (wholemode) {
          upperData[P_LFO3Delay] = value;
        }
      }
      LFO3Delaystr = value;  // for display
      updateLFO3Delay(1);
      break;

    case CCLFO1Slope:
      if (upperSW) {
        upperData[P_LFO1Slope] = value;
      } else {
        lowerData[P_LFO1Slope] = value;
        if (wholemode) {
          upperData[P_LFO1Slope] = value;
        }
      }
      LFO1Slopestr = value;  // for display
      updateLFO1Slope(1);
      break;

    case CCfilterCutoff:
      if (upperSW) {
        upperData[P_filterCutoff] = value;
        oldfilterCutoffU = value;
      } else {
        lowerData[P_filterCutoff] = value;
        oldfilterCutoffL = value;
        if (wholemode) {
          upperData[P_filterCutoff] = value;
          oldfilterCutoffU = value;
        }
      }
      filterCutoffstr = FILTERCUTOFF[value];
      updateFilterCutoff(1);
      break;

    case CCfilterLFO:
      if (upperSW) {
        upperData[P_filterLFO] = value;
      } else {
        lowerData[P_filterLFO] = value;
        if (wholemode) {
          upperData[P_filterLFO] = value;
        }
      }
      filterLFOstr = value;
      updatefilterLFO(1);
      break;

    case CCfilterRes:
      if (upperSW) {
        upperData[P_filterRes] = value;
      } else {
        lowerData[P_filterRes] = value;
        if (wholemode) {
          upperData[P_filterRes] = value;
        }
      }
      filterResstr = int(value);
      updatefilterRes(1);
      break;

    case CCfilterType:
      if (upperSW) {
        upperData[P_filterType] = value;
      } else {
        lowerData[P_filterType] = value;
        if (wholemode) {
          upperData[P_filterType] = value;
        }
      }
      updateFilterType(1);
      break;

    case CCfilterEGlevel:
      if (upperSW) {
        upperData[P_filterEGlevel] = value;
      } else {
        lowerData[P_filterEGlevel] = value;
        if (wholemode) {
          upperData[P_filterEGlevel] = value;
        }
      }
      filterEGlevelstr = int(value);
      updatefilterEGlevel(1);
      break;

    case CCLFO1Rate:
      if (upperSW) {
        upperData[P_LFO1Rate] = value;
      } else {
        lowerData[P_LFO1Rate] = value;
        if (wholemode) {
          upperData[P_LFO1Rate] = value;
        }
      }
      LFO1Ratestr = LFOTEMPO[value];  // for display
      updateLFO1Rate(1);
      break;

    case CCarpRate:
      lowerData[P_arpRate] = value;
      arpRatestr = LFOTEMPO[value];  // for display
      updatearpRate(1);
      break;

    case CCLFO3Rate:
      if (upperSW) {
        upperData[P_LFO3Rate] = value;
      } else {
        lowerData[P_LFO3Rate] = value;
        if (wholemode) {
          upperData[P_LFO3Rate] = value;
        }
      }
      LFO3Ratestr = LFOTEMPO[value];  // for display
      updateLFO3Rate(1);
      break;

    case CCmodWheelDepth:
      if (upperSW) {
        upperData[P_modWheelDepth] = value;
      } else {
        lowerData[P_modWheelDepth] = value;
        if (wholemode) {
          upperData[P_modWheelDepth] = value;
        }
      }
      modWheelDepthstr = value;  // for display
      updatemodWheelDepth(1);
      break;

    case CCPitchBend:
      if (upperSW) {
        upperData[P_PitchBendLevel] = value;
      } else {
        lowerData[P_PitchBendLevel] = value;
        if (wholemode) {
          upperData[P_PitchBendLevel] = value;
        }
      }
      PitchBendLevelstr = value;  // for display
      updatePitchBendDepth(1);
      break;

    case CCeffectPot1:
      if (upperSW) {
        upperData[P_effectPot1] = value;
      } else {
        lowerData[P_effectPot1] = value;
        if (wholemode) {
          upperData[P_effectPot1] = value;
        }
      }
      effectPot1str = value;  // for display
      updateeffectPot1(1);
      break;

    case CCeffectPot2:
      if (upperSW) {
        upperData[P_effectPot2] = value;
      } else {
        lowerData[P_effectPot2] = value;
        if (wholemode) {
          upperData[P_effectPot2] = value;
        }
      }
      effectPot2str = value;  // for display
      updateeffectPot2(1);
      break;

    case CCeffectPot3:
      if (upperSW) {
        upperData[P_effectPot3] = value;
      } else {
        lowerData[P_effectPot3] = value;
        if (wholemode) {
          upperData[P_effectPot3] = value;
        }
      }
      effectPot3str = value;  // for display
      updateeffectPot3(1);
      break;

    case CCeffectsMix:
      if (upperSW) {
        upperData[P_effectsMix] = value;
      } else {
        lowerData[P_effectsMix] = value;
        if (wholemode) {
          upperData[P_effectsMix] = value;
        }
      }
      effectsMixstr = LINEARCENTREZERO[value];  // for display
      updateeffectsMix(1);
      break;

    case CCpitchAttack:
      if (upperSW) {
        upperData[P_pitchAttack] = value;
      } else {
        lowerData[P_pitchAttack] = value;
        if (wholemode) {
          upperData[P_pitchAttack] = value;
        }
      }
      pitchAttackstr = ENVTIMES[value];
      updatepitchAttack(1);
      break;

    case CCpitchDecay:
      if (upperSW) {
        upperData[P_pitchDecay] = value;
      } else {
        lowerData[P_pitchDecay] = value;
        if (wholemode) {
          upperData[P_pitchDecay] = value;
        }
      }
      pitchDecaystr = ENVTIMES[value];
      updatepitchDecay(1);
      break;

    case CCpitchSustain:
      if (upperSW) {
        upperData[P_pitchSustain] = value;
      } else {
        lowerData[P_pitchSustain] = value;
        if (wholemode) {
          upperData[P_pitchSustain] = value;
        }
      }
      pitchSustainstr = LINEAR_FILTERMIXERSTR[value];
      updatepitchSustain(1);
      break;

    case CCpitchRelease:
      if (upperSW) {
        upperData[P_pitchRelease] = value;
      } else {
        lowerData[P_pitchRelease] = value;
        if (wholemode) {
          upperData[P_pitchRelease] = value;
        }
      }
      pitchReleasestr = ENVTIMES[value];
      updatepitchRelease(1);
      break;

    case CCfilterAttack:
      if (upperSW) {
        upperData[P_filterAttack] = value;
      } else {
        lowerData[P_filterAttack] = value;
        if (wholemode) {
          upperData[P_filterAttack] = value;
        }
      }
      filterAttackstr = ENVTIMES[value];
      updatefilterAttack(1);
      break;

    case CCfilterDecay:
      if (upperSW) {
        upperData[P_filterDecay] = value;
      } else {
        lowerData[P_filterDecay] = value;
        if (wholemode) {
          upperData[P_filterDecay] = value;
        }
      }
      filterDecaystr = ENVTIMES[value];
      updatefilterDecay(1);
      break;

    case CCfilterSustain:
      if (upperSW) {
        upperData[P_filterSustain] = value;
      } else {
        lowerData[P_filterSustain] = value;
        if (wholemode) {
          upperData[P_filterSustain] = value;
        }
      }
      filterSustainstr = LINEAR_FILTERMIXERSTR[value];
      updatefilterSustain(1);
      break;

    case CCfilterRelease:
      if (upperSW) {
        upperData[P_filterRelease] = value;
      } else {
        lowerData[P_filterRelease] = value;
        if (wholemode) {
          upperData[P_filterRelease] = value;
        }
      }
      filterReleasestr = ENVTIMES[value];
      updatefilterRelease(1);
      break;

    case CCampAttack:
      if (upperSW) {
        upperData[P_ampAttack] = value;
      } else {
        lowerData[P_ampAttack] = value;
        if (wholemode) {
          upperData[P_ampAttack] = value;
        }
      }
      ampAttackstr = ENVTIMES[value];
      updateampAttack(1);
      break;

    case CCampDecay:
      if (upperSW) {
        upperData[P_ampDecay] = value;
        upperData[P_oldampDecay] = value;
      } else {
        lowerData[P_ampDecay] = value;
        lowerData[P_oldampDecay] = value;
        if (wholemode) {
          upperData[P_ampDecay] = value;
          upperData[P_oldampDecay] = value;
        }
      }
      ampDecaystr = ENVTIMES[value];
      updateampDecay(1);
      break;

    case CCampSustain:
      if (upperSW) {
        upperData[P_ampSustain] = value;
        upperData[P_oldampSustain] = value;
      } else {
        lowerData[P_ampSustain] = value;
        lowerData[P_oldampSustain] = value;
        if (wholemode) {
          upperData[P_ampSustain] = value;
          upperData[P_oldampSustain] = value;
        }
      }
      ampSustainstr = LINEAR_FILTERMIXERSTR[value];
      updateampSustain(1);
      break;

    case CCampRelease:
      if (upperSW) {
        upperData[P_ampRelease] = value;
        upperData[P_oldampRelease] = value;
      } else {
        lowerData[P_ampRelease] = value;
        lowerData[P_oldampRelease] = value;
        if (wholemode) {
          upperData[P_ampRelease] = value;
          upperData[P_oldampRelease] = value;
        }
      }
      ampReleasestr = ENVTIMES[value];
      updateampRelease(1);
      break;

    case CCvolumeControl:
      if (upperSW) {
        upperData[P_volumeControl] = value;
      } else {
        lowerData[P_volumeControl] = value;
        if (wholemode) {
          upperData[P_volumeControl] = value;
        }
      }
      volumeControlstr = value;
      updatevolumeControl(1);
      break;

    case CCfilterLevel1:
      if (upperSW) {
        upperData[P_filterLevel1] = value;
      } else {
        lowerData[P_filterLevel1] = value;
        if (wholemode) {
          upperData[P_filterLevel1] = value;
        }
      }
      filterLevel1str = value;
      updatefilterLevel1(1);
      break;

    case CCfilterLevel2:
      if (upperSW) {
        upperData[P_filterLevel2] = value;
      } else {
        lowerData[P_filterLevel2] = value;
        if (wholemode) {
          upperData[P_filterLevel2] = value;
        }
      }
      filterLevel2str = value;
      updatefilterLevel2(1);
      break;

    case CCosc1sawDetune:
      if (upperSW) {
        upperData[P_osc1sawDetune] = value;
      } else {
        lowerData[P_osc1sawDetune] = value;
        if (wholemode) {
          upperData[P_osc1sawDetune] = value;
        }
      }
      osc1sawDetunestr = value;
      updateosc1sawDetune(1);
      break;

    case CCkeyTrack:
      if (upperSW) {
        upperData[P_keytrack] = value;
      } else {
        lowerData[P_keytrack] = value;
        if (wholemode) {
          upperData[P_keytrack] = value;
        }
      }
      keytrackstr = value;
      updatekeytrack(1);
      break;


    case CCamDepth:
      if (upperSW) {
        upperData[P_amDepth] = value;
      } else {
        lowerData[P_amDepth] = value;
        if (wholemode) {
          upperData[P_amDepth] = value;
        }
      }
      amDepthstr = value;
      updateamDepth(1);
      break;

      //   ////////////////////////////////////////////////

    case CCplayMode:
      updateplayMode(1);
      break;

    case CCNotePriority:
      if (upperData[P_keyboardMode] >= 2) {
        if (upperSW) {
          upperData[P_NotePriority] = value;
        }
        updateNotePriority(1);
      }
      if (lowerData[P_keyboardMode] >= 2) {
        if (lowerSW) {
          lowerData[P_NotePriority] = value;
        }
        updateNotePriority(1);
      }
      break;

    case CCkeyboardMode:
      if (upperSW) {
        upperData[P_keyboardMode] = panelData[P_keyboardMode];
      } else {
        lowerData[P_keyboardMode] = panelData[P_keyboardMode];
      }
      updatekeyboardMode(1);
      break;

    case CCglideSW:
      if (upperSW) {
        upperData[P_glideSW] = !upperData[P_glideSW];
      } else {
        lowerData[P_glideSW] = !lowerData[P_glideSW];
      }
      updateglideSW(1);
      break;

    case CCfilterPoleSW:
      if (upperSW) {
        upperData[P_filterPoleSW] = value;
      } else {
        lowerData[P_filterPoleSW] = value;
      }
      updatefilterPoleSwitch(1);
      break;

    case CCdco_at_SW:
      if (upperSW) {
        upperData[P_dco_at_SW] = value;
      } else {
        lowerData[P_dco_at_SW] = value;
        if (wholemode) {
          upperData[P_dco_at_SW] = value;
        }
      }
      updatedco_at_SW(1);
      break;

    case CCfilter_at_SW:
      if (upperSW) {
        upperData[P_filter_at_SW] = value;
      } else {
        lowerData[P_filter_at_SW] = value;
        if (wholemode) {
          upperData[P_filter_at_SW] = value;
        }
      }
      updatefilter_at_SW(1);
      break;

    case CCfilterVel:
      if (upperSW) {
        upperData[P_filterVel] = !upperData[P_filterVel];
      } else {
        lowerData[P_filterVel] = !lowerData[P_filterVel];
      }
      updatefilterVel(1);
      break;

    case CCfilterEGinv:
      if (upperSW) {
        upperData[P_filterEGinv] = !upperData[P_filterEGinv];
      } else {
        lowerData[P_filterEGinv] = !lowerData[P_filterEGinv];
      }
      updatefilterEGinv(1);
      break;

    case CCsyncSW:
      if (upperSW) {
        upperData[P_sync] = value;
      } else {
        lowerData[P_sync] = value;
      }
      updatesyncSW(1);
      break;

    case CCeffectparam3:
      if (value > 63) {
        if (upperSW) {
          upperfootPedal = true;
        } else {
          lowerfootPedal = true;
        }
        updatefootSwitch();
      }
      break;

    case CCkeyTrackSW:
      if (upperSW) {
        upperData[P_keytrackSW] = !upperData[P_keytrackSW];
      } else {
        lowerData[P_keytrackSW] = !lowerData[P_keytrackSW];
      }
      updatekeyTrackSW(1);
      break;

    case CCfilterenvLinLogSW:
      if (upperSW) {
        upperData[P_filterLogLin] = !upperData[P_filterLogLin];
      } else {
        lowerData[P_filterLogLin] = !lowerData[P_filterLogLin];
      }
      updatefilterenvLogLin(1);
      break;

    case CCampenvLinLogSW:
      if (upperSW) {
        upperData[P_ampLogLin] = !upperData[P_ampLogLin];
      } else {
        lowerData[P_ampLogLin] = !lowerData[P_ampLogLin];
      }
      updateampenvLogLin(1);
      break;

    case CCnoiseSrc:
      if (upperSW) {
        upperData[P_noiseSrc] = !upperData[P_noiseSrc];
      } else {
        lowerData[P_noiseSrc] = !lowerData[P_noiseSrc];
      }
      updatenoiseSrc(1);
      break;

    case CCFilterLoop:
      if (upperSW) {
        upperData[P_filterLoop] = value;
      } else {
        lowerData[P_filterLoop] = value;
      }
      updatefilterLoop(1);
      break;

    case CCAmpLoop:
      if (upperSW) {
        upperData[P_vcaLoop] = value;
      } else {
        lowerData[P_vcaLoop] = value;
      }
      updatevcaLoop(1);
      break;

    case CCchordHoldSW:
      if (upperSW) {
        chordHoldU = !chordHoldU;
      } else {
        chordHoldL = !chordHoldL;
      }
      updatechordHoldSW(1);
      break;

    case CCvcaVel:
      if (upperSW) {
        upperData[P_vcaVel] = !upperData[P_vcaVel];
      } else {
        lowerData[P_vcaVel] = !lowerData[P_vcaVel];
      }
      updatevcaVel(1);
      break;

    case CCeffectBankSW:
      if (upperSW) {
        upperData[P_effectBank] = value;
      } else {
        lowerData[P_effectBank] = value;
      }
      updateeffectBankSW(1);
      break;

    case CClfoMult:
      if (upperSW) {
        upperData[P_lfoMultiplier] = value;
      } else {
        lowerData[P_lfoMultiplier] = value;
      }
      updatelfoMultiplier(1);
      break;

    case CCeffectNumSW:
      if (upperSW) {
        upperData[P_effectNum] = value;
      } else {
        lowerData[P_effectNum] = value;
      }
      updateeffectNumSW(1);
      break;

    case CCvcaGate:
      if (upperSW) {
        upperData[P_vcaGate] = !upperData[P_vcaGate];
      } else {
        lowerData[P_vcaGate] = !lowerData[P_vcaGate];
      }
      updatevcaGate(1);
      break;

    case CCmonoMulti:
      if (upperSW) {
        upperData[P_monoMulti] = !upperData[P_monoMulti];
      } else {
        lowerData[P_monoMulti] = !lowerData[P_monoMulti];
      }
      updateMonoMulti(1);
      break;

    case CClfo1retrig:
      if (upperSW) {
        upperData[P_lfo1retrig] = !upperData[P_lfo1retrig];
      } else {
        lowerData[P_lfo1retrig] = !lowerData[P_lfo1retrig];
      }
      updateLFO1retrig(1);
      break;

    case CCLFO1Waveform:
      if (upperSW) {
        switch (value) {
          case 0:
            upperData[P_LFO1Waveform] = 0;
            break;

          case 1:
            upperData[P_LFO1Waveform] = 63;
            break;

          case 2:
            upperData[P_LFO1Waveform] = 127;
            break;
        }
      } else {
        switch (value) {
          case 0:
            lowerData[P_LFO1Waveform] = 0;
            break;

          case 1:
            lowerData[P_LFO1Waveform] = 63;
            break;

          case 2:
            lowerData[P_LFO1Waveform] = 127;
            break;
        }
        if (wholemode) {
          upperData[P_LFO1Waveform] = lowerData[P_LFO1Waveform];
        }
      }
      updateLFO1Waveform(1);
      break;

    case CCLFO2Waveform:
      if (upperSW) {
        switch (value) {
          case 0:
            upperData[P_LFO2Waveform] = 0;
            break;

          case 1:
            upperData[P_LFO2Waveform] = 63;
            break;

          case 2:
            upperData[P_LFO2Waveform] = 127;
            break;
        }
      } else {
        switch (value) {
          case 0:
            lowerData[P_LFO2Waveform] = 0;
            break;

          case 1:
            lowerData[P_LFO2Waveform] = 63;
            break;

          case 2:
            lowerData[P_LFO2Waveform] = 127;
            break;
        }
        if (wholemode) {
          upperData[P_LFO2Waveform] = lowerData[P_LFO2Waveform];
        }
      }
      updateLFO2Waveform(1);
      break;

    case CCLFO3Waveform:
      if (upperSW) {
        upperData[P_LFO3Waveform] = value;
      } else {
        lowerData[P_LFO3Waveform] = value;
        if (wholemode) {
          upperData[P_LFO3Waveform] = value;
        }
      }
      updateLFO3Waveform(1);
      break;

    case CCupperSW:
      upperSW = true;
      lowerSW = false;
      updateupperSW(1);
      break;

    case CClowerSW:
      lowerSW = true;
      upperSW = false;
      updatelowerSW(1);
      break;

    case CCmodwheel:
      if (upperSW) {
        midiCCOut89(WSmodwheel, value / 8);  // divided by 8 because the convert bumps it up to 1023
      } else {
        midiCCOut79(WSmodwheel, value / 8);
        if (wholemode) {
          midiCCOut89(WSmodwheel, value / 8);
        }
      }
      break;

    case CCallnotesoff:
      allNotesOff();
      break;
  }
}

void myProgramChange(byte channel, byte program) {
  if (inPerformanceMode) {
    if (program < performances.size()) {
      performanceIndex = program;
      currentPerformance = performances[performanceIndex];

      // Update playmode and patch indices
      playMode = currentPerformance.mode;
      wholemode = (playMode == WHOLE);
      updateplayMode(0);

      // Set patch indices
      for (int i = 0; i < patches.size(); i++) {
        if (patches[i].patchNo == currentPerformance.upperPatchNo) upperPatchIndex = i;
        if (patches[i].patchNo == currentPerformance.lowerPatchNo) lowerPatchIndex = i;
      }

      // Recall both patches
      upperSW = true;
      recallPatch(currentPerformance.upperPatchNo);
      upperSW = false;
      recallPatch(currentPerformance.lowerPatchNo);

      refreshPatchDisplayFromState();
    }
  } else {
    // Normal patch recall
    state = PATCH;
    patchNo = program + 1;
    recallPatch(patchNo);
    state = PARAMETER;
  }
}

void myAfterTouch(byte channel, byte value) {

  afterTouch = (value * 1023) / 127;  // Exact scaling, range 1023
  afterTouchU = (afterTouch * upperData[P_ATDepth]) / 1023;
  afterTouchL = (afterTouch * lowerData[P_ATDepth]) / 1023;

  switch (upperData[P_AfterTouchDest]) {
    case 1:
        MIDI8.sendAfterTouch(value, 9);

      break;
    case 2:
      upperData[P_filterCutoff] = (oldfilterCutoffU + afterTouchU);
      if (afterTouchU < 10) {
        upperData[P_filterCutoff] = oldfilterCutoffU;
      }
      if (upperData[P_filterCutoff] > 1023) {
        upperData[P_filterCutoff] = 1023;
      }
      break;
    case 3:
      upperData[P_filterLFO] = afterTouchU;
      break;
    case 4:
      upperData[P_amDepth] = afterTouchU;
      break;
  }
  switch (lowerData[P_AfterTouchDest]) {
    case 1:
      MIDI7.sendAfterTouch(value, 9);

      if (wholemode) {
        MIDI8.sendAfterTouch(value, 1);

      }
      break;
    case 2:
      lowerData[P_filterCutoff] = (oldfilterCutoffL + afterTouchL);
      if (afterTouchL < 10) {
        lowerData[P_filterCutoff] = oldfilterCutoffL;
      }
      if (lowerData[P_filterCutoff] > 1023) {
        lowerData[P_filterCutoff] = 1023;
      }
      break;
    case 3:
      lowerData[P_filterLFO] = afterTouchL;
      break;
    case 4:
      lowerData[P_amDepth] = afterTouchL;
      break;
  }
}

void recallPatch(int patchNo) {
  allNotesOff();

  File patchFile = SD.open(String(patchNo).c_str());
  if (!patchFile) {
    Serial.println("File not found");
  } else {
    String data[NO_OF_PARAMS];
    recallPatchData(patchFile, data);
    patchFile.close();

    // Find matching patch in the circular buffer to set name and number
    for (int i = 0; i < patches.size(); i++) {

      if (patches[i].patchNo == patchNo) {
        if (upperSW) {
          upperPatchIndex = i;
          currentPgmNumU = String(patches[i].patchNo);
          currentPatchNameU = patches[i].patchName;
          //storeLastPatchU(currentPgmNumU)
        } else {
          lowerPatchIndex = i;
          currentPgmNumL = String(patches[i].patchNo);
          currentPatchNameL = patches[i].patchName;
          //storeLastPatchL(currentPgmNumL)
        }

        break;
      }
    }

    setCurrentPatchData(data);
  }
}

void setCurrentPatchData(String data[]) {
  int tempData[95];  // Temporary array for converted integers

  // Convert data from String to int once
  for (int i = 1; i <= 94; i++) {
    tempData[i] = data[i].toInt();
  }

  if (upperSW) {
    patchNameU = data[0];
    tempData[0] = 1;
    memcpy(upperData, tempData, sizeof(tempData));

    oldfilterCutoffU = upperData[P_filterCutoff];
    upperParamsToDisplay();
    setAllButtons();
  } else {
    patchNameL = data[0];
    tempData[0] = 1;
    memcpy(lowerData, tempData, sizeof(tempData));

    oldfilterCutoffL = lowerData[P_filterCutoff];
    lowerParamsToDisplay();
    setAllButtons();

    if (wholemode) {

      // Update previous values and pick-up flags
      for (int i = 1; i <= 94; i++) {
        upperData[i] = lowerData[i];  // Store previous value
      }

      oldfilterCutoffU = upperData[P_filterCutoff];
      upperParamsToDisplay();
      setAllButtons();
    }
  }

  updatePatchname();
}

void upperParamsToDisplay() {

  updateglideTime(0);
  updateosc1PW(0);
  updateosc1PWM(0);
  updateOsc1SawLevel(0);
  updateOsc1PulseLevel(0);
  updateOsc1SubLevel(0);
  updatefmDepth(0);
  updateosc2PW(0);
  updateosc2PWM(0);
  updateOsc2SawLevel(0);
  updateOsc2PulseLevel(0);
  updateOsc2TriangleLevel(0);
  updateosc2Detune(0);
  updateosc2Interval(0);
  updateOsc2EnvDepth(0);
  updateFilterCutoff(0);
  updatefilterRes(0);
  updatefilterEGlevel(0);
  updatekeytrack(0);
  updatefilterLFO(0);
  updatepitchAttack(0);
  updatepitchDecay(0);
  updatepitchSustain(0);
  updatepitchRelease(0);
  updatefilterAttack(0);
  updatefilterDecay(0);
  updatefilterSustain(0);
  updatefilterRelease(0);
  updateampAttack(0);
  updateampDecay(0);
  updateampSustain(0);
  updateampRelease(0);
  updateLFO1Rate(0);
  updateLFO1Delay(0);
  updateLFO1Slope(0);
  updateLFO1retrig(0);
  updateLFO2Rate(0);
  updateLFO3Rate(0);
  updateLFO3Delay(0);
  updateeffectPot1(0);
  updateeffectPot2(0);
  updateeffectPot3(0);
  updateeffectsMix(0);
  updatenoiseLevel(0);
  updatemodWheelDepth(0);
  updatePitchBendDepth(0);
  updatevolumeControl(0);
  updatefilterLevel1(0);
  updatefilterLevel2(0);
  updateosc1sawDetune(0);
  updateosc1sawCount(0);
  updateATDepth(0);
  updateamDepth(0);
  updateFilterType(0);
  updateLFO3Waveform(0); 
  updateeffectBankSW(0);
  updateeffectNumSW(0);
  updatearpRate(0);
  updateosc1envPWM(0);
  updateosc2envPWM(0);
}

void lowerParamsToDisplay() {

  updateglideTime(0);
  updateosc1PW(0);
  updateosc1PWM(0);
  updateOsc1SawLevel(0);
  updateOsc1PulseLevel(0);
  updateOsc1SubLevel(0);
  updatefmDepth(0);
  updateosc2PW(0);
  updateosc2PWM(0);
  updateOsc2SawLevel(0);
  updateOsc2PulseLevel(0);
  updateOsc2TriangleLevel(0);
  updateosc2Detune(0);
  updateosc2Interval(0);
  updateOsc2EnvDepth(0);
  updateFilterCutoff(0);
  updatefilterRes(0);
  updatefilterEGlevel(0);
  updatekeytrack(0);
  updatefilterLFO(0);
  updatepitchAttack(0);
  updatepitchDecay(0);
  updatepitchSustain(0);
  updatepitchRelease(0);
  updatefilterAttack(0);
  updatefilterDecay(0);
  updatefilterSustain(0);
  updatefilterRelease(0);
  updateampAttack(0);
  updateampDecay(0);
  updateampSustain(0);
  updateampRelease(0);
  updateLFO1Rate(0);
  updateLFO1Delay(0);
  updateLFO1Slope(0);
  updateLFO1retrig(0);
  updateLFO2Rate(0);
  updateLFO3Rate(0);
  updateLFO3Delay(0);
  updateeffectPot1(0);
  updateeffectPot2(0);
  updateeffectPot3(0);
  updateeffectsMix(0);
  updatenoiseLevel(0);
  updatemodWheelDepth(0);
  updatePitchBendDepth(0);
  updatevolumeControl(0);
  updatefilterLevel1(0);
  updatefilterLevel2(0);
  updateosc1sawDetune(0);
  updateosc1sawCount(0);
  updateamDepth(0);
  updateATDepth(0);
  updateFilterType(0);
  updateLFO3Waveform(0);
  updateeffectBankSW(0);
  updateeffectNumSW(0);
  updatearpRate(0);
  updateosc1envPWM(0);
  updateosc2envPWM(0);
}

void setAllButtons() {
  updateosc1Range(0);
  updateosc2Range(0);
  updateLFO1Waveform(0);
  updateLFO2Waveform(0);
  updatekeyboardMode(0);
  updateNotePriority(0);
  updateglideSW(0);
  updatesyncSW(0);
  updatefilterPoleSwitch(0);
  updatefilterEGinv(0);
  updatevcaGate(0);
  updatekeyTrackSW(0);
  updatedco_at_SW(0);
  updatefilter_at_SW(0);
  updateMonoMulti(0);
  updatelfoMultiplier(0);
  updatevcaVel(0);
  updatefilterLoop(0);
  updatevcaLoop(0);
  updatefilterenvLogLin(0);
  updateampenvLogLin(0);
  updatefilterVel(0);
  updatenoiseSrc(0);
}

String getCurrentPatchData() {
  if (upperSW) {
    return patchNameU + "," + String(upperData[P_LFO2Rate]) + "," + String(upperData[P_fmDepth]) + "," + String(upperData[P_osc2PW]) + "," + String(upperData[P_osc2PWM])
           + "," + String(upperData[P_osc1PW]) + "," + String(upperData[P_osc1PWM]) + "," + String(upperData[P_osc1Range]) + "," + String(upperData[P_osc2Range]) + "," + String(upperData[P_osc2Interval])
           + "," + String(upperData[P_glideTime]) + "," + String(upperData[P_osc2Detune]) + "," + String(upperData[P_noiseLevel]) + "," + String(upperData[P_osc2SawLevel])
           + "," + String(upperData[P_osc1SawLevel]) + "," + String(upperData[P_osc2PulseLevel]) + "," + String(upperData[P_osc1PulseLevel]) + "," + String(upperData[P_filterCutoff])
           + "," + String(upperData[P_filterLFO]) + "," + String(upperData[P_filterRes]) + "," + String(upperData[P_filterType]) + "," + String(upperData[P_modWheelDepth])
           + "," + String(upperData[P_effectsMix]) + "," + String(upperData[P_LFODelayGo]) + "," + String(upperData[P_filterEGlevel]) + "," + String(upperData[P_LFO1Rate])
           + "," + String(upperData[P_LFO1Waveform]) + "," + String(upperData[P_filterAttack]) + "," + String(upperData[P_filterDecay]) + "," + String(upperData[P_filterSustain])
           + "," + String(upperData[P_filterRelease]) + "," + String(upperData[P_ampAttack]) + "," + String(upperData[P_ampDecay]) + "," + String(upperData[P_ampSustain])
           + "," + String(upperData[P_ampRelease]) + "," + String(upperData[P_volumeControl]) + "," + String(upperData[P_glideSW]) + "," + String(upperData[P_keytrack])
           + "," + String(upperData[P_filterPoleSW]) + "," + String(upperData[P_filterLoop]) + "," + String(upperData[P_filterEGinv]) + "," + String(upperData[P_filterVel])
           + "," + String(upperData[P_vcaLoop]) + "," + String(upperData[P_vcaVel]) + "," + String(upperData[P_vcaGate]) + "," + String(upperData[P_lfoAlt]) + "," + String(upperData[P_filterLevel1])
           + "," + String(upperData[P_filterLevel2]) + "," + String(upperData[P_monoMulti]) + "," + String(upperData[P_modWheelLevel]) + "," + String(upperData[P_PitchBendLevel])
           + "," + String(upperData[P_amDepth]) + "," + String(upperData[P_sync]) + "," + String(upperData[P_effectPot1]) + "," + String(upperData[P_effectPot2]) + "," + String(upperData[P_effectPot3])
           + "," + String(upperData[P_oldampAttack]) + "," + String(upperData[P_oldampDecay]) + "," + String(upperData[P_oldampSustain]) + "," + String(upperData[P_oldampRelease])
           + "," + String(upperData[P_AfterTouchDest]) + "," + String(upperData[P_filterLogLin]) + "," + String(upperData[P_ampLogLin]) + "," + String(upperData[P_osc2TriangleLevel])
           + "," + String(upperData[P_osc1SubLevel]) + "," + String(upperData[P_keyboardMode]) + "," + String(upperData[P_LFO1Delay]) + "," + String(upperData[P_effectNum]) + "," + String(upperData[P_effectBank])
           + "," + String(upperData[P_LFO1Slope]) + "," + String(upperData[P_LFO3Rate]) + "," + String(upperData[P_lfoMultiplier]) + "," + String(upperData[P_NotePriority]) + "," + String(upperData[P_keytrackSW])
           + "," + String(upperData[P_ATDepth]) + "," + String(upperData[P_pitchAttack]) + "," + String(upperData[P_pitchDecay]) + "," + String(upperData[P_pitchSustain]) + "," + String(upperData[P_pitchRelease])
           + "," + String(upperData[P_LFO3Delay]) + "," + String(upperData[P_osc1sawDetune]) + "," + String(upperData[P_osc1sawCount]) + "," + String(upperData[P_arpRate])
           + "," + String(upperData[P_LFO3Waveform]) + "," + String(upperData[P_LFO2Waveform]) + "," + String(upperData[P_osc2envDepth]) + "," + String(upperData[P_noiseSrc]) + "," + String(upperData[P_lfo1retrig])
           + "," + String(upperData[P_osc1envPWM]) + "," + String(upperData[P_osc2envPWM]) + "," + String(upperData[P_dco_at_SW]) + "," + String(upperData[P_filter_at_SW]);
  } else {
    return patchNameL + "," + String(lowerData[P_LFO2Rate]) + "," + String(lowerData[P_fmDepth]) + "," + String(lowerData[P_osc2PW]) + "," + String(lowerData[P_osc2PWM])
           + "," + String(lowerData[P_osc1PW]) + "," + String(lowerData[P_osc1PWM]) + "," + String(lowerData[P_osc1Range]) + "," + String(lowerData[P_osc2Range]) + "," + String(lowerData[P_osc2Interval])
           + "," + String(lowerData[P_glideTime]) + "," + String(lowerData[P_osc2Detune]) + "," + String(lowerData[P_noiseLevel]) + "," + String(lowerData[P_osc2SawLevel])
           + "," + String(lowerData[P_osc1SawLevel]) + "," + String(lowerData[P_osc2PulseLevel]) + "," + String(lowerData[P_osc1PulseLevel]) + "," + String(lowerData[P_filterCutoff])
           + "," + String(lowerData[P_filterLFO]) + "," + String(lowerData[P_filterRes]) + "," + String(lowerData[P_filterType]) + "," + String(lowerData[P_modWheelDepth])
           + "," + String(lowerData[P_effectsMix]) + "," + String(lowerData[P_LFODelayGo]) + "," + String(lowerData[P_filterEGlevel]) + "," + String(lowerData[P_LFO1Rate])
           + "," + String(lowerData[P_LFO1Waveform]) + "," + String(lowerData[P_filterAttack]) + "," + String(lowerData[P_filterDecay]) + "," + String(lowerData[P_filterSustain])
           + "," + String(lowerData[P_filterRelease]) + "," + String(lowerData[P_ampAttack]) + "," + String(lowerData[P_ampDecay]) + "," + String(lowerData[P_ampSustain])
           + "," + String(lowerData[P_ampRelease]) + "," + String(lowerData[P_volumeControl]) + "," + String(lowerData[P_glideSW]) + "," + String(lowerData[P_keytrack])
           + "," + String(lowerData[P_filterPoleSW]) + "," + String(lowerData[P_filterLoop]) + "," + String(lowerData[P_filterEGinv]) + "," + String(lowerData[P_filterVel])
           + "," + String(lowerData[P_vcaLoop]) + "," + String(lowerData[P_vcaVel]) + "," + String(lowerData[P_vcaGate]) + "," + String(lowerData[P_lfoAlt]) + "," + String(lowerData[P_filterLevel1])
           + "," + String(lowerData[P_filterLevel2]) + "," + String(lowerData[P_monoMulti]) + "," + String(lowerData[P_modWheelLevel]) + "," + String(lowerData[P_PitchBendLevel])
           + "," + String(lowerData[P_amDepth]) + "," + String(lowerData[P_sync]) + "," + String(lowerData[P_effectPot1]) + "," + String(lowerData[P_effectPot2]) + "," + String(lowerData[P_effectPot3])
           + "," + String(lowerData[P_oldampAttack]) + "," + String(lowerData[P_oldampDecay]) + "," + String(lowerData[P_oldampSustain]) + "," + String(lowerData[P_oldampRelease])
           + "," + String(lowerData[P_AfterTouchDest]) + "," + String(lowerData[P_filterLogLin]) + "," + String(lowerData[P_ampLogLin]) + "," + String(lowerData[P_osc2TriangleLevel])
           + "," + String(lowerData[P_osc1SubLevel]) + "," + String(lowerData[P_keyboardMode]) + "," + String(lowerData[P_LFO1Delay]) + "," + String(lowerData[P_effectNum]) + "," + String(lowerData[P_effectBank])
           + "," + String(lowerData[P_LFO1Slope]) + "," + String(lowerData[P_LFO3Rate]) + "," + String(lowerData[P_lfoMultiplier]) + "," + String(lowerData[P_NotePriority]) + "," + String(lowerData[P_keytrackSW])
           + "," + String(lowerData[P_ATDepth]) + "," + String(lowerData[P_pitchAttack]) + "," + String(lowerData[P_pitchDecay]) + "," + String(lowerData[P_pitchSustain]) + "," + String(lowerData[P_pitchRelease])
           + "," + String(lowerData[P_LFO3Delay]) + "," + String(lowerData[P_osc1sawDetune]) + "," + String(lowerData[P_osc1sawCount]) + "," + String(lowerData[P_arpRate])
           + "," + String(lowerData[P_LFO3Waveform]) + "," + String(lowerData[P_LFO2Waveform]) + "," + String(lowerData[P_osc2envDepth]) + "," + String(lowerData[P_noiseSrc]) + "," + String(lowerData[P_lfo1retrig])
           + "," + String(lowerData[P_osc1envPWM]) + "," + String(lowerData[P_osc2envPWM]) + "," + String(lowerData[P_dco_at_SW]) + "," + String(lowerData[P_filter_at_SW]);
  }
}

void midiCCOut(byte cc, byte value) {
  MIDI.sendControlChange(cc, value, midiChannel);  //MIDI DIN main out
}

void midiCCOut79(byte cc, byte value) {
  MIDI7.sendControlChange(cc, value, 9);  //MIDI to lower board DCOs
}

void midiCCOut710(byte cc, byte value) {
  MIDI7.sendControlChange(cc, value, 10);  //MIDI to lower board Filters etc
}

void midiCCOut711(byte cc, byte value) {
  MIDI7.sendControlChange(cc, value, 11);  //MIDI to lower board Filters etc
}

void midiCCOut89(byte cc, byte value) {
  MIDI8.sendControlChange(cc, value, 9);  //MIDI to upper board DCOs
}

void midiCCOut810(byte cc, byte value) {
  MIDI8.sendControlChange(cc, value, 10);  //MIDI to upper board Filters etc
}

void midiCCOut811(byte cc, byte value) {
  MIDI8.sendControlChange(cc, value, 11);  //MIDI to upper board Filters etc
}

void midiCCOut61(byte cc, byte value) {
  MIDI6.sendControlChange(cc, value, 1);  //MIDI DIN to panel for switches
}

void midiCCOut62(byte cc, byte value) {
  MIDI6.sendControlChange(cc, value, 2);  //MIDI DIN to panel for switches
}

void showSettingsPage() {
  showSettingsPage(settings::current_setting(), settings::current_setting_value(), state);
}

void showPerformancePage(String perfNum, String name, int upperNo, String upperName, int lowerNo, String lowerName) {
  currentPerfNum = perfNum;
  currentPerfName = name;
  currentUpperPatchNo = upperNo;
  currentUpperPatchName = upperName;
  currentLowerPatchNo = lowerNo;
  currentLowerPatchName = lowerName;
}

void reinitialiseToPanel() {
  if (upperSW) {
    for (int i = 1; i < 77; i++) {
      upperData[i] = 0;
    }
    upperData[P_osc1SawLevel] = 127;
    upperData[P_osc2SawLevel] = 127;
    upperData[P_osc2Detune] = 8;
    upperData[P_filterCutoff] = 127;
    upperData[P_ampSustain] = 127;
    upperData[P_volumeControl] = 127;
    upperData[P_noiseLevel] = 63;
    upperData[P_osc1PW] = 63;
    upperData[P_osc2PW] = 63;
    upperParamsToDisplay();
    setAllButtons();
  } else {
    for (int i = 1; i < 77; i++) {
      lowerData[i] = 0;
    }
    lowerData[P_osc1SawLevel] = 127;
    lowerData[P_osc2SawLevel] = 127;
    lowerData[P_osc2Detune] = 8;
    lowerData[P_filterCutoff] = 127;
    lowerData[P_ampSustain] = 127;
    lowerData[P_volumeControl] = 127;
    lowerData[P_noiseLevel] = 63;
    lowerData[P_osc1PW] = 63;
    lowerData[P_osc2PW] = 63;
    lowerParamsToDisplay();
    setAllButtons();
    if (wholemode) {
      for (int i = 1; i < 77; i++) {
        upperData[i] = 0;
      }
      upperData[P_osc1SawLevel] = 127;
      upperData[P_osc2SawLevel] = 127;
      upperData[P_osc2Detune] = 8;
      upperData[P_filterCutoff] = 127;
      upperData[P_ampSustain] = 127;
      upperData[P_volumeControl] = 127;
      upperData[P_noiseLevel] = 63;
      upperData[P_osc1PW] = 63;
      upperData[P_osc2PW] = 63;
      upperParamsToDisplay();
      setAllButtons();
    }
  }
  patchName = INITPATCHNAME;
  showPatchPage("Initial", "Patch Settings", "", "");
}

void deletePerformance(int perfNo) {
  char filename[32];
  snprintf(filename, sizeof(filename), "/performances/perf%03d", perfNo);
  if (SD.exists(filename)) {
    SD.remove(filename);
    Serial.print("[DELETE] Removed performance: ");
    Serial.println(filename);
  }
}

void renumberPerformancesOnSD() {
  char filename[32];
  for (int i = 0; i < performances.size(); i++) {
    Performance p = performances[i];
    p.performanceNo = i + 1;
    performances[i] = p;

    snprintf(filename, sizeof(filename), "/performances/perf%03d", p.performanceNo);
    savePerformance(filename, p);
  }
}

void checkSwitches() {


  saveButton.update();
  if (saveButton.held()) {
    if (inPerformanceMode && (state == PARAMETER || state == PATCH)) {
      state = PERFORMANCE_DELETE;
    } else if (state == PARAMETER || state == PATCH) {
      state = DELETE;
    }
    refreshScreen();
  } else if (saveButton.numClicks() == 1) {
    switch (state) {
      case SAVE:
        {
          if (renamedPatch.length() == 0) {
            renamedPatch = INITPATCHNAME;  // fallback if no rename occurred
          }

          // Update patch name depending on upper or lower
          if (upperSW) {
            patchNameU = renamedPatch;
            currentPatchNameU = renamedPatch;
            currentPgmNumU = String(patches.last().patchNo);
          } else {
            patchNameL = renamedPatch;
            currentPatchNameL = renamedPatch;
            currentPgmNumL = String(patches.last().patchNo);
          }

          // ✅ Update last patch in the buffer before saving
          patches.last().patchName = renamedPatch;

          // ✅ Save updated patch data
          String patchData = getCurrentPatchData();
          savePatch(String(patches.last().patchNo).c_str(), patchData);

          // ✅ Reload and reorder patches explicitly
          loadPatches();
          setPatchesOrdering(patches.last().patchNo);

          // ✅ Correctly update patch index for immediate display
          for (int i = 0; i < patches.size(); i++) {
            if (patches[i].patchNo == patches.last().patchNo) {
              if (upperSW) upperPatchIndex = i;
              else lowerPatchIndex = i;
              break;
            }
          }

          // ✅ Immediately refresh display with updated data
          refreshPatchDisplayFromState();

          renamedPatch = "";
          state = PARAMETER;
        }
        refreshScreen();
        break;


      case PATCHNAMING:
        {
          //Serial.println("renamedPatch BEFORE SAVING: " + renamedPatch);

          if (renamedPatch.length() == 0) {
            renamedPatch = patches.last().patchName;  // fallback to existing name
          }

          // Update correct upper/lower patch name based on current layer
          if (upperSW) {
            patchNameU = renamedPatch;
            currentPatchNameU = renamedPatch;  // Update immediately
            currentPgmNumU = String(patches.last().patchNo);
          } else {
            patchNameL = renamedPatch;
            currentPatchNameL = renamedPatch;  // Update immediately
            currentPgmNumL = String(patches.last().patchNo);
          }

          // Update last patch in the patches buffer
          patches.last().patchName = renamedPatch;

          // Save patch data (with the correct name included)
          String patchData = getCurrentPatchData();
          savePatch(String(patches.last().patchNo).c_str(), patchData);

          loadPatches();                   // Refresh patches list from SD card
          refreshPatchDisplayFromState();  // immediately update the display
          setPatchesOrdering(patches.last().patchNo);

          renamedPatch = "";
          state = PARAMETER;
        }
        refreshScreen();
        break;


      case PARAMETER:
        if (inPerformanceMode) {
          if (performances.size() < PERFORMANCES_LIMIT) {
            int newPerfNo = performances.size() + 1;
            Performance newPerf = {
              newPerfNo,
              patches[upperPatchIndex].patchNo,
              patches[lowerPatchIndex].patchNo,
              INITPATCHNAME,
              (PlayMode)playMode
            };
            currentPerformance = newPerf;
            performances.push(newPerf);
            performanceIndex = performances.size() - 1;

            showPerformancePage(
              String(newPerf.performanceNo),
              newPerf.name,
              newPerf.upperPatchNo,
              getPatchName(newPerf.upperPatchNo),
              newPerf.lowerPatchNo,
              getPatchName(newPerf.lowerPatchNo));

            state = PERFORMANCE_SAVE;
          }
        } else {
          // 🛠 PATCH SAVE FLOW
          if (patches.size() < PATCHES_LIMIT) {
            resetPatchesOrdering();  // start from patch 1
            patches.push({ patches.size() + 1, INITPATCHNAME });
            state = SAVE;
          }
        }
        refreshScreen();
        break;

      case PERFORMANCE_SAVE:
        currentPerformance = performances[performanceIndex];
        state = PERFORMANCE_NAMING;
        renamedPatch = currentPerformance.name;
        charIndex = 0;
        currentCharacter = CHARACTERS[charIndex];
        startedRenaming = false;
        showRenamingPage(renamedPatch);
        refreshScreen();
        break;

      case PERFORMANCE_NAMING:
        if (saveButton.numClicks() == 1) {
          if (renamedPatch.length() > 0) {
            currentPerformance.name = renamedPatch;
          }

          upperSW = true;
          savePatch(String(currentPerformance.upperPatchNo).c_str(), getCurrentPatchData());

          upperSW = false;
          savePatch(String(currentPerformance.lowerPatchNo).c_str(), getCurrentPatchData());

          upperSW = true;

          // Update full performance data
          currentPerformance.upperPatchNo = patches[upperPatchIndex].patchNo;
          currentPerformance.lowerPatchNo = patches[lowerPatchIndex].patchNo;
          currentPerformance.mode = (PlayMode)playMode;

          for (int i = 0; i < performances.size(); i++) {
            if (performances[i].performanceNo == currentPerformance.performanceNo) {
              performances[i] = currentPerformance;
              break;
            }
          }

          char filename[16];
          snprintf(filename, sizeof(filename), "perf%03d", currentPerformance.performanceNo);

          savePerformance(filename, currentPerformance);
          loadPerformances();

          renamedPatch = "";
          charIndex = 0;
          currentCharacter = CHARACTERS[0];
          startedRenaming = false;
          state = PARAMETER;
        } else if (recallButton.numClicks() == 1) {
          if (renamedPatch.length() < 12) {
            renamedPatch.concat(String(currentCharacter));
            charIndex = 0;
            currentCharacter = CHARACTERS[charIndex];
            showRenamingPage(renamedPatch);
          }
        } else if (backButton.numClicks() == 1) {
          renamedPatch = "";
          charIndex = 0;
          startedRenaming = false;
          state = PARAMETER;
          if (performances.size() > 0 && performances.last().name == INITPATCHNAME) {
            performances.pop();
          }
        }
        refreshScreen();
        break;
    }
  }

  settingsButton.update();
  if (settingsButton.held()) {
    //If recall held, set current patch to match current hardware state
    //Reinitialise all hardware values to force them to be re-read if different
    state = REINITIALISE;
    reinitialiseToPanel();
  } else if (settingsButton.numClicks() == 1) {
    switch (state) {
      case PARAMETER:
        state = SETTINGS;
        showSettingsPage();
        refreshScreen();
        break;
      case SETTINGS:
        showSettingsPage();
      case SETTINGSVALUE:
        settings::save_current_value();
        state = SETTINGS;
        showSettingsPage();
        refreshScreen();
        break;
    }
  }

  backButton.update();
  if (backButton.held()) {
    //If Back button held, Panic - all notes off
  } else if (backButton.numClicks() == 1) {
    switch (state) {
      case RECALL:
        setPatchesOrdering(patchNo);
        state = PARAMETER;
        refreshScreen();
        break;
      case SAVE:
        renamedPatch = "";
        state = PARAMETER;
        loadPatches();  //Remove patch that was to be saved
        setPatchesOrdering(patchNo);
        refreshScreen();
        break;
      case PATCHNAMING:
        charIndex = 0;
        renamedPatch = "";
        state = SAVE;
        refreshScreen();
        break;
      case DELETE:
        setPatchesOrdering(patchNo);
        state = PARAMETER;
        refreshScreen();
        break;
      case SETTINGS:
        state = PARAMETER;
        refreshScreen();
        break;
      case SETTINGSVALUE:
        state = SETTINGS;
        showSettingsPage();
        refreshScreen();
        break;
      case PERFORMANCE_NAMING:
        renamedPatch = "";
        charIndex = 0;
        state = PARAMETER;
        // Optionally remove the unsaved performance from the buffer:
        if (performances.size() > 0 && performances.last().name == INITPATCHNAME) {
          performances.pop();
        }
        refreshScreen();
        break;
      case PERFORMANCE_DELETE:
        setPerformancesOrdering(currentPerformance.performanceNo);
        state = PARAMETER;
        refreshScreen();
        break;
    }
  }

  // Encoder switch
  recallButton.update();
  if (recallButton.held()) {
    if (!recallHeldToggleLatch) {
      inPerformanceMode = !inPerformanceMode;
      recallHeldToggleLatch = true;

      //Serial.print("[MODE] Switched to ");
      //Serial.println(inPerformanceMode ? "Performance Mode" : "Patch Mode");

      showCurrentParameterPage("Mode", inPerformanceMode ? "Performance" : "Patch");

      if (inPerformanceMode && performances.size() > 0) {
        // Entering Performance Mode
        performanceIndex = 0;
        currentPerformance = performances[performanceIndex];

        showPerformancePage(
          String(currentPerformance.performanceNo),
          currentPerformance.name,
          currentPerformance.upperPatchNo,
          getPatchName(currentPerformance.upperPatchNo),
          currentPerformance.lowerPatchNo,
          getPatchName(currentPerformance.lowerPatchNo));

      } else {
        // Returning to Patch Mode
        refreshPatchDisplayFromState();
      }
    }
  } else {
    recallHeldToggleLatch = false;
  }
  if (recallButton.numClicks() == 1) {
    switch (state) {
      case RECALL:
        //Serial.println("[INFO] Ignored default RECALL to avoid overwriting performance recall.");
        state = PARAMETER;
        refreshScreen();
        break;
      case SAVE:
        showRenamingPage(patches.last().patchName);
        patchName = patches.last().patchName;
        state = PATCHNAMING;
        refreshScreen();
        break;
      case PATCHNAMING:
        if (renamedPatch.length() < 12)  //actually 12 chars
        {
          renamedPatch.concat(String(currentCharacter));
          charIndex = 0;
          currentCharacter = CHARACTERS[charIndex];
          showRenamingPage(renamedPatch);
        }
        refreshScreen();
        break;
      case DELETE:
        //Don't delete final patch
        if (patches.size() > 1) {
          state = DELETEMSG;
          patchNo = patches.first().patchNo;     //PatchNo to delete from SD card
          patches.shift();                       //Remove patch from circular buffer
          deletePatch(String(patchNo).c_str());  //Delete from SD card
          loadPatches();                         //Repopulate circular buffer to start from lowest Patch No
          renumberPatchesOnSD();
          loadPatches();                      //Repopulate circular buffer again after delete
          patchNo = patches.first().patchNo;  //Go back to 1
          recallPatch(patchNo);               //Load first patch
        }
        state = PARAMETER;
        refreshScreen();
        break;
      case SETTINGS:
        state = SETTINGSVALUE;
        showSettingsPage();
        refreshScreen();
        break;
      case SETTINGSVALUE:
        settings::save_current_value();
        state = SETTINGS;
        showSettingsPage();
        refreshScreen();
        break;

      case PARAMETER:
        // Enter performance recall
        if (performances.size() > 0) {
          currentPerformance = performances.first();
          showPerformancePage(
            String(currentPerformance.performanceNo),
            currentPerformance.name,
            currentPerformance.upperPatchNo,
            getPatchName(currentPerformance.upperPatchNo),
            currentPerformance.lowerPatchNo,
            getPatchName(currentPerformance.lowerPatchNo));
          state = PERFORMANCE_RECALL;
        }
        refreshScreen();
        break;

      case PERFORMANCE_RECALL:
        for (int i = 0; i < patches.size(); i++) {
          if (patches[i].patchNo == currentPerformance.upperPatchNo) {
            upperPatchIndex = i;
          }
          if (patches[i].patchNo == currentPerformance.lowerPatchNo) {
            lowerPatchIndex = i;
          }
        }

        playMode = currentPerformance.mode;
        wholemode = (playMode == WHOLE);
        updateplayMode(0);

        upperSW = true;
        recallPatch(currentPerformance.upperPatchNo);

        upperSW = false;
        recallPatch(currentPerformance.lowerPatchNo);

        refreshPatchDisplayFromState();

        state = PARAMETER;
        patchNo = 0;  // ✅ Clear global patchNo to avoid accidental reuse
        refreshScreen();
        return;

      case PERFORMANCE_NAMING:
        if (renamedPatch.length() < 12) {
          renamedPatch.concat(String(currentCharacter));
          charIndex = 0;
          currentCharacter = CHARACTERS[charIndex];
          showRenamingPage(renamedPatch);
        }
        refreshScreen();
        break;

      case PERFORMANCE_DELETE:
        if (performances.size() > 0) {
          state = PERFORMANCE_DELETEMSG;

          int deletedNo = performances.first().performanceNo;
          performances.shift();          // Remove from buffer
          deletePerformance(deletedNo);  // Delete file
          loadPerformances();            // Refresh buffer
          renumberPerformancesOnSD();    // Reorder files
          loadPerformances();            // Reload to apply new order

          currentPerformance = performances.first();
          recallPerformance(currentPerformance);
        }
        state = PARAMETER;
        refreshScreen();
        return;


      case PERFORMANCE_DELETEMSG:
        // Show deletion complete screen briefly
        tft.fillScreen(ST7735_BLACK);
        tft.setFont(&FreeSans12pt7b);
        tft.setTextColor(ST7735_YELLOW);
        tft.setCursor(10, 60);
        tft.println("Renumbering");
        tft.setCursor(10, 100);
        tft.println("Performances...");
        tft.updateScreen();
        delay(1000);
        state = PARAMETER;
        refreshScreen();
        break;
    }
  }
}

// Updated checkEncoder() with upperPatchIndex and lowerPatchIndex
void checkEncoder() {
  long encRead = encoder.read();
  bool moved = false;

  if ((encCW && encRead > encPrevious + 3) || (!encCW && encRead < encPrevious - 3)) {
    moved = true;

    switch (state) {

      case PERFORMANCE_DELETE:
        if (encCW) {
          performances.push(performances.shift());
        } else {
          performances.unshift(performances.pop());
        }
        break;

      case PERFORMANCE_SAVE:
        performanceIndex++;
        if (performanceIndex >= performances.size()) performanceIndex = 0;
        currentPerformance = performances[performanceIndex];
        showPerformancePage(
          String(currentPerformance.performanceNo),
          currentPerformance.name,
          currentPerformance.upperPatchNo,
          getPatchName(currentPerformance.upperPatchNo),
          currentPerformance.lowerPatchNo,
          getPatchName(currentPerformance.lowerPatchNo));
        break;

      case PERFORMANCE_RECALL:
        performanceIndex++;
        if (performanceIndex >= performances.size()) performanceIndex = 0;
        currentPerformance = performances[performanceIndex];
        showPerformancePage(
          String(currentPerformance.performanceNo),
          currentPerformance.name,
          currentPerformance.upperPatchNo,
          getPatchName(currentPerformance.upperPatchNo),
          currentPerformance.lowerPatchNo,
          getPatchName(currentPerformance.lowerPatchNo));
        break;

      case PERFORMANCE_NAMING:
        if (!startedRenaming) {
          renamedPatch = "";
          startedRenaming = true;
        }

        charIndex++;
        if (charIndex >= TOTALCHARS) charIndex = 0;
        currentCharacter = CHARACTERS[charIndex];
        showRenamingPage(renamedPatch + currentCharacter);
        break;

      case PARAMETER:
        if (inPerformanceMode) {
          performanceIndex++;
          if (performanceIndex >= performances.size()) performanceIndex = 0;
          currentPerformance = performances[performanceIndex];

          for (int i = 0; i < patches.size(); i++) {
            if (patches[i].patchNo == currentPerformance.upperPatchNo) upperPatchIndex = i;
            if (patches[i].patchNo == currentPerformance.lowerPatchNo) lowerPatchIndex = i;
          }

          playMode = currentPerformance.mode;
          wholemode = (playMode == WHOLE);
          updateplayMode(0);

          upperSW = true;
          recallPatch(currentPerformance.upperPatchNo);
          upperSW = false;
          recallPatch(currentPerformance.lowerPatchNo);
        } else {
          if (upperSW) {
            upperPatchIndex++;
            if (upperPatchIndex >= patches.size()) upperPatchIndex = 0;
            patchNo = patches[upperPatchIndex].patchNo;
            recallPatch(patchNo);
          } else {
            lowerPatchIndex++;
            if (lowerPatchIndex >= patches.size()) lowerPatchIndex = 0;
            patchNo = patches[lowerPatchIndex].patchNo;
            recallPatch(patchNo);
          }
        }
        refreshPatchDisplayFromState();
        refreshScreen();
        break;

      case RECALL:
      case SAVE:
      case DELETE:
        patches.push(patches.shift());
        refreshScreen();
        break;

      case PATCHNAMING:
        if (charIndex == TOTALCHARS) charIndex = 0;
        currentCharacter = CHARACTERS[charIndex++];
        showRenamingPage(renamedPatch + currentCharacter);
        refreshScreen();
        break;

      case SETTINGS:
        settings::increment_setting();
        showSettingsPage();
        refreshScreen();
        break;

      case SETTINGSVALUE:
        settings::increment_setting_value();
        showSettingsPage();
        refreshScreen();
        break;
    }
  } else if ((encCW && encRead < encPrevious - 3) || (!encCW && encRead > encPrevious + 3)) {
    moved = true;

    switch (state) {

      case PERFORMANCE_DELETE:
        if (encCW) {
          performances.push(performances.shift());
        } else {
          performances.unshift(performances.pop());
        }
        break;

      case PERFORMANCE_SAVE:
        performanceIndex--;
        if (performanceIndex < 0) performanceIndex = performances.size() - 1;
        currentPerformance = performances[performanceIndex];
        showPerformancePage(
          String(currentPerformance.performanceNo),
          currentPerformance.name,
          currentPerformance.upperPatchNo,
          getPatchName(currentPerformance.upperPatchNo),
          currentPerformance.lowerPatchNo,
          getPatchName(currentPerformance.lowerPatchNo));
        break;

      case PERFORMANCE_RECALL:
        performanceIndex--;
        if (performanceIndex < 0) performanceIndex = performances.size() - 1;
        currentPerformance = performances[performanceIndex];
        showPerformancePage(
          String(currentPerformance.performanceNo),
          currentPerformance.name,
          currentPerformance.upperPatchNo,
          getPatchName(currentPerformance.upperPatchNo),
          currentPerformance.lowerPatchNo,
          getPatchName(currentPerformance.lowerPatchNo));
        break;

      case PERFORMANCE_NAMING:
        if (!startedRenaming) {
          renamedPatch = "";
          startedRenaming = true;
        }

        charIndex--;
        if (charIndex < 0) charIndex = TOTALCHARS - 1;
        currentCharacter = CHARACTERS[charIndex];
        showRenamingPage(renamedPatch + currentCharacter);
        break;

      case PARAMETER:
        if (inPerformanceMode) {
          performanceIndex--;
          if (performanceIndex < 0) performanceIndex = performances.size() - 1;
          currentPerformance = performances[performanceIndex];

          for (int i = 0; i < patches.size(); i++) {
            if (patches[i].patchNo == currentPerformance.upperPatchNo) upperPatchIndex = i;
            if (patches[i].patchNo == currentPerformance.lowerPatchNo) lowerPatchIndex = i;
          }

          playMode = currentPerformance.mode;
          wholemode = (playMode == WHOLE);
          updateplayMode(0);

          upperSW = true;
          recallPatch(currentPerformance.upperPatchNo);
          upperSW = false;
          recallPatch(currentPerformance.lowerPatchNo);
        } else {
          if (upperSW) {
            upperPatchIndex--;
            if (upperPatchIndex < 0) upperPatchIndex = patches.size() - 1;
            patchNo = patches[upperPatchIndex].patchNo;
            recallPatch(patchNo);
          } else {
            lowerPatchIndex--;
            if (lowerPatchIndex < 0) lowerPatchIndex = patches.size() - 1;
            patchNo = patches[lowerPatchIndex].patchNo;
            recallPatch(patchNo);
          }
        }
        refreshPatchDisplayFromState();
        refreshScreen();
        break;


      case RECALL:
      case SAVE:
      case DELETE:
        patches.unshift(patches.pop());
        refreshScreen();
        break;

      case PATCHNAMING:
        if (charIndex == -1) charIndex = TOTALCHARS - 1;
        currentCharacter = CHARACTERS[charIndex--];
        showRenamingPage(renamedPatch + currentCharacter);
        refreshScreen();
        break;

      case SETTINGS:
        settings::decrement_setting();
        showSettingsPage();
        refreshScreen();
        break;

      case SETTINGSVALUE:
        settings::decrement_setting_value();
        showSettingsPage();
        refreshScreen();
        break;
    }
  }

  if (moved) {
    encPrevious = encRead;
  }
}

String getPatchName(int patchNo) {
  for (int i = 0; i < patches.size(); i++) {
    if (patches[i].patchNo == patchNo) return patches[i].patchName;
  }
  return "-";
}

void setPerformancesOrdering(int no) {
  if (performances.size() < 2) return;
  while (performances.first().performanceNo != no) {
    performances.push(performances.shift());
  }
}

void checkChordHold() {
  if (chordHoldActive && chordHoldWaitingForNotes && chordHoldCaptureWindowActive) {
    if (millis() - chordHoldStartTime >= CHORD_HOLD_CAPTURE_WINDOW) {
      // Window is over, memorize chord from current voices
      memorizeChordFromVoices();
      chordHoldCaptureWindowActive = false;
      // Now chordHoldWaitingForNotes is false if a chord was captured
    }
  }
}

void loop() {

  checkSwitches();
  pollAllMCPs();
  checkEncoder();
  midi1.read(midiChannel);  //USB HOST MIDI Class Compliant
  MIDI.read(midiChannel);
  MIDI6.read(6);
  MIDI7.read(7);
  MIDI8.read(8);
  usbMIDI.read(midiChannel);
  LFODelayHandle();
  changeSpeed();
  checkChordHold();

  if (waitingToUpdate && (millis() - lastDisplayTriggerTime >= displayTimeout)) {
    refreshScreen();  // retrigger
    waitingToUpdate = false;
  }
}
