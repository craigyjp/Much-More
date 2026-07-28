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

// arp mode values (match the values your mode buttons write into P_arpMode)
enum ArpMode { ARP_OFF = 0,
               ARP_UP = 1,
               ARP_DOWN = 2,
               ARP_UPDOWN = 3,
               ARP_RANDOM = 4 };

// JP-8 behaviour: in SPLIT the arp runs on the lower section only
const bool arpLowerOnlyWhenSplit = true;

uint8_t arpPattern[12];  // held notes, in the order pressed
uint8_t arpLen = 0;      // how many notes are in the pattern
int16_t arpPos = -1;     // current step in the unfolded (octave) sequence
int8_t arpDir = +1;      // +1/-1 for Up-Down
uint8_t arpCurrentNote = 0;
uint8_t arpCurrentVel = 100;
bool arpNoteActive = false;

bool keyDownArp[128] = { false };      // physically held (within arp scope)
bool holdLatchedArp[128] = { false };  // kept by latch after release

// internal smoothed clock
float arpHzTarget = 1.0f;
float arpHzSmooth = 1.0f;
uint32_t arpLastSmoothUs = 0;
uint32_t arpNextStepUs = 0;
// =========================================================================

// ---- Parameter-display throttling (keeps TFT redraws off the arp clock) ----
bool paramDisplayDirty = false;
unsigned long lastParamDrawTime = 0;
const unsigned long paramDrawInterval = 45;  // ms; min spacing between param redraws

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
  delay(10);
  mcp15.begin(6, Wire1);

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

  // Arpeggiator init
  for (int i = 0; i < 128; i++) {
    keyDownArp[i] = false;
    holdLatchedArp[i] = false;
  }
  arpClearPattern();
  if (lowerData[P_arpRange] < 1 || lowerData[P_arpRange] > 4) lowerData[P_arpRange] = 1;
  arpHzTarget = LFOTEMPO[constrain(lowerData[P_arpRate], 0, 127)];
  arpHzSmooth = arpHzTarget;


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

  MIDI6.begin();
  MIDI6.turnThruOn(midi::Thru::Mode::Off);

  MIDI7.begin();
  MIDI7.turnThruOn(midi::Thru::Mode::Off);

  MIDI8.begin();
  MIDI8.turnThruOn(midi::Thru::Mode::Off);

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
    MIDI7.sendNoteOn(noteon, 64, i + 1);
    delay(1);
    MIDI7.sendNoteOff(noteon, 64, i + 1);
    delay(5);
    MIDI8.sendNoteOn(noteon, 64, i + 1);
    delay(1);
    MIDI8.sendNoteOff(noteon, 64, i + 1);
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
      }

      updateOsc1PulseLevel(1);
      break;

    case 26:
      if (upperSW) {
        upperData[P_osc2SubLevel] = (upperData[P_osc2SubLevel] + speed);
        upperData[P_osc2SubLevel] = constrain(upperData[P_osc2SubLevel], 0, 127);
        osc2SubLevelstr = upperData[P_osc2SubLevel];
      } else {
        lowerData[P_osc2SubLevel] = (lowerData[P_osc2SubLevel] + speed);
        lowerData[P_osc2SubLevel] = constrain(lowerData[P_osc2SubLevel], 0, 127);
        osc2SubLevelstr = lowerData[P_osc2SubLevel];
      }

      updateosc2SubLevel(1);
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
      }

      updateOsc2PulseLevel(1);
      break;

    case 14:
      if (upperSW) {
        upperData[P_osc2Interval] = (upperData[P_osc2Interval] + speed);
        upperData[P_osc2Interval] = constrain(upperData[P_osc2Interval], 0, 127);
        osc2Intervalstr = upperData[P_osc2Interval];
      } else {
        lowerData[P_osc2Interval] = (lowerData[P_osc2Interval] + speed);
        lowerData[P_osc2Interval] = constrain(lowerData[P_osc2Interval], 0, 127);
        osc2Intervalstr = lowerData[P_osc2Interval];
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
      }

      updatenoiseLevel(1);
      break;

    case 9:
      if (upperSW) {
        upperData[P_osc1TriangleLevel] = (upperData[P_osc1TriangleLevel] + speed);
        upperData[P_osc1TriangleLevel] = constrain(upperData[P_osc1TriangleLevel], 0, 127);
        osc1TriangleLevelstr = upperData[P_osc1TriangleLevel];
      } else {
        lowerData[P_osc1TriangleLevel] = (lowerData[P_osc1TriangleLevel] + speed);
        lowerData[P_osc1TriangleLevel] = constrain(lowerData[P_osc1TriangleLevel], 0, 127);
        osc1TriangleLevelstr = lowerData[P_osc1TriangleLevel];
      }

      updateOsc1TriangleLevel(1);
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
      }

      updateeffectPot1(1);
      break;

    case 54:
      if (upperSW) {
        upperData[P_effectsMix] = (upperData[P_effectsMix] + speed);
        upperData[P_effectsMix] = constrain(upperData[P_effectsMix], 0, 127);
        effectsMixstr = LINEARCENTREZERO[upperData[P_effectsMix]];
      } else {
        lowerData[P_effectsMix] = (lowerData[P_effectsMix] + speed);
        lowerData[P_effectsMix] = constrain(lowerData[P_effectsMix], 0, 127);
        effectsMixstr = LINEARCENTREZERO[lowerData[P_effectsMix]];
      }

      updateeffectsMix(1);
      break;

    case 55:
      if (upperSW) {
        upperData[P_effectPot3] = (upperData[P_effectPot3] + speed);
        upperData[P_effectPot3] = constrain(upperData[P_effectPot3], 0, 127);
        effectPot3str = upperData[P_effectPot3];
      } else {
        lowerData[P_effectPot3] = (lowerData[P_effectPot3] + speed);
        lowerData[P_effectPot3] = constrain(lowerData[P_effectPot3], 0, 127);
        effectPot3str = lowerData[P_effectPot3];
      }

      updateeffectPot3(1);
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
      }

      updateOsc2EnvDepth(1);
      break;


    case 57:
      if (upperSW) {
        upperData[P_vcfATDepth] = (upperData[P_vcfATDepth] + speed);
        upperData[P_vcfATDepth] = constrain(upperData[P_vcfATDepth], 0, 127);
        vcfATDepthstr = upperData[P_vcfATDepth];
      } else {
        lowerData[P_vcfATDepth] = (lowerData[P_vcfATDepth] + speed);
        lowerData[P_vcfATDepth] = constrain(lowerData[P_vcfATDepth], 0, 127);
        vcfATDepthstr = lowerData[P_vcfATDepth];
      }

      updatevcfATDepth(1);
      break;

    case 58:
      if (upperSW) {
        upperData[P_unisonDetune] = (upperData[P_unisonDetune] + speed);
        upperData[P_unisonDetune] = constrain(upperData[P_unisonDetune], 0, 127);
        unisonDetunestr = upperData[P_unisonDetune];
      } else {
        lowerData[P_unisonDetune] = (lowerData[P_unisonDetune] + speed);
        lowerData[P_unisonDetune] = constrain(lowerData[P_unisonDetune], 0, 127);
        unisonDetunestr = lowerData[P_unisonDetune];
      }

      updateunisonDetune(1);
      break;

    case 59:
      if (playMode == 1) {
        upperData[P_dualDetune] = (upperData[P_dualDetune] + speed);
        upperData[P_dualDetune] = constrain(upperData[P_dualDetune], 0, 127);
        updatedualDetune(1);
      }
      break;

    case 60:
      if (upperSW) {
        upperData[P_driftDepth] = (upperData[P_driftDepth] + speed);
        upperData[P_driftDepth] = constrain(upperData[P_driftDepth], 0, 127);
        driftDepthstr = upperData[P_driftDepth];
      } else {
        lowerData[P_driftDepth] = (lowerData[P_driftDepth] + speed);
        lowerData[P_driftDepth] = constrain(lowerData[P_driftDepth], 0, 127);
        driftDepthstr = lowerData[P_driftDepth];
      }

      updatedriftDepth(1);
      break;
  }

  //rotaryEncoderChanged(id, clockwise, speed);
}

void mainButtonChanged(Button *btn, bool released) {

  switch (btn->id) {

    // ----------------------------- ARPEGGIATOR -----------------------------
    case ARP_START_STOP_BUTTON:
      if (!released) {
        lowerData[P_arpStartStop] = !lowerData[P_arpStartStop];
        if (lowerData[P_arpStartStop]) {
          if (lowerData[P_arpMode] == ARP_OFF) lowerData[P_arpMode] = ARP_UP;  // default pattern
          arpStartTransport();
        } else {
          arpAllOff();
        }
        arpMsg(lowerData[P_arpStartStop] ? "Running" : "Stopped");
        updateArpLEDs();
      }
      break;

    case ARP_LATCH_BUTTON:
      if (!released) {
        lowerData[P_arpLatch] = !lowerData[P_arpLatch];
        if (!lowerData[P_arpLatch]) {
          // dropping latch: let go of anything not still physically held
          for (int i = 0; i < 128; i++) {
            if (holdLatchedArp[i] && !keyDownArp[i]) arpRemoveNote((byte)i);
            holdLatchedArp[i] = false;
          }
        }
        arpMsg(lowerData[P_arpLatch] ? "Latch On" : "Latch Off");
        updateArpLEDs();
      }
      break;

    case ARP_OCT1_BUTTON:
      if (!released) {
        lowerData[P_arpRange] = 1;
        if (arpPos >= arpUnfoldedLength()) arpPos = -1;
        arpMsg("Range 1 Octave");
        updateArpLEDs();
      }
      break;
    case ARP_OCT2_BUTTON:
      if (!released) {
        lowerData[P_arpRange] = 2;
        if (arpPos >= arpUnfoldedLength()) arpPos = -1;
        arpMsg("Range 2 Octaves");
        updateArpLEDs();
      }
      break;
    case ARP_OCT3_BUTTON:
      if (!released) {
        lowerData[P_arpRange] = 3;
        if (arpPos >= arpUnfoldedLength()) arpPos = -1;
        arpMsg("Range 3 Octaves");
        updateArpLEDs();
      }
      break;
    case ARP_OCT4_BUTTON:
      if (!released) {
        lowerData[P_arpRange] = 4;
        if (arpPos >= arpUnfoldedLength()) arpPos = -1;
        arpMsg("Range 4 Octaves");
        updateArpLEDs();
      }
      break;

    case ARP_UP_BUTTON:
      if (!released) {
        lowerData[P_arpMode] = ARP_UP;
        arpRestart();
        arpMsg("Mode Up");
        updateArpLEDs();
      }
      break;
    case ARP_DOWN_BUTTON:
      if (!released) {
        lowerData[P_arpMode] = ARP_DOWN;
        arpRestart();
        arpMsg("Mode Down");
        updateArpLEDs();
      }
      break;
    case ARP_UP_DOWN_BUTTON:
      if (!released) {
        lowerData[P_arpMode] = ARP_UPDOWN;
        arpRestart();
        arpMsg("Mode Up/Down");
        updateArpLEDs();
      }
      break;
    case ARP_RAND_BUTTON:
      if (!released) {
        lowerData[P_arpMode] = ARP_RANDOM;
        arpRestart();
        arpMsg("Mode Random");
        updateArpLEDs();
      }
      break;
      // ---------------------------------------------------------------------

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

    case DRIFT_BUTTON:
      if (!released) {
        panelData[P_driftSW] = !panelData[P_driftSW];
        myControlChange(midiChannel, CCdriftSW, panelData[P_driftSW]);
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

    case VCA_PUNCH_BUTTON:
      if (!released) {
        panelData[P_env3_punch] = !panelData[P_env3_punch];
        myControlChange(midiChannel, CCenv3_punch, panelData[P_env3_punch]);
      }
      break;

    case ENV2_3_ADSR_BUTTON:
      if (!released) {
        panelData[P_env2_env3_adsr] = !panelData[P_env2_env3_adsr];
        myControlChange(midiChannel, CCenv2_env3_adsr, panelData[P_env2_env3_adsr]);
      }
      break;

    case VCF_PUNCH_BUTTON:
      if (!released) {
        panelData[P_env2_punch] = !panelData[P_env2_punch];
        myControlChange(midiChannel, CCenv2_punch, panelData[P_env2_punch]);
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

    case FX_BYPASS_BUTTON:
      if (!released) {
        panelData[P_fx_Bypass] = !panelData[P_fx_Bypass];
        myControlChange(midiChannel, CCfx_Bypass, panelData[P_fx_Bypass]);
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
  getDelayTime();

  // ---------------- UPPER ----------------
  unsigned long currentMillisU = millis();
  if (numberOfNotesU < 0) numberOfNotesU = 0;          // safety clamp

  if (numberOfNotesU > 0) {
    // Retrigger ON: any rise in held-note count restarts the delay
    if (upperData[P_monoMulti] && (numberOfNotesU > oldnumberOfNotesU)) {
      previousMillisU = currentMillisU;
    }
    if (currentMillisU - previousMillisU >= intervalU) {
      if (!upperData[P_LFODelayGo]) {                  // edge only
        upperData[P_LFODelayGo] = 1;
        midiCCVoiceUpper(VB_FILTER_LFO3, upperData[P_filterLFO]);
      }
    } else {
      if (upperData[P_LFODelayGo]) {                   // edge only
        upperData[P_LFODelayGo] = 0;
        midiCCVoiceUpper(VB_FILTER_LFO3, 0);
      }
    }
  } else {
    upperData[P_LFODelayGo] = 1;
    previousMillisU = currentMillisU;                  // armed for next phrase
  }
  oldnumberOfNotesU = numberOfNotesU;                  // track up AND down, every pass


    // ---------------- LOWER ----------------
  unsigned long currentMillisL = millis();
  if (numberOfNotesL < 0) numberOfNotesL = 0;          // safety clamp

  if (numberOfNotesL > 0) {
    // Retrigger ON: any rise in held-note count restarts the delay
    if (lowerData[P_monoMulti] && (numberOfNotesL > oldnumberOfNotesL)) {
      previousMillisL = currentMillisL;
    }
    if (currentMillisL - previousMillisL >= intervalL) {
      if (!lowerData[P_LFODelayGo]) {                  // edge only
        lowerData[P_LFODelayGo] = 1;
        midiCCVoiceLower(VB_FILTER_LFO3, lowerData[P_filterLFO]);
      }
    } else {
      if (lowerData[P_LFODelayGo]) {                   // edge only
        lowerData[P_LFODelayGo] = 0;
        midiCCVoiceLower(VB_FILTER_LFO3, 0);
      }
    }
  } else {
    lowerData[P_LFODelayGo] = 1;
    previousMillisL = currentMillisL;                  // armed for next phrase
  }
  oldnumberOfNotesL = numberOfNotesL;                  // track up AND down, every pass

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

// ============================== ARP ENGINE ==============================

inline bool arpActive() {
  return (lowerData[P_arpStartStop] != 0) && (lowerData[P_arpMode] != ARP_OFF);
}

inline uint8_t arpRangeVal() {
  int r = lowerData[P_arpRange];
  if (r < 1) r = 1;
  if (r > 4) r = 4;
  return (uint8_t)r;
}

// Does this note belong to the arp, given the current play mode?
inline bool arpInScope(byte note) {
  if (playMode == 2 && arpLowerOnlyWhenSplit) return (note < splitPoint);
  return true;  // Whole & Dual: whole keyboard
}

inline bool anyArpKeyDown() {
  for (int i = 0; i < 128; i++)
    if (keyDownArp[i]) return true;
  return false;
}

bool arpPatternContains(uint8_t n) {
  for (uint8_t i = 0; i < arpLen; i++)
    if (arpPattern[i] == n) return true;
  return false;
}

void arpClearPattern() {
  arpLen = 0;
  arpPos = -1;
  arpDir = +1;
}

void arpAddNote(uint8_t n) {
  if (arpLen >= 12) return;
  if (arpPatternContains(n)) return;
  arpPattern[arpLen++] = n;
  if (arpLen == 1) {  // first note: start the sequence cleanly
    arpPos = -1;
    arpDir = +1;
  }
}

void arpRemoveNote(uint8_t n) {
  for (uint8_t i = 0; i < arpLen; i++) {
    if (arpPattern[i] == n) {
      for (uint8_t j = i; j + 1 < arpLen; j++) arpPattern[j] = arpPattern[j + 1];
      arpLen--;
      if (arpLen == 0) {
        arpPos = -1;
        arpDir = +1;
      } else {
        int16_t L = (int16_t)arpLen * (int16_t)arpRangeVal();
        if (arpPos >= L) arpPos = -1;
      }
      return;
    }
  }
}

inline int16_t arpUnfoldedLength() {
  return (int16_t)arpLen * (int16_t)arpRangeVal();
}

inline uint8_t arpUnfoldedNoteAt(int16_t p) {
  uint8_t idx = (uint8_t)(p % arpLen);
  uint8_t oct = (uint8_t)(p / arpLen);
  int16_t n = (int16_t)arpPattern[idx] + (int16_t)(12 * oct);
  if (n < 0) n = 0;
  if (n > 127) n = 127;
  return (uint8_t)n;
}

int16_t arpNextPos(int16_t L) {
  if (L <= 1) return 0;
  switch (lowerData[P_arpMode]) {
    case ARP_UP:
      return (int16_t)((arpPos + 1) % L);
    case ARP_DOWN:
      return (arpPos <= 0) ? (L - 1) : (arpPos - 1);
    case ARP_UPDOWN:
      {
        int16_t np = arpPos + arpDir;
        if (np >= L) {
          arpDir = -1;
          np = L - 2;
        }
        if (np < 0) {
          arpDir = +1;
          np = 1;
        }
        return np;
      }
    case ARP_RANDOM:
      return (int16_t)(random(L));
    default:
      return arpPos;
  }
}

// Release the currently sounding arp note (if any), on the correct voice/board
void arpStopCurrent() {
  if (!arpNoteActive) return;
  byte note = arpCurrentNote;

  if (playMode == 2 && arpLowerOnlyWhenSplit) {
    // SPLIT: lower only
    if (lowerData[P_keyboardMode] == 2) {
      commandMonoNoteOffLower(note);
    } else if (lowerData[P_keyboardMode] == 3) {
      commandUnisonNoteOffLower(note);
    } else {
      int v = voiceAssignmentLower[note];
      if (v >= 0 && v <= 5) releaseVoice(note, v);
    }
  } else if (playMode == 1) {
    // DUAL: both engines
    if (lowerData[P_keyboardMode] == 2) commandMonoNoteOffLower(note);
    else if (lowerData[P_keyboardMode] == 3) commandUnisonNoteOffLower(note);
    else {
      int vl = voiceAssignmentLower[note];
      if (vl >= 0 && vl <= 5) releaseVoice(note, vl);
    }

    if (upperData[P_keyboardMode] == 2) commandMonoNoteOffUpper(note);
    else if (upperData[P_keyboardMode] == 3) commandUnisonNoteOffUpper(note);
    else {
      int vu = voiceAssignmentUpper[note];
      if (vu >= 6 && vu <= 11) releaseVoice(note, vu);
    }
  } else {
    // WHOLE
    if (lowerData[P_keyboardMode] == 2) {
      commandMonoNoteOff(note);
    } else if (lowerData[P_keyboardMode] == 3) {
      commandUnisonNoteOff(note);
    } else {
      for (int v = 0; v < 12; v++) {
        if (voices[v].noteOn && voices[v].note == note) releaseVoice(note, v);
      }
    }
  }
  arpNoteActive = false;
}

// Play one arp note, using the existing voice-allocation rules
void arpPlayNote(uint8_t note, uint8_t vel) {

  // SPLIT: lower only
  if (playMode == 2 && arpLowerOnlyWhenSplit) {
    switch (lowerData[P_keyboardMode]) {
      case 0:
        {
          int v = getLowerSplitVoice(note);
          assignVoice(note, vel, v);
          voiceAssignmentLower[note] = v;
          voiceToNoteLower[v] = note;
        }
        break;
      case 1:
        {
          int v = getLowerSplitVoicePoly2(note);
          int old = voiceToNoteLower[v];
          if (old >= 0) {
            releaseVoice(old, v);
            voiceAssignmentLower[old] = -1;
          }
          assignVoice(note, vel, v);
          voiceAssignmentLower[note] = v;
          voiceToNoteLower[v] = note;
        }
        break;
      case 2: commandMonoNoteOnLower(note, vel, lowerData[P_NotePriority]); break;
      case 3: commandUnisonNoteOnLower(note, vel, lowerData[P_NotePriority]); break;
    }
    return;
  }

  // DUAL: drive both lower and upper
  if (playMode == 1) {
    // Lower
    if (lowerData[P_keyboardMode] == 1) {
      int v = getLowerSplitVoicePoly2(note);
      int old = voiceToNoteLower[v];
      if (old >= 0) {
        releaseVoice(old, v);
        voiceAssignmentLower[old] = -1;
      }
      assignVoice(note, vel, v);
      voiceAssignmentLower[note] = v;
      voiceToNoteLower[v] = note;
    } else if (lowerData[P_keyboardMode] == 0) {
      int v = getLowerSplitVoice(note);
      assignVoice(note, vel, v);
      voiceAssignmentLower[note] = v;
      voiceToNoteLower[v] = note;
    } else if (lowerData[P_keyboardMode] == 2) {
      commandMonoNoteOnLower(note, vel, lowerData[P_NotePriority]);
    } else if (lowerData[P_keyboardMode] == 3) {
      commandUnisonNoteOnLower(note, vel, lowerData[P_NotePriority]);
    }
    // Upper
    if (upperData[P_keyboardMode] == 1) {
      int v = getUpperSplitVoicePoly2(note);
      int old = voiceToNoteUpper[v - 6];
      if (old >= 0) {
        releaseVoice(old, v);
        voiceAssignmentUpper[old] = -1;
      }
      assignVoice(note, vel, v);
      voiceAssignmentUpper[note] = v;
      voiceToNoteUpper[v - 6] = note;
    } else if (upperData[P_keyboardMode] == 0) {
      int v = getUpperSplitVoice(note);
      assignVoice(note, vel, v);
      voiceAssignmentUpper[note] = v;
      voiceToNoteUpper[v - 6] = note;
    } else if (upperData[P_keyboardMode] == 2) {
      commandMonoNoteOnUpper(note, vel, upperData[P_NotePriority]);
    } else if (upperData[P_keyboardMode] == 3) {
      commandUnisonNoteOnUpper(note, vel, upperData[P_NotePriority]);
    }
    return;
  }

  // WHOLE
  if (playMode == 0) {
    int voiceNum = -1;
    switch (lowerData[P_keyboardMode]) {
      case 0:
        voiceNum = getVoiceNo(-1) - 1;
        assignVoice(note, vel, voiceNum);
        voiceAssignment[note] = voiceNum;
        break;
      case 1:
        voiceNum = getVoiceNoPoly2(-1) - 1;
        assignVoice(note, vel, voiceNum);
        voiceAssignment[note] = voiceNum;
        break;
      case 2: commandMonoNoteOn(note, vel); break;
      case 3: commandUnisonNoteOn(note, vel); break;
    }
    return;
  }
}

// Smooth the step rate so turning the Arp Rate knob doesn't zipper
inline void arpUpdateSmoothHz() {
  uint32_t now = micros();
  if (arpLastSmoothUs == 0) {
    arpLastSmoothUs = now;
    arpHzSmooth = arpHzTarget;
    return;
  }
  float dt = (now - arpLastSmoothUs) * 1e-6f;
  arpLastSmoothUs = now;
  const float tau = 0.20f;  // larger = smoother/slower response
  float a = dt / (tau + dt);
  arpHzSmooth += (arpHzTarget - arpHzSmooth) * a;
  if (arpHzSmooth < 0.02f) arpHzSmooth = 0.02f;
  if (arpHzSmooth > 20.0f) arpHzSmooth = 20.0f;
}

inline bool arpShouldStepNow() {
  arpUpdateSmoothHz();
  uint32_t now = micros();
  if (arpNextStepUs == 0) {
    arpNextStepUs = now;
    return true;  // step immediately on (re)start
  }
  if ((int32_t)(now - arpNextStepUs) < 0) return false;
  float intervalUsF = 1000000.0f / arpHzSmooth;
  uint32_t intervalUs = (uint32_t)(intervalUsF + 0.5f);
  arpNextStepUs += intervalUs;  // advance by one interval to reduce jitter
  if ((int32_t)(now - arpNextStepUs) > (int32_t)intervalUs) {
    arpNextStepUs = now + intervalUs;  // resync if we fell behind
  }
  return true;
}

// Called every loop()
void arpEngine() {
  if (!arpActive() || arpLen == 0) {
    if (arpNoteActive) arpStopCurrent();
    return;
  }
  if (!arpShouldStepNow()) return;

  if (arpNoteActive) arpStopCurrent();  // tight JP-8 feel: off at step boundary

  int16_t L = arpUnfoldedLength();
  if (L <= 0) return;

  arpPos = arpNextPos(L);
  uint8_t nextNote = arpUnfoldedNoteAt(arpPos);

  arpPlayNote(nextNote, arpCurrentVel);
  arpCurrentNote = nextNote;
  arpNoteActive = true;
}

// ---- transport helpers used by the buttons ----
void arpStartTransport() {
  arpPos = -1;
  arpDir = +1;
  arpNextStepUs = 0;
  arpLastSmoothUs = 0;
  arpHzTarget = LFOTEMPO[constrain(lowerData[P_arpRate], 0, 127)];
  arpHzSmooth = arpHzTarget;
  updateArpLEDs();
}

void arpAllOff() {
  if (arpNoteActive) arpStopCurrent();
  arpClearPattern();
  for (int i = 0; i < 128; i++) {
    keyDownArp[i] = false;
    holdLatchedArp[i] = false;
  }
}

void arpRestart() {  // clean restart on a mode change (keeps held notes)
  if (arpNoteActive) arpStopCurrent();
  arpPos = -1;
  arpDir = +1;
}

void arpMsg(const String &val) {
  showCurrentParameterPage("Arpeggiator", val);
  startParameterDisplay();
}

void updateArpLEDs() {
  // Always present a valid mode/range so the panel shows the current
  // selection even before the arp is started.
  if (lowerData[P_arpRange] < 1 || lowerData[P_arpRange] > 4) lowerData[P_arpRange] = 1;
  if (lowerData[P_arpMode] < ARP_UP || lowerData[P_arpMode] > ARP_RANDOM) lowerData[P_arpMode] = ARP_UP;

  uint8_t r = arpRangeVal();
  mcp1.digitalWrite(ARP_OCT1_LED, (r == 1) ? HIGH : LOW);
  mcp1.digitalWrite(ARP_OCT2_LED, (r == 2) ? HIGH : LOW);
  mcp1.digitalWrite(ARP_OCT3_LED, (r == 3) ? HIGH : LOW);
  mcp1.digitalWrite(ARP_OCT4_LED, (r == 4) ? HIGH : LOW);
  midiCCDisplaySW(CCarpRange, r);

  mcp1.digitalWrite(ARP_UP_LED, (lowerData[P_arpMode] == ARP_UP) ? HIGH : LOW);
  mcp1.digitalWrite(ARP_DOWN_LED, (lowerData[P_arpMode] == ARP_DOWN) ? HIGH : LOW);
  mcp1.digitalWrite(ARP_UP_DOWN_LED, (lowerData[P_arpMode] == ARP_UPDOWN) ? HIGH : LOW);
  mcp1.digitalWrite(ARP_RAND_LED, (lowerData[P_arpMode] == ARP_RANDOM) ? HIGH : LOW);
  midiCCDisplaySW(CCarpMode, lowerData[P_arpMode]);

  mcp2.digitalWrite(ARP_START_STOP_LED, lowerData[P_arpStartStop] ? HIGH : LOW);
  midiCCDisplaySW(CCarpStartStop, lowerData[P_arpStartStop]);
  mcp2.digitalWrite(ARP_LATCH_LED, lowerData[P_arpLatch] ? HIGH : LOW);
  midiCCDisplaySW(CCarpLatch, lowerData[P_arpLatch]);
}
// =========================================================================

void myNoteOn(byte channel, byte note, byte velocity) {

  numberOfNotesU++;
  numberOfNotesL++;

  // Every key press pulses NOTES_HELD to re-arm the DCO LFO1 delay.
  // Sent for each press, not just the first, so the DCOs can retrigger.
  midiCCDCOUpper(CC_NOTES_HELD, 127);
  midiCCDCOLower(CC_NOTES_HELD, 127);

  prevNote = note;

  // ---- ARP: capture held notes; the chord itself does not sound ----
  if (arpActive() && arpInScope(note)) {
    // Latch: the first key of a new phrase clears the previous chord
    if (lowerData[P_arpLatch] && !anyArpKeyDown()) {
      arpClearPattern();
      for (int i = 0; i < 128; i++) holdLatchedArp[i] = false;
    }
    keyDownArp[note] = true;
    holdLatchedArp[note] = false;
    arpAddNote(note);
    arpCurrentVel = velocity;
    return;  // arp consumes this key
  }

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
          break;                                             
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


  if (numberOfNotesU > 0) {
    numberOfNotesU--;  
  }
  if (numberOfNotesL > 0) {
    numberOfNotesL--;
  }

  if (numberOfNotesU == 0) {
    midiCCDCOUpper(CC_NOTES_HELD, 0);
  }
  if (numberOfNotesL == 0) {
    midiCCDCOLower(CC_NOTES_HELD, 0);
  }

  // ---- ARP: release key from the pattern (or keep it if latched) ----
  if (arpActive() && arpInScope(note)) {
    keyDownArp[note] = false;
    if (lowerData[P_arpLatch]) {
      holdLatchedArp[note] = true;  // stays until a new phrase starts
    } else {
      arpRemoveNote(note);
    }
    return;  // arp consumes this key-up
  }

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
          if (upperVoice >= 6 && upperVoice <= 11 && voiceToNoteUpper[upperVoice - 6] == note) {
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
            if (lowerVoice >= 0 && lowerVoice <= 5 && voiceToNoteLower[lowerVoice] == note) {
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
      MIDI7.sendNoteOn(note, velocity, voiceIdx + 1);  // lower board, voices 1-6
    } else {
      MIDI8.sendNoteOn(note, velocity, voiceIdx - 5);  // upper board, voices 1-6
    }

    voiceOn[voiceIdx] = true;
  }
}

void releaseVoice(byte note, int voiceIdx) {
  if (voiceIdx >= 0 && voiceIdx < 12 && voices[voiceIdx].note == note) {
    if (voiceIdx < 6) {
      MIDI7.sendNoteOff(note, 0, voiceIdx + 1);  // lower board, voices 1-6
    } else {
      MIDI8.sendNoteOff(note, 0, voiceIdx - 5);  // upper board, voices 1-6
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
  midiCCDCOLower(WSallNotesOff, 127);
  midiCCDCOUpper(WSallNotesOff, 127);
}

FLASHMEM void updateLFO2Rate(boolean announce) {

  if (announce) {
    showCurrentParameterPage("LFO2 Rate", String(LFO2Ratestr) + " Hz");
    startParameterDisplay();
  }
  if (upperSW) {
    midiCCDCOUpper(CC_LFO2_RATE, upperData[P_LFO2Rate]);
    midiCCOut(CCLFO2Rate, upperData[P_LFO2Rate]);
    midiCCDisplay(CCLFO2Rate, upperData[P_LFO2Rate]);
  } else {
    midiCCDCOLower(CC_LFO2_RATE, lowerData[P_LFO2Rate]);
    midiCCOut(CCLFO2Rate, lowerData[P_LFO2Rate]);
    midiCCDisplay(CCLFO2Rate, lowerData[P_LFO2Rate]);
    if (wholemode) {
      midiCCDCOUpper(CC_LFO2_RATE, lowerData[P_LFO2Rate]);
    }
  }
}

FLASHMEM void updatefmDepth(boolean announce) {
  if (announce) {
    showCurrentParameterPage("FM Depth", int(fmDepthstr));
    startParameterDisplay();
  }
  if (upperSW) {
    midiCCDCOUpper(CC_LFO1_FM_DEPTH, upperData[P_fmDepth]);
    midiCCOut(CCfmDepth, upperData[P_fmDepth]);
    midiCCDisplay(CCfmDepth, upperData[P_fmDepth]);
  } else {
    midiCCDCOLower(CC_LFO1_FM_DEPTH, lowerData[P_fmDepth]);
    midiCCOut(CCfmDepth, lowerData[P_fmDepth]);
    midiCCDisplay(CCfmDepth, lowerData[P_fmDepth]);
    if (wholemode) {
      midiCCDCOUpper(CC_LFO1_FM_DEPTH, lowerData[P_fmDepth]);
    }
  }
}

FLASHMEM void updateATDepth(boolean announce) {
  if (announce) {
    showCurrentParameterPage("AT Depth", int(ATDepthstr));
    startParameterDisplay();
  }
  if (upperSW) {
    midiCCDCOUpper(CC_AT_FM_DEPTH, upperData[P_ATDepth]);
    midiCCOut(CCATDepth, upperData[P_ATDepth]);
    midiCCDisplay(CCATDepth, upperData[P_ATDepth]);
  } else {
    midiCCDCOLower(CC_AT_FM_DEPTH, lowerData[P_ATDepth]);
    midiCCOut(CCATDepth, lowerData[P_ATDepth]);
    midiCCDisplay(CCATDepth, lowerData[P_ATDepth]);
    if (wholemode) {
      midiCCDCOUpper(CC_AT_FM_DEPTH, lowerData[P_ATDepth]);
    }
  }
}

FLASHMEM void updateosc2PW(boolean announce) {
  if (announce) {
    showCurrentParameterPage("OSC2 PW", String(osc2PWstr) + " %");
    startParameterDisplay();
  }
  if (upperSW) {
    midiCCDCOUpper(CC_DCO2_PULSE_WIDTH, upperData[P_osc2PW]);
    midiCCOut(CCosc2PW, upperData[P_osc2PW]);
    midiCCDisplay(CCosc2PW, upperData[P_osc2PW]);
  } else {
    midiCCDCOLower(CC_DCO2_PULSE_WIDTH, lowerData[P_osc2PW]);
    midiCCOut(CCosc2PW, lowerData[P_osc2PW]);
    midiCCDisplay(CCosc2PW, lowerData[P_osc2PW]);
    if (wholemode) {
      midiCCDCOUpper(CC_DCO2_PULSE_WIDTH, lowerData[P_osc2PW]);
    }
  }
}

FLASHMEM void updateosc2PWM(boolean announce) {
  if (announce) {
    showCurrentParameterPage("OSC2 LFO PWM", int(osc2PWMstr));
    startParameterDisplay();
  }
  if (upperSW) {
    midiCCDCOUpper(CC_DCO2_LFO2_PWM, upperData[P_osc2PWM]);
    midiCCOut(CCosc2PWM, upperData[P_osc2PWM]);
    midiCCDisplay(CCosc2PWM, upperData[P_osc2PWM]);
  } else {
    midiCCDCOLower(CC_DCO2_LFO2_PWM, lowerData[P_osc2PWM]);
    midiCCOut(CCosc2PWM, lowerData[P_osc2PWM]);
    midiCCDisplay(CCosc2PWM, lowerData[P_osc2PWM]);
    if (wholemode) {
      midiCCDCOUpper(CC_DCO2_LFO2_PWM, lowerData[P_osc2PWM]);
    }
  }
}

FLASHMEM void updateosc1PW(boolean announce) {

  if (announce) {
    showCurrentParameterPage("OSC1 PW", String(osc1PWstr) + " %");
    startParameterDisplay();
  }
  if (upperSW) {
    midiCCDCOUpper(CC_DCO1_PULSE_WIDTH, upperData[P_osc1PW]);
    midiCCOut(CCosc1PW, upperData[P_osc1PW]);
    midiCCDisplay(CCosc1PW, upperData[P_osc1PW]);
  } else {
    midiCCDCOLower(CC_DCO1_PULSE_WIDTH, lowerData[P_osc1PW]);
    midiCCOut(CCosc1PW, lowerData[P_osc1PW]);
    midiCCDisplay(CCosc1PW, lowerData[P_osc1PW]);
    if (wholemode) {
      midiCCDCOUpper(CC_DCO1_PULSE_WIDTH, lowerData[P_osc1PW]);
    }
  }
}

FLASHMEM void updateosc1PWM(boolean announce) {
  if (announce) {
    showCurrentParameterPage("OSC1 LFO PWM", int(osc1PWMstr));
    startParameterDisplay();
  }
  if (upperSW) {
    midiCCDCOUpper(CC_DCO1_LFO2_PWM, upperData[P_osc1PWM]);
    midiCCOut(CCosc1PWM, upperData[P_osc1PWM]);
    midiCCDisplay(CCosc1PWM, upperData[P_osc1PWM]);
  } else {
    midiCCDCOLower(CC_DCO1_LFO2_PWM, lowerData[P_osc1PWM]);
    midiCCOut(CCosc1PWM, lowerData[P_osc1PWM]);
    midiCCDisplay(CCosc1PWM, lowerData[P_osc1PWM]);
    if (wholemode) {
      midiCCDCOUpper(CC_DCO1_LFO2_PWM, lowerData[P_osc1PWM]);
    }
  }
}

FLASHMEM void updateosc1envPWM(boolean announce) {
  if (announce) {
    showCurrentParameterPage("OSC1 ENV1 PWM", int(osc1PWMstr));
    startParameterDisplay();
  }
  if (upperSW) {
    midiCCDCOUpper(CC_ENV_DCO1_PWM, upperData[P_osc1envPWM]);
    midiCCOut(CCosc1envPWM, upperData[P_osc1envPWM]);
    midiCCDisplay(CCosc1envPWM, upperData[P_osc1envPWM]);
  } else {
    midiCCDCOLower(CC_ENV_DCO1_PWM, lowerData[P_osc1envPWM]);
    midiCCOut(CCosc1envPWM, lowerData[P_osc1envPWM]);
    midiCCDisplay(CCosc1envPWM, lowerData[P_osc1envPWM]);
    if (wholemode) {
      midiCCDCOUpper(CC_ENV_DCO1_PWM, lowerData[P_osc1envPWM]);
    }
  }
}

FLASHMEM void updateosc2envPWM(boolean announce) {
  if (announce) {
    showCurrentParameterPage("OSC1 ENV2 PWM", int(osc2PWMstr));
    startParameterDisplay();
  }
  if (upperSW) {
    midiCCDCOUpper(CC_ENV_DCO2_PWM, upperData[P_osc2envPWM]);
    midiCCOut(CCosc2envPWM, upperData[P_osc2envPWM]);
    midiCCDisplay(CCosc2envPWM, upperData[P_osc2envPWM]);
  } else {
    midiCCDCOLower(CC_ENV_DCO2_PWM, lowerData[P_osc2envPWM]);
    midiCCOut(CCosc2envPWM, lowerData[P_osc2envPWM]);
    midiCCDisplay(CCosc2envPWM, lowerData[P_osc2envPWM]);
    if (wholemode) {
      midiCCDCOUpper(CC_ENV_DCO2_PWM, lowerData[P_osc2envPWM]);
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
      midiCCDCOUpper(CC_DCO1_OCTAVE, 127);
      midiCCDisplaySW(CCosc1Oct, 2);
      mcp5.digitalWrite(DCO1_OCT_LED_RED, LOW);
      mcp5.digitalWrite(DCO1_OCT_LED_GREEN, HIGH);
    } else if (upperData[P_osc1Range] == 1) {
      if (announce) {
        showCurrentParameterPage("Osc1 Range", String("16"));
        startParameterDisplay();
      }
      midiCCOut(CCosc1Oct, 1);
      midiCCDCOUpper(CC_DCO1_OCTAVE, 64);
      midiCCDisplaySW(CCosc1Oct, 1);
      mcp5.digitalWrite(DCO1_OCT_LED_RED, HIGH);
      mcp5.digitalWrite(DCO1_OCT_LED_GREEN, HIGH);
    } else {
      if (announce) {
        showCurrentParameterPage("Osc1 Range", String("32"));
        startParameterDisplay();
      }
      midiCCOut(CCosc1Oct, 0);
      midiCCDCOUpper(CC_DCO1_OCTAVE, 0);
      midiCCDisplaySW(CCosc1Oct, 0);
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
      midiCCDCOLower(CC_DCO1_OCTAVE, 127);
      midiCCDisplaySW(CCosc1Oct, 2);
      if (wholemode) {
        midiCCDCOUpper(CC_DCO1_OCTAVE, 127);
      }
      mcp5.digitalWrite(DCO1_OCT_LED_RED, LOW);
      mcp5.digitalWrite(DCO1_OCT_LED_GREEN, HIGH);
    } else if (lowerData[P_osc1Range] == 1) {
      if (announce) {
        showCurrentParameterPage("Osc1 Range", String("16"));
        startParameterDisplay();
      }
      midiCCOut(CCosc1Oct, 1);
      midiCCDCOLower(CC_DCO1_OCTAVE, 64);
      midiCCDisplaySW(CCosc1Oct, 1);
      if (wholemode) {
        midiCCDCOUpper(CC_DCO1_OCTAVE, 64);
      }
      mcp5.digitalWrite(DCO1_OCT_LED_RED, HIGH);
      mcp5.digitalWrite(DCO1_OCT_LED_GREEN, HIGH);
    } else {
      if (announce) {
        showCurrentParameterPage("Osc1 Range", String("32"));
        startParameterDisplay();
      }
      midiCCOut(CCosc1Oct, 0);
      midiCCDCOLower(CC_DCO1_OCTAVE, 0);
      midiCCDisplaySW(CCosc1Oct, 0);
      if (wholemode) {
        midiCCDCOUpper(CC_DCO1_OCTAVE, 0);
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
      midiCCDCOUpper(CC_DCO2_OCTAVE, 127);
      midiCCDisplaySW(CCosc2Oct, 2);
      midiCCOut(CCosc2Oct, 2);
      mcp7.digitalWrite(DCO2_OCT_LED_RED, LOW);
      mcp7.digitalWrite(DCO2_OCT_LED_GREEN, HIGH);
    } else if (upperData[P_osc2Range] == 1) {
      if (announce) {
        showCurrentParameterPage("Osc2 Range", String("16"));
        startParameterDisplay();
      }
      midiCCDCOUpper(CC_DCO2_OCTAVE, 64);
      midiCCDisplaySW(CCosc2Oct, 1);
      midiCCOut(CCosc2Oct, 1);
      mcp7.digitalWrite(DCO2_OCT_LED_RED, HIGH);
      mcp7.digitalWrite(DCO2_OCT_LED_GREEN, HIGH);
    } else {
      if (announce) {
        showCurrentParameterPage("Osc2 Range", String("32"));
        startParameterDisplay();
      }
      midiCCOut(CCosc2Oct, 0);
      midiCCDCOUpper(CC_DCO2_OCTAVE, 0);
      midiCCDisplaySW(CCosc2Oct, 0);
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
      midiCCDCOLower(CC_DCO2_OCTAVE, 127);
      midiCCDisplaySW(CCosc2Oct, 2);
      if (wholemode) {
        midiCCDCOUpper(CC_DCO2_OCTAVE, 127);
      }
      mcp7.digitalWrite(DCO2_OCT_LED_RED, LOW);
      mcp7.digitalWrite(DCO2_OCT_LED_GREEN, HIGH);
    } else if (lowerData[P_osc2Range] == 1) {
      if (announce) {
        showCurrentParameterPage("Osc2 Range", String("16"));
        startParameterDisplay();
      }
      midiCCOut(CCosc2Oct, 1);
      midiCCDCOLower(CC_DCO2_OCTAVE, 64);
      midiCCDisplaySW(CCosc2Oct, 1);
      if (wholemode) {
        midiCCDCOUpper(CC_DCO2_OCTAVE, 64);
      }
      mcp7.digitalWrite(DCO2_OCT_LED_RED, HIGH);
      mcp7.digitalWrite(DCO2_OCT_LED_GREEN, HIGH);
    } else {
      if (announce) {
        showCurrentParameterPage("Osc2 Range", String("32"));
        startParameterDisplay();
      }
      midiCCOut(CCosc2Oct, 0);
      midiCCDCOLower(CC_DCO2_OCTAVE, 0);
      midiCCDisplaySW(CCosc2Oct, 0);
      if (wholemode) {
        midiCCDCOUpper(CC_DCO2_OCTAVE, 0);
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
    midiCCDCOUpper(CC_PORTAMENTO_TIME, upperData[P_glideTime]);
    midiCCOut(CCglideTime, upperData[P_glideTime]);
    midiCCDisplay(CCglideTime, upperData[P_glideTime]);
  } else {
    midiCCDCOLower(CC_PORTAMENTO_TIME, lowerData[P_glideTime]);
    midiCCOut(CCglideTime, lowerData[P_glideTime]);
    midiCCDisplay(CCglideTime, lowerData[P_glideTime]);
    if (wholemode) {
      midiCCDCOUpper(CC_PORTAMENTO_TIME, lowerData[P_glideTime]);
    }
  }
}

FLASHMEM void updateosc2Detune(boolean announce) {
  uint8_t v = upperSW ? upperData[P_osc2Detune] : lowerData[P_osc2Detune];
  int det = (v < 64) ? ((int)v - 63) : ((int)v - 64);

  if (announce) {
    String disp = (det > 0) ? ("+" + String(det)) : String(det);
    showCurrentParameterPage("OSC2 Detune", disp);
    startParameterDisplay();
  }
  if (upperSW) {
    midiCCDCOUpper(CC_DCO2_DETUNE, upperData[P_osc2Detune]);
    midiCCOut(CCosc2Detune, upperData[P_osc2Detune]);
    midiCCDisplay(CCosc2Detune, upperData[P_osc2Detune]);
  } else {
    midiCCDCOLower(CC_DCO2_DETUNE, lowerData[P_osc2Detune]);
    midiCCOut(CCosc2Detune, lowerData[P_osc2Detune]);
    midiCCDisplay(CCosc2Detune, lowerData[P_osc2Detune]);
    if (wholemode) {
      midiCCDCOUpper(CC_DCO2_DETUNE, lowerData[P_osc2Detune]);
    }
  }
}

FLASHMEM void updatedriftDepth(boolean announce) {
  if (announce) {
    showCurrentParameterPage("Analogue Drift", int(driftDepthstr));
    startParameterDisplay();
  }
  if (upperSW) {
    midiCCDCOUpper(CC_DRIFT_DEPTH, upperData[P_driftDepth]);
    midiCCOut(CCdriftDepth, upperData[P_driftDepth]);
    midiCCDisplay(CCdriftDepth, upperData[P_driftDepth]);
  } else {
    midiCCDCOLower(CC_DRIFT_DEPTH, lowerData[P_driftDepth]);
    midiCCOut(CCdriftDepth, lowerData[P_driftDepth]);
    midiCCDisplay(CCdriftDepth, lowerData[P_driftDepth]);
    if (wholemode) {
      midiCCDCOUpper(CC_DRIFT_DEPTH, lowerData[P_driftDepth]);
    }
  }
}

FLASHMEM void updatedualDetune(boolean announce) {
  uint8_t v = upperData[P_dualDetune];
  int det = (v < 64) ? ((int)v - 63) : ((int)v - 64);

  if (announce) {
    String disp = (det > 0) ? ("+" + String(det)) : String(det);
    showCurrentParameterPage("Dual Detune", disp);
    startParameterDisplay();
  }
  if (playMode == 1) {
    midiCCDCOUpper(CC_VOICE_DETUNE, upperData[P_dualDetune]);
    midiCCOut(CCdualDetune, upperData[P_dualDetune]);
    midiCCDisplay(CCdualDetune, upperData[P_dualDetune]);
  } else {
    if (wholemode) {
      midiCCDCOUpper(CC_VOICE_DETUNE, 64);
    }
    midiCCOut(CCdualDetune, 64);
    midiCCDisplay(CCdualDetune, 64);
  }
}

FLASHMEM void updateunisonDetune(boolean announce) {

  if (announce) {
    showCurrentParameterPage("Unsion Detune", int(unisonDetunestr));
    startParameterDisplay();
  }
  if (upperSW) {
    midiCCDCOUpper(CC_UNISON_DETUNE, upperData[P_unisonDetune]);
    midiCCOut(CCunisonDetune, upperData[P_unisonDetune]);
    midiCCDisplay(CCunisonDetune, upperData[P_unisonDetune]);
  } else {
    midiCCDCOLower(CC_UNISON_DETUNE, lowerData[P_unisonDetune]);
    midiCCOut(CCunisonDetune, lowerData[P_unisonDetune]);
    midiCCDisplay(CCunisonDetune, lowerData[P_unisonDetune]);
    if (wholemode) {
      midiCCDCOUpper(CC_UNISON_DETUNE, lowerData[P_unisonDetune]);
    }
  }
}

FLASHMEM void updateosc2Interval(boolean announce) {
  uint8_t v = upperSW ? upperData[P_osc2Interval] : lowerData[P_osc2Interval];
  int semis = (int)roundf(((float)v - 64.0f) / 64.0f * 12.0f);
  if (semis > 12) semis = 12;
  if (semis < -12) semis = -12;

  if (announce) {
    String disp = (semis > 0) ? ("+" + String(semis)) : String(semis);
    showCurrentParameterPage("OSC2 Interval", disp);
    startParameterDisplay();
  }
  if (upperSW) {
    midiCCDCOUpper(CC_DCO2_INTERVAL, upperData[P_osc2Interval]);
    midiCCOut(CCosc2Interval, upperData[P_osc2Interval]);
    midiCCDisplay(CCosc2Interval, upperData[P_osc2Interval]);
  } else {
    midiCCDCOLower(CC_DCO2_INTERVAL, lowerData[P_osc2Interval]);
    midiCCOut(CCosc2Interval, lowerData[P_osc2Interval]);
    midiCCDisplay(CCosc2Interval, lowerData[P_osc2Interval]);
    if (wholemode) {
      midiCCDCOUpper(CC_DCO2_INTERVAL, lowerData[P_osc2Interval]);
    }
  }
}

FLASHMEM void updatenoiseLevel(boolean announce) {
  if (announce) {
    showCurrentParameterPage("Noise Level", String(noiseLevelstr));
    startParameterDisplay();
  }
  if (upperSW) {
    midiCCVoiceUpper(VB_NOISE_LEVEL, upperData[P_noiseLevel]);
    midiCCOut(CCnoiseLevel, upperData[P_noiseLevel]);
    midiCCDisplay(CCnoiseLevel, upperData[P_noiseLevel]);
  } else {
    midiCCVoiceLower(VB_NOISE_LEVEL, lowerData[P_noiseLevel]);
    midiCCOut(CCnoiseLevel, lowerData[P_noiseLevel]);
    midiCCDisplay(CCnoiseLevel, lowerData[P_noiseLevel]);
    if (wholemode) {
      midiCCVoiceUpper(VB_NOISE_LEVEL, lowerData[P_noiseLevel]);
    }
  }
}

FLASHMEM void updateOsc2SawLevel(boolean announce) {
  if (announce) {
    showCurrentParameterPage("OSC2 Saw", int(osc2SawLevelstr));
    startParameterDisplay();
  }
  if (upperSW) {
    midiCCDCOUpper(CC_DCO2_SAW_LEVEL, upperData[P_osc2SawLevel]);
    midiCCOut(CCosc2SawLevel, upperData[P_osc2SawLevel]);
    midiCCDisplay(CCosc2SawLevel, upperData[P_osc2SawLevel]);
  } else {
    midiCCDCOLower(CC_DCO2_SAW_LEVEL, lowerData[P_osc2SawLevel]);
    midiCCOut(CCosc2SawLevel, lowerData[P_osc2SawLevel]);
    midiCCDisplay(CCosc2SawLevel, lowerData[P_osc2SawLevel]);
    if (wholemode) {
      midiCCDCOUpper(CC_DCO2_SAW_LEVEL, lowerData[P_osc2SawLevel]);
    }
  }
}

FLASHMEM void updateOsc1SawLevel(boolean announce) {
  if (announce) {
    showCurrentParameterPage("OSC1 Saw", int(osc1SawLevelstr));
    startParameterDisplay();
  }
  if (upperSW) {
    midiCCDCOUpper(CC_DCO1_SAW_LEVEL, upperData[P_osc1SawLevel]);
    midiCCOut(CCosc1SawLevel, upperData[P_osc1SawLevel]);
    midiCCDisplay(CCosc1SawLevel, upperData[P_osc1SawLevel]);
  } else {
    midiCCDCOLower(CC_DCO1_SAW_LEVEL, lowerData[P_osc1SawLevel]);
    midiCCOut(CCosc1SawLevel, lowerData[P_osc1SawLevel]);
    midiCCDisplay(CCosc1SawLevel, lowerData[P_osc1SawLevel]);
    if (wholemode) {
      midiCCDCOUpper(CC_DCO1_SAW_LEVEL, lowerData[P_osc1SawLevel]);
    }
  }
}

FLASHMEM void updateOsc2PulseLevel(boolean announce) {
  if (announce) {
    showCurrentParameterPage("OSC2 Pulse", int(osc2PulseLevelstr));
    startParameterDisplay();
  }
  if (upperSW) {
    midiCCDCOUpper(CC_DCO2_PULSE_LEVEL, upperData[P_osc2PulseLevel]);
    midiCCOut(CCosc2PulseLevel, upperData[P_osc2PulseLevel]);
    midiCCDisplay(CCosc2PulseLevel, upperData[P_osc2PulseLevel]);
  } else {
    midiCCDCOLower(CC_DCO2_PULSE_LEVEL, lowerData[P_osc2PulseLevel]);
    midiCCOut(CCosc2PulseLevel, lowerData[P_osc2PulseLevel]);
    midiCCDisplay(CCosc2PulseLevel, lowerData[P_osc2PulseLevel]);
    if (wholemode) {
      midiCCDCOUpper(CC_DCO2_PULSE_LEVEL, lowerData[P_osc2PulseLevel]);
    }
  }
}

FLASHMEM void updateOsc1PulseLevel(boolean announce) {
  if (announce) {
    showCurrentParameterPage("OSC1 Pulse", int(osc1PulseLevelstr));
    startParameterDisplay();
  }
  if (upperSW) {
    midiCCDCOUpper(CC_DCO1_PULSE_LEVEL, upperData[P_osc1PulseLevel]);
    midiCCOut(CCosc1PulseLevel, upperData[P_osc1PulseLevel]);
    midiCCDisplay(CCosc1PulseLevel, upperData[P_osc1PulseLevel]);
  } else {
    midiCCDCOLower(CC_DCO1_PULSE_LEVEL, lowerData[P_osc1PulseLevel]);
    midiCCOut(CCosc1PulseLevel, lowerData[P_osc1PulseLevel]);
    midiCCDisplay(CCosc1PulseLevel, lowerData[P_osc1PulseLevel]);
    if (wholemode) {
      midiCCDCOUpper(CC_DCO1_PULSE_LEVEL, lowerData[P_osc1PulseLevel]);
    }
  }
}

FLASHMEM void updateOsc1TriangleLevel(boolean announce) {
  if (announce) {
    showCurrentParameterPage("OSC1 Triangle", int(osc1TriangleLevelstr));
    startParameterDisplay();
  }
  if (upperSW) {
    midiCCDCOUpper(CC_DCO1_TRI_LEVEL, upperData[P_osc1TriangleLevel]);
    midiCCOut(CCosc1TriangleLevel, upperData[P_osc1TriangleLevel]);
    midiCCDisplay(CCosc1TriangleLevel, upperData[P_osc1TriangleLevel]);
  } else {
    midiCCDCOLower(CC_DCO1_TRI_LEVEL, lowerData[P_osc1TriangleLevel]);
    midiCCOut(CCosc1TriangleLevel, lowerData[P_osc1TriangleLevel]);
    midiCCDisplay(CCosc1TriangleLevel, lowerData[P_osc1TriangleLevel]);
    if (wholemode) {
      midiCCDCOUpper(CC_DCO1_TRI_LEVEL, lowerData[P_osc1TriangleLevel]);
    }
  }
}

FLASHMEM void updateosc2SubLevel(boolean announce) {
  if (announce) {
    showCurrentParameterPage("OSC2 Sub", int(osc2SubLevelstr));
    startParameterDisplay();
  }
  if (upperSW) {
    midiCCDCOUpper(CC_DCO2_SUB_LEVEL, upperData[P_osc2SubLevel]);
    midiCCOut(CCosc2SubLevel, upperData[P_osc2SubLevel]);
    midiCCDisplay(CCosc2SubLevel, upperData[P_osc2SubLevel]);
  } else {
    midiCCDCOLower(CC_DCO2_SUB_LEVEL, lowerData[P_osc2SubLevel]);
    midiCCOut(CCosc2SubLevel, lowerData[P_osc2SubLevel]);
    midiCCDisplay(CCosc2SubLevel, lowerData[P_osc2SubLevel]);
    if (wholemode) {
      midiCCDCOUpper(CC_DCO2_SUB_LEVEL, lowerData[P_osc2SubLevel]);
    }
  }
}

FLASHMEM void updateOsc2EnvDepth(boolean announce) {
  if (announce) {
    showCurrentParameterPage("OSC2 Pitch Env", int(osc2envDepthstr));
    startParameterDisplay();
  }
  if (upperSW) {
    midiCCDCOUpper(CC_ENV_DEPTH, upperData[P_osc2envDepth]);
    midiCCOut(CCosc2EnvDepth, upperData[P_osc2envDepth]);
    midiCCDisplay(CCosc2EnvDepth, upperData[P_osc2envDepth]);
  } else {
    midiCCDCOLower(CC_ENV_DEPTH, lowerData[P_osc2envDepth]);
    midiCCOut(CCosc2EnvDepth, lowerData[P_osc2envDepth]);
    midiCCDisplay(CCosc2EnvDepth, lowerData[P_osc2envDepth]);
    if (wholemode) {
      midiCCDCOUpper(CC_ENV_DEPTH, lowerData[P_osc2envDepth]);
    }
  }
}

FLASHMEM void updateamDepth(boolean announce) {
  if (announce) {
    showCurrentParameterPage("AM Depth", int(amDepthstr));
    startParameterDisplay();
  }
  if (upperSW) {
    midiCCVoiceUpper(VB_AMP_LFO3, upperData[P_amDepth]);
    midiCCOut(CCamDepth, upperData[P_amDepth]);
    midiCCDisplay(CCamDepth, upperData[P_amDepth]);
  } else {
    midiCCVoiceLower(VB_AMP_LFO3, lowerData[P_amDepth]);
    midiCCOut(CCamDepth, lowerData[P_amDepth]);
    midiCCDisplay(CCamDepth, lowerData[P_amDepth]);
    if (wholemode) {
      midiCCVoiceUpper(VB_AMP_LFO3, lowerData[P_amDepth]);
    }
  }
}

FLASHMEM void updateFilterCutoff(boolean announce) {
  if (announce) {
    showCurrentParameterPage("Cutoff", String(filterCutoffstr) + " Hz");
    startParameterDisplay();
  }
  if (upperSW) {
    midiCCVoiceUpper(VB_FILTER_CUTOFF, upperData[P_filterCutoff]);
    midiCCOut(CCfilterCutoff, upperData[P_filterCutoff]);
    midiCCDisplay(CCfilterCutoff, upperData[P_filterCutoff]);
  } else {
    midiCCVoiceLower(VB_FILTER_CUTOFF, lowerData[P_filterCutoff]);
    midiCCOut(CCfilterCutoff, lowerData[P_filterCutoff]);
    midiCCDisplay(CCfilterCutoff, lowerData[P_filterCutoff]);
    if (wholemode) {
      midiCCVoiceUpper(VB_FILTER_CUTOFF, lowerData[P_filterCutoff]);
    }
  }
}

FLASHMEM void updatefilterLFO(boolean announce) {
  if (announce) {
    showCurrentParameterPage("TM depth", int(filterLFOstr));
    startParameterDisplay();
  }
  if (upperSW) {
    midiCCVoiceUpper(VB_FILTER_LFO3, upperData[P_filterLFO]);
    midiCCOut(CCfilterLFO, upperData[P_filterLFO]);
    midiCCDisplay(CCfilterLFO, upperData[P_filterLFO]);
  } else {
    midiCCVoiceLower(VB_FILTER_LFO3, lowerData[P_filterLFO]);
    midiCCOut(CCfilterLFO, lowerData[P_filterLFO]);
    midiCCDisplay(CCfilterLFO, lowerData[P_filterLFO]);
    if (wholemode) {
      midiCCVoiceUpper(VB_FILTER_LFO3, lowerData[P_filterLFO]);
    }
  }
}

FLASHMEM void updatefilterRes(boolean announce) {
  if (announce) {
    showCurrentParameterPage("Resonance", int(filterResstr));
    startParameterDisplay();
  }
  if (upperSW) {
    midiCCVoiceUpper(VB_FILTER_RES, upperData[P_filterRes]);
    midiCCOut(CCfilterRes, upperData[P_filterRes]);
    midiCCDisplay(CCfilterRes, upperData[P_filterRes]);
  } else {
    midiCCVoiceLower(VB_FILTER_RES, lowerData[P_filterRes]);
    midiCCOut(CCfilterRes, lowerData[P_filterRes]);
    midiCCDisplay(CCfilterRes, lowerData[P_filterRes]);
    if (wholemode) {
      midiCCVoiceUpper(VB_FILTER_RES, lowerData[P_filterRes]);
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
        midiCCDisplaySW(CCfilterType, 0);
        midiCCOut(CCfilterType, 0);
        midiCCVoiceUpper(VB_FILTER_A, 0);
        midiCCVoiceUpper(VB_FILTER_B, 0);
        midiCCVoiceUpper(VB_FILTER_C, 0);
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
        midiCCDisplaySW(CCfilterType, 1);
        midiCCOut(CCfilterType, 1);
        midiCCVoiceUpper(VB_FILTER_A, 127);
        midiCCVoiceUpper(VB_FILTER_B, 0);
        midiCCVoiceUpper(VB_FILTER_C, 0);
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
        midiCCDisplaySW(CCfilterType, 2);
        midiCCOut(CCfilterType, 2);
        midiCCVoiceUpper(VB_FILTER_A, 0);
        midiCCVoiceUpper(VB_FILTER_B, 127);
        midiCCVoiceUpper(VB_FILTER_C, 0);
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
        midiCCDisplaySW(CCfilterType, 3);
        midiCCOut(CCfilterType, 3);
        midiCCVoiceUpper(VB_FILTER_A, 127);
        midiCCVoiceUpper(VB_FILTER_B, 127);
        midiCCVoiceUpper(VB_FILTER_C, 0);
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
        midiCCDisplaySW(CCfilterType, 4);
        midiCCOut(CCfilterType, 4);
        midiCCVoiceUpper(VB_FILTER_A, 0);
        midiCCVoiceUpper(VB_FILTER_B, 0);
        midiCCVoiceUpper(VB_FILTER_C, 127);
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
        midiCCDisplaySW(CCfilterType, 5);
        midiCCOut(CCfilterType, 5);
        midiCCVoiceUpper(VB_FILTER_A, 127);
        midiCCVoiceUpper(VB_FILTER_B, 0);
        midiCCVoiceUpper(VB_FILTER_C, 127);
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
        midiCCDisplaySW(CCfilterType, 6);
        midiCCOut(CCfilterType, 6);
        midiCCVoiceUpper(VB_FILTER_A, 0);
        midiCCVoiceUpper(VB_FILTER_B, 127);
        midiCCVoiceUpper(VB_FILTER_C, 127);
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
        midiCCDisplaySW(CCfilterType, 7);
        midiCCOut(CCfilterType, 7);
        midiCCVoiceUpper(VB_FILTER_A, 127);
        midiCCVoiceUpper(VB_FILTER_B, 127);
        midiCCVoiceUpper(VB_FILTER_C, 127);
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
        midiCCDisplaySW(CCfilterType, 0);
        midiCCOut(CCfilterType, 0);
        midiCCVoiceLower(VB_FILTER_A, 0);
        midiCCVoiceLower(VB_FILTER_B, 0);
        midiCCVoiceLower(VB_FILTER_C, 0);
        if (wholemode) {
          midiCCVoiceUpper(VB_FILTER_A, 0);
          midiCCVoiceUpper(VB_FILTER_B, 0);
          midiCCVoiceUpper(VB_FILTER_C, 0);
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
        midiCCDisplaySW(CCfilterType, 1);
        midiCCOut(CCfilterType, 1);
        midiCCVoiceLower(VB_FILTER_A, 127);
        midiCCVoiceLower(VB_FILTER_B, 0);
        midiCCVoiceLower(VB_FILTER_C, 0);
        if (wholemode) {
          midiCCVoiceUpper(VB_FILTER_A, 127);
          midiCCVoiceUpper(VB_FILTER_B, 0);
          midiCCVoiceUpper(VB_FILTER_C, 0);
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
        midiCCDisplaySW(CCfilterType, 2);
        midiCCOut(CCfilterType, 2);
        midiCCVoiceLower(VB_FILTER_A, 0);
        midiCCVoiceLower(VB_FILTER_B, 127);
        midiCCVoiceLower(VB_FILTER_C, 0);
        if (wholemode) {
          midiCCVoiceUpper(VB_FILTER_A, 0);
          midiCCVoiceUpper(VB_FILTER_B, 127);
          midiCCVoiceUpper(VB_FILTER_C, 0);
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
        midiCCDisplaySW(CCfilterType, 3);
        midiCCOut(CCfilterType, 3);
        midiCCVoiceLower(VB_FILTER_A, 127);
        midiCCVoiceLower(VB_FILTER_B, 127);
        midiCCVoiceLower(VB_FILTER_C, 0);
        if (wholemode) {
          midiCCVoiceUpper(VB_FILTER_A, 127);
          midiCCVoiceUpper(VB_FILTER_B, 127);
          midiCCVoiceUpper(VB_FILTER_C, 0);
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
        midiCCDisplaySW(CCfilterType, 4);
        midiCCOut(CCfilterType, 4);
        midiCCVoiceLower(VB_FILTER_A, 0);
        midiCCVoiceLower(VB_FILTER_B, 0);
        midiCCVoiceLower(VB_FILTER_C, 127);
        if (wholemode) {
          midiCCVoiceUpper(VB_FILTER_A, 0);
          midiCCVoiceUpper(VB_FILTER_B, 0);
          midiCCVoiceUpper(VB_FILTER_C, 127);
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
        midiCCDisplaySW(CCfilterType, 5);
        midiCCOut(CCfilterType, 5);
        midiCCVoiceLower(VB_FILTER_A, 127);
        midiCCVoiceLower(VB_FILTER_B, 0);
        midiCCVoiceLower(VB_FILTER_C, 127);
        if (wholemode) {
          midiCCVoiceUpper(VB_FILTER_A, 127);
          midiCCVoiceUpper(VB_FILTER_B, 0);
          midiCCVoiceUpper(VB_FILTER_C, 127);
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
        midiCCDisplaySW(CCfilterType, 6);
        midiCCOut(CCfilterType, 6);
        midiCCVoiceLower(VB_FILTER_A, 0);
        midiCCVoiceLower(VB_FILTER_B, 127);
        midiCCVoiceLower(VB_FILTER_C, 127);
        if (wholemode) {
          midiCCVoiceUpper(VB_FILTER_A, 0);
          midiCCVoiceUpper(VB_FILTER_B, 127);
          midiCCVoiceUpper(VB_FILTER_C, 127);
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
        midiCCDisplaySW(CCfilterType, 7);
        midiCCOut(CCfilterType, 7);
        midiCCVoiceLower(VB_FILTER_A, 127);
        midiCCVoiceLower(VB_FILTER_B, 127);
        midiCCVoiceLower(VB_FILTER_C, 127);
        if (wholemode) {
          midiCCVoiceUpper(VB_FILTER_A, 127);
          midiCCVoiceUpper(VB_FILTER_B, 127);
          midiCCVoiceUpper(VB_FILTER_C, 127);
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
    midiCCVoiceUpper(VB_EG_DEPTH, upperData[P_filterEGlevel]);
    midiCCOut(CCfilterEGlevel, upperData[P_filterEGlevel]);
    midiCCDisplay(CCfilterEGlevel, upperData[P_filterEGlevel]);
  } else {
    midiCCVoiceLower(VB_EG_DEPTH, lowerData[P_filterEGlevel]);
    midiCCOut(CCfilterEGlevel, lowerData[P_filterEGlevel]);
    midiCCDisplay(CCfilterEGlevel, lowerData[P_filterEGlevel]);
    if (wholemode) {
      midiCCVoiceUpper(VB_EG_DEPTH, lowerData[P_filterEGlevel]);
    }
  }
}

FLASHMEM void updatekeytrack(boolean announce) {
  if (announce) {
    showCurrentParameterPage("Keytrack", int(keytrackstr));
    startParameterDisplay();
  }
  if (upperSW) {
    midiCCDCOUpper(CC_KEYTRACK_DEPTH, upperData[P_keytrack]);
    midiCCOut(CCkeyTrack, upperData[P_keytrack]);
    midiCCDisplay(CCkeyTrack, upperData[P_keytrack]);
  } else {
    midiCCDCOLower(CC_KEYTRACK_DEPTH, lowerData[P_keytrack]);
    midiCCOut(CCkeyTrack, lowerData[P_keytrack]);
    midiCCDisplay(CCkeyTrack, lowerData[P_keytrack]);
    if (wholemode) {
      midiCCDCOUpper(CC_KEYTRACK_DEPTH, lowerData[P_keytrack]);
    }
  }
}

FLASHMEM void updatearpRate(boolean announce) {

  arpHzTarget = LFOTEMPO[constrain(lowerData[P_arpRate], 0, 127)];
  if (announce) {
    showCurrentParameterPage("Arp Rate", String(arpRatestr) + " Hz");
    startParameterDisplay();
  }
  midiCCOut(CCarpRate, lowerData[P_arpRate]);
  midiCCDisplay(CCarpRate, lowerData[P_arpRate]);
}

FLASHMEM void updateLFO1Rate(boolean announce) {

  if (announce) {
    showCurrentParameterPage("LFO1 Rate", String(LFO1Ratestr) + " Hz");
    startParameterDisplay();
  }
  if (upperSW) {
    midiCCDCOUpper(CC_LFO1_RATE, upperData[P_LFO1Rate]);
    midiCCOut(CCLFO1Rate, upperData[P_LFO1Rate]);
    midiCCDisplay(CCLFO1Rate, upperData[P_LFO1Rate]);
  } else {
    midiCCDCOLower(CC_LFO1_RATE, lowerData[P_LFO1Rate]);
    midiCCOut(CCLFO1Rate, lowerData[P_LFO1Rate]);
    midiCCDisplay(CCLFO1Rate, lowerData[P_LFO1Rate]);
    if (wholemode) {
      midiCCDCOUpper(CC_LFO1_RATE, lowerData[P_LFO1Rate]);
    }
  }
}

FLASHMEM void updateLFO1Delay(boolean announce) {
  if (announce) {
    showCurrentParameterPage("LFO1 Delay", String(LFO1Delaystr));
    startParameterDisplay();
  }
  if (upperSW) {
    midiCCDCOUpper(CC_LFO1_DELAY_TIME, upperData[P_LFO1Delay]);
    midiCCOut(CCLFO1Delay, upperData[P_LFO1Delay]);
    midiCCDisplay(CCLFO1Delay, upperData[P_LFO1Delay]);
  } else {
    midiCCDCOLower(CC_LFO1_DELAY_TIME, lowerData[P_LFO1Delay]);
    midiCCOut(CCLFO1Delay, lowerData[P_LFO1Delay]);
    midiCCDisplay(CCLFO1Delay, lowerData[P_LFO1Delay]);
    if (wholemode) {
      midiCCDCOUpper(CC_LFO1_DELAY_TIME, lowerData[P_LFO1Delay]);
    }
  }
}

FLASHMEM void updateLFO1Slope(boolean announce) {
  if (announce) {
    showCurrentParameterPage("LFO1 Slope", String(LFO1Slopestr));
    startParameterDisplay();
  }
  if (upperSW) {
    midiCCDCOUpper(CC_LFO1_DELAY_RAMP, upperData[P_LFO1Slope]);
    midiCCOut(CCLFO1Slope, upperData[P_LFO1Slope]);
    midiCCDisplay(CCLFO1Slope, upperData[P_LFO1Slope]);
  } else {
    midiCCDCOLower(CC_LFO1_DELAY_RAMP, lowerData[P_LFO1Slope]);
    midiCCOut(CCLFO1Slope, lowerData[P_LFO1Slope]);
    midiCCDisplay(CCLFO1Slope, lowerData[P_LFO1Slope]);
    if (wholemode) {
      midiCCDCOUpper(CC_LFO1_DELAY_RAMP, lowerData[P_LFO1Slope]);
    }
  }
}

FLASHMEM void updateLFO3Rate(boolean announce) {

  if (announce) {
    showCurrentParameterPage("LFO3 Rate", String(LFO3Ratestr) + " Hz");
    startParameterDisplay();
  }
  if (upperSW) {
    midiCCVoiceUpper(VB_LFO3_RATE, upperData[P_LFO3Rate]);
    midiCCOut(CCLFO3Rate, upperData[P_LFO3Rate]);
    midiCCDisplay(CCLFO3Rate, upperData[P_LFO3Rate]);
  } else {
    midiCCVoiceLower(VB_LFO3_RATE, lowerData[P_LFO3Rate]);
    midiCCOut(CCLFO3Rate, lowerData[P_LFO3Rate]);
    midiCCDisplay(CCLFO3Rate, lowerData[P_LFO3Rate]);
    if (wholemode) {
      midiCCVoiceUpper(VB_LFO3_RATE, lowerData[P_LFO3Rate]);
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
    midiCCDisplay(CCLFO3Delay, upperData[P_LFO3Delay]);
  } else {
    midiCCOut(CCLFO3Delay, lowerData[P_LFO3Delay]);
    midiCCDisplay(CCLFO3Delay, lowerData[P_LFO3Delay]);
  }
}

FLASHMEM void updatemodWheelDepth(boolean announce) {
  if (announce) {
    showCurrentParameterPage("Mod Wheel Depth", String(modWheelDepthstr));
    startParameterDisplay();
  }
  if (upperSW) {
    midiCCDCOUpper(CC_MW_FM_DEPTH, upperData[P_modWheelDepth]);
    midiCCOut(CCmodWheelDepth, upperData[P_modWheelDepth]);
    midiCCDisplay(CCmodWheelDepth, upperData[P_modWheelDepth]);
  } else {
    midiCCDCOLower(CC_MW_FM_DEPTH, lowerData[P_modWheelDepth]);
    midiCCOut(CCmodWheelDepth, lowerData[P_modWheelDepth]);
    midiCCDisplay(CCmodWheelDepth, lowerData[P_modWheelDepth]);
    if (wholemode) {
      midiCCDCOUpper(CC_MW_FM_DEPTH, lowerData[P_modWheelDepth]);
    }
  }
}

FLASHMEM void updatePitchBendDepth(boolean announce) {
  if (announce) {
    showCurrentParameterPage("Pitch Bend Depth", String(PitchBendLevelstr));
    startParameterDisplay();
  }
  if (upperSW) {
    midiCCDCOUpper(CC_PITCHBEND_RANGE, upperData[P_PitchBendLevel]);
    midiCCDisplay(CCPitchBend, int(upperData[P_PitchBendLevel] * 10.58));
  } else {
    midiCCDCOLower(CC_PITCHBEND_RANGE, lowerData[P_PitchBendLevel]);
    midiCCDisplay(CCPitchBend, int(lowerData[P_PitchBendLevel] * 10.58));
    if (wholemode) {
      midiCCDCOUpper(CC_PITCHBEND_RANGE, lowerData[P_PitchBendLevel]);
    }
  }
}

FLASHMEM void updateeffectPot1(boolean announce) {
  if (announce) {
    showCurrentParameterPage("Effect Pot 1", String(effectPot1str));
    startParameterDisplay();
  }
  if (upperSW) {
    midiCCVoiceUpper(VB_EFFECT_POT1, upperData[P_effectPot1]);
    midiCCOut(CCeffectPot1, upperData[P_effectPot1]);
    midiCCDisplay(CCeffectPot1, upperData[P_effectPot1]);
  } else {
    midiCCVoiceLower(VB_EFFECT_POT1, lowerData[P_effectPot1]);
    midiCCOut(CCeffectPot1, lowerData[P_effectPot1]);
    midiCCDisplay(CCeffectPot1, lowerData[P_effectPot1]);
    if (wholemode) {
      midiCCVoiceUpper(VB_EFFECT_POT1, lowerData[P_effectPot1]);
    }
  }
}

FLASHMEM void updateeffectPot2(boolean announce) {
  if (announce) {
    showCurrentParameterPage("Effect Pot 2", String(effectPot2str));
    startParameterDisplay();
  }
  if (upperSW) {
    midiCCVoiceUpper(VB_EFFECT_POT2, upperData[P_effectPot2]);
    midiCCOut(CCeffectPot2, upperData[P_effectPot2]);
    midiCCDisplay(CCeffectPot2, upperData[P_effectPot2]);
  } else {
    midiCCVoiceLower(VB_EFFECT_POT2, lowerData[P_effectPot2]);
    midiCCOut(CCeffectPot2, lowerData[P_effectPot2]);
    midiCCDisplay(CCeffectPot2, lowerData[P_effectPot2]);
    if (wholemode) {
      midiCCVoiceUpper(VB_EFFECT_POT2, lowerData[P_effectPot2]);
    }
  }
}

FLASHMEM void updateeffectPot3(boolean announce) {
  if (announce) {
    showCurrentParameterPage("Effect Pot 3", String(effectPot3str));
    startParameterDisplay();
  }
  if (upperSW) {
    midiCCVoiceUpper(VB_EFFECT_POT3, upperData[P_effectPot3]);
    midiCCOut(CCeffectPot3, upperData[P_effectPot3]);
    midiCCDisplay(CCeffectPot3, upperData[P_effectPot3]);
  } else {
    midiCCVoiceLower(VB_EFFECT_POT3, lowerData[P_effectPot3]);
    midiCCOut(CCeffectPot3, lowerData[P_effectPot3]);
    midiCCDisplay(CCeffectPot3, lowerData[P_effectPot3]);
    if (wholemode) {
      midiCCVoiceUpper(VB_EFFECT_POT3, lowerData[P_effectPot3]);
    }
  }
}

FLASHMEM void updatevcfATDepth(boolean announce) {
  if (announce) {
    showCurrentParameterPage("VCF AT Depth", String(vcfATDepthstr));
    startParameterDisplay();
  }
  if (upperSW) {
    midiCCDCOUpper(CC_AT_VCF_DEPTH, upperData[P_vcfATDepth]);
    midiCCOut(CCvcfATDepth, upperData[P_vcfATDepth]);
    midiCCDisplay(CCvcfATDepth, upperData[P_vcfATDepth]);
  } else {
    midiCCDCOLower(CC_AT_VCF_DEPTH, lowerData[P_vcfATDepth]);
    midiCCOut(CCvcfATDepth, lowerData[P_vcfATDepth]);
    midiCCDisplay(CCvcfATDepth, lowerData[P_vcfATDepth]);
    if (wholemode) {
      midiCCDCOUpper(CC_AT_VCF_DEPTH, lowerData[P_vcfATDepth]);
    }
  }
}

FLASHMEM void updateeffectsMix(boolean announce) {
  if (announce) {
    showCurrentParameterPage("Effects Mix", String(effectsMixstr));
    startParameterDisplay();
  }
  if (upperSW) {
    midiCCVoiceUpper(VB_EFFECT_MIX, upperData[P_effectsMix]);
    midiCCOut(CCeffectsMix, upperData[P_effectsMix]);
    midiCCDisplay(CCeffectsMix, upperData[P_effectsMix]);
  } else {
    midiCCVoiceLower(VB_EFFECT_MIX, lowerData[P_effectsMix]);
    midiCCOut(CCeffectsMix, lowerData[P_effectsMix]);
    midiCCDisplay(CCeffectsMix, lowerData[P_effectsMix]);
    if (wholemode) {
      midiCCVoiceUpper(VB_EFFECT_MIX, lowerData[P_effectsMix]);
    }
  }
}

FLASHMEM void updateLFO1Waveform(boolean announce) {

  if (upperSW) {
    panelData[P_LFO1Waveform] = map(upperData[P_LFO1Waveform], 0, 127, 0, 2);
  } else {
    panelData[P_LFO1Waveform] = map(lowerData[P_LFO1Waveform], 0, 127, 0, 2);
  }

  switch (panelData[P_LFO1Waveform]) {
    case 0:
      StratusLFOWaveform = "Triangle";
      midiCCDisplaySW(CCLFO1Waveform, 0);
      mcp13.digitalWrite(LFO1_WAVE_LED_RED, HIGH);
      mcp13.digitalWrite(LFO1_WAVE_LED_GREEN, LOW);
      break;

    case 1:
      StratusLFOWaveform = "Square";
      midiCCDisplaySW(CCLFO1Waveform, 1);
      mcp13.digitalWrite(LFO1_WAVE_LED_RED, HIGH);
      mcp13.digitalWrite(LFO1_WAVE_LED_GREEN, HIGH);
      break;

    case 2:
      StratusLFOWaveform = "Sawtooth";
      midiCCDisplaySW(CCLFO1Waveform, 2);
      mcp13.digitalWrite(LFO1_WAVE_LED_RED, LOW);
      mcp13.digitalWrite(LFO1_WAVE_LED_GREEN, HIGH);
      break;
  }

  if (announce) {
    showCurrentParameterPage("LFO1 Waveform", StratusLFOWaveform);
    startParameterDisplay();
  }
  if (upperSW) {
    midiCCDCOUpper(CC_LFO1_WAVEFORM, upperData[P_LFO1Waveform]);
  } else {
    midiCCDCOLower(CC_LFO1_WAVEFORM, lowerData[P_LFO1Waveform]);
    if (wholemode) {
      midiCCDCOUpper(CC_LFO1_WAVEFORM, lowerData[P_LFO1Waveform]);
    }
  }
}

FLASHMEM void updateLFO2Waveform(boolean announce) {

  if (upperSW) {
    panelData[P_LFO2Waveform] = map(upperData[P_LFO2Waveform], 0, 127, 0, 2);
  } else {
    panelData[P_LFO2Waveform] = map(lowerData[P_LFO2Waveform], 0, 127, 0, 2);
  }

  switch (panelData[P_LFO2Waveform]) {
    case 0:
      midiCCDisplaySW(CCLFO2Waveform, 0);
      mcp13.digitalWrite(LFO2_WAVE_LED_RED, HIGH);
      mcp13.digitalWrite(LFO2_WAVE_LED_GREEN, LOW);
      break;

    case 1:
      midiCCDisplaySW(CCLFO2Waveform, 1);
      mcp13.digitalWrite(LFO2_WAVE_LED_RED, HIGH);
      mcp13.digitalWrite(LFO2_WAVE_LED_GREEN, HIGH);
      break;

    case 2:
      midiCCDisplaySW(CCLFO2Waveform, 2);
      mcp13.digitalWrite(LFO2_WAVE_LED_RED, LOW);
      mcp13.digitalWrite(LFO2_WAVE_LED_GREEN, HIGH);
      break;
  }

  if (announce) {
    showCurrentParameterPage("LFO2 Waveform", StratusLFOWaveform);
    startParameterDisplay();
  }
  if (upperSW) {
    midiCCDCOUpper(CC_LFO2_WAVEFORM, upperData[P_LFO2Waveform]);
  } else {
    midiCCDCOLower(CC_LFO2_WAVEFORM, lowerData[P_LFO2Waveform]);
    if (wholemode) {
      midiCCDCOUpper(CC_LFO2_WAVEFORM, lowerData[P_LFO2Waveform]);
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
      break;

    case 1:
      StratusLFOWaveform = "Sawtooth Down";
      LFOWaveCV = 20;
      panelData[P_lfoAlt] = 127;
      break;

    case 2:
      StratusLFOWaveform = "Squarewave";
      LFOWaveCV = 35;
      panelData[P_lfoAlt] = 127;
      break;

    case 3:
      StratusLFOWaveform = "Triangle";
      LFOWaveCV = 50;
      panelData[P_lfoAlt] = 127;
      break;

    case 4:
      StratusLFOWaveform = "Sinewave";
      LFOWaveCV = 74;
      panelData[P_lfoAlt] = 127;
      break;

    case 5:
      StratusLFOWaveform = "Sweeps";
      LFOWaveCV = 90;
      panelData[P_lfoAlt] = 127;
      break;

    case 6:
      StratusLFOWaveform = "Lumps";
      LFOWaveCV = 107;
      panelData[P_lfoAlt] = 127;
      break;

    case 7:
      StratusLFOWaveform = "Sample & Hold";
      LFOWaveCV = 122;
      panelData[P_lfoAlt] = 127;
      break;

    case 8:
      StratusLFOWaveform = "Saw +Oct";
      LFOWaveCV = 1;
      panelData[P_lfoAlt] = 0;
      break;

    case 9:
      StratusLFOWaveform = "Quad Saw";
      LFOWaveCV = 20;
      panelData[P_lfoAlt] = 0;
      break;

    case 10:
      StratusLFOWaveform = "Quad Pulse";
      LFOWaveCV = 35;
      panelData[P_lfoAlt] = 0;
      break;

    case 11:
      StratusLFOWaveform = "Tri Step";
      LFOWaveCV = 50;
      panelData[P_lfoAlt] = 0;
      break;

    case 12:
      StratusLFOWaveform = "Sine +Oct";
      LFOWaveCV = 74;
      panelData[P_lfoAlt] = 0;
      break;

    case 13:
      StratusLFOWaveform = "Sine +3rd";
      LFOWaveCV = 90;
      panelData[P_lfoAlt] = 0;
      break;

    case 14:
      StratusLFOWaveform = "Sine +4th";
      LFOWaveCV = 107;
      panelData[P_lfoAlt] = 0;
      break;

    case 15:
      StratusLFOWaveform = "Rand Slopes";
      LFOWaveCV = 122;
      panelData[P_lfoAlt] = 0;
      break;
  }


  if (announce) {
    showCurrentParameterPage("LFO3 Wave", StratusLFOWaveform);
    startParameterDisplay();
  }
  if (upperSW) {
    midiCCVoiceUpper(VB_LFO3_ALT, panelData[P_lfoAlt]);
    midiCCVoiceUpper(VB_LFO3_WAVE, LFOWaveCV);
    midiCCDisplay(CCLFO3Waveform, (int(upperData[P_LFO3Waveform] * 8.47)));
    delay(5);
    midiCCDisplaySW(CCLFO3Waveform, panelData[P_LFO3Waveform]);
  } else {
    midiCCVoiceLower(VB_LFO3_ALT, panelData[P_lfoAlt]);
    midiCCVoiceLower(VB_LFO3_WAVE, LFOWaveCV);
    midiCCDisplay(CCLFO3Waveform, (int(lowerData[P_LFO3Waveform] * 8.47)));
    delay(5);
    midiCCDisplaySW(CCLFO3Waveform, panelData[P_LFO3Waveform]);
    if (wholemode) {
      midiCCVoiceUpper(VB_LFO3_ALT, panelData[P_lfoAlt]);
      midiCCVoiceUpper(VB_LFO3_WAVE, LFOWaveCV);
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
    midiCCDCOUpper(CC_ENV_ATTACK, upperData[P_pitchAttack]);
    midiCCOut(CCpitchAttack, upperData[P_pitchAttack]);
    midiCCDisplay(CCpitchAttack, upperData[P_pitchAttack]);
  } else {
    midiCCDCOLower(CC_ENV_ATTACK, lowerData[P_pitchAttack]);
    midiCCOut(CCpitchAttack, lowerData[P_pitchAttack]);
    midiCCDisplay(CCpitchAttack, lowerData[P_pitchAttack]);
    if (wholemode) {
      midiCCDCOUpper(CC_ENV_ATTACK, lowerData[P_pitchAttack]);
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
    midiCCDCOUpper(CC_ENV_DECAY, upperData[P_pitchDecay]);
    midiCCOut(CCpitchDecay, upperData[P_pitchDecay]);
    midiCCDisplay(CCpitchDecay, upperData[P_pitchDecay]);
  } else {
    midiCCDCOLower(CC_ENV_DECAY, lowerData[P_pitchDecay]);
    midiCCOut(CCpitchDecay, lowerData[P_pitchDecay]);
    midiCCDisplay(CCpitchDecay, lowerData[P_pitchDecay]);
    if (wholemode) {
      midiCCDCOUpper(CC_ENV_DECAY, lowerData[P_pitchDecay]);
    }
  }
}

FLASHMEM void updatepitchSustain(boolean announce) {
  if (announce) {
    showCurrentParameterPage("Pitch Sustain", String(pitchSustainstr), FILTER_ENV);
    startParameterDisplay();
  }
  if (upperSW) {
    midiCCDCOUpper(CC_ENV_SUSTAIN, upperData[P_pitchSustain]);
    midiCCOut(CCpitchSustain, upperData[P_pitchSustain]);
    midiCCDisplay(CCpitchSustain, upperData[P_pitchSustain]);
  } else {
    midiCCDCOLower(CC_ENV_SUSTAIN, lowerData[P_pitchSustain]);
    midiCCOut(CCpitchSustain, lowerData[P_pitchSustain]);
    midiCCDisplay(CCpitchSustain, lowerData[P_pitchSustain]);
    if (wholemode) {
      midiCCDCOUpper(CC_ENV_SUSTAIN, lowerData[P_pitchSustain]);
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
    midiCCDCOUpper(CC_ENV_RELEASE, upperData[P_pitchRelease]);
    midiCCOut(CCpitchRelease, upperData[P_pitchRelease]);
    midiCCDisplay(CCpitchRelease, upperData[P_pitchRelease]);
  } else {
    midiCCDCOLower(CC_ENV_RELEASE, lowerData[P_pitchRelease]);
    midiCCOut(CCpitchRelease, lowerData[P_pitchRelease]);
    midiCCDisplay(CCpitchRelease, lowerData[P_pitchRelease]);
    if (wholemode) {
      midiCCDCOUpper(CC_ENV_RELEASE, lowerData[P_pitchRelease]);
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
    midiCCVoiceUpper(VB_VCF_ATTACK, upperData[P_filterAttack]);
    midiCCOut(CCfilterAttack, upperData[P_filterAttack]);
    midiCCDisplay(CCfilterAttack, upperData[P_filterAttack]);
  } else {
    midiCCVoiceLower(VB_VCF_ATTACK, lowerData[P_filterAttack]);
    midiCCOut(CCfilterAttack, lowerData[P_filterAttack]);
    midiCCDisplay(CCfilterAttack, lowerData[P_filterAttack]);
    if (wholemode) {
      midiCCVoiceUpper(VB_VCF_ATTACK, lowerData[P_filterAttack]);
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
    midiCCVoiceUpper(VB_VCF_DECAY, upperData[P_filterDecay]);
    midiCCOut(CCfilterDecay, upperData[P_filterDecay]);
    midiCCDisplay(CCfilterDecay, upperData[P_filterDecay]);
  } else {
    midiCCVoiceLower(VB_VCF_DECAY, lowerData[P_filterDecay]);
    midiCCOut(CCfilterDecay, lowerData[P_filterDecay]);
    midiCCDisplay(CCfilterDecay, lowerData[P_filterDecay]);
    if (wholemode) {
      midiCCVoiceUpper(VB_VCF_DECAY, lowerData[P_filterDecay]);
    }
  }
}

FLASHMEM void updatefilterSustain(boolean announce) {
  if (announce) {
    showCurrentParameterPage("VCF Sustain", String(filterSustainstr), FILTER_ENV);
    startParameterDisplay();
  }
  if (upperSW) {
    midiCCVoiceUpper(VB_VCF_SUSTAIN, upperData[P_filterSustain]);
    midiCCOut(CCfilterSustain, upperData[P_filterSustain]);
    midiCCDisplay(CCfilterSustain, upperData[P_filterSustain]);
  } else {
    midiCCVoiceLower(VB_VCF_SUSTAIN, lowerData[P_filterSustain]);
    midiCCOut(CCfilterSustain, lowerData[P_filterSustain]);
    midiCCDisplay(CCfilterSustain, lowerData[P_filterSustain]);
    if (wholemode) {
      midiCCVoiceUpper(VB_VCF_SUSTAIN, lowerData[P_filterSustain]);
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
    midiCCVoiceUpper(VB_VCF_RELEASE, upperData[P_filterRelease]);
    midiCCOut(CCfilterRelease, upperData[P_filterRelease]);
    midiCCDisplay(CCfilterRelease, upperData[P_filterRelease]);
  } else {
    midiCCVoiceLower(VB_VCF_RELEASE, lowerData[P_filterRelease]);
    midiCCOut(CCfilterRelease, lowerData[P_filterRelease]);
    midiCCDisplay(CCfilterRelease, lowerData[P_filterRelease]);
    if (wholemode) {
      midiCCVoiceUpper(VB_VCF_RELEASE, lowerData[P_filterRelease]);
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
    midiCCVoiceUpper(VB_VCA_ATTACK, upperData[P_ampAttack]);
    midiCCOut(CCampAttack, upperData[P_ampAttack]);
    midiCCDisplay(CCampAttack, upperData[P_ampAttack]);
    upperData[P_oldampAttack] = upperData[P_ampAttack];
  } else {
    midiCCVoiceLower(VB_VCA_ATTACK, lowerData[P_ampAttack]);
    midiCCOut(CCampAttack, lowerData[P_ampAttack]);
    midiCCDisplay(CCampAttack, lowerData[P_ampAttack]);
    lowerData[P_oldampAttack] = lowerData[P_ampAttack];
    if (wholemode) {
      midiCCVoiceUpper(VB_VCA_ATTACK, lowerData[P_ampAttack]);
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
    midiCCVoiceUpper(VB_VCA_DECAY, upperData[P_ampDecay]);
    midiCCOut(CCampDecay, upperData[P_ampDecay]);
    midiCCDisplay(CCampDecay, upperData[P_ampDecay]);
    upperData[P_oldampDecay] = upperData[P_ampDecay];
  } else {
    midiCCVoiceLower(VB_VCA_DECAY, lowerData[P_ampDecay]);
    midiCCOut(CCampDecay, lowerData[P_ampDecay]);
    midiCCDisplay(CCampDecay, lowerData[P_ampDecay]);
    lowerData[P_oldampDecay] = lowerData[P_ampDecay];
    if (wholemode) {
      midiCCVoiceUpper(VB_VCA_DECAY, lowerData[P_ampDecay]);
    }
  }
}

FLASHMEM void updateampSustain(boolean announce) {
  if (announce) {
    showCurrentParameterPage("VCA Sustain", String(ampSustainstr), AMP_ENV);
    startParameterDisplay();
  }
  if (upperSW) {
    midiCCVoiceUpper(VB_VCA_SUSTAIN, upperData[P_ampSustain]);
    midiCCOut(CCampSustain, upperData[P_ampSustain]);
    midiCCDisplay(CCampSustain, upperData[P_ampSustain]);
    upperData[P_oldampSustain] = upperData[P_ampSustain];
  } else {
    midiCCVoiceLower(VB_VCA_SUSTAIN, lowerData[P_ampSustain]);
    midiCCOut(CCampSustain, lowerData[P_ampSustain]);
    midiCCDisplay(CCampSustain, lowerData[P_ampSustain]);
    lowerData[P_oldampSustain] = lowerData[P_ampSustain];
    if (wholemode) {
      midiCCVoiceUpper(VB_VCA_SUSTAIN, lowerData[P_ampSustain]);
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
    midiCCVoiceUpper(VB_VCA_RELEASE, upperData[P_ampRelease]);
    midiCCOut(CCampRelease, upperData[P_ampRelease]);
    midiCCDisplay(CCampRelease, upperData[P_ampRelease]);
    upperData[P_oldampRelease] = upperData[P_ampRelease];
  } else {
    midiCCVoiceLower(VB_VCA_RELEASE, lowerData[P_ampRelease]);
    midiCCOut(CCampRelease, lowerData[P_ampRelease]);
    midiCCDisplay(CCampRelease, lowerData[P_ampRelease]);
    lowerData[P_oldampRelease] = lowerData[P_ampRelease];
    if (wholemode) {
      midiCCVoiceUpper(VB_VCA_RELEASE, lowerData[P_ampRelease]);
    }
  }
}

FLASHMEM void updatevolumeControl(boolean announce) {
  if (announce) {
    showCurrentParameterPage("Volume", int(volumeControlstr));
    startParameterDisplay();
  }
  if (upperSW) {
    midiCCVoiceUpper(VB_VOLUME, upperData[P_volumeControl]);
    midiCCOut(CCvolumeControl, upperData[P_volumeControl]);
    midiCCDisplay(CCvolumeControl, upperData[P_volumeControl]);
  } else {
    midiCCVoiceLower(VB_VOLUME, lowerData[P_volumeControl]);
    midiCCOut(CCvolumeControl, lowerData[P_volumeControl]);
    midiCCDisplay(CCvolumeControl, lowerData[P_volumeControl]);
    if (wholemode) {
      midiCCVoiceUpper(VB_VOLUME, lowerData[P_volumeControl]);
    }
  }
}

FLASHMEM void updatefilterLevel1(boolean announce) {
  if (announce) {
    showCurrentParameterPage("Filter Level 1", int(filterLevel1str));
    startParameterDisplay();
  }
  if (upperSW) {
    midiCCVoiceUpper(VB_FILTER1_LEVEL, upperData[P_filterLevel1]);
    midiCCOut(CCfilterLevel1, upperData[P_filterLevel1]);
    midiCCDisplay(CCfilterLevel1, upperData[P_filterLevel1]);
  } else {
    midiCCVoiceLower(VB_FILTER1_LEVEL, lowerData[P_filterLevel1]);
    midiCCOut(CCfilterLevel1, lowerData[P_filterLevel1]);
    midiCCDisplay(CCfilterLevel1, lowerData[P_filterLevel1]);
    if (wholemode) {
      midiCCVoiceUpper(VB_FILTER1_LEVEL, lowerData[P_filterLevel1]);
    }
  }
}

FLASHMEM void updatefilterLevel2(boolean announce) {
  if (announce) {
    showCurrentParameterPage("Filter Level 2", int(filterLevel2str));
    startParameterDisplay();
  }
  if (upperSW) {
    midiCCVoiceUpper(VB_FILTER2_LEVEL, upperData[P_filterLevel2]);
    midiCCOut(CCfilterLevel2, upperData[P_filterLevel2]);
    midiCCDisplay(CCfilterLevel2, upperData[P_filterLevel2]);
  } else {
    midiCCVoiceLower(VB_FILTER2_LEVEL, lowerData[P_filterLevel2]);
    midiCCOut(CCfilterLevel2, lowerData[P_filterLevel2]);
    midiCCDisplay(CCfilterLevel2, lowerData[P_filterLevel2]);
    if (wholemode) {
      midiCCVoiceUpper(VB_FILTER2_LEVEL, lowerData[P_filterLevel2]);
    }
  }
}

FLASHMEM void updateosc1sawDetune(boolean announce) {
  if (announce) {
    showCurrentParameterPage("Osc1 Saw Detune", int(osc1sawDetunestr));
    startParameterDisplay();
  }
  if (upperSW) {
    midiCCDCOUpper(CC_DCO1_SAW_DETUNE, upperData[P_osc1sawDetune]);
    midiCCOut(CCosc1sawDetune, upperData[P_osc1sawDetune]);
    midiCCDisplay(CCosc1sawDetune, upperData[P_osc1sawDetune]);
  } else {
    midiCCDCOLower(CC_DCO1_SAW_DETUNE, lowerData[P_osc1sawDetune]);
    midiCCOut(CCosc1sawDetune, lowerData[P_osc1sawDetune]);
    midiCCDisplay(CCosc1sawDetune, lowerData[P_osc1sawDetune]);
    if (wholemode) {
      midiCCDCOUpper(CC_DCO1_SAW_DETUNE, lowerData[P_osc1sawDetune]);
    }
  }
}

FLASHMEM void updateosc1sawCount(boolean announce) {
  if (announce) {
    showCurrentParameterPage("Osc1 Saw Count", int(osc1sawCountstr));
    startParameterDisplay();
  }
  if (upperSW) {
    midiCCDCOUpper(CC_DCO1_SAW_COUNT, upperData[P_osc1sawCount]);
    midiCCOut(CCosc1sawCount, upperData[P_osc1sawCount]);
    midiCCDisplay(CCosc1sawCount, upperData[P_osc1sawCount]);
  } else {
    midiCCDCOLower(CC_DCO1_SAW_COUNT, lowerData[P_osc1sawCount]);
    midiCCOut(CCosc1sawCount, lowerData[P_osc1sawCount]);
    midiCCDisplay(CCosc1sawCount, lowerData[P_osc1sawCount]);
    if (wholemode) {
      midiCCDCOUpper(CC_DCO1_SAW_COUNT, lowerData[P_osc1sawCount]);
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
      midiCCDisplaySW(CCchordHoldSW, 0);
      onHoldButtonReleased();
    } else {
      if (announce) {
        showCurrentParameterPage("Chord Hold", "On");
      }
      midiCCOut(CCchordHoldSW, 127);
      midiCCDisplaySW(CCchordHoldSW, 127);
      onHoldButtonPressed();
    }
  } else {
    if (chordHoldL == 0) {
      if (announce) {
        showCurrentParameterPage("Chord Hold", "Off");
      }
      midiCCOut(CCchordHoldSW, 0);
      midiCCDisplaySW(CCchordHoldSW, 0);
      onHoldButtonReleased();
    } else {
      if (announce) {
        showCurrentParameterPage("Chord Hold", "On");
      }
      midiCCOut(CCchordHoldSW, 127);
      midiCCDisplaySW(CCchordHoldSW, 127);
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
    midiCCDisplaySW(CCplayMode, 0);
    midiCCOut(CCplayMode, 0);
    midiCCVoiceLower(LFO3_SYNC, 127);
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
    midiCCDisplaySW(CCplayMode, 1);
    delay(1);
    midiCCOut(CCplayMode, 1);
    midiCCVoiceLower(LFO3_SYNC, 0);
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
    midiCCDisplaySW(CCplayMode, 2);
    midiCCOut(CCplayMode, 2);
    midiCCVoiceLower(LFO3_SYNC, 0);
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
      midiCCDCOUpper(CC_UNISON_MODE, 0);
      mcp3.digitalWrite(POLY1_LED, HIGH);
      mcp3.digitalWrite(POLY2_LED, LOW);
      mcp3.digitalWrite(UNISON_LED, LOW);
      mcp3.digitalWrite(MONO_LED, LOW);
      midiCCDisplaySW(CCkeyboardMode, 0);
      midiCCOut(CCkeyboardMode, 0);

    } else if (upperData[P_keyboardMode] == 1) {
      if (announce) {
        showCurrentParameterPage("Keyboard Mode", "Poly 2");
        startParameterDisplay();
      }
      midiCCDCOUpper(CC_UNISON_MODE, 0);      
      mcp3.digitalWrite(POLY1_LED, LOW);
      mcp3.digitalWrite(POLY2_LED, HIGH);
      mcp3.digitalWrite(UNISON_LED, LOW);
      mcp3.digitalWrite(MONO_LED, LOW);
      midiCCDisplaySW(CCkeyboardMode, 1);
      midiCCOut(CCkeyboardMode, 1);
    } else if (upperData[P_keyboardMode] == 2) {
      if (announce) {
        showCurrentParameterPage("Keyboard Mode", "Mono");
        startParameterDisplay();
      }
      midiCCDisplaySW(CCkeyboardMode, 3);
      midiCCOut(CCkeyboardMode, 3);
      midiCCDCOUpper(CC_UNISON_MODE, 0);
      mcp3.digitalWrite(POLY1_LED, LOW);
      mcp3.digitalWrite(POLY2_LED, LOW);
      mcp3.digitalWrite(UNISON_LED, LOW);
      mcp3.digitalWrite(MONO_LED, HIGH);
    } else if (upperData[P_keyboardMode] == 3) {
      if (announce) {
        showCurrentParameterPage("Keyboard Mode", "Unison");
        startParameterDisplay();
      }
      midiCCDCOUpper(CC_UNISON_MODE, 127);
      midiCCDisplaySW(CCkeyboardMode, 2);
      midiCCOut(CCkeyboardMode, 2);
      mcp3.digitalWrite(POLY1_LED, LOW);
      mcp3.digitalWrite(POLY2_LED, LOW);
      mcp3.digitalWrite(UNISON_LED, HIGH);
      mcp3.digitalWrite(MONO_LED, LOW);
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
      midiCCDCOLower(CC_UNISON_MODE, 0);
      if (wholemode) {
        midiCCDCOUpper(CC_UNISON_MODE, 0);
      }
      midiCCDisplaySW(CCkeyboardMode, 0);
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
      midiCCDCOLower(CC_UNISON_MODE, 0);
      if (wholemode) {
        midiCCDCOUpper(CC_UNISON_MODE, 0);
      }
      mcp3.digitalWrite(POLY1_LED, LOW);
      mcp3.digitalWrite(POLY2_LED, HIGH);
      mcp3.digitalWrite(UNISON_LED, LOW);
      mcp3.digitalWrite(MONO_LED, LOW);
      midiCCDisplaySW(CCkeyboardMode, 1);
      midiCCOut(CCkeyboardMode, 1);
    } else if (lowerData[P_keyboardMode] == 2) {
      if (announce) {
        showCurrentParameterPage("Keyboard Mode", "Mono");
        startParameterDisplay();
      }
      midiCCDCOLower(CC_UNISON_MODE, 0);
      if (wholemode) {
        midiCCDCOUpper(CC_UNISON_MODE, 0);
      }      
      mcp3.digitalWrite(POLY1_LED, LOW);
      mcp3.digitalWrite(POLY2_LED, LOW);
      mcp3.digitalWrite(UNISON_LED, LOW);
      mcp3.digitalWrite(MONO_LED, HIGH);
      midiCCDisplaySW(CCkeyboardMode, 3);
      midiCCOut(CCkeyboardMode, 3);
    } else if (lowerData[P_keyboardMode] == 3) {
      if (announce) {
        showCurrentParameterPage("Keyboard Mode", "Unison");
        startParameterDisplay();
      }
      midiCCDCOLower(CC_UNISON_MODE, 127);
      if (wholemode) {
        midiCCDCOUpper(CC_UNISON_MODE, 127);
      }      
      mcp3.digitalWrite(POLY1_LED, LOW);
      mcp3.digitalWrite(POLY2_LED, LOW);
      mcp3.digitalWrite(UNISON_LED, HIGH);
      mcp3.digitalWrite(MONO_LED, LOW);
      midiCCDisplaySW(CCkeyboardMode, 2);
      midiCCOut(CCkeyboardMode, 2);
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
      midiCCDCOUpper(CC_FV1_EFFECT_0, 0);
      midiCCDCOUpper(CC_FV1_EFFECT_1, 0);
      midiCCDCOUpper(CC_FV1_EFFECT_2, 0);
      midiCCDisplaySW(CCeffectNumSW, 0);
      midiCCOut(CCeffectNumSW, 0);

    } else if (upperData[P_effectNum] == 1) {
      if (announce) {
        showCurrentParameterPage("Effect", "2");
        startParameterDisplay();
      }
      midiCCDCOUpper(CC_FV1_EFFECT_0, 127);
      midiCCDCOUpper(CC_FV1_EFFECT_1, 0);
      midiCCDCOUpper(CC_FV1_EFFECT_2, 0);
      midiCCDisplaySW(CCeffectNumSW, 1);
      midiCCOut(CCeffectNumSW, 1);

    } else if (upperData[P_effectNum] == 2) {
      if (announce) {
        showCurrentParameterPage("Effect", "3");
        startParameterDisplay();
      }
      midiCCDCOUpper(CC_FV1_EFFECT_0, 0);
      midiCCDCOUpper(CC_FV1_EFFECT_1, 127);
      midiCCDCOUpper(CC_FV1_EFFECT_2, 0);
      midiCCDisplaySW(CCeffectNumSW, 2);
      midiCCOut(CCeffectNumSW, 2);

    } else if (upperData[P_effectNum] == 3) {
      if (announce) {
        showCurrentParameterPage("Effect", "4");
        startParameterDisplay();
      }
      midiCCDCOUpper(CC_FV1_EFFECT_0, 127);
      midiCCDCOUpper(CC_FV1_EFFECT_1, 127);
      midiCCDCOUpper(CC_FV1_EFFECT_2, 0);
      midiCCDisplaySW(CCeffectNumSW, 3);
      midiCCOut(CCeffectNumSW, 3);

    } else if (upperData[P_effectNum] == 4) {
      if (announce) {
        showCurrentParameterPage("Effect", "5");
        startParameterDisplay();
      }
      midiCCDCOUpper(CC_FV1_EFFECT_0, 0);
      midiCCDCOUpper(CC_FV1_EFFECT_1, 0);
      midiCCDCOUpper(CC_FV1_EFFECT_2, 127);
      midiCCDisplaySW(CCeffectNumSW, 4);
      midiCCOut(CCeffectNumSW, 4);

    } else if (upperData[P_effectNum] == 5) {
      if (announce) {
        showCurrentParameterPage("Effect", "6");
        startParameterDisplay();
      }
      midiCCDCOUpper(CC_FV1_EFFECT_0, 127);
      midiCCDCOUpper(CC_FV1_EFFECT_1, 0);
      midiCCDCOUpper(CC_FV1_EFFECT_2, 127);
      midiCCDisplaySW(CCeffectNumSW, 5);
      midiCCOut(CCeffectNumSW, 5);

    } else if (upperData[P_effectNum] == 6) {
      if (announce) {
        showCurrentParameterPage("Effect", "7");
        startParameterDisplay();
      }
      midiCCDCOUpper(CC_FV1_EFFECT_0, 0);
      midiCCDCOUpper(CC_FV1_EFFECT_1, 127);
      midiCCDCOUpper(CC_FV1_EFFECT_2, 127);
      midiCCDisplaySW(CCeffectNumSW, 6);
      midiCCOut(CCeffectNumSW, 6);

    } else if (upperData[P_effectNum] == 7) {
      if (announce) {
        showCurrentParameterPage("Effect", "8");
        startParameterDisplay();
      }
      midiCCDCOUpper(CC_FV1_EFFECT_0, 127);
      midiCCDCOUpper(CC_FV1_EFFECT_1, 127);
      midiCCDCOUpper(CC_FV1_EFFECT_2, 127);
      midiCCDisplaySW(CCeffectNumSW, 7);
      midiCCOut(CCeffectNumSW, 7);
    }

  } else {
    if (lowerData[P_effectNum] == 0) {
      if (announce) {
        showCurrentParameterPage("Effect", "1");
        startParameterDisplay();
      }
      midiCCDCOLower(CC_FV1_EFFECT_0, 0);
      midiCCDCOLower(CC_FV1_EFFECT_1, 0);
      midiCCDCOLower(CC_FV1_EFFECT_2, 0);
      if (wholemode) {
        midiCCDCOUpper(CC_FV1_EFFECT_0, 0);
        midiCCDCOUpper(CC_FV1_EFFECT_1, 0);
        midiCCDCOUpper(CC_FV1_EFFECT_2, 0);
      }
      midiCCDisplaySW(CCeffectNumSW, 0);
      midiCCOut(CCeffectNumSW, 0);

    } else if (lowerData[P_effectNum] == 1) {
      if (announce) {
        showCurrentParameterPage("Effect", "2");
        startParameterDisplay();
      }
      midiCCDCOLower(CC_FV1_EFFECT_0, 127);
      midiCCDCOLower(CC_FV1_EFFECT_1, 0);
      midiCCDCOLower(CC_FV1_EFFECT_2, 0);
      if (wholemode) {
        midiCCDCOUpper(CC_FV1_EFFECT_0, 127);
        midiCCDCOUpper(CC_FV1_EFFECT_1, 0);
        midiCCDCOUpper(CC_FV1_EFFECT_2, 0);
      }
      midiCCDisplaySW(CCeffectNumSW, 1);
      midiCCOut(CCeffectNumSW, 1);

    } else if (lowerData[P_effectNum] == 2) {
      if (announce) {
        showCurrentParameterPage("Effect", "3");
        startParameterDisplay();
      }
      midiCCDCOLower(CC_FV1_EFFECT_0, 0);
      midiCCDCOLower(CC_FV1_EFFECT_1, 127);
      midiCCDCOLower(CC_FV1_EFFECT_2, 0);
      if (wholemode) {
        midiCCDCOUpper(CC_FV1_EFFECT_0, 0);
        midiCCDCOUpper(CC_FV1_EFFECT_1, 127);
        midiCCDCOUpper(CC_FV1_EFFECT_2, 0);
      }
      midiCCDisplaySW(CCeffectNumSW, 2);
      midiCCOut(CCeffectNumSW, 2);

    } else if (lowerData[P_effectNum] == 3) {
      if (announce) {
        showCurrentParameterPage("Effect", "4");
        startParameterDisplay();
      }
      midiCCDCOLower(CC_FV1_EFFECT_0, 127);
      midiCCDCOLower(CC_FV1_EFFECT_1, 127);
      midiCCDCOLower(CC_FV1_EFFECT_2, 0);
      if (wholemode) {
        midiCCDCOUpper(CC_FV1_EFFECT_0, 127);
        midiCCDCOUpper(CC_FV1_EFFECT_1, 127);
        midiCCDCOUpper(CC_FV1_EFFECT_2, 0);
      }
      midiCCDisplaySW(CCeffectNumSW, 3);
      midiCCOut(CCeffectNumSW, 3);

    } else if (lowerData[P_effectNum] == 4) {
      if (announce) {
        showCurrentParameterPage("Effect", "5");
        startParameterDisplay();
      }
      midiCCDCOLower(CC_FV1_EFFECT_0, 0);
      midiCCDCOLower(CC_FV1_EFFECT_1, 0);
      midiCCDCOLower(CC_FV1_EFFECT_2, 127);
      if (wholemode) {
        midiCCDCOUpper(CC_FV1_EFFECT_0, 0);
        midiCCDCOUpper(CC_FV1_EFFECT_1, 0);
        midiCCDCOUpper(CC_FV1_EFFECT_2, 127);
      }
      midiCCDisplaySW(CCeffectNumSW, 4);
      midiCCOut(CCeffectNumSW, 4);

    } else if (lowerData[P_effectNum] == 5) {
      if (announce) {
        showCurrentParameterPage("Effect", "6");
        startParameterDisplay();
      }
      midiCCDCOLower(CC_FV1_EFFECT_0, 127);
      midiCCDCOLower(CC_FV1_EFFECT_1, 0);
      midiCCDCOLower(CC_FV1_EFFECT_2, 127);
      if (wholemode) {
        midiCCDCOUpper(CC_FV1_EFFECT_0, 127);
        midiCCDCOUpper(CC_FV1_EFFECT_1, 0);
        midiCCDCOUpper(CC_FV1_EFFECT_2, 127);
      }
      midiCCDisplaySW(CCeffectNumSW, 5);
      midiCCOut(CCeffectNumSW, 5);

    } else if (lowerData[P_effectNum] == 6) {
      if (announce) {
        showCurrentParameterPage("Effect", "7");
        startParameterDisplay();
      }
      midiCCDCOLower(CC_FV1_EFFECT_0, 0);
      midiCCDCOLower(CC_FV1_EFFECT_1, 127);
      midiCCDCOLower(CC_FV1_EFFECT_2, 127);
      if (wholemode) {
        midiCCDCOUpper(CC_FV1_EFFECT_0, 0);
        midiCCDCOUpper(CC_FV1_EFFECT_1, 127);
        midiCCDCOUpper(CC_FV1_EFFECT_2, 127);
      }
      midiCCDisplaySW(CCeffectNumSW, 6);
      midiCCOut(CCeffectNumSW, 6);

    } else if (lowerData[P_effectNum] == 7) {
      if (announce) {
        showCurrentParameterPage("Effect", "8");
        startParameterDisplay();
      }
      midiCCDCOLower(CC_FV1_EFFECT_0, 127);
      midiCCDCOLower(CC_FV1_EFFECT_1, 127);
      midiCCDCOLower(CC_FV1_EFFECT_2, 127);
      if (wholemode) {
        midiCCDCOUpper(CC_FV1_EFFECT_0, 127);
        midiCCDCOUpper(CC_FV1_EFFECT_1, 127);
        midiCCDCOUpper(CC_FV1_EFFECT_2, 127);
      }
      midiCCDisplaySW(CCeffectNumSW, 7);
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
    // // Step 1: Enter external mode
    // midiCCDCOUpper(CC_FV1_INTERNAL, 127);
    // delay(10);

    // Step 2: Reset all CS lines
    midiCCDCOUpper(CC_FV1_BANK_0, 0);
    midiCCDCOUpper(CC_FV1_BANK_1, 127);
    midiCCDCOUpper(CC_FV1_BANK_2, 127);


    if (bank == 0) {
      // Internal ROM selected
      midiCCDCOUpper(CC_FV1_INTERNAL, 0);
      delay(10);
    } else {
      // Select only the chosen EEPROM
      if (bank == 1) {
        midiCCDCOUpper(CC_FV1_BANK_0, 0);
        midiCCDCOUpper(CC_FV1_BANK_1, 127);
        midiCCDCOUpper(CC_FV1_BANK_2, 127);
      } else if (bank == 2) {
        midiCCDCOUpper(CC_FV1_BANK_0, 0);
        midiCCDCOUpper(CC_FV1_BANK_1, 127);
        midiCCDCOUpper(CC_FV1_BANK_2, 127);
      } else if (bank == 3) {
        midiCCDCOUpper(CC_FV1_BANK_0, 127);
        midiCCDCOUpper(CC_FV1_BANK_1, 127);
        midiCCDCOUpper(CC_FV1_BANK_2, 0);
      }
      midiCCDCOUpper(CC_FV1_INTERNAL, 0);
      delay(10);
      midiCCDCOUpper(CC_FV1_INTERNAL, 127);
    }
  } else {
    // // Step 1: Enter external mode
    // midiCCDCOLower(CC_FV1_INTERNAL, 127);

    // Step 2: Reset all CS lines
    midiCCDCOLower(CC_FV1_BANK_0, 0);
    midiCCDCOLower(CC_FV1_BANK_1, 127);
    midiCCDCOLower(CC_FV1_BANK_2, 127);
    if (wholemode) {
      // midiCCDCOUpper(CC_FV1_INTERNAL, 127);

      // Step 2: Reset all CS lines
      midiCCDCOUpper(CC_FV1_BANK_0, 0);
      midiCCDCOUpper(CC_FV1_BANK_1, 127);
      midiCCDCOUpper(CC_FV1_BANK_2, 127);
    }

    if (bank == 0) {
      midiCCDCOLower(CC_FV1_INTERNAL, 0);
      if (wholemode) {
        midiCCDCOUpper(CC_FV1_INTERNAL, 0);
      }
    } else {
      if (bank == 1) {
        midiCCDCOLower(CC_FV1_BANK_0, 0);
        midiCCDCOLower(CC_FV1_BANK_1, 127);
        midiCCDCOLower(CC_FV1_BANK_2, 127);
        if (wholemode) {
          midiCCDCOUpper(CC_FV1_BANK_0, 0);
          midiCCDCOUpper(CC_FV1_BANK_1, 127);
          midiCCDCOUpper(CC_FV1_BANK_2, 127);
        }
      } else if (bank == 2) {
        midiCCDCOLower(CC_FV1_BANK_0, 127);
        midiCCDCOLower(CC_FV1_BANK_1, 0);
        midiCCDCOLower(CC_FV1_BANK_2, 127);
        if (wholemode) {
          midiCCDCOUpper(CC_FV1_BANK_0, 127);
          midiCCDCOUpper(CC_FV1_BANK_1, 0);
          midiCCDCOUpper(CC_FV1_BANK_2, 127);
        }
      } else if (bank == 3) {
        midiCCDCOLower(CC_FV1_BANK_0, 127);
        midiCCDCOLower(CC_FV1_BANK_1, 127);
        midiCCDCOLower(CC_FV1_BANK_2, 0);
        if (wholemode) {
          midiCCDCOUpper(CC_FV1_BANK_0, 127);
          midiCCDCOUpper(CC_FV1_BANK_1, 127);
          midiCCDCOUpper(CC_FV1_BANK_2, 0);
        }
      }
      delay(10);
      midiCCDCOLower(CC_FV1_INTERNAL, 0);
      delay(10);
      midiCCDCOLower(CC_FV1_INTERNAL, 127);
      if (wholemode) {
        delay(10);
        midiCCDCOUpper(CC_FV1_INTERNAL, 0);
        delay(10);
        midiCCDCOUpper(CC_FV1_INTERNAL, 127);
      }
    }

    // Send MIDI
    midiCCDisplaySW(CCeffectBankSW, bank);
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
      midiCCVoiceUpper(VB_MULTIPLIER_BIT0, 0);
      midiCCVoiceUpper(VB_MULTIPLIER_BIT1, 0);
      midiCCVoiceUpper(VB_MULTIPLIER_BIT2, 0);
      midiCCDisplaySW(CClfoMult, 0);
      midiCCOut(CClfoMult, 0);
      mcp13.digitalWrite(LFO3_MULT_LED_RED, LOW);
      mcp13.digitalWrite(LFO3_MULT_LED_GREEN, LOW);
    } else if (upperData[P_lfoMultiplier] == 1) {
      if (announce) {
        showCurrentParameterPage("LFO Multiplier", "x1.0");
        startParameterDisplay();
      }
      midiCCVoiceUpper(VB_MULTIPLIER_BIT0, 127);
      midiCCVoiceUpper(VB_MULTIPLIER_BIT1, 0);
      midiCCVoiceUpper(VB_MULTIPLIER_BIT2, 0);
      midiCCDisplaySW(CClfoMult, 1);
      midiCCOut(CClfoMult, 1);
      mcp13.digitalWrite(LFO3_MULT_LED_RED, HIGH);
      mcp13.digitalWrite(LFO3_MULT_LED_GREEN, LOW);
    } else if (upperData[P_lfoMultiplier] == 2) {
      if (announce) {
        showCurrentParameterPage("LFO Multiplier", "x1.5");
        startParameterDisplay();
      }
      midiCCVoiceUpper(VB_MULTIPLIER_BIT0, 0);
      midiCCVoiceUpper(VB_MULTIPLIER_BIT1, 127);
      midiCCVoiceUpper(VB_MULTIPLIER_BIT2, 0);
      midiCCDisplaySW(CClfoMult, 2);
      midiCCOut(CClfoMult, 2);
      mcp13.digitalWrite(LFO3_MULT_LED_RED, LOW);
      mcp13.digitalWrite(LFO3_MULT_LED_GREEN, HIGH);
    } else if (upperData[P_lfoMultiplier] == 3) {
      if (announce) {
        showCurrentParameterPage("LFO Multiplier", "x2.0");
        startParameterDisplay();
      }
      midiCCVoiceUpper(VB_MULTIPLIER_BIT0, 127);
      midiCCVoiceUpper(VB_MULTIPLIER_BIT1, 127);
      midiCCVoiceUpper(VB_MULTIPLIER_BIT2, 0);
      midiCCDisplaySW(CClfoMult, 3);
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
      midiCCVoiceLower(VB_MULTIPLIER_BIT0, 0);
      midiCCVoiceLower(VB_MULTIPLIER_BIT1, 0);
      midiCCVoiceLower(VB_MULTIPLIER_BIT2, 0);
      if (wholemode) {
        midiCCVoiceUpper(VB_MULTIPLIER_BIT0, 127);
        midiCCVoiceUpper(VB_MULTIPLIER_BIT1, 0);
        midiCCVoiceUpper(VB_MULTIPLIER_BIT2, 0);
      }
      midiCCDisplaySW(CClfoMult, 0);
      midiCCOut(CClfoMult, 0);
      mcp13.digitalWrite(LFO3_MULT_LED_RED, LOW);
      mcp13.digitalWrite(LFO3_MULT_LED_GREEN, LOW);
    } else if (lowerData[P_lfoMultiplier] == 1) {
      if (announce) {
        showCurrentParameterPage("LFO Multiplier", "x1.0");
        startParameterDisplay();
      }
      midiCCVoiceLower(VB_MULTIPLIER_BIT0, 127);
      midiCCVoiceLower(VB_MULTIPLIER_BIT1, 0);
      midiCCVoiceLower(VB_MULTIPLIER_BIT2, 0);
      if (wholemode) {
        midiCCVoiceUpper(VB_MULTIPLIER_BIT0, 127);
        midiCCVoiceUpper(VB_MULTIPLIER_BIT1, 0);
        midiCCVoiceUpper(VB_MULTIPLIER_BIT2, 0);
      }
      midiCCDisplaySW(CClfoMult, 1);
      midiCCOut(CClfoMult, 1);
      mcp13.digitalWrite(LFO3_MULT_LED_RED, HIGH);
      mcp13.digitalWrite(LFO3_MULT_LED_GREEN, LOW);
    } else if (lowerData[P_lfoMultiplier] == 2) {
      if (announce) {
        showCurrentParameterPage("LFO Multiplier", "x1.5");
        startParameterDisplay();
      }
      midiCCVoiceLower(VB_MULTIPLIER_BIT0, 0);
      midiCCVoiceLower(VB_MULTIPLIER_BIT1, 127);
      midiCCVoiceLower(VB_MULTIPLIER_BIT2, 0);
      if (wholemode) {
        midiCCVoiceUpper(VB_MULTIPLIER_BIT0, 127);
        midiCCVoiceUpper(VB_MULTIPLIER_BIT1, 0);
        midiCCVoiceUpper(VB_MULTIPLIER_BIT2, 0);
      }
      midiCCDisplaySW(CClfoMult, 2);
      midiCCOut(CClfoMult, 2);
      mcp13.digitalWrite(LFO3_MULT_LED_RED, LOW);
      mcp13.digitalWrite(LFO3_MULT_LED_GREEN, HIGH);
    } else if (lowerData[P_lfoMultiplier] == 3) {
      if (announce) {
        showCurrentParameterPage("LFO Multiplier", "x2.0");
        startParameterDisplay();
      }
      midiCCVoiceLower(VB_MULTIPLIER_BIT0, 127);
      midiCCVoiceLower(VB_MULTIPLIER_BIT1, 127);
      midiCCVoiceLower(VB_MULTIPLIER_BIT2, 0);
      if (wholemode) {
        midiCCVoiceUpper(VB_MULTIPLIER_BIT0, 127);
        midiCCVoiceUpper(VB_MULTIPLIER_BIT1, 0);
        midiCCVoiceUpper(VB_MULTIPLIER_BIT2, 0);
      }
      midiCCDisplaySW(CClfoMult, 3);
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
      midiCCDCOUpper(CC_PORTAMENTO_SW, 0);
      midiCCDisplaySW(CCglideSW, 0);
      mcp4.digitalWrite(GLIDE_LED_GREEN, LOW);
    } else {
      if (announce) {
        showCurrentParameterPage("Glide", "On");
        startParameterDisplay();
      }
      midiCCDCOUpper(CC_PORTAMENTO_TIME, upperData[P_glideTime]);
      midiCCDCOUpper(CC_PORTAMENTO_SW, 127);
      midiCCDisplay(CCglideTime, upperData[P_glideTime]);
      midiCCDisplaySW(CCglideSW, 1);
      mcp4.digitalWrite(GLIDE_LED_GREEN, HIGH);
    }
  } else {
    if (lowerData[P_glideSW] == 0) {
      if (announce) {
        showCurrentParameterPage("Glide", "Off");
        startParameterDisplay();
      }
      midiCCDCOLower(CC_PORTAMENTO_SW, 0);
      midiCCDisplaySW(CCglideSW, 0);
      if (wholemode) {
        midiCCDCOUpper(CC_PORTAMENTO_SW, 0);
        mcp4.digitalWrite(GLIDE_LED_GREEN, LOW);
      }
      mcp4.digitalWrite(GLIDE_LED_RED, LOW);
    } else {
      if (announce) {
        showCurrentParameterPage("Glide", "On");
        startParameterDisplay();
      }
      midiCCDCOLower(CC_PORTAMENTO_TIME, lowerData[P_glideTime]);
      midiCCDCOLower(CC_PORTAMENTO_SW, 127);

      midiCCDisplay(CCglideTime, lowerData[P_glideTime]);
      midiCCDisplaySW(CCglideSW, 1);
      mcp4.digitalWrite(GLIDE_LED_RED, HIGH);
      if (wholemode) {
        midiCCDCOUpper(CC_PORTAMENTO_TIME, lowerData[P_glideTime]);
        midiCCDCOUpper(CC_PORTAMENTO_SW, 127);
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
      midiCCVoiceUpper(VB_FILTER_POLE, 127);
      midiCCOut(CCfilterPoleSW, 127);
      midiCCDisplaySW(CCfilterPoleSW, 1);
      mcp8.digitalWrite(VCF_POLE_LED, HIGH);
    } else {
      if (announce) {
        updateFilterType(1);
      }
      midiCCVoiceUpper(VB_FILTER_POLE, 0);
      midiCCOut(CCfilterPoleSW, 0);
      midiCCDisplaySW(CCfilterPoleSW, 0);
      mcp8.digitalWrite(VCF_POLE_LED, LOW);
    }
  } else {
    if (lowerData[P_filterPoleSW] == 1) {
      if (announce) {
        updateFilterType(1);
      }
      midiCCVoiceLower(VB_FILTER_POLE, 127);
      if (wholemode) {
        midiCCVoiceUpper(VB_FILTER_POLE, 127);
      }
      midiCCOut(CCfilterPoleSW, 127);
      midiCCDisplaySW(CCfilterPoleSW, 1);
      mcp8.digitalWrite(VCF_POLE_LED, HIGH);
    } else {
      if (announce) {
        updateFilterType(1);
      }
      midiCCVoiceLower(VB_FILTER_POLE, 0);
      if (wholemode) {
        midiCCVoiceUpper(VB_FILTER_POLE, 0);
      }
      midiCCOut(CCfilterPoleSW, 0);
      midiCCDisplaySW(CCfilterPoleSW, 0);
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
        midiCCVoiceUpper(VB_FILTER_LOOP_BIT0, 0);
        midiCCVoiceUpper(VB_FILTER_LOOP_BIT1, 0);
        midiCCDisplaySW(CCFilterLoop, 0);
        midiCCOut(CCFilterLoop, 0);
        mcp9.digitalWrite(VCF_LOOP_LED_RED, LOW);
        mcp10.digitalWrite(VCF_LOOP_LED_GREEN, LOW);
        break;

      case 1:
        if (announce) {
          showCurrentParameterPage("VCF LFO Loop", "Gated");
          startParameterDisplay();
        }
        midiCCVoiceUpper(VB_FILTER_LOOP_BIT0, 127);
        midiCCVoiceUpper(VB_FILTER_LOOP_BIT1, 0);
        midiCCDisplaySW(CCFilterLoop, 1);
        midiCCOut(CCFilterLoop, 63);
        mcp9.digitalWrite(VCF_LOOP_LED_RED, HIGH);
        mcp10.digitalWrite(VCF_LOOP_LED_GREEN, LOW);
        break;

      case 2:
        if (announce) {
          showCurrentParameterPage("VCF Looping", "LFO");
          startParameterDisplay();
        }
        midiCCVoiceUpper(VB_FILTER_LOOP_BIT0, 0);
        midiCCVoiceUpper(VB_FILTER_LOOP_BIT1, 127);
        midiCCDisplaySW(CCFilterLoop, 2);
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
        midiCCVoiceLower(VB_FILTER_LOOP_BIT0, 0);
        midiCCVoiceLower(VB_FILTER_LOOP_BIT1, 0);
        if (wholemode) {
          midiCCVoiceUpper(VB_FILTER_LOOP_BIT0, 0);
          midiCCVoiceUpper(VB_FILTER_LOOP_BIT1, 0);
        }
        midiCCDisplaySW(CCFilterLoop, 0);
        midiCCOut(CCFilterLoop, 0);
        mcp9.digitalWrite(VCF_LOOP_LED_RED, LOW);
        mcp10.digitalWrite(VCF_LOOP_LED_GREEN, LOW);
        break;

      case 1:
        if (announce) {
          showCurrentParameterPage("VCF LFO Loop", "Gated");
          startParameterDisplay();
        }
        midiCCVoiceLower(VB_FILTER_LOOP_BIT0, 127);
        midiCCVoiceLower(VB_FILTER_LOOP_BIT1, 0);
        if (wholemode) {
          midiCCVoiceUpper(VB_FILTER_LOOP_BIT0, 127);
          midiCCVoiceUpper(VB_FILTER_LOOP_BIT1, 0);
        }
        midiCCDisplaySW(CCFilterLoop, 1);
        midiCCOut(CCFilterLoop, 63);
        mcp9.digitalWrite(VCF_LOOP_LED_RED, HIGH);
        mcp10.digitalWrite(VCF_LOOP_LED_GREEN, LOW);
        break;

      case 2:
        if (announce) {
          showCurrentParameterPage("VCF Looping", "LFO");
          startParameterDisplay();
        }
        midiCCVoiceLower(VB_FILTER_LOOP_BIT0, 0);
        midiCCVoiceLower(VB_FILTER_LOOP_BIT1, 127);
        if (wholemode) {
          midiCCVoiceUpper(VB_FILTER_LOOP_BIT0, 0);
          midiCCVoiceUpper(VB_FILTER_LOOP_BIT1, 127);
        }
        midiCCDisplaySW(CCFilterLoop, 2);
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
      midiCCVoiceUpper(VB_EG_INVERT, 0);
      midiCCOut(CCfilterEGinv, 0);
      midiCCDisplaySW(CCfilterEGinv, 0);
      mcp8.digitalWrite(VCF_EG_INV_LED, LOW);
    } else {
      if (announce) {
        showCurrentParameterPage("Filter Env", "Negative");
        startParameterDisplay();
      }
      midiCCVoiceUpper(VB_EG_INVERT, 127);
      midiCCOut(CCfilterEGinv, 127);
      midiCCDisplaySW(CCfilterEGinv, 127);
      mcp8.digitalWrite(VCF_EG_INV_LED, HIGH);
    }
  } else {
    if (lowerData[P_filterEGinv] == 0) {
      if (announce) {
        showCurrentParameterPage("Filter Env", "Positive");
        startParameterDisplay();
      }
      midiCCVoiceLower(VB_EG_INVERT, 0);
      if (wholemode) {
        midiCCVoiceUpper(VB_EG_INVERT, 0);
      }
      midiCCOut(CCfilterEGinv, 0);
      midiCCDisplaySW(CCfilterEGinv, 0);
      mcp8.digitalWrite(VCF_EG_INV_LED, LOW);

    } else {
      if (announce) {
        showCurrentParameterPage("Filter Env", "Negative");
        startParameterDisplay();
      }
      midiCCVoiceLower(VB_EG_INVERT, 127);
      if (wholemode) {
        midiCCVoiceUpper(VB_EG_INVERT, 127);
      }
      midiCCOut(CCfilterEGinv, 127);
      midiCCDisplaySW(CCfilterEGinv, 127);
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
      midiCCDCOUpper(CC_KEYTRACK_SW, 0);
      midiCCOut(CCkeyTrackSW, 0);
      midiCCDisplaySW(CCkeyTrackSW, 0);
      mcp8.digitalWrite(VCF_KEYTRACK_LED, LOW);
    } else {
      if (announce) {
        showCurrentParameterPage("Keytrack", "On");
        startParameterDisplay();
      }
      midiCCDCOUpper(CC_KEYTRACK_SW, 127);
      midiCCOut(CCkeyTrackSW, 127);
      midiCCDisplaySW(CCkeyTrackSW, 1);
      mcp8.digitalWrite(VCF_KEYTRACK_LED, HIGH);
    }
  } else {
    if (!lowerData[P_keytrackSW]) {
      if (announce) {
        showCurrentParameterPage("Keytrack", "Off");
        startParameterDisplay();
      }
      midiCCDCOLower(CC_KEYTRACK_SW, 0);
      midiCCOut(CCkeyTrackSW, 0);
      midiCCDisplaySW(CCkeyTrackSW, 0);
      if (wholemode) {
        midiCCDCOUpper(CC_KEYTRACK_SW, 0);
      }
      mcp8.digitalWrite(VCF_KEYTRACK_LED, LOW);
    } else {
      if (announce) {
        showCurrentParameterPage("Keytrack", "On");
        startParameterDisplay();
      }
      midiCCDCOLower(CC_KEYTRACK_SW, 127);
      midiCCOut(CCkeyTrackSW, 127);
      midiCCDisplaySW(CCkeyTrackSW, 1);
      if (wholemode) {
        midiCCDCOUpper(CC_KEYTRACK_SW, 127);
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
      midiCCDCOUpper(CC_SYNC_MODE, 0);
      midiCCOut(CCsyncSW, 0);
      midiCCDisplaySW(CCsyncSW, 0);
      mcp7.digitalWrite(DCO2_SYNC_LED_RED, LOW);
      mcp7.digitalWrite(DCO2_SYNC_LED_GREEN, LOW);
    }
    if (upperData[P_sync] == 1) {
      if (announce) {
        showCurrentParameterPage("Sync", "Soft");
        startParameterDisplay();
      }
      midiCCDCOUpper(CC_SYNC_MODE, 64);
      midiCCOut(CCsyncSW, 64);
      midiCCDisplaySW(CCsyncSW, 1);
      mcp7.digitalWrite(DCO2_SYNC_LED_RED, HIGH);
      mcp7.digitalWrite(DCO2_SYNC_LED_GREEN, LOW);
    }
    if (upperData[P_sync] == 2) {
      if (announce) {
        showCurrentParameterPage("Sync", "Hard");
        startParameterDisplay();
      }
      midiCCDCOUpper(CC_SYNC_MODE, 127);
      midiCCOut(CCsyncSW, 127);
      midiCCDisplaySW(CCsyncSW, 2);
      mcp7.digitalWrite(DCO2_SYNC_LED_RED, LOW);
      mcp7.digitalWrite(DCO2_SYNC_LED_GREEN, HIGH);
    }
  } else {
    if (lowerData[P_sync] == 0) {
      if (announce) {
        showCurrentParameterPage("Sync", "Off");
        startParameterDisplay();
      }
      midiCCDCOLower(CC_SYNC_MODE, 0);
      midiCCOut(CCsyncSW, 0);
      midiCCDisplaySW(CCsyncSW, 0);
      if (wholemode) {
        midiCCDCOUpper(CC_SYNC_MODE, 0);
      }
      mcp7.digitalWrite(DCO2_SYNC_LED_RED, LOW);
      mcp7.digitalWrite(DCO2_SYNC_LED_GREEN, LOW);
    }
    if (lowerData[P_sync] == 1) {
      if (announce) {
        showCurrentParameterPage("Sync", "Soft");
        startParameterDisplay();
      }
      midiCCDCOLower(CC_SYNC_MODE, 64);
      midiCCOut(CCsyncSW, 127);
      midiCCDisplaySW(CCsyncSW, 1);
      if (wholemode) {
        midiCCDCOUpper(CC_SYNC_MODE, 64);
      }
      mcp7.digitalWrite(DCO2_SYNC_LED_RED, HIGH);
      mcp7.digitalWrite(DCO2_SYNC_LED_GREEN, LOW);
    }
    if (lowerData[P_sync] == 2) {
      if (announce) {
        showCurrentParameterPage("Sync", "Hard");
        startParameterDisplay();
      }
      midiCCDCOLower(CC_SYNC_MODE, 127);
      midiCCOut(CCsyncSW, 127);
      midiCCDisplaySW(CCsyncSW, 2);
      if (wholemode) {
        midiCCDCOUpper(CC_SYNC_MODE, 127);
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
          midiCCVoiceUpper(VB_EFFECT_POT3, upperData[P_effectPot3]);
          midiCCDisplay(CCeffectPot3, upperData[P_effectPot3]);
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
          midiCCVoiceUpper(VB_EFFECT_POT3, upperData[P_effectPot3]);
          midiCCDisplay(CCeffectPot3, upperData[P_effectPot3]);
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
          midiCCVoiceLower(VB_EFFECT_POT3, lowerData[P_effectPot3]);
          midiCCDisplay(CCeffectPot3, lowerData[P_effectPot3]);
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
          midiCCVoiceLower(VB_EFFECT_POT3, lowerData[P_effectPot3]);
          midiCCDisplay(CCeffectPot3, lowerData[P_effectPot3]);
          lowerLastSentPot3 = lowerData[P_effectPot3];
        }
      } else {
        lowerfootPedal = false;
        lowerfast = false;
      }
    }
  }
}

FLASHMEM void updatefx_Bypass(boolean announce) {

  if (upperSW) {
    if (!upperData[P_fx_Bypass]) {
      if (announce) {
        showCurrentParameterPage("FX loop", "Off");
        startParameterDisplay();
      }
      midiCCDCOUpper(CC_FV1_INTERNAL, 0);
      midiCCOut(CCfx_Bypass, 0);
      midiCCDisplaySW(CCfx_Bypass, 0);
      mcp15.digitalWrite(FX_BYPASS_LED, LOW);
    } else {
      if (announce) {
        showCurrentParameterPage("FX Loop", "On");
        startParameterDisplay();
      }
      midiCCDCOUpper(CC_FV1_INTERNAL, 127);
      midiCCOut(CCfx_Bypass, 127);
      midiCCDisplaySW(CCfx_Bypass, 1);
      mcp15.digitalWrite(FX_BYPASS_LED, HIGH);
    }
  } else {
    if (!lowerData[P_fx_Bypass]) {
      if (announce) {
        showCurrentParameterPage("FX Loop", "Off");
        startParameterDisplay();
      }
      midiCCDCOLower(CC_FV1_INTERNAL, 0);
      if (wholemode) {
        midiCCDCOUpper(CC_FV1_INTERNAL, 0);
      }
      midiCCOut(CCfx_Bypass, 0);
      midiCCDisplaySW(CCfx_Bypass, 0);
      mcp15.digitalWrite(FX_BYPASS_LED, LOW);
    } else {
      if (announce) {
        showCurrentParameterPage("FX Loop", "On");
        startParameterDisplay();
      }
      midiCCDCOLower(CC_FV1_INTERNAL, 127);
      if (wholemode) {
        midiCCDCOUpper(CC_FV1_INTERNAL, 127);
      }
      midiCCOut(CCfx_Bypass, 127);
      midiCCDisplaySW(CCfx_Bypass, 1);
      mcp15.digitalWrite(FX_BYPASS_LED, HIGH);
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
      midiCCVoiceUpper(VB_FILTER_LIN_LOG, 0);
      midiCCOut(CCfilterenvLinLogSW, 0);
      midiCCDisplaySW(CCfilterenvLinLogSW, 0);
      mcp10.digitalWrite(VCF_LIN_LOG_LED_RED, HIGH);
      mcp10.digitalWrite(VCF_LIN_LOG_LED_GREEN, LOW);
    } else {
      if (announce) {
        showCurrentParameterPage("Filter Env", "Log");
        startParameterDisplay();
      }
      midiCCVoiceUpper(VB_FILTER_LIN_LOG, 127);
      midiCCOut(CCfilterenvLinLogSW, 127);
      midiCCDisplaySW(CCfilterenvLinLogSW, 1);
      mcp10.digitalWrite(VCF_LIN_LOG_LED_RED, LOW);
      mcp10.digitalWrite(VCF_LIN_LOG_LED_GREEN, HIGH);
    }
  } else {
    if (!lowerData[P_filterLogLin]) {
      if (announce) {
        showCurrentParameterPage("Filter Env", "Linear");
        startParameterDisplay();
      }
      midiCCVoiceLower(VB_FILTER_LIN_LOG, 0);
      if (wholemode) {
        midiCCVoiceUpper(VB_FILTER_LIN_LOG, 0);
      }
      midiCCOut(CCfilterenvLinLogSW, 0);
      midiCCDisplaySW(CCfilterenvLinLogSW, 0);
      mcp10.digitalWrite(VCF_LIN_LOG_LED_RED, HIGH);
      mcp10.digitalWrite(VCF_LIN_LOG_LED_GREEN, LOW);
    } else {
      if (announce) {
        showCurrentParameterPage("Filter Env", "Log");
        startParameterDisplay();
      }
      midiCCVoiceLower(VB_FILTER_LIN_LOG, 127);
      if (wholemode) {
        midiCCVoiceUpper(VB_FILTER_LIN_LOG, 127);
      }
      midiCCOut(CCfilterenvLinLogSW, 127);
      midiCCDisplaySW(CCfilterenvLinLogSW, 1);
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
      midiCCVoiceUpper(VB_AMP_LIN_LOG, 0);
      midiCCOut(CCampenvLinLogSW, 0);
      midiCCDisplaySW(CCampenvLinLogSW, 0);
      mcp12.digitalWrite(AMP_LIN_LOG_LED_RED, HIGH);
      mcp12.digitalWrite(AMP_LIN_LOG_LED_GREEN, LOW);
    } else {
      if (announce) {
        showCurrentParameterPage("Amp Env", "Log");
        startParameterDisplay();
      }
      midiCCVoiceUpper(VB_AMP_LIN_LOG, 127);
      midiCCOut(CCampenvLinLogSW, 127);
      midiCCDisplaySW(CCampenvLinLogSW, 1);
      mcp12.digitalWrite(AMP_LIN_LOG_LED_RED, LOW);
      mcp12.digitalWrite(AMP_LIN_LOG_LED_GREEN, HIGH);
    }
  } else {
    if (!lowerData[P_ampLogLin]) {
      if (announce) {
        showCurrentParameterPage("Amp Env", "Linear");
        startParameterDisplay();
      }
      midiCCVoiceLower(VB_AMP_LIN_LOG, 0);
      if (wholemode) {
        midiCCVoiceUpper(VB_AMP_LIN_LOG, 0);
      }
      midiCCOut(CCampenvLinLogSW, 0);
      midiCCDisplaySW(CCampenvLinLogSW, 0);
      mcp12.digitalWrite(AMP_LIN_LOG_LED_RED, HIGH);
      mcp12.digitalWrite(AMP_LIN_LOG_LED_GREEN, LOW);
    } else {
      if (announce) {
        showCurrentParameterPage("Amp Env", "Log");
        startParameterDisplay();
      }
      midiCCVoiceLower(VB_AMP_LIN_LOG, 127);
      if (wholemode) {
        midiCCVoiceUpper(VB_AMP_LIN_LOG, 127);
      }
      midiCCOut(CCampenvLinLogSW, 127);
      midiCCDisplaySW(CCampenvLinLogSW, 1);
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
      midiCCVoiceUpper(VB_NOISE_SOURCE, 0);
      midiCCOut(CCnoiseSrc, 0);
      midiCCDisplaySW(CCnoiseSrc, 0);
      mcp6.digitalWrite(NOISE_SRC_LED_RED, HIGH);
      mcp6.digitalWrite(NOISE_SRC_LED_GREEN, LOW);
    } else {
      if (announce) {
        showCurrentParameterPage("Noise Source", "Pink");
        startParameterDisplay();
      }
      midiCCVoiceUpper(VB_NOISE_SOURCE, 127);
      midiCCOut(CCnoiseSrc, 127);
      midiCCDisplaySW(CCnoiseSrc, 1);
      mcp6.digitalWrite(NOISE_SRC_LED_RED, LOW);
      mcp6.digitalWrite(NOISE_SRC_LED_GREEN, HIGH);
    }
  } else {
    if (!lowerData[P_noiseSrc]) {
      if (announce) {
        showCurrentParameterPage("Noise Source", "White");
        startParameterDisplay();
      }
      midiCCVoiceLower(VB_NOISE_SOURCE, 0);
      if (wholemode) {
        midiCCVoiceUpper(VB_NOISE_SOURCE, 0);
      }
      midiCCOut(CCnoiseSrc, 0);
      midiCCDisplaySW(CCnoiseSrc, 0);
      mcp6.digitalWrite(NOISE_SRC_LED_RED, HIGH);
      mcp6.digitalWrite(NOISE_SRC_LED_GREEN, LOW);
    } else {
      if (announce) {
        showCurrentParameterPage("Noise Source", "Pink");
        startParameterDisplay();
      }
      midiCCVoiceLower(VB_NOISE_SOURCE, 127);
      if (wholemode) {
        midiCCVoiceUpper(VB_NOISE_SOURCE, 127);
      }
      midiCCOut(CCnoiseSrc, 127);
      midiCCDisplaySW(CCnoiseSrc, 1);
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
      midiCCDCOUpper(CC_AT_FM_ENABLE, 0);
      midiCCDisplaySW(CCdco_at_SW, 0);
      midiCCOut(CCdco_at_SW, 0);
      mcp5.digitalWrite(DCO_AT_LED, LOW);
    } else {
      if (announce) {
        showCurrentParameterPage("DCO Aftertouch", "On");
        startParameterDisplay();
      }
      midiCCDCOUpper(CC_AT_FM_ENABLE, 127);
      midiCCDisplaySW(CCdco_at_SW, 1);
      midiCCOut(CCdco_at_SW, 127);
      mcp5.digitalWrite(DCO_AT_LED, HIGH);
    }
  } else {
    if (lowerData[P_dco_at_SW] == 0) {
      if (announce) {
        showCurrentParameterPage("DCO Aftertouch", "Off");
        startParameterDisplay();
      }
      midiCCDCOLower(CC_AT_FM_ENABLE, 0);
      if (wholemode) {
        midiCCDCOUpper(CC_AT_FM_ENABLE, 0);
      }
      midiCCDisplaySW(CCdco_at_SW, 0);
      midiCCOut(CCdco_at_SW, 0);
      mcp5.digitalWrite(DCO_AT_LED, LOW);
    } else {
      if (announce) {
        showCurrentParameterPage("DCO Aftertouch", "On");
        startParameterDisplay();
      }
      midiCCDCOLower(CC_AT_FM_ENABLE, 127);
      if (wholemode) {
        midiCCDCOUpper(CC_AT_FM_ENABLE, 127);
      }
      midiCCDisplaySW(CCdco_at_SW, 1);
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
      midiCCDCOUpper(CC_AT_FILTER_ENABLE, 0);
      midiCCDisplaySW(CCfilter_at_SW, 0);
      midiCCOut(CCfilter_at_SW, 0);
      mcp5.digitalWrite(FILTER_AT_LED, LOW);
    } else {
      if (announce) {
        showCurrentParameterPage("Filter Aftertouch", "On");
        startParameterDisplay();
      }
      midiCCDCOUpper(CC_AT_FILTER_ENABLE, 127);
      midiCCDisplaySW(CCfilter_at_SW, 1);
      midiCCOut(CCfilter_at_SW, 127);
      mcp5.digitalWrite(FILTER_AT_LED, HIGH);
    }
  } else {
    if (lowerData[P_filter_at_SW] == 0) {
      if (announce) {
        showCurrentParameterPage("Filter Aftertouch", "Off");
        startParameterDisplay();
      }
      midiCCDCOLower(CC_AT_FILTER_ENABLE, 0);
      if (wholemode) {
        midiCCDCOUpper(CC_AT_FILTER_ENABLE, 0);
      }
      midiCCDisplaySW(CCfilter_at_SW, 0);
      midiCCOut(CCfilter_at_SW, 0);
      mcp5.digitalWrite(FILTER_AT_LED, LOW);
    } else {
      if (announce) {
        showCurrentParameterPage("Filter Aftertouch", "On");
        startParameterDisplay();
      }
      midiCCDCOLower(CC_AT_FILTER_ENABLE, 127);
      if (wholemode) {
        midiCCDCOUpper(CC_AT_FILTER_ENABLE, 127);
      }
      midiCCDisplaySW(CCfilter_at_SW, 1);
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
      midiCCVoiceUpper(VB_FILTER_VELOCITY, 0);
      midiCCDisplaySW(CCfilterVel, 0);
      midiCCOut(CCfilterVel, 0);
      mcp9.digitalWrite(VCF_VELOCITY_LED, LOW);
    } else {
      if (announce) {
        showCurrentParameterPage("VCF Velocity", "On");
        startParameterDisplay();
      }
      midiCCVoiceUpper(VB_FILTER_VELOCITY, 127);
      midiCCDisplaySW(CCfilterVel, 1);
      midiCCOut(CCfilterVel, 127);
      mcp9.digitalWrite(VCF_VELOCITY_LED, HIGH);
    }
  } else {
    if (lowerData[P_filterVel] == 0) {
      if (announce) {
        showCurrentParameterPage("VCF Velocity", "Off");
        startParameterDisplay();
      }
      midiCCVoiceLower(VB_FILTER_VELOCITY, 0);
      if (wholemode) {
        midiCCVoiceUpper(VB_FILTER_VELOCITY, 0);
      }
      midiCCDisplaySW(CCfilterVel, 0);
      midiCCOut(CCfilterVel, 0);
      mcp9.digitalWrite(VCF_VELOCITY_LED, LOW);
    } else {
      if (announce) {
        showCurrentParameterPage("VCF Velocity", "On");
        startParameterDisplay();
      }
      midiCCVoiceLower(VB_FILTER_VELOCITY, 127);
      if (wholemode) {
        midiCCVoiceUpper(VB_FILTER_VELOCITY, 127);
      }
      midiCCDisplaySW(CCfilterVel, 1);
      midiCCOut(CCfilterVel, 127);
      mcp9.digitalWrite(VCF_VELOCITY_LED, HIGH);
    }
  }
}

FLASHMEM void updateenv2_punch(boolean announce) {
  if (upperSW) {
    if (upperData[P_env2_punch] == 0) {
      if (announce) {
        showCurrentParameterPage("VCF Env Punch", "Off");
        startParameterDisplay();
      }
      midiCCVoiceUpper(VB_VCF_ENV_PUNCH, 127);
      midiCCDisplaySW(CCenv2_punch, 0);
      midiCCOut(CCenv2_punch, 0);
      mcp15.digitalWrite(VCF_PUNCH_LED, LOW);
    } else {
      if (announce) {
        showCurrentParameterPage("VCF Env Punch", "On");
        startParameterDisplay();
      }
      midiCCVoiceUpper(VB_VCF_ENV_PUNCH, 0);
      midiCCDisplaySW(CCenv2_punch, 1);
      midiCCOut(CCenv2_punch, 127);
      mcp15.digitalWrite(VCF_PUNCH_LED, HIGH);
    }
  } else {
    if (lowerData[P_env2_punch] == 0) {
      if (announce) {
        showCurrentParameterPage("VCF Env Punch", "Off");
        startParameterDisplay();
      }
      midiCCVoiceLower(VB_VCF_ENV_PUNCH, 127);
      if (wholemode) {
        midiCCVoiceUpper(VB_VCF_ENV_PUNCH, 127);
      }
      midiCCDisplaySW(CCenv2_punch, 0);
      midiCCOut(CCenv2_punch, 0);
      mcp15.digitalWrite(VCF_PUNCH_LED, LOW);
    } else {
      if (announce) {
        showCurrentParameterPage("VCF Env Punch", "On");
        startParameterDisplay();
      }
      midiCCVoiceLower(VB_VCF_ENV_PUNCH, 0);
      if (wholemode) {
        midiCCVoiceUpper(VB_VCF_ENV_PUNCH, 0);
      }
      midiCCDisplaySW(CCenv2_punch, 1);
      midiCCOut(CCenv2_punch, 127);
      mcp15.digitalWrite(VCF_PUNCH_LED, HIGH);
    }
  }
}

FLASHMEM void updateenv3_punch(boolean announce) {
  if (upperSW) {
    if (upperData[P_env3_punch] == 0) {
      if (announce) {
        showCurrentParameterPage("VCA Env Punch", "Off");
        startParameterDisplay();
      }
      midiCCVoiceUpper(VB_AMP_ENV_PUNCH, 127);
      midiCCDisplaySW(CCenv3_punch, 0);
      midiCCOut(CCenv3_punch, 0);
      mcp14.digitalWrite(VCA_ENV_PUNCH_LED, LOW);
    } else {
      if (announce) {
        showCurrentParameterPage("VCA Env Punch", "On");
        startParameterDisplay();
      }
      midiCCVoiceUpper(VB_AMP_ENV_PUNCH, 0);
      midiCCDisplaySW(CCenv3_punch, 1);
      midiCCOut(CCenv3_punch, 127);
      mcp14.digitalWrite(VCA_ENV_PUNCH_LED, HIGH);
    }
  } else {
    if (lowerData[P_env3_punch] == 0) {
      if (announce) {
        showCurrentParameterPage("VCA Env Punch", "Off");
        startParameterDisplay();
      }
      midiCCVoiceLower(VB_AMP_ENV_PUNCH, 127);
      if (wholemode) {
        midiCCVoiceUpper(VB_AMP_ENV_PUNCH, 127);
      }
      midiCCDisplaySW(CCenv3_punch, 0);
      midiCCOut(CCenv3_punch, 0);
      mcp14.digitalWrite(VCA_ENV_PUNCH_LED, LOW);
    } else {
      if (announce) {
        showCurrentParameterPage("VCA Env Punch", "On");
        startParameterDisplay();
      }
      midiCCVoiceLower(VB_AMP_ENV_PUNCH, 0);
      if (wholemode) {
        midiCCVoiceUpper(VB_AMP_ENV_PUNCH, 0);
      }
      midiCCDisplaySW(CCenv3_punch, 1);
      midiCCOut(CCenv3_punch, 127);
      mcp14.digitalWrite(VCA_ENV_PUNCH_LED, HIGH);
    }
  }
}

FLASHMEM void updateenv2_env3_adsr(boolean announce) {
  if (upperSW) {
    if (upperData[P_env2_env3_adsr] == 0) {
      if (announce) {
        showCurrentParameterPage("VCF/VCA Env Type", "ADSR");
        startParameterDisplay();
      }
      midiCCDCOUpper(CC_GATE_ENABLE, 1);
      midiCCDisplaySW(CCenv2_env3_adsr, 0);
      midiCCOut(CCenv2_env3_adsr, 0);
      mcp15.digitalWrite(ENV2_3_ADSR_LED_RED, HIGH);
      mcp15.digitalWrite(ENV2_3_ADSR_LED_GREEN, LOW);
    } else {
      if (announce) {
        showCurrentParameterPage("VCF/VCA Env Type", "ADR");
        startParameterDisplay();
      }
      midiCCDCOUpper(CC_GATE_ENABLE, 0);
      midiCCDisplaySW(CCenv2_env3_adsr, 1);
      midiCCOut(CCenv2_env3_adsr, 1);
      mcp15.digitalWrite(ENV2_3_ADSR_LED_RED, LOW);
      mcp15.digitalWrite(ENV2_3_ADSR_LED_GREEN, HIGH);
    }
  } else {
    if (lowerData[P_env2_env3_adsr] == 0) {
      if (announce) {
        showCurrentParameterPage("VCF/VCA Env Type", "ADSR");
        startParameterDisplay();
      }
      midiCCDCOLower(CC_GATE_ENABLE, 1);
      if (wholemode) {
        midiCCDCOUpper(CC_GATE_ENABLE, 1);
      }
      midiCCDisplaySW(CCenv2_env3_adsr, 0);
      midiCCOut(CCenv2_env3_adsr, 0);
      mcp15.digitalWrite(ENV2_3_ADSR_LED_RED, HIGH);
      mcp15.digitalWrite(ENV2_3_ADSR_LED_GREEN, LOW);
    } else {
      if (announce) {
        showCurrentParameterPage("VCF/VCA Env Type", "ADR");
        startParameterDisplay();
      }
      midiCCDCOLower(CC_GATE_ENABLE, 0);
      if (wholemode) {
        midiCCDCOUpper(CC_GATE_ENABLE, 0);
      }
      midiCCDisplaySW(CCenv2_env3_adsr, 1);
      midiCCOut(CCenv2_env3_adsr, 1);
      mcp15.digitalWrite(ENV2_3_ADSR_LED_RED, LOW);
      mcp15.digitalWrite(ENV2_3_ADSR_LED_GREEN, HIGH);
    }
  }
}

void updateNotePriority(boolean announce) {
  if (upperSW) {

    switch (upperData[P_NotePriority]) {
      case 0:
        if (announce) {
          showCurrentParameterPage("Note Priority", "Top");
          startParameterDisplay();
        }
        mcp4.digitalWrite(PRIORITY_LED_RED, LOW);
        mcp4.digitalWrite(PRIORITY_LED_GREEN, HIGH);

        midiCCOut(CCNotePriority, 0);
        midiCCDisplaySW(CCNotePriority, 0);
        break;

      case 1:
        if (announce) {
          showCurrentParameterPage("Note Priority", "Bottom");
          startParameterDisplay();
        }
        mcp4.digitalWrite(PRIORITY_LED_RED, HIGH);
        mcp4.digitalWrite(PRIORITY_LED_GREEN, LOW);

        midiCCOut(CCNotePriority, 63);
        midiCCDisplaySW(CCNotePriority, 1);
        break;

      case 2:
        if (announce) {
          showCurrentParameterPage("Note Priority", "Last");
          startParameterDisplay();
        }
        mcp4.digitalWrite(PRIORITY_LED_RED, HIGH);
        mcp4.digitalWrite(PRIORITY_LED_GREEN, HIGH);

        midiCCOut(CCNotePriority, 127);
                midiCCDisplaySW(CCNotePriority, 2);
        break;
    }
    if (dualmode) {
      lowerData[P_NotePriority] = upperData[P_NotePriority];
    }
  } else {

    switch (lowerData[P_NotePriority]) {
      case 0:
        if (announce) {
          showCurrentParameterPage("Note Priority", "Top");
          startParameterDisplay();
        }
        mcp4.digitalWrite(PRIORITY_LED_RED, LOW);
        mcp4.digitalWrite(PRIORITY_LED_GREEN, HIGH);
        midiCCDisplaySW(CCNotePriority, 0);
        midiCCOut(CCNotePriority, 0);
        break;

      case 1:
        if (announce) {
          showCurrentParameterPage("Note Priority", "Bottom");
          startParameterDisplay();
        }
        mcp4.digitalWrite(PRIORITY_LED_RED, HIGH);
        mcp4.digitalWrite(PRIORITY_LED_GREEN, LOW);
        midiCCDisplaySW(CCNotePriority, 1);
        midiCCOut(CCNotePriority, 63);
        break;

      case 2:
        if (announce) {
          showCurrentParameterPage("Note Priority", "Last");
          startParameterDisplay();
        }
        mcp4.digitalWrite(PRIORITY_LED_RED, HIGH);
        mcp4.digitalWrite(PRIORITY_LED_GREEN, HIGH);
        midiCCDisplaySW(CCNotePriority, 2);
        midiCCOut(CCNotePriority, 127);
        break;
    }
    if (dualmode) {
      upperData[P_NotePriority] = lowerData[P_NotePriority];
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
        midiCCVoiceUpper(VB_AMP_LOOP_BIT0, 0);
        midiCCVoiceUpper(VB_AMP_LOOP_BIT1, 0);
        midiCCDisplaySW(CCAmpLoop, 0);
        midiCCOut(CCAmpLoop, 0);
        mcp11.digitalWrite(AMP_LOOP_LED_RED, LOW);
        mcp12.digitalWrite(AMP_LOOP_LED_GREEN, LOW);
        break;

      case 1:
        if (announce) {
          showCurrentParameterPage("VCA Loop", "Gated");
          startParameterDisplay();
        }
        midiCCVoiceUpper(VB_AMP_LOOP_BIT0, 127);
        midiCCVoiceUpper(VB_AMP_LOOP_BIT1, 0);
        midiCCDisplaySW(CCAmpLoop, 1);
        midiCCOut(CCAmpLoop, 63);
        mcp11.digitalWrite(AMP_LOOP_LED_RED, HIGH);
        mcp12.digitalWrite(AMP_LOOP_LED_GREEN, LOW);
        break;

      case 2:
        if (announce) {
          showCurrentParameterPage("VCA Loop", "LFO");
          startParameterDisplay();
        }
        midiCCVoiceUpper(VB_AMP_LOOP_BIT0, 0);
        midiCCVoiceUpper(VB_AMP_LOOP_BIT1, 127);
        midiCCDisplaySW(CCAmpLoop, 2);
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
        midiCCVoiceLower(VB_AMP_LOOP_BIT0, 0);
        midiCCVoiceLower(VB_AMP_LOOP_BIT1, 0);
        if (wholemode) {
          midiCCVoiceUpper(VB_AMP_LOOP_BIT0, 0);
          midiCCVoiceUpper(VB_AMP_LOOP_BIT1, 0);
        }
        midiCCDisplaySW(CCAmpLoop, 0);
        midiCCOut(CCAmpLoop, 0);
        mcp11.digitalWrite(AMP_LOOP_LED_RED, LOW);
        mcp12.digitalWrite(AMP_LOOP_LED_GREEN, LOW);
        break;

      case 1:
        if (announce) {
          showCurrentParameterPage("VCA Loop", "Gated");
          startParameterDisplay();
        }
        midiCCVoiceLower(VB_AMP_LOOP_BIT0, 127);
        midiCCVoiceLower(VB_AMP_LOOP_BIT1, 0);
        if (wholemode) {
          midiCCVoiceUpper(VB_AMP_LOOP_BIT0, 127);
          midiCCVoiceUpper(VB_AMP_LOOP_BIT1, 0);
        }
        midiCCDisplaySW(CCAmpLoop, 1);
        midiCCOut(CCAmpLoop, 63);
        mcp11.digitalWrite(AMP_LOOP_LED_RED, HIGH);
        mcp12.digitalWrite(AMP_LOOP_LED_GREEN, LOW);
        break;

      case 2:
        if (announce) {
          showCurrentParameterPage("VCA Loop", "LFO");
          startParameterDisplay();
        }
        midiCCVoiceLower(VB_AMP_LOOP_BIT0, 0);
        midiCCVoiceLower(VB_AMP_LOOP_BIT1, 127);
        if (wholemode) {
          midiCCVoiceUpper(VB_AMP_LOOP_BIT0, 0);
          midiCCVoiceUpper(VB_AMP_LOOP_BIT1, 127);
        }
        midiCCDisplaySW(CCAmpLoop, 2);
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
      midiCCVoiceUpper(VB_AMP_VELOCITY, 0);
      midiCCDisplaySW(CCvcaVel, 0);
      midiCCOut(CCvcaVel, 0);
      mcp11.digitalWrite(AMP_VELOCITY_LED, LOW);
    } else {
      if (announce) {
        showCurrentParameterPage("VCA Velocity", "On");
        startParameterDisplay();
      }
      midiCCVoiceUpper(VB_AMP_VELOCITY, 127);
      midiCCDisplaySW(CCvcaVel, 1);
      midiCCOut(CCvcaVel, 127);
      mcp11.digitalWrite(AMP_VELOCITY_LED, HIGH);
    }
  } else {
    if (lowerData[P_vcaVel] == 0) {
      if (announce) {
        showCurrentParameterPage("VCA Velocity", "Off");
        startParameterDisplay();
      }
      midiCCVoiceLower(VB_AMP_VELOCITY, 0);
      if (wholemode) {
        midiCCVoiceUpper(VB_AMP_VELOCITY, 0);
      }
      midiCCDisplaySW(CCvcaVel, 0);
      midiCCOut(CCvcaVel, 0);
      mcp11.digitalWrite(AMP_VELOCITY_LED, LOW);
    } else {
      if (announce) {
        showCurrentParameterPage("VCA Velocity", "On");
        startParameterDisplay();
      }
      midiCCVoiceLower(VB_AMP_VELOCITY, 127);
      if (wholemode) {
        midiCCVoiceUpper(VB_AMP_VELOCITY, 127);
      }
      midiCCDisplaySW(CCvcaVel, 1);
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
      midiCCDisplaySW(CCvcaGate, 0);
      upperData[P_ampAttack] = upperData[P_oldampAttack];
      upperData[P_ampDecay] = upperData[P_oldampDecay];
      upperData[P_ampSustain] = upperData[P_oldampSustain];
      upperData[P_ampRelease] = upperData[P_oldampRelease];
      midiCCVoiceUpper(VB_VCA_ATTACK, upperData[P_ampAttack]);
      midiCCVoiceUpper(VB_VCA_DECAY, upperData[P_ampDecay]);
      midiCCVoiceUpper(VB_VCA_SUSTAIN, upperData[P_ampSustain]);
      midiCCVoiceUpper(VB_VCA_RELEASE, upperData[P_ampRelease]);
      mcp13.digitalWrite(AMP_ENV_GATE_LED, LOW);
    } else {
      if (announce) {
        showCurrentParameterPage("VCA Gate", "On");
        startParameterDisplay();
      }
      midiCCOut(CCvcaGate, 127);
      midiCCDisplaySW(CCvcaGate, 1);
      midiCCVoiceUpper(VB_VCA_ATTACK, 0);
      midiCCVoiceUpper(VB_VCA_DECAY, 0);
      midiCCVoiceUpper(VB_VCA_SUSTAIN, 127);
      midiCCVoiceUpper(VB_VCA_RELEASE, 0);
      mcp13.digitalWrite(AMP_ENV_GATE_LED, HIGH);
    }
  } else {
    if (!lowerData[P_vcaGate]) {
      if (announce) {
        showCurrentParameterPage("VCA Gate", "Off");
        startParameterDisplay();
      }
      midiCCOut(CCvcaGate, 0);
      midiCCDisplaySW(CCvcaGate, 0);
      lowerData[P_ampAttack] = lowerData[P_oldampAttack];
      lowerData[P_ampDecay] = lowerData[P_oldampDecay];
      lowerData[P_ampSustain] = lowerData[P_oldampSustain];
      lowerData[P_ampRelease] = lowerData[P_oldampRelease];
      midiCCVoiceLower(VB_VCA_ATTACK, lowerData[P_ampAttack]);
      midiCCVoiceLower(VB_VCA_DECAY, lowerData[P_ampDecay]);
      midiCCVoiceLower(VB_VCA_SUSTAIN, lowerData[P_ampSustain]);
      midiCCVoiceLower(VB_VCA_RELEASE, lowerData[P_ampRelease]);
      mcp13.digitalWrite(AMP_ENV_GATE_LED, LOW);
      if (wholemode) {
        midiCCVoiceUpper(VB_VCA_ATTACK, lowerData[P_ampAttack]);
        midiCCVoiceUpper(VB_VCA_DECAY, lowerData[P_ampDecay]);
        midiCCVoiceUpper(VB_VCA_SUSTAIN, lowerData[P_ampSustain]);
        midiCCVoiceUpper(VB_VCA_RELEASE, lowerData[P_ampRelease]);
      }
    } else {
      if (announce) {
        showCurrentParameterPage("VCA Gate", "On");
        startParameterDisplay();
      }
      midiCCOut(CCvcaGate, 127);
      midiCCDisplaySW(CCvcaGate, 1);
      midiCCVoiceLower(VB_VCA_ATTACK, 0);
      midiCCVoiceLower(VB_VCA_DECAY, 0);
      midiCCVoiceLower(VB_VCA_SUSTAIN, 127);
      midiCCVoiceLower(VB_VCA_RELEASE, 0);
      mcp13.digitalWrite(AMP_ENV_GATE_LED, HIGH);
      if (wholemode) {
        midiCCVoiceUpper(VB_VCA_ATTACK, 0);
        midiCCVoiceUpper(VB_VCA_DECAY, 0);
        midiCCVoiceUpper(VB_VCA_SUSTAIN, 127);
        midiCCVoiceUpper(VB_VCA_RELEASE, 0);
      }
    }
  }
}

FLASHMEM void updateupperSW(boolean announce) {
  if (!wholemode) {
    if (upperSW) {
      midiCCDisplaySW(CCupperSW, 1);
      mcp3.digitalWrite(UPPER_LED, HIGH);
      mcp3.digitalWrite(LOWER_LED, LOW);
      upperParamsToDisplay();
      setAllButtons();
    }
  }
}

FLASHMEM void updatelowerSW(boolean announce) {
  if (lowerSW) {
    midiCCDisplaySW(CClowerSW, 1);
    mcp3.digitalWrite(UPPER_LED, LOW);
    mcp3.digitalWrite(LOWER_LED, HIGH);
    lowerParamsToDisplay();
    setAllButtons();
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
      midiCCDisplaySW(CCmonoMulti, 0);
      mcp13.digitalWrite(LFO3_RETRIG_LED, LOW);
    } else {
      if (announce) {
        showCurrentParameterPage("LFO 3 Retrigger", "On");
        startParameterDisplay();
      }
      midiCCOut(CCmonoMulti, 127);
      midiCCDisplaySW(CCmonoMulti, 1);
      mcp13.digitalWrite(LFO3_RETRIG_LED, HIGH);
    }
  } else {
    if (!lowerData[P_monoMulti]) {
      if (announce) {
        showCurrentParameterPage("LFO 3 Retrigger", "Off");
        startParameterDisplay();
      }
      midiCCOut(CCmonoMulti, 0);
      midiCCDisplaySW(CCmonoMulti, 0);
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
      midiCCDisplaySW(CCmonoMulti, 1);
      if (wholemode) {
        upperData[P_monoMulti] = lowerData[P_monoMulti];
      }
      mcp13.digitalWrite(LFO3_RETRIG_LED, HIGH);
    }
  }
}

FLASHMEM void updatedriftSW(boolean announce) {
  if (upperSW) {
    if (!upperData[P_driftSW]) {
      if (announce) {
        showCurrentParameterPage("Analogue Drift", "Off");
        startParameterDisplay();
      }
      midiCCDCOUpper(CC_DRIFT_SW, 0);
      midiCCOut(CCdriftSW, 0);
      mcp2.digitalWrite(DRIFT_LED, LOW);
    } else {
      if (announce) {
        showCurrentParameterPage("Analogue Drift", "On");
        startParameterDisplay();
      }
      midiCCDCOUpper(CC_DRIFT_SW, 127);
      midiCCOut(CCdriftSW, 127);
      mcp2.digitalWrite(DRIFT_LED, HIGH);
    }
  } else {
    if (!lowerData[P_driftSW]) {
      if (announce) {
        showCurrentParameterPage("Analogue Drift", "Off");
        startParameterDisplay();
      }
      midiCCDCOLower(CC_DRIFT_SW, 0);
      midiCCOut(CCdriftSW, 0);
      if (wholemode) {
        midiCCDCOUpper(CC_DRIFT_SW, 0);
      }
      mcp2.digitalWrite(DRIFT_LED, LOW);
    } else {
      if (announce) {
        showCurrentParameterPage("Analogue Drift", "On");
        startParameterDisplay();
      }
      midiCCDCOLower(CC_DRIFT_SW, 127);
      midiCCOut(CCdriftSW, 127);
      if (wholemode) {
        midiCCDCOUpper(CC_DRIFT_SW, 127);
      }
      mcp2.digitalWrite(DRIFT_LED, HIGH);
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
      midiCCDCOUpper(CC_LFO1_RETRIG, 0);
      midiCCOut(CClfo1retrig, 0);
      midiCCDisplaySW(CClfo1retrig, 0);
      mcp14.digitalWrite(LFO1_RETRIG_LED, LOW);
    } else {
      if (announce) {
        showCurrentParameterPage("LFO 1 Retrigger", "On");
        startParameterDisplay();
      }
      midiCCDCOUpper(CC_LFO1_RETRIG, 127);
      midiCCOut(CClfo1retrig, 127);
      midiCCDisplaySW(CClfo1retrig, 1);
      mcp14.digitalWrite(LFO1_RETRIG_LED, HIGH);
    }
  } else {
    if (!lowerData[P_lfo1retrig]) {
      if (announce) {
        showCurrentParameterPage("LFO 1 Retrigger", "Off");
        startParameterDisplay();
      }
      midiCCDCOLower(CC_LFO1_RETRIG, 0);
      midiCCOut(CClfo1retrig, 0);
      midiCCDisplaySW(CClfo1retrig, 0);
      if (wholemode) {
        midiCCDCOUpper(CC_LFO1_RETRIG, 0);
      }
      mcp14.digitalWrite(LFO1_RETRIG_LED, LOW);
    } else {
      if (announce) {
        showCurrentParameterPage("LFO 1 Retrigger", "On");
        startParameterDisplay();
      }
      midiCCDCOLower(CC_LFO1_RETRIG, 127);
      midiCCOut(CClfo1retrig, 127);
      midiCCDisplaySW(CClfo1retrig, 1);
      if (wholemode) {
        midiCCDCOUpper(CC_LFO1_RETRIG, 127);
      }
      mcp14.digitalWrite(LFO1_RETRIG_LED, HIGH);
    }
  }
}

void startParameterDisplay() {
  // Defer the (blocking) TFT redraw to loop() so turning controls doesn't
  // stall the main loop and stutter the arpeggiator.
  lastDisplayTriggerTime = millis();
  waitingToUpdate = true;
  paramDisplayDirty = true;
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
      }
      glideTimestr = LINEAR[value];
      updateglideTime(1);
      break;

    case CCLFO2Rate:
      if (upperSW) {
        upperData[P_LFO2Rate] = value;
      } else {
        lowerData[P_LFO2Rate] = value;
      }
      LFO2Ratestr = LFOTEMPO[value];  // for display
      updateLFO2Rate(1);
      break;

    case CCfmDepth:
      if (upperSW) {
        upperData[P_fmDepth] = value;
      } else {
        lowerData[P_fmDepth] = value;
      }
      fmDepthstr = value;
      updatefmDepth(1);
      break;

    case CCosc2PW:
      if (upperSW) {
        upperData[P_osc2PW] = value;
      } else {
        lowerData[P_osc2PW] = value;
      }
      osc2PWstr = PULSEWIDTH[value];
      updateosc2PW(1);
      break;

    case CCosc2PWM:
      if (upperSW) {
        upperData[P_osc2PWM] = value;
      } else {
        lowerData[P_osc2PWM] = value;
      }
      osc2PWMstr = value;
      updateosc2PWM(1);
      break;

    case CCosc1PW:
      if (upperSW) {
        upperData[P_osc1PW] = value;
      } else {
        lowerData[P_osc1PW] = value;
      }
      osc1PWstr = PULSEWIDTH[value];
      updateosc1PW(1);
      break;

    case CCosc1PWM:
      if (upperSW) {
        upperData[P_osc1PWM] = value;
      } else {
        lowerData[P_osc1PWM] = value;
      }
      osc1PWMstr = value;
      updateosc1PWM(1);
      break;

    case CCosc1envPWM:
      if (upperSW) {
        upperData[P_osc1envPWM] = value;
      } else {
        lowerData[P_osc1envPWM] = value;
      }
      osc1PWMstr = value;
      updateosc1envPWM(1);
      break;

    case CCosc2envPWM:
      if (upperSW) {
        upperData[P_osc2envPWM] = value;
      } else {
        lowerData[P_osc2envPWM] = value;
      }
      osc2PWMstr = value;
      updateosc2envPWM(1);
      break;

    case CCosc1Oct:
      if (upperSW) {
        upperData[P_osc1Range] = value;
      } else {
        lowerData[P_osc1Range] = value;
      }
      updateosc1Range(1);
      break;

    case CCosc2Oct:
      if (upperSW) {
        upperData[P_osc2Range] = value;
      } else {
        lowerData[P_osc2Range] = value;
      }
      updateosc2Range(1);
      break;

    case CCosc2Detune:
      if (upperSW) {
        upperData[P_osc2Detune] = value;
      } else {
        lowerData[P_osc2Detune] = value;
      }
      updateosc2Detune(1);
      break;

    case CCdualDetune:
      if (upperSW) {
        upperData[P_dualDetune] = value;
      }
      updatedualDetune(1);
      break;

    case CCunisonDetune:
      if (upperSW) {
        upperData[P_unisonDetune] = value;       
      } else {
        lowerData[P_unisonDetune] = value;
      }
      unisonDetunestr = value;
      updateunisonDetune(1);
      break;

    case CCdriftDepth:
      if (upperSW) {
        upperData[P_driftDepth] = value;       
      } else {
        lowerData[P_driftDepth] = value;
      }
      driftDepthstr = value;
      updatedriftDepth(1);
      break;

    case CCosc2Interval:
      if (upperSW) {
        upperData[P_osc2Interval] = value;
      } else {
        lowerData[P_osc2Interval] = value;
      }
      osc2Intervalstr = value;
      updateosc2Interval(1);
      break;

    case CCATDepth:
      if (upperSW) {
        upperData[P_ATDepth] = value;
      } else {
        lowerData[P_ATDepth] = value;
      }
      ATDepthstr = value;
      updateATDepth(1);
      break;

    case CCnoiseLevel:
      if (upperSW) {
        upperData[P_noiseLevel] = value;
      } else {
        lowerData[P_noiseLevel] = value;
      }
      noiseLevelstr = value;
      updatenoiseLevel(1);
      break;

    case CCosc2SawLevel:
      if (upperSW) {
        upperData[P_osc2SawLevel] = value;
      } else {
        lowerData[P_osc2SawLevel] = value;
      }
      osc2SawLevelstr = value;  // for display
      updateOsc2SawLevel(1);
      break;

    case CCosc1SawLevel:
      if (upperSW) {
        upperData[P_osc1SawLevel] = value;
      } else {
        lowerData[P_osc1SawLevel] = value;
      }
      osc1SawLevelstr = value;  // for display
      updateOsc1SawLevel(1);
      break;

    case CCosc2PulseLevel:
      if (upperSW) {
        upperData[P_osc2PulseLevel] = value;
      } else {
        lowerData[P_osc2PulseLevel] = value;
      }
      osc2PulseLevelstr = value;  // for display
      updateOsc2PulseLevel(1);
      break;

    case CCosc1PulseLevel:
      if (upperSW) {
        upperData[P_osc1PulseLevel] = value;
      } else {
        lowerData[P_osc1PulseLevel] = value;
      }
      osc1PulseLevelstr = value;  // for display
      updateOsc1PulseLevel(1);
      break;

    case CCosc1TriangleLevel:
      if (upperSW) {
        upperData[P_osc1TriangleLevel] = value;
      } else {
        lowerData[P_osc1TriangleLevel] = value;
      }
      osc1TriangleLevelstr = value;  // for display
      updateOsc1TriangleLevel(1);
      break;

    case CCosc2SubLevel:
      if (upperSW) {
        upperData[P_osc2SubLevel] = value;
      } else {
        lowerData[P_osc2SubLevel] = value;
      }
      osc2SubLevelstr = value;  // for display
      updateosc2SubLevel(1);
      break;

    case CCosc2EnvDepth:
      if (upperSW) {
        upperData[P_osc2envDepth] = value;
      } else {
        lowerData[P_osc2envDepth] = value;
      }
      osc2envDepthstr = value;  // for display
      updateOsc2EnvDepth(1);
      break;

    case CCLFO1Delay:
      if (upperSW) {
        upperData[P_LFO1Delay] = value;
      } else {
        lowerData[P_LFO1Delay] = value;
      }
      LFO1Delaystr = value;  // for display
      updateLFO1Delay(1);
      break;

    case CCLFO3Delay:
      if (upperSW) {
        upperData[P_LFO3Delay] = value;
      } else {
        lowerData[P_LFO3Delay] = value;
      }
      LFO3Delaystr = value;  // for display
      updateLFO3Delay(1);
      break;

    case CCLFO1Slope:
      if (upperSW) {
        upperData[P_LFO1Slope] = value;
      } else {
        lowerData[P_LFO1Slope] = value;
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
      }
      filterCutoffstr = FILTERCUTOFF[value];
      updateFilterCutoff(1);
      break;

    case CCfilterLFO:
      if (upperSW) {
        upperData[P_filterLFO] = value;
      } else {
        lowerData[P_filterLFO] = value;
      }
      filterLFOstr = value;
      updatefilterLFO(1);
      break;

    case CCfilterRes:
      if (upperSW) {
        upperData[P_filterRes] = value;
      } else {
        lowerData[P_filterRes] = value;
      }
      filterResstr = int(value);
      updatefilterRes(1);
      break;

    case CCfilterType:
      if (upperSW) {
        upperData[P_filterType] = value;
      } else {
        lowerData[P_filterType] = value;
      }
      updateFilterType(1);
      break;

    case CCfilterEGlevel:
      if (upperSW) {
        upperData[P_filterEGlevel] = value;
      } else {
        lowerData[P_filterEGlevel] = value;
      }
      filterEGlevelstr = int(value);
      updatefilterEGlevel(1);
      break;

    case CCLFO1Rate:
      if (upperSW) {
        upperData[P_LFO1Rate] = value;
      } else {
        lowerData[P_LFO1Rate] = value;
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
      }
      LFO3Ratestr = LFOTEMPO[value];  // for display
      updateLFO3Rate(1);
      break;

    case CCmodWheelDepth:
      if (upperSW) {
        upperData[P_modWheelDepth] = value;
      } else {
        lowerData[P_modWheelDepth] = value;
      }
      modWheelDepthstr = value;  // for display
      updatemodWheelDepth(1);
      break;

    case CCPitchBend:
      if (upperSW) {
        upperData[P_PitchBendLevel] = value;
      } else {
        lowerData[P_PitchBendLevel] = value;
      }
      PitchBendLevelstr = value;  // for display
      updatePitchBendDepth(1);
      break;

    case CCeffectPot1:
      if (upperSW) {
        upperData[P_effectPot1] = value;
      } else {
        lowerData[P_effectPot1] = value;
      }
      effectPot1str = value;  // for display
      updateeffectPot1(1);
      break;

    case CCeffectPot2:
      if (upperSW) {
        upperData[P_effectPot2] = value;
      } else {
        lowerData[P_effectPot2] = value;
      }
      effectPot2str = value;  // for display
      updateeffectPot2(1);
      break;

    case CCeffectPot3:
      if (upperSW) {
        upperData[P_effectPot3] = value;
      } else {
        lowerData[P_effectPot3] = value;
      }
      effectPot3str = value;  // for display
      updateeffectPot3(1);
      break;

    case CCvcfATDepth:
      if (upperSW) {
        upperData[P_vcfATDepth] = value;
      } else {
        lowerData[P_vcfATDepth] = value;
      }
      vcfATDepthstr = value;  // for display
      updatevcfATDepth(1);
      break;

    case CCeffectsMix:
      if (upperSW) {
        upperData[P_effectsMix] = value;
      } else {
        lowerData[P_effectsMix] = value;
      }
      effectsMixstr = LINEARCENTREZERO[value];  // for display
      updateeffectsMix(1);
      break;

    case CCpitchAttack:
      if (upperSW) {
        upperData[P_pitchAttack] = value;
      } else {
        lowerData[P_pitchAttack] = value;
      }
      pitchAttackstr = ENVTIMES[value];
      updatepitchAttack(1);
      break;

    case CCpitchDecay:
      if (upperSW) {
        upperData[P_pitchDecay] = value;
      } else {
        lowerData[P_pitchDecay] = value;
      }
      pitchDecaystr = ENVTIMES[value];
      updatepitchDecay(1);
      break;

    case CCpitchSustain:
      if (upperSW) {
        upperData[P_pitchSustain] = value;
      } else {
        lowerData[P_pitchSustain] = value;
      }
      pitchSustainstr = LINEAR_FILTERMIXERSTR[value];
      updatepitchSustain(1);
      break;

    case CCpitchRelease:
      if (upperSW) {
        upperData[P_pitchRelease] = value;
      } else {
        lowerData[P_pitchRelease] = value;
      }
      pitchReleasestr = ENVTIMES[value];
      updatepitchRelease(1);
      break;

    case CCfilterAttack:
      if (upperSW) {
        upperData[P_filterAttack] = value;
      } else {
        lowerData[P_filterAttack] = value;
      }
      filterAttackstr = ENVTIMES[value];
      updatefilterAttack(1);
      break;

    case CCfilterDecay:
      if (upperSW) {
        upperData[P_filterDecay] = value;
      } else {
        lowerData[P_filterDecay] = value;
      }
      filterDecaystr = ENVTIMES[value];
      updatefilterDecay(1);
      break;

    case CCfilterSustain:
      if (upperSW) {
        upperData[P_filterSustain] = value;
      } else {
        lowerData[P_filterSustain] = value;
      }
      filterSustainstr = LINEAR_FILTERMIXERSTR[value];
      updatefilterSustain(1);
      break;

    case CCfilterRelease:
      if (upperSW) {
        upperData[P_filterRelease] = value;
      } else {
        lowerData[P_filterRelease] = value;
      }
      filterReleasestr = ENVTIMES[value];
      updatefilterRelease(1);
      break;

    case CCampAttack:
      if (upperSW) {
        upperData[P_ampAttack] = value;
      } else {
        lowerData[P_ampAttack] = value;
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
      }
      ampReleasestr = ENVTIMES[value];
      updateampRelease(1);
      break;

    case CCvolumeControl:
      if (upperSW) {
        upperData[P_volumeControl] = value;
      } else {
        lowerData[P_volumeControl] = value;
      }
      volumeControlstr = value;
      updatevolumeControl(1);
      break;

    case CCfilterLevel1:
      if (upperSW) {
        upperData[P_filterLevel1] = value;
      } else {
        lowerData[P_filterLevel1] = value;
      }
      filterLevel1str = value;
      updatefilterLevel1(1);
      break;

    case CCfilterLevel2:
      if (upperSW) {
        upperData[P_filterLevel2] = value;
      } else {
        lowerData[P_filterLevel2] = value;
      }
      filterLevel2str = value;
      updatefilterLevel2(1);
      break;

    case CCosc1sawDetune:
      if (upperSW) {
        upperData[P_osc1sawDetune] = value;
      } else {
        lowerData[P_osc1sawDetune] = value;
      }
      osc1sawDetunestr = value;
      updateosc1sawDetune(1);
      break;

    case CCkeyTrack:
      if (upperSW) {
        upperData[P_keytrack] = value;
      } else {
        lowerData[P_keytrack] = value;
      }
      keytrackstr = value;
      updatekeytrack(1);
      break;


    case CCamDepth:
      if (upperSW) {
        upperData[P_amDepth] = value;
      } else {
        lowerData[P_amDepth] = value;
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
      }
      updatedco_at_SW(1);
      break;

    case CCfilter_at_SW:
      if (upperSW) {
        upperData[P_filter_at_SW] = value;
      } else {
        lowerData[P_filter_at_SW] = value;
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

    case CCenv2_punch:
      if (upperSW) {
        upperData[P_env2_punch] = !upperData[P_env2_punch];
      } else {
        lowerData[P_env2_punch] = !lowerData[P_env2_punch];
      }
      updateenv2_punch(1);
      break;

    case CCenv3_punch:
      if (upperSW) {
        upperData[P_env3_punch] = !upperData[P_env3_punch];
      } else {
        lowerData[P_env3_punch] = !lowerData[P_env3_punch];
      }
      updateenv3_punch(1);
      break;

    case CCenv2_env3_adsr:
      if (upperSW) {
        upperData[P_env2_env3_adsr] = !upperData[P_env2_env3_adsr];
      } else {
        lowerData[P_env2_env3_adsr] = !lowerData[P_env2_env3_adsr];
      }
      updateenv2_env3_adsr(1);
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

    case CCfx_Bypass:
      if (upperSW) {
        upperData[P_fx_Bypass] = !upperData[P_fx_Bypass];
      } else {
        lowerData[P_fx_Bypass] = !lowerData[P_fx_Bypass];
      }
      updatefx_Bypass(1);
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

    case CCdriftSW:
      if (upperSW) {
        upperData[P_driftSW] = !upperData[P_driftSW];
      } else {
        lowerData[P_driftSW] = !lowerData[P_driftSW];
      }
      updatedriftSW(1);
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
      }
      updateLFO2Waveform(1);
      break;

    case CCLFO3Waveform:
      if (upperSW) {
        upperData[P_LFO3Waveform] = value;
      } else {
        lowerData[P_LFO3Waveform] = value;
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
        midiCCDCOUpper(WSmodwheel, value / 8);  // divided by 8 because the convert bumps it up to 1023
      } else {
        midiCCDCOLower(WSmodwheel, value / 8);
        if (wholemode) {
          midiCCDCOUpper(WSmodwheel, value / 8);
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
  int tempData[112];  // Temporary array for converted integers

  // Convert data from String to int once
  for (int i = 1; i <= 107; i++) {
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

    // if (wholemode) {

    //   // Update previous values and pick-up flags
    //   for (int i = 1; i <= 105; i++) {
    //     upperData[i] = lowerData[i];  // Store previous value
    //   }

    //   oldfilterCutoffU = upperData[P_filterCutoff];
    //   upperParamsToDisplay();
    //   setAllButtons();
    // }
  }

  updatePatchname();
}

void upperParamsToDisplay() {

  updateglideTime(0);
  updateosc1PW(0);
  updateosc1PWM(0);
  updateOsc1SawLevel(0);
  updateOsc1PulseLevel(0);
  updateosc2SubLevel(0);
  updatefmDepth(0);
  updateosc2PW(0);
  updateosc2PWM(0);
  updateOsc2SawLevel(0);
  updateOsc2PulseLevel(0);
  updateOsc1TriangleLevel(0);
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
  updateLFO3Waveform(0);
  updateeffectPot1(0);
  updateeffectPot2(0);
  updateeffectPot3(0);
  updatevcfATDepth(0);
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
  updateeffectBankSW(0);
  updateeffectNumSW(0);
  updatearpRate(0);
  updateosc1envPWM(0);
  updateosc2envPWM(0);
  updatedualDetune(0);
  updateunisonDetune(0);
  updatedriftDepth(0);
}

void lowerParamsToDisplay() {

  updateglideTime(0);
  updateosc1PW(0);
  updateosc1PWM(0);
  updateOsc1SawLevel(0);
  updateOsc1PulseLevel(0);
  updateosc2SubLevel(0);
  updatefmDepth(0);
  updateosc2PW(0);
  updateosc2PWM(0);
  updateOsc2SawLevel(0);
  updateOsc2PulseLevel(0);
  updateOsc1TriangleLevel(0);
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
  updateLFO3Waveform(0);  
  updateeffectPot1(0);
  updateeffectPot2(0);
  updateeffectPot3(0);
  updatevcfATDepth(0);
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
  updateeffectBankSW(0);
  updateeffectNumSW(0);
  updatearpRate(0);
  updateosc1envPWM(0);
  updateosc2envPWM(0);
  updatedualDetune(0);
  updateunisonDetune(0);
  updatedriftDepth(0);
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
  updateenv2_punch(0);
  updateenv3_punch(0);
  updateenv2_env3_adsr(0);
  updatenoiseSrc(0);
  updateArpLEDs();
  updatedriftSW(0);
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
           + "," + String(upperData[P_AfterTouchDest]) + "," + String(upperData[P_filterLogLin]) + "," + String(upperData[P_ampLogLin]) + "," + String(upperData[P_osc1TriangleLevel])
           + "," + String(upperData[P_osc2SubLevel]) + "," + String(upperData[P_keyboardMode]) + "," + String(upperData[P_LFO1Delay]) + "," + String(upperData[P_effectNum]) + "," + String(upperData[P_effectBank])
           + "," + String(upperData[P_LFO1Slope]) + "," + String(upperData[P_LFO3Rate]) + "," + String(upperData[P_lfoMultiplier]) + "," + String(upperData[P_NotePriority]) + "," + String(upperData[P_keytrackSW])
           + "," + String(upperData[P_ATDepth]) + "," + String(upperData[P_pitchAttack]) + "," + String(upperData[P_pitchDecay]) + "," + String(upperData[P_pitchSustain]) + "," + String(upperData[P_pitchRelease])
           + "," + String(upperData[P_LFO3Delay]) + "," + String(upperData[P_osc1sawDetune]) + "," + String(upperData[P_osc1sawCount]) + "," + String(upperData[P_arpRate])
           + "," + String(upperData[P_LFO3Waveform]) + "," + String(upperData[P_LFO2Waveform]) + "," + String(upperData[P_osc2envDepth]) + "," + String(upperData[P_noiseSrc]) + "," + String(upperData[P_lfo1retrig])
           + "," + String(upperData[P_osc1envPWM]) + "," + String(upperData[P_osc2envPWM]) + "," + String(upperData[P_dco_at_SW]) + "," + String(upperData[P_filter_at_SW])
           + "," + String(upperData[P_arpStartStop]) + "," + String(upperData[P_arpRange]) + "," + String(upperData[P_arpMode]) + "," + String(upperData[P_arpLatch])
           + "," + String(upperData[P_vcfATDepth]) + "," + String(upperData[P_fx_Bypass]) + "," + String(upperData[P_unisonDetune]) + "," + String(upperData[P_dualDetune])
           + "," + String(upperData[P_env2_env3_adsr]) + "," + String(upperData[P_env1_adsr]) + "," + String(upperData[P_env1_punch]) + "," + String(upperData[P_env2_punch])
           + "," + String(upperData[P_env3_punch]) + "," + String(upperData[P_driftDepth]) + "," + String(upperData[P_driftSW]);

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
           + "," + String(lowerData[P_AfterTouchDest]) + "," + String(lowerData[P_filterLogLin]) + "," + String(lowerData[P_ampLogLin]) + "," + String(lowerData[P_osc1TriangleLevel])
           + "," + String(lowerData[P_osc2SubLevel]) + "," + String(lowerData[P_keyboardMode]) + "," + String(lowerData[P_LFO1Delay]) + "," + String(lowerData[P_effectNum]) + "," + String(lowerData[P_effectBank])
           + "," + String(lowerData[P_LFO1Slope]) + "," + String(lowerData[P_LFO3Rate]) + "," + String(lowerData[P_lfoMultiplier]) + "," + String(lowerData[P_NotePriority]) + "," + String(lowerData[P_keytrackSW])
           + "," + String(lowerData[P_ATDepth]) + "," + String(lowerData[P_pitchAttack]) + "," + String(lowerData[P_pitchDecay]) + "," + String(lowerData[P_pitchSustain]) + "," + String(lowerData[P_pitchRelease])
           + "," + String(lowerData[P_LFO3Delay]) + "," + String(lowerData[P_osc1sawDetune]) + "," + String(lowerData[P_osc1sawCount]) + "," + String(lowerData[P_arpRate])
           + "," + String(lowerData[P_LFO3Waveform]) + "," + String(lowerData[P_LFO2Waveform]) + "," + String(lowerData[P_osc2envDepth]) + "," + String(lowerData[P_noiseSrc]) + "," + String(lowerData[P_lfo1retrig])
           + "," + String(lowerData[P_osc1envPWM]) + "," + String(lowerData[P_osc2envPWM]) + "," + String(lowerData[P_dco_at_SW]) + "," + String(lowerData[P_filter_at_SW])
           + "," + String(lowerData[P_arpStartStop]) + "," + String(lowerData[P_arpRange]) + "," + String(lowerData[P_arpMode]) + "," + String(lowerData[P_arpLatch])
           + "," + String(lowerData[P_vcfATDepth]) + "," + String(lowerData[P_fx_Bypass]) + "," + String(lowerData[P_unisonDetune]) + "," + String(lowerData[P_dualDetune])
           + "," + String(lowerData[P_env2_env3_adsr]) + "," + String(lowerData[P_env1_adsr]) + "," + String(lowerData[P_env1_punch]) + "," + String(lowerData[P_env2_punch])
           + "," + String(lowerData[P_env3_punch]) + "," + String(lowerData[P_driftDepth]) + "," + String(lowerData[P_driftSW]);
  }
}

void midiCCOut(byte cc, byte value) {
  MIDI.sendControlChange(cc, value, midiChannel);  //MIDI DIN main out
}

void midiCCDCOLower(byte cc, byte value) {
  MIDI7.sendControlChange(cc, value, 9);  //MIDI to lower board DCOs
}

void midiCCVoiceLower(byte cc, byte value) {
  MIDI7.sendControlChange(cc, value, 10);  //MIDI to lower board Filters etc
}

void midiCCDCOUpper(byte cc, byte value) {
  MIDI8.sendControlChange(cc, value, 9);  //MIDI to upper board DCOs
}

void midiCCVoiceUpper(byte cc, byte value) {
  MIDI8.sendControlChange(cc, value, 10);  //MIDI to upper board Filters etc
}

void midiCCDisplay(byte cc, byte value) {
  MIDI6.sendControlChange(cc, value, 1);  //MIDI DIN to panel for switches
  delay(1);
}

void midiCCDisplaySW(byte cc, byte value) {
  MIDI6.sendControlChange(cc, value, 2);  //MIDI DIN to panel for switches
  delay(1);
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
  usbMIDI.read(midiChannel);
  LFODelayHandle();
  changeSpeed();
  checkChordHold();
  arpEngine();

  // Draw the parameter page here (after the arp step has been serviced),
  // throttled, so turning controls doesn't block the loop and stutter the arp.
  if (paramDisplayDirty && (millis() - lastParamDrawTime >= paramDrawInterval) && !tft.asyncUpdateActive()) {
    refreshScreen();
    lastParamDrawTime = millis();
    paramDisplayDirty = false;
  }

  if (waitingToUpdate && (millis() - lastDisplayTriggerTime >= displayTimeout) && !tft.asyncUpdateActive()) {
    refreshScreen();  // retrigger
    waitingToUpdate = false;
  }
}
