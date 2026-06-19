// This optional setting causes Encoder to use more optimized code,
// It must be defined before Encoder.h is included.
#define ENCODER_OPTIMIZE_INTERRUPTS
#include <Encoder.h>
#include <Bounce.h>
#include "TButton.h"

#include "Rotary.h"
#include "RotaryEncOverMCP.h"

// MCP1
#define ARP_OCT1_BUTTON 0
#define ARP_OCT2_BUTTON 1
#define ARP_OCT3_BUTTON 2
#define ARP_OCT4_BUTTON 3
#define ARP_UP_BUTTON 4
#define ARP_DOWN_BUTTON 5
#define ARP_UP_DOWN_BUTTON 6
#define ARP_RAND_BUTTON 7
//MCP2
#define ARP_START_STOP_BUTTON 8
#define ARP_LATCH_BUTTON 9
//MCP3
#define MODE_BUTTON 10
#define HOLD_BUTTON 11
#define UPPER_BUTTON 12
#define LOWER_BUTTON 13
#define UNISON_BUTTON 14
#define MONO_BUTTON 15
#define POLY2_BUTTON 16
#define POLY1_BUTTON 17
//MCP4
#define GLIDE_BUTTON 18
#define PRIORITY_BUTTON 19
//MCP5
#define DCO_AT_BUTTON 20
#define DCO1_OCT_BUTTON 21
//MCP6
// no buttons
//MCP7
#define DCO2_OCT_BUTTON 22
#define DCO2_SYNC_BUTTON 23
//MCP8
#define VCF_TYPE_BUTTON 24
#define VCF_POLE_BUTTON 25
#define VCF_EG_INV_BUTTON 26
#define VCF_KEYTRACK_BUTTON 27
//MCP9
#define VCF_VELOCITY_BUTTON 28
#define VCF_LOOP_BUTTON 29
//MCP10
#define VCF_LIN_LOG_BUTTON 30
//MCP11
#define AMP_VELOCITY_BUTTON 31
#define AMP_LOOP_BUTTON 32
//MCP12
#define AMP_LIN_LOG_BUTTON 33
//MCP13
#define LFO1_WAVE_BUTTON 34
#define LFO2_WAVE_BUTTON 35
#define AMP_ENV_GATE_BUTTON 36
#define LFO3_RETRIG_BUTTON 37
#define LFO3_MULT_BUTTON 38
//MCP14
#define EFFECT_NUM_BUTTON 39
#define EFFECT_BANK_BUTTON 40
#define EFFECT_ROTARY_BUTTON 41
#define LFO1_RETRIG_BUTTON 42
#define NOISE_SRC_BUTTON 43
#define FILTER_AT_BUTTON 44



// Pins for MCP23017
#define GPA0 0
#define GPA1 1
#define GPA2 2
#define GPA3 3
#define GPA4 4
#define GPA5 5
#define GPA6 6
#define GPA7 7
#define GPB0 8
#define GPB1 9
#define GPB2 10
#define GPB3 11
#define GPB4 12
#define GPB5 13
#define GPB6 14
#define GPB7 15

void RotaryEncoderChanged(bool clockwise, int id);

void mainButtonChanged(Button *btn, bool released);

Adafruit_MCP23017 mcp1;
Adafruit_MCP23017 mcp2;
Adafruit_MCP23017 mcp3;
Adafruit_MCP23017 mcp4;
Adafruit_MCP23017 mcp5;
Adafruit_MCP23017 mcp6;
Adafruit_MCP23017 mcp7;
Adafruit_MCP23017 mcp8;
Adafruit_MCP23017 mcp9;
Adafruit_MCP23017 mcp10;
Adafruit_MCP23017 mcp11;
Adafruit_MCP23017 mcp12;
Adafruit_MCP23017 mcp13;
Adafruit_MCP23017 mcp14;

//Array of pointers of all MCPs
Adafruit_MCP23017 *allMCPs[] = { &mcp1, &mcp2, &mcp3, &mcp4, &mcp5, &mcp6, &mcp7, &mcp8, &mcp9, &mcp10, &mcp11, &mcp12, &mcp13, &mcp14 };

/* Array of all rotary encoders and their pins */
RotaryEncOverMCP rotaryEncoders[] = {
  RotaryEncOverMCP(&mcp2, 2, 3, &RotaryEncoderChanged, 1),

  RotaryEncOverMCP(&mcp4, 2, 3, &RotaryEncoderChanged, 2),
  RotaryEncOverMCP(&mcp4, 10, 11, &RotaryEncoderChanged, 3),
  RotaryEncOverMCP(&mcp4, 12, 13, &RotaryEncoderChanged, 4),
  RotaryEncOverMCP(&mcp4, 14, 6, &RotaryEncoderChanged, 5),

  RotaryEncOverMCP(&mcp5, 0, 1, &RotaryEncoderChanged, 6),
  RotaryEncOverMCP(&mcp5, 2, 3, &RotaryEncoderChanged, 7),
  RotaryEncOverMCP(&mcp5, 4, 5, &RotaryEncoderChanged, 8),
  RotaryEncOverMCP(&mcp5, 6, 13, &RotaryEncoderChanged, 9),
  RotaryEncOverMCP(&mcp5, 8, 9, &RotaryEncoderChanged, 10),

  RotaryEncOverMCP(&mcp6, 0, 1, &RotaryEncoderChanged, 11),
  RotaryEncOverMCP(&mcp6, 2, 3, &RotaryEncoderChanged, 12),
  RotaryEncOverMCP(&mcp6, 4, 5, &RotaryEncoderChanged, 13),
  RotaryEncOverMCP(&mcp6, 14, 6, &RotaryEncoderChanged, 14),
  RotaryEncOverMCP(&mcp6, 8, 9, &RotaryEncoderChanged, 15),
  RotaryEncOverMCP(&mcp6, 10, 11, &RotaryEncoderChanged, 16),
  RotaryEncOverMCP(&mcp6, 12, 13, &RotaryEncoderChanged, 17),

  RotaryEncOverMCP(&mcp7, 0, 1, &RotaryEncoderChanged, 18),
  RotaryEncOverMCP(&mcp7, 2, 3, &RotaryEncoderChanged, 19),
  RotaryEncOverMCP(&mcp7, 4, 5, &RotaryEncoderChanged, 20),
  RotaryEncOverMCP(&mcp7, 8, 9, &RotaryEncoderChanged, 21),
  RotaryEncOverMCP(&mcp7, 13, 14, &RotaryEncoderChanged, 22),

  RotaryEncOverMCP(&mcp8, 0, 1, &RotaryEncoderChanged, 23),
  RotaryEncOverMCP(&mcp8, 2, 3, &RotaryEncoderChanged, 24),
  RotaryEncOverMCP(&mcp8, 4, 5, &RotaryEncoderChanged, 25),
  RotaryEncOverMCP(&mcp8, 9, 10, &RotaryEncoderChanged, 26),

  RotaryEncOverMCP(&mcp9, 0, 1, &RotaryEncoderChanged, 27),
  RotaryEncOverMCP(&mcp9, 2, 3, &RotaryEncoderChanged, 28),
  RotaryEncOverMCP(&mcp9, 4, 5, &RotaryEncoderChanged, 29),
  RotaryEncOverMCP(&mcp9, 8, 9, &RotaryEncoderChanged, 30),
  RotaryEncOverMCP(&mcp9, 10, 11, &RotaryEncoderChanged, 31),
  RotaryEncOverMCP(&mcp9, 12, 13, &RotaryEncoderChanged, 32),

  RotaryEncOverMCP(&mcp10, 0, 1, &RotaryEncoderChanged, 33),
  RotaryEncOverMCP(&mcp10, 2, 3, &RotaryEncoderChanged, 34),
  RotaryEncOverMCP(&mcp10, 4, 5, &RotaryEncoderChanged, 35),
  RotaryEncOverMCP(&mcp10, 8, 9, &RotaryEncoderChanged, 36),
  RotaryEncOverMCP(&mcp10, 10, 11, &RotaryEncoderChanged, 37),
  RotaryEncOverMCP(&mcp10, 12, 13, &RotaryEncoderChanged, 38),

  RotaryEncOverMCP(&mcp11, 0, 1, &RotaryEncoderChanged, 39),
  RotaryEncOverMCP(&mcp11, 2, 3, &RotaryEncoderChanged, 40),
  RotaryEncOverMCP(&mcp11, 4, 5, &RotaryEncoderChanged, 41),
  RotaryEncOverMCP(&mcp11, 8, 9, &RotaryEncoderChanged, 42),
  RotaryEncOverMCP(&mcp11, 10, 11, &RotaryEncoderChanged, 43),
  RotaryEncOverMCP(&mcp11, 12, 13, &RotaryEncoderChanged, 44),

  RotaryEncOverMCP(&mcp12, 0, 1, &RotaryEncoderChanged, 45),
  RotaryEncOverMCP(&mcp12, 2, 3, &RotaryEncoderChanged, 46),
  RotaryEncOverMCP(&mcp12, 4, 5, &RotaryEncoderChanged, 47),
  RotaryEncOverMCP(&mcp12, 8, 9, &RotaryEncoderChanged, 48),
  RotaryEncOverMCP(&mcp12, 10, 11, &RotaryEncoderChanged, 49),
  RotaryEncOverMCP(&mcp12, 12, 13, &RotaryEncoderChanged, 50),

  RotaryEncOverMCP(&mcp13, 0, 1, &RotaryEncoderChanged, 51),

  RotaryEncOverMCP(&mcp14, 0, 1, &RotaryEncoderChanged, 52),
  RotaryEncOverMCP(&mcp14, 2, 3, &RotaryEncoderChanged, 53),
  RotaryEncOverMCP(&mcp14, 4, 5, &RotaryEncoderChanged, 54),
  RotaryEncOverMCP(&mcp14, 8, 9, &RotaryEncoderChanged, 55),
  RotaryEncOverMCP(&mcp2, 5, 4, &RotaryEncoderChanged, 56),

};

// after your rotaryEncoders[] definition
constexpr size_t NUM_MCP = sizeof(allMCPs) / sizeof(allMCPs[0]);
constexpr int numMCPs = (int)(sizeof(allMCPs) / sizeof(*allMCPs));
constexpr int numEncoders = (int)(sizeof(rotaryEncoders) / sizeof(*rotaryEncoders));

// an array of vectors to hold pointers to the encoders on each MCP
std::vector<RotaryEncOverMCP *> encByMCP[NUM_MCP];

Button arp_oct1_Button = Button(&mcp1, 0, ARP_OCT1_BUTTON, &mainButtonChanged);
Button arp_oct2_Button = Button(&mcp1, 1, ARP_OCT2_BUTTON, &mainButtonChanged);
Button arp_oct3_Button = Button(&mcp1, 3, ARP_OCT3_BUTTON, &mainButtonChanged);
Button arp_oct4_Button = Button(&mcp1, 4, ARP_OCT4_BUTTON, &mainButtonChanged);
Button arp_up_Button = Button(&mcp1, 8, ARP_UP_BUTTON, &mainButtonChanged);
Button arp_down_Button = Button(&mcp1, 9, ARP_DOWN_BUTTON, &mainButtonChanged);
Button arp_up_down_Button = Button(&mcp1, 10, ARP_UP_DOWN_BUTTON, &mainButtonChanged);
Button arp_rand_Button = Button(&mcp1, 11, ARP_RAND_BUTTON, &mainButtonChanged);

Button arp_start_stop_Button = Button(&mcp2, 0, ARP_START_STOP_BUTTON, &mainButtonChanged);
Button arp_latch_Button = Button(&mcp2, 1, ARP_LATCH_BUTTON, &mainButtonChanged);
Button filter_at_Button = Button(&mcp2, 8, FILTER_AT_BUTTON, &mainButtonChanged);

Button mode_Button = Button(&mcp3, 0, MODE_BUTTON, &mainButtonChanged);
Button hold_Button = Button(&mcp3, 1, HOLD_BUTTON, &mainButtonChanged);
Button upper_Button = Button(&mcp3, 2, UPPER_BUTTON, &mainButtonChanged);
Button lower_Button = Button(&mcp3, 3, LOWER_BUTTON, &mainButtonChanged);
Button unison_Button = Button(&mcp3, 8, UNISON_BUTTON, &mainButtonChanged);
Button mono_Button = Button(&mcp3, 9, MONO_BUTTON, &mainButtonChanged);
Button poly2_Button = Button(&mcp3, 10, POLY2_BUTTON, &mainButtonChanged);
Button poly1_Button = Button(&mcp3, 11, POLY1_BUTTON, &mainButtonChanged);

Button glide_Button = Button(&mcp4, 0, GLIDE_BUTTON, &mainButtonChanged);
Button priority_Button = Button(&mcp4, 1, PRIORITY_BUTTON, &mainButtonChanged);

Button dco_at_Button = Button(&mcp5, 10, DCO_AT_BUTTON, &mainButtonChanged);
Button dco1_oct_Button = Button(&mcp5, 11, DCO1_OCT_BUTTON, &mainButtonChanged);

Button dco2_oct_Button = Button(&mcp7, 6, DCO2_OCT_BUTTON, &mainButtonChanged);
Button dco2_sync_Button = Button(&mcp7, 10, DCO2_SYNC_BUTTON, &mainButtonChanged);

Button vcf_type_Button = Button(&mcp8, 6, VCF_TYPE_BUTTON, &mainButtonChanged);
Button noise_src_Button = Button(&mcp8, 8, NOISE_SRC_BUTTON, &mainButtonChanged);
Button vcf_pole_Button = Button(&mcp8, 11, VCF_POLE_BUTTON, &mainButtonChanged);
Button vcf_eg_inv_Button = Button(&mcp8, 12, VCF_EG_INV_BUTTON, &mainButtonChanged);
Button vcf_keytrack_Button = Button(&mcp8, 13, VCF_KEYTRACK_BUTTON, &mainButtonChanged);

Button vcf_velocity_Button = Button(&mcp9, 6, VCF_VELOCITY_BUTTON, &mainButtonChanged);
Button vcf_loop_Button = Button(&mcp9, 14, VCF_LOOP_BUTTON, &mainButtonChanged);

Button vcf_lin_log_Button = Button(&mcp10, 6, VCF_LIN_LOG_BUTTON, &mainButtonChanged);

Button amp_velocity_Button = Button(&mcp11, 6, AMP_VELOCITY_BUTTON, &mainButtonChanged);
Button amp_loop_Button = Button(&mcp11, 14, AMP_LOOP_BUTTON, &mainButtonChanged);

Button amp_lin_log_Button = Button(&mcp12, 6, AMP_LIN_LOG_BUTTON, &mainButtonChanged);

Button lfo1_wave_Button = Button(&mcp13, 2, LFO1_WAVE_BUTTON, &mainButtonChanged);
Button lfo2_wave_Button = Button(&mcp13, 5, LFO2_WAVE_BUTTON, &mainButtonChanged);
Button amp_env_gate_Button = Button(&mcp13, 8, AMP_ENV_GATE_BUTTON, &mainButtonChanged);
Button lfo3_retrig_Button = Button(&mcp13, 11, LFO3_RETRIG_BUTTON, &mainButtonChanged);
Button lfo3_mult_Button = Button(&mcp13, 13, LFO3_MULT_BUTTON, &mainButtonChanged);

Button lfo1_retrig_Button = Button(&mcp14, 6, LFO1_RETRIG_BUTTON, &mainButtonChanged);
Button effect_num_Button = Button(&mcp14, 10, EFFECT_NUM_BUTTON, &mainButtonChanged);
Button effect_bank_Button = Button(&mcp14, 11, EFFECT_BANK_BUTTON, &mainButtonChanged);
Button effect_rotary_Button = Button(&mcp14, 13, EFFECT_ROTARY_BUTTON, &mainButtonChanged);



Button *mainButtons[] = {
  &arp_oct1_Button,
  &arp_oct2_Button,
  &arp_oct3_Button,
  &arp_oct4_Button,
  &arp_up_Button,
  &arp_down_Button,
  &arp_up_down_Button,
  &arp_rand_Button,
  &arp_start_stop_Button,
  &arp_latch_Button,
  &mode_Button,
  &hold_Button,
  &upper_Button,
  &lower_Button,
  &unison_Button,
  &mono_Button,
  &poly2_Button,
  &poly1_Button,
  &glide_Button,
  &priority_Button,
  &dco_at_Button,
  &dco1_oct_Button,
  &dco2_oct_Button,
  &dco2_sync_Button,
  &vcf_type_Button,
  &vcf_pole_Button,
  &vcf_eg_inv_Button,
  &vcf_keytrack_Button,
  &vcf_velocity_Button,
  &vcf_loop_Button,
  &vcf_lin_log_Button,
  &amp_velocity_Button,
  &amp_loop_Button,
  &amp_lin_log_Button,
  &lfo1_wave_Button,
  &lfo2_wave_Button,
  &lfo3_retrig_Button,
  &lfo3_mult_Button,
  &effect_num_Button,
  &effect_bank_Button,
  &effect_rotary_Button,
  &amp_env_gate_Button,
  &lfo1_retrig_Button,
  &noise_src_Button,
  &filter_at_Button,
};

Button *allButtons[] = {
  &arp_oct1_Button,
  &arp_oct2_Button,
  &arp_oct3_Button,
  &arp_oct4_Button,
  &arp_up_Button,
  &arp_down_Button,
  &arp_up_down_Button,
  &arp_rand_Button,
  &arp_start_stop_Button,
  &arp_latch_Button,
  &mode_Button,
  &hold_Button,
  &upper_Button,
  &lower_Button,
  &unison_Button,
  &mono_Button,
  &poly2_Button,
  &poly1_Button,
  &glide_Button,
  &priority_Button,
  &dco_at_Button,
  &dco1_oct_Button,
  &dco2_oct_Button,
  &dco2_sync_Button,
  &vcf_type_Button,
  &vcf_pole_Button,
  &vcf_eg_inv_Button,
  &vcf_keytrack_Button,
  &vcf_velocity_Button,
  &vcf_loop_Button,
  &vcf_lin_log_Button,
  &amp_velocity_Button,
  &amp_loop_Button,
  &amp_lin_log_Button,
  &lfo1_wave_Button,
  &lfo2_wave_Button,
  &lfo3_retrig_Button,
  &lfo3_mult_Button,
  &effect_num_Button,
  &effect_bank_Button,
  &effect_rotary_Button,
  &amp_env_gate_Button,
  &lfo1_retrig_Button,
  &noise_src_Button,
  &filter_at_Button,
};

// MCP LEDS
// GP1
#define ARP_OCT1_LED 4
#define ARP_OCT2_LED 5
#define ARP_OCT3_LED 6
#define ARP_OCT4_LED 7

#define ARP_UP_LED 12
#define ARP_DOWN_LED 13
#define ARP_UP_DOWN_LED 14
#define ARP_RAND_LED 15

// GP2
#define ARP_START_STOP_LED 6
#define ARP_LATCH_LED 7

// GP3
#define LOWER_LED 4
#define UPPER_LED 5
#define HOLD_LED 6
#define MODE_LED_RED 7

#define MONO_LED 12
#define UNISON_LED 13
#define POLY2_LED 14
#define POLY1_LED 15

// GP4
#define GLIDE_LED_RED 4
#define GLIDE_LED_GREEN 5
#define PRIORITY_LED_GREEN 7
#define MODE_LED_GREEN 8
#define PRIORITY_LED_RED 15

// GP5
#define FILTER_AT_LED 7
#define DCO_AT_LED 12
#define DCO1_OCT_LED_GREEN 14
#define DCO1_OCT_LED_RED 15

// GP6
#define NOISE_SRC_LED_RED 7
#define NOISE_SRC_LED_GREEN 15

// GP7
#define DCO2_OCT_LED_RED 7
#define DCO2_SYNC_LED_RED 11
#define DCO2_SYNC_LED_GREEN 12
#define DCO2_OCT_LED_GREEN 15

// GP8
#define VCF_POLE_LED 7
#define VCF_EG_INV_LED 14
#define VCF_KEYTRACK_LED 15

// GP9
#define VCF_VELOCITY_LED 7
#define VCF_LOOP_LED_RED 15

// GP10
#define VCF_LOOP_LED_GREEN 7
#define VCF_LIN_LOG_LED_RED 14
#define VCF_LIN_LOG_LED_GREEN 15

// GP11
#define AMP_VELOCITY_LED 7
#define AMP_LOOP_LED_RED 15

// GP12
#define AMP_LOOP_LED_GREEN 7
#define AMP_LIN_LOG_LED_RED 14
#define AMP_LIN_LOG_LED_GREEN 15

// GP13
#define LFO1_WAVE_LED_RED 3
#define LFO1_WAVE_LED_GREEN 4
#define LFO2_WAVE_LED_RED 6
#define LFO2_WAVE_LED_GREEN 7
#define AMP_ENV_GATE_LED 9
#define LFO3_RETRIG_LED 12
#define LFO3_MULT_LED_RED 14
#define LFO3_MULT_LED_GREEN 15

// GP14
#define LFO1_RETRIG_LED 7
#define EFFECT_ROTARY_LED_RED 14
#define EFFECT_ROTARY_LED_GREEN 15


//Note DAC
#define MULT1V 107
#define MULT1_2V 123
#define MULT2V 210
#define MULT5V 260
#define MULT33V 172
#define MULT3V 344
#define CLAMP2V 26500  // DAC value that corresponds to 2V

// System Switches etc

#define RECALL_SW 30
#define SAVE_SW 33
#define SETTINGS_SW 32
#define BACK_SW 31

#define ENCODER_PINA 5
#define ENCODER_PINB 4

#define DEBOUNCE 30

static long encPrevious = 0;

TButton saveButton{ SAVE_SW, LOW, HOLD_DURATION, DEBOUNCE, CLICK_DURATION };
TButton settingsButton{ SETTINGS_SW, LOW, HOLD_DURATION, DEBOUNCE, CLICK_DURATION };
TButton backButton{ BACK_SW, LOW, HOLD_DURATION, DEBOUNCE, CLICK_DURATION };
TButton recallButton{ RECALL_SW, LOW, HOLD_DURATION, DEBOUNCE, CLICK_DURATION };  //On encoder

Encoder encoder(ENCODER_PINB, ENCODER_PINA);  //This often needs the pins swapping depending on the encoder

void setupHardware() {

  //Switches

  pinMode(RECALL_SW, INPUT_PULLUP);  //On encoder
  pinMode(SAVE_SW, INPUT_PULLUP);
  pinMode(SETTINGS_SW, INPUT_PULLUP);
  pinMode(BACK_SW, INPUT_PULLUP);

  // LEDs

  mcp1.pinMode(4, OUTPUT);   // pin 7 = GPA7 of MCP2301X
  mcp1.pinMode(5, OUTPUT);  // pin 15 = GPB7 of MCP2301X
  mcp1.pinMode(6, OUTPUT);   // pin 7 = GPA7 of MCP2301X
  mcp1.pinMode(7, OUTPUT);  // pin 15 = GPB7 of MCP2301X
  mcp1.pinMode(12, OUTPUT);   // pin 7 = GPA7 of MCP2301X
  mcp1.pinMode(13, OUTPUT);  // pin 15 = GPB7 of MCP2301X
  mcp1.pinMode(14, OUTPUT);   // pin 7 = GPA7 of MCP2301X
  mcp1.pinMode(15, OUTPUT);  // pin 15 = GPB7 of MCP2301X

  mcp2.pinMode(6, OUTPUT);   // pin 7 = GPA7 of MCP2301X
  mcp2.pinMode(7, OUTPUT);  // pin 15 = GPB7 of MCP2301X

  mcp3.pinMode(4, OUTPUT);   // pin 7 = GPA7 of MCP2301X
  mcp3.pinMode(5, OUTPUT);  // pin 15 = GPB7 of MCP2301X
  mcp3.pinMode(6, OUTPUT);   // pin 7 = GPA7 of MCP2301X
  mcp3.pinMode(7, OUTPUT);  // pin 15 = GPB7 of MCP2301X
  mcp3.pinMode(12, OUTPUT);   // pin 7 = GPA7 of MCP2301X
  mcp3.pinMode(13, OUTPUT);  // pin 15 = GPB7 of MCP2301X
  mcp3.pinMode(14, OUTPUT);   // pin 7 = GPA7 of MCP2301X
  mcp3.pinMode(15, OUTPUT);  // pin 15 = GPB7 of MCP2301X

  mcp4.pinMode(4, OUTPUT);   // pin 7 = GPA7 of MCP2301X
  mcp4.pinMode(5, OUTPUT);  // pin 15 = GPB7 of MCP2301X
  mcp4.pinMode(7, OUTPUT);   // pin 7 = GPA7 of MCP2301X
  mcp4.pinMode(8, OUTPUT);  // pin 15 = GPB7 of MCP2301X
  mcp4.pinMode(15, OUTPUT);  // pin 15 = GPB7 of MCP2301X

  mcp5.pinMode(7, OUTPUT);   // pin 7 = GPA7 of MCP2301X
  mcp5.pinMode(12, OUTPUT);   // pin 7 = GPA7 of MCP2301X
  mcp5.pinMode(14, OUTPUT);  // pin 15 = GPB7 of MCP2301X
  mcp5.pinMode(15, OUTPUT);  // pin 15 = GPB7 of MCP2301X

  mcp6.pinMode(7, OUTPUT);  // pin 15 = GPB7 of MCP2301X
  mcp6.pinMode(15, OUTPUT);  // pin 15 = GPB7 of MCP2301X

  mcp7.pinMode(7, OUTPUT);   // pin 7 = GPA7 of MCP2301X
  mcp7.pinMode(11, OUTPUT);   // pin 7 = GPA7 of MCP2301X
  mcp7.pinMode(12, OUTPUT);   // pin 7 = GPA7 of MCP2301X
  mcp7.pinMode(15, OUTPUT);  // pin 15 = GPB7 of MCP2301X

  mcp8.pinMode(7, OUTPUT);   // pin 7 = GPA7 of MCP2301X
  mcp8.pinMode(14, OUTPUT);   // pin 7 = GPA7 of MCP2301X
  mcp8.pinMode(15, OUTPUT);  // pin 15 = GPB7 of MCP2301X

  mcp9.pinMode(7, OUTPUT);   // pin 7 = GPA7 of MCP2301X
  mcp9.pinMode(15, OUTPUT);  // pin 15 = GPB7 of MCP2301X

  mcp10.pinMode(7, OUTPUT);   // pin 7 = GPA7 of MCP2301X
  mcp10.pinMode(14, OUTPUT);   // pin 7 = GPA7 of MCP2301X
  mcp10.pinMode(15, OUTPUT);  // pin 15 = GPB7 of MCP2301X

  mcp11.pinMode(7, OUTPUT);   // pin 7 = GPA7 of MCP2301X
  mcp11.pinMode(15, OUTPUT);  // pin 15 = GPB7 of MCP2301X

  mcp12.pinMode(7, OUTPUT);   // pin 7 = GPA7 of MCP2301X
  mcp12.pinMode(14, OUTPUT);   // pin 7 = GPA7 of MCP2301X
  mcp12.pinMode(15, OUTPUT);  // pin 15 = GPB7 of MCP2301X  

  mcp13.pinMode(3, OUTPUT);   // pin 7 = GPA7 of MCP2301X
  mcp13.pinMode(4, OUTPUT);   // pin 7 = GPA7 of MCP2301X
  mcp13.pinMode(6, OUTPUT);  // pin 15 = GPB7 of MCP2301X  
  mcp13.pinMode(7, OUTPUT);   // pin 7 = GPA7 of MCP2301X
  mcp13.pinMode(9, OUTPUT);   // pin 7 = GPA7 of MCP2301X
  mcp13.pinMode(12, OUTPUT);   // pin 7 = GPA7 of MCP2301X
  mcp13.pinMode(14, OUTPUT);  // pin 15 = GPB7 of MCP2301X  
  mcp13.pinMode(15, OUTPUT);  // pin 15 = GPB7 of MCP2301X   

  mcp14.pinMode(7, OUTPUT);   // pin 7 = GPA7 of MCP2301X
  mcp14.pinMode(14, OUTPUT);   // pin 7 = GPA7 of MCP2301X
  mcp14.pinMode(15, OUTPUT);  // pin 15 = GPB7 of MCP2301X     
}
