//MIDI CC control numbers
//These broadly follow standard CC assignments
#define   CCmodwheel  1 //pitch LFO amount - less from mod wheel

#define   CCfx_Bypass 4
#define   CCglideTime 5
#define   CCdco_at_SW 6
#define   CCvolumeControl 7
#define   CCfilterA 8
#define   CCfilterB 9
#define   CCamDepth 11
#define   CCosc2Interval 12
#define   CCfilter_at_SW 13
#define   CCATDepth 14
#define   CCfmDepth 15
#define   CCosc1PW 16
#define   CCosc2PW 17
#define   CCosc1PWM 18
#define   CCosc2PWM 19
#define   CCmodWheelDepth 20
#define   CCvcaGate 21
#define   CCfilterLFO 22
#define   CCnoiseLevel 23
#define   CCfilterPoleSW 24
#define   CCfilterLevel2 25
#define   CCfilterLevel1 26
#define   CCFilterLoop 27
#define   CCAmpLoop 28
#define   CCosc2envPWM 29
#define   CCkeyTrack 30
#define   CCkeyTrackSW 31
#define   CCwholemode 32
#define   CCdualmode 33
#define   CCsplitmode 34
#define   CCeffectNumSW 35
#define   CCeffectBankSW 36
#define   CCmonoMulti 37
#define   CCnoiseSrc 38
#define   CCfilterType 39
#define   CClfo1retrig 40
#define   CCosc2EnvDepth 41
#define   CCvcaVel 42
#define   CCfilterVel 43
#define   CCfilterRelease 44
#define   CCfilterAttack 45
#define   CCfilterSustain 46
#define   CCfilterDecay 47
#define   CCfilterLevel 48
#define   CCosc1sawCount 49
#define   CCfilterEGinv 50
#define   CCosc1sawDetune 51
#define   CCfilterEGlevel 53
#define   CCkeyboardMode 54
#define   CCNotePriority 55
#define   CCarpRate 56
#define   CCampRelease 57
#define   CCampAttack 58
#define   CCampSustain 59
#define   CCampDecay 60
#define   CCosc1TriangleLevel 61
#define   CCosc2SubLevel 62
#define   CCLFO1Delay 63

#define   CCglideSW 65
#define   CCPitchBend 66

#define   CCosc1PulseLevel 68
#define   CCosc1SawLevel 69
#define   CCLFO1Slope 70
#define   CCosc2Detune 71
#define   CCLFO3Rate 72
#define   CCLFO3Delay 73
#define   CCfilterCutoff 74
#define   CCosc1envPWM 75
#define   CClfoAlt 76
#define   CCLFO1Rate 77
#define   CClfoMult 78
#define   CCLFO2Waveform 79
#define   CCosc1Oct 80
#define   CCosc2Oct 81
#define   CCLFO3Waveform 82
#define   CCeffectPot1 83
#define   CCeffectPot2  84
#define   CCeffectPot3  85
#define   CCeffectsMix  86
#define   CCLFO2Rate 87
#define   CCfilterenvLinLogSW 88
#define   CCampenvLinLogSW 89
#define   CCplayMode 90
#define   CCLFO1Waveform 91
#define   CCdumpCompleteSW 92
#define   CCdumpStartedSW 93
#define   CCfilterRes 94
#define   CCsyncSW 95
#define   CCchordHoldSW 96
#define   CCupperSW 97
#define   CClowerSW 98

#define   CCvcfATDepth 101
#define   CCosc2PulseLevel 102
#define   CCosc2SawLevel 103
#define   CCeffectparam3 104
#define   CCpitchRelease 105
#define   CCpitchAttack 106
#define   CCpitchSustain 107
#define   CCpitchDecay 108
#define   CCenv2_punch 109
#define   CCenv3_punch 110
#define   CCenv1_punch 111
#define   CCenv2_env3_adsr 112
#define   CCdualDetune 113
#define   CCunisonDetune 114
#define   CCarpStartStop 115
#define   CCarpLatch 116
#define   CCarpMode 117
#define   CCarpRange 118
#define   CCdriftDepth 119
#define   CCdriftSW 120

#define   CCallnotesoff 123//Panic button

// CC DCOS parameters

#define CC_MOD_WHEEL        1   /* modulation wheel                    */
#define CC_PORTAMENTO_TIME  5   /* portamento rate                     */
#define CC_PORTAMENTO_SW    65  /* portamento on/off (>=64=on)         */

#define CC_DCO1_OCTAVE   15  /* octave switch DCO1                */
#define CC_DCO2_OCTAVE   16  /* octave switch DCO2                */

/* --- DCO1 oscillator --- */
#define CC_DCO1_SAW_DETUNE  17  /* saw detune spread                   */
#define CC_DCO1_SAW_COUNT   18  /* saw voice count 1-5                 */
#define CC_DCO1_PULSE_WIDTH 19  /* DCO1 pulse width                    */
#define CC_DCO1_PWM_DEPTH   20  /* DCO1 mod wheel PWM depth            */
#define CC_DCO1_SAW_LEVEL   21  /* saw level in mix                    */
#define CC_DCO1_PULSE_LEVEL 22  /* DCO1 pulse level in mix             */
#define CC_DCO1_TRI_LEVEL   23  /* DCO1 triangle level                 */

/* --- Pitch --- */
#define CC_PITCHBEND_RANGE  24  /* pitchbend range 1-12 semitones      */

/* --- LFO1 (FM/vibrato) --- */
#define CC_LFO1_RATE        25  /* LFO1 rate 0.1-20Hz                  */
#define CC_LFO1_WAVEFORM    26  /* LFO1 waveform tri/sq/saw            */
#define CC_LFO1_FM_DEPTH    27  /* LFO1 -> FM depth                    */
#define CC_LFO1_DELAY_TIME  55  /* LFO1 delay time before onset        */
#define CC_LFO1_DELAY_RAMP  56  /* LFO1 ramp up time after delay       */
#define CC_LFO1_RETRIG      57  /* >= 64 retrigger on, < 64 legato     */
#define CC_NOTES_HELD       58  /* >= 64 notes held, < 64 all released */
#define CC_AT_FM_DEPTH      28  /* aftertouch -> vibrato depth         */
#define CC_MW_FM_DEPTH      29  /* mod wheel -> FM depth               */
#define CC_AT_FM_ENABLE     68  /* aftertouch -> DCO FM on/off (>=64)  */
#define CC_AT_FILTER_ENABLE 69  /* aftertouch -> filter CV on/off      */
#define CC_AT_VCF_DEPTH     70  /* aftertouch -> filter CV depth       */
#define CC_GATE_ENABLE      59  /* <64 gate on (ADSR), >=64 gate off (ADR) */

/* --- LFO2 (PWM) --- */
#define CC_LFO2_RATE        31  /* LFO2 rate 0.1-20Hz                  */
#define CC_LFO2_WAVEFORM    32  /* LFO2 waveform tri/sq/saw            */
#define CC_DCO1_LFO2_PWM    33  /* LFO2 -> DCO1 PWM depth              */
#define CC_DCO2_LFO2_PWM    34  /* LFO2 -> DCO2 PWM depth              */

/* --- DCO2 oscillator --- */
#define CC_DCO2_SAW_LEVEL   36  /* DCO2 sawtooth level                 */
#define CC_DCO2_PULSE_WIDTH 37  /* DCO2 pulse width                    */
#define CC_DCO2_PULSE_LEVEL 38  /* DCO2 pulse level                    */
#define CC_DCO2_SUB_LEVEL   39  /* DCO2 sub level                      */
#define CC_DCO2_PWM_DEPTH   40  /* DCO2 mod wheel PWM depth            */
#define CC_DCO2_DETUNE      42  /* DCO2 detune cents, centre=64        */
#define CC_DCO2_INTERVAL    43  /* DCO2 interval semitones, centre=64  */

/* --- Oscillator sync --- */
#define CC_SYNC_MODE        44  /* 0-42=off 43-84=soft 85-127=hard     */

/* --- DCO2 sweep envelope --- */
#define CC_ENV_ATTACK       45  /* attack  0=slow 127=fast             */
#define CC_ENV_DECAY        46  /* decay   0=slow 127=fast             */
#define CC_ENV_SUSTAIN      47  /* sustain level                       */
#define CC_ENV_RELEASE      48  /* release 0=slow 127=fast             */
#define CC_ENV_DEPTH        49  /* envelope -> DCO2 pitch depth        */
#define CC_KEYTRACK_DEPTH   54  /* keytrack CV output scaling          */
#define CC_KEYTRACK_SW      52  /* >= 64 key on, < 64 key off          */
#define CC_VOICE_DETUNE     53  /* master detune both DCOs, centre=64  */
#define CC_UNISON_MODE      30  /* unison on/off (>=64=on)             */
#define CC_UNISON_DETUNE    35  /* unison detune spread, 0 = none      */
/* FV1 effects processor switch outputs - 0=low, 127=high */
#define CC_FV1_BANK_0     60
#define CC_FV1_BANK_1     61
#define CC_FV1_BANK_2     62
#define CC_FV1_EFFECT_0     63
#define CC_FV1_EFFECT_1     64
#define CC_FV1_EFFECT_2     66
#define CC_FV1_INTERNAL     67
#define CC_ENV_DCO1_PWM     50  /* envelope -> DCO1 PWM depth          */
#define CC_ENV_DCO2_PWM     51  /* envelope -> DCO2 PWM depth          */
#define CC_DRIFT_SW       71  /* analog drift on/off (>=64=on)  */
#define CC_DRIFT_DEPTH    72  /* analog drift depth, 0 = none   */

// CC voice board params.

#define VB_EFFECT_POT1 41
#define VB_EFFECT_POT2 42
#define VB_EFFECT_POT3 43
#define VB_EFFECT_MIX 44
#define VB_NOISE_LEVEL 45
#define VB_FILTER_LFO3 46
#define VB_AMP_LFO3 47
#define VB_EG_DEPTH 48

#define VB_VCF_ATTACK 9
#define VB_VCF_DECAY 10
#define VB_VCF_SUSTAIN 11
#define VB_VCF_RELEASE 12
#define VB_VCA_ATTACK 13
#define VB_VCA_DECAY 14
#define VB_VCA_SUSTAIN 15
#define VB_VCA_RELEASE 16

#define VB_FILTER1_LEVEL 17
#define VB_FILTER2_LEVEL 18
#define VB_FILTER_CUTOFF 19
#define VB_FILTER_RES 20
#define VB_LFO3_RATE 21
#define VB_LFO3_WAVE 22
#define VB_VOLUME 23
#define VB_EFFECT_POT4 24

// Switches CC number

#define VB_FILTER_POLE 64
#define VB_EG_INVERT 65
#define VB_FILTER_LIN_LOG 66
#define VB_AMP_LIN_LOG 67
#define VB_FILTER_VELOCITY 68
#define VB_AMP_VELOCITY 69
#define VB_FILTER_LOOP_BIT1 70
#define VB_FILTER_LOOP_BIT0 71

#define VB_NOISE_SOURCE 72
#define VB_AMP_LOOP_BIT1 73
#define VB_AMP_LOOP_BIT0 74
#define VB_VCF_ENV_PUNCH 75
#define VB_AMP_ENV_PUNCH 76
#define LFO3_SYNC 77
// 78
// 79

#define VB_FILTER_A 80
#define VB_FILTER_B 81
#define VB_FILTER_C 82
// 83
#define VB_MULTIPLIER_BIT2 84
#define VB_MULTIPLIER_BIT1 85
#define VB_MULTIPLIER_BIT0 86
#define VB_LFO3_ALT 87



