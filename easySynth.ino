/*
 * Copyright (c) 2022 Marcel Licence
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 * Dieses Programm ist Freie Software: Sie können es unter den Bedingungen
 * der GNU General Public License, wie von der Free Software Foundation,
 * Version 3 der Lizenz oder (nach Ihrer Wahl) jeder neueren
 * veröffentlichten Version, weiter verteilen und/oder modifizieren.
 *
 * Dieses Programm wird in der Hoffnung bereitgestellt, dass es nützlich sein wird, jedoch
 * OHNE JEDE GEWÄHR,; sogar ohne die implizite
 * Gewähr der MARKTFÄHIGKEIT oder EIGNUNG FÜR EINEN BESTIMMTEN ZWECK.
 * Siehe die GNU General Public License für weitere Einzelheiten.
 *
 * Sie sollten eine Kopie der GNU General Public License zusammen mit diesem
 * Programm erhalten haben. Wenn nicht, siehe <https://www.gnu.org/licenses/>.
 */

/*
 * Implementation of a simple polyphonic synthesizer module
 * - it supports different waveforms
 * - it supports polyphony
 * - implemented ADSR for velocity and filter
 * - allows usage of multiple oscillators per voice
 *
 */


#ifdef __CDT_PARSER__
#include "cdt.h"
#endif


/* requires the ML_SynthTools library: https://github.com/marcel-licence/ML_SynthTools */
#include <ml_filter.h>
#include <ml_waveform.h>

/*
 * activate the following macro to enable unison mode
 * by default the saw wave form will be used
 * the waveform controllers are remapped to
 * - waveform1 -> detune
 * - waveform2 -> oscillator count
 */
//#define USE_UNISON

#define CHANNEL_MAX 16

/*
 * Param indices for Synth_SetParam function
 */
#define SYNTH_PARAM_VEL_ENV_ATTACK  0
#define SYNTH_PARAM_VEL_ENV_DECAY   1
#define SYNTH_PARAM_VEL_ENV_SUSTAIN 2
#define SYNTH_PARAM_VEL_ENV_RELEASE 3
#define SYNTH_PARAM_FIL_ENV_ATTACK  4
#define SYNTH_PARAM_FIL_ENV_DECAY   5
#define SYNTH_PARAM_FIL_ENV_SUSTAIN 6
#define SYNTH_PARAM_FIL_ENV_RELEASE 7
#ifdef USE_UNISON
#define SYNTH_PARAM_DETUNE_1        8
#define SYNTH_PARAM_UNISON_2        9
#else
#define SYNTH_PARAM_WAVEFORM_1      8
#define SYNTH_PARAM_WAVEFORM_2      9
#endif
#define SYNTH_PARAM_MAIN_FILT_CUTOFF    10
#define SYNTH_PARAM_MAIN_FILT_RESO      11
#define SYNTH_PARAM_VOICE_FILT_RESO     12
#define SYNTH_PARAM_VOICE_NOISE_LEVEL   13

#define SYNTH_PARAM_VOICE_PORT_TIME     14
                                                // New Parameters ( Live = always able to edit / Note = only updates value when a new note is pressed )
#define SYNTH_PARAM_LFO_MOD_SPEED      15       // LFO frequency control - Live but not working ok
#define SYNTH_PARAM_OSC1_LEVEL         16       // Osc 1 level           - Live
#define SYNTH_PARAM_OSC2_LEVEL         17       // Osc 2 level           - Live
#define SYNTH_PARAM_OSC1_TRANSPOSE     18       // Osc 1 Coarse Pitch    - Note on // +-1 octave, stepped in semitones
#define SYNTH_PARAM_OSC2_TRANSPOSE     19       // Osc 1 Coarse Pitch    - Note on // +-1 octave, stepped in semitones
#define SYNTH_PARAM_FILTER_KBD_TRACKING 20      // Filter Cutoff Tracking from keyboard - Live
#define SYNTH_PARAM_LFO_WAVEFORM       21       // LFO Waveform          - Live // 0-0.33->Tri  0.33-0.66->Square  0.66-1.0->Saw 
#define SYNTH_PARAM_LFO_MOD_CUTOFF     22       // Filter Cutoff mod from LFO - Live
#define SYNTH_PARAM_FILTER_ENV_AMT     23       // Filter Cutoff mod from Filter envelope - Live
#define SYNTH_PARAM_VOICE_FILTER_CUTOFF 24      // Filter Cutoff  - Live
#define SYNTH_PARAM_PITCH_ENV          25       // Osc 1 pitch mod by filter envelope - Live
#define SYNTH_PARAM_OSC2_FINE          26       // Osc 2 Fine Tune - Live
#define SYNTH_PARAM_OSC2_LOW_FREQ      27       // Turns Osc 2 into a LFO  - Live  // 0->Normal 1->LFO
#define SYNTH_PARAM_OSC2_KBD_TRACKING  28       // Disables Keyboard Tracking of Osc 2, turns Osc 2 Coarse Pitch parameter into Frequency control - Live // 0->Kbd Tracking 1->No Kbd Tracking
#define SYNTH_PARAM_OSC2_FILTER        29       // Filter Cutoff mod from Osc 2 output - Live
#define SYNTH_PARAM_OSC1_FM            30       // Osc 1 pitch mod from Osc 2 output - Live
#define SYNTH_PARAM_OSC1_PW            31       // Osc 1 Pulse width - Live // 10%-90% only when pulse waveform is selected
#define SYNTH_PARAM_OSC2_PW            32       // Osc 2 Pulse width - Live // 10%-90% only when pulse waveform is selected
#define SYNTH_PARAM_OSC1_PWM           33       // Osc 1 Pulse width modulation from LFO - Live
#define SYNTH_PARAM_OSC2_PWM           34       // Osc 2 Pulse width modulation from LFO - Live
#define SYNTH_PARAM_OSC1_PWM_ENV       35       // Osc 1 Pulse width modulation from filter envelope - Live
#define SYNTH_PARAM_LFO_SOURCE         36       // LFO output mix between LFO signal and Noise signal - Live // 0->100% LFO signal 1->100% Noise signal

#define SYNTH_PARAM_GLOBAL_GAIN        38       // Global Gain - Live
#define SYNTH_PARAM_OSC1_PITCH_MOD     39       // Osc 1 pitch mod from LFO - Live
#define SYNTH_PARAM_OSC2_PITCH_MOD     40       // Osc 2 pitch mod from LFO - Live

/*
 * Following defines can be changed for different puprposes
 */
#ifdef USE_UNISON
/* use another setting, because unison supports more than 2 osc per voice */
#define MAX_DETUNE      12 /* 1 + 11 additional tones */
#define MAX_POLY_OSC    36 /* osc polyphony, always active reduces single voices max poly */
#define MAX_POLY_VOICE  3  /* max single voices, can use multiple osc */
#else
#define MAX_POLY_OSC    10 /* osc polyphony, always active reduces single voices max poly */
#define MAX_POLY_VOICE  5 /* max single voices, can use multiple osc */
#endif


#define MIDI_NOTE_CNT 128
static uint32_t midi_note_to_add[MIDI_NOTE_CNT]; /* lookup to playback waveforms with correct frequency */

#ifdef USE_UNISON
uint32_t midi_note_to_add50c[MIDI_NOTE_CNT]; /* lookup for detuning */
#endif

/*
 * set the correct count of available waveforms
 */
#define WAVEFORM_TYPE_COUNT 7
/*
 * add here your waveforms
 */
#if 0
float *sine = NULL;
#else
float sine[WAVEFORM_CNT];
#endif
float *saw = NULL;
float *square = NULL;
float *pulse = NULL;
float *tri = NULL;
float *crappy_noise = NULL;
float *silence = NULL;

/*
 * do not forget to enter the waveform pointer addresses here
 */
float *waveFormLookUp[WAVEFORM_TYPE_COUNT];

struct adsrT
{
    float a;
    float d;
    float s;
    float r;
};

typedef enum
{
    attack, decay, sustain, release
} adsr_phaseT;

/* this prototype is required .. others not -  i still do not know what magic arduino is doing */
inline bool ADSR_Process(const struct adsrT *ctrl, float *ctrlSig, adsr_phaseT *phase);


static struct filterCoeffT filterGlobalC;
static struct filterProcT mainFilterL, mainFilterR;


#define NOTE_STACK_MAX  8


struct channelSetting_s
{
#ifdef USE_UNISON
    float detune; /* detune parameter */
    uint8_t unison; /* additional osc per voice count */
    float *selectedWaveForm;
    float *selectedWaveForm2;
#else
    float *selectedWaveForm;
    float *selectedWaveForm2;

    float osc1level;
    float osc2level;
    int32_t osc1transpose;
    float osc2transpose;
    float osc2fine;
    float osc2lowfreq;
    uint8_t osc2kbdtrk;
    float osc1Fm;
    float osc1Pw;
    float osc2Pw;
    float osc1PwmLfo;
    float osc2PwmLfo;
    float osc1Pw_modulation;
    float osc2Pw_modulation;
    float osc1Pw_env_mod;
    float filterOsc2;
    float filterKbdTrack;
    float lfo_waveform;
    float lfo_mod_cutoff;
    float f_env_amt;
    float f_modulation;
    float f_cutoff;
    float p_envelope;
    float pwm_envelope;
    float lfoSource;
    float globalGain;
    float osc1PitchMod;
    float osc2PitchMod;
#endif

    float soundFiltReso;
    float soundNoiseLevel;

    struct adsrT adsr_vol;
    struct adsrT adsr_fil;

    /* modulation */
    float modulationDepth;
    float modulationSpeed;
    float modulationPitch;

    /* pitchbend */
    float pitchBendValue;
    float pitchMultiplier;
    float pitchMultiplier2;
    
    /* mono mode variables */
    bool mono;

    float portAdd;
    float port;
    float noteA;
    float noteB;

    uint32_t noteCnt;
    uint32_t noteStack[NOTE_STACK_MAX];
};

static struct channelSetting_s chCfg[CHANNEL_MAX];
static struct channelSetting_s *curChCfg = &chCfg[0];
int8_t presetNumber = 0;

struct oscillatorT
{
    float **waveForm;
    float *dest;
    uint32_t samplePos;
    uint32_t addVal;
    float pan_l;
    float pan_r;
    struct channelSetting_s *cfg;
};

float voiceSink[2];
struct oscillatorT oscPlayer[MAX_POLY_OSC];

static uint32_t osc_act = 0;

struct notePlayerT
{
    float lastSample[2];

    float velocity;
    bool active;
    adsr_phaseT phase;

    uint8_t midiCh;
    uint8_t midiNote;

    float control_sign;
    float out_level;

    struct filterCoeffT filterC;
    struct filterProcT filterL;
    struct filterProcT filterR;
    float f_control_sign;
    float f_control_sign_slow;
    adsr_phaseT f_phase;
    float f_cutoff_mod;
    
    struct channelSetting_s *cfg;
};



struct notePlayerT voicePlayer[MAX_POLY_VOICE];

uint32_t voc_act = 0;



void Synth_Init()
{
#ifdef ESP32
    randomSeed(34547379);
#endif

    /*
     * we do not check if malloc was successful
     * if there is not enough memory left the application will crash
     */
#if 0
    sine = (float *)malloc(sizeof(float) * WAVEFORM_CNT);
#endif
    saw = (float *)malloc(sizeof(float) * WAVEFORM_CNT);
    square = (float *)malloc(sizeof(float) * WAVEFORM_CNT);
    pulse = (float *)malloc(sizeof(float) * WAVEFORM_CNT);
    tri = (float *)malloc(sizeof(float) * WAVEFORM_CNT);
    crappy_noise = (float *)malloc(sizeof(float) * WAVEFORM_CNT);
    silence = (float *)malloc(sizeof(float) * WAVEFORM_CNT);

    /*
     * let us calculate some waveforms
     * - using lookup tables can save a lot of processing power later
     * - but it does consume memory
     */
    #define ANTIALIAS_FILTER 2 // Soften a little the transition from +1 to -1 to reduce saw alias. To do: also implement in pulse waveform. 
    
    for (int i = 0; i < WAVEFORM_CNT; i++)
    {
        float val = (float)sin(i * 2.0 * PI / WAVEFORM_CNT);    // SINE WAVE
        sine[i] = val;
        
        saw[i] = (2.0f * ((float)i) / ((float)WAVEFORM_CNT)) - 1.0f;    // SAW WAVE
        
        if( i < ANTIALIAS_FILTER )
          saw[i] = ((float)i) / -1.0f / ANTIALIAS_FILTER;
        if( i> (WAVEFORM_CNT-ANTIALIAS_FILTER) )
          saw[i] = ((float)(i-(WAVEFORM_CNT-1))) / -1.0f / ANTIALIAS_FILTER;
          
        square[i] = (i > (WAVEFORM_CNT / 2)) ? 1 : -1;        // SQUARE WAVE
 
        pulse[i] = ((i > (WAVEFORM_CNT / 2)) ? (((4.0f * (float)i) / ((float)WAVEFORM_CNT)) - 1.0f) : (3.0f - ((4.0f * (float)i) / ((float)WAVEFORM_CNT)))) - 2.0f;   // PWM SQUARE WAVE
        
        tri[i] = ((i > (WAVEFORM_CNT / 2)) ? (((4.0f * (float)i) / ((float)WAVEFORM_CNT)) - 1.0f) : (3.0f - ((4.0f * (float)i) / ((float)WAVEFORM_CNT)))) - 2.0f;    // TRI WAVE
        
        crappy_noise[i] = -1.0f * saw[i];   //  INVERTED SAW
        
        silence[i] = 0;
    }
    
    waveFormLookUp[0] = sine;
    waveFormLookUp[1] = saw;
    waveFormLookUp[2] = square;
    waveFormLookUp[3] = pulse;
    waveFormLookUp[4] = tri;
    waveFormLookUp[5] = crappy_noise;
    waveFormLookUp[6] = silence;

    /*
     * initialize all oscillators
     */
    for (int i = 0; i < MAX_POLY_OSC; i++)
    {
        oscillatorT *osc = &oscPlayer[i];
        osc->waveForm = &silence;
        osc->dest = voiceSink;
        osc->cfg = &chCfg[0];
    }

    /*
     * initialize all voices
     */
    for (int i = 0; i < MAX_POLY_VOICE; i++)
    {
        notePlayerT *voice = &voicePlayer[i];
        voice->active = false;
        voice->lastSample[0] = 0.0f;
        voice->lastSample[1] = 0.0f;
        voice->filterL.filterCoeff = &voice->filterC;
        voice->filterR.filterCoeff = &voice->filterC;
        voice->cfg = &chCfg[0];
    }

    /*
     * prepare lookup for constants to drive oscillators
     */
    for (int i = 0; i < MIDI_NOTE_CNT; i++)
    {
        float f = ((pow(2.0f, (float)(i - 69) / 12.0f) * 440.0f));
        uint32_t add = (uint32_t)(f * ((float)(1ULL << 32ULL) / ((float)SAMPLE_RATE)));
        midi_note_to_add[i] = add;
#ifdef USE_UNISON
        /* filling the table which will be used for detuning */
        float f1 = (pow(2.0f, ((float)(i - 69) + 0.5f) / 12.0f) * 440.0f);
        float f2 = (pow(2.0f, ((float)(i - 69) - 0.5f) / 12.0f) * 440.0f);

        midi_note_to_add50c[i] = (uint32_t)((f1 - f2) * ((float)(1ULL << 32ULL) / ((float)SAMPLE_RATE)));
#endif
    }

    /*
     * assign main filter
     */
    mainFilterL.filterCoeff = &filterGlobalC;
    mainFilterR.filterCoeff = &filterGlobalC;

    Filter_Proc_Init(&mainFilterL);
    Filter_Proc_Init(&mainFilterR);
    Filter_Coeff_Init(mainFilterL.filterCoeff);

    Filter_Calculate(1.0f, 1.0f, &filterGlobalC);

    for (int i = 0; i < CHANNEL_MAX; i++)
    {
        Synth_ChannelSettingInit(&chCfg[i]);
    }
}

static struct filterCoeffT mainFilt;

static float filtCutoff = 0.8f;
static float filtReso = 0.5f;

static void Synth_ChannelSettingInit(struct channelSetting_s *setting)
{
#ifdef USE_UNISON
    setting->detune = 0.1; /* detune parameter */
    setting->unison = 0; /* additional osc per voice count */
    setting->selectedWaveForm = saw;
    setting->selectedWaveForm2 = saw;
#else
    setting->selectedWaveForm = saw;
    setting->selectedWaveForm2 = saw;
#endif

    setting->soundFiltReso = 0.5f;
    setting->soundNoiseLevel = 0.0f;

    struct adsrT adsr_vol_def = {0.865f, 0.865f, 0.75f, 0.865f};
    struct adsrT adsr_fil_def = {0.865f, 0.865f, 0.75f, 0.865f};
    
    memcpy(&setting->adsr_vol, &adsr_vol_def, sizeof(adsr_vol_def));
    memcpy(&setting->adsr_fil, &adsr_fil_def, sizeof(adsr_vol_def));

    setting->modulationDepth = 0.0f;
    setting->modulationSpeed = 5.0f;
    setting->modulationPitch = 1.0f;

    setting->pitchBendValue = 0.0f;
    setting->pitchMultiplier = 1.0f;
    setting->pitchMultiplier2 = 1.0f;
    
    setting->mono = false;
    setting->portAdd = 0.01f; /*!< speed of portamento */
    setting->port = 1.0f;
    setting->noteA = 0;
    setting->noteB = 0;

    setting->noteCnt = 0;
    /* setting->noteStack[NOTE_STACK_MAX]; can be left uninitialized */
    setting->osc1transpose = 0;
    setting->osc2transpose = 0;
    setting->osc1level = 1.0f;
    setting->osc2level = 1.0f;
    setting->filterKbdTrack = 0.0f;
    setting->lfo_waveform = 0;
    setting->lfo_mod_cutoff = 0;
    setting->f_env_amt = 0.5f;
    setting->f_modulation = 0;
    setting->f_cutoff = 0.5f;
    setting->p_envelope = 0;
    setting->osc2fine = 0.5f;
    setting->osc2lowfreq = 0;
    setting->osc2kbdtrk = 0;
    setting->filterOsc2 = 0;
    setting->osc1Fm = 0;
    setting->osc1Pw = 0;
    setting->osc2Pw = 0;
    setting->osc1PwmLfo = 0;
    setting->osc2PwmLfo = 0;
    setting->osc1Pw_modulation = 0;
    setting->osc1Pw_modulation = 0;
    setting->pwm_envelope = 0;
    setting->osc1Pw_env_mod = 0;
    setting->lfoSource = 0;
    setting->globalGain = 0.4f;
    setting->osc1PitchMod = 0.0f;
    setting->osc2PitchMod = 0.0f;
}

/*
 * very bad and simple implementation of ADSR
 * - but it works for the start
 */
inline bool ADSR_Process(const struct adsrT *ctrl, float *ctrlSig, adsr_phaseT *phase)  // Exponential adsr inspired on https://github.com/m0xpd/ADSRduino
{
    switch (*phase)
    {
    case attack:
        *ctrlSig = ( 1.0f - ctrl->a ) + (*ctrlSig * ctrl->a); // y(k) = (1 - alpha) * x(k) + alpha * y(k-1)  //  y(k)=*ctrlSig ... x(k) = 1.0f ... alpha = attack time constant
        if (*ctrlSig >= 0.993f)                               // 0.993 = 5*Tau                         
        {
            *ctrlSig = 1.0f;
            *phase = decay;
        }
        break;
    case decay:
        *ctrlSig = ( 1.0f - ctrl->d ) * ctrl->s + (*ctrlSig * ctrl->d); //  y(k)=*ctrlSig ... x(k) = sustain value ... alpha = decay time constant
        if (*ctrlSig <= (ctrl->s * 1.007f) )                            // 1.007 = 1/(5*Tau)
        {
            *ctrlSig = ctrl->s;
            *phase = sustain;
        }
        break;
    case sustain:
        break;
    case release:
        *ctrlSig = *ctrlSig * ctrl->r; //  y(k)=*ctrlSig ... x(k) = 0.0f ... alpha = release time constant
        if (*ctrlSig <= 0.01f)
        {
            *ctrlSig = 0.0f;
            //voice->active = false;
            return false;
        }
    }
    return true;
}

void Voice_Off(uint32_t i)
{
    notePlayerT *voice = &voicePlayer[i];
    for (int f = 0; f < MAX_POLY_OSC; f++)
    {
        oscillatorT *osc = &oscPlayer[f];
        if (osc->dest == voice->lastSample)
        {
            osc->dest = voiceSink;
            osc_act -= 1;
        }
    }
    voc_act -= 1;
}

inline
float SineNorm(float alpha_div2pi)
{
    uint32_t index = ((uint32_t)(alpha_div2pi * ((float)WAVEFORM_CNT))) % WAVEFORM_CNT;

    if(curChCfg->lfo_waveform > 0.66f)
      return square[index];               // 0.66 < LFO square output < 1.0
    else
      if(curChCfg->lfo_waveform > 0.33f)  // 0.33 < LFO saw output < 0.66
         return saw[index];
      else
          return tri[index];              // 0.0 < LFO saw output < 0.33
}

inline
float GetModulation(uint8_t ch)
{
    float modSpeed = curChCfg->modulationSpeed;
    return curChCfg->modulationPitch * (SineNorm((modSpeed * ((float)millis()) / 1000.0f)));  // OSC 1 PITCH MODULATION
}

inline
float GetFilterModulation(uint8_t ch)
{
    float modSpeed = curChCfg->modulationSpeed;
    return curChCfg->modulationPitch * (SineNorm((modSpeed * ((float)millis()) / 1000.0f)));
}

inline
float GetPwmModulation(uint8_t ch)
{
    float modSpeed = curChCfg->modulationSpeed;
    return curChCfg->modulationPitch * (SineNorm((modSpeed * ((float)millis()) / 1000.0f))); // OSC 1 PWM
}

static uint32_t count = 0;
uint32_t lfsr = 0xACE1FFFF;  // NOISE - linear feedback shift register with non-zero seed number

//[[gnu::noinline, gnu::optimize ("fast-math")]]
inline void Synth_Process(float *left, float *right, uint32_t len) 
{
    /*
     * update pitch bending / modulation
     */
    float noise_signal;                               // White Noise Generator - https://github.com/GadgetReboot/White_Noise_LFSR
    {
        
        for (int i = 0; i < CHANNEL_MAX; i++)
        {
            uint32_t reg = ((lfsr >> 1) ^ (lfsr >> 29)) & 1;  // perform xor of tap points with result in reg LSB (bit 0)
            lfsr = (lfsr >> 1) | (reg << 31);                 // perform LFSR shift for this clock cycle and feed output back to input
            noise_signal = lfsr & 1;                          // send the lfsr output to the noise variable

            float osc1_pitchMod = (  GetModulation(i)          * (1.0f - chCfg[i].lfoSource) +  noise_signal * chCfg[i].lfoSource * 1.5f ) * chCfg[i].osc1PitchMod    * chCfg[i].modulationDepth;
            float osc2_pitchMod = (  GetModulation(i)          * (1.0f - chCfg[i].lfoSource) +  noise_signal * chCfg[i].lfoSource * 1.5f ) * chCfg[i].osc2PitchMod    * chCfg[i].modulationDepth;
            float filter_modulation = ( GetFilterModulation(i) * (1.0f - chCfg[i].lfoSource) +  noise_signal * chCfg[i].lfoSource * 1.5f ) * chCfg[i].lfo_mod_cutoff  * chCfg[i].modulationDepth;
            float osc1_pw_modulation = (GetPwmModulation(i)    * (1.0f - chCfg[i].lfoSource) +  noise_signal * chCfg[i].lfoSource * 1.5f ) * chCfg[i].osc1PwmLfo      * chCfg[i].modulationDepth;
            float osc2_pw_modulation = (GetPwmModulation(i)    * (1.0f - chCfg[i].lfoSource) +  noise_signal * chCfg[i].lfoSource * 1.5f ) * chCfg[i].osc2PwmLfo      * chCfg[i].modulationDepth;
                         
            chCfg[i].port += chCfg[i].portAdd; /* active portamento */
            chCfg[i].port = chCfg[i].port > 1.0f ? 1.0f : chCfg[i].port; /* limit value to max of 1.0f */

            float portVal = (((float)(chCfg[i].noteA)) * (1.0f - chCfg[i].port) + ((float)(chCfg[i].noteB)) * chCfg[i].port);

            float pitchVar1 = chCfg[i].pitchBendValue + osc1_pitchMod + portVal;
            float pitchVar2 = chCfg[i].pitchBendValue + osc2_pitchMod + portVal;

            chCfg[i].pitchMultiplier  = pow(2.0f, pitchVar1 / 12.0f);
            chCfg[i].pitchMultiplier2 = pow(2.0f, pitchVar2 / 12.0f) + (chCfg[i].osc2fine - 0.5f) * 0.01f - 0.9f * chCfg[i].osc2lowfreq;
                                          //    Osc 2                       Frequency Fine                 +   Low frequency
            chCfg[i].f_modulation = filter_modulation;
            chCfg[i].osc1Pw_modulation = osc1_pw_modulation;
            chCfg[i].osc2Pw_modulation = osc2_pw_modulation;
        }
    }

    for (uint32_t n = 0; n < len; n++)
    {
        /* counter required to optimize processing */
        count += 1;

        /*
         * destination for unused oscillators
         */
        voiceSink[0] = 0;
        voiceSink[1] = 0;

        /*
         * oscillator processing -> mix to voice
         */
        for (int i = 0; i < MAX_POLY_OSC; i++)
        {
            oscillatorT *osc = &oscPlayer[i];

                if( i % 2 == 0)
                  osc->samplePos += (uint32_t)(osc->cfg->pitchMultiplier  * ((float)osc->addVal)); // OSC 1
                else
                  osc->samplePos += (uint32_t)(osc->cfg->pitchMultiplier2 * ((float)osc->addVal) +  curChCfg->osc2transpose * 100000000.0f * curChCfg->osc2kbdtrk );  //OSC 2

                float sig = (*osc->waveForm)[WAVEFORM_I(osc->samplePos)];

                if( (i % 2 == 0) && (curChCfg->selectedWaveForm == pulse) )   // OSC 1 PULSE WIDTH MODULATION
                {
                    float pwm1 = curChCfg->osc1Pw + curChCfg->osc1Pw_modulation + curChCfg->pwm_envelope * curChCfg->osc1Pw_env_mod;
                    pwm1 = pwm1 > 0.98f ? 0.98f : pwm1;     // apply limits to pwm modulation
                    pwm1 = pwm1 < -0.98f ? -0.98f : pwm1;
                    sig = sig > pwm1 ? 1.0f : -1.0f;    // Pulse output from Triangle input signal
                }
                
                if( (i % 2 == 1) && (curChCfg->selectedWaveForm2 == pulse) )  // OSC 2 PULSE WIDTH MODULATION
                {
                    float pwm2 = curChCfg->osc2Pw + curChCfg->osc2Pw_modulation;
                    pwm2 = pwm2 > 0.98f ? 0.98f : pwm2;   // apply limits to pwm modulation
                    pwm2 = pwm2 < -0.98f ? -0.98f : pwm2;
                    sig = sig > pwm2 ? 1.0f : -1.0f;    // Pulse output from Triangle input signal
                }    
                
                osc->dest[i % 2] += osc->pan_l * sig;
                osc->dest[i % 2] += osc->pan_r * sig;
        }

        /*
         * voice processing
         */
        for (int i = 0; i < MAX_POLY_VOICE; i++) /* one loop is faster than two loops */
        {
            notePlayerT *voice = &voicePlayer[i];
            
            if (voice->active)
            {
                if (n % 4 == 0)  // Enters 24000 times in this function every second
                {
                    voice->active = ADSR_Process(&voice->cfg->adsr_vol, &voice->control_sign, &voice->phase);
                    (void)ADSR_Process(&voice->cfg->adsr_fil, &voice->f_control_sign, &voice->f_phase);
                    if (voice->active == false)
                    {
                        Voice_Off(i);
                    }
                    /*
                     * make is slow to avoid bad things .. or crying ears
                     */
                }
                                                                  // White Noise Generator - https://github.com/GadgetReboot/White_Noise_LFSR
                uint32_t reg = ((lfsr >> 1) ^ (lfsr >> 29)) & 1;  // perform xor of tap points with result in reg LSB (bit 0)
                lfsr = (lfsr >> 1) | (reg << 31);                 // perform LFSR shift for this clock cycle and feed output back to input
                noise_signal = lfsr & 1;                          // send the lfsr output to the noise variable

                curChCfg->pitchMultiplier = curChCfg->pitchMultiplier + voice->f_control_sign_slow * curChCfg->p_envelope * 0.005f + voice->lastSample[1] * curChCfg->osc1Fm * 0.0625f; // OSC A PITCH MODULATION
                //                                                             filter envelope modulation                          +     osc2->osc1 FM
                 
                voice->lastSample[0] = voice->lastSample[0] * curChCfg->osc1level + noise_signal * voice->cfg->soundNoiseLevel;   // add noise + individual OSC 1 level
                voice->lastSample[1] = voice->lastSample[1] * curChCfg->osc2level + noise_signal * voice->cfg->soundNoiseLevel;   // add noise + individual OSC 2 level
                
                if (count % 32 == 0)
                {
                    voice->f_control_sign_slow = 0.05 * voice->f_control_sign + 0.95 * voice->f_control_sign_slow;
                    
                    curChCfg->pwm_envelope = voice->f_control_sign_slow;
                    
                    voice->f_cutoff_mod = curChCfg->f_cutoff + voice->f_control_sign_slow * curChCfg->f_env_amt + ( curChCfg->f_modulation*0.5f * (1.0f - curChCfg->lfoSource) + noise_signal * curChCfg->lfoSource ) * curChCfg->lfo_mod_cutoff * 0.7f + ((float)voice->midiNote/127) * curChCfg->filterKbdTrack + voice->lastSample[1] * curChCfg->filterOsc2 * 0.25f;
                    //                     Main cutoff value +     Filter envelope amount                       +          LFO/Noise modulation                                                                                                         +    Keyboard Tracking
                    voice->f_cutoff_mod = voice->f_cutoff_mod > 0.99f ? 0.99f : voice->f_cutoff_mod;
                    voice->f_cutoff_mod = voice->f_cutoff_mod < 0.0f ? 0.0f : voice->f_cutoff_mod; // Limit filter value
                            
                    Filter_Calculate(voice->f_cutoff_mod, voice->cfg->soundFiltReso, &voice->filterC);
                }

                Filter_Process(&voice->lastSample[0], &voice->filterL);
                Filter_Process(&voice->lastSample[1], &voice->filterR);
                
                voice->lastSample[0] *= voice->control_sign * voice->velocity;   // Applies OSC 1 VCA + Velocity
                voice->lastSample[1] *= voice->control_sign * voice->velocity;   // Applies OSC 2 VCA + Velocity
                
                left[n]  = left[n]  + voice->lastSample[0] + voice->lastSample[1];
                right[n] = right[n] + voice->lastSample[0] + voice->lastSample[1];

                voice->lastSample[0] = 0.0f;
                voice->lastSample[1] = 0.0f;
            }
        }
    }

    /*
     * process main filter
     */
    Filter_Process_Buffer(left, &mainFilterL, len);
    Filter_Process_Buffer(right, &mainFilterR, len);
        
    /*
     * reduce level a bit to avoid distortion
     */
    for (int i = 0; i < SAMPLE_BUFFER_SIZE; i++)
    {
        left[i]  *= curChCfg->globalGain * 0.25f;
        right[i] *= curChCfg->globalGain * 0.25f;
    }

#ifdef LIMITER_ACTIVE
    for (int i = 0; i < SAMPLE_BUFFER_SIZE; i++)
    {
        left[i] = left[i] > 0.5f ? 0.5 : left[i];
        left[i] = left[i] < -0.5f ? -0.5 : left[i];
        right[i] = right[i] > 0.5f ? 0.5 : right[i];
        right[i] = right[i] < -0.5f ? -0.5 : right[i];
    }
#endif
}

struct oscillatorT *getFreeOsc()
{
    for (int i = 0; i < MAX_POLY_OSC ; i++)
    {
        if (oscPlayer[i].dest == voiceSink)
        {
            return &oscPlayer[i];
        }
    }
    return NULL;
}

static struct notePlayerT *getFreeVoice(void)
{
    for (int i = 0; i < MAX_POLY_VOICE ; i++)
    {
        if (voicePlayer[i].active == false)
        {
            return &voicePlayer[i];
        }
    }
    Voice_Off(0);           // steal the first note played
    return &voicePlayer[0]; 
}

inline void Synth_NoteOn(uint8_t ch, uint8_t note, float vel)
{
    struct notePlayerT *voice = getFreeVoice();
    struct oscillatorT *osc = getFreeOsc();

    /* put note onto stack */
    if (chCfg[ch].mono)
    {
        if (chCfg[ch].noteCnt < (NOTE_STACK_MAX - 1))
        {
            chCfg[ch].noteStack[chCfg[ch].noteCnt] = note;
            chCfg[ch].noteCnt++;
            //Status_ValueChangedIntArr("noteCnt", chCfg[ch].noteCnt, ch);
        }

        if (chCfg[ch].noteCnt > 1)
        {
            for (int i = 0; i < MAX_POLY_VOICE ; i++)
            {
                if ((voicePlayer[i].active) && (voicePlayer[i].midiCh == ch))
                {
                    float diff = note - voicePlayer[i].midiNote;

                    voicePlayer[i].cfg->noteA = voicePlayer[i].cfg->port * ((float)voicePlayer[i].cfg->noteB) + (1.0f - voicePlayer[i].cfg->port) * voicePlayer[i].cfg->noteA;
                    voicePlayer[i].cfg->port = 0.0f;

                    voicePlayer[i].cfg->noteB += diff;
                    voicePlayer[i].midiNote = note;

                    return;
                }
            }
        }
    }

    /*
     * No free voice found, return otherwise crash xD
     */
    if ((voice == NULL) || (osc == NULL))
    {
        //Serial.printf("voc: %d, osc: %d\n", voc_act, osc_act);
        return ;
    }

    voice->cfg = &chCfg[presetNumber];
    voice->midiCh = ch;
    voice->midiNote = note;
#ifdef MIDI_USE_CONST_VELOCITY
    voice->velocity = 1.0f;
#else
    voice->velocity = vel;
#endif
    voice->lastSample[0] = 0.0f;
    voice->lastSample[1] = 0.0f;
    voice->control_sign = 0.0f;

    /* default values to avoid portamento */
    voice->cfg->port = 1.0f;
    voice->cfg->noteB = 0;

    voice->f_control_sign = 0;
    voice->f_control_sign_slow = 0;
    voice->active = true;
    voice->phase = attack;
    voice->f_phase = attack;    // trigger filter envelope
    
    /* update all values to avoid audible artifacts */
    ADSR_Process(&voice->cfg->adsr_vol, &voice->control_sign, &voice->phase);
    ADSR_Process(&voice->cfg->adsr_fil, &voice->f_control_sign, &voice->f_phase);
    
    Filter_Calculate(voice->f_control_sign_slow, voice->cfg->soundFiltReso, &voice->filterC);

    voc_act += 1;

    /*
     * add oscillator
     */
#ifdef USE_UNISON
    if (voice->cfg->unison > 0)
    {
        /*
         * shift first oscillator down
         */
        osc->addVal = midi_note_to_add[note] + ((0 - (voice->cfg->unison * 0.5)) * midi_note_to_add50c[note] * voice->cfg->detune / voice->cfg->unison);
    }
    else
#endif
    {
        osc->addVal = midi_note_to_add[note + chCfg[presetNumber].osc1transpose];
    }
    osc->samplePos = 0;
    osc->waveForm = &chCfg[presetNumber].selectedWaveForm;
    osc->dest = voice->lastSample;
    osc->pan_l = 1;
    osc->pan_r = 1;

    osc->cfg = &chCfg[presetNumber];

    osc_act += 1;

#ifdef USE_UNISON

    int8_t pan = 1;

    /*
     * attach more oscillators to voice
     */
    for (int i = 0; i < voice->cfg->unison; i++)
    {
        osc = getFreeOsc();
        if (osc == NULL)
        {
            //Serial.printf("voc: %d, osc: %d\n", voc_act, osc_act);
            return ;
        }

        osc->addVal = midi_note_to_add[note] + ((i + 1 - (voice->cfg->unison * 0.5)) * midi_note_to_add50c[note] * voice->cfg->detune / voice->cfg->unison);
        osc->samplePos = (uint32_t)random(1 << 31); /* otherwise it sounds ... bad!? */
        osc->waveForm = &chCfg[ch].selectedWaveForm2;
        osc->dest = voice->lastSample;

        /*
         * put last osc in the middle
         */
        if ((voice->cfg->unison - 1) == i)
        {
            osc->pan_l = 1;
            osc->pan_r = 1;
        }
        else if (pan == 1)
        {
            osc->pan_l = 1;
            osc->pan_r = 0.5;
        }
        else
        {
            osc->pan_l = 0.5;
            osc->pan_r = 1;
        }
        pan = -pan; /* make a stereo sound by putting the oscillator left/right */

        osc->cfg = &chCfg[ch];

        osc_act += 1;
    }
#else
    osc = getFreeOsc();
    if (osc != NULL)
    {
        if (note < 128)
        {
            if( curChCfg->osc2kbdtrk == 0 )
              osc->addVal = midi_note_to_add[ note + (int32_t)((chCfg[presetNumber].osc2transpose*24.01f)-12.0f) ];
            else  
              osc->addVal = 0;
            osc->samplePos = 0; /* we could add some offset maybe */
            osc->waveForm = &chCfg[presetNumber].selectedWaveForm2;
            osc->dest = voice->lastSample;
            osc->pan_l = 1;
            osc->pan_r = 1;

            osc->cfg = &chCfg[presetNumber];

            osc_act += 1;
        }
    }
#endif

    /*
     * trying to avoid audible suprises
     */
    Filter_Reset(&voice->filterL);
    Filter_Reset(&voice->filterR);
    Filter_Process(&voice->lastSample[0], &voice->filterL);
    Filter_Process(&voice->lastSample[0], &voice->filterL);
    Filter_Process(&voice->lastSample[0], &voice->filterL);

    Filter_Process(&voice->lastSample[1], &voice->filterR);
    Filter_Process(&voice->lastSample[1], &voice->filterR);
    Filter_Process(&voice->lastSample[1], &voice->filterR);
}

inline void Synth_NoteOff(uint8_t ch, uint8_t note)
{
    for (int j = 0; j < chCfg[ch].noteCnt; j++)
    {
        if (chCfg[ch].noteStack[j] == note)
        {
            for (int k = j; k < NOTE_STACK_MAX - 1; k++)
            {
                chCfg[ch].noteStack[k] = chCfg[ch].noteStack[k + 1];
            }
            chCfg[ch].noteCnt = (chCfg[ch].noteCnt > 0) ? (chCfg[ch].noteCnt - 1) : 0;
            Status_ValueChangedIntArr("noteCnt-", chCfg[ch].noteCnt, ch);
        }
    }

    for (int i = 0; i < MAX_POLY_VOICE ; i++)
    {
        if ((voicePlayer[i].active) && (voicePlayer[i].midiNote == note) && (voicePlayer[i].midiCh == ch))
        {
            if ((voicePlayer[i].cfg->noteCnt > 0) && (voicePlayer[i].cfg->mono))
            {
                uint8_t midiNote = voicePlayer[i].cfg->noteStack[voicePlayer[i].cfg->noteCnt - 1];

                float diff = midiNote - voicePlayer[i].midiNote;

                voicePlayer[i].cfg->noteA = voicePlayer[i].cfg->port * ((float)voicePlayer[i].cfg->noteB) + (1.0f - voicePlayer[i].cfg->port) * voicePlayer[i].cfg->noteA;
                voicePlayer[i].cfg->port = 0.0f;

                voicePlayer[i].cfg->noteB += diff;
                voicePlayer[i].midiNote = midiNote;
            }
            else
            {
                voicePlayer[i].phase = release;   
                voicePlayer[i].f_phase = release;   // put filter envelope into release phase
            }
        }
    }
}

void Synth_ModulationWheel(uint8_t ch, float value)
{
    chCfg[presetNumber].modulationDepth = value;
}

void Synth_ModulationSpeed(uint8_t ch, float value)
{
    chCfg[ch].modulationSpeed = value * 10;
    //Status_ValueChangedFloat("ModulationSpeed", modulationSpeed);
}

void Synth_ModulationPitch(uint8_t ch, float value)
{
    chCfg[ch].modulationPitch = value * 5;
    //Status_ValueChangedFloat("ModulationDepth", modulationPitch);
}

void Synth_PitchBend(uint8_t ch, float bend)
{
    chCfg[ch].pitchBendValue = bend;
    //Serial.printf("pitchBendValue: %0.3f\n", chCfg[ch].pitchBendValue);
}

void Synth_PortTime(float value)
{
    float min = 0.02f; /* 1/(0.02 * 1000) -> 0.05s */
    float max = 0.0002f; /* 1/(0.0002 * 1000) -> 5s */

    curChCfg->portAdd = (pow(2.0f, value) - 1.0f) * (max - min) + min;
}

void Synth_SetCurCh(uint8_t ch, float value)
{
    if (value > 0)
    {
        for (int i = 0; i < MAX_POLY_VOICE ; i++)
        {
          Voice_Off(i); 
        }
        presetNumber = (uint8_t)(value * 16.0f);
        curChCfg = &chCfg[presetNumber];
        Status_ValueChangedInt("Current ch", ch);
        Serial.printf("Current Ch: %d\n", presetNumber+1);
    }
}

void Synth_ToggleMono(uint8_t ch, float value)
{
    if (value > 0)
    {
        curChCfg->mono = !curChCfg->mono;
        Status_LogMessage(curChCfg->mono ? "Mono" : "Poly");
    }
}

void Synth_SetParam(uint8_t slider, float value)
{
    switch (slider)
    {
    case SYNTH_PARAM_VEL_ENV_ATTACK:
        if( value == 0 )
          value=0.0000000001f;
        value = 1.0f / (value * -50.0f);
        chCfg[presetNumber].adsr_vol.a = ( pow(2, value ) ) / 7.3055f + 0.865f;
        //Serial.printf("adsr_vol->a: %0.8f\n", curChCfg->adsr_vol.a); // Value Min -> 0.865 (aprox 3ms) // Value Max -> 0.9999865 (aprox 30s)
        break;
        
    case SYNTH_PARAM_VEL_ENV_DECAY:
        if( value == 0 )
          value=0.0000000001f;
        value = 1.0f / (value * -50.0f);
        chCfg[presetNumber].adsr_vol.d = ( pow(2, value ) ) / 7.3055f + 0.865f;
        break;
        
    case SYNTH_PARAM_VEL_ENV_SUSTAIN:
        chCfg[presetNumber].adsr_vol.s = 0.25 * (pow(5, value) - 1.0f);
        break;
        
    case SYNTH_PARAM_VEL_ENV_RELEASE:
        if( value == 0 )
          value=0.0000000001f;
        value = 1.0f / (value * -50.0f);
        chCfg[presetNumber].adsr_vol.r = ( pow(2, value ) ) / 7.3055f + 0.865f;
        break;
        
    case SYNTH_PARAM_FIL_ENV_ATTACK:
        if( value == 0 )
          value=0.0000000001f;
        value = 1.0f / (value * -50.0f);
        chCfg[presetNumber].adsr_fil.a = ( pow(2, value ) ) / 7.3055f + 0.865f; 
        break;
        
    case SYNTH_PARAM_FIL_ENV_DECAY:
        if( value == 0 )
          value=0.0000000001f;
        value = 1.0f / (value * -50.0f);
        chCfg[presetNumber].adsr_fil.d = ( pow(2, value ) ) / 7.3055f + 0.865f;
        break;
        
    case SYNTH_PARAM_FIL_ENV_SUSTAIN:
        chCfg[presetNumber].adsr_fil.s = 0.25 * (pow(5, value) - 1.0f);;
        break;
        
    case SYNTH_PARAM_FIL_ENV_RELEASE:
        if( value == 0 )
          value=0.0000000001f;
        value = 1.0f / (value * -50.0f);
        chCfg[presetNumber].adsr_fil.r = ( pow(2, value ) ) / 7.3055f + 0.865f;
        break;

#ifdef USE_UNISON
    case SYNTH_PARAM_DETUNE_1:
        curChCfg->detune = value;
        //Serial.printf("detune: %0.3f cent\n", curChCfg->detune * 50);
        break;
    case SYNTH_PARAM_UNISON_2:
        curChCfg->unison = (uint8_t)(MAX_DETUNE * value);
        //Serial.printf("unison: 1 + %d\n", curChCfg->unison);
        break;
#else
    case SYNTH_PARAM_WAVEFORM_1:
        {
            uint8_t selWaveForm = (value) * (WAVEFORM_TYPE_COUNT);
            chCfg[presetNumber].selectedWaveForm = waveFormLookUp[selWaveForm];
        }
        break;
    case SYNTH_PARAM_WAVEFORM_2:
        {
            uint8_t selWaveForm = (value) * (WAVEFORM_TYPE_COUNT);
            chCfg[presetNumber].selectedWaveForm2 = waveFormLookUp[selWaveForm];
        }
        break;
#endif
    case SYNTH_PARAM_MAIN_FILT_CUTOFF:
        filtCutoff = value;
        Filter_Calculate(filtCutoff, filtReso, &filterGlobalC);
        break;
    case SYNTH_PARAM_MAIN_FILT_RESO:
        filtReso =  0.5f + 10 * value * value * value; /* min q is 0.5 here */
        Filter_Calculate(filtCutoff, filtReso, &filterGlobalC);
        break;

    case SYNTH_PARAM_VOICE_FILT_RESO:
        chCfg[presetNumber].soundFiltReso = 0.5f + 10 * value * value * value; /* min q is 0.5 here */
        break;

    case SYNTH_PARAM_VOICE_NOISE_LEVEL:
        chCfg[presetNumber].soundNoiseLevel = value;
        break;

    case SYNTH_PARAM_VOICE_PORT_TIME:
        chCfg[presetNumber].portAdd = value * value * value * value;
        break;
//////////////////////////////////////////////////////////////////////////////////////////
    case SYNTH_PARAM_LFO_MOD_SPEED:
        chCfg[presetNumber].modulationSpeed = value * 15;
        break;
        
    case SYNTH_PARAM_OSC1_LEVEL:
        chCfg[presetNumber].osc1level = value;
        break;
        
    case SYNTH_PARAM_OSC2_LEVEL:
        chCfg[presetNumber].osc2level = value;
        break;
        
    case SYNTH_PARAM_OSC1_TRANSPOSE:
        chCfg[presetNumber].osc1transpose = (int32_t)((value*24.01f)-12.0f);
        break;
        
    case SYNTH_PARAM_OSC2_TRANSPOSE:
        chCfg[presetNumber].osc2transpose = value;
        break;

    case SYNTH_PARAM_FILTER_KBD_TRACKING:
        chCfg[presetNumber].filterKbdTrack = value;
        break;

    case SYNTH_PARAM_LFO_WAVEFORM:
        chCfg[presetNumber].lfo_waveform = value;
        break;
        
    case SYNTH_PARAM_LFO_MOD_CUTOFF:
        chCfg[presetNumber].lfo_mod_cutoff = value;   // Button
        break;

    case SYNTH_PARAM_FILTER_ENV_AMT:
        chCfg[presetNumber].f_env_amt = value;
        break;

    case SYNTH_PARAM_VOICE_FILTER_CUTOFF:
        chCfg[presetNumber].f_cutoff = value;
        break; 

    case SYNTH_PARAM_PITCH_ENV:
        chCfg[presetNumber].p_envelope = value;
        break;

    case SYNTH_PARAM_OSC2_FINE:
        chCfg[presetNumber].osc2fine = value;
        break;

    case SYNTH_PARAM_OSC2_LOW_FREQ:
        chCfg[presetNumber].osc2lowfreq = value;
        break;

    case SYNTH_PARAM_OSC2_KBD_TRACKING:
        value = value < 0.5f ? 0.0f : 1.0f;     // ON/OFF button response
        chCfg[presetNumber].osc2kbdtrk = (uint8_t)value;
        break; 

    case SYNTH_PARAM_OSC2_FILTER:
        chCfg[presetNumber].filterOsc2 = value;
        break;
        
    case SYNTH_PARAM_OSC1_FM:
        chCfg[presetNumber].osc1Fm = value;
        break;
        
    case SYNTH_PARAM_OSC1_PW:
        chCfg[presetNumber].osc1Pw = (value * 1.8f) - 0.9f; // 10% < pulse width < 90% 
        break;

    case SYNTH_PARAM_OSC2_PW:
        chCfg[presetNumber].osc2Pw = (value * 1.8f) - 0.9f; // 10% < pulse width < 90% 
        break;

    case SYNTH_PARAM_OSC1_PWM:
        chCfg[presetNumber].osc1PwmLfo = value;   // Button
        break;

    case SYNTH_PARAM_OSC2_PWM:
        chCfg[presetNumber].osc2PwmLfo = value;   // Button
        break;

    case SYNTH_PARAM_OSC1_PWM_ENV:
        chCfg[presetNumber].osc1Pw_env_mod = value;
        break; 

    case SYNTH_PARAM_LFO_SOURCE:
        chCfg[presetNumber].lfoSource = value;
        break; 
        
    case SYNTH_PARAM_GLOBAL_GAIN:
        chCfg[presetNumber].globalGain = value * 0.8f; // globalGain == 0.4f -> original Gain
        break; 
        
    case SYNTH_PARAM_OSC1_PITCH_MOD:                
        chCfg[presetNumber].osc1PitchMod = value;   // Button
        break;

    case SYNTH_PARAM_OSC2_PITCH_MOD:
        chCfg[presetNumber].osc2PitchMod = value;   // Button
        break; 
///////////////////////////////////////////////////////////////////////////////////////////
    default:
        /* not connected */
        break;
    }
}
