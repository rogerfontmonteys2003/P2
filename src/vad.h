#ifndef _VAD_H
#define _VAD_H
#include <stdio.h>

/* TODO: add the needed states */
typedef enum {ST_UNDEF=0, ST_SILENCE, ST_VOICE, ST_INIT, ST_MAYBE_SILENCE, ST_MAYBE_VOICE} VAD_STATE;

/* Return a string label associated to each state */
const char *state2str(VAD_STATE st);

/* TODO: add the variables needed to control the VAD 
   (counts, thresholds, etc.) */

typedef struct {
  VAD_STATE state;
  float sampling_rate;
  unsigned int frame_length;
  float last_feature; /* for debuggin purposes */

  float noiseE; //Energy of the noise
  float thr_enter; //Treshold to enter in voice state
  float thr_leave; //Treshold to leave the voice state
  unsigned hang_cnt; 

  unsigned char hist; //record of 5 bits
  int hist_len; //How many bits are full (<=5)

   unsigned min_voice_frames; 
   unsigned min_silence_frames;
   unsigned cur_run;
   int cur_is_voice; //1 if current run is voice, 0 if silence 

   float k0_db;             // nivell (soroll) en dB
   float k1_db;             // llindar "pot ser veu"
   float k2_db;             // confirmació de veu

   int   ninit;             // #frames inicials per estimar k0
   int   init_cnt;          // comptador d'aquesta fase

   float alpha1_db;         // k1 = k0 + alpha1 (p.ex. 6 dB)
   float alpha2_db;         // k2 = k1 + alpha2 (p.ex. 4 dB)

   unsigned tmax_mv;        // màxim frames en MAYBE_VOICE
   unsigned tmax_ms;        // màxim frames en MAYBE_SILENCE

   float k0_db_min;     // mínim observat durant inici
   double k0_db_sum;    // suma per fer mitjana
   
   float zcr0_sum, zcr0_min, zcr0_max;
   float zcr_vmax, zcr_smin;


} VAD_DATA;

const char* state2str(VAD_STATE st);

/* Call this function before using VAD: 
   It should return allocated and initialized values of vad_data

   sampling_rate: ... the sampling rate */
VAD_DATA *vad_open(float sampling_rate);

/* vad works frame by frame.
   This function returns the frame size so that the program knows how
   many samples have to be provided */
unsigned int vad_frame_size(VAD_DATA *);

/* Main function. For each 'time', compute the new state 
   It returns:
    ST_UNDEF   (0) : undefined; it needs more frames to take decission
    ST_SILENCE (1) : silence
    ST_VOICE   (2) : voice

    x: input frame
       It is assumed the length is frame_length */
VAD_STATE vad(VAD_DATA *vad_data, float *x);

/* Free memory
   Returns the state of the last (undecided) states. */
VAD_STATE vad_close(VAD_DATA *vad_data);

/* Print actual state of vad, for debug purposes */
//void vad_show_state(const VAD_DATA *, FILE *);

void vad_show_state(const VAD_DATA *vad_data, FILE *out);


#endif
