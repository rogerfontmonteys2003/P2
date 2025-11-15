#include <math.h>
#include <stdlib.h>
#include <stdio.h>

#include "pav_analysis.h"
#include "vad.h"



/*------------------------------------------------------------------------------------------*/
//CONSTANTS VAD
const float FRAME_TIME = 10.0F;               //ms
static const float SAMPLE_RATE = 16000.0f;    //Hz

const float NOISE_ALPHA_UP = 0.01f;           //Com de rapid augmenta el soroll
const float NOISE_ALPHA_DOWN = 0.03f;         //Com de rapid disminueix el soroll

const float ENTER_RATIO = 3.5f;               //Treshold to enter = noiseE * ratio
const float LEAVE_RATIO = 2.0f;               //Treshold to leave = noiseE * ratio

const float PREEMPH = 0.97f;                   // pre-èmfasi

static const float ALPHA1_DB = 7.5f;          // k1 = k0 + alpha1
static const float ALPHA2_DB = 4.0f;          // k2 = k1 + alpha2

static const unsigned HIST_WIN = 10;          // finestreta de suavitzat (majories)
static const float    MIN_V_MS = 100.0f;      // ms mínims per confirmar veu
static const float    MIN_S_MS = 160.0f;      // ms mínims per confirmar silenci

static const unsigned TMAX_MV = 1;            // frames per confirmar MAYBE_VOICE -> VOICE
static const unsigned TMAX_MS = 1;            // frames per confirmar MAYBE_SILENCE -> SILENCE

static const float AM_HIGH = 0.0035f;         //A partir d'aquest valor es considera AM alt
static const float AM_LOW  = 0.001f;          //Per sota d'aquest valor es considera AM baix

/*------------------------------------------------------------------------------------------*/

//ESTATS VAD
const char *state_str[] = {
  "UNDEF", "SILENCE", "VOICE", "INIT","MAYBE_SILENCE","MAYBE_VOICE"
};

//Retorna una cadena amb l'estat VAD
const char *state2str(VAD_STATE st) {
  return state_str[st];
}

//Estructura per guardar les característiques d'un frame
typedef struct {
  float zcr;
  float p;
  float am;
  float logp;
} Features;



//Calcul de les característiques d'un frame
static Features 
compute_features(const float *x, int N) {
  Features feat;

    float p = compute_power(x, N);

    feat.p = p;
    feat.am = compute_am(x, N);
    feat.zcr =  compute_zcr(x, N, SAMPLE_RATE);
    feat.logp = 10.0f*log10f(p + 1e-12f);
  return feat;
}

//Suavitzador que calcula dels últims HIST_WIN valors la majoria i assigna aquell valor

//Calcula quats bits a 1 hi ha a x
static int bitcount(unsigned x) {
  int c = 0;
  while (x) { 
    c += x & 1u; 
    x >>= 1; 
  }
  return c;
}

//Afegeix un nou valor is_voice (0/1) a l'historial
static void hist_push(VAD_DATA* v, int is_voice) {
  v->hist = ((v->hist << 1) | (is_voice ? 1u : 0u)) & ((1u<<HIST_WIN)-1u);
  if (v->hist_len < (int)HIST_WIN) v->hist_len++;
}

//Calcula la majoria dels últims valors a l'historial
static int hist_majority(const VAD_DATA* v) {
  int k = v->hist_len;
  if (k == 0) return 0;
  int ones = bitcount(v->hist & ((1u<<k)-1u));
  return (ones * 2 >= k) ? 1 : 0;
}

//Inicialitza l'estructura de dades per al VAD
VAD_DATA * vad_open(float rate) {
  VAD_DATA *vad_data = (VAD_DATA*)malloc(sizeof(VAD_DATA));
  
  vad_data->state = ST_INIT;
  vad_data->sampling_rate = rate;
  vad_data->frame_length = rate * FRAME_TIME * 1e-3;
  vad_data->last_feature = 0.0F;

  vad_data->noiseE = 1e-6f;
  vad_data->thr_enter = ENTER_RATIO * vad_data->noiseE;
  vad_data->thr_leave = LEAVE_RATIO * vad_data->noiseE;

  vad_data->init_cnt = 0;
  vad_data->hang_cnt = 0;
  vad_data->hist = 0;
  vad_data->hist_len = 0;

  vad_data->cur_run = 0;
  vad_data->cur_is_voice = 0;

  vad_data->min_voice_frames = (unsigned)(MIN_V_MS / FRAME_TIME);
  vad_data->min_silence_frames = (unsigned)(MIN_S_MS / FRAME_TIME);
  
  vad_data->k0_db_min = 1e9f;
  vad_data->k0_db_sum = 0.0;

  vad_data->k0_db = -1e9f;
  vad_data->k1_db = -1e9f;
  vad_data->k2_db = -1e9f;

  vad_data->ninit = 20;

  vad_data->alpha1_db = ALPHA1_DB;
  vad_data->alpha2_db = ALPHA2_DB;
  
  vad_data->tmax_mv = TMAX_MV;
  vad_data->tmax_ms = TMAX_MS;

  vad_data->zcr0_sum = 0.0f;
  vad_data->zcr0_min = 1e9f;
  vad_data->zcr0_max = -1e9f;

  vad_data->zcr_vmax = 0.20f;
  vad_data->zcr_smin = 0.30f;

  return vad_data;
}

//Retorna l'estat actual i allibera la memòria de l'estructura VAD_DATA
VAD_STATE vad_close(VAD_DATA *vad_data) {
  
  if (!vad_data) return ST_UNDEF;
  VAD_STATE state = vad_data->state;
  
  free(vad_data);
  return state;
}

//Retorna la mida del frame utilitzat pel VAD
unsigned int vad_frame_size(VAD_DATA *vad_data) {
  return vad_data->frame_length;
}

//Processa un frame i retorna l'estat VAD
VAD_STATE vad(VAD_DATA *v, float *x) {
    Features f = compute_features(x, v->frame_length);
    v->last_feature = f.p;

    static float init_logp[256];
    static float init_zcr[256];

    //Calcula per ninits frames inicials els llindars k0,k1,k2
    if (v->state == ST_INIT) {

      //Guarda ZCR i logP dels frames d'inici
      if (v->init_cnt < v->ninit) {
        init_logp[v->init_cnt] = f.logp;
        init_zcr[v->init_cnt]  = f.zcr;
        v->init_cnt++;
        return v->state;
      }

      unsigned n = v->ninit;

      //Ordena els valors de logp
      for (unsigned i = 1; i < n; ++i) {
        float t = init_logp[i]; 
        int j = (int)i - 1;
        while (j >= 0 && init_logp[j] > t) { 
          init_logp[j+1] = init_logp[j]; 
          --j; 
        }
        init_logp[j+1] = t;
      }

      //Ordena els valors de zcr
      for (unsigned i = 1; i < n; ++i) {
        float t = init_zcr[i]; 
        int j = (int)i - 1;
        while (j >= 0 && init_zcr[j] > t) { 
          init_zcr[j+1] = init_zcr[j]; 
          --j; 
        }
        init_zcr[j+1] = t;
      }

      //Calcula k com el 35% dels frames d'inici o mínim 5
      unsigned k = (unsigned)(0.35f * n);
      if (k < 5) k = (n < 5 ? n : 5);
      double sum_db = 0.0, sum_z_hi = 0.0, sum_z_lo = 0.0;

      //Calcula la mitjana dels k valors més baixos de logp i ZCR
      for (unsigned i = 0; i < k; ++i) { 
        sum_db += init_logp[i]; 
        sum_z_hi += init_zcr[i];
        sum_z_lo += init_zcr[n - 1 - i]; 
      }

      float mean_db = (float)(sum_db / (double)k);
      float zcr0_lo = (float)(sum_z_lo  / (double)k);
      float zcr0_hi = (float)(sum_z_hi  / (double)k);

      v->k0_db = 0.6f * mean_db + 0.4f * init_logp[0]; 
      v->k1_db = v->k0_db + v->alpha1_db;
      v->k2_db = v->k1_db + v->alpha2_db;

      //Calcula llindars ZCR
      float zcr_vmax = fminf(0.20f, fmaxf(0.06f, 0.80f * zcr0_lo));
      float zcr_smin = fmaxf(0.30f, fminf(0.50f, 0.80f * zcr0_hi));

      v->zcr_vmax = zcr_vmax;
      v->zcr_smin = zcr_smin;

      v->state    = ST_MAYBE_SILENCE;
      v->hang_cnt = 0;
      v->hist = 0;
      v->hist_len = 0;
      v->cur_is_voice = 0;
      v->cur_run = 0;
    }

    //Determina si el frame és veu o silenci segons energia, ZCR i AM
    int v_by_energy = (f.logp > v->k2_db);             // veu clara per energia
    int s_by_energy = (f.logp < (v->k1_db - 1.2f));    // silenci clar amb marge

    int zcr_voice = (f.zcr < v->zcr_vmax);              // veu per ZCR
    int zcr_silence = (f.zcr > v->zcr_smin);            // silenci per ZCR

    int am_high = (f.am > AM_HIGH);                     // veu per AM
    int am_low  = (f.am < AM_LOW);                      // silenci per AM

    int raw_voice;

    if (v_by_energy) raw_voice = 1;
    else if (s_by_energy) raw_voice = 0;
    else {
      // Zona grisa: confirma amb ZCR/AM o manté l’estat per estabilitat 
      int pass_voice = ((f.logp > v->k1_db) && (zcr_voice || am_high)); //pot ser veu ja que te ZCR baix o AM alt
      int pass_sil   = (zcr_silence && ((f.am < AM_LOW && f.logp < v->k1_db) || (f.logp < v->k1_db))); //pot ser silenci ja que te ZCR alt i AM baix
      raw_voice = pass_voice ? 1 : (pass_sil ? 0 : v->cur_is_voice); 
    }

    //Suavitzat fent que fins que no tinguem un seguit de frames iguals no canviem d’estat
    hist_push(v, raw_voice);
    int smooth_voice = hist_majority(v);

    if (smooth_voice == v->cur_is_voice) {  //mateix estat que abans
      v->cur_run++;
    } else {
      unsigned need = v->cur_is_voice ? v->min_voice_frames : v->min_silence_frames; //frames necessaris per canviar d’estat

      if (v->cur_run >= need) {   //canvia d'estat si s'ha complert el mínim
        v->cur_is_voice = smooth_voice;
        v->cur_run = 1;

      } else {  //manté l'estat anterior si no s'ha complert el mínim
        smooth_voice = v->cur_is_voice;
        v->cur_run++;
      }
    }

    //Actualitza k0,k1,k2 segons el silenci detectat
    if (v->state == ST_SILENCE && v->cur_run >= v->min_silence_frames) {
    float x = f.logp;
    float a = (f.logp < v->k0_db ? 0.4f : 0.01f); //més ràpid si el nivell és més baix que k0

    v->k0_db = (1.0f - a) * v->k0_db + a * x;
    v->k1_db = v->k0_db + v->alpha1_db;
    v->k2_db = v->k1_db + v->alpha2_db;
    }

    //Canvia l'estat segons les decisions anteriors

    //Si hi ha veu per energia o silenci per energia, ZCR o AM, es mou cap a aquell estat
    int toward_silence = (s_by_energy || (zcr_silence && am_low));
    int toward_voice = (v_by_energy || raw_voice || smooth_voice);

    //Si sobrepassa molt els llindars es fa una decisió més definitiva
    int hard_voice = ((f.logp > v->k2_db));
    int hard_silence = ((f.logp < v->k1_db));

    //INICIO MÀQUINA D'ESTATS VAD
    switch (v->state) {

        case ST_INIT:
            default:
            v->state = ST_INIT;
            v->hang_cnt = 0;
            break;
        
        
        case ST_UNDEF:
          v->state = ST_MAYBE_SILENCE; 
          v->hang_cnt = 0; 
          break;

        case ST_MAYBE_SILENCE:
          if (toward_silence) { //No pot confirmar silenci encara
            unsigned need = hard_silence ? 1u : v->tmax_ms;
            if (++v->hang_cnt >= need) { //Afageix contador per confirmar silenci
              v->state = ST_SILENCE; 
              v->hang_cnt = 0; 
            }
          } else if (toward_voice) { //Detecta una possible veu
              unsigned need = hard_voice ? 1u : v->tmax_mv;
              if (++v->hang_cnt >= need) { //Afageix contador per confirmar veu
                v->state = ST_VOICE; 
                v->hang_cnt = 0;
              }
          } else{ //Si no hi ha ni veu ni silenci, manté l'estat i reinicia el contador
            v->hang_cnt = 0;
          }
          break;

        case ST_SILENCE:
          if (toward_voice) { //Si detecta veu lleu, canvia a MAYBE_VOICE
            v->state = ST_MAYBE_VOICE; 
            v->hang_cnt = 0;
          } else{ //Si no hi ha veu, manté silenci
            v->state = ST_SILENCE; 
            v->hang_cnt = 0;
          }
          break;

        case ST_MAYBE_VOICE:
          if (toward_voice) { //No pot confirmar veu encara
            unsigned need = hard_voice ? 1u : v->tmax_mv;
            if (++v->hang_cnt >= need){ //Afageix contador per confirmar veu
              v->state = ST_VOICE; 
              v->hang_cnt = 0;
            }
          } else if (toward_silence) { //No pot confirmar silenci encara
            unsigned need = hard_silence ? 1u : v->tmax_ms;
            if (++v->hang_cnt >= need) { //Afageix contador per confirmar silenci
              v->state = ST_SILENCE; 
              v->hang_cnt = 0; 
            }
          } else { //Si no hi ha ni veu ni silenci, manté l'estat i reinicia el contador
            v->hang_cnt = 0;
          }
          break;

        case ST_VOICE:
          if (toward_silence) { //Si detecta silenci lleu, canvia a MAYBE_SILENCE
            v->state = ST_MAYBE_SILENCE; 
            v->hang_cnt = 0;
          } else{ //Si no hi ha silenci, manté veu
            v->state = ST_VOICE; 
            v->hang_cnt = 0;
          }
          break;
    }
  return v->state;
}

void vad__show_state(const VAD_DATA *vad_data, FILE *out) {
  static int initialized = 0;
  static VAD_STATE last = ST_UNDEF;
  static float t = 0.0f;
  static float seg_start = 0.0f;
  static float frame_sec = 0.0f;

  //Finalitza l'ultim segment i en cas de que hi hagi un estat MAYBE n'assigna el mes proper
  if(vad_data == NULL){
    if(initialized){
      VAD_STATE outlast = (last == ST_VOICE || last == ST_MAYBE_VOICE)? ST_VOICE : ST_SILENCE;
      fprintf(out, "%.3f %.3f %c\n",seg_start, t, (outlast == ST_VOICE ? 'V' : 'S'));
    }
    initialized = 0;
    last = ST_UNDEF;
    t = 0.0f;
    seg_start = 0.0f;
    frame_sec = 0.0f;
    return;
  }

  VAD_STATE cur = vad_data->state;
  VAD_STATE mapped = (cur == ST_VOICE || cur == ST_MAYBE_VOICE)? ST_VOICE : ST_SILENCE;
  //Si és el primer frame, inicialitza les variables
  if (!initialized) {
    initialized = 1;
    frame_sec = (float)vad_data->frame_length / vad_data->sampling_rate;
    seg_start = 0.0f;
    t = 0.0f;
    last = mapped;
  } else if (mapped != last) {
    VAD_STATE outlast = (last == ST_VOICE) ? ST_VOICE : ST_SILENCE;
    fprintf(out, "%.3f %.3f %c\n",seg_start, t, (outlast == ST_VOICE ? 'V' : 'S'));
    seg_start = t;
    last = mapped;
  }
  t += frame_sec;
}

