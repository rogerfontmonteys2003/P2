#include <stdio.h>
#include <stdlib.h>
#include <errno.h>
#include <sndfile.h>

#include "vad.h"
#include "vad_docopt.h"

#define DEBUG_VAD 0x1

static inline char sv_label(VAD_STATE st) {
    return (st == ST_VOICE || st == ST_MAYBE_VOICE) ? 'V' : 'S';
}

typedef struct {
    int initialized;
    char last_label;     
    double seg_start;    
    double t;            
    double frame_sec;    
} VadPrinter;

static void vp_init(VadPrinter* p, unsigned frame_size, int samplerate) {
    p->initialized = 0;
    p->last_label  = 'S';
    p->seg_start   = 0.0;
    p->t           = 0.0;
    p->frame_sec   = (double)frame_size / (double)samplerate;
}

static void vp_feed(VadPrinter* p, VAD_STATE st, FILE* out) {
    char cur = sv_label(st);
    if (!p->initialized) {
        p->initialized = 1;
        p->last_label  = cur;
        p->seg_start   = 0.0;
    } else if (cur != p->last_label) {
        fprintf(out, "%.5f\t%.5f\t%c\n", p->seg_start, p->t, p->last_label);
        p->seg_start = p->t;
        p->last_label = cur;
    }
    p->t += p->frame_sec;
}

static void vp_close(VadPrinter* p, FILE* out) {
    if (!p->initialized) return;
    fprintf(out, "%.5f\t%.5f\t%c\n", p->seg_start, p->t, p->last_label);
    p->initialized = 0; /* reset opcional per a pròxim fitxer */
}
/* --- Fi printer local --- */


int main(int argc, char *argv[]) {
  int verbose = 0; /* To show internal state of vad: verbose = DEBUG_VAD; */

  SNDFILE *sndfile_in, *sndfile_out = 0;
  SF_INFO sf_info;
  FILE *vadfile;
  int n_read = 0, i;

  VAD_DATA *vad_data;
  VAD_STATE state;

  float *buffer, *buffer_zeros;
  int frame_size;         /* in samples */

  char *input_wav, *output_vad, *output_wav;

  DocoptArgs args = docopt(argc, argv, /* help */ 1, /* version */ "2.0");

  verbose    = args.verbose ? DEBUG_VAD : 0;
  input_wav  = args.input_wav;
  output_vad = args.output_vad;
  output_wav = args.output_wav;

  if (input_wav == 0 || output_vad == 0) {
    fprintf(stderr, "%s\n", args.usage_pattern);
    return -1;
  }

  /* Open input sound file */
  if ((sndfile_in = sf_open(input_wav, SFM_READ, &sf_info)) == 0) {
    fprintf(stderr, "Error opening input file %s (%s)\n", input_wav, strerror(errno));
    return -1;
  }

  if (sf_info.channels != 1) {
    fprintf(stderr, "Error: the input file has to be mono: %s\n", input_wav);
    return -2;
  }

  /* Open vad file */
  if ((vadfile = fopen(output_vad, "wt")) == 0) {
    fprintf(stderr, "Error opening output vad file %s (%s)\n", output_vad, strerror(errno));
    return -1;
  }

  /* Open output sound file, with same format, channels, etc. than input */
  if (output_wav) {
    if ((sndfile_out = sf_open(output_wav, SFM_WRITE, &sf_info)) == 0) {
      fprintf(stderr, "Error opening output wav file %s (%s)\n", output_wav, strerror(errno));
      return -1;
    }
  }

  vad_data = vad_open(sf_info.samplerate);

  /* Allocate memory for buffers */
  frame_size   = vad_frame_size(vad_data);
  buffer       = (float *) malloc(frame_size * sizeof(float));
  buffer_zeros = (float *) malloc(frame_size * sizeof(float));
  for (i = 0; i < frame_size; ++i) buffer_zeros[i] = 0.0F;

  /* Printer d’intervals */
  VadPrinter pr;
  vp_init(&pr, (unsigned)frame_size, sf_info.samplerate);

  /* PROCÉS PER FRAME */
  for (;;) {
    n_read = sf_read_float(sndfile_in, buffer, frame_size);
    if (n_read != frame_size) break;   // <-- (arreglat) només una lectura

    state = vad(vad_data, buffer);

    // DEBUG (opcional): mostra S/V per frame per pantalla
    if (verbose & DEBUG_VAD) {
        fputc(sv_label(state), stdout);
        fputc('\n', stdout);
    }

    // Escriu intervals cap al .vad
    vp_feed(&pr, state, vadfile);

    // OPCIONAL: escriu WAV amb zeros en silenci
    if (sndfile_out != 0) {
        int is_voice = (state == ST_VOICE || state == ST_MAYBE_VOICE);
        sf_write_float(sndfile_out, is_voice ? buffer : buffer_zeros, n_read);
    }
  }

  // Tanca l’últim interval
  vp_close(&pr, vadfile);

  vad_close(vad_data);

  /* clean up: free memory, close open files */
  free(buffer);
  free(buffer_zeros);
  sf_close(sndfile_in);
  fclose(vadfile);
  if (sndfile_out) sf_close(sndfile_out);
  return 0;
}