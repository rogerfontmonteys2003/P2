#include <math.h>
#include "pav_analysis.h"

static inline float mean_frame(const float *x, unsigned int N) {
    float sum = 0.0;
    for (unsigned int i = 0; i < N; i++){
        sum += x[i];
    }
    return sum / N;
}

float compute_power(const float *x, unsigned int N) {
    float mu = mean_frame(x, N);
    float P = 0.0;
    for (unsigned int i = 0; i < N; i++){
        float xi = x[i] - mu;
        P += (double)xi*(double)xi;
    }
    return ((double)P/ (double)N);
}

float compute_am(const float *x, unsigned int N) {
    float mu = mean_frame(x, N);
    float A = 0.0;
    for (unsigned int i = 0; i < N; i++){
        A += fabs(x[i] - mu);
    }
    return (float)(A/(double)N);
}

float compute_zcr(const float *x, unsigned int N, float fm) {
    float mu = mean_frame(x, N);
    double A = 0.0;
    for (unsigned i = 0; i < N; ++i) A += fabsf(x[i] - mu);
    float am = (float)(A / (double)N);
    const float eps = fmaxf(1e-5f, 0.02f * am); // 2% de l’AM (típic: 1–3%)

    unsigned z = 0;
    float prev = x[0] - mu;
    for (unsigned i = 1; i < N; ++i) {
        float cur = x[i] - mu;
        int s0 = (prev >  eps) ?  1 : (prev < -eps ? -1 : 0);
        int s1 = (cur  >  eps) ?  1 : (cur  < -eps ? -1 : 0);
        if (s0 != 0 && s1 != 0 && s0 != s1) ++z;
        prev = cur;
    }
    // Normalització unitless (0..1) amb límit teòric ≈ 1
    return (float)z / (float)(N - 1);
}
