#ifndef KALMAN_FILTER_H_
#define KALMAN_FILTER_H_

#include "imu.h"

typedef struct {
    float angle;
    float angular_velocity;
    float estimate_uncertainty;
    float process_noise;
    float measurement_noise;
}KalmanFilter_t;

extern KalmanFilter_t kalmanFilter;

void init_kalman_filter(KalmanFilter_t *kf, float initial_measurement, float initial_uncertainty);
void predict(KalmanFilter_t *kf, float time_step);
void update(KalmanFilter_t *kf, float measured_angle, float measured_angular_velocity);

#endif /*KALMAN_FILTER_H_*/