#include "kalman_filter.h"

KalmanFilter_t kalmanFilter = {

    .estimate_uncertainty = 0.0f,
    .measurement_noise = 0.0f,
};

void init_kalman_filter(KalmanFilter_t *kf, float initial_measurement, float initial_uncertainty){

    kf->angle = initial_measurement;
    kf->angular_velocity = 0.0f;
    kf->estimate_uncertainty = initial_uncertainty;
    kf->process_noise = 0.01f;
    kf->measurement_noise = 0.1f;
}

void predict(KalmanFilter_t *kf, float time_step){

    // predict the state
    kf->angle += kf->angular_velocity * time_step;

    // update uncertainty 
    kf->estimate_uncertainty += kf->process_noise;
}

void update(KalmanFilter_t *kf, float measured_angle, float measured_angular_velocity){

    float kalman_gain = kf->estimate_uncertainty / (kf->estimate_uncertainty + kf->measurement_noise);

    // update the state estimate 
    float angle_diffrence = measured_angle - kf->angle;
    float angular_velocity_difference = measured_angular_velocity - kf->angular_velocity;

    kf->angle += kalman_gain * angle_diffrence;
    kf->angular_velocity += kalman_gain * angular_velocity_difference;

    // update the uncertainty
    kf->estimate_uncertainty = (1 - kalman_gain) * kf->estimate_uncertainty;


}