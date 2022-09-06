/* Copyright (C) 2012 Kristian Lauszus, TKJ Electronics. All rights reserved.


 This software may be distributed and modified under the terms of the GNU
 General Public License version 2 (GPL2) as published by the Free Software
 Foundation and appearing in the file GPL2.TXT included in the packaging of
 this file. Please note that GPL2 Section 2[b] requires that all works based
 on this software must also be made publicly available under the terms of
 the GPL2 ("Copyleft").

 Contact information
 -------------------

 Kristian Lauszus, TKJ Electronics
 Web      :  http://www.tkjelectronics.com
 e-mail   :  kristianl@tkjelectronics.com
 */

#include "Kalman_Vel.h"

Kalman_Vel::Kalman_Vel() {
    /* We will set the variables like so, these can also be tuned by the user */
    Q_velocity = 0.5f;
    Q_bias = 0.003f;
    R_measure = 0.03f;

    velocity = 0.0f; // Reset the angle
    bias = 0.0f; // Reset bias

    P[0][0] = 0.0f; // Since we assume that the bias is 0 and we know the starting angle (use setAngle), the error covariance matrix is set like so - see: http://en.wikipedia.org/wiki/Kalman_filter#Example_application.2C_technical
    P[0][1] = 0.0f;
    P[1][0] = 0.0f;
    P[1][1] = 0.0f;
};

// The angle should be in degrees and the rate should be in degrees per second and the delta time in seconds
float Kalman_Vel::getVelocity(float newVelocity, float newRate, float dt) {
    rate = newRate - bias;
    velocity += dt * rate;
    P[0][0] += dt * (dt * P[1][1] - P[0][1] - P[1][0] + Q_velocity);
    P[0][1] -= dt * P[1][1];
    P[1][0] -= dt * P[1][1];
    P[1][1] += Q_bias * dt;
    float S = P[0][0] + R_measure;
    float K[2];

    K[0] = P[0][0] / S;
    K[1] = P[1][0] / S;

    float y = newVelocity - velocity;
    velocity += K[0] * y;
    bias += K[1] * y;

    float P00_temp = P[0][0];
    float P01_temp = P[0][1];

    P[0][0] -= K[0] * P00_temp;
    P[0][1] -= K[0] * P01_temp;
    P[1][0] -= K[1] * P00_temp;
    P[1][1] -= K[1] * P01_temp;

    return velocity;
};

void Kalman_Vel::setVelocity(float velocity) { this->velocity= velocity; }; // Used to set angle, this should be set as the starting angle
float Kalman_Vel::getRate() { return this->rate; }; // Return the unbiased rate

/* These are used to tune the Kalman filter */
void Kalman_Vel::setQvelocity(float Q_velocity) { this->Q_velocity = Q_velocity; };
void Kalman_Vel::setQbias(float Q_bias) { this->Q_bias = Q_bias; };
void Kalman_Vel::setRmeasure(float R_measure) { this->R_measure = R_measure; };

float Kalman_Vel::getQvelocity() { return this->Q_velocity; };
float Kalman_Vel::getQbias() { return this->Q_bias; };
float Kalman_Vel::getRmeasure() { return this->R_measure; };
