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

#include "Kalman_Pos.h"

Kalman_Pos::Kalman_Pos() {
    /* We will set the variables like so, these can also be tuned by the user */
    Q_position = 0.001f;
    Q_bias = 0.001f;
    R_measure = 0.001f;

    position = 0.0f; // Reset the angle
    bias = 0.0f; // Reset bias

    P[0][0] = 0.0f; // Since we assume that the bias is 0 and we know the starting angle (use setAngle), the error covariance matrix is set like so - see: http://en.wikipedia.org/wiki/Kalman_filter#Example_application.2C_technical
    P[0][1] = 0.0f;
    P[1][0] = 0.0f;
    P[1][1] = 0.0f;
};

// The angle should be in degrees and the rate should be in degrees per second and the delta time in seconds
float Kalman_Pos::getPosition(float newPosition, float newRate, float dt) {
    rate = newRate - bias;
    position += dt * rate;
    P[0][0] += dt * (dt * P[1][1] - P[0][1] - P[1][0] + Q_position);
    P[0][1] -= dt * P[1][1];
    P[1][0] -= dt * P[1][1];
    P[1][1] += Q_bias * dt;
    float S = P[0][0] + R_measure;
    float K[2];

    K[0] = P[0][0] / S;
    K[1] = P[1][0] / S;

    float y = newPosition - position;
    velocity += K[0] * y;
    bias += K[1] * y;

    float P00_temp = P[0][0];
    float P01_temp = P[0][1];

    P[0][0] -= K[0] * P00_temp;
    P[0][1] -= K[0] * P01_temp;
    P[1][0] -= K[1] * P00_temp;
    P[1][1] -= K[1] * P01_temp;

    return position;
};

void Kalman_Pos::setVelocity(float velocity) { this->velocity= velocity; }; // Used to set angle, this should be set as the starting angle
float Kalman_Pos::getRate() { return this->rate; }; // Return the unbiased rate

/* These are used to tune the Kalman filter */
void Kalman_Pos::setQvelocity(float Q_position) { this->Q_position = Q_position; };
void Kalman_Pos::setQbias(float Q_bias) { this->Q_bias = Q_bias; };
void Kalman_Pos::setRmeasure(float R_measure) { this->R_measure = R_measure; };

float Kalman_Pos::getQvelocity() { return this->Q_position; };
float Kalman_Pos::getQbias() { return this->Q_bias; };
float Kalman_Pos::getRmeasure() { return this->R_measure; };
