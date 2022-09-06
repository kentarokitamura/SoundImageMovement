
#ifndef _Kalman_Pos_h_
#define _Kalman_Pos_h_

class Kalman_Pos {
public:
    Kalman_Pos();

    // The angle should be in degrees and the rate should be in degrees per second and the delta time in seconds
    float getPosition(float newPosition, float newRate, float dt);

    void setVelocity(float velocity); // Used to set angle, this should be set as the starting angle
    float getRate(); // Return the unbiased rate

    /* These are used to tune the Kalman filter */
    void setQvelocity(float Q_velocity);
    /**
     * setQbias(float Q_bias)
     * Default value (0.003f) is in Kalman.cpp. 
     * Raise this to follow input more closely,
     * lower this to smooth result of kalman filter.
     */
    void setQbias(float Q_bias);
    void setRmeasure(float R_measure);

    float getQvelocity();
    float getQbias();
    float getRmeasure();

private:
    /* Kalman filter variables */
    float Q_position; // Process noise variance for the accelerometer
    float Q_bias; // Process noise variance for the gyro bias
    float R_measure; // Measurement noise variance - this is actually the variance of the measurement noise

    float velocity; // The angle calculated by the Kalman filter - part of the 2x1 state vector
    float bias; // The gyro bias calculated by the Kalman filter - part of the 2x1 state vector
    float rate; // Unbiased rate calculated from the rate and the calculated bias - you have to call getAngle to update the rate
    float position;

    float P[2][2]; // Error covariance matrix - This is a 2x2 matrix
};

#endif
