#ifndef KALMAN_FILTER_H
#define KALMAN_FILTER_H
#include <iostream>

class Kalman {
public:
    Kalman();
    // The angle should be in degrees and the rate should be in degrees per second and the delta time in seconds
    /**
     * @brief getAngle
     * @param newAngle --> obsereved from Accelerometer
     * @param newRate --> rad/s or degree/s
     * @param dt
     * @return
     */
    double getAngle(double newAngle, double newRate, double dt);
    void setAngle(double newAngle) { angle = newAngle; } // Used to set angle, this should be set as the starting angle
    double getRate() { return rate; } // Return the unbiased rate

    /* These are used to tune the Kalman filter */
    void setQangle(double newQ_angle) { Q_angle = newQ_angle; }
    void setQbias(double newQ_bias) { Q_bias = newQ_bias; }
    void setRmeasure(double newR_measure) { R_measure = newR_measure; }

    double getQangle() { return Q_angle; }
    double getQbias() { return Q_bias; }
    double getRmeasure() { return R_measure; }

private:
    /* Kalman filter variables */
    double Q_angle; // Process noise variance for the accelerometer
    double Q_bias; // Process noise variance for the gyro bias
    double R_measure; // Measurement noise variance - this is actually the variance of the measurement noise

    double angle; // The angle calculated by the Kalman filter - part of the 2x1 state vector
    double bias; // The gyro bias calculated by the Kalman filter - part of the 2x1 state vector
    double rate; // Unbiased rate calculated from the rate and the calculated bias - you have to call getAngle to update the rate

    double P[2][2]; // Error covariance matrix - This is a 2x2 matrix
    double K[2]; // Kalman gain - This is a 2x1 vector
    double y; // Angle difference
    double S; // Estimate error
};


#endif // KALMAN_FILTER_H
