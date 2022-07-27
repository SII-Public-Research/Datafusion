//! This is a simplified version of the Kalman filter.

use nalgebra::{Matrix2, Vector2};

#[derive(Copy, Clone, Debug)]
/// This Kalman filter is inspired from this [post](http://blog.tkjelectronics.dk/2012/09/a-practical-approach-to-kalman-filter-and-how-to-implement-it/) 
/// and is the conversion of the same C library in Rust.
pub struct Kalman {
    angle: f32, /// The angle calculated by the Kalman filter - part of the 2x1 state vector
    bias: f32, /// The gyro bias calculated by the Kalman filter - part of the 2x1 state vector
    rate: f32, /// Unbiased rate calculated from the rate and the calculated bias
    cov_error: Matrix2<f32>, /// Error covariance matrix - This is a 2x2 matrix
    k: Vector2<f32>, /// Kalman gain - This is a 2x1 vector
    y: f32, ///Angle difference
    s: f32, ///Estimate error
    q_angle: f32, /// Process noise variance for the accelerometer
    q_bias: f32, /// Process noise variance for the gyro bias
    r_measure: f32, // Measurement noise variance - this is actually the variance of the measurement noise
}

impl Kalman {
    /// Creates a new Kalman filter
    pub fn new() -> Self {
        Kalman { 
            angle: 0.0, 
            bias: 0.0, 
            rate: 0.0, 
            cov_error: Matrix2::<f32>::identity(), 
            k: Vector2::<f32>::zeros(), 
            y: 0.0, 
            s: 0.0, 
            q_angle: 0.001, 
            q_bias: 0.003, 
            r_measure: 0.03 
        }
    }
    /// The angle should be in degrees and the rate should be in degrees per second and the delta time in seconds.
    /// Returns the updated angle.
    pub fn compute_angle(&mut self, new_angle: f32, new_rate: f32, dt: f32) {
        // Discrete Kalman filter time update equations - Time Update ("Predict")
        // Update what - Project the state ahead
        /* Step 1 */
        self.rate = new_rate - self.bias;
        self.angle += dt * self.rate;
        // Update estimation error covariance - Project the error covariance ahead
        /* Step 2 */
        self.cov_error[(0, 0)] += dt * (dt * self.cov_error[(1, 1)] - self.cov_error[(0, 1)] - self.cov_error[(1, 0)] + self.q_angle);
        self.cov_error[(0, 1)] -= dt * self.cov_error[(1, 1)];
        self.cov_error[(1, 0)] -= dt * self.cov_error[(1, 1)];
        self.cov_error[(1, 1)] += self.q_bias * dt;

        // Calculate angle and bias - Update estimate with measurement zk (newAngle)
        /* Step 3 */
        self.y = new_angle - self.angle;

        // Discrete Kalman filter measurement update equations - Measurement Update ("Correct")
        // Calculate Kalman gain - Compute the Kalman gain
        /* Step 4 */
        self.s = self.cov_error[(0, 0)] + self.r_measure;

        /* Step 5 */
        self.k[0] = self.cov_error[(0, 0)] / self.s;
        self.k[1] = self.cov_error[(1, 0)] / self.s;

        // Calculate angle and bias - Update estimate with measurement zk (newAngle)
        /* Step 6 */
        self.angle += self.k[0] * self.y;
        self.bias += self.k[1] * self.y;

        // Calculate estimation error covariance - Update the error covariance
        /* Step 7 */
        self.cov_error[(0, 0)] -= self.k[0] * self.cov_error[(0, 0)];
        self.cov_error[(0, 1)] -= self.k[0] * self.cov_error[(0, 1)];
        self.cov_error[(1, 0)] -= self.k[1] * self.cov_error[(0, 0)];
        self.cov_error[(1, 1)] -= self.k[1] * self.cov_error[(0, 1)];

    }

    /// Sets the angle, this should be set as the starting angle of the filter
    pub fn set_angle(&mut self, angle: f32) {
        self.angle = angle;
    }

    /// Returns the current angle
    pub fn get_angle(&self) -> f32 {
        self.angle
    }

    /// Return the unbiased rate
    pub fn get_rate(&self) -> f32 {
        self.rate
    }

    /* The functions below are used to tune the Kalman filter */

    /// Sets the process noise variance for the accelerometer.
    pub fn set_q_angle(&mut self, q_angle: f32) {
        self.q_angle = q_angle;
    }

    /// Sets the process noise variance for the gyro bias.
    pub fn set_q_bias(&mut self, q_bias: f32) {
        self.q_bias = q_bias;
    }

    /// Sets the Measurement noise variance - this is actually the variance of the measurement noise.
    pub fn set_r_measure(&mut self, r_measure: f32) {
        self.r_measure = r_measure;
    }

    /// Returns the process noise variance for the accelerometer.
    pub fn get_q_angle(&self) -> f32 {
        self.q_angle
    }

    /// Returns the process noise variance for the gyro bias.
    pub fn get_q_bias(&self) -> f32 {
        self.q_bias
    }

    /// Returns the Measurement noise variance.
    pub fn get_r_measure(&self) -> f32 {
        self.r_measure
    }
}