//! # Datafusion library for 6 and 9 degrees of freedom sensors.
//! 
//! [![made-with-rust](https://img.shields.io/badge/Made%20with-Rust-1f425f?style=plastic)](https://www.rust-lang.org/)
//! [![powered-by-sii](https://img.shields.io/badge/Powered%20By-SII-blue?style=plastic)](https://sii-group.com/fr-FR/sii-sud-ouest)
//! 
//! You have the choice in this library to apply the filters on two types of sensors:
//! - A 6 degrees of freedom sensor with a 3-axis accelerometer and 3-axis gyroscope -> Mode `Dof6`
//! - A 9 degrees of freedom sensor with a 3-axis accelerometer, 3-axis gyroscope and 3-axis magnetometer -> Mode `Dof9`
//!  
//! > 🔴 ***For now, the sensor needs to be perfectly flat to work. Also, using angle and distance measurement at the same time is not recommended with a 6Dof sensor.***  
//! > More specifically, for the distance it doesn't really matter as the high pass filter will automatically delete the angle offset after a few second.  
//! > However, for the angle (except with a magnetometer) the sensor has to be flat or at least stay within the same inclination on the X and Y-axis after initialization.  
//! 
//! In the example provided below, we use our own driver for an Adafruit sensor. So of course, you can use any other driver you want.
//! 
//! ### Angle data
//! When the sensor is flat and whatever the mode used, X and Y angles are absolute roll and pitch values in degrees.
//! A Kalman filter is applied between acceleration and angular speed to produce a reliable output.
//! While using a Dof6 sensor, e.g without a magnetometer, the Kalman filter output on the the Z or Yaw-axis is an angular speed in degrees per second.
//! Which is then integrated to produce the absolute angle in degrees. But this method is not really reliable as it heavily rely on which speed the sensor is moving.
//! However, when using a Dof9 sensor, the Kalman filter output on the Z or Yaw-axis is an absolute angle in degrees. This is the preferred method.
//! As long as there is no magnetic interference, you we will be able a obtain a heading angle down to a 2 to 3 degree accuracy.
//! In the case of magnetic interferences, if it is much greater than the earth magnetic field, then the output won't be the magnetic north but a relative angle to the magnetic interference.
//! 
//! ### Distance data
//! For the distance data, the algorithm is the same whatever the mode used as it only uses acceleration data on X and Y.
//! It is a succession of high pass filters, low pass filters and integrations. 
//! It works perfectly fine on a short distance, e.g < 5cm, and it extremely accurate in this range. 
//! However, over this range the result highly rely on the speed. 
//! This means that, if the speed is too low, the distance will be under estimated. 
//! Same problem if the speed is too high, the distance will be over estimated. 
//! This problem is currently being worked on and the lib will be updated if a workaround is found. 
//! 
//! You have the ability to disable the distance measurement by calling the `disable_distance` function.
//! 
//! ***Please note that distances measurements are purely experimental but it's a place to start as the accuracy is not optimal.***
//! 
//! ### Usage
//! The example below uses [rppal](https://crates.io/crates/rppal) crates and runs on a Raspberry Pi.
//! 
//! Please note that this example and library have been developed in parallel with a driver for a Dof9 sensor from Adafruit.
//! 
//! ```no_run
//! use std::time::Instant;
//! 
//! use rppal::hal::Delay;
//! use rppal::i2c::I2c;
//! 
//! use embedded_hal::blocking::delay::*;
//! 
//! use adafruit::*;
//! use datafusion::{self as _, Fusion};
//! 
//! fn main() -> Result<(), SensorError<rppal::i2c::Error>> {
//! 
//!     // Init a delay used in certain functions and between each loop.
//!     let mut delay = Delay::new();
//! 
//!     // Setup the raspberry's I2C interface to create the sensor.
//!     let i2c = I2c::new().unwrap();
//! 
//!     // Create an Adafruit object
//!     let mut sensor = AdafruitNXP::new(0x8700A, 0x8700B, 0x0021002C, i2c);
//! 
//!     // Check if the sensor is ready to go
//!     let ready = sensor.begin()?;
//!     if !ready {
//!         std::eprintln!("Sensor not detected, check your wiring!");
//!         std::process::exit(1);
//!     }
//! 
//!     sensor.set_accel_range(config::AccelMagRange::Range8g)?;
//!     sensor.set_gyro_range(config::GyroRange::Range500dps)?;
//!     // etc...
//! 
//!     // Initialize the sensor
//!     sensor.read_data()?;
//! 
//!     let acc_x = sensor.accel_sensor.get_scaled_x();
//!     let acc_y = sensor.accel_sensor.get_scaled_y();
//!     let acc_z = sensor.accel_sensor.get_scaled_z();
//! 
//!     let gyro_x = sensor.gyro_sensor.get_scaled_x();
//!     let gyro_y = sensor.gyro_sensor.get_scaled_y();
//!     let gyro_z = sensor.gyro_sensor.get_scaled_z();
//! 
//!     let mag_rx = sensor.mag_sensor.get_scaled_x();
//!     let mag_ry = sensor.mag_sensor.get_scaled_y();
//!     let mag_rz = sensor.mag_sensor.get_scaled_z();
//! 
//!     // Create a datafusion object
//!     let mut fusion = Fusion::new(0.05, 20., 50);
//!     fusion.set_mode(datafusion::Mode::Dof9);
//! 
//!     // Set data to the fusion object
//!     fusion.set_data_dof9(acc_x, acc_y, acc_z, gyro_x, gyro_y, gyro_z, mag_rx, mag_ry, mag_rz);
//! 
//!     // Initialize the datafusion object
//!     fusion.init();
//! 
//!     // Set magnetic declination --> 1.39951° in Toulouse, France
//!     fusion.set_declination(1.39951);
//! 
//!     // Setting up the delta time
//!     let mut time = Instant::now();
//! 
//!     loop {
//! 
//!         // Calculate delta time in seconds
//!         let dt = time.elapsed().as_micros() as f32 / 1_000_000.;
//!         time = Instant::now();
//! 
//!         // Update old values for the next loop
//!         fusion.set_old_values(acc_x, acc_y);
//! 
//!         sensor.read_data()?;
//! 
//!         let acc_x = sensor.accel_sensor.get_scaled_x();
//!         let acc_y = sensor.accel_sensor.get_scaled_y();
//!         let acc_z = sensor.accel_sensor.get_scaled_z();
//! 
//!         let gyro_x = sensor.gyro_sensor.get_scaled_x();
//!         let gyro_y = sensor.gyro_sensor.get_scaled_y();
//!         let gyro_z = sensor.gyro_sensor.get_scaled_z();
//! 
//!         let mag_rx = sensor.mag_sensor.get_scaled_x();
//!         let mag_ry = sensor.mag_sensor.get_scaled_y();
//!         let mag_rz = sensor.mag_sensor.get_scaled_z();
//! 
//!         // Set data to the fusion object
//!         fusion.set_data_dof9(acc_x, acc_y, acc_z, gyro_x, gyro_y, gyro_z, mag_rx, mag_ry, mag_rz);
//! 
//!         // Perform a step of the algorithm
//!         fusion.step(dt);
//! 
//!         // Collect outputs
//!         let angle_x = fusion.get_x_angle();
//!         let angle_y = fusion.get_y_angle();
//!         let angle_z = fusion.get_heading();
//!         let distance = fusion.get_final_distance();
//! 
//!         // Print data
//!         std::println!("Angle X: {} °", angle_x);
//!         std::println!("Angle Y: {} °", angle_y);
//!         std::println!("Angle Z: {} °", angle_z);
//!         std::println!("Total distance traveled: {} cm", distance);
//! 
//!         delay.delay_ms(5_u8);
//!     }
//! }
//! ```

#![no_std]
#![deny(missing_docs)]

#[macro_use]
extern crate alloc;

pub mod kalman;
pub mod filters;
use crate::kalman::Kalman;
use crate::filters::{Smooth, HPFilter, LPFilter};

#[allow(unused_imports)]
use micromath::F32Ext;

/// Gravity constant.
pub const G: f32 = 9.80665;
/// Squared G.
pub const G2: f32 = G * G;
/// PI. 3.14159274_f32
pub const PI: f32 = core::f32::consts::PI;
/// Constant to convert radians to degrees.
pub const RAD2DEG: f32 = 180.0 / PI;
/// Constant to convert degrees to radians.
pub const DEG2RAD: f32 = PI / 180.0;
/// Square root of g.
pub const SQRT_G: f32 = 3.13155712;
/// Tweaked square root of g. Used to correct the output value.
pub const SQRT_G_KALMAN: f32 = SQRT_G * 3.0 / 4.0;
/// Converts g to a scale value, here in squared centimeters.
pub const G_TO_SCALE: f32 = 100.0;

#[derive(Debug, PartialEq, Clone, Copy)]
/// #### Enum used to define the mode of the datafusion algorithm.
/// Use ***Dof9*** for 9 degrees of freedom ie, *Accelerometer, Gyroscope & Magnetometer*.
/// Use ***Dof6*** for 6 degrees of freedom ie, *Accelerometer & Gyroscope*.
pub enum Mode {
    /// ***9*** degrees of freedom ie, *Accelerometer, Gyroscope & Magnetometer*.
    Dof6,
    /// ***6*** degrees of freedom ie, *Accelerometer & Gyroscope*.
    Dof9
}

/// #### Datafusion structure to compute distance traveled and angle.
/// 
/// This will allow you to: 
/// - Chose which degrees of freedom to use.
/// - Performs a Kalman filter:
///     - Angle for X and Y-axis
///     - Angle for X-Axis (9Dof) or Angular velocity for Z-Axis (6Dof and 9Dof)
/// - Get the Z-axis angle traveled (6Dof)
/// - Perform a series of filters X and Y-axis to:
///     - Get the velocity on each axis
///     - Get the distance traveled
pub struct Fusion {
    //Generic Variables
    acc_x: f32,
    acc_y: f32,
    acc_z: f32,
    gyro_x: f32,
    gyro_y: f32,
    gyro_z: f32,
    mag_rx: f32,
    mag_ry: f32,
    mag_rz: f32,
    mag_x: f32,
    mag_y: f32,
    roll: f32,
    pitch: f32,
    yaw: f32,
    heading: f32,
    /// Degrees of freedom.
    mode: Mode,
    //Kalman Variables
    /// Final X-Axis angle.
    kalman_x_angle: f32,
    /// Final Y-Axis angle.
    kalman_y_angle: f32,
    /// Intermediate Z-Axis angle. When don't have a magnetometer, this is actually the angular velocity around the Z-axis.
    /// When using a magnetometer, this is the angle between the magnetic field and the Z-axis.
    kalman_z_angle: f32,
    /// Kalman filter used to correct the output value on the X-axis.
    kalman_x: Kalman,
    /// Kalman filter used to correct the output value on the Y-axis.
    kalman_y: Kalman,
    /// Kalman filter used to correct the output value on the Z-axis.
    kalman_z: Kalman,
    /// Moving average filter used to smooth the output value of the heading.
    ma_yaw: Smooth,
    /// Corrects the final Z-axis angle. (6Dof only)
    offset_kalman: f32,
    /// In which position the point is relatively to the abscissa. -1 is left, 0 is neutral, 1 is right. (6Dof only)
    sens: i8,
    /// Last recorded sens. (6Dof only)
    sens_prec: i8,
    /// How many times the curve has crossed the abscissa. (6Dof only)
    sens_count: i32,
    /// Final Z-Axis angle. When we don't have a magnetometer, this is actually the angular velocity around the Z-axis.
    /// When using a magnetometer, this variable is always 0.0. (6Dof only)
    total_z_angle: f32,
    /// Final angular velocity around the Z-axis. Only use this value when we use a magnetometer. (9Dof only)
    angular_vel_z: f32,
    /// Magnetic declination. (9Dof only)
    declination: f32,
    //Distance Variables
    /// Enable or disable the distance computation.
    enable_distance: bool,
    //X-AXIS
    //SPEED
    /// High pass filter for the speed of the X-axis.
    hp_vx: HPFilter,
    /// High pass filter entry for the speed of the X-axis.
    hp_e_vx: f32,
    /// Old high pass filter entry for the speed of the X-axis.
    old_hp_e_vx: f32,
    /// High pass filter output for the speed of the X-axis.
    hp_s_vx: f32,
    /// Speed on the X-axis.
    speed_x: f32,
    /// Distance on the X-axis.
    distance_x: f32,
    /// Low pass filter for the speed of the X-axis.
    lp_vx: LPFilter,
    /// Low pass filter entry for the speed of the X-axis.
    lp_e_vx: f32,
    /// Low pass filter output for the speed of the X-axis.
    lp_s_vx: f32,
    //DISTANCE
    /// High pass filter for the distance of the X-axis.
    hp_dx: HPFilter,
    /// High pass filter entry for the distance of the X-axis.
    hp_e_dx: f32,
    /// Old high pass filter entry for the distance of the X-axis.
    old_hp_e_dx: f32,
    /// High pass filter output for the distance of the X-axis.
    hp_s_dx: f32,
    /// Low pass filter for the distance of the X-axis.
    lp_dx: LPFilter,
    /// Low pass filter entry for the distance of the X-axis.
    lp_e_dx: f32,
    /// Low pass filter output for the distance of the X-axis.
    lp_s_dx: f32,
    //MOVING AVERAGE
    /// Moving average filter of the X-axis.
    ma_x: Smooth,
    /// Moving average filter output value of the X-axis.
    average_x: f32,
    /// Offset of the X-axis.
    offset_x: f32,
    //Y-AXIS
    //SPEED
    /// High pass filter for the speed of the Y-axis.
    hp_vy: HPFilter,
    /// High pass filter entry for the speed of the Y-axis.
    hp_e_vy : f32,
    /// Old high pass filter entry for the speed of the Y-axis.
    old_hp_e_vy: f32,
    /// High pass filter output for the speed of the Y-axis.
    hp_s_vy: f32,
    /// Speed on the Y-axis.
    speed_y: f32,
    /// Distance on the Y-axis.
    distance_y: f32,
    /// Low pass filter for the speed of the Y-axis.
    lp_vy: LPFilter,
    /// Low pass filter entry for the speed of the Y-axis.
    lp_e_vy: f32,
    /// Low pass filter output for the speed of the Y-axis.
    lp_s_vy: f32,
    //DISTANCE
    //High pass filter for the distance of the Y-axis.
    hp_dy: HPFilter,
    /// High pass filter entry for the distance of the Y-axis.
    hp_e_dy: f32,
    /// Old high pass filter entry for the distance of the Y-axis.
    old_hp_e_dy: f32,
    /// High pass filter output for the distance of the Y-axis.
    hp_s_dy: f32,
    /// Low pass filter for the distance of the Y-axis.
    lp_dy: LPFilter,
    /// Low pass filter entry for the distance of the Y-axis.
    lp_e_dy: f32,
    /// Low pass filter output for the distance of the Y-axis.
    lp_s_dy: f32,
    // MOVING AVERAGE
    /// Moving average filter of the Y-axis.
    ma_y: Smooth,
    /// Moving average filter output value of the Y-axis.
    average_y: f32,
    /// Offset of the Y-axis.
    offset_y: f32,
    //Other variables
    /// Counter used to set both X and Y-Axis offset.
    counter: u32,
    /// When the offsets are sets we start the filters.
    ready_2_go: bool,
    /// Final distance traveled by the sensor.
    final_distance: f32,
    // Useful constants that can be tweaked.
    /// Factor used to scale up or down the tolerance. 1 is normal. 100 is 100 times the tolerance.
    /// Defaults to 100 for better results. Tweaks BUFFER_ZONE_X and BUFFER_ZONE_Y.
    scale_factor: f32,
    /// Buffer zone used on the X-axis to compute the distance.
    buffer_zone_x: f32,
    /// Buffer zone used on the Y-axis to compute the distance.
    buffer_zone_y: f32,
    /// Correction factor to tweak the output value of our distance.
    correction_factor: f32,
    /// Buffer zone used with the Kalman filter (Dof6 only).
    buffer_zone_kalman: f32,
    /// Counter used to set the offset.
    counter_offset: u32,
}

impl Fusion {
    /// ### Create a new Fusion object. 
    /// ***You will need to provide data from a sensor!***
    ///  
    /// *hp_fc* and *lp_fc* are the cut-off frequencies of the high-pass and low-pass filters.
    /// We recommend 0.05 Hz for the high pass filter and 20.0 for the low pass filter.
    /// If you chose to disable distance computation, the first two parameters are not important and can be set to 0.0.
    /// 
    /// *num_readings* is the number of readings used to compute the moving average.
    /// As the number of readings increases, more information about the signal will be lost. 
    /// On the other hand, the lower the number of readings, the rougher the signal and the more approximate the measurement. 
    /// It is therefore necessary to find a middle ground, so we recommend a value between 25 and 50.
    /// 
    /// All variables are initialized to 0.0.
    /// 
    /// ***Usage:***
    /// ```no_run
    /// use datafusion::{self as _, Fusion};
    /// let mut fusion = Fusion::new(0.05, 20., 50);
    /// ```
    pub fn new(hp_fc: f32, lp_fc: f32, num_readings: usize) -> Self {
        Fusion {
            //General Variables
            acc_x: 0.,
            acc_y: 0.,
            acc_z: 0.,
            gyro_x: 0.,
            gyro_y: 0.,
            gyro_z: 0.,
            mag_rx: 0.,
            mag_ry: 0.,
            mag_rz: 0.,
            mag_x: 0.,
            mag_y: 0.,
            roll: 0.,
            pitch: 0.,
            yaw: 0.,
            heading: 0.,
            mode: Mode::Dof6,
            //Kalman Variables
            kalman_x_angle: 0.,
            kalman_y_angle: 0.,
            kalman_z_angle: 0.,
            kalman_x: Kalman::new(),
            kalman_y: Kalman::new(),
            kalman_z: Kalman::new(),
            ma_yaw: Smooth::new(num_readings),
            offset_kalman: 0.,
            sens: 0,
            sens_prec: 0,
            sens_count: 0,
            total_z_angle: 0.,
            angular_vel_z: 0.,
            declination: 0.,
            //Distance Variables
            enable_distance: true,
            //X-AXIS
            //SPEED
            //High pass filter for the speed of the X-axis
            hp_vx: HPFilter::new(hp_fc),
            hp_e_vx: 0.,
            old_hp_e_vx: 0.,
            hp_s_vx: 0.,
            //We define speed and distance variables
            speed_x: 0.,
            distance_x: 0.,
            //Low pass filter for the speed of the X-axis
            lp_vx: LPFilter::new(lp_fc),
            lp_e_vx: 0.,
            lp_s_vx: 0.,
            //DISTANCE
            //High pass filter for the distance of the X-axis
            hp_dx: HPFilter::new(hp_fc),
            hp_e_dx: 0.,
            old_hp_e_dx: 0.,
            hp_s_dx: 0.,
            //Low pass filter for the distance of the X-axis
            lp_dx: LPFilter::new(lp_fc),
            lp_e_dx: 0.,
            lp_s_dx: 0.,
            //MOVING AVERAGE
            //Moving average filter of the X-axis
            ma_x: Smooth::new(num_readings),
            average_x: 0.,
            offset_x: 0.,
            //Y-AXIS
            //SPEED
            //High pass filter for the speed of the Y-axis
            hp_vy: HPFilter::new(hp_fc),
            hp_e_vy : 0.,
            old_hp_e_vy: 0.,
            hp_s_vy: 0.,
            //We define speed and distance variables
            speed_y: 0.,
            distance_y: 0.,
            //Low pass filter for the speed of the Y-axis
            lp_vy: LPFilter::new(lp_fc),
            lp_e_vy: 0.,
            lp_s_vy: 0.,
            //DISTANCE
            //High pass filter for the distance of the Y-axis
            hp_dy: HPFilter::new(hp_fc),
            hp_e_dy: 0.,
            old_hp_e_dy: 0.,
            hp_s_dy: 0.,
            //Low pass filter for the distance of the Y-axis
            lp_dy: LPFilter::new(lp_fc),
            lp_e_dy: 0.,
            lp_s_dy: 0.,
            //MOVING AVERAGE
            //Moving average filter of the Y-axis
            ma_y: Smooth::new(num_readings),
            average_y: 0.,
            offset_y: 0.,
            //Other variables
            counter: 0,
            ready_2_go: false,
            final_distance: 0.,
            scale_factor: 100.,
            buffer_zone_x: 13.,
            buffer_zone_y: 10.,
            correction_factor: G2 * SQRT_G,
            buffer_zone_kalman: 0.05,
            counter_offset: 500,
        }
    }

    /// ### Initialize the device. 
    /// ***You must provide raw data before doing calling the function. Also, the mode defaults to a 6Dof sensor. So you might need to change the mode.***
    /// 
    /// - Initialize the Kalman filter.
    /// - The distance filters have been initialized with the default values within the creation of the device.
    /// 
    /// ***Usage (9Dof):***
    /// ```no_run
    /// use datafusion::{self as _, Fusion};
    /// let mut fusion = Fusion::new(0.05, 20., 50);
    /// fusion.set_mode(datafusion::Mode::Dof9);
    /// fusion.set_data_dof9(acc_x, acc_y, acc_z, gyro_x, gyro_y, gyro_z, mag_rx, mag_ry, mag_rz);
    /// fusion.init();
    /// ```
    /// ***Usage (6Dof):***
    /// ```no_run
    /// use datafusion::{self as _, Fusion};
    /// let mut fusion = Fusion::new(0.05, 20., 50);
    /// fusion.set_data_dof6(acc_x, acc_y, acc_z, gyro_x, gyro_y, gyro_z);
    /// fusion.init();
    /// ```
    pub fn init(&mut self) {

        // Initialize the Kalman filter
        self.roll = self.compute_roll();
        self.pitch = self.compute_pitch();
        self.yaw = self.compute_yaw();

        self.kalman_x.set_angle(self.roll);
        self.kalman_y.set_angle(self.pitch);
        self.kalman_z.set_angle(self.yaw);

        self.offset_kalman = self.yaw;

    }


    /// ### Performs a step of all filters.
    /// ***You must provide your own delta time in seconds.*** 
    /// 
    /// The more accurate the delta time, the more accurate the results.
    /// > 🔴 ***Always set raw data before calling this function.***
    /// - It updates the Kalman filters on all axis.
    /// - Then, it updates the speed and distance filters on all axis.
    /// - Finally, it updates the moving average filter on all axis.
    /// 
    /// ***Usage:***
    /// ```no_run
    /// // Init a delay used in certain functions and between each loop.
    /// let mut delay = Delay::new();
    /// // Setting up the delta time within a std environment
    /// let mut time = Instant::now();
    /// loop {
    ///     // Calculate delta time in seconds
    ///    let dt = time.elapsed().as_micros() as f32 / 1_000_000.;
    ///    time = Instant::now();
    /// 
    ///    // Set the raw data
    ///    fusion.set_data_dof9(acc_x, acc_y, acc_z, gyro_x, gyro_y, gyro_z, mag_rx, mag_ry, mag_rz);
    /// 
    ///    // Perform a step of the algorithm
    ///    fusion.step(dt);
    /// 
    ///    delay.delay_ms(5);
    /// }
    /// ```
    /// ***You can get all the results by calling the get_\* functions.***
    /// > ***NB: The results are not updated until you call the step function again.***
    pub fn step(&mut self, dt: f32) {

        // KALMAN
        // Calculate roll, pitch and yaw
        self.roll = self.compute_roll();
        self.pitch = self.compute_pitch();
        self.yaw = self.compute_yaw();

        // Calculate the angular speed of the x, y, z axes
        let mut gyro_x_rate = self.gyro_x * RAD2DEG;
        let gyro_y_rate = self.gyro_y * RAD2DEG;
        let gyro_z_rate = self.gyro_z * RAD2DEG;

        // Restrict Pitch
        // This fixes the transition problem when the accelerometer angle jumps between -180 and 180 degrees
        if ((self.pitch < -90.0) && (self.kalman_y_angle > 90.0)) || ((self.pitch > 90.0) && (self.kalman_y_angle < -90.0)) {
            self.kalman_y.set_angle(self.pitch);
            self.kalman_y_angle = self.pitch;
        } else {
            self.kalman_y.compute_angle(self.pitch, gyro_y_rate, dt);
            self.kalman_y_angle = self.kalman_y.get_angle(); 
        }

        // Invert rate, so it fits the restricted accelerometer reading
        if self.kalman_y_angle.abs() > 90.0 {
            gyro_x_rate = -gyro_x_rate;
        }

        // Update angles for x and z axis
        self.kalman_x.compute_angle(self.roll, gyro_x_rate, dt);
        self.kalman_x_angle = self.kalman_x.get_angle();

        if self.mode == Mode::Dof6 {
            self.kalman_z.compute_angle(self.yaw, gyro_z_rate, dt);
            self.kalman_z_angle = self.kalman_z.get_angle();

            let angle_z = self.kalman_z_angle - self.offset_kalman;

            // Tries to correct the drift of the angle
            // Acts like a buffer zone
            if angle_z < -self.buffer_zone_kalman {
                self.sens = -1;
            } else if angle_z > self.buffer_zone_kalman {
                self.sens = 1;
            } else {
                self.sens = 0;
            }

            // Counts the number of times the angle has changed
            if (self.sens != self.sens_prec) && (self.sens != 0) {
                self.sens_count += 1;
                self.sens_prec = self.sens;
            }

            // Movement is done when the angle has changed more than 3 times
            if self.sens_count > 3 {
                self.sens_count = 0;
                self.sens_prec = 0;
            } else {
                self.total_z_angle += self.compute_integral(angle_z, dt) * SQRT_G_KALMAN;
            }
        } else {
            self.kalman_z.compute_angle(self.yaw, gyro_z_rate, dt);
            self.kalman_z_angle = self.kalman_z.get_angle(); 

            // We add a smoothing filter to get better results
            self.ma_yaw.add_reading(self.kalman_z_angle);
            self.heading = self.ma_yaw.get_average();

            self.angular_vel_z = self.heading * dt;
        }

        if self.enable_distance {
            // DISTANCE
            // Offset management
            if self.counter == self.counter_offset {
                self.offset_x = self.average_x;
                self.offset_y = self.average_y;
                self.ready_2_go = true;
                self.distance_x = 0.;
                self.distance_y = 0.;
            }
            if self.counter < self.counter_offset + 1 {
                self.counter += 1;
            }

            // Scale acceleration
            let accel_x = self.acc_x * G_TO_SCALE;
            let accel_y = self.acc_y * G_TO_SCALE;

            // X-AXIS
            // We update the old value of the speed HP filter
            //self.old_hp_e_vx = self.hp_e_vx; // We don't need this anymore as it is handled with the function set_old_values.

            // HP filter for the speed of the X-axis
            self.hp_e_vx = accel_x;
            self.hp_s_vx = self.hp_vx.compute(self.hp_e_vx, self.old_hp_e_vx, self.hp_s_vx, dt);

            // LP filter for the speed of the X-axis
            self.lp_e_vx = self.hp_s_vx;
            self.lp_s_vx = self.lp_vx.compute(self.lp_e_vx, self.lp_s_vx, dt);

            // We compute VX
            self.speed_x = self.compute_integral(self.lp_s_vx, dt);

            // we update the old value of the distance HP filter
            self.old_hp_e_dx = self.hp_e_dx;

            // HP filter for the distance of the X-axis
            self.hp_e_dx = self.speed_x;
            self.hp_s_dx = self.hp_dx.compute(self.hp_e_dx, self.old_hp_e_dx, self.hp_s_dx, dt);

            // LP filter for the distance of the X-axis
            self.lp_e_dx = self.hp_s_dx;
            self.lp_s_dx = self.lp_dx.compute(self.lp_e_dx, self.lp_s_dx, dt);


            // Y-AXIS
            // We update the old value of the speed HP filter
            //self.old_hp_e_vy = self.hp_e_vy; // We don't need this anymore as it is handled with the function set_old_values.

            // HP filter for the speed of the Y-axis
            self.hp_e_vy = accel_y;
            self.hp_s_vy = self.hp_vy.compute(self.hp_e_vy, self.old_hp_e_vy, self.hp_s_vy, dt);

            // LP filter for the speed of the Y-axis
            self.lp_e_vy = self.hp_s_vy;
            self.lp_s_vy = self.lp_vy.compute(self.lp_e_vy, self.lp_s_vy, dt);

            // We compute VY
            self.speed_y = self.compute_integral(self.lp_s_vy, dt);

            // we update the old value of the distance HP filter
            self.old_hp_e_dy = self.hp_e_dy;

            // HP filter for the distance of the Y-axis
            self.hp_e_dy = self.speed_y;
            self.hp_s_dy = self.hp_dy.compute(self.hp_e_dy, self.old_hp_e_dy, self.hp_s_dy, dt);

            // LP filter for the distance of the Y-axis
            self.lp_e_dy = self.hp_s_dy;
            self.lp_s_dy = self.lp_dy.compute(self.lp_e_dy, self.lp_s_dy, dt);

            // Smoothing the curves
            self.ma_x.add_reading(self.lp_s_dx);
            self.average_x = self.ma_x.get_average() * self.scale_factor - self.offset_x;
            self.ma_y.add_reading(self.lp_s_dy);
            self.average_y = self.ma_y.get_average() * self.scale_factor - self.offset_y;

            let factor = self.correction_factor / self.scale_factor;

            // We compute the distance on the X-axis
            if (self.average_x.abs() > self.buffer_zone_x) && self.ready_2_go {
                self.distance_x += self.compute_integral(self.average_x, dt) * factor;
            }

            // We compute the distance on the Y-axis
            if (self.average_y.abs() > self.buffer_zone_y) && self.ready_2_go {
                self.distance_y += self.compute_integral(self.average_y, dt) * factor;
            }

            let dax = self.kalman_x_angle.sin() * G;
            let day = self.kalman_y_angle.sin() * G;
            let dpx = 0.5 * dax * dt * dt;
            let dpy = 0.5 * day * dt * dt;

            self.distance_x = (self.distance_x - dpx * self.distance_x).abs();
            self.distance_y = (self.distance_y - dpy * self.distance_y).abs();

            // We compute the final distance
            self.final_distance = self.distance(self.distance_x, self.distance_y);
        }
    }

    /// #### Set data from the accelerometer.
    /// Should be in *m/s²*.
    pub fn set_accel_data(&mut self, acc_x: f32, acc_y: f32, acc_z: f32) {
        self.acc_x = acc_x;
        self.acc_y = acc_y;
        self.acc_z = acc_z;
    }

    /// #### Set data from the gyroscope.
    /// Should be in *rad/s*.
    pub fn set_gyro_data(&mut self, gyro_x: f32, gyro_y: f32, gyro_z: f32) {
        self.gyro_x = gyro_x;
        self.gyro_y = gyro_y;
        self.gyro_z = gyro_z;
    }

    /// #### Set data from the magnetometer.
    /// Should be in *uT*.
    pub fn set_mag_data(&mut self, mag_x: f32, mag_y: f32, mag_z: f32) {
        self.mag_rx = mag_x;
        self.mag_ry = mag_y;
        self.mag_rz = mag_z;
    }

    /// #### Set data all in once for the accelerometer, gyroscope and magnetometer.
    /// Should be in *m/s²*, *rad/s* and *uT*.
    pub fn set_data_dof9(&mut self, acc_x: f32, acc_y: f32, acc_z: f32, gyro_x: f32, gyro_y: f32, gyro_z: f32, mag_rx: f32, mag_ry: f32, mag_rz: f32) {
        self.acc_x = acc_x;
        self.acc_y = acc_y;
        self.acc_z = acc_z;
        self.gyro_x = gyro_x;
        self.gyro_y = gyro_y;
        self.gyro_z = gyro_z;
        self.mag_rx = mag_rx;
        self.mag_ry = mag_ry;
        self.mag_rz = mag_rz;
    }


    /// #### Set data all in once for the accelerometer and gyroscope.
    /// Should be in *m/s²* and *rad/s*.
    pub fn set_data_dof6(&mut self, acc_x: f32, acc_y: f32, acc_z: f32, gyro_x: f32, gyro_y: f32, gyro_z: f32) {
        self.acc_x = acc_x;
        self.acc_y = acc_y;
        self.acc_z = acc_z;
        self.gyro_x = gyro_x;
        self.gyro_y = gyro_y;
        self.gyro_z = gyro_z;
    }

    /// #### Set old value for the High Pass Filter.
    /// Should be in *m/s²*.
    /// 
    /// ***This function must be called before reading data from the sensor.***
    pub fn set_old_values(&mut self, acc_x: f32, acc_y: f32) {
        self.old_hp_e_vx = acc_x * G_TO_SCALE;
        self.old_hp_e_vy = acc_y * G_TO_SCALE;
    }
    
    /// Returns the scaled value of the magnetic field on the X-Axis once computed with roll and pitch.
    /// Data is in *microtesla (uT)*.
    pub fn get_mag_x(&self) -> f32 {
        self.mag_x
    }

    /// Returns the scaled value of the magnetic field on the Y-Axis once computed with roll.
    /// Data is in *microtesla (uT)*.
    pub fn get_mag_y(&self) -> f32 {
        self.mag_y
    }

    /// Computes the integral of the angular velocity.
    /// Returns the integral in degrees.
    /// Angle should be in degrees/second.
    /// dt is the time in seconds since the last call to this function.
    fn compute_integral(&self, value: f32, dt: f32) -> f32 {
        value * dt
    }

    /// Computes the distance
    fn distance(&self, x: f32, y: f32) -> f32 {
        (x * x + y * y).sqrt()
    }

    /// #### Computes roll from accelerometer data.
    /// Computes and returns the roll in *degrees*.
    pub fn compute_roll(&self) -> f32 {
        self.acc_y.atan2(self.acc_z) * RAD2DEG
    }

    /// #### Computes pitch from accelerometer data.
    /// computes and returns the pitch in *degrees*.
    pub fn compute_pitch(&self) -> f32 {
        (-self.acc_x / (self.acc_y * self.acc_y + self.acc_z * self.acc_z).sqrt()).atan() * RAD2DEG
    }

    /// #### Computes yaw from accelerometer data.
    /// Computes and returns the yaw in *degrees* for Dof9 and the yaw velocity in *degrees/s* for Dof6. 
    pub fn compute_yaw(&mut self) -> f32 {

        if self.mode == Mode::Dof6 {
            (self.acc_z / (self.acc_x * self.acc_x + self.acc_z * self.acc_z).sqrt()).atan() * RAD2DEG
        } else {
            let roll_rad = self.roll * DEG2RAD;
            let pitch_rad = self.pitch * DEG2RAD;
            // Tilt compensation
            self.mag_x = self.mag_rx * pitch_rad.cos() + self.mag_ry * roll_rad.sin() * pitch_rad.sin() - self.mag_rz * self.roll.cos() * pitch_rad.sin();
            self.mag_y = -self.mag_ry * roll_rad.cos() + self.mag_rz * roll_rad.sin();
            (-self.mag_y.atan2(self.mag_x)) * RAD2DEG
        }
    }

    /// #### Get the heading from the magnetometer.
    /// This is the heading of the sensor after the Kalman filter and the moving average.
    /// The heading is updated every time the `step` function is called.
    /// 
    /// Returns the heading in *degrees*.
    /// 
    /// In Dof9 mode the heading is computed from the magnetometer data.
    /// Thus, this is the real heading of the sensor.
    /// In Dof6 mode the heading is computed from the accelerometer data.
    /// Thus, this is the same as the yaw.
    /// 
    /// ***Note***: This function is just a getter. If you want to compute and get the actual yaw, use `compute_yaw()` or run a step.
    pub fn get_heading(&self) -> f32 {
        if self.mode == Mode::Dof9 {
            self.heading + self.declination
        } else {
            self.yaw
        }
    }

    /// #### Set the measured distance to 0.
    /// Acts like an artificial reset.
    pub fn reset_distance(&mut self) {
        self.distance_x = 0.;
        self.distance_y = 0.;
        self.final_distance = 0.;
    }

    /// #### Set the measured angle to 0. 
    /// Acts like an artificial reset. 
    /// 
    /// ***NB: This is only useful for Dof6.***
    pub fn reset_angle_z(&mut self) {
        if self.mode == Mode::Dof6 {
            self.total_z_angle = 0.;
        }
    }

    /// #### Set the distance offsets to the current state of the system.
    /// 
    /// ***Warning: you can't go back to the previous state once changed.***
    /// 
    /// Usually used when auto-resetting the distance variables. 
    /// Must be called when the sensor does NOT move. Otherwise, next data won't be accurate.
    pub fn reset_offset_distance(&mut self) {
        self.offset_x = self.average_x;
        self.offset_y = self.average_y;
    }

    /// #### Set the Kalman offsets to the current state of the system.
    /// ***Warning: you can't go back to the previous state once changed.***
    /// 
    /// Usually used when auto-resetting the angle variables.  
    /// Must be called when the sensor does ***NOT*** move. Otherwise, next data won't be accurate.
    /// 
    /// ***NB: This is only useful for Dof6.***
    pub fn reset_offset_angle(&mut self) {
        if self.mode == Mode::Dof6 {
            self.offset_kalman = self.yaw;
        }
    }

    /// #### Get the Final X-Axis angle in *degrees*.
    /// Use DEG2RAD const to convert to *rad*.
    pub fn get_x_angle(&self) -> f32 {
        self.kalman_x_angle
    }

    /// #### Get the Final Y-Axis angle in *degrees*.
    /// Use DEG2RAD const to convert to *rad*.
    pub fn get_y_angle(&self) -> f32 {
        self.kalman_y_angle
    }

    /// #### Get the Final Z-Axis angle in *degrees*.
    /// If the sensor is in Dof9 mode, the angle is the yaw angle in after the Kalman filter.
    /// 
    /// If the sensor is in Dof6 mode, this is the accumulated angle traveled by the sensor.
    /// 
    /// Use DEG2RAD const to convert to *radians*.
    pub fn get_z_angle(&self) -> f32 {
        if self.mode == Mode::Dof6 {
            self.total_z_angle
        } else {
            self.kalman_z_angle
        }
        
    }

    /// #### Get the Z-axis angle velocity in *degrees/s*. 
    /// If the sensor is in Dof9 mode, the value is the derivative of the yaw angle.
    /// 
    /// If the sensor is in Dof6 mode, the value is the output of kalman filter.
    /// 
    /// Use DEG2RAD const to convert to *rad/s*.
    pub fn get_z_angular_velocity(&self) -> f32 {
        
        if self.mode == Mode::Dof6 {
            self.kalman_z_angle - self.offset_kalman
        } else {
            self.angular_vel_z
        }
    }

    /// Get the current speed on the X-axis in *cm/s*.
    pub fn get_x_speed(&self) -> f32 {
        self.speed_x
    }

    /// Get the current speed on the Y-axis in *cm/s*.
    pub fn get_y_speed(&self) -> f32 {
        self.speed_y
    }

    /// Get the current distance traveled on the X-axis in *cm*.
    pub fn get_x_distance(&self) -> f32 {
        self.distance_x
    }

    /// Get the current distance traveled on the Y-axis in *cm*.
    pub fn get_y_distance(&self) -> f32 {
        self.distance_y
    }

    /// Get the current final distance traveled in *cm*.
    pub fn get_final_distance(&self) -> f32 {
        self.final_distance
    }

    /// Get the fusion's mode.
    pub fn get_mode(&self) -> Mode {
        self.mode
    }

    /// #### Set the fusion's mode. 
    /// Either Dof6 or Dof9, respectively for 6-axis and 9-axis degrees of freedom.
    pub fn set_mode(&mut self, mode: Mode) {
        self.mode = mode;
    }

    /// #### Set the declination of the magnetic field.
    /// Must be in degrees. (Dof9 only)
    ///
    /// You can visit this [website](https://www.ngdc.noaa.gov/geomag/calculators/magcalc.shtml?#declination) to get the declination.
    pub fn set_declination(&mut self, declination: f32) {
        self.declination = declination;
    }

    /// #### Get the declination of the magnetic field.
    /// Returns the declination in degrees as a f32 (Dof9 only).
    /// The declination is the difference between the true north and the magnetic north.
    pub fn get_declination(&self) -> f32 {
        self.declination
    }

    /// #### Set the buffer zone on the X-Axis.
    /// This buffer zone is used while measuring distance.
    /// It's made to avoid to constantly count the distance.
    /// 
    /// It must be a percentage of the scale factor (that defaults to 100).
    /// 
    /// Defaults to 0.13 (13% of the scale factor).
    pub fn set_buffer_zone_x(&mut self, buffer_zone: f32) {
        self.buffer_zone_x = buffer_zone * self.scale_factor;
    }

    /// #### Set the buffer zone on the Y-Axis.
    /// This buffer zone is used while measuring distance.
    /// It's made to avoid to constantly count the distance.
    /// 
    /// It must be a percentage of the scale factor (that defaults to 100).
    /// 
    /// Defaults to 0.10 (10% of the scale factor).
    pub fn set_buffer_zone_y(&mut self, buffer_zone: f32) {
        self.buffer_zone_y = buffer_zone * self.scale_factor;
    }

    /// #### Set the scale factor for the distance measurements.
    /// This is the factor that is used to get a more accurate distance measurement
    /// and get a better readability of the distance while working in cm.
    /// 
    /// Defaults to 100.
    pub fn set_scale_factor(&mut self, scale_factor: f32) {
        self.scale_factor = scale_factor;
    }

    /// #### Set the correction factor for all Axis.
    /// This is the factor that is used to correct the distance measurements.
    /// While filtering the data, we are losing some precision in amplitude.
    /// To avoid this, we are applying a correction factor to the data.
    /// 
    /// Defaults to G2 / SQRT_G, where G2 is the gravity constant squared and SQRT_G is the square root of G.
    pub fn set_correction_factor(&mut self, correction_factor: f32) {
        self.correction_factor = correction_factor;
    }

    /// #### Set the buffer zone on the Z-Axis for the Kalman filter.
    /// DOF6 only. 
    /// Defaults to 0.05.
    pub fn set_buffer_zone_kalman(&mut self, buffer_zone: f32) {
        self.buffer_zone_kalman = buffer_zone;
    }

    /// #### Set the offset counter for the distance measurements.
    /// To measure the distance, we need the signal to be stable.
    /// So we ignore the first few measurements.
    /// 
    /// Defaults to 500.
    pub fn set_counter_offset(&mut self, counter_offset: u32) {
        self.counter_offset = counter_offset;
    }

    /// #### Disable the distance measurements.
    /// This allows to gain execution speed.
    /// 
    /// Defaults to true. (Distance measurements are enabled by default)
    pub fn disable_distance(&mut self, disable: bool) {
        self.enable_distance = disable;
    }

}
