# Datafusion library for 6 and 9 degrees of freedom sensors.

[![made-with-rust](https://img.shields.io/badge/Made%20with-Rust-1f425f?style=plastic)](https://www.rust-lang.org/)
[![powered-by-sii](https://img.shields.io/badge/Powered%20By-SII-blue?style=plastic)](https://sii-group.com/fr-FR/sii-sud-ouest)

You have the choice in this library to apply the filters on two types of sensors:
- A 6 degrees of freedom sensor with a 3-axis accelerometer and 3-axis gyroscope -> Mode `Dof6`
- A 9 degrees of freedom sensor with a 3-axis accelerometer, 3-axis gyroscope and 3-axis magnetometer -> Mode `Dof9`
 
> 🔴 ***For now, the sensor needs to be perfectly flat to work. Also, using angle and distance measurement at the same time is not recommended with a 6Dof sensor.***  
> More specifically, for the distance it doesn't really matter as the high pass filter will automatically delete the angle offset after a few second.  
> However, for the angle (except with a magnetometer) the sensor has to be flat or at least stay within the same inclination on the X and Y-axis after initialization.  

In the example provided below, we use our own driver for an Adafruit sensor. So of course, you can use any other driver you want.

### Angle data
When the sensor is flat and whatever the mode used, X and Y angles are absolute roll and pitch values in degrees.
A Kalman filter is applied between acceleration and angular speed to produce a reliable output.
While using a Dof6 sensor, e.g without a magnetometer, the Kalman filter output on the the Z or Yaw-axis is an angular speed in degrees per second.
Which is then integrated to produce the absolute angle in degrees. But this method is not really reliable as it heavily rely on which speed the sensor is moving.
However, when using a Dof9 sensor, the Kalman filter output on the Z or Yaw-axis is an absolute angle in degrees. This is the preferred method.
As long as there is no magnetic interference, you we will be able a obtain a heading angle down to a 2 to 3 degree accuracy.
In the case of magnetic interferences, if it is much greater than the earth magnetic field, then the output won't be the magnetic north but a relative angle to the magnetic interference.

### Distance data
For the distance data, the algorithm is the same whatever the mode used as it only uses acceleration data on X and Y.
It is a succession of high pass filters, low pass filters and integrations. 
It works perfectly fine on a short distance, e.g < 5cm, and is impressively accurate in this range. 
However, over this range the result highly rely on the speed. 
This means that, if the speed is too low, the distance will be under estimated. 
Same problem if the speed is too high, the distance will be over estimated. 
This problem is currently being worked on and the lib will be updated if a workaround is found. 

You have the ability to disable the distance measurement by calling the `disable_distance` function.

***Please note that distances measurements are purely experimental but it's a place to start as the accuracy is not optimal.***

### Usage
The example below uses [rppal](https://crates.io/crates/rppal) crates and runs on a Raspberry Pi.

Please note that this example and library have been developed in parallel with a driver for a Dof9 sensor from Adafruit.

```rust
use std::time::Instant;

use rppal::hal::Delay;
use rppal::i2c::I2c;

use embedded_hal::blocking::delay::*;

use adafruit::*;
use datafusion::{self as _, Fusion};

fn main() -> Result<(), SensorError<rppal::i2c::Error>> {

    // Init a delay used in certain functions and between each loop.
    let mut delay = Delay::new();

    // Setup the raspberry's I2C interface to create the sensor.
    let i2c = I2c::new().unwrap();

    // Create an Adafruit object
    let mut sensor = AdafruitNXP::new(0x8700A, 0x8700B, 0x0021002C, i2c);

    // Check if the sensor is ready to go
    let ready = sensor.begin()?;
    if !ready {
        std::eprintln!("Sensor not detected, check your wiring!");
        std::process::exit(1);
    }

    sensor.set_accel_range(config::AccelMagRange::Range8g)?;
    sensor.set_gyro_range(config::GyroRange::Range500dps)?;
    // etc...

    // Initialize the sensor
    sensor.read_data()?;

    let acc_x = sensor.accel_sensor.get_scaled_x();
    let acc_y = sensor.accel_sensor.get_scaled_y();
    let acc_z = sensor.accel_sensor.get_scaled_z();

    let gyro_x = sensor.gyro_sensor.get_scaled_x();
    let gyro_y = sensor.gyro_sensor.get_scaled_y();
    let gyro_z = sensor.gyro_sensor.get_scaled_z();

    let mag_rx = sensor.mag_sensor.get_scaled_x();
    let mag_ry = sensor.mag_sensor.get_scaled_y();
    let mag_rz = sensor.mag_sensor.get_scaled_z();

    // Create a datafusion object
    let mut fusion = Fusion::new(0.05, 20., 50);
    fusion.set_mode(datafusion::Mode::Dof9);

    // Set data to the fusion object
    fusion.set_data_dof9(acc_x, acc_y, acc_z, gyro_x, gyro_y, gyro_z, mag_rx, mag_ry, mag_rz);

    // Initialize the datafusion object
    fusion.init();

    // Set magnetic declination --> 1.39951° in Toulouse, France
    fusion.set_declination(1.39951);

    // Setting up the delta time
    let mut time = Instant::now();

    loop {

        // Calculate delta time in seconds
        let dt = time.elapsed().as_micros() as f32 / 1_000_000.;
        time = Instant::now();

        // Update old values for the next loop
        fusion.set_old_values(acc_x, acc_y);

        sensor.read_data()?;

        let acc_x = sensor.accel_sensor.get_scaled_x();
        let acc_y = sensor.accel_sensor.get_scaled_y();
        let acc_z = sensor.accel_sensor.get_scaled_z();

        let gyro_x = sensor.gyro_sensor.get_scaled_x();
        let gyro_y = sensor.gyro_sensor.get_scaled_y();
        let gyro_z = sensor.gyro_sensor.get_scaled_z();

        let mag_rx = sensor.mag_sensor.get_scaled_x();
        let mag_ry = sensor.mag_sensor.get_scaled_y();
        let mag_rz = sensor.mag_sensor.get_scaled_z();

        // Set data to the fusion object
        fusion.set_data_dof9(acc_x, acc_y, acc_z, gyro_x, gyro_y, gyro_z, mag_rx, mag_ry, mag_rz);

        // Perform a step of the algorithm
        fusion.step(dt);

        // Collect outputs
        let angle_x = fusion.get_x_angle();
        let angle_y = fusion.get_y_angle();
        let angle_z = fusion.get_heading();
        let distance = fusion.get_final_distance();

        // Print data
        std::println!("Angle X: {} °", angle_x);
        std::println!("Angle Y: {} °", angle_y);
        std::println!("Angle Z: {} °", angle_z);
        std::println!("Total distance traveled: {} cm", distance);

        delay.delay_ms(5_u8);
    }
}
```