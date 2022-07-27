use std::time::Instant;

use rppal::hal::Delay;
use rppal::i2c::I2c;

use embedded_hal::blocking::delay::*;

use adafruit_nxp::*;
use datafusion_imu::{self as _, Fusion, Mode};


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

    sensor.set_accel_range(config::AccelMagRange::Range2g)?;
    sensor.set_gyro_range(config::GyroRange::Range500dps)?;
    sensor.set_accelmag_output_data_rate(config::AccelMagODR::ODR200HZ)?;
    sensor.set_gyro_output_data_rate(config::GyroODR::ODR200HZ)?;

    // Initialize the sensor
    sensor.read_data()?;

    let acc_x = sensor.accel_sensor.get_scaled_x();
    let acc_y = sensor.accel_sensor.get_scaled_y();
    let acc_z = sensor.accel_sensor.get_scaled_z();

    let gyro_x = sensor.gyro_sensor.get_scaled_x();
    let gyro_y = sensor.gyro_sensor.get_scaled_y();
    let gyro_z = sensor.gyro_sensor.get_scaled_z();

    /* Mode DOF9 
    let mag_rx = sensor.mag_sensor.get_scaled_x();
    let mag_ry = sensor.mag_sensor.get_scaled_y();
    let mag_rz = sensor.mag_sensor.get_scaled_z();
    */

    // Create a datafusion object
    let mut fusion = Fusion::new(0.05, 20., 50);
    fusion.set_mode(Mode::Dof6);

    /* DOF9
    // Set data to the fusion object DOF9
    //fusion.set_data_dof9(acc_x, acc_y, acc_z, gyro_x, gyro_y, gyro_z, mag_rx, mag_ry, mag_rz);
    */

    // Set data to the fusion object DOF6
    fusion.set_data_dof6(acc_x, acc_y, acc_z, gyro_x, gyro_y, gyro_z);

    // Initialize the datafusion object
    fusion.init();

    // Distance auto reset variables
    let mut distance= 0.0;
    let mut old_distance:f32;
    let mut counter_reset_distance = 0;

    // Angle auto reset variables --> DOF6 Only 
    let mut angle_z = 0.0;
    let mut old_angle_z: f32;
    let mut counter_reset_angle_z = 0;

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

        // Set data to the fusion object
        fusion.set_data_dof6(acc_x, acc_y, acc_z, gyro_x, gyro_y, gyro_z);

        // Update old variables
        old_distance = distance;
        old_angle_z = angle_z;

        // Perform a step of the algorithm
        fusion.step(dt);

        // Collect outputs
        angle_z = fusion.get_z_angle();
        distance = fusion.get_final_distance();

        // Compare data for distance
        if (old_distance - distance).abs() == 0.0 {
            if counter_reset_distance == 500 {
                counter_reset_distance = 0;
                fusion.reset_distance();
            } else {
                counter_reset_distance += 1;
            }
        } else {
            counter_reset_distance = 0;
        }

        // Compare data for angle_z --> DOF6 Only
        if (old_angle_z - angle_z).abs() == 0.0 {
            if counter_reset_angle_z == 500 {
                counter_reset_angle_z = 0;
                fusion.reset_angle_z();
            } else {
                counter_reset_angle_z += 1;
            }
        } else {
            counter_reset_angle_z = 0;
        }

        // Print data
        std::println!("Angle Z: {} °", angle_z);
        std::println!("Total distance traveled: {} cm", distance);

        delay.delay_ms(5_u8);
    }
}
