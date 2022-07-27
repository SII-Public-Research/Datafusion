use std::time::Instant;

use rppal::hal::Delay;
use rppal::i2c::I2c;

use embedded_hal::blocking::delay::*;

use adafruit_nxp::*;
use datafusion_imu::{self as _, Fusion};

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

    let mag_rx = sensor.mag_sensor.get_scaled_x();
    let mag_ry = sensor.mag_sensor.get_scaled_y();
    let mag_rz = sensor.mag_sensor.get_scaled_z();

    // Create a datafusion object
    let mut fusion = Fusion::new(0.05, 20., 50);
    fusion.set_mode(datafusion_imu::Mode::Dof9);

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
        std::print!("Angle X: {} °,", angle_x);
        std::print!(" Angle Y: {} °,", angle_y);
        std::println!(" Angle Z: {} °", angle_z);
        std::println!("Total distance traveled: {} cm", distance);

        delay.delay_ms(5_u8);
    }
}