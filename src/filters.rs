//! This library is used to define multiple types of filters.
//!
//! We have the following filters:
//! 1. Smooth Filter - This is a filter that is used to smooth the data, it's a moving average.
//! 2. HPFilter - High Pass Filter.
//! 3. LPFilter - Low Pass Filter.

#[cfg(feature = "alloc")]
use alloc::vec::Vec;

use core::f32::consts::PI;

/// # AvgFilter
/// All averaging filters should implement this trait.
pub trait AvgFilter {
    /// Add a new reading to the filter.
    fn add_reading(&mut self, reading: f32);

    /// Get the current average reading.
    fn get_average(&self) -> f32;
}

#[derive(Clone, Debug)]
#[cfg(feature = "alloc")]
/// This is a simple moving average filter.
pub struct Smooth {
    n: usize,
    readings: Vec<f32>,
    read_index: usize,
    average: f32,
}

#[cfg(feature = "alloc")]
impl Smooth {
    /// ### Create new filter
    /// *num_readings* Number of readings used to compute the moving average.
    /// As the number of readings increases, more information about the signal will be lost.
    /// On the other hand, the lower the number of readings, the rougher the signal and the more approximate the measurement.
    /// It is therefore necessary to find a middle ground, so we recommend a value between 25 and 50.
    pub fn new(num_readings: usize) -> Self {
        Smooth {
            n: num_readings,
            readings: vec![0.0; num_readings],
            read_index: 0,
            average: 0.0,
        }
    }
}

#[cfg(feature = "alloc")]
impl AvgFilter for Smooth {
    /// Add a new reading to the filter.
    fn add_reading(&mut self, reading: f32) {
        self.readings[self.read_index] = reading;

        self.read_index += 1;
        self.read_index %= self.n;

        self.average = self.readings.iter().fold(0.0, |s, &x| s + x) / self.n as f32;
    }

    /// Get the current average reading.
    fn get_average(&self) -> f32 {
        self.average
    }
}

#[derive(Clone, Debug)]
/// This is a simple moving average filter.
pub struct FixedSmooth<const SIZE: usize> {
    readings: [f32; SIZE],
    read_index: usize,
    average: f32,
}

impl<const SIZE: usize> FixedSmooth<SIZE> {
    /// ### Create new filter
    /// *SIZE* Number of readings used to compute the moving average.
    /// As the number of readings increases, more information about the signal will be lost.
    /// On the other hand, the lower the number of readings, the rougher the signal and the more approximate the measurement.
    /// It is therefore necessary to find a middle ground, so we recommend a value between 25 and 50.
    pub fn new() -> Self {
        FixedSmooth {
            readings: [0.0; SIZE],
            read_index: 0,
            average: 0.0,
        }
    }
}

impl<const SIZE: usize> Default for FixedSmooth<SIZE> {
    fn default() -> Self {
        Self::new()
    }
}

impl<const SIZE: usize> AvgFilter for FixedSmooth<SIZE> {
    /// Add a new reading to the filter.
    fn add_reading(&mut self, reading: f32) {
        self.readings[self.read_index] = reading;

        self.read_index += 1;
        self.read_index %= SIZE;

        self.average = self.readings.iter().fold(0.0, |s, &x| s + x) / SIZE as f32;
    }

    /// Get the current average reading.
    fn get_average(&self) -> f32 {
        self.average
    }
}

#[derive(Copy, Clone, Debug)]
/// High pass filter. Simply provide the cutoff frequency.
pub struct HPFilter {
    output: f32,
    tau: f32,
}

impl HPFilter {
    /// Create a new High Pass Filter with the given cutout frequency.
    pub fn new(fc: f32) -> Self {
        HPFilter {
            output: 0.0,
            tau: 1. / (2.0 * PI * fc),
        }
    }

    /// Computes the output of the filter.
    /// You should provide the input, the n-1 input, the n-1 output, and the time elapsed since the last call.
    pub fn compute(&mut self, input: f32, old_input: f32, old_output: f32, dt: f32) -> f32 {
        self.output = old_output + (input - old_input) - (dt / self.tau) * old_output;
        self.output
    }

    /// Get the current output.
    pub fn get_output(&self) -> f32 {
        self.output
    }
}

#[derive(Copy, Clone, Debug)]
/// Low pass filter. Simply provide the cutout frequency.
pub struct LPFilter {
    output: f32,
    tau: f32,
}

impl LPFilter {
    /// Create a new High Pass Filter with the given cutout frequency.
    pub fn new(fc: f32) -> Self {
        LPFilter {
            output: 0.0,
            tau: 1. / (2.0 * PI * fc),
        }
    }

    /// Computes the output of the filter.
    /// You should provide the input, the n-1 output, and the time elapsed since the last call.
    pub fn compute(&mut self, input: f32, old_output: f32, dt: f32) -> f32 {
        self.output = old_output + (input - old_output) * (dt / self.tau);
        self.output
    }

    /// Get the current output.
    pub fn get_output(&self) -> f32 {
        self.output
    }
}

#[cfg(test)]
mod tests {
    use super::{AvgFilter, FixedSmooth};

    #[test]
    fn test_fixed_avg_filter() {
        let mut filter = FixedSmooth::<5>::new();
        filter.add_reading(2.0);
        filter.add_reading(2.0);
        filter.add_reading(3.0);
        filter.add_reading(4.0);
        filter.add_reading(4.0);

        assert_eq!(filter.get_average(), 3.0);

        filter.add_reading(1.0);
        filter.add_reading(1.0);
        filter.add_reading(2.0);
        filter.add_reading(3.0);
        filter.add_reading(3.0);

        assert_eq!(filter.get_average(), 2.0);
    }
}
