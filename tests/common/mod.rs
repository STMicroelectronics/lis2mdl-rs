//! Common test utilities and helpers for LIS2MDL testing

#![cfg(test)]

// LIS2MDL Register Addresses - values extracted from DS to provide a comparison
#[allow(dead_code)]
pub mod registers {
    // Hard-iron offset registers
    pub const OFFSET_X_REG_L: u8 = 0x45;
    pub const OFFSET_X_REG_H: u8 = 0x46;
    pub const OFFSET_Y_REG_L: u8 = 0x47;
    pub const OFFSET_Y_REG_H: u8 = 0x48;
    pub const OFFSET_Z_REG_L: u8 = 0x49;
    pub const OFFSET_Z_REG_H: u8 = 0x4A;

    // Device identification
    pub const WHO_AM_I: u8 = 0x4F;

    // Configuration registers
    pub const CFG_REG_A: u8 = 0x60;
    pub const CFG_REG_B: u8 = 0x61;
    pub const CFG_REG_C: u8 = 0x62;

    // Interrupt registers
    pub const INT_CRTL_REG: u8 = 0x63;
    pub const INT_SOURCE_REG: u8 = 0x64;
    pub const INT_THS_L_REG: u8 = 0x65;
    pub const INT_THS_H_REG: u8 = 0x66;

    // Status and output registers
    pub const STATUS_REG: u8 = 0x67;
    pub const OUTX_L_REG: u8 = 0x68;
    pub const OUTX_H_REG: u8 = 0x69;
    pub const OUTY_L_REG: u8 = 0x6A;
    pub const OUTY_H_REG: u8 = 0x6B;
    pub const OUTZ_L_REG: u8 = 0x6C;
    pub const OUTZ_H_REG: u8 = 0x6D;
    pub const TEMP_OUT_L_REG: u8 = 0x6E;
    pub const TEMP_OUT_H_REG: u8 = 0x6F;
}

// LIS2MDL Constants
#[allow(dead_code)]
pub mod constants {
    pub const I2C_ADDRESS: u8 = 0x1E; // 7-bit address
    pub const WHO_AM_I_VALUE: u8 = 0x40;
    pub const SENSITIVITY_MGAUSS_PER_LSB: f32 = 1.5;
    pub const TEMP_SENSITIVITY_LSB_PER_C: f32 = 8.0;
    pub const FULL_SCALE_GAUSS: f32 = 49.152;

    // Default register values after reset
    pub const CFG_REG_A_DEFAULT: u8 = 0x03;
    pub const CFG_REG_B_DEFAULT: u8 = 0x00;
    pub const CFG_REG_C_DEFAULT: u8 = 0x00;
    pub const INT_CTRL_REG_DEFAULT: u8 = 0xE0;
}

/// Convert raw magnetic data to milligauss
pub fn raw_to_milligauss(raw: i16) -> f32 {
    raw as f32 * constants::SENSITIVITY_MGAUSS_PER_LSB
}

/// Convert raw temperature data to Celsius
pub fn raw_to_celsius(raw: i16) -> f32 {
    raw as f32 / constants::TEMP_SENSITIVITY_LSB_PER_C
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_raw_to_milligauss_conversion() {
        let raw: i16 = 1000;
        let mgauss = raw_to_milligauss(raw);
        assert_eq!(mgauss, 1500.0);
    }

    #[test]
    fn test_raw_to_celsius_conversion() {
        let raw: i16 = 200; // 200 LSB = 25°C
        let celsius = raw_to_celsius(raw);
        assert_eq!(celsius, 25.0);
    }

    #[test]
    fn test_negative_temperature() {
        let raw: i16 = -80; // -80 LSB = -10°C
        let celsius = raw_to_celsius(raw);
        assert_eq!(celsius, -10.0);
    }
}
