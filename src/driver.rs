use super::{
    BusOperation, DelayNs, I2c, RegisterOperation, SensorOperation, SevenBitAddress, SpiDevice,
    bisync, i2c, prelude::*, spi,
};

use core::fmt::Debug;
use core::marker::PhantomData;

/// Driver for LIS2MDL sensor.
///
/// The struct takes a bus object to write to the registers.
/// The bus is generalized over the BusOperation trait, allowing the use
/// of I2C or SPI protocols; this also allows the user to implement sharing
/// techniques to share the underlying bus.
#[bisync]
pub struct Lis2mdl<B, T, S>
where
    B: BusOperation,
    T: DelayNs,
    S: SensorState,
{
    /// The bus driver.
    pub bus: B,
    pub tim: T,
    _state: PhantomData<S>,
}

/// Driver errors.
#[derive(Debug)]
#[bisync]
pub enum Error<B> {
    Bus(B),          // Error at the bus level
    UnexpectedValue, // Unexpected value read from a register
    InvalidConfiguration,
    HwNoResponse,
}

#[bisync]
impl<B, T, S> Lis2mdl<B, T, S>
where
    B: BusOperation,
    T: DelayNs,
    S: SensorState,
{
    /// Constructor method using a generic Bus that implements BusOperation
    pub fn new_bus(bus: B, tim: T) -> Self {
        Self {
            bus,
            tim,
            _state: PhantomData,
        }
    }
}

#[bisync]
impl<P, T> Lis2mdl<i2c::I2cBus<P>, T, OnState>
where
    P: I2c,
    T: DelayNs,
{
    /// Constructor method for using the I2C bus.
    pub fn new_i2c(i2c: P, address: I2CAddress, tim: T) -> Self {
        // Initialize the I2C bus with the COMPONENT address
        let bus = i2c::I2cBus::new(i2c, address as SevenBitAddress);
        Self {
            bus,
            tim,
            _state: PhantomData,
        }
    }
}

#[bisync]
impl<P, T, S> Lis2mdl<spi::SpiBus<P>, T, S>
where
    P: SpiDevice,
    T: DelayNs,
    S: SensorState,
{
    /// Constructor method for using the SPI bus.
    pub fn new_spi(spi: P, tim: T) -> Self {
        // Initialize the SPI bus
        let bus = spi::SpiBus::new(spi);
        Self {
            bus,
            tim,
            _state: PhantomData,
        }
    }
}

#[bisync]
impl<B, T, S> SensorOperation for Lis2mdl<B, T, S>
where
    B: BusOperation,
    T: DelayNs,
    S: SensorState,
{
    type Error = Error<B::Error>;

    async fn write_to_register(&mut self, reg: u8, buf: &[u8]) -> Result<(), Error<B::Error>> {
        self.bus
            .write_to_register(reg, buf)
            .await
            .map_err(Error::Bus)
    }

    async fn read_from_register(&mut self, reg: u8, buf: &mut [u8]) -> Result<(), Error<B::Error>> {
        self.bus
            .read_from_register(reg, buf)
            .await
            .map_err(Error::Bus)
    }
}

#[bisync]
impl<B, T> Lis2mdl<B, T, OnState>
where
    B: BusOperation,
    T: DelayNs,
{
    /// Set magnetic user offset for hard-iron to compensate environmental effects.
    ///
    /// Data format is the same of output data raw: two's complement
    /// with 1LSb = 1.5mg. These values act on the magnetic output data
    /// value in order to delete the environmental offset.
    pub async fn mag_user_offset_set(&mut self, val: &[i16; 3]) -> Result<(), Error<B::Error>> {
        let offset = OffsetXYZ {
            x: val[0],
            y: val[1],
            z: val[2],
        };
        offset.write(self).await?;

        Ok(())
    }

    /// Get magnetic user offset for hard-iron to compensate environmental effects.
    ///
    /// Data format is the same of output data raw: two's complement
    /// with 1LSb = 1.5mg. These values act on the magnetic output data
    /// value in order to delete the environmental offset.
    pub async fn mag_user_offset_get(&mut self) -> Result<[i16; 3], Error<B::Error>> {
        let val = OffsetXYZ::read(self).await?;

        Ok([val.x, val.y, val.z])
    }

    /// Set Operating mode.
    pub async fn operating_mode_set(&mut self, val: Md) -> Result<(), Error<B::Error>> {
        let mut reg = CfgRegA::read(self).await?;
        reg.set_md(val as u8);
        reg.write(self).await
    }
    /// Get Operating mode.
    pub async fn operating_mode_get(&mut self) -> Result<Md, Error<B::Error>> {
        let reg = CfgRegA::read(self).await?;

        let val = Md::try_from(reg.md()).unwrap_or_default();
        Ok(val)
    }
    /// Set Output data rate(ODR).
    pub async fn data_rate_set(&mut self, val: Odr) -> Result<(), Error<B::Error>> {
        let mut reg = CfgRegA::read(self).await?;
        reg.set_odr(val as u8);
        reg.write(self).await
    }
    /// Get Output data rate(ODR).
    pub async fn data_rate_get(&mut self) -> Result<Odr, Error<B::Error>> {
        let reg = CfgRegA::read(self).await?;
        let val = Odr::try_from(reg.odr()).unwrap_or_default();
        Ok(val)
    }
    /// Enables/Disable high-resolution/low-power mode.
    pub async fn power_mode_set(&mut self, val: PowerMode) -> Result<(), Error<B::Error>> {
        let mut reg = CfgRegA::read(self).await?;
        reg.set_lp(val as u8);
        reg.write(self).await?;

        Ok(())
    }
    /// Get high-resolution/low-power mode.
    pub async fn power_mode_get(&mut self) -> Result<PowerMode, Error<B::Error>> {
        let reg = CfgRegA::read(self).await?;

        let val = PowerMode::try_from(reg.lp()).unwrap_or_default();
        Ok(val)
    }
    /// Enables/Disables the magnetometer temperature compensation.
    pub async fn offset_temp_comp_set(&mut self, val: u8) -> Result<(), Error<B::Error>> {
        let mut reg = CfgRegA::read(self).await?;
        reg.set_comp_temp_en(val);
        reg.write(self).await?;

        Ok(())
    }
    /// Get configuration (enable/disable) for magnetometer temperature compensation.
    pub async fn offset_temp_comp_get(&mut self) -> Result<u8, Error<B::Error>> {
        let val: u8 = CfgRegA::read(self).await?.comp_temp_en();

        Ok(val)
    }
    /// Set Low-pass bandwidth selection.
    pub async fn low_pass_bandwidth_set(
        &mut self,
        val: LowPassFilter,
    ) -> Result<(), Error<B::Error>> {
        let mut reg = CfgRegB::read(self).await?;
        reg.set_lpf(val as u8);
        reg.write(self).await?;

        Ok(())
    }
    /// Get Low-pass bandwidth.
    pub async fn low_pass_bandwidth_get(&mut self) -> Result<LowPassFilter, Error<B::Error>> {
        let reg = CfgRegB::read(self).await?;

        let val = LowPassFilter::try_from(reg.lpf()).unwrap_or_default();
        Ok(val)
    }
    /// Set Reset mode.
    pub async fn set_rst_mode_set(&mut self, val: SetRst) -> Result<(), Error<B::Error>> {
        let mut reg = CfgRegB::read(self).await?;
        reg.set_set_rst(val as u8);
        reg.write(self).await?;

        Ok(())
    }
    /// Get Reset mode.
    pub async fn set_rst_mode_get(&mut self) -> Result<SetRst, Error<B::Error>> {
        let reg = CfgRegB::read(self).await?;

        let val = SetRst::try_from(reg.set_rst()).unwrap_or_default();
        Ok(val)
    }
    /// Enables/Disables offset cancellation in single measurement mode.
    ///
    /// The OFF_CANC bit must be set to 1 when enabling offset
    /// cancellation in single measurement mode, which means a
    /// call function: set_rst_mode(SENS_OFF_CANC_EVERY_ODR)
    /// is needed.
    pub async fn set_rst_sensor_single_set(&mut self, val: u8) -> Result<(), Error<B::Error>> {
        let mut reg = CfgRegB::read(self).await?;
        let rst = SetRst::try_from(reg.set_rst()).map_err(|_| Error::UnexpectedValue)?;

        if val == 1 && rst != SetRst::SensOffCancEveryOdr {
            return Err(Error::InvalidConfiguration);
        }

        reg.set_off_canc_one_shot(val);
        reg.write(self).await?;

        Ok(())
    }
    /// Get configuration (enable/disable) for offset cancellation in single measurement mode.
    ///
    /// The OFF_CANC bit must be set to 1 when enabling offset
    /// cancellation in single measurement mode. This means calling
    /// the function set_rst_mode(SensOffCancEveryOdr) is needed.
    pub async fn set_rst_sensor_single_get(&mut self) -> Result<u8, Error<B::Error>> {
        let val: u8 = CfgRegB::read(self).await?.off_canc_one_shot();

        Ok(val)
    }
    /// Enable/Disable Block data update.
    ///
    /// If val is 1: BDU is active
    pub async fn block_data_update_set(&mut self, val: u8) -> Result<(), Error<B::Error>> {
        let mut reg = CfgRegC::read(self).await?;
        reg.set_bdu(val);
        reg.write(self).await?;

        Ok(())
    }
    /// Get Block data update configuration (enable/disable).
    pub async fn block_data_update_get(&mut self) -> Result<u8, Error<B::Error>> {
        let val: u8 = CfgRegC::read(self).await?.bdu();

        Ok(val)
    }
    /// Get magnetic data ready data available.
    ///
    /// If event available returns 1.
    pub async fn mag_data_ready_get(&mut self) -> Result<u8, Error<B::Error>> {
        let val: u8 = StatusReg::read(self).await?.zyxda();

        Ok(val)
    }
    /// Get magnetic data overrun event.
    ///
    /// 0: no overrun has occurred. 1: a new set of data has overwritten the previous set
    pub async fn mag_data_ovr_get(&mut self) -> Result<u8, Error<B::Error>> {
        let val: u8 = StatusReg::read(self).await?.zyxor();

        Ok(val)
    }
    /// Get Magnetic raw value.
    ///
    /// Need conversion using `from_lsb_to_mgauss`
    pub async fn magnetic_raw_get(&mut self) -> Result<[i16; 3], Error<B::Error>> {
        let val = OutXYZ::read(self).await?;

        Ok([val.x, val.y, val.z])
    }

    /// Get temperature raw value.
    pub async fn temperature_raw_get(&mut self) -> Result<i16, Error<B::Error>> {
        TempOut::read(self).await.map(|temp| temp.0)
    }

    /// Get Device Id.
    ///
    /// Reads the WHO_AM_I register.
    pub async fn device_id_get(&mut self) -> Result<u8, Error<B::Error>> {
        WhoAmI::read(self).await.map(|reg| reg.who_am_i())
    }

    /// Execute a software reset.
    ///
    /// Resets control registers to their default values.
    /// The software reset procedure takes approximately 5 µs; this function handles the wait internally
    /// (one reading after 5 us for a maximum of 3 attempts).
    ///
    /// Returns `Error::HwNoResponse` if the SOFT_RST bit is not cleared by hardware within 3 ms.
    pub async fn sw_reset(&mut self) -> Result<(), Error<B::Error>> {
        let mut cfg_reg_a = CfgRegA::default();
        let mut retry: u8 = 0;

        /* 1. Set the SOFT_RST bit of the CFG_REG_A register to 1. */
        cfg_reg_a.set_soft_rst(1);
        cfg_reg_a.write(self).await?;

        /* 2. Poll the SOFT_RST bit of the CFG_REG_A register until il returns to 0. (should require 5us) */
        loop {
            cfg_reg_a = CfgRegA::read(self).await?;

            if cfg_reg_a.soft_rst() == 0 {
                return Ok(());
            }

            retry += 1;
            if retry > 3 {
                break;
            }

            self.tim.delay_us(5).await;
        }

        Err(Error::HwNoResponse)
    }

    /// Set Software reset.
    ///
    /// If val is 1: Restore the default values in user registers.
    /// Flash register keep their values.
    #[deprecated(since = "2.0.0", note = "please use sw_reset")]
    pub async fn reset_set(&mut self, val: u8) -> Result<(), Error<B::Error>> {
        let mut reg = CfgRegA::read(self).await?;
        reg.set_soft_rst(val);
        reg.write(self).await?;

        Ok(())
    }
    /// Get Software reset configuration.
    ///
    /// If set to 1: Restore the default values in user registers.
    #[deprecated(since = "2.0.0", note = "please use sw_reset")]
    pub async fn reset_get(&mut self) -> Result<u8, Error<B::Error>> {
        let val: u8 = CfgRegA::read(self).await?.soft_rst();

        Ok(val)
    }

    pub async fn reboot(&mut self) -> Result<(), Error<B::Error>> {
        let mut reg = CfgRegA::read(self).await?;
        reg.set_reboot(1);
        reg.write(self).await?;

        self.tim.delay_ms(20).await;

        Ok(())
    }

    /// Set Reboot memory content.
    ///
    /// If val is 1: Reload the calibration parameters.
    #[deprecated(since = "2.0.0", note = "please use reboot")]
    pub async fn boot_set(&mut self, val: u8) -> Result<(), Error<B::Error>> {
        let mut reg = CfgRegA::read(self).await?;
        reg.set_reboot(val);
        reg.write(self).await?;

        Ok(())
    }
    /// Reboot memory content. Reload the calibration parameters.
    #[deprecated(since = "2.0.0", note = "please use reboot")]
    pub async fn boot_get(&mut self) -> Result<u8, Error<B::Error>> {
        let val: u8 = CfgRegA::read(self).await?.reboot();

        Ok(val)
    }
    /// Set Selftest mode.
    pub async fn self_test_set(&mut self, val: u8) -> Result<(), Error<B::Error>> {
        let mut reg = CfgRegC::read(self).await?;
        reg.set_self_test(val);
        reg.write(self).await?;

        Ok(())
    }
    /// Get Selftest configuration.
    pub async fn self_test_get(&mut self) -> Result<u8, Error<B::Error>> {
        let val: u8 = CfgRegC::read(self).await?.self_test();

        Ok(val)
    }
    /// Set Big/Little Endian data format.
    pub async fn data_format_set(&mut self, val: Ble) -> Result<(), Error<B::Error>> {
        let mut reg = CfgRegC::read(self).await?;
        reg.set_ble(val as u8);
        reg.write(self).await?;

        Ok(())
    }
    /// Get Big/Little Endian data format.
    pub async fn data_format_get(&mut self) -> Result<Ble, Error<B::Error>> {
        let reg = CfgRegC::read(self).await?;

        let val = Ble::try_from(reg.ble()).unwrap_or_default();
        Ok(val)
    }
    /// Get info about device status.
    pub async fn status_get(&mut self) -> Result<StatusReg, Error<B::Error>> {
        let val = StatusReg::read(self).await?;

        Ok(val)
    }
    /// Set the interrupt block recognition checks data after/before the
    /// hard-iron correction to discover the interrupt.
    pub async fn offset_int_conf_set(&mut self, val: IntOnDataoff) -> Result<(), Error<B::Error>> {
        let mut reg = CfgRegB::read(self).await?;
        reg.set_int_on_dataoff(val as u8);
        reg.write(self).await?;

        Ok(())
    }
    /// Retrive the interrupt block recognition checks data after/before the
    /// hard-iron correction to discover the interrupt.
    pub async fn offset_int_conf_get(&mut self) -> Result<IntOnDataoff, Error<B::Error>> {
        let reg = CfgRegB::read(self).await?;

        let val = IntOnDataoff::try_from(reg.int_on_dataoff()).unwrap_or_default();
        Ok(val)
    }
    /// Set Data-ready signal on INT_DRDY pin.
    pub async fn drdy_on_pin_set(&mut self, val: u8) -> Result<(), Error<B::Error>> {
        let mut reg = CfgRegC::read(self).await?;
        reg.set_drdy_on_pin(val);
        reg.write(self).await?;

        Ok(())
    }
    /// Get Data-ready signal on INT_DRDY pin.
    pub async fn drdy_on_pin_get(&mut self) -> Result<u8, Error<B::Error>> {
        let val: u8 = CfgRegC::read(self).await?.drdy_on_pin();

        Ok(val)
    }
    /// Set Interrupt signal on INT_DRDY pin.
    pub async fn int_on_pin_set(&mut self, val: u8) -> Result<(), Error<B::Error>> {
        let mut reg = CfgRegC::read(self).await?;
        reg.set_int_on_pin(val);
        reg.write(self).await?;

        Ok(())
    }
    /// Get Interrupt signal on INT_DRDY pin.
    pub async fn int_on_pin_get(&mut self) -> Result<u8, Error<B::Error>> {
        let val: u8 = CfgRegC::read(self).await?.int_on_pin();

        Ok(val)
    }
    /// Set Interrupt generator configuration register.
    pub async fn int_gen_conf_set(&mut self, val: &IntCrtlReg) -> Result<(), Error<B::Error>> {
        val.write(self).await
    }
    /// Get Interrupt generator configuration register.
    pub async fn int_gen_conf_get(&mut self) -> Result<IntCrtlReg, Error<B::Error>> {
        IntCrtlReg::read(self).await
    }
    /// Get Interrupt generator source register.
    pub async fn int_gen_source_get(&mut self) -> Result<IntSourceReg, Error<B::Error>> {
        IntSourceReg::read(self).await
    }
    /// Set User-defined threshold value for xl interrupt event on generator.
    ///
    /// Data format is the same as output data raw: two's complement with
    /// 1LSb = 1.5mg.
    pub async fn int_gen_threshold_set(&mut self, val: u16) -> Result<(), Error<B::Error>> {
        IntThs::new().with_threshold(val).write(self).await
    }
    /// Get User-defined threshold value for xl interrupt event on generator.
    ///
    /// Data format is the same as output data raw: two�s complement with
    /// 1LSb = 1.5mg.
    pub async fn int_gen_threshold_get(&mut self) -> Result<u16, Error<B::Error>> {
        Ok(IntThs::read(self).await?.threshold())
    }
    /// Set SPI Serial Interface Mode.
    ///
    /// If Sim::Spi4Wire enable SDO line on pin 7.
    pub async fn spi_mode_set(&mut self, val: Sim) -> Result<(), Error<B::Error>> {
        let mut reg_val = CfgRegC::read(self).await?;
        reg_val.set_four_wire_spi(val as u8);
        reg_val.write(self).await?;

        Ok(())
    }

    /// Get SPI Serial Interface Mode.
    pub async fn spi_mode_get(&mut self) -> Result<Sim, Error<B::Error>> {
        let reg_val = CfgRegC::read(self).await?;

        let val = Sim::try_from(reg_val.four_wire_spi()).unwrap_or_default();
        Ok(val)
    }

    /// Disable/Enable I2C interface.
    ///
    /// As default I2C interface is enabled, use this function to switch if needed
    pub async fn i2c_interface_set(&mut self, val: I2cSwitch) -> Result<(), Error<B::Error>> {
        let mut reg = CfgRegC::read(self).await?;
        reg.set_i2c_dis(val as u8);
        reg.write(self).await?;

        Ok(())
    }
    /// Get I2C interface configuration (enable/disable).
    pub async fn i2c_interface_get(&mut self) -> Result<I2cSwitch, Error<B::Error>> {
        let reg = CfgRegC::read(self).await?;

        let val = I2cSwitch::try_from(reg.i2c_dis()).unwrap_or_default();
        Ok(val)
    }
}

#[bisync]
pub fn from_lsb_to_mgauss(lsb: i16) -> f32 {
    lsb as f32 * 1.5
}

#[bisync]
pub fn from_lsb_to_celsius(lsb: i16) -> f32 {
    lsb as f32 / 8.0 + 25.0
}

#[repr(u8)]
#[derive(Clone, Copy, PartialEq)]
#[bisync]
pub enum I2CAddress {
    I2cAdd = 0x1E,
}

#[bisync]
pub const ID: u8 = 0x40;
