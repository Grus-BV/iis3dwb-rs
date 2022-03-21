use crate::register::*;
use core::{convert::TryInto, str::ParseBoolError};
use super::SPI_READ;
use crate::{IIM42652, spi, OutputPin};
use modular_bitfield::prelude::*;

#[derive(BitfieldSpecifier,Clone, Copy, Debug)]
#[bits = 2]
pub enum TimestampType {
    NoTstamp = 0b00,
    None,
    OdrTstamp = 0b10,
    FsyncTstamp = 0b11,
}

#[bitfield(bits = 8)]
#[derive(Copy, Clone, Debug)]
pub struct Header {
    odr_gyro_different:bool,
    odr_accel_different:bool,
    #[bits = 2]
    timestamp_type: TimestampType,
    extended_20bit_data: bool,
    gyro_available: bool,
    accel_available: bool,
    fifo_is_empty:bool,
}

#[derive(Debug, Copy, Clone)]
pub struct AccelPacket {
    header:Header,
    accel_x_l: u8,
    accel_x_h: u8,
    accel_y_l: u8,
    accel_y_h: u8,
    accel_z_l: u8,
    accel_z_h: u8,
    temp: u8,
}

#[bitfield(bits = 16)]
#[derive(Debug, Copy, Clone, Eq, PartialEq)]
pub struct Watermark {
    watermark: B12,
    #[skip] __: B4,
}

/// Default watermark thresold is 2000 bytes
impl Default for Watermark {
    fn default () -> Self {
        Self::new().with_watermark(0x7d0) 
    }
}

#[bitfield(bits=8)]
#[derive(Copy, Clone, Debug, Eq, PartialEq)]
pub struct FifoConfigReg {
    pub accel_en: bool,
    pub gyro_en: bool,
    pub temp_en: bool,
    pub timestamp_fsync_en: bool,
    pub hi_resolution_en: bool,
    pub watermark_interrupt_en: bool,
    pub resume_partial_read_en: bool,
    #[skip] __: B1
}

/// Any functionality needed should be enabled explicitly.
impl Default for FifoConfigReg {
    fn default() -> Self {
        Self::new()
            .with_accel_en(false)
            .with_gyro_en(false)
            .with_temp_en(false)
            .with_timestamp_fsync_en(false)
            .with_hi_resolution_en(false)
            .with_watermark_interrupt_en(false)
            .with_resume_partial_read_en(false)
    }
}

#[derive(Copy, Clone, Debug, Eq, PartialEq, BitfieldSpecifier)]
#[bits=2]
pub enum FifoMode {
    Bypass = 0b00,
    StreamToFifo = 0b01,
    StopOnFull = 0b10,
    None,
}

impl Default for FifoMode {
    fn default() -> Self {
        Self::Bypass
    }
}

#[bitfield(bits=8)]
#[derive(Copy, Clone, Debug, Eq, PartialEq, Default)]
pub struct FifoConfigMode {
    #[skip] __: B6,
    mode: FifoMode,
}

#[derive(Copy, Clone, Debug, Eq, PartialEq, Default)]
pub struct FifoConfig {
    pub mode:FifoMode,
    pub watermark: Watermark,
    pub config_reg: FifoConfigReg,
}

#[repr(u16)]
#[bitfield(bits=16)]
#[derive(Copy, Clone, Debug, Eq, PartialEq, Default)]
pub struct FifoCount {
    count_lsb: B8,
    count_hsb: B8,
}

impl <SPI,CS,E,PinError> IIM42652 <SPI,CS>
where 
    SPI: spi::Transfer<u8, Error=E> + spi::Write<u8, Error=E>,
    CS: OutputPin <Error= PinError>
{
    pub fn configure_fifo(&mut self, config: FifoConfig){
        self.write_reg(Register::FIFO_CONFIG1.addr(),config.config_reg.into_bytes()[0]); //
        self.write_reg(Register::FIFO_CONFIG2.addr(),config.watermark.into_bytes()[0]); //
        self.write_reg(Register::FIFO_CONFIG3.addr(),config.watermark.into_bytes()[1]); //
    }
    pub fn get_config(&self) -> FifoConfig{
        todo!();        
    }
    pub fn set_watermark(&mut self, watermark:Watermark){
        self.write_reg(Register::FIFO_CONFIG2.addr(),watermark.into_bytes()[0]); //
        self.write_reg(Register::FIFO_CONFIG3.addr(),watermark.into_bytes()[1]); //
    }
    pub fn flush_fifo(&mut self){
       // changes mode to Bypass 
        todo!();
    }
    pub fn unread_data_count(&mut self) -> FifoCount {
        let mut buffer = [0u8;2];  
        self.read_reg(Register::FIFO_COUNTL.addr(),
                &mut buffer);
        FifoCount::from_bytes(buffer)
    }

    pub fn overrun(&self) -> bool {
        // fifo status 2
        todo!();
    }
 
    pub fn fifo_read(&mut self) -> [u8;2000] {
        let mut bytes =  [0u8;2001];
        bytes[0] = Register::FIFO_DATA.addr() | SPI_READ;
        self.read(&mut bytes);
        bytes[1..2001].try_into().unwrap() // this copies
    }
} 