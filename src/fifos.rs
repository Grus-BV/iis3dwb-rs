use crate::register::*;
use core::{convert::TryInto, str::ParseBoolError};
use super::SPI_READ;
use crate::{IIM42652, spi, OutputPin};
use modular_bitfield::{prelude::*, private::PushBuffer};


#[derive(BitfieldSpecifier,Clone, Copy, Debug)]
#[bits = 2]
pub enum TimestampType {
    NoTstamp = 0b00,
    None,
    OdrTstamp = 0b10,
    FsyncTstamp = 0b11,
}

#[bitfield(bits = 8)]
#[derive(BitfieldSpecifier,Copy, Clone, Debug)]
pub struct Header {
    pub odr_gyro_different:bool,
    pub odr_accel_different:bool,
    #[bits = 2]
    pub timestamp_type: TimestampType,
    pub extended_20bit_data: bool,
    pub gyro_available: bool,
    pub accel_available: bool,
    pub fifo_is_empty:bool,
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

#[derive(Debug, Copy, Clone)]
pub struct AccelGyroPacketBigEndian {
    pub header: Header,
    pub accel_x: i16,
    pub accel_y: i16,
    pub accel_z: i16,
    pub gyro_x: i16,
    pub gyro_y: i16,
    pub gyro_z: i16,
    pub temp: u8,
    pub tstamp: u16,
}

impl AccelGyroPacketBigEndian {
    /// We need to handle endiannes here.
    pub fn from_bytes( bytes: &[u8]) -> AccelGyroPacketBigEndian {
        AccelGyroPacketBigEndian { header: Header::from_bytes([bytes[0]]), 
                          accel_x: ((bytes[2] as u16) + ((bytes[1] as u16) << 8)) as i16, 
                          accel_y: ((bytes[4] as u16) + ((bytes[3] as u16) << 8)) as i16, 
                          accel_z: ((bytes[6] as u16) + ((bytes[5] as u16) << 8)) as i16, 
                          gyro_x:  ((bytes[8] as u16)  + ((bytes[7] as u16) << 8)) as i16, 
                          gyro_y:  ((bytes[10] as u16)  + ((bytes[9] as u16) << 8)) as i16, 
                          gyro_z:  ((bytes[12] as u16) + ((bytes[12] as u16) << 8)) as i16, 
                          temp:    bytes[13] , 
                          tstamp:   (bytes[15] as u16) + ((bytes[14] as u16) << 8),
                        }
    }
}


#[bitfield(bits = 16)]
#[derive(BitfieldSpecifier, Debug, Copy, Clone, Eq, PartialEq)]
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
#[derive(BitfieldSpecifier, Copy, Clone, Debug, Eq, PartialEq)]
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

#[bitfield(bits=8)]
#[derive(BitfieldSpecifier, Copy, Clone, Debug, Eq, PartialEq)]
pub struct FifoINTFConfigReg {
    #[skip] __: B4,
    pub sensor_data_big_endian: bool,
    pub fifo_count_big_endian: bool,
    pub fifo_count_in_records: bool,
    pub fifo_hold_last_data_en: bool,
}

impl Default for FifoINTFConfigReg {
    fn default() -> Self {
        Self::new()
            .with_fifo_hold_last_data_en(false)
            .with_fifo_count_in_records(false)
            .with_fifo_count_big_endian(true)
            .with_sensor_data_big_endian(true)
    }
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
#[derive(BitfieldSpecifier, Copy, Clone, Debug, Eq, PartialEq, Default)]
pub struct FifoConfigMode {
    #[skip] __: B6,
    pub mode: FifoMode,
}

#[derive( Copy, Clone, Debug, Eq, PartialEq, Default)]
pub struct FifoConfig {
    pub mode:FifoConfigMode,
    pub watermark: Watermark,
    pub config_reg: FifoConfigReg,
    pub intf_config_reg: FifoINTFConfigReg,
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
        self.write_reg(Register::FIFO_CONFIG.addr(),config.mode.into_bytes()[0]); //
        self.write_reg(Register::INTF_CONFIG0.addr(), config.intf_config_reg.into_bytes()[0])//
    }
    pub fn get_fifo_config(&mut self) -> FifoConfig {
        let mut config = FifoConfig::default();
        let mut buffer =  [0u8];
        let buffer_2 =  [0u8;2];
        self.read_reg(Register::FIFO_CONFIG1.addr(), &mut buffer);
        config.config_reg = FifoConfigReg::from_bytes(buffer);
        self.read_reg(Register::FIFO_CONFIG2.addr(), &mut [buffer_2[0]]);
        self.read_reg(Register::FIFO_CONFIG3.addr(), &mut [buffer_2[1]]);
        config.watermark = Watermark::from_bytes(buffer_2);
        self.read_reg(Register::FIFO_CONFIG.addr(), &mut buffer);
        config.mode = FifoConfigMode::from_bytes(buffer);
        self.read_reg(Register::INTF_CONFIG0.addr(),&mut buffer);
        config.intf_config_reg= FifoINTFConfigReg::from_bytes(buffer);
        config
    }
    pub fn set_watermark(&mut self, watermark:Watermark){
        self.write_reg(Register::FIFO_CONFIG2.addr(),watermark.into_bytes()[0]); //
        self.write_reg(Register::FIFO_CONFIG3.addr(),watermark.into_bytes()[1]); //
    }
    pub fn flush_fifo(&mut self){
       // changes mode to Bypass 
        let acc_mode = self.get_acc_mode(); 
        let gyro_mode = self.get_gyro_mode();
        if (acc_mode == AccelerometerMode::Off && gyro_mode == GyroMode::Off){
            let mut fifo_cfg = self.get_fifo_config();
            fifo_cfg.mode.set_mode(FifoMode::Bypass);
            self.configure_fifo(fifo_cfg);
        }
        else {
            self.modify_register(Register::SIGNAL_PATH_RESET.addr(), 
                                FIFO_FLUSH, 
                                 1).unwrap();
        }
    }

    // here, burst reading did not work.
    pub fn unread_data_count(&mut self) -> FifoCount {
        let mut buffer_l = [0xaau8];  
        let mut buffer_h = [0xaau8];  
        self.read_reg(Register::FIFO_COUNTL.addr(),
                &mut buffer_l);  
        self.read_reg(Register::FIFO_COUNTH.addr(),
                &mut buffer_h);
        FifoCount::from_bytes([buffer_l[0],buffer_h[0]])
    }

    pub fn overrun(&self) -> bool {
        // fifo status 2
        todo!();
    }
 
    pub fn fifo_read(&mut self) -> [u8;1400] {
        let mut bytes =  [0u8;1401];
        bytes[0] = Register::FIFO_DATA.addr() | SPI_READ;
        self.read(&mut bytes);
        bytes[1..1401].try_into().unwrap() // this copies
    } 
    // TODO sort out error propogation
    //CURSED Fix the fact that this also returns the command sent@!@!
    pub fn fifo_read_to_bytes(&mut self, bytes: &mut [u8]) -> () {
        let mut cmd = [Register::FIFO_DATA.addr() | SPI_READ];
        self.read_with_cmd(&mut cmd, bytes);
    }
} 