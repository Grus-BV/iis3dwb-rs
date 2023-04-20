#![no_std]

mod register;
//mod interrupts;
mod wakeups;
mod fifos;   

/// Is there a way to improve this to non blocking? Does that require async? 
pub use embedded_hal::blocking::spi;

use embedded_hal::digital::v2::OutputPin;
//pub use interrupts::{Interrupt1,InterruptConfigSrc1,InterruptSource1};
pub use accelerometer::{
    Accelerometer,
    AccelerometerError,
    RawAccelerometer,
    error,
    Error,
    vector::{I16x3,F32x3,I32x3},
    Vector,
};
use core::ops::Index;
use micromath::generic_array::{arr, GenericArray, typenum::U3};


pub use fifos::{FifoConfig,
                FifoMode,
                Watermark,AccelGyroPacketBigEndian};
// use interrupts::{Interrupt1, Interrupt2};
// use wakeups::{WakeUp};
pub use register::{Bank, DataRate, Mode, Range, Register, Register_3, Register_1,
                    InternalFreqFinetuned, Timestamp,
                    AccelerometerMode, GyroMode};
use register::{DEVICE_ID,
                MASK_BANK_SEL,
                MASK_ACCEL_MODE,
                MASK_ACCEL_ODR,
                MASK_ACCEL_UI_FS_SEL,
                MASK_GYRO_MODE,
                MASK_GYRO_ODR,
                // MASK_GYRO_UI_FS_SEL,
                SOFT_RESET_CONFIG,
                PIN1_PU_EN,
                TMST_EN,
                TMST_TO_REGS_EN,
                TMST_STROBE,
            };

use core::fmt::Debug;


pub const SPI_READ: u8 = 0b1000_0000;
const SPI_WRITE: u8 = 0x0000_0000;

pub struct Config {
    pub mode: Mode,
    pub datarate: DataRate,
    pub enable_x_axis: bool,   
    pub enable_y_axis: bool,    
    pub enable_z_axis: bool,   
    pub enable_temp:bool,
    pub range: Range,
    // pub interrupt1: Interrupt1,
    // pub fifo: FifoConfig,
    // pub interrupt2: Range,
    // pub wake_up: WakeUp,
}
 
impl Default for Config{
    fn default() -> Self {
        Self {
            mode: Mode::HighResolution,
            datarate: DataRate::Hz_32000,
            enable_x_axis: true,   
            enable_y_axis: true,   
            enable_z_axis: true,   
            enable_temp: true,
            range: Range::G2,
            // interrupt1: Interrupt1::default(),
            // fifo: FifoConfig::default(),
        }
    }
}

/// Driver's structure
pub struct IIM42652 <SPI, CS>{
    spi : SPI,
    cs  : CS,
    // configuration 
    range : Range,
    // interrupt1: Interrupt1,
    // fifo: FifoConfig,
}

/// Driver's implementation for given SPI and CS
impl<SPI, CS, E, PinError> IIM42652<SPI,CS>
where 
    SPI: spi::Transfer<u8, Error=E> + spi::Write<u8, Error=E>,
    CS: OutputPin <Error= PinError>
{
    pub fn new(spi: SPI, cs: CS, config: &Config) -> Result<Self,E> {
        let mut IIM42652 = IIM42652 {
            spi,
            cs, 
            range: config.range,
            // interrupt1: config.interrupt1,
            // fifo: config.fifo,
        };
        //IIM42652.reset();
        cortex_m::asm::delay(10000);
        let id= IIM42652.get_device_id();
        if id != DEVICE_ID {
            // raise
        }
        IIM42652.set_range(IIM42652.range);
        IIM42652.set_data_rate(config.datarate);
        // IIM42652.set_timestamp_en(true);
        // IIM42652.configure_fifo(IIM42652.fifo);
        // IIM42652.set_interrupt_1(IIM42652.interrupt1);
        Ok(IIM42652)
    }  

    fn switch_reg_bank(&mut self, bank: Bank){
        self.modify_register(Register::REG_BANK_SEL.addr(),
                            MASK_BANK_SEL,
                            bank.bits()).unwrap();
    }

    pub fn set_pull_up_miso(&mut self, mode: u8){
        self.switch_reg_bank(Bank::Bank3);
        self.modify_register(Register_3::PU_PD_CONFIG2.addr(),
                            PIN1_PU_EN,
                            mode).unwrap();
                            
        self.switch_reg_bank(Bank::Bank0);
    }

    pub fn set_acc_mode(&mut self, mode: AccelerometerMode) {
        self.modify_register( Register::PWR_MGMT0.addr(), 
                             MASK_ACCEL_MODE, 
                             mode.bits()).unwrap();
        cortex_m::asm::delay(12800); // wait 200us
    }  

    pub fn get_acc_mode(&mut self) -> AccelerometerMode {
        let reg = Register::PWR_MGMT0.addr();
        let mut output = [0u8];
        self.read_reg(reg, &mut output);
        AccelerometerMode::from_bytes(output[0])
    }  

    pub fn set_gyro_mode(&mut self, mode: GyroMode) {
        self.modify_register( Register::PWR_MGMT0.addr(), 
                             MASK_GYRO_MODE, 
                             mode.bits()).unwrap();
        cortex_m::asm::delay(12800); // wait 200us

    }  

    pub fn get_gyro_mode(&mut self)-> GyroMode {
        let reg = Register::PWR_MGMT0.addr();
        let mut output = [0u8];
        self.read_reg(reg, &mut output);
        GyroMode::from_bytes(output[0] >> 2)  // TODO Eliminate these one of these things
    }  

    pub fn stop(&mut self) {
        self.set_acc_mode(AccelerometerMode::Off);
        self.set_gyro_mode(GyroMode::Off);
    }
    
    pub fn reset(&mut self) {
        self.modify_register( Register::DEVICE_CONFIG.addr(), 
                            SOFT_RESET_CONFIG, 
                            0b1).unwrap();        
    }    

    pub fn resetting(&mut self) -> bool {
        unimplemented!();
    }

    pub fn set_range(&mut self, range: Range) {
        defmt::debug!("RANGE bits:{=u8:b}", range.bits());
        self.range = range;
        self.modify_register( Register::ACCEL_CONFIG0.addr(), 
                                MASK_ACCEL_UI_FS_SEL, 
                              range.bits()).unwrap();
    }

    pub fn set_data_rate(&mut self, odr: DataRate) {
        defmt::debug!("ODR bits:{=u8:b}",odr.bits());
        self.modify_register( Register::ACCEL_CONFIG0.addr(), 
                                MASK_ACCEL_ODR, 
                              odr.bits()).unwrap();
        self.modify_register( Register::GYRO_CONFIG0.addr(), 
                                MASK_GYRO_ODR, 
                            odr.bits()).unwrap();
    }

    pub fn disable_i2c(&mut self) {
        unimplemented!();
    }

    /// Get the device ID
    pub fn get_device_id(&mut self) -> u8 {
        let reg = Register::WHO_AM_I.addr();
        let mut output = [1u8];
        self.read_reg(reg, &mut output);
        output[0]
    }

    /// Returns the raw contents of the temperature registers
    pub fn read_temp_raw(&mut self) -> i16 {
        let mut bytes = [Register::TEMP_DATA1_UI.addr() | SPI_READ, 0, 0];
        self.read(&mut bytes);

        let temp_h = ((bytes[1] & 0xFF) as u16) << 8;
        let temp_l = (bytes[2] as u16) & 0x00FF;

        (temp_h | temp_l) as i16
    }

    /// Returns temperature in Celcuis
      pub fn get_temp_celcius(&mut self) -> f32 {
        let raw_temp = self.read_temp_raw();
        (raw_temp / 132.48) + 25.0
    }

    fn write_reg(&mut self, reg: u8, value: u8) {
        let mut bytes = [ reg | SPI_WRITE, value];
        defmt::debug!("write cmd spi_bytes: {=[u8]:x}", bytes);
        self.cs.set_low().ok();
        self.spi.write(&mut bytes).ok();
        self.cs.set_high().ok();
    }

    fn read_reg(&mut self, reg: u8, buffer: &mut [u8]) {
        let mut bytes = [ reg | SPI_READ, 0];
        defmt::debug!("read cmd spi_bytes: {=[u8]:x}", bytes);
        self.cs.set_low().ok();
        self.spi.transfer(&mut bytes).ok();
        defmt::debug!("after xfer: {=[u8]:x}", bytes);
        self.cs.set_high().ok();
        buffer[0] = bytes[1];
    }

    fn read(&mut self, bytes: &mut [u8]) {
        self.cs.set_low().ok();
        defmt::trace!("Read before xfer {=[u8]:x}",bytes);
        self.spi.transfer(bytes).ok();
        defmt::trace!("Read after xfer {=[u8]:x}",bytes);
        self.cs.set_high().ok();
    }
    // TODO replace all reads to this, and make this read.
    fn read_with_cmd(&mut self, cmd: &mut [u8], bytes: &mut [u8]) {
        self.cs.set_low().ok();
        defmt::trace!("Read before xfer {=[u8]:x}",bytes);
        self.spi.transfer(cmd).ok();
        self.spi.transfer(bytes).ok();
        defmt::trace!("Read after xfer {=[u8]:x}",bytes);
        self.cs.set_high().ok();
    }


    fn read_consecutive_regs(){
        unimplemented!();
    }

    fn modify_register(&mut self, reg: u8, mask: u8, val: u8) -> Result<(),()>
    {
        if mask == 0 {return Err(());}
        let mut register_value=[0u8]; 
        self.read_reg( reg,&mut register_value);
        let mut bitshift = 0u8;
        let mut sacrificial_mask = mask.clone();
        //ugly dangerous loop calculating shifts 
        while sacrificial_mask != 0 {
            if sacrificial_mask & 0x01 == 0x01 { break; }
            else{
                sacrificial_mask = sacrificial_mask >> 1;
                bitshift += 1;
            }
        }

        // 0x0010_0110 <- init
        // 0x0111_0000 <- mask
        // 0x0000_0101 <- value
        // 
        // clear init with clear mask
        // 0x1000_1111 & 0x0010_0110 = 0x0000_0110
        // 
        // set init with set mask
        // 0x0000_0110 | (val << 4) which is 0b0101_0000 = 0b0101_0110

        let clear_mask = !mask;
        let set_mask = val << bitshift;
        register_value[0] = (register_value[0] & clear_mask) | set_mask;
        self.write_reg( reg, register_value[0]);
        Ok(())
    }
    

    pub fn get_odr(&mut self) -> DataRate {
        let mut buffer = [0u8];
        self.read_reg(Register::ACCEL_CONFIG0.addr(), &mut buffer);
        DataRate::from_raw(buffer[0] & MASK_ACCEL_ODR)
    }

    pub fn get_timestamp (&mut self) -> Timestamp {
        self.modify_register( Register::SIGNAL_PATH_RESET.addr(), 
                             TMST_STROBE, 
                              0x1).unwrap();
        self.switch_reg_bank(Bank::Bank1);
        let mut bytes = [0u8; 3+1];
        bytes[0] = Register_1::TMSTVAL0.addr() | SPI_READ;
        self.read(&mut bytes);
        self.switch_reg_bank(Bank::Bank0);
        let tstamp = bytes[1] as u32 + 
                        (bytes[2] as u32) * 0x100+
                        ((bytes[3] & 0b0000_1111) as u32) * 0x10000;
        Timestamp(tstamp)
    }

    pub fn set_if_increment (&mut self, state: bool){
        unimplemented!()
    }

    pub fn set_rounding (&mut self, state: bool){
        unimplemented!()
    }
    
    pub fn set_timestamp_en (&mut self, state: bool){
        self.modify_register( Register::TMST_CONFIG.addr(), 
        TMST_EN, state as u8).unwrap();
        self.modify_register( Register::TMST_CONFIG.addr(), 
        TMST_TO_REGS_EN, state as u8).unwrap();
    }

    pub fn reset_timestamp (&mut self){
        unimplemented!()
    }
    
    pub fn one_shot_acc(&mut self) -> (f32,f32,f32) {
        // TODO read multiple regs
        let mut bytes = [0u8; 6+1];
        bytes[0] = Register::ACCEL_DATA_X1_UI.addr() | SPI_READ;
        self.read(&mut bytes);

        let x_u16 = bytes[2] as u16 + (bytes[1] as u16) * 256;
        let y_u16 = bytes[4] as u16 + (bytes[3] as u16) * 256;
        let z_u16 = bytes[6] as u16 + (bytes[5] as u16) * 256;

        let x = x_u16 as i16;
        let y = y_u16 as i16;
        let z = z_u16 as i16;

        (x as f32 * 0.061, y as f32 * 0.061, z as f32 * 0.061)
    }

    // destroy the instance and return the spi bus and its cs pin
    pub fn destroy(self) -> (SPI, CS) {
        (self.spi, self.cs)
    }
}

impl<SPI, CS, E, PinError> RawAccelerometer<I16x3> for IIM42652 <SPI, CS>
where
    SPI: spi::Transfer<u8, Error=E> + spi::Write<u8, Error=E>,
    CS: OutputPin<Error = PinError>,
    E: Debug
{
    type Error = E;

    /// Gets acceleration vector reading from the accelerometer
    /// Returns a 3D vector with x,y,z, fields in a Result
    fn accel_raw(&mut self) -> Result<I16x3, Error<E>> {
        let mut bytes = [0u8; 6+1];
        bytes[0] = Register::ACCEL_DATA_X1_UI.addr() | SPI_READ;
        self.read(&mut bytes);

        // TEST THIS 
        let x = (bytes[2] as u16 + (bytes[1] as u16) * 256 ) as i16;
        let y = (bytes[4] as u16 + (bytes[3] as u16) * 256 ) as i16;
        let z = (bytes[6] as u16 + (bytes[5] as u16) * 256 ) as i16;

        Ok(I16x3::new(x, y, z))
    }
}

impl<SPI, CS, E, PinError> Accelerometer for IIM42652 <SPI, CS>
where
    SPI: spi::Transfer<u8, Error=E> + spi::Write<u8, Error=E>,
    CS: OutputPin<Error = PinError>,
    E: Debug
{
    type Error = E;

    fn accel_norm(&mut self) -> Result<F32x3, Error<Self::Error>> {
        // acceleration has 10 meaningful bits
        // in 2G config 0xFFF to 0x000 has 4G range.
        // 4000 mg / 0xFFF is 
        
        let lsb_as_mg = self.range.lsb_as_mg();
        let raw_values: I16x3 = self.accel_raw().unwrap();
        let norm_values: F32x3 = F32x3::new((raw_values[0] as f64 * lsb_as_mg/1000.0) as f32, // from G to mG
                                            (raw_values[1] as f64 * lsb_as_mg/1000.0) as f32 ,
                                            (raw_values[2] as f64 * lsb_as_mg/1000.0) as f32 );
        Ok(norm_values)
    }
    

    fn sample_rate(&mut self) -> Result<f32, Error<Self::Error>> {
        Ok(self.get_odr().sample_rate())
    }
}

#[doc="This is a clone of I16x3 implementation of Vector in the accelerometer crate"]
#[derive(Copy, Clone, Debug, Default, PartialEq)]
pub struct TimestampedAcceleration {
    pub x:i16,
    pub y:i16,
    pub z:i16,
    pub t:Option<Timestamp>,
}

impl TimestampedAcceleration {
    /// Instantiate from X,Y,Z,T components
    pub fn new(x: i16, y: i16, z: i16, t: Option<Timestamp>) -> Self {
        TimestampedAcceleration { x, y, z, t }
    }
}

/// This omits timestamp, since it is not of the same type
impl Index<usize> for TimestampedAcceleration {
    type Output = i16;

    fn index(&self, i: usize) -> &i16 {
        match i {
            0 => &self.x,
            1 => &self.y,
            2 => &self.z,
            _ => panic!("index out of range")
        }
    }
}

/// This might be cursed, since I need a type that I can use with RawAccelerometer,
/// but optionally have the capability to reach the timestamp, if it is available. 
impl Vector for TimestampedAcceleration {
    type Component = i16;
    type Axes = U3;

    const MIN: i16 = core::i16::MIN;
    const MAX: i16 = core::i16::MAX;

    fn from_iter<I>(into_iter: I) -> Self
    where
        I: IntoIterator<Item = Self::Component>
    {
        let mut iter = into_iter.into_iter();

        let x = iter.next().expect("no x-axis component in slice");
        let y = iter.next().expect("no y-axis component in slice");
        let z = iter.next().expect("no z-axis component in slice");
        debug_assert!(iter.next().is_none(), "too many items in 3-axis component slice");

        Self::new(x, y, z, None)
    }

    fn get(self, i: usize) -> Option<Self::Component> {
        if i <= 2 {
            Some(self[i])
        } else {
            None
        }
    }

    fn to_array(self) -> GenericArray<i16, U3> {
        arr![i16; self.x, self.y, self.z]
    }

}

impl<SPI, CS, E, PinError> RawAccelerometer<TimestampedAcceleration> for IIM42652 <SPI, CS>
where
    SPI: spi::Transfer<u8, Error=E> + spi::Write<u8, Error=E>,
    CS: OutputPin<Error = PinError>,
    E: Debug
{
    type Error = E;

    fn accel_raw(&mut self) -> Result<TimestampedAcceleration, Error<E>> {
        
        let mut bytes = [0u8; 6+1];
        bytes[0] = Register::ACCEL_DATA_X1_UI.addr() | SPI_READ;
        self.read(&mut bytes);

        let x = (bytes[2] as u16 + (bytes[1] as u16) * 256 ) as i16;
        let y = (bytes[4] as u16 + (bytes[3] as u16) * 256 ) as i16;
        let z = (bytes[6] as u16 + (bytes[5] as u16) * 256 ) as i16;

        let t = self.get_timestamp();
        
        Ok(TimestampedAcceleration::new(x, y, z, Some(t)))
    }
}
