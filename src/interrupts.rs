//! Since interrupts are constructed slightly different, 
//! I have separate implementations for each.
//! Might later alter the implementation that does not expose this.
//! Now, we have INT1 and INT2 as different entities, unrelated. 

use crate::register::*;
use crate::{IIS3DWB, spi, OutputPin};
use accelerometer::Error;

#[derive(Copy, Clone, Debug, Eq, PartialEq)]
#[repr(u8)]
pub enum InterruptSource1 {
    AccDataReady = 0b0000_0001, 
    BootStatus = 0b0000_0100, 
    FifoThreshold = 0b0000_1000, 
    FifoOverrun = 0b0001_0000, 
    FifoFull = 0b0010_0000, 
    BDRCounter = 0b0100_0000, 
}

impl InterruptSource1{
    pub const fn raw(self) -> u8 {
        self as u8
    }
}

/// An easy way to associate these bools to the enums?
pub struct InterruptConfigSrc1 {
    AccDataReady :bool,
    BootStatus :bool,
    FifoThreshold  :bool,
    FifoOverrun :bool,
    FifoFull :bool,
    BDRCounter :bool,
}
impl InterruptConfigSrc1 {
    pub const fn none()-> Self{
        Self {
            AccDataReady: false,
            BootStatus: false,
            FifoThreshold : false,
            FifoOverrun: false,
            FifoFull: false,
            BDRCounter: false,
        }
    }
    pub const fn from_raw(val:u8)-> Self{
        let mut self_init = InterruptConfigSrc1::none();
        if val & InterruptSource1::BDRCounter.raw() > 0 {self_init.BDRCounter = true;}
        if val & InterruptSource1::FifoFull.raw() > 0 {self_init.FifoFull = true;}
        if val & InterruptSource1::FifoOverrun.raw() > 0 {self_init.FifoOverrun = true;}
        if val & InterruptSource1::FifoThreshold.raw() > 0 {self_init.FifoThreshold = true;}
        if val & InterruptSource1::BootStatus.raw() > 0 {self_init.BootStatus = true;}
        if val & InterruptSource1::AccDataReady.raw() > 0 {self_init.AccDataReady = true;}
        self_init
    }
    pub const fn raw(self) -> u8 {
        let mut iters = 0u8;
        if self.AccDataReady {iters += InterruptSource1::AccDataReady.raw()};
        if self.BootStatus {iters += InterruptSource1::BootStatus.raw()};
        if self.FifoThreshold {iters += InterruptSource1::FifoThreshold.raw()};
        if self.FifoOverrun {iters += InterruptSource1::FifoOverrun.raw()};
        if self.FifoFull {iters += InterruptSource1::FifoFull.raw()};
        if self.BDRCounter {iters += InterruptSource1::BDRCounter.raw()};
        iters
    }
}

impl Default for InterruptConfigSrc1{
    fn default() -> Self {
        Self::none()
    }
}

pub struct Interrupt1 {
    cfg: InterruptConfigSrc1,
}
impl Default for Interrupt1{
    fn default() -> Self {
        Self{ cfg: InterruptConfigSrc1::default()}
    }
}

impl<SPI,CS,E,PinError> IIS3DWB <SPI,CS> 
where 
    SPI: spi::Transfer<u8, Error=E> + spi::Write<u8, Error=E>,
    CS: OutputPin <Error= PinError>
{
    pub fn set_interrupt_1 (&mut self, int1: Interrupt1) -> Result<(),E> {
        let requested_ints_cfg = int1.cfg.raw();
        self.write_reg(Register::INT1_CTRL.addr(), requested_ints_cfg);
        Ok(())
    } 

    /// We are enabling all interrupts here, TODO, .
    pub fn enable_all_interrupts(&self){
        self.write_reg(Register::INTERRUPTS_EN.addr(), INTERRUPTS_EN);
    }

    
}