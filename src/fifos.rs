use crate::register::*;
use crate::{IIS3DWB, spi, OutputPin};

// 
// In IIS3DWB, Interrupts are distinct from Wake up sources
// Therefore, we need to implement a wake up lib. 
// 


// Is this crazy? W8 is at one register and W[7:0] at another.
#[derive(Debug, Copy, Clone)]
pub struct Watermark{
    lsb: u8,
    hsb: bool,
}

// Cannot be higher than the FIFO size, 3000 bytes.
impl Watermark {
    pub fn from_bytes(watermark_u16: u16) -> Self {
        Self{
            lsb: watermark_u16 as u8, 
            hsb:((watermark_u16 >> 8) != 0) as bool,
        }
    }
}


#[derive(Copy, Clone, Debug, Eq, PartialEq)]
#[repr(u8)]
pub enum FifoTimestampDecimation {
    Omitted = 0b00,
    Decimation1 = 0b01,
    Decimation8 = 0b10,
    Decimation32 = 0b11,
}

#[derive(Copy, Clone, Debug, Eq, PartialEq)]
#[repr(u8)]
pub enum FifoAccBatchDataRate {
    Omitted = 0b0000,
    BDR26667Hz = 0b1010,
}

#[derive(Copy, Clone, Debug, Eq, PartialEq)]
#[repr(u8)]
pub enum FifoTempBatchDataRate {
    Omitted = 0b00,
    BDR104Hz   = 0b11,
}

#[derive(Copy, Clone, Debug, Eq, PartialEq)]
#[repr(u8)]
pub enum FifoMode{
    Disabled = 0b000,
    FifoMode = 0b001,
    ContinuousToFifo = 0b011,
    BypassToContinuous = 0b100,
    Continuous = 0b110,
    BypassToFifo = 0b111,
}
impl FifoMode {
    pub const fn raw(self) -> u8 {
        self as u8
    }
 }


#[derive(Debug, Copy, Clone)]
pub struct FifoConfig
{
    pub mode: FifoMode,
    pub watermark:Watermark,
    pub stop_in_watermark: bool,
    pub temperature: FifoTempBatchDataRate,
    pub timestamp: FifoTimestampDecimation,
    pub acceleration: FifoAccBatchDataRate
}

impl Default for FifoConfig {
    fn default() -> Self {
        Self{
            mode: FifoMode::Disabled,
            watermark:Watermark::from_bytes(0u16),
            stop_in_watermark:false,
            temperature: FifoTempBatchDataRate::Omitted,
            timestamp: FifoTimestampDecimation::Omitted,
            acceleration: FifoAccBatchDataRate::Omitted,
        }
    }
}

impl FifoConfig {
    fn as_registers(&self) -> [u8;4] {
        let mut registers = [0;4];
        registers[0] = self.watermark.lsb as u8;
        registers[1] = self.watermark.hsb as u8 & FIFO_WTM8 + 
                       self.stop_in_watermark as u8 & STOP_ON_WTM << 7;
        registers[2] = self.acceleration as u8;
        registers[3] = self.mode as u8 & FIFO_MODE_MASK +
                       (self.temperature as u8 & ODR_T_MASK) << 4;
                       (self.timestamp as u8 & DEC_TS_MASK) << 6;
        registers
    }
    fn from_registers(&self, registers: &[u8;4]) {
    }
}


impl <SPI,CS,E,PinError> IIS3DWB <SPI,CS>
where 
    SPI: spi::Transfer<u8, Error=E> + spi::Write<u8, Error=E>,
    CS: OutputPin <Error= PinError>
{
    pub fn configure_fifo(&mut self, config: FifoConfig){
        let registers = config.as_registers();
        self.write(Register::FIFO_CTRL_1.addr(),registers); //TODOTODO
    }
    pub fn get_config(&self) -> FifoConfig{
        todo!();        
    }
    pub fn set_fifo_mode(&mut self, mode: FifoMode){
        self.modify_register(Register::FIFO_CTRL_4.addr(), 
                                  FIFO_MODE_MASK, 
                              mode.raw());
    }
    pub fn set_watermark(&mut self, watermark:Watermark){
        self.write_reg(Register::FIFO_CTRL_1.addr(), 
                        watermark.lsb as u8);
        self.modify_register(Register::FIFO_CTRL_2.addr(), 
                            FIFO_WTM8, 
                            watermark.hsb as u8);   
    }
    pub fn flush_fifo(&mut self){
       // changes mode to Bypass 
        todo!();
    }
    pub fn unread_data_count(&mut self) -> u16 {
        let mut buffer = [0u8;2];  // Chip increments address for the second
        self.read_reg(Register::FIFO_STATUS1.addr(),
                        &mut buffer);
        buffer[0] as u16 + ((buffer[1] & 0b0000_0011) as u16) << 8
    }
    pub fn watermark_exceeded(&mut self) -> bool {
        let mut buffer = [0u8];  // Chip increments address for the second
        self.read_reg(Register::FIFO_STATUS1.addr(),
                        &mut buffer);
        (buffer[0] >> 7) == 1
    }
    pub fn overrun(&self) -> bool {
        // fifo status 2
        todo!();
    }


} 