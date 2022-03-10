#![allow(unused_variables)]
use num_enum::TryFromPrimitive;

#[derive(Copy, Clone, Debug, Eq, PartialEq)]
#[repr(u8)]
pub enum Bank {
    Bank0 = 0b000,
    Bank1 = 0b001,
    Bank2 = 0b010,
    Bank3 = 0b011,
    Bank4 = 0b100,
}

impl Bank {
    pub const fn bits(self) -> u8 {
        self as u8
    }
}


pub enum AccelerometerMode {
    Off = 0b00,
    LowPower = 0b10,
    LowNoise = 0b11,
}
impl AccelerometerMode {
    pub const fn bits(self) -> u8 {
        self as u8
    }
}

pub enum GyroMode {
    Off = 0b00,
    Standby = 0b01,
    LowNoise = 0b11,
}

impl GyroMode {
    pub const fn bits(self) -> u8 {
        self as u8
    }
}

pub enum TemperatureMode {
    Off = 0b1,
    On = 0b0,
}


/// Enumerate all device registers.
#[allow(dead_code, non_camel_case_types, clippy::upper_case_acronyms)]
#[derive(Copy, Clone, Debug, Eq, PartialEq)]
#[repr(u8)]
pub enum Register {
    DEVICE_CONFIG =	0x11,
    DRIVE_CONFIG =	0x13,
    INT_CONFIG =	0x14,
    FIFO_CONFIG =	0x16,
    TEMP_DATA1_UI =	0x1D,
    TEMP_DATA0_UI =	0x1E,
    ACCEL_DATA_X1_UI =	0x1F,
    ACCEL_DATA_X0_UI =	0x20,
    ACCEL_DATA_Y1_UI =	0x21,
    ACCEL_DATA_Y0_UI =	0x22,
    ACCEL_DATA_Z1_UI =	0x23,
    ACCEL_DATA_Z0_UI =	0x24,
    GYRO_DATA_X1_UI =	0x25,
    GYRO_DATA_X0_UI = 0x26,
    GYRO_DATA_Y1_UI = 0x27,
    GYRO_DATA_Y0_UI = 0x28,
    GYRO_DATA_Z1_UI = 0x29,
    GYRO_DATA_Z0_UI = 0x2A,
    TMST_FSYNCH =	0x2B,
    TMST_FSYNCL =	0x2C,
    INT_STATUS =	0x2D,
    FIFO_COUNTH =	0x2E,
    FIFO_COUNTL =	0x2F,
    FIFO_DATA =	    0x30,
    APEX_DATA0 =	0x31,
    APEX_DATA1 =	0x32,
    APEX_DATA2 =	0x33,
    APEX_DATA3 =	0x34,
    APEX_DATA4 =	0x35,
    APEX_DATA5 =	0x36,
    INT_STATUS2 =	0x37,
    INT_STATUS3 =	0x38,
    SIGNAL_PATH_RESET =	0x4B,
    INTF_CONFIG0 =	0x4C,
    INTF_CONFIG1 =	0x4D,
    PWR_MGMT0 =	0x4E,
    GYRO_CONFIG0 =	0x4F,
    ACCEL_CONFIG0 =	0x50,
    GYRO_CONFIG1 =	0x51,
    GYRO_ACCEL_CONFIG0 =	0x52,
    ACCEL_CONFIG1 =	0x53,
    TMST_CONFIG =	0x54,
    APEX_CONFIG0 =	0x56,
    SMD_CONFIG =	0x57,
    FIFO_CONFIG1 =	0x5F,
    FIFO_CONFIG2 =	0x60,
    FIFO_CONFIG3 =	0x61,
    FSYNC_CONFIG =	0x62,
    INT_CONFIG0 =	0x63,
    INT_CONFIG1 =	0x64,
    INT_SOURCE0 =	0x65,
    INT_SOURCE1 =	0x66,
    INT_SOURCE3 =	0x68,
    INT_SOURCE4 =	0x69,
    FIFO_LOST_PKT0 =	0x6C,
    FIFO_LOST_PKT1 =	0x6D,
    SELF_TEST_CONFIG =	0x70,
    WHO_AM_I =	0x75,
    REG_BANK_SEL =	0x76,
}
 /// Enumerate all device registers.
#[allow(dead_code, non_camel_case_types, clippy::upper_case_acronyms)]
#[derive(Copy, Clone, Debug, Eq, PartialEq)]
#[repr(u8)]
pub enum Register_1 {    
    SENSOR_CONFIG0 =	0x3,
    GYRO_CONFIG_STATIC2 =	0x0B,
    GYRO_CONFIG_STATIC3 =	0x0C,
    GYRO_CONFIG_STATIC4 =	0x0D,
    GYRO_CONFIG_STATIC5 =	0x0E,
    GYRO_CONFIG_STATIC6 =	0x0F,
    GYRO_CONFIG_STATIC7 =	0x10,
    GYRO_CONFIG_STATIC8 =	0x11,
    GYRO_CONFIG_STATIC9 =	0x12,
    GYRO_CONFIG_STATIC10 =	0x13,
    XG_ST_DATA =	0x5F,
    YG_ST_DATA =	0x60,
    ZG_ST_DATA =	0x61,
    TMSTVAL0 =	0x62,
    TMSTVAL1 =	0x63,
    TMSTVAL2 =	0x64,
    INTF_CONFIG4 =	0x7A,
    INTF_CONFIG5 =	0x7B,
    INTF_CONFIG6 =	0x7C,
}

impl Register_1 {
    /// Get register address
    pub fn addr(self) -> u8 {
        self as u8
    }
}

 /// Enumerate all device registers.
 #[allow(dead_code, non_camel_case_types, clippy::upper_case_acronyms)]
 #[derive(Copy, Clone, Debug, Eq, PartialEq)]
 #[repr(u8)]
 pub enum Register_2 { 
    ACCEL_CONFIG_STATIC2 =	0x3,
    ACCEL_CONFIG_STATIC3 =	0x4,
    ACCEL_CONFIG_STATIC4 =	0x5,
    XA_ST_DATA =	0x3B,
    YA_ST_DATA =	0x3C,
    ZA_ST_DATA =	0x3D,
 }

 impl Register_2 {
    /// Get register address
    pub fn addr(self) -> u8 {
        self as u8
    }
}

 /// Enumerate all device registers.
 #[allow(dead_code, non_camel_case_types, clippy::upper_case_acronyms)]
 #[derive(Copy, Clone, Debug, Eq, PartialEq)]
 #[repr(u8)]
 pub enum Register_3 { 
    PU_PD_CONFIG1 =	0x6,
    PU_PD_CONFIG2 =	0x0E,
 }

 
impl Register_3 {
    /// Get register address
    pub fn addr(self) -> u8 {
        self as u8
    }
}
 
 /// Enumerate all device registers.
 #[allow(dead_code, non_camel_case_types, clippy::upper_case_acronyms)]
 #[derive(Copy, Clone, Debug, Eq, PartialEq)]
 #[repr(u8)]
 pub enum Register_4 { 
    FDR_CONFIG =	0x9,
    APEX_CONFIG1 =	0x40,
    APEX_CONFIG2 =	0x41,
    APEX_CONFIG3 =	0x42,
    APEX_CONFIG4 =	0x43,
    APEX_CONFIG5 =	0x44,
    APEX_CONFIG6 =	0x45,
    APEX_CONFIG7 =	0x46,
    APEX_CONFIG8 =	0x47,
    APEX_CONFIG9 =	0x48,
    APEX_CONFIG10 =	0x49,
    ACCEL_WOM_X_THR =	0x4A,
    ACCEL_WOM_Y_THR =	0x4B,
    ACCEL_WOM_Z_THR =	0x4C,
    INT_SOURCE6 =	0x4D,
    INT_SOURCE7 =	0x4E,
    INT_SOURCE8 =	0x4F,
    INT_SOURCE9 =	0x50,
    INT_SOURCE10 =	0x51,
    OFFSET_USER0 =	0x77,
    OFFSET_USER1 =	0x78,
    OFFSET_USER2 =	0x79,
    OFFSET_USER3 =	0x7A,
    OFFSET_USER4 =	0x7B,
    OFFSET_USER5 =	0x7C,
    OFFSET_USER6 =	0x7D,
    OFFSET_USER7 =	0x7E,
    OFFSET_USER8 =	0x7F,
}

impl Register {
    /// Get register address
    pub fn addr(self) -> u8 {
        self as u8
    }

    /// Is the register read-only?
    pub fn read_only(self) -> bool {
        matches!(
            self,
            | Register::INT_STATUS
            | Register::FIFO_COUNTH
            | Register::FIFO_COUNTL
            | Register::FIFO_DATA
            | Register::APEX_DATA2
            | Register::APEX_DATA3
            | Register::APEX_DATA4
            | Register::APEX_DATA5
            | Register::INT_STATUS2
            | Register::INT_STATUS3
            | Register::FIFO_LOST_PKT0
            | Register::FIFO_LOST_PKT1
            | Register::WHO_AM_I
        )
    }
}



/// Full-scale selection.
#[allow(non_camel_case_types)]
#[derive(Copy, Clone, Debug, Eq, PartialEq, TryFromPrimitive)]
#[repr(u8)]
pub enum Range {
    G16 = 0b000,
    G8 = 0b001,
    G4 = 0b010,
    G2 = 0b011,
}

impl Range {
    pub const fn bits(self) -> u8 {
        self as u8
    }

    /// Convert the range into an value in mili-g
    pub const fn as_mg(self) -> u8 {
        match self {
            Range::G16 => 186,
            Range::G8 => 62,
            Range::G4 => 32,
            Range::G2 => 16,
        }
    }
}

impl Default for Range {
    fn default() -> Self {
        Range::G2
    }
}

#[derive(Copy, Clone, Debug, Eq, PartialEq)]
pub struct Threshold(pub(crate) u8);

impl Threshold {
    /// Convert a value in multiples of the `g` constant (roughly 9.81) to a threshold.
    ///
    ///     assert_eq!(Threshold::g(Range::G2, 1.1), 69);
    #[inline(always)]
    pub fn g(range: Range, gs: f32) -> Self {
        Self::mg(range, gs * 1000.0)
    }

    #[inline(always)]
    pub fn mg(range: Range, mgs: f32) -> Self {
        let value = mgs / (range.as_mg() as f32);
        let truncated = value as u64;
        let round_up = value - (truncated as f32) > 0.5;
        let result = if round_up { truncated + 1 } else { truncated }; 
        Threshold(result as u8)
    }
}

/// Output data rate.
#[allow(non_camel_case_types)]
#[derive(Copy, Clone, Debug, Eq, PartialEq, TryFromPrimitive)]
#[repr(u8)]
pub enum DataRate {
    Hz_32000    = 0b0001,
    Hz_16000    = 0b0010,
    Hz_8000     = 0b0011,
    Hz_4000     = 0b0100,
    Hz_2000     = 0b0101,
    Hz_1000     = 0b0110,
    Hz_500      = 0b1111,
    Hz_200      = 0b0111,
    Hz_100      = 0b1000,
    Hz_50       = 0b1001,
    Hz_25       = 0b1010,
    Hz_12p5     = 0b1011,
    Hz_6p25     = 0b1100,
    Hz_3p125    = 0b1101,
    Hz_1p5625   = 0b1110,
    
}

impl DataRate {
    pub fn from_raw(bits:u8) -> Self {
        match bits {
            0b0001  => DataRate::Hz_32000  ,
            0b0010  => DataRate::Hz_16000  ,
            0b0011  => DataRate::Hz_8000   ,
            0b0100  => DataRate::Hz_4000   ,
            0b0101  => DataRate::Hz_2000   ,
            0b0110  => DataRate::Hz_1000   ,
            0b1111  => DataRate::Hz_500    ,
            0b0111  => DataRate::Hz_200    ,
            0b1000  => DataRate::Hz_100    ,
            0b1001  => DataRate::Hz_50     ,
            0b1010  => DataRate::Hz_25     ,
            0b1011  => DataRate::Hz_12p5   ,
            0b1100  => DataRate::Hz_6p25   ,
            0b1101  => DataRate::Hz_3p125  ,
            0b1110  => DataRate::Hz_1p5625 ,
            _ => unreachable!(),
        }
    }

    pub const fn bits(self) -> u8 {
        self as u8
    }

    pub const fn sample_rate(self) -> f32 {
        match self {
            DataRate::Hz_32000  => 32000.0,
            DataRate::Hz_16000  => 16000.0,
            DataRate::Hz_8000   => 8000.0,
            DataRate::Hz_4000   => 4000.0,
            DataRate::Hz_2000   => 2000.0,
            DataRate::Hz_1000   => 1000.0,
            DataRate::Hz_500    => 500.0,
            DataRate::Hz_200    => 200.0,
            DataRate::Hz_100    => 100.0,
            DataRate::Hz_50     => 50.0,
            DataRate::Hz_25     => 25.0,
            DataRate::Hz_12p5   => 12.50,
            DataRate::Hz_6p25   => 6.250,
            DataRate::Hz_3p125  => 3.1250,
            DataRate::Hz_1p5625 => 1.5625,
        }
    }
}

/// This is 
pub struct InternalFreqFinetuned (pub(crate) u8);
impl InternalFreqFinetuned {
    pub const fn raw(self) -> i8 {
        self.0 as i8
    }

    pub fn hz (self) -> f32 {
        26667.0 + (((self.0 as f32)* 0.0015)* 26667.0)
    }
}

#[derive(Copy, Clone, Debug, Eq, PartialEq)]
pub struct Duration(pub(crate) u8);

impl Duration {
    /// Convert a number of seconds into a duration. Internally a duration is represented
    /// as a multiple of `1 / ODR` where ODR (the output data rate) is of type [`DataRate`].
    #[inline(always)]
    pub fn seconds(output_data_rate: DataRate, seconds: f32) -> Self {
        let duration = output_data_rate.sample_rate() * seconds;

        Self(duration as u8)
    }

    /// Convert a number of miliseconds into a duration. Internally a duration is represented
    /// as a multiple of `1 / ODR` where ODR (the output data rate) is of type [`DataRate`].
    ///
    ///     assert_eq!(Duration::miliseconds(DataRate::Hz_25, 25.0), 667.5);
    #[inline(always)]
    pub fn miliseconds(output_data_rate: DataRate, miliseconds: f32) -> Self {
        Self::seconds(output_data_rate, miliseconds * 1000.0)
    }
}

#[derive(Copy,Clone,Debug,PartialEq, Eq)]
pub struct Timestamp(pub(crate) u32);
impl Timestamp {
    pub fn raw(self) -> u32 {
        self.0
    }
    pub fn us(self, odr: InternalFreqFinetuned) -> f32 {
       (self.raw() as f32) * 1f32 / (80000f32 + (0.0015 * odr.raw() as f32 * 80000f32))
    }
}


// TODO: Repurpose tis with FIFO Statuses.
// 

/// Data status structure. Decoded from the `STATUS_REG` register.
///
/// `STATUS_REG` has the following bit fields:
///   * `ZYXOR` - X, Y and Z-axis data overrun
///   * `ZOR` - Z-axis data overrun
///   * `YOR` - Y-axis data overrun
///   * `XOR` - X-axis data overrun
///   * `ZYXDA` - X, Y and Z-axis new data available
///   * `ZDA` - Z-axis new data available
///   * `YDA` Y-axis new data available
///   * `XDA` X-axis new data available
///
/// This struct splits the fields into more convenient groups:
///  * `zyxor` -> `ZYXOR`
///  * `xyzor` -> (`XOR`, `YOR`, `ZOR`)
///  * `zyxda` -> `ZYXDA`
///  * `xyzda` -> (`XDA`, `YDA`, `ZDA`)
#[derive(Debug)]
pub struct DataStatus {
    /// ZYXOR bit
    pub zyxor: bool,

    /// (XOR, YOR, ZOR) bits
    pub xyzor: (bool, bool, bool),

    /// ZYXDA bit
    pub zyxda: bool,

    /// (XDA, YDA, ZDA) bits
    pub xyzda: (bool, bool, bool),
}


/// Operating mode.
#[derive(Copy, Clone, Debug, Eq, PartialEq)]
#[repr(u8)]
pub enum Mode {
    /// High-resolution mode (12-bit data output)
    HighResolution,

    /// Normal mode (10-bit data output)
    Normal,

    /// Low-power mode (8-bit data output)
    LowPower,
}

// === WHO_AMI_I (0Fh) ===

/// `WHO_AM_I` device identification register
pub const DEVICE_ID: u8 = 0x6F;

// DEVICE CONFIG
pub const SOFT_RESET_CONFIG:u8 = 0b0000_0001;

// REG_BANK_SEL
pub const MASK_BANK_SEL:u8 = 0b0000_0111;

// PWR_MGMT0
pub const MASK_ACCEL_MODE:u8 = 0b0000_0011;
pub const MASK_GYRO_MODE:u8 = 0b0000_1100;
pub const IDLE:u8 = 0b0001_0000;
pub const TEMP_DIS:u8 = 0b0010_0000;


// GYRO_CONFIG0
pub const MASK_GYRO_ODR: u8 = 0b0000_1111;
pub const MASK_GYRO_UI_FS_SEL: u8 = 0b1110_0000;

// ACCEL_CONFIG0
pub const MASK_ACCEL_ODR: u8 = 0b0000_1111;
pub const MASK_ACCEL_UI_FS_SEL: u8 = 0b1110_0000;

// TMST_CONFIG
pub const TMST_TO_REGS_EN: u8 = 0b0001_0000;
pub const TMST_RES: u8 = 0b0000_1000;
pub const TMST_DELTA_EN: u8 = 0b0000_0100;
pub const TMST_FSYNC_EN: u8 = 0b0000_0010;
pub const TMST_EN: u8 = 0b0000_0001;


// BANK 3 
// PU_PD_CONFIG2
pub const PIN1_PU_EN:u8 = 0b1000_0000;

