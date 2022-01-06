//! This example is for the nRF 52 DK 

#![deny(unsafe_code)]
#![no_std]
#![no_main]

use defmt_rtt as _;
use cortex_m_rt::entry;
use defmt::*;
use defmt::panic;
use embedded_hal::blocking::spi::*;
use embedded_hal::digital::v2::InputPin;
use embedded_hal::digital::v2::OutputPin;
use nrf52840_hal:: {gpio,
                    spim,
                    gpio::p0,
                    gpio::p1,   
                    gpio::Level,
                    Spim,
                    };

use iis3dwb::{Config as IIS3DWBConfig, Range, IIS3DWB, 
                        Accelerometer, RawAccelerometer};


use panic_probe as _;
#[defmt::panic_handler]
fn panic() -> ! {
    cortex_m::asm::udf()
}

pub fn exit() -> ! {
    loop {
        cortex_m::asm::bkpt();
    }
}


#[entry]
fn main() -> ! {
    info!("running!");

    let p = nrf52840_hal::pac::Peripherals::take().unwrap();
    let port0 = p0::Parts::new(p.P0);
    let port1 = p1::Parts::new(p.P1);
    let mut led1 = port1.p1_02.into_push_pull_output(Level::Low);
    let mut led2 = port1.p1_04.into_push_pull_output(Level::Low);
    let mut led3 = port1.p1_06.into_push_pull_output(Level::Low);
    let ncs = port0.p0_14.into_push_pull_output(Level::High);
    let spiclk = port0.p0_13.into_push_pull_output(Level::Low).degrade();
  
    #[cfg (not(feature = "board_alpha"))]
    let spimiso = port0.p0_15.into_pullup_input().degrade();
    #[cfg (not(feature = "board_alpha"))]
    let spimosi = port0.p0_16.into_push_pull_output(Level::Low).degrade();
    #[cfg (feature = "board_alpha")]
    let spimiso = port0.p0_16.into_floating_input().degrade();
    #[cfg (feature = "board_alpha")]
    let spimosi = port0.p0_15.into_push_pull_output(Level::Low).degrade();

    let spi_pins = nrf52840_hal::spim::Pins {
        sck: spiclk,
        miso: Some(spimiso),
        mosi: Some(spimosi),
    };

    let mut spi =   Spim::new(
        p.SPIM3,
        spi_pins,
        nrf52840_hal::spim::Frequency::M1,
        nrf52840_hal::spim::MODE_3, 
        0
    );

    let mut acc_cfg = IIS3DWBConfig::default();

    let mut accelerometer = IIS3DWB::new(spi, ncs, &acc_cfg).unwrap();
    let id = accelerometer.get_device_id();
    defmt::info!("The device ID is: 0x{=u8:x}", id);
    // let temp = accelerometer.read_temp_raw();
    // defmt::info!("The device temperature is: 0x{=u16:x}", temp);

    accelerometer.start();
    accelerometer.set_timestamp_en(true);
    loop{
        cortex_m::asm::delay(25_000_000);   // KISS.
        cortex_m::asm::delay(25_000_000);   // KISS.
        let mut acc  = accelerometer.accel_norm().unwrap();
        let mut odr  = accelerometer.sample_rate().unwrap();
        let mut tstamp   = accelerometer.get_timestamp();
        defmt::info!("{} gs,{} gs,{} gs,{} Hz,{} us",acc.x,acc.y,acc.z,odr,tstamp.us(accelerometer.get_odr()));
    }

    exit();
}

