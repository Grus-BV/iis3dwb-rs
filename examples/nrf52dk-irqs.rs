//! This example is for the nRF 52 DK 
#![no_std]
#![no_main]

use defmt_rtt as _;
use cortex_m_rt::entry;
use defmt::*;
use defmt::panic;
use embedded_hal::digital::v2::OutputPin;
use embedded_hal::blocking::spi::*;
use nrf52840_hal:: {gpio,
                    spim,
                    gpio::p0,   
                    gpio::Level,
                    gpiote::*,
                    Spim,
                    pac::CLOCK,
                    };

use iis3dwb::{Config as IIS3DWBConfig, Range, IIS3DWB, 
                        Accelerometer, RawAccelerometer};
use nrf52840_hal::pac::interrupt;

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

#[interrupt]
fn GPIOTE (){
    debug!("in!");
}

#[entry]
fn main() -> ! {
    info!("running!");

    // let _clocks = nrf52840_hal::clocks::Clocks::new(ctx.device.CLOCK).enable_ext_hfosc();
    
    let p = nrf52840_hal::pac::Peripherals::take().unwrap();
    let port0 = p0::Parts::new(p.P0);
    let ncs = port0.p0_14.into_push_pull_output(Level::High);
    let mut heartbeat = port0.p0_00.into_push_pull_output(Level::High);
    let spiclk = port0.p0_13.into_push_pull_output(Level::Low).degrade();
    let spimiso = port0.p0_15.into_floating_input().degrade();
    let spimosi = port0.p0_16.into_push_pull_output(Level::Low).degrade();
    let int1 = port0.p0_12.into_pulldown_input().degrade();

    let gpiote = Gpiote::new( p.GPIOTE);

    gpiote
            .channel0()
            .input_pin(&int1)
            .lo_to_hi()
            .enable_interrupt();

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
    let mut irqs = acc_cfg.interrupt1.cfg;
    irqs.AccDataReady = true;

    let mut accelerometer = IIS3DWB::new(spi, ncs, &acc_cfg).unwrap();
    let id = accelerometer.get_device_id();
    defmt::info!("The device ID is: 0x{=u8:x}", id);
    // let temp = accelerometer.read_temp_raw();
    // defmt::info!("The device temperature is: 0x{=u16:x}", temp);
    accelerometer.start();
    accelerometer.enable_all_interrupts();
    loop {
        cortex_m::asm::delay(1000_0000);
        info!("Lo");
        heartbeat.set_low().unwrap();
        cortex_m::asm::delay(1000_0000);
        info!("Hi");
        heartbeat.set_high().unwrap();
    }
    exit();
}



