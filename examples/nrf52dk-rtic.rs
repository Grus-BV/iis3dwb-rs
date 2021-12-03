//! This example is for the nRF 52 DK 
#![no_std]
#![no_main]

use defmt_rtt as _;
use cortex_m_rt::entry;
use defmt::*;
use defmt::panic;
use embedded_hal::digital::v2::OutputPin;
use embedded_hal::digital::v2::InputPin;
use embedded_hal::blocking::spi::*;
use iis3dwb::F32x3;
use nrf52840_hal:: {Spim, gpio, gpio::Level, gpio::p0, gpio::{Input,PullDown,PushPull,Output,Pin}, gpiote::*, pac::{CLOCK, SPIM3}, spim};

use iis3dwb::{Config as IIS3DWBConfig, Range, IIS3DWB, 
                        Accelerometer, RawAccelerometer,Error};
use nrf52840_hal::pac::interrupt;
use core::fmt::{self, Debug, Display};

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
#[derive(Debug)]
pub enum DummyError {}

#[rtic::app(device = nrf52840_hal::pac, peripherals = true, monotonic = rtic::cyccnt::CYCCNT)]
const APP: () = {
    struct Resources {
        gpiote: Gpiote,
        int1: Pin<Input<PullDown>>,
        accelerometer: IIS3DWB<Spim<SPIM3>, Pin<Output<PushPull>>>,
        counter: u16
    }

    #[init]
    fn init(mut ctx: init::Context) -> init::LateResources {
        info!("running!");
        let port0 = p0::Parts::new(ctx.device.P0);
        let ncs = port0.p0_14.into_push_pull_output(Level::High).degrade();
        let mut heartbeat = port0.p0_24.into_push_pull_output(Level::High);
        let spiclk = port0.p0_13.into_push_pull_output(Level::Low).degrade();
        let spimiso = port0.p0_15.into_floating_input().degrade();
        let spimosi = port0.p0_16.into_push_pull_output(Level::Low).degrade();
        let int1 = port0.p0_12.into_pulldown_input().degrade();

        let gpiote = Gpiote::new(ctx.device.GPIOTE);

        gpiote.channel0()
                .input_pin(&int1)
                .lo_to_hi()
                .enable_interrupt();

        let spi_pins = nrf52840_hal::spim::Pins {
            sck: spiclk,
            miso: Some(spimiso),
            mosi: Some(spimosi),
        };
        
        let mut spi =   Spim::new(
            ctx.device.SPIM3,
            spi_pins,
            nrf52840_hal::spim::Frequency::M8,
            nrf52840_hal::spim::MODE_3, 
            0
        );

        let mut acc_cfg = IIS3DWBConfig::default();
        acc_cfg.interrupt1.cfg.AccDataReady = true;
        defmt::info!("IRQ CFG: 0x{=u8:x}", acc_cfg.interrupt1.cfg.raw());

        let mut accelerometer = IIS3DWB::new(spi, ncs, &acc_cfg).unwrap();
        let id = accelerometer.get_device_id();
        defmt::info!("The device ID is: 0x{=u8:x}", id);
        // let temp = accelerometer.read_temp_raw();
        // defmt::info!("The device temperature is: 0x{=u16:x}", temp);
        accelerometer.start();
        accelerometer.enable_drdy();
        accelerometer.enable_all_interrupts();
        accelerometer.accel_raw();

        let mut acc  = accelerometer.accel_norm().unwrap();
        let mut counter = 0u16;
        init::LateResources {
            gpiote,
            int1,
            accelerometer,
            counter,
        }
    }

    #[idle]
    fn idle(_: idle::Context) -> ! {
        loop {
            cortex_m::asm::wfi();
        }
    }

    #[task(binds = GPIOTE, resources = [gpiote, int1, accelerometer,counter])]
    fn irq_service(ctx: irq_service::Context) {
        ctx.resources.gpiote.reset_events();
        let accelerometer_irqd = ctx.resources.int1.is_low().unwrap();
        if accelerometer_irqd {
            defmt::info!("New data!");
        }
        let accel = ctx.resources.accelerometer.accel_norm().unwrap();
        *ctx.resources.counter += 1;
        if *ctx.resources.counter > 100{
            defmt::info!("IRQ!, {},{},{}",accel.x, accel.y, accel.z );
            *ctx.resources.counter = 0u16;
        }
    }
    extern "C" {
        fn SWI0_EGU0();
    }
};



