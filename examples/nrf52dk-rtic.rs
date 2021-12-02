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
                    gpio::{Input,PullDown,Pin},
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


#[rtic::app(device = nrf52840_hal::pac, peripherals = true, monotonic = rtic::cyccnt::CYCCNT)]
const APP: () = {
    struct Resources {
        gpiote: Gpiote,
        int1: Pin<Input<PullDown>>,
        accelerometer: Accelerometer,
    }

    #[init]
    fn init(mut ctx: init::Context) -> init::LateResources {
        info!("running!");
        let _clocks = hal::clocks::Clocks::new(ctx.device.CLOCK).enable_ext_hfosc();
        let port0 = p0::Parts::new(ctx.device.P0);
        let ncs = port0.p0_14.into_push_pull_output(Level::High);
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
            p.SPIM3,
            spi_pins,
            nrf52840_hal::spim::Frequency::K125,
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

        let mut acc  = accelerometer.accel_norm().unwrap();
                
        ctx.core.DCB.enable_trace();
        ctx.core.DWT.enable_cycle_counter();
        
        init::LateResources {
            gpiote,
            int1,
            accelerometer,
        }
    }

    #[idle]
    fn idle(_: idle::Context) -> ! {
        loop {
            cortex_m::asm::wfi();
        }
    }

    #[task(resources = [gpiote, int1])]
    fn irq_service(ctx: irq_service::Context) {
        let accelerometer_irqd = ctx.resources.int1.is_low().unwrap();
        if accelerometer_irqd {
            rprintln!("New data!");
        }
    }
};



