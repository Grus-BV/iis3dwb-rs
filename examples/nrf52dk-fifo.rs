#![no_std]
#![no_main]
#![deny(unsafe_code)]
#![feature(type_alias_impl_trait)]

use defmt_rtt as _;
use cortex_m_rt::entry;
use defmt::*;
use defmt::panic;
use embedded_hal::blocking::spi::*;
use embassy::executor::Spawner;
use embassy::time::{Duration, Timer, Instant};
use embassy_nrf::Peripherals;
use embassy_nrf::gpio::{Level, Input, Output, OutputDrive, Pull};
use embassy_nrf::{interrupt, qspi, spim};

use iim42652::{Config as IIM42652Config, Range, IIM42652, 
                        Accelerometer, RawAccelerometer,
                        AccelerometerMode,
                        FifoMode,FifoConfig,
                        Watermark,
                        TimestampedAcceleration};

const PAGE_SIZE: usize = 4096;

// Workaround for alignment requirements.
// Nicer API will probably come in the future.
#[repr(C, align(4))]
struct AlignedBuf([u8; 4096]);
#[repr(C, align(4))]
struct AlignedBuf2000([u8; 2000]);


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


#[embassy::main]
async fn main(spawner: Spawner, mut p: Peripherals){

    // defmt::info!("FLASH ACTIVE");
    // let mut config = qspi::Config::default();
    // config.read_opcode = qspi::ReadOpcode::READ4IO;
    // config.write_opcode = qspi::WriteOpcode::PP4IO;
    // config.write_page_size = qspi::WritePageSize::_256BYTES;    

    
    // let mut irq = interrupt::take!(QSPI);

    // let mut q = qspi::Qspi::new(
    //     &mut p.QSPI, 
    //     &mut irq, 
    // &mut p.P0_19, 
    // &mut p.P0_17,
    // &mut p.P0_20,
    // &mut p.P0_21,
    // &mut p.P0_22,
    // &mut p.P0_23,
    //     config,
    // ).await;
    
    
    // defmt::info!("FLASH ACTIVE");

    // // Read Config
    // let mut rdcr =  [0; 1];
    // unwrap!(q.custom_instruction(0x15, &[], &mut rdcr).await);
    // info!("rdcr: {=[u8]:x}", rdcr);
    
    // // Read ID
    // let mut id = [1; 3];
    // unwrap!(q.custom_instruction(0x9F, &[], &mut id).await);
    // info!("id: {=[u8]:x}", id);

    // // Read status register
    // let mut status = [4; 1];
    // unwrap!(q.custom_instruction(0x05, &[], &mut status).await);
    // info!("status: {:x}", status[0]);

    // unwrap!(q.custom_instruction(0x06, &[], &mut []).await);
    // info!("WREN!");

    // let mut status_and_rdcr = [status[0],rdcr[0]];
    // if status_and_rdcr[0] & 0x40 == 0 {
    //     status_and_rdcr[0] |= 0x40;
    // }

    // unwrap!(q.custom_instruction(0x01, &status_and_rdcr, &mut []).await);
    // info!("enabled quad in status");

    // info!("page erasing... ", );
    // let before = Instant::now().as_ticks();
    // unwrap!(q.erase(0*PAGE_SIZE).await);
    // unwrap!(q.erase(1*PAGE_SIZE).await);
    // unwrap!(q.erase(2*PAGE_SIZE).await);
    // unwrap!(q.erase(3*PAGE_SIZE).await);
    // unwrap!(q.erase(4*PAGE_SIZE).await);
    // unwrap!(q.erase(5*PAGE_SIZE).await);
    // unwrap!(q.erase(6*PAGE_SIZE).await);
    // unwrap!(q.erase(7*PAGE_SIZE).await);
    // unwrap!(q.erase(8*PAGE_SIZE).await);
    // let after = Instant::now().as_ticks();
    // info!("took {} ticks", after-before); 

        let mut config = spim::Config::default();
        config.frequency = spim::Frequency::M32;
    

        let mut irq = interrupt::take!(SPIM3);
        let mut spim = spim::Spim::new( &mut p.SPI3, 
                                                        &mut irq, 
                                                        &mut p.P0_13,
                                                        &mut p.P0_16,
                                                        &mut p.P0_15,
                                                        config);

        let mut ncs = Output::new(&mut p.P0_14, Level::High, OutputDrive::Standard);
        
        defmt::info!("ACC CONFIG");        

    let mut acc_cfg = IIM42652Config::default();
    let mut accelerometer = IIM42652::new(spim, ncs, &acc_cfg).unwrap();
    let id = accelerometer.get_device_id();
    defmt::info!("The device ID is: 0x{=u8:x}", id);
    
    let mut fifo_cfg = iim42652::FifoConfig::default();
    fifo_cfg.mode = FifoMode::StreamToFifo;
    fifo_cfg.config_reg
            .with_accel_en(true)
            .with_temp_en(true)
            .with_timestamp_fsync_en(true)
            .with_resume_partial_read_en(true);
    accelerometer.configure_fifo(fifo_cfg);
    cortex_m::asm::delay(5000000);   // KISS.

    accelerometer.set_acc_mode(AccelerometerMode::LowNoise);
    
    let mut buffer =  AlignedBuf2000([0u8; 2000]);
    let mut before_meas = Instant::now().as_ticks();
    loop {
        let mut fifo_level = accelerometer.unread_data_count();
        if u16::from(fifo_level) > 2000 {
            info!("fifo full, level: {}",u16::from(fifo_level));  
            let before = Instant::now().as_ticks();
            buffer.0 = accelerometer.fifo_read();
            let after = Instant::now().as_ticks();
            info!("measuring took {} ticks", after-before);  
            let before = Instant::now().as_ticks();
            unwrap!(q.write(0, &buffer.0).await);
            let after = Instant::now().as_ticks();
            info!("writing took {} ticks", after-before); 
        
            let after_meas = Instant::now().as_ticks();
            info!("duration of all measurement: {} ticks", after_meas-before_meas);
            before_meas = Instant::now().as_ticks();
            info!("fifo after meas, level: {}",u16::from(accelerometer.unread_data_count()));  
        }
        else{
            cortex_m::asm::delay(5000);   // KISS.
        }
    }
    
    accelerometer.stop();
    accelerometer.reset();
    
    while accelerometer.resetting() {
        cortex_m::asm::wfe();
    }
    defmt::info!("Resetted!");

   
    exit();
}