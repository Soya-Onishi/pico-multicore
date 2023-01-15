#![no_std]
#![no_main]

use bsp::{entry, hal::gpio::bank0::Gpio25};
use embedded_hal::digital::v2::OutputPin;
use panic_halt as _;

use rp_pico as bsp;

use bsp::hal::{
    clocks::{init_clocks_and_plls, Clock},
    gpio::pin::{Output, PushPull},
    gpio::Pin,
    multicore, pac,
    sio::Sio,
    usb,
    watchdog::Watchdog,
};

use usb_device::{class_prelude::*, prelude::*};
use usbd_serial::SerialPort;

use core::fmt::Write;
use heapless::String;

static mut CORE1_STACK: multicore::Stack<4096> = multicore::Stack::new();

fn core0_task(mut led: Pin<Gpio25, Output<PushPull>>) -> ! {
    loop {
        led.set_high().unwrap();
        for _ in 0..u16::MAX {}
        led.set_low().unwrap();
        for _ in 0..u16::MAX {}
    }
}

fn core1_task<B: UsbBus>(
    mut serial: SerialPort<B>,
    mut usb_dev: UsbDevice<B>,
    mut delay: cortex_m::delay::Delay,
) -> ! {
    let mut count = 0u32;
    loop {
        while !usb_dev.poll(&mut [&mut serial]) {
            delay.delay_ms(5);
        }
        // let mut str: String<128> = String::new();
        // write!(&mut str, "Hello {}\r\n", count).unwrap();
        serial.write(b"Hello World\r\n").unwrap();

        let (ret, _) = count.overflowing_add(1);
        count = ret;
    }
}

#[entry]
fn main() -> ! {
    let mut pac = pac::Peripherals::take().unwrap();
    let core = pac::CorePeripherals::take().unwrap();
    let mut watchdog = Watchdog::new(pac.WATCHDOG);
    let mut sio = Sio::new(pac.SIO);

    // External high-speed crystal on the pico board is 12Mhz
    let external_xtal_freq_hz = 12_000_000u32;
    let clocks = init_clocks_and_plls(
        external_xtal_freq_hz,
        pac.XOSC,
        pac.CLOCKS,
        pac.PLL_SYS,
        pac.PLL_USB,
        &mut pac.RESETS,
        &mut watchdog,
    )
    .ok()
    .unwrap();

    let usb_bus = UsbBusAllocator::new(usb::UsbBus::new(
        pac.USBCTRL_REGS,
        pac.USBCTRL_DPRAM,
        clocks.usb_clock,
        true,
        &mut pac.RESETS,
    ));

    let mut serial = SerialPort::new(&usb_bus);

    let mut usb_dev = UsbDeviceBuilder::new(&usb_bus, UsbVidPid(0x16C0, 0x27DD))
        .manufacturer("Fake company")
        .product("serial port")
        .serial_number("TEST")
        .device_class(2)
        .build();

    let pins = bsp::Pins::new(
        pac.IO_BANK0,
        pac.PADS_BANK0,
        sio.gpio_bank0,
        &mut pac.RESETS,
    );

    let mut mc = multicore::Multicore::new(&mut pac.PSM, &mut pac.PPB, &mut sio.fifo);
    let cores = mc.cores();
    let core1 = &mut cores[1];
    let _ = core1.spawn(unsafe { &mut CORE1_STACK.mem }, move || {
        let led_pin = pins.led.into_push_pull_output();

        core0_task(led_pin);
    });

    let mut delay = cortex_m::delay::Delay::new(core.SYST, clocks.system_clock.freq().to_Hz());
    let mut count = 0u32;
    loop {
        for _ in 0..100 {
            let _ = usb_dev.poll(&mut [&mut serial]);
            delay.delay_ms(5);
        }

        // let mut str: String<128> = String::new();
        // write!(&mut str, "Hello {}\r\n", count).unwrap();
        while !usb_dev.poll(&mut [&mut serial]) {}
        serial.write(b"Hello World\r\n").unwrap();

        let (ret, _) = count.overflowing_add(1);
        count = ret;
    }
    // core1_task(serial, usb_dev, delay);
}

// End of file
