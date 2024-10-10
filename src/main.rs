// #![deny(warnings)]
#![no_main]
#![no_std]

use core::cell::Cell;
use core::cell::RefCell;
use core::fmt::Write;
use cortex_m::interrupt::Mutex;
use cortex_m_rt::entry;
use heapless::{Deque, String};
use lis3dh::accelerometer::Accelerometer;
use lis3dh::Lis3dh;
use panic_semihosting as _;
use stm32h7xx_hal::rcc::rec::UsbClkSel;
use stm32h7xx_hal::usb_hs::{UsbBus, USB1};
use stm32h7xx_hal::{
    gpio::gpiod::PD2,
    gpio::{Edge, ExtiPin, Input, PinState},
    interrupt,
    pac,
    prelude::*,
    // stm32,
    stm32::NVIC,
};
use usb_device::device::UsbDeviceBuilder;
use usb_device::device::UsbVidPid;
use usbd_serial::SerialPort;

// use log::info;

static BUTTON_PIN: Mutex<RefCell<Option<PD2<Input>>>> = Mutex::new(RefCell::new(None));
static BUTTON_PRESS: Mutex<Cell<bool>> = Mutex::new(Cell::new(false));

static MSG_BUF: Mutex<RefCell<Option<Deque<String<64>, 8>>>> = Mutex::new(RefCell::new(None));

static mut USB_BUF: [u32; 1024] = [0; 1024];

#[entry]
fn main() -> ! {
    let mut cp = cortex_m::Peripherals::take().unwrap();
    let dp = pac::Peripherals::take().unwrap();

    // Constrain and Freeze power
    let pwr = dp.PWR.constrain();
    let pwrcfg = pwr.freeze();

    // Constrain and Freeze clock
    let rcc = dp.RCC.constrain();
    let mut ccdr = rcc.sys_ck(80.MHz()).freeze(pwrcfg, &dp.SYSCFG);

    // Need 48MHz CLOCK for USB
    let _ = ccdr.clocks.hsi48_ck().expect("HSI48 must run");
    ccdr.peripheral.kernel_usb_clk_mux(UsbClkSel::Hsi48);

    // Needed for interrupt handler
    let mut syscfg = dp.SYSCFG;
    let mut exti = dp.EXTI;

    // GPIO Buses
    let gpiob = dp.GPIOB.split(ccdr.peripheral.GPIOB);
    let gpiod = dp.GPIOD.split(ccdr.peripheral.GPIOD);
    let gpioe = dp.GPIOC.split(ccdr.peripheral.GPIOC);

    // Delay provider.
    let _delay = cp.SYST.delay(ccdr.clocks);

    //// USB

    // Ensure USP_SWAP is low to enable USB (vs DFU)
    gpiod.pd8.into_push_pull_output_in_state(PinState::Low);

    // USB CDC
    let pin_dm = gpiob.pb14.into_alternate();
    let pin_dp = gpiob.pb15.into_alternate();

    let usb = USB1::new(
        dp.OTG1_HS_GLOBAL,
        dp.OTG1_HS_DEVICE,
        dp.OTG1_HS_PWRCLK,
        pin_dm,
        pin_dp,
        ccdr.peripheral.USB1OTG,
        &ccdr.clocks,
    );

    let usb_bus = unsafe {
        let usb_buf: &'static mut [u32] = &mut USB_BUF[..];
        UsbBus::new(usb, usb_buf)
    };

    let mut serial = SerialPort::new(&usb_bus);

    let mut usb_dev = UsbDeviceBuilder::new(&usb_bus, UsbVidPid(0x16c0, 0x27dd))
        .strings(&[usb_device::device::StringDescriptors::default()
            .manufacturer("32blit")
            .product("Serial Port")
            .serial_number("32blit")])
        .unwrap()
        .device_class(usbd_serial::USB_CLASS_CDC)
        .build();

    // Initialise MSG_BUF
    cortex_m::interrupt::free(|cs| {
        MSG_BUF
            .borrow(cs)
            .replace(Some(Deque::<String<64>, 8>::new()));
    });

    //// LEDS

    let led_red = gpioe.pc8.into_push_pull_output().erase();
    let led_green = gpioe.pc9.into_push_pull_output().erase();
    let led_blue = gpiob.pb5.into_push_pull_output().erase();

    // Turn LEDs off
    let mut leds = [led_red, led_green, led_blue];
    for led in &mut leds {
        led.set_high();
    }

    //// BUTTON

    // Button A (Interrupt)
    let mut sw_a = gpiod.pd2.into_pull_up_input();
    sw_a.make_interrupt_source(&mut syscfg);
    sw_a.trigger_on_edge(&mut exti, Edge::Falling);
    sw_a.enable_interrupt(&mut exti);

    cortex_m::interrupt::free(|cs| {
        BUTTON_PIN.borrow(cs).replace(Some(sw_a));
    });

    // Enable the button interrupt
    unsafe {
        cp.NVIC.set_priority(interrupt::EXTI2, 1);
        NVIC::unmask::<interrupt>(interrupt::EXTI2);
    }

    //// I2C

    // Configure the SCL and the SDA pin for our I2C bus
    let scl = gpiod.pd12.into_alternate_open_drain();
    let sda = gpiod.pd13.into_alternate_open_drain();

    let i2c = dp
        .I2C4
        .i2c((scl, sda), 100.kHz(), ccdr.peripheral.I2C4, &ccdr.clocks);

    let mut lis3dh = Lis3dh::new_i2c(i2c, lis3dh::SlaveAddr::Default).unwrap();

    //// TIMERS
    let mut usb_timer = dp.TIM1.timer(1.Hz(), ccdr.peripheral.TIM1, &ccdr.clocks);
    let mut led_timer = dp.TIM2.timer(1.Hz(), ccdr.peripheral.TIM2, &ccdr.clocks);

    // Start timers
    usb_timer.start(200.Hz());
    led_timer.start(5.Hz());

    let mut counter = 0_u32;
    let mut led_counter = 0_u32;
    let mut usb_counter = 0_u32;

    loop {
        // need to call usb_dev.poll at least every 10ms
        if let Ok(_) = usb_timer.wait() {
            if usb_dev.poll(&mut [&mut serial]) {
                cortex_m::interrupt::free(|cs| {
                    if let Some(buf) = MSG_BUF.borrow(cs).borrow_mut().as_mut() {
                        /*
                            let mut read_buf = [0_u8; 64];
                            match serial.read(&mut read_buf[..]) {
                                Ok(count) => {
                                    let mut msg: String<64> = String::new();
                                    let _ = write!(msg, "[USB READ] :: ");
                                    for i in 0..count {
                                        match msg.write_char(read_buf[i] as char) {
                                            Ok(_) => {}
                                            Err(_) => break,
                                        }
                                    }
                                    let _ = buf.push_back(msg);
                                }
                                Err(_) => {}
                            };
                        */

                        if usb_counter % 200 == 0 {
                            let mut msg: String<64> = String::new();
                            let _ = write!(msg, "[USB] :: {} {}\r\n", usb_counter, counter);
                            let _ = buf.push_back(msg);
                        }

                        if let Some(msg) = buf.pop_front() {
                            let _ = serial.write(msg.as_bytes());
                            let _ = serial.flush();
                        }
                    }
                });
            }
            // Restart timer
            usb_timer.start(200.Hz());
            usb_counter += 1;
        }
        if let Ok(_) = led_timer.wait() {
            cortex_m::interrupt::free(|cs| {
                if let Some(buf) = MSG_BUF.borrow(cs).borrow_mut().as_mut() {
                    if BUTTON_PRESS.borrow(cs).get() {
                        BUTTON_PRESS.borrow(cs).set(false);
                        let mut msg: String<64> = String::new();
                        let _ = write!(msg, "[BUTTON] :: {}\r\n", counter);
                        let _ = buf.push_back(msg);
                    }
                    let mut msg: String<64> = String::new();
                    let _ = write!(msg, "[LED] :: {} {}\r\n", led_counter, counter);
                    let _ = buf.push_back(msg);

                    if let Ok(true) = lis3dh.is_data_ready() {
                        let mut msg: String<64> = String::new();
                        match lis3dh.accel_norm() {
                            Ok(accel) => {
                                let _ = write!(
                                    msg,
                                    "[LIS3 READ] :: x={:.2} y={:.2} z={:.2}\r\n",
                                    accel.x, accel.y, accel.z
                                );
                                let _ = buf.push_back(msg);
                            }
                            Err(_) => {
                                let _ = write!(msg, "[LIS3 READ] :: ERROR\r\n");
                                let _ = buf.push_back(msg);
                            }
                        };
                    }
                    /*
                    // Poll LIS3 I2C device - Whoami Register 0x0f)
                    let i2c_reg = [0x0f_u8];
                    let mut i2c_buf = [0_u8];
                    if let Ok(_) = i2c.write_read(0x18, &i2c_reg, &mut i2c_buf) {
                        let mut msg: String<64> = String::new();
                        let _ = write!(msg, "[I2C READ] :: {}\r\n", i2c_buf[0]);
                        let _ = buf.push_back(msg);
                    } else {
                        let mut msg: String<64> = String::new();
                        let _ = write!(msg, "[I2C READ] :: FAILED\r\n");
                        let _ = buf.push_back(msg);
                    }
                    */
                }
            });

            leds[0].toggle();
            led_timer.start(5.Hz());
            led_counter += 1;
        }
        /*for led in &mut leds {
            let mut next = false;
            while !next {
                if usb_dev.poll(&mut [&mut serial]) {
                    let _ = serial.write("Hello!\n".as_bytes());
                }
                led.set_low();
                delay.delay_ms(250_u16);
                led.set_high();
                delay.delay_ms(250_u16);
                cortex_m::interrupt::free(|cs| {
                    if BUTTON_PRESS.borrow(cs).get() {
                        BUTTON_PRESS.borrow(cs).set(false);
                        next = true;
                    }
                });
            }
        }
        */
        counter += 1;
    }
}

#[interrupt]
fn EXTI2() {
    cortex_m::interrupt::free(|cs| {
        if let Some(b) = BUTTON_PIN.borrow(cs).borrow_mut().as_mut() {
            b.clear_interrupt_pending_bit()
        }
        BUTTON_PRESS.borrow(cs).set(true);
    });
}
