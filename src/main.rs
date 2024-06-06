#![no_std]
#![no_main]

use fugit::RateExtU32;
use hal::pac;
use hal::uart::{DataBits, StopBits, UartConfig};
use rp2040_hal::Clock;
use rp_pico::entry;

use embedded_hal::digital::v2::OutputPin;

use panic_halt as _;
use rp2040_hal as hal;

use rp2040_lib::bme280::i2c::BME280;
use rp2040_lib::bme280::DEVICE_ADDRESS;

use rp2040_lib::my_macro::UART_TRANSMITTER;
use rp2040_lib::print;
use rp2040_lib::println;

use rp2040_hal::gpio::bank0::Gpio2;
use rp2040_hal::gpio::bank0::Gpio3;
use rp2040_hal::gpio::FunctionI2c;
use rp2040_hal::gpio::Pin;
use rp2040_hal::gpio::PullDown;
use rp2040_hal::i2c::I2C;
use rp2040_hal::pac::I2C1;

#[entry]
fn main() -> ! {
    let mut pac = pac::Peripherals::take().unwrap();
    let core = pac::CorePeripherals::take().unwrap();
    let mut watchdog = hal::Watchdog::new(pac.WATCHDOG);
    let clocks = hal::clocks::init_clocks_and_plls(
        rp_pico::XOSC_CRYSTAL_FREQ,
        pac.XOSC,
        pac.CLOCKS,
        pac.PLL_SYS,
        pac.PLL_USB,
        &mut pac.RESETS,
        &mut watchdog,
    )
    .ok()
    .unwrap();

    let mut delay = cortex_m::delay::Delay::new(core.SYST, clocks.system_clock.freq().to_Hz());

    let sio = hal::Sio::new(pac.SIO);

    let pins = rp_pico::Pins::new(
        // (8-6-7)
        pac.IO_BANK0,
        pac.PADS_BANK0,
        sio.gpio_bank0,
        &mut pac.RESETS,
    );

    let uart_pins = (pins.gpio0.reconfigure(), pins.gpio1.reconfigure());
    let uart = hal::uart::UartPeripheral::new(pac.UART0, uart_pins, &mut pac.RESETS)
        .enable(
            UartConfig::new(9600.Hz(), DataBits::Eight, None, StopBits::One),
            clocks.peripheral_clock.freq(),
        )
        .unwrap();

    let (_, uart_tx) = uart.split();

    critical_section::with(|_| unsafe {
        UART_TRANSMITTER = Some(uart_tx);
    });

    let mut led_pin = pins.led.into_push_pull_output();

    let mut i2c = hal::I2C::i2c1(
        pac.I2C1,
        pins.gpio2.into_function(), // sda
        pins.gpio3.into_function(), // scl
        400.kHz(),
        &mut pac.RESETS,
        125_000_000.Hz(),
    );

    let mut bme280 = BME280::<
        I2C<
            I2C1,
            (
                Pin<Gpio2, FunctionI2c, PullDown>,
                Pin<Gpio3, FunctionI2c, PullDown>,
            ),
        >,
    >::new(i2c, DEVICE_ADDRESS);

    // DeviceのIDコード(0x60)を正しく読めれば成功としている
    if bme280.init() {
        println!("BME280 initialization successful.");
        println!("BME280 ID = 0x60.\r\n");
    } else {
        println!("BME280 initialization failed.\r\n");
    }

    loop {
        bme280.read_data();
        let (mut temp, mut humi, mut pres) = bme280.get_elements();

        println!("T = {:.2} ℃", temp);
        println!("H = {:.2} %", humi);
        println!("P = {:.2} hPa\r\n", pres);

        led_pin.set_high().unwrap();
        delay.delay_ms(200);
        led_pin.set_low().unwrap();
        delay.delay_ms(800);
    }
}
