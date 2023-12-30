# pm1006 Rust driver

A platform-agnostic Rust driver for the `pm1006` particulate matter sensor. Based on `embedded-io` traits.
This sensor is located in `IKEA VINDRIKTNING` air quality sensor.

## Usage

Example usage with ESP32:

```rust
use pm1006::Pm1006;

use esp_idf_svc::hal::gpio::Gpio0;
use esp_idf_svc::hal::gpio::Gpio1;
use esp_idf_svc::hal::uart::UartConfig;
use esp_idf_svc::hal::uart::UartDriver;
use esp_idf_svc::hal::units::Hertz;
use log::*;

let config = UartConfig::new().baudrate(Hertz(9_600));
let uart_driver = UartDriver::new(
    uart1,
    pins.gpio17,
    pins.gpio16,
    Option::<Gpio0>::None,
    Option::<Gpio1>::None,
    &config,
)
.unwrap();

let pm1006 = Pm1006::new(uart_driver);
let pm25 = pm1006.read_pm25().unwrap();
info!("PM2.5: {}", pm25);

```
