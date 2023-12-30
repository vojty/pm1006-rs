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
info!("PM2.5: {}ug/m3", pm25);

```

## Additional info

### Air Quality Index (AQI) breakpoints for PM2.5

| AQI Category                   | Index Values | PM2.5 (ug/m3, 24-hour average) |
| ------------------------------ | ------------ | ------------------------------ |
| Good                           | 0 - 50       | 0.0 - 12.0                     |
| Moderate                       | 51 - 100     | 12.1 - 35.4                    |
| Unhealthy for Sensitive Groups | 101 – 150    | 35.5 – 55.4                    |
| Unhealthy                      | 151 – 200    | 55.5 – 150.4                   |
| Very Unhealthy                 | 201 – 300    | 150.5 – 250.4                  |
| Hazardous                      | 301 – 400    | 250.5 – 350.4                  |
| Hazardous                      | 401 – 500    | 350.5 – 500                    |

Source https://aqicn.org/faq/2013-09-09/revised-pm25-aqi-breakpoints/

## Development

### Release new version

1. Bump version in `Cargo.toml`
2. Commit changes
3. Tag commit with the version using `git tag -a v0.1.0 -m "v0.1.0"`
4. Release with `cargo release`
5. Push changes to GitHub
