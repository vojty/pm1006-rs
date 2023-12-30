use embedded_io::{Read, Write};

// Based on https://github.com/bertrik/pm1006/

// More info on the protocol:
// https://revspace.nl/VINDRIKTNING
// https://threadreaderapp.com/thread/1415291684569632768.html

pub struct Pm1006<Uart> {
    uart: Uart,
    buffer: [u8; 20],
}

const REQUEST_HEADER: u8 = 0x11;
const RESPONSE_HEADER: u8 = 0x16;
const COMMAND_SEQUENCE: [u8; 5] = [REQUEST_HEADER, 0x02, 0x0b, 0x01, 0xe1];

/// Response structure:
/// ```raw
///   1 byte:   0x16
///   1 byte:   length N of response data
///   N bytes:  response data (CMD + Data frames)
///   1 byte:   check sum
///
/// PM2.5 = DF3 * 256 + DF4 (indexed from 1)
/// ````
fn parse_response<E>(response: &[u8]) -> Result<u16, errors::Error<E>> {
    // Check header
    if response[0] != RESPONSE_HEADER {
        return Err(errors::Error::InvalidHeader(response[0]));
    }

    // header + length
    let data_offset = 2;
    let mut checksum: u8 = response.iter().take(data_offset).sum();
    let length = response[1];

    // PM2.5 = DF3 * 256 + DF4 (indexed from 1)
    let mut result: u16 = 0;

    // Loop through data
    for i in 0..length {
        let index = data_offset + i as usize;
        let byte = response[index];
        match i {
            // CMD
            0 => {
                if byte != 0x0b {
                    return Err(errors::Error::InvalidCommandResponse(byte));
                }
            }
            // DF1
            1 => (),

            // DF2
            2 => (),

            // DF3
            3 => result = byte as u16 * 256,

            // DF4
            4 => result += byte as u16,

            // DF5 - DF16 (not used)
            _ => (),
        }
        // checksum += byte;
        checksum = checksum.wrapping_add(byte);
    }

    let expected_checksum = response[data_offset + length as usize];
    let diff = expected_checksum.wrapping_add(checksum);
    if diff != 0 {
        return Err(errors::Error::InvalidChecksum(ChecksumMismatch {
            expected: expected_checksum,
            calculated: checksum,
        }));
    }

    Ok(result)
}

impl<Uart, E> Pm1006<Uart>
where
    Uart: Read<Error = E> + Write,
{
    pub fn new(uart: Uart) -> Self {
        Self {
            uart,
            buffer: [0; 20],
        }
    }

    pub fn read_pm25(&mut self) -> Result<u16, errors::Error<E>> {
        self.send_command()?;
        self.read_response()?;

        parse_response::<E>(&self.buffer)
    }

    fn read_response(&mut self) -> Result<usize, errors::Error<E>> {
        self.uart
            .read(&mut self.buffer)
            .map_err(errors::Error::SerialReadFail)
    }

    fn send_command(&mut self) -> Result<usize, errors::Error<E>> {
        self.uart
            .write(&COMMAND_SEQUENCE)
            .map_err(errors::Error::SerialWriteFail)
    }
}

#[derive(Debug)]
pub struct ChecksumMismatch {
    pub expected: u8,
    pub calculated: u8,
}

pub mod errors {
    #[derive(Debug)]
    pub enum Error<E> {
        InvalidHeader(u8),
        InvalidCommandResponse(u8),
        InvalidChecksum(super::ChecksumMismatch),
        SerialReadFail(E),
        SerialWriteFail(E),
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_parse_response() {
        // an example response taken from the actual device
        let response: [u8; 20] = [
            22,  // 0x16 Response header (fixed)
            17,  // 0x11 Response length (CMD echo + DFs)
            11,  // 0x0b CMD echo
            0,   // 0x00 DF1
            0,   // 0x00 DF2
            0,   // 0x00 DF3 <----
            9,   // 0x09 DF4 <----
            0,   // 0x00 DF5
            0,   // 0x00 DF6
            3,   // 0x03 DF7
            238, // 0xee DF8
            0,   // 0x00 DF9
            0,   // 0x00 DF10
            0,   // 0x00 DF11
            64,  // 0x40 DF12
            2,   // 0x02 DF13
            0,   // 0x00 DF14
            0,   // 0x00 DF15
            55,  // 0x37 DF16
            91,  // 0x5b checksum
        ];
        // result = DF3 * 256 + DF4
        // 0 * 256 + 9 = 9
        let result = parse_response::<()>(&response);
        assert!(result.is_ok());
        assert!(result.unwrap() == 9);
    }

    #[test]
    fn test_parse_response_with_2_data_frames() {
        let response: [u8; 20] = [
            22,  // 0x16 Response header (fixed)
            17,  // 0x11 Response length (CMD echo + DFs)
            11,  // 0x0b CMD echo
            0,   // 0x00 DF1
            0,   // 0x00 DF2
            1,   // 0x01 DF3 <----
            9,   // 0x09 DF4 <----
            0,   // 0x00 DF5
            0,   // 0x00 DF6
            3,   // 0x03 DF7
            238, // 0xee DF8
            0,   // 0x00 DF9
            0,   // 0x00 DF10
            0,   // 0x00 DF11
            64,  // 0x40 DF12
            2,   // 0x02 DF13
            0,   // 0x00 DF14
            0,   // 0x00 DF15
            55,  // 0x37 DF16
            90,  // 0x5a checksum
        ];
        // result = DF3 * 256 + DF4
        // 1 * 256 + 9 = 265
        let result = parse_response::<()>(&response);
        assert!(result.is_ok());
        assert!(result.unwrap() == 265);
    }
}
