use embedded_io::{Read, Write};

// Based on https://github.com/bertrik/pm1006/

// More info on the protocol:
// https://revspace.nl/VINDRIKTNING
// https://threadreaderapp.com/thread/1415291684569632768.html

const REQUEST_HEADER: u8 = 0x11;
const RESPONSE_HEADER: u8 = 0x16;
const COMMAND: u8 = 0x0b;
const COMMAND_SEQUENCE: [u8; 5] = [REQUEST_HEADER, 0x02, COMMAND, 0x01, 0xe1];

// header + length
const DATA_OFFSET: usize = 2;
// CMD echo + DF1 - DF16
const DATA_LENGTH: u8 = 17;
// header + length + data + checksum
const FRAME_LENGTH: usize = DATA_OFFSET + DATA_LENGTH as usize + 1;

// Maximum number of stray bytes skipped while looking for the response header
const MAX_SYNC_BYTES: usize = 2 * FRAME_LENGTH;

pub struct Pm1006<Uart> {
    uart: Uart,
    buffer: [u8; FRAME_LENGTH],
}

/// Response structure:
/// ```text
///   1 byte:   0x16
///   1 byte:   length N of response data
///   N bytes:  response data (CMD + Data frames)
///   1 byte:   check sum
///
/// PM2.5 = DF3 * 256 + DF4 (indexed from 1)
/// ```
fn parse_response<E>(response: &[u8; FRAME_LENGTH]) -> Result<u16, errors::Error<E>> {
    // Check header
    if response[0] != RESPONSE_HEADER {
        return Err(errors::Error::InvalidHeader(response[0]));
    }

    let length = response[1];
    if length != DATA_LENGTH {
        return Err(errors::Error::InvalidLength(length));
    }

    let data = &response[DATA_OFFSET..DATA_OFFSET + length as usize];

    // CMD
    if data[0] != COMMAND {
        return Err(errors::Error::InvalidCommandResponse(data[0]));
    }

    // The checksum byte is chosen so that all bytes of the frame sum to 0
    let sum = response[..DATA_OFFSET + length as usize]
        .iter()
        .fold(0u8, |sum, byte| sum.wrapping_add(*byte));
    let calculated = 0u8.wrapping_sub(sum);

    let expected = response[DATA_OFFSET + length as usize];
    if expected != calculated {
        return Err(errors::Error::InvalidChecksum(ChecksumMismatch {
            expected,
            calculated,
        }));
    }

    // PM2.5 = DF3 * 256 + DF4 (indexed from 1)
    Ok(u16::from_be_bytes([data[3], data[4]]))
}

impl<Uart, E> Pm1006<Uart>
where
    Uart: Read<Error = E> + Write,
{
    pub fn new(uart: Uart) -> Self {
        Self {
            uart,
            buffer: [0; FRAME_LENGTH],
        }
    }

    pub fn read_pm25(&mut self) -> Result<u16, errors::Error<E>> {
        self.send_command()?;
        self.read_response()?;

        parse_response::<E>(&self.buffer)
    }

    fn read_response(&mut self) -> Result<(), errors::Error<E>> {
        // Skip any stray bytes until the response header is found
        let mut skipped = 0;
        loop {
            self.uart.read_exact(&mut self.buffer[..1])?;
            if self.buffer[0] == RESPONSE_HEADER {
                break;
            }
            skipped += 1;
            if skipped >= MAX_SYNC_BYTES {
                return Err(errors::Error::InvalidHeader(self.buffer[0]));
            }
        }

        self.uart.read_exact(&mut self.buffer[1..DATA_OFFSET])?;
        let length = self.buffer[1];
        if length != DATA_LENGTH {
            return Err(errors::Error::InvalidLength(length));
        }

        self.uart
            .read_exact(&mut self.buffer[DATA_OFFSET..FRAME_LENGTH])?;
        Ok(())
    }

    fn send_command(&mut self) -> Result<(), errors::Error<E>> {
        self.uart
            .write_all(&COMMAND_SEQUENCE)
            .map_err(errors::Error::SerialWriteFail)?;
        self.uart.flush().map_err(errors::Error::SerialWriteFail)
    }
}

impl<Uart> Pm1006<Uart> {
    /// Consumes the driver and returns the underlying UART
    pub fn release(self) -> Uart {
        self.uart
    }
}

pub use errors::ChecksumMismatch;

pub mod errors {
    use core::fmt;
    use embedded_io::ReadExactError;

    #[derive(Debug, Clone, Copy, PartialEq, Eq)]
    pub struct ChecksumMismatch {
        /// Checksum byte received from the sensor
        pub expected: u8,
        /// Checksum calculated from the received frame
        pub calculated: u8,
    }

    #[derive(Debug)]
    pub enum Error<E> {
        InvalidHeader(u8),
        InvalidLength(u8),
        InvalidCommandResponse(u8),
        InvalidChecksum(ChecksumMismatch),
        UnexpectedEof,
        SerialReadFail(E),
        SerialWriteFail(E),
    }

    impl<E: fmt::Debug> fmt::Display for Error<E> {
        fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
            match self {
                Error::InvalidHeader(byte) => write!(f, "invalid response header: {byte:#04x}"),
                Error::InvalidLength(length) => write!(f, "invalid response length: {length}"),
                Error::InvalidCommandResponse(byte) => {
                    write!(f, "invalid command in response: {byte:#04x}")
                }
                Error::InvalidChecksum(mismatch) => write!(
                    f,
                    "invalid checksum: expected {:#04x}, calculated {:#04x}",
                    mismatch.expected, mismatch.calculated
                ),
                Error::UnexpectedEof => write!(f, "unexpected end of response"),
                Error::SerialReadFail(e) => write!(f, "serial read failed: {e:?}"),
                Error::SerialWriteFail(e) => write!(f, "serial write failed: {e:?}"),
            }
        }
    }

    impl<E: fmt::Debug> core::error::Error for Error<E> {}

    impl<E> From<ReadExactError<E>> for Error<E> {
        fn from(error: ReadExactError<E>) -> Self {
            match error {
                ReadExactError::UnexpectedEof => Error::UnexpectedEof,
                ReadExactError::Other(e) => Error::SerialReadFail(e),
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::errors::Error;
    use super::*;
    use core::convert::Infallible;

    // an example response taken from the actual device
    const RESPONSE: [u8; FRAME_LENGTH] = [
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

    /// UART mock returning at most `chunk` bytes per read/write call
    struct MockUart {
        rx: Vec<u8>,
        rx_pos: usize,
        tx: Vec<u8>,
        chunk: usize,
    }

    impl MockUart {
        fn new(rx: &[u8], chunk: usize) -> Self {
            Self {
                rx: rx.to_vec(),
                rx_pos: 0,
                tx: Vec::new(),
                chunk,
            }
        }
    }

    impl embedded_io::ErrorType for MockUart {
        type Error = Infallible;
    }

    impl Read for MockUart {
        fn read(&mut self, buf: &mut [u8]) -> Result<usize, Self::Error> {
            let n = buf.len().min(self.chunk).min(self.rx.len() - self.rx_pos);
            buf[..n].copy_from_slice(&self.rx[self.rx_pos..self.rx_pos + n]);
            self.rx_pos += n;
            Ok(n)
        }
    }

    impl Write for MockUart {
        fn write(&mut self, buf: &[u8]) -> Result<usize, Self::Error> {
            let n = buf.len().min(self.chunk);
            self.tx.extend_from_slice(&buf[..n]);
            Ok(n)
        }

        fn flush(&mut self) -> Result<(), Self::Error> {
            Ok(())
        }
    }

    #[test]
    fn test_parse_response() {
        // result = DF3 * 256 + DF4
        // 0 * 256 + 9 = 9
        let result = parse_response::<()>(&RESPONSE);
        assert!(matches!(result, Ok(9)));
    }

    #[test]
    fn test_parse_response_with_2_data_frames() {
        let mut response = RESPONSE;
        response[5] = 1; // DF3
        response[19] = 90; // checksum
                           // 1 * 256 + 9 = 265
        let result = parse_response::<()>(&response);
        assert!(matches!(result, Ok(265)));
    }

    #[test]
    fn test_parse_invalid_header() {
        let mut response = RESPONSE;
        response[0] = 0x17;
        let result = parse_response::<()>(&response);
        assert!(matches!(result, Err(Error::InvalidHeader(0x17))));
    }

    #[test]
    fn test_parse_invalid_length_does_not_panic() {
        for length in [0, 4, 16, 18, 200, 255] {
            let mut response = RESPONSE;
            response[1] = length;
            let result = parse_response::<()>(&response);
            assert!(matches!(result, Err(Error::InvalidLength(l)) if l == length));
        }
    }

    #[test]
    fn test_parse_invalid_command() {
        let mut response = RESPONSE;
        response[2] = 0x0c;
        let result = parse_response::<()>(&response);
        assert!(matches!(result, Err(Error::InvalidCommandResponse(0x0c))));
    }

    #[test]
    fn test_parse_invalid_checksum() {
        let mut response = RESPONSE;
        response[19] = 0;
        let result = parse_response::<()>(&response);
        assert!(matches!(
            result,
            Err(Error::InvalidChecksum(ChecksumMismatch {
                expected: 0,
                calculated: 91,
            }))
        ));
    }

    #[test]
    fn test_error_display() {
        let error = Error::<()>::InvalidChecksum(ChecksumMismatch {
            expected: 0,
            calculated: 0x5b,
        });
        assert_eq!(
            error.to_string(),
            "invalid checksum: expected 0x00, calculated 0x5b"
        );
        assert_eq!(
            Error::<()>::InvalidHeader(0x17).to_string(),
            "invalid response header: 0x17"
        );
    }

    #[test]
    fn test_release() {
        let mut sensor = Pm1006::new(MockUart::new(&RESPONSE, usize::MAX));
        assert!(matches!(sensor.read_pm25(), Ok(9)));
        let uart = sensor.release();
        assert_eq!(uart.tx, COMMAND_SEQUENCE);
    }

    #[test]
    fn test_read_pm25() {
        let mut sensor = Pm1006::new(MockUart::new(&RESPONSE, usize::MAX));
        assert!(matches!(sensor.read_pm25(), Ok(9)));
        assert_eq!(sensor.uart.tx, COMMAND_SEQUENCE);
    }

    #[test]
    fn test_read_pm25_with_short_reads_and_writes() {
        let mut sensor = Pm1006::new(MockUart::new(&RESPONSE, 1));
        assert!(matches!(sensor.read_pm25(), Ok(9)));
        assert_eq!(sensor.uart.tx, COMMAND_SEQUENCE);
    }

    #[test]
    fn test_read_pm25_truncated_response() {
        let mut sensor = Pm1006::new(MockUart::new(&RESPONSE[..10], 3));
        assert!(matches!(sensor.read_pm25(), Err(Error::UnexpectedEof)));
    }

    #[test]
    fn test_read_pm25_does_not_return_stale_value() {
        let mut rx = RESPONSE.to_vec();
        rx.extend_from_slice(&RESPONSE[..5]);
        let mut sensor = Pm1006::new(MockUart::new(&rx, 7));
        assert!(matches!(sensor.read_pm25(), Ok(9)));
        assert!(matches!(sensor.read_pm25(), Err(Error::UnexpectedEof)));
    }

    #[test]
    fn test_read_pm25_skips_stray_bytes() {
        let mut rx = vec![0x00, 0xff, 0x0b, 0x11];
        rx.extend_from_slice(&RESPONSE);
        let mut sensor = Pm1006::new(MockUart::new(&rx, 4));
        assert!(matches!(sensor.read_pm25(), Ok(9)));
    }

    #[test]
    fn test_read_pm25_header_not_found() {
        let rx = [0x00; MAX_SYNC_BYTES + FRAME_LENGTH];
        let mut sensor = Pm1006::new(MockUart::new(&rx, usize::MAX));
        assert!(matches!(
            sensor.read_pm25(),
            Err(Error::InvalidHeader(0x00))
        ));
    }

    #[test]
    fn test_read_pm25_invalid_length() {
        let mut rx = RESPONSE;
        rx[1] = 200;
        let mut sensor = Pm1006::new(MockUart::new(&rx, usize::MAX));
        assert!(matches!(sensor.read_pm25(), Err(Error::InvalidLength(200))));
    }
}
