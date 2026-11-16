//! I2C master access. Every `write` / `read` is one transaction ended by STOP.
use std::fs::{File, OpenOptions};
use std::io::{self, Read, Write};
use std::os::fd::AsRawFd;

pub trait I2cBus: Send {
    fn write(&mut self, data: &[u8]) -> io::Result<()>;
    fn read(&mut self, buffer: &mut [u8]) -> io::Result<()>;
}

/// `/dev/i2c-N` bound to one 7-bit device address.
pub struct LinuxI2c {
    file: File,
}

// <linux/i2c-dev.h>
const I2C_SLAVE: libc::c_ulong = 0x0703;

impl LinuxI2c {
    pub fn open(path: &str, address: u8) -> Result<Self, String> {
        if address > 0x7f {
            return Err(format!(
                "I2C address 0x{address:02x} is not a 7-bit address"
            ));
        }
        let file = OpenOptions::new()
            .read(true)
            .write(true)
            .open(path)
            .map_err(|error| format!("{path}: {error}"))?;
        // SAFETY: the descriptor is open and I2C_SLAVE takes the address by value.
        let result =
            unsafe { libc::ioctl(file.as_raw_fd(), I2C_SLAVE, libc::c_ulong::from(address)) };
        if result < 0 {
            return Err(format!(
                "{path}: I2C_SLAVE 0x{address:02x}: {}",
                io::Error::last_os_error()
            ));
        }
        Ok(Self { file })
    }
}

impl I2cBus for LinuxI2c {
    fn write(&mut self, data: &[u8]) -> io::Result<()> {
        // A single write() is a single transfer; write_all could split it.
        if self.file.write(data)? != data.len() {
            return Err(io::Error::other("short I2C write"));
        }
        Ok(())
    }

    fn read(&mut self, buffer: &mut [u8]) -> io::Result<()> {
        if self.file.read(buffer)? != buffer.len() {
            return Err(io::Error::other("short I2C read"));
        }
        Ok(())
    }
}
