//! I2C master access. Every `write` / `read` is one transaction ended by STOP;
//! `write_read` joins its two halves with a repeated START.
use std::fs::{File, OpenOptions};
use std::io::{self, Read, Write};
use std::os::fd::AsRawFd;

pub trait I2cBus: Send {
    fn write(&mut self, data: &[u8]) -> io::Result<()>;
    fn read(&mut self, buffer: &mut [u8]) -> io::Result<()>;

    /// Write, repeated START, read. Not emulated with `write` + `read`: a device that
    /// needs the repeated START would see a STOP instead.
    fn write_read(&mut self, _data: &[u8], _buffer: &mut [u8]) -> io::Result<()> {
        Err(io::Error::from(io::ErrorKind::Unsupported))
    }
}

/// `/dev/i2c-N` bound to one 7-bit device address.
pub struct LinuxI2c {
    file: File,
    address: u8,
}

// <linux/i2c-dev.h>
const I2C_SLAVE: libc::c_ulong = 0x0703;
const I2C_RDWR: libc::c_ulong = 0x0707;
// <linux/i2c.h>
const I2C_M_RD: u16 = 0x0001;

/// `struct i2c_msg`
#[repr(C)]
struct I2cMsg {
    addr: u16,
    flags: u16,
    len: u16,
    buf: *mut u8,
}

/// `struct i2c_rdwr_ioctl_data`
#[repr(C)]
struct I2cRdwrData {
    msgs: *mut I2cMsg,
    nmsgs: u32,
}

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
        Ok(Self { file, address })
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

    fn write_read(&mut self, data: &[u8], buffer: &mut [u8]) -> io::Result<()> {
        let length = |bytes: usize| {
            u16::try_from(bytes).map_err(|_| io::Error::from(io::ErrorKind::InvalidInput))
        };
        let mut messages = [
            I2cMsg {
                addr: u16::from(self.address),
                flags: 0,
                len: length(data.len())?,
                // The kernel only reads from a message without I2C_M_RD.
                buf: data.as_ptr().cast_mut(),
            },
            I2cMsg {
                addr: u16::from(self.address),
                flags: I2C_M_RD,
                len: length(buffer.len())?,
                buf: buffer.as_mut_ptr(),
            },
        ];
        let mut request = I2cRdwrData {
            msgs: messages.as_mut_ptr(),
            nmsgs: messages.len() as u32,
        };
        // SAFETY: `request`, `messages` and both buffers outlive the call, and each `len`
        // is the length of its buffer.
        let result = unsafe {
            libc::ioctl(
                self.file.as_raw_fd(),
                I2C_RDWR,
                &raw mut request as *mut libc::c_void,
            )
        };
        match result {
            count if count < 0 => Err(io::Error::last_os_error()),
            2 => Ok(()),
            _ => Err(io::Error::from(io::ErrorKind::UnexpectedEof)),
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn ioctl_structs_match_the_64_bit_kernel_layout() {
        assert_eq!(std::mem::size_of::<I2cMsg>(), 16);
        assert_eq!(std::mem::offset_of!(I2cMsg, buf), 8);
        assert_eq!(std::mem::size_of::<I2cRdwrData>(), 16);
        assert_eq!(std::mem::offset_of!(I2cRdwrData, nmsgs), 8);
    }
}
