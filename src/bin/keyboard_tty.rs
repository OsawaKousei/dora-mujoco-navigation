use std::env;
use std::io::{self, Read, Write};
use std::os::fd::AsRawFd;
use std::os::unix::net::UnixStream;

use dora_mujoco_navigation::key_input::{EscapeParser, KeyAction};
use eyre::{Result, WrapErr, bail};

struct RawTerminal {
    fd: i32,
    original: libc::termios,
}

impl RawTerminal {
    fn new(fd: i32) -> Result<Self> {
        let mut original = unsafe { std::mem::zeroed() };
        if unsafe { libc::tcgetattr(fd, &mut original) } != 0 {
            return Err(io::Error::last_os_error().into());
        }
        let mut raw = original;
        unsafe { libc::cfmakeraw(&mut raw) };
        if unsafe { libc::tcsetattr(fd, libc::TCSANOW, &raw) } != 0 {
            return Err(io::Error::last_os_error().into());
        }
        Ok(Self { fd, original })
    }
}

impl Drop for RawTerminal {
    fn drop(&mut self) {
        unsafe { libc::tcsetattr(self.fd, libc::TCSANOW, &self.original) };
    }
}

fn main() -> Result<()> {
    let socket = env::args_os()
        .nth(1)
        .ok_or_else(|| eyre::eyre!("socket path is required"))?;
    let mut stream = UnixStream::connect(socket).wrap_err("could not connect to keyboard node")?;
    let stdin = io::stdin();
    if unsafe { libc::isatty(stdin.as_raw_fd()) } != 1 {
        bail!("keyboard_tty must run inside a terminal");
    }

    println!("Arrow keys: forward / reverse / turn   Space: stop   Q: quit");
    println!("Keep this terminal focused while watching the MuJoCo window.");
    io::stdout().flush()?;
    let _raw = RawTerminal::new(stdin.as_raw_fd())?;
    let mut input = stdin.lock();
    let mut parser = EscapeParser::default();

    loop {
        let mut fds = [
            libc::pollfd {
                fd: stdin.as_raw_fd(),
                events: libc::POLLIN,
                revents: 0,
            },
            libc::pollfd {
                fd: stream.as_raw_fd(),
                events: libc::POLLIN,
                revents: 0,
            },
        ];
        let ready = unsafe { libc::poll(fds.as_mut_ptr(), fds.len() as libc::nfds_t, -1) };
        if ready < 0 {
            let error = io::Error::last_os_error();
            if error.kind() == io::ErrorKind::Interrupted {
                continue;
            }
            return Err(error.into());
        }

        if fds[1].revents != 0 {
            // The keyboard node has stopped or closed the connection.
            let mut byte = [0];
            match stream.read(&mut byte) {
                Ok(0) | Err(_) => break,
                Ok(_) => {}
            }
        }
        if fds[0].revents != 0 {
            let mut bytes = [0; 32];
            let count = input.read(&mut bytes)?;
            if count == 0 {
                break;
            }
            for byte in &bytes[..count] {
                if let Some(action) = parser.push(*byte) {
                    stream.write_all(&[action as u8])?;
                    if action == KeyAction::Quit {
                        return Ok(());
                    }
                }
            }
        }
    }
    Ok(())
}
