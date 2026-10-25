#![cfg(unix)]

use std::fs::{self, File};
use std::io::{self, Read, Write};
use std::os::fd::{AsRawFd, FromRawFd};
use std::os::unix::net::UnixListener;
use std::path::PathBuf;
use std::process::{Child, Command, Stdio};
use std::thread;
use std::time::{Duration, Instant, SystemTime, UNIX_EPOCH};

const TIMEOUT: Duration = Duration::from_secs(3);

struct ChildGuard(Child);

impl Drop for ChildGuard {
    fn drop(&mut self) {
        let _ = self.0.kill();
        let _ = self.0.wait();
    }
}

struct SocketGuard(PathBuf);

impl Drop for SocketGuard {
    fn drop(&mut self) {
        let _ = fs::remove_file(&self.0);
    }
}

fn terminal_settings(fd: i32) -> libc::termios {
    let mut settings = unsafe { std::mem::zeroed() };
    assert_eq!(unsafe { libc::tcgetattr(fd, &mut settings) }, 0);
    settings
}

#[test]
fn tty_sends_actions_and_restores_terminal_on_quit() -> io::Result<()> {
    let unique = SystemTime::now()
        .duration_since(UNIX_EPOCH)
        .unwrap()
        .as_nanos();
    let path = std::env::temp_dir().join(format!(
        "keyboard-tty-test-{}-{unique}.sock",
        std::process::id()
    ));
    let _socket_guard = SocketGuard(path.clone());
    let listener = UnixListener::bind(&path)?;
    listener.set_nonblocking(true)?;

    let (mut master_fd, mut slave_fd) = (-1, -1);
    if unsafe {
        libc::openpty(
            &mut master_fd,
            &mut slave_fd,
            std::ptr::null_mut(),
            std::ptr::null_mut(),
            std::ptr::null_mut(),
        )
    } != 0
    {
        return Err(io::Error::last_os_error());
    }
    let mut master = unsafe { File::from_raw_fd(master_fd) };
    let slave = unsafe { File::from_raw_fd(slave_fd) };
    let original = terminal_settings(slave.as_raw_fd());
    let mut child = ChildGuard(
        Command::new(env!("CARGO_BIN_EXE_keyboard_tty"))
            .arg(&path)
            .stdin(Stdio::from(slave.try_clone()?))
            .stdout(Stdio::null())
            .stderr(Stdio::piped())
            .spawn()?,
    );

    let started = Instant::now();
    let mut connection = loop {
        match listener.accept() {
            Ok((stream, _)) => break stream,
            Err(error) if error.kind() == io::ErrorKind::WouldBlock => {
                assert!(started.elapsed() < TIMEOUT, "keyboard_tty did not connect");
                thread::sleep(Duration::from_millis(10));
            }
            Err(error) => return Err(error),
        }
    };
    connection.set_read_timeout(Some(TIMEOUT))?;

    while terminal_settings(slave.as_raw_fd()).c_lflag & libc::ICANON != 0 {
        assert!(
            started.elapsed() < TIMEOUT,
            "terminal did not enter raw mode"
        );
        thread::sleep(Duration::from_millis(10));
    }

    master.write_all(b"\x1b[A \x1b[Bq")?;
    let mut actions = [0_u8; 4];
    connection.read_exact(&mut actions)?;
    assert_eq!(actions, [1, 5, 2, 6]);

    let status = loop {
        if let Some(status) = child.0.try_wait()? {
            break status;
        }
        assert!(
            started.elapsed() < TIMEOUT,
            "keyboard_tty did not exit on Q"
        );
        thread::sleep(Duration::from_millis(10));
    };
    assert!(status.success(), "keyboard_tty exited with {status}");

    let restored = terminal_settings(slave.as_raw_fd());
    assert_eq!(restored.c_iflag, original.c_iflag);
    assert_eq!(restored.c_oflag, original.c_oflag);
    assert_eq!(restored.c_cflag, original.c_cflag);
    assert_eq!(restored.c_lflag, original.c_lflag);
    assert_eq!(restored.c_cc, original.c_cc);
    Ok(())
}
