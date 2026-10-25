use std::env;
use std::fs;
use std::io::{self, Read};
use std::os::unix::net::{UnixListener, UnixStream};
use std::path::{Path, PathBuf};
use std::process::Command;
use std::thread;
use std::time::{Duration, Instant};

use dora_mujoco_navigation::key_input::KeyAction;
use dora_node_api::{DoraNode, Event, TryRecvError};
use eyre::{Result, WrapErr, bail};

struct SocketPath(PathBuf);

fn valid_key_bytes(bytes: &[u8]) -> impl Iterator<Item = u8> + '_ {
    bytes
        .iter()
        .copied()
        .filter(|byte| KeyAction::from_byte(*byte).is_some())
}

impl Drop for SocketPath {
    fn drop(&mut self) {
        let _ = fs::remove_file(&self.0);
    }
}

fn spawn_terminal(helper: &Path, socket: &Path) -> Result<()> {
    let terminal = env::var("TELEOP_TERMINAL").unwrap_or_else(|_| "gnome-terminal".to_owned());
    let mut command = Command::new(&terminal);
    if terminal.contains("gnome-terminal") {
        command.arg("--");
    } else {
        command.arg("-e");
    }
    command.arg(helper).arg(socket);
    command
        .spawn()
        .wrap_err_with(|| format!("failed to launch terminal: {terminal}"))?;
    Ok(())
}

fn main() -> Result<()> {
    let (mut node, mut events) = DoraNode::init_from_env()?;
    let helper = env::current_exe()?.with_file_name("keyboard_tty");
    if !helper.is_file() {
        bail!(
            "keyboard helper is missing: {}; build --bin keyboard_tty",
            helper.display()
        );
    }

    let socket_path = env::temp_dir().join(format!("dora-keyboard-{}.sock", std::process::id()));
    let _socket_path = SocketPath(socket_path.clone());
    let listener = UnixListener::bind(&socket_path).wrap_err("failed to bind keyboard socket")?;
    listener.set_nonblocking(true)?;
    spawn_terminal(&helper, &socket_path)?;

    let started = Instant::now();
    let mut stream: Option<UnixStream> = None;
    loop {
        match events.try_recv() {
            Ok(Event::Stop(_)) | Err(TryRecvError::Closed) => break,
            Err(TryRecvError::Empty) | Ok(_) => {}
        }

        if stream.is_none() {
            match listener.accept() {
                Ok((connected, _)) => {
                    connected.set_nonblocking(true)?;
                    eprintln!("keyboard: input terminal connected");
                    stream = Some(connected);
                }
                Err(error) if error.kind() == io::ErrorKind::WouldBlock => {
                    if started.elapsed() > Duration::from_secs(10) {
                        bail!("keyboard terminal did not connect within 10 seconds");
                    }
                }
                Err(error) => return Err(error.into()),
            }
        }

        if let Some(connected) = stream.as_mut() {
            let mut bytes = [0; 32];
            match connected.read(&mut bytes) {
                Ok(0) => {
                    node.send_output(
                        "key_event".into(),
                        Default::default(),
                        vec![KeyAction::Stop as u8],
                    )?;
                    break;
                }
                Ok(count) => {
                    for byte in valid_key_bytes(&bytes[..count]) {
                        node.send_output("key_event".into(), Default::default(), vec![byte])?;
                    }
                }
                Err(error) if error.kind() == io::ErrorKind::WouldBlock => {}
                Err(error) => return Err(error.into()),
            }
        }
        thread::sleep(Duration::from_millis(10));
    }
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn forwards_only_known_key_actions_in_order() {
        assert_eq!(
            valid_key_bytes(&[0, 1, 7, 5, 255, 6]).collect::<Vec<_>>(),
            vec![1, 5, 6]
        );
    }
}
