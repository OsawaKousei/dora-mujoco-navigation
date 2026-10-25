use dora_mujoco_navigation::key_input::KeyAction;
use dora_node_api::{DoraNode, Event, into_vec};
use eyre::Result;

const LINEAR_SPEED: f64 = 0.24;
const ANGULAR_SPEED: f64 = 1.0;

#[derive(Default)]
struct TeleopCommand {
    command: [f64; 2],
}

impl TeleopCommand {
    fn on_key(&mut self, key: u8) -> [f64; 2] {
        self.command = match KeyAction::from_byte(key) {
            Some(KeyAction::Forward) => [LINEAR_SPEED, 0.0],
            Some(KeyAction::Backward) => [-LINEAR_SPEED, 0.0],
            Some(KeyAction::Left) => [0.0, ANGULAR_SPEED],
            Some(KeyAction::Right) => [0.0, -ANGULAR_SPEED],
            Some(KeyAction::Stop | KeyAction::Quit) => [0.0, 0.0],
            None => self.command,
        };
        self.command
    }

    fn current(&self) -> [f64; 2] {
        self.command
    }

    fn stop(&mut self) -> [f64; 2] {
        self.command = [0.0, 0.0];
        self.command
    }
}

fn send_command(node: &mut DoraNode, command: [f64; 2]) -> Result<()> {
    node.send_output("cmd_vel".into(), Default::default(), command.to_vec())?;
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn maps_every_direction_and_stop_key() {
        let mut state = TeleopCommand::default();
        assert_eq!(state.on_key(KeyAction::Forward as u8), [0.24, 0.0]);
        assert_eq!(state.on_key(KeyAction::Backward as u8), [-0.24, 0.0]);
        assert_eq!(state.on_key(KeyAction::Left as u8), [0.0, 1.0]);
        assert_eq!(state.on_key(KeyAction::Right as u8), [0.0, -1.0]);
        assert_eq!(state.on_key(KeyAction::Stop as u8), [0.0, 0.0]);
        assert_eq!(state.on_key(KeyAction::Forward as u8), [0.24, 0.0]);
        assert_eq!(state.on_key(KeyAction::Quit as u8), [0.0, 0.0]);
    }

    #[test]
    fn keeps_command_on_tick_or_unknown_key_and_stops_on_disconnect() {
        let mut state = TeleopCommand::default();
        assert_eq!(state.current(), [0.0, 0.0]);
        state.on_key(KeyAction::Forward as u8);
        assert_eq!(state.current(), [0.24, 0.0]);
        assert_eq!(state.on_key(255), [0.24, 0.0]);
        assert_eq!(state.stop(), [0.0, 0.0]);
        assert_eq!(state.current(), [0.0, 0.0]);
    }
}

fn main() -> Result<()> {
    let (mut node, mut events) = DoraNode::init_from_env()?;
    let mut command = TeleopCommand::default();
    while let Some(event) = events.recv() {
        match event {
            Event::Input { id, data, .. } if id.as_str() == "key_event" => {
                let values = into_vec::<u8>(&data)?;
                if let [key] = values.as_slice() {
                    send_command(&mut node, command.on_key(*key))?;
                }
            }
            Event::Input { id, .. } if id.as_str() == "tick" => {
                send_command(&mut node, command.current())?;
            }
            Event::InputClosed { id } if id.as_str() == "key_event" => {
                send_command(&mut node, command.stop())?;
            }
            Event::Stop(_) => {
                send_command(&mut node, command.stop())?;
                break;
            }
            _ => {}
        }
    }
    Ok(())
}
