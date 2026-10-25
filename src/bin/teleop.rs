use dora_mujoco_navigation::key_input::KeyAction;
use dora_node_api::{DoraNode, Event, into_vec};
use eyre::Result;

const LINEAR_SPEED: f64 = 0.24;
const ANGULAR_SPEED: f64 = 1.0;

fn send_command(node: &mut DoraNode, command: [f64; 2]) -> Result<()> {
    node.send_output("cmd_vel".into(), Default::default(), command.to_vec())?;
    Ok(())
}

fn main() -> Result<()> {
    let (mut node, mut events) = DoraNode::init_from_env()?;
    let mut command = [0.0, 0.0];
    while let Some(event) = events.recv() {
        match event {
            Event::Input { id, data, .. } if id.as_str() == "key_event" => {
                let values = into_vec::<u8>(&data)?;
                if let [key] = values.as_slice() {
                    command = match KeyAction::from_byte(*key) {
                        Some(KeyAction::Forward) => [LINEAR_SPEED, 0.0],
                        Some(KeyAction::Backward) => [-LINEAR_SPEED, 0.0],
                        Some(KeyAction::Left) => [0.0, ANGULAR_SPEED],
                        Some(KeyAction::Right) => [0.0, -ANGULAR_SPEED],
                        Some(KeyAction::Stop | KeyAction::Quit) => [0.0, 0.0],
                        None => command,
                    };
                    send_command(&mut node, command)?;
                }
            }
            Event::Input { id, .. } if id.as_str() == "tick" => {
                send_command(&mut node, command)?;
            }
            Event::InputClosed { id } if id.as_str() == "key_event" => {
                command = [0.0, 0.0];
                send_command(&mut node, command)?;
            }
            Event::Stop(_) => {
                send_command(&mut node, [0.0, 0.0])?;
                break;
            }
            _ => {}
        }
    }
    Ok(())
}
