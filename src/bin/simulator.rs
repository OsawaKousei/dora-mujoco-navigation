use std::env;
use std::time::{Duration, Instant};

use dora_mujoco_navigation::wheel_speeds;
use dora_node_api::{DoraNode, Event, into_vec};
use eyre::{Result, WrapErr, eyre};
use mujoco_rs::prelude::{MjModel, MjtObj};
use mujoco_rs::viewer::MjViewer;

const COMMAND_TIMEOUT: Duration = Duration::from_millis(300);

fn main() -> Result<()> {
    let model_path = env::var("MUJOCO_MODEL_PATH")
        .unwrap_or_else(|_| concat!(env!("CARGO_MANIFEST_DIR"), "/assets/robot.xml").to_owned());
    let model = MjModel::from_xml(&model_path).wrap_err("failed to load MuJoCo model")?;
    let robot_id = model
        .name_to_id(MjtObj::mjOBJ_BODY, "robot")
        .ok_or_else(|| eyre!("body 'robot' is missing"))?;
    let left_id = model
        .name_to_id(MjtObj::mjOBJ_ACTUATOR, "left_motor")
        .ok_or_else(|| eyre!("actuator 'left_motor' is missing"))?;
    let right_id = model
        .name_to_id(MjtObj::mjOBJ_ACTUATOR, "right_motor")
        .ok_or_else(|| eyre!("actuator 'right_motor' is missing"))?;

    let mut data = model.make_data();
    let mut viewer = MjViewer::builder()
        .build_passive(&model)
        .wrap_err("failed to open MuJoCo viewer")?;
    let (mut node, mut events) = DoraNode::init_from_env()?;
    let mut command = [0.0_f64; 2];
    let mut last_command: Option<Instant> = None;
    let mut tick_count = 0_u64;

    while let Some(event) = events.recv() {
        match event {
            Event::Input {
                id, data: input, ..
            } if id.as_str() == "cmd_vel" => match into_vec::<f64>(&input) {
                Ok(values) if values.len() == 2 && values.iter().all(|v| v.is_finite()) => {
                    command.copy_from_slice(&values);
                    last_command = Some(Instant::now());
                }
                _ => {
                    eprintln!("simulator: invalid cmd_vel; stopping robot");
                    command = [0.0, 0.0];
                    last_command = None;
                }
            },
            Event::Input { id, .. } if id.as_str() == "tick" => {
                if last_command.is_none_or(|last| last.elapsed() > COMMAND_TIMEOUT) {
                    command = [0.0, 0.0];
                }
                let [left, right] = wheel_speeds(command[0], command[1]);
                data.ctrl_mut()[left_id] = left;
                data.ctrl_mut()[right_id] = right;
                // robot.xml uses a 5 ms physics step; the dora timer runs at 100 Hz.
                data.step();
                data.step();

                let [x, y, _] = data.xpos()[robot_id];
                let [w, qx, qy, qz] = data.xquat()[robot_id];
                let yaw = (2.0 * (w * qz + qx * qy)).atan2(1.0 - 2.0 * (qy * qy + qz * qz));
                node.send_output("robot_pose".into(), Default::default(), vec![x, y, yaw])?;

                tick_count += 1;
                if tick_count.is_multiple_of(2) {
                    viewer.sync_data(&mut data);
                    viewer.render()?;
                    if !viewer.running() {
                        break;
                    }
                }
            }
            Event::Stop(_) => break,
            _ => {}
        }
    }
    Ok(())
}
