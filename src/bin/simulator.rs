use std::env;
use std::time::{Duration, Instant};

use dora_mujoco_navigation::differential_drive::wheel_speeds;
use dora_mujoco_navigation::nav_messages::{Stamp, stamped_parameters};
use dora_mujoco_navigation::navigation::Scenario;
use dora_node_api::{DoraNode, Event, into_vec};
use eyre::{Result, WrapErr, eyre};
use mujoco_rs::prelude::{MjModel, MjtObj};
use mujoco_rs::viewer::MjViewer;

const COMMAND_TIMEOUT: Duration = Duration::from_millis(300);

#[derive(Default)]
struct SimulatorCommand {
    command: [f64; 2],
    last_received: Option<Instant>,
}

impl SimulatorCommand {
    fn accept(&mut self, values: &[f64], now: Instant) -> bool {
        match values {
            [linear, angular] if linear.is_finite() && angular.is_finite() => {
                self.command = [*linear, *angular];
                self.last_received = Some(now);
                true
            }
            _ => {
                self.stop();
                false
            }
        }
    }

    fn stop(&mut self) {
        self.command = [0.0, 0.0];
        self.last_received = None;
    }

    fn current(&mut self, now: Instant) -> [f64; 2] {
        if self
            .last_received
            .is_none_or(|last| now.duration_since(last) > COMMAND_TIMEOUT)
        {
            self.stop();
        }
        self.command
    }
}

fn main() -> Result<()> {
    let autonomous = env::var_os("AUTO_DRIVE_SCENARIO").is_some();
    let scenario = autonomous.then(Scenario::load_from_env).transpose()?;
    let model = if let Some(scenario) = &scenario {
        MjModel::from_xml_string(&scenario.mujoco_xml()?)
            .wrap_err("failed to build autonomous MuJoCo model")?
    } else {
        let model_path = env::var("MUJOCO_MODEL_PATH").unwrap_or_else(|_| {
            concat!(env!("CARGO_MANIFEST_DIR"), "/assets/robot.xml").to_owned()
        });
        MjModel::from_xml(&model_path).wrap_err("failed to load MuJoCo model")?
    };
    let robot_id = model
        .name_to_id(MjtObj::mjOBJ_BODY, "robot")
        .ok_or_else(|| eyre!("body 'robot' is missing"))?;
    let left_id = model
        .name_to_id(MjtObj::mjOBJ_ACTUATOR, "left_motor")
        .ok_or_else(|| eyre!("actuator 'left_motor' is missing"))?;
    let right_id = model
        .name_to_id(MjtObj::mjOBJ_ACTUATOR, "right_motor")
        .ok_or_else(|| eyre!("actuator 'right_motor' is missing"))?;
    let wall_ids: Vec<_> = if let Some(scenario) = &scenario {
        (0..scenario.obstacles.len())
            .map(|index| {
                model
                    .name_to_id(MjtObj::mjOBJ_GEOM, &format!("nav_wall_{index}"))
                    .ok_or_else(|| eyre!("MuJoCo wall {index} is missing"))
            })
            .collect::<Result<Vec<_>>>()?
    } else {
        Vec::new()
    };

    let mut data = model.make_data();
    let mut viewer = if env::var("MUJOCO_HEADLESS").as_deref() == Ok("1") {
        None
    } else {
        Some(
            MjViewer::builder()
                .build_passive(&model)
                .wrap_err("failed to open MuJoCo viewer")?,
        )
    };
    let (mut node, mut events) = DoraNode::init_from_env()?;
    let mut command = SimulatorCommand::default();
    let mut tick_count = 0_u64;
    let mut collided = false;

    while let Some(event) = events.recv() {
        match event {
            Event::Input {
                id, data: input, ..
            } if id.as_str() == "cmd_vel" => {
                if !into_vec::<f64>(&input)
                    .is_ok_and(|values| command.accept(&values, Instant::now()))
                {
                    eprintln!("simulator: invalid cmd_vel; stopping robot");
                    command.stop();
                }
            }
            Event::Input { id, .. } if id.as_str() == "tick" => {
                let [linear, angular] = if collided {
                    [0.0, 0.0]
                } else {
                    command.current(Instant::now())
                };
                let speeds = wheel_speeds(linear, angular);
                data.ctrl_mut()[left_id] = speeds.left;
                data.ctrl_mut()[right_id] = speeds.right;
                // robot.xml uses a 5 ms physics step; the dora timer runs at 100 Hz.
                data.step();
                data.step();

                let [x, y, _] = data.xpos()[robot_id];
                let [w, qx, qy, qz] = data.xquat()[robot_id];
                let yaw = (2.0 * (w * qz + qx * qy)).atan2(1.0 - 2.0 * (qy * qy + qz * qz));
                tick_count += 1;
                if scenario.is_some() {
                    let stamp = Stamp {
                        step: tick_count as i64,
                        sim_time_ns: (data.time() * 1e9).round() as i64,
                    };
                    node.send_output(
                        "encoder".into(),
                        stamped_parameters(stamp),
                        vec![data.sensordata()[0], data.sensordata()[1]],
                    )?;
                    node.send_output(
                        "robot_pose".into(),
                        stamped_parameters(stamp),
                        vec![x, y, yaw],
                    )?;
                    if !collided
                        && data.contact().iter().any(|contact| {
                            wall_ids.contains(&(contact.geom1 as usize))
                                || wall_ids.contains(&(contact.geom2 as usize))
                        })
                    {
                        collided = true;
                        eprintln!("simulator: collision at x={x:.3}, y={y:.3}");
                        command.stop();
                        node.send_output(
                            "collision".into(),
                            stamped_parameters(stamp),
                            vec![1_u8],
                        )?;
                    }
                } else {
                    node.send_output("robot_pose".into(), Default::default(), vec![x, y, yaw])?;
                }
                if tick_count.is_multiple_of(2)
                    && let Some(viewer) = viewer.as_mut()
                {
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

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn accepts_finite_pair_and_expires_after_300_ms() {
        let start = Instant::now();
        let mut state = SimulatorCommand::default();
        assert_eq!(state.current(start), [0.0, 0.0]);
        assert!(state.accept(&[0.24, -1.0], start));
        assert_eq!(state.current(start + COMMAND_TIMEOUT), [0.24, -1.0]);
        assert_eq!(
            state.current(start + COMMAND_TIMEOUT + Duration::from_millis(1)),
            [0.0, 0.0]
        );
    }

    #[test]
    fn malformed_or_nonfinite_command_stops_immediately() {
        let now = Instant::now();
        for values in [
            &[][..],
            &[0.1][..],
            &[0.1, 0.2, 0.3][..],
            &[f64::NAN, 0.0][..],
            &[0.0, f64::INFINITY][..],
        ] {
            let mut state = SimulatorCommand::default();
            assert!(state.accept(&[0.24, 0.0], now));
            assert!(!state.accept(values, now));
            assert_eq!(state.current(now), [0.0, 0.0]);
        }
    }
}
