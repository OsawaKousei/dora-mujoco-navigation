use dora_mujoco_navigation::differential_drive::{TRACK_WIDTH, WHEEL_RADIUS};
use dora_mujoco_navigation::nav_messages::{
    finite_values, stamp_from_metadata, stamped_parameters,
};
use dora_node_api::{DoraNode, Event, into_vec};
use eyre::{Result, ensure};

fn wheel_delta(previous: [f64; 2], current: [f64; 2]) -> [f64; 3] {
    let left = (current[0] - previous[0]) * WHEEL_RADIUS;
    let right = (current[1] - previous[1]) * WHEEL_RADIUS;
    [(left + right) / 2.0, 0.0, (right - left) / TRACK_WIDTH]
}

fn main() -> Result<()> {
    let (mut node, mut events) = DoraNode::init_from_env()?;
    let mut previous: Option<(i64, [f64; 2])> = None;
    while let Some(event) = events.recv() {
        match event {
            Event::Input { id, data, metadata } if id.as_str() == "encoder" => {
                let values = into_vec::<f64>(&data)?;
                finite_values(&values, 2)?;
                let stamp = stamp_from_metadata(&metadata)?;
                let current = [values[0], values[1]];
                let delta = if let Some((last_step, last_angles)) = previous {
                    ensure!(
                        stamp.step == last_step + 1,
                        "encoder step gap: {last_step} -> {}",
                        stamp.step
                    );
                    wheel_delta(last_angles, current)
                } else {
                    [0.0; 3]
                };
                previous = Some((stamp.step, current));
                node.send_output("delta".into(), stamped_parameters(stamp), delta.to_vec())?;
            }
            Event::InputClosed { id } if id.as_str() == "encoder" => break,
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
    fn differential_wheel_motion() {
        assert_eq!(wheel_delta([0.0, 0.0], [1.0, 1.0]), [0.06, 0.0, 0.0]);
        let turn = wheel_delta([0.0, 0.0], [-1.0, 1.0]);
        assert!(turn[0].abs() < 1e-12 && (turn[2] - 0.5).abs() < 1e-12);
    }
}
