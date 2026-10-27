use std::time::{Duration, Instant};

use dora_mujoco_navigation::nav_messages::{
    pose_from_values, stamp_from_metadata, stamped_parameters,
};
use dora_mujoco_navigation::navigation::{Pose, Scenario};
use dora_node_api::{DoraNode, Event, Parameter, into_vec};
use eyre::Result;

struct Noise(u64);

impl Noise {
    fn gaussian(&mut self) -> f64 {
        self.0 ^= self.0 << 13;
        self.0 ^= self.0 >> 7;
        self.0 ^= self.0 << 17;
        let a = ((self.0 >> 11) as f64 + 1.0) / ((1_u64 << 53) as f64 + 1.0);
        self.0 ^= self.0 << 13;
        self.0 ^= self.0 >> 7;
        self.0 ^= self.0 << 17;
        let b = (self.0 >> 11) as f64 / (1_u64 << 53) as f64;
        (-2.0 * a.ln()).sqrt() * (std::f64::consts::TAU * b).cos()
    }
}

fn scan(scenario: &Scenario, pose: Pose, noise: &mut Noise) -> Vec<f64> {
    (0..scenario.lidar.ray_count)
        .map(|index| {
            let angle = scenario.lidar.min_angle + index as f64 * scenario.lidar_angle_increment();
            let distance = scenario.raycast(pose, angle);
            if distance >= scenario.lidar.max_range {
                scenario.lidar.max_range
            } else {
                (distance + scenario.lidar.noise_stddev * noise.gaussian())
                    .clamp(0.0, scenario.lidar.max_range)
            }
        })
        .collect()
}

fn main() -> Result<()> {
    let scenario = Scenario::load_from_env()?;
    let (mut node, mut events) = DoraNode::init_from_env()?;
    let mut latest = None;
    let mut last_sent = -1;
    let mut noise = Noise(0x9e3779b97f4a7c15);
    while let Some(event) = events.recv() {
        match event {
            Event::Input { id, data, metadata } if id.as_str() == "robot_pose" => {
                let values = into_vec::<f64>(&data)?;
                latest = Some((
                    pose_from_values(&values)?,
                    stamp_from_metadata(&metadata)?,
                    Instant::now(),
                ));
            }
            Event::Input { id, .. } if id.as_str() == "tick" => {
                if let Some((pose, stamp, received)) = latest
                    && stamp.step > last_sent
                    && received.elapsed() < Duration::from_millis(100)
                {
                    let mut params = stamped_parameters(stamp);
                    params.insert(
                        "min_angle".into(),
                        Parameter::Float(scenario.lidar.min_angle),
                    );
                    params.insert(
                        "angle_increment".into(),
                        Parameter::Float(scenario.lidar_angle_increment()),
                    );
                    params.insert(
                        "max_range".into(),
                        Parameter::Float(scenario.lidar.max_range),
                    );
                    node.send_output("scan".into(), params, scan(&scenario, pose, &mut noise))?;
                    last_sent = stamp.step;
                }
            }
            Event::InputClosed { id } if id.as_str() == "robot_pose" => break,
            Event::Stop(_) => break,
            _ => {}
        }
    }
    Ok(())
}
