use std::collections::{BTreeMap, VecDeque};
use std::time::{Duration, Instant};

use dora_mujoco_navigation::localization::EkfState;
use dora_mujoco_navigation::nav_messages::{
    Stamp, finite_values, float_parameter, pose_values, stamp_from_metadata, stamped_parameters,
};
use dora_mujoco_navigation::navigation::Scenario;
use dora_node_api::{DoraNode, Event, into_vec};
use eyre::{Result, ensure};

#[derive(Clone)]
struct History {
    stamp: Stamp,
    delta: [f64; 3],
    state: EkfState,
}

fn apply_scan(
    history: &mut VecDeque<History>,
    step: i64,
    ranges: &[f64],
    scenario: &Scenario,
) -> bool {
    let Some(index) = history.iter().position(|entry| entry.stamp.step == step) else {
        return false;
    };
    let mut state = history[index].state;
    state.update_scan(ranges, scenario);
    history[index].state = state;
    for next in index + 1..history.len() {
        state.predict(history[next].delta);
        history[next].state = state;
    }
    true
}

fn send_latest(node: &mut DoraNode, history: &VecDeque<History>) -> Result<()> {
    let latest = history.back().expect("history has initial state");
    node.send_output(
        "pose".into(),
        stamped_parameters(latest.stamp),
        pose_values(latest.state.pose),
    )?;
    Ok(())
}

fn main() -> Result<()> {
    let scenario = Scenario::load_from_env()?;
    let (mut node, mut events) = DoraNode::init_from_env()?;
    let initial = History {
        stamp: Stamp {
            step: 0,
            sim_time_ns: 0,
        },
        delta: [0.0; 3],
        state: EkfState::new(scenario.robot.start),
    };
    let mut history = VecDeque::from([initial]);
    let mut pending = BTreeMap::<i64, Vec<f64>>::new();
    let started = Instant::now();
    let mut last_scan = None;
    let mut latest_step = 0;
    while let Some(event) = events.recv() {
        match event {
            Event::Input { id, data, metadata } if id.as_str() == "odom_delta" => {
                let values = into_vec::<f64>(&data)?;
                finite_values(&values, 3)?;
                let stamp = stamp_from_metadata(&metadata)?;
                ensure!(
                    stamp.step == latest_step + 1,
                    "odometry step gap: {latest_step} -> {}",
                    stamp.step
                );
                let delta = [values[0], values[1], values[2]];
                let mut state = history.back().unwrap().state;
                state.predict(delta);
                history.push_back(History {
                    stamp,
                    delta,
                    state,
                });
                latest_step = stamp.step;
                while history.len() > 120 {
                    history.pop_front();
                }
                let due: Vec<_> = pending
                    .range(..=latest_step)
                    .map(|(&step, _)| step)
                    .collect();
                for step in due {
                    if let Some(ranges) = pending.remove(&step) {
                        apply_scan(&mut history, step, &ranges, &scenario);
                    }
                }
                if last_scan.is_some_and(|at: Instant| at.elapsed() <= Duration::from_millis(800))
                    || started.elapsed() <= Duration::from_millis(800)
                {
                    send_latest(&mut node, &history)?;
                }
            }
            Event::Input { id, data, metadata } if id.as_str() == "scan" => {
                let ranges = into_vec::<f64>(&data)?;
                finite_values(&ranges, scenario.lidar.ray_count)?;
                ensure!(
                    ranges
                        .iter()
                        .all(|v| *v >= 0.0 && *v <= scenario.lidar.max_range),
                    "invalid lidar range"
                );
                ensure!(
                    (float_parameter(&metadata, "min_angle")? - scenario.lidar.min_angle).abs()
                        < 1e-9,
                    "lidar min angle mismatch"
                );
                ensure!(
                    (float_parameter(&metadata, "angle_increment")?
                        - scenario.lidar_angle_increment())
                    .abs()
                        < 1e-9,
                    "lidar increment mismatch"
                );
                ensure!(
                    (float_parameter(&metadata, "max_range")? - scenario.lidar.max_range).abs()
                        < 1e-9,
                    "lidar max range mismatch"
                );
                let stamp = stamp_from_metadata(&metadata)?;
                last_scan = Some(Instant::now());
                if stamp.step > latest_step {
                    if stamp.step <= latest_step + 120 {
                        pending.insert(stamp.step, ranges);
                    }
                } else if apply_scan(&mut history, stamp.step, &ranges, &scenario) {
                    send_latest(&mut node, &history)?;
                }
            }
            Event::InputClosed { id } if matches!(id.as_str(), "odom_delta" | "scan") => break,
            Event::Stop(_) => break,
            _ => {}
        }
    }
    Ok(())
}
