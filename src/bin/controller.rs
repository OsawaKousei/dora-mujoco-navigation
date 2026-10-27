use std::time::{Duration, Instant};

use dora_mujoco_navigation::nav_messages::{finite_values, float_parameter, pose_from_values};
use dora_mujoco_navigation::navigation::{Point, Pose, Scenario, normalize_angle};
use dora_node_api::{DoraNode, Event, Metadata, MetadataParameters, Parameter, into_vec};
use eyre::{Result, ensure};

#[derive(Clone)]
struct Route {
    points: Vec<Point>,
    goal: Point,
    tolerance: f64,
    id: String,
}

fn parse_route(values: &[f64], metadata: &Metadata) -> Result<Route> {
    ensure!(
        values.len() >= 4 && values.len().is_multiple_of(2),
        "path has invalid length"
    );
    ensure!(
        values.iter().all(|value| value.is_finite()),
        "path contains nonfinite values"
    );
    let points = values
        .chunks_exact(2)
        .map(|v| Point { x: v[0], y: v[1] })
        .collect();
    let id = match metadata.parameters.get("plan_id") {
        Some(Parameter::String(value)) => value.clone(),
        _ => eyre::bail!("path has no plan_id"),
    };
    let tolerance = float_parameter(metadata, "goal_tolerance")?;
    ensure!(tolerance > 0.0, "invalid goal tolerance");
    Ok(Route {
        points,
        goal: Point {
            x: float_parameter(metadata, "goal_x")?,
            y: float_parameter(metadata, "goal_y")?,
        },
        tolerance,
        id,
    })
}

fn target(path: &[Point], pose: Pose, lookahead: f64) -> Point {
    let mut best_segment = 0;
    let mut best_t = 0.0;
    let mut best_distance = f64::INFINITY;
    for (index, pair) in path.windows(2).enumerate() {
        let dx = pair[1].x - pair[0].x;
        let dy = pair[1].y - pair[0].y;
        let length_squared = dx * dx + dy * dy;
        let t = if length_squared > 0.0 {
            (((pose.x - pair[0].x) * dx + (pose.y - pair[0].y) * dy) / length_squared)
                .clamp(0.0, 1.0)
        } else {
            0.0
        };
        let x = pair[0].x + t * dx;
        let y = pair[0].y + t * dy;
        let distance = (pose.x - x).hypot(pose.y - y);
        if distance < best_distance {
            best_distance = distance;
            best_segment = index;
            best_t = t;
        }
    }
    let mut current = Point {
        x: path[best_segment].x + best_t * (path[best_segment + 1].x - path[best_segment].x),
        y: path[best_segment].y + best_t * (path[best_segment + 1].y - path[best_segment].y),
    };
    let mut remaining = lookahead;
    for next in &path[best_segment + 1..] {
        let length = (next.x - current.x).hypot(next.y - current.y);
        if length >= remaining && length > 0.0 {
            let t = remaining / length;
            return Point {
                x: current.x + t * (next.x - current.x),
                y: current.y + t * (next.y - current.y),
            };
        }
        remaining -= length;
        current = *next;
    }
    *path.last().unwrap()
}

fn command(route: &Route, pose: Pose, scenario: &Scenario) -> [f64; 2] {
    let point = target(&route.points, pose, scenario.navigation.lookahead_distance);
    let dx = point.x - pose.x;
    let dy = point.y - pose.y;
    let distance_squared = dx * dx + dy * dy;
    if distance_squared < 1e-8 {
        return [0.0, 0.0];
    }
    let local_x = pose.theta.cos() * dx + pose.theta.sin() * dy;
    let local_y = -pose.theta.sin() * dx + pose.theta.cos() * dy;
    if local_x <= 0.02 {
        let error = normalize_angle(dy.atan2(dx) - pose.theta);
        return [
            0.0,
            (2.0 * error).clamp(
                -scenario.navigation.angular_speed,
                scenario.navigation.angular_speed,
            ),
        ];
    }
    let curvature = 2.0 * local_y / distance_squared;
    let mut linear = scenario.navigation.linear_speed;
    if curvature.abs() > 1e-9 {
        linear = linear.min(scenario.navigation.angular_speed / curvature.abs());
    }
    [
        linear,
        (curvature * linear).clamp(
            -scenario.navigation.angular_speed,
            scenario.navigation.angular_speed,
        ),
    ]
}

fn send_status(node: &mut DoraNode, code: u8, reason: &str) -> Result<()> {
    eprintln!("controller: status={code} reason={reason}");
    let mut params = MetadataParameters::default();
    params.insert("reason".into(), Parameter::String(reason.into()));
    node.send_output("status".into(), params, vec![code])?;
    Ok(())
}

fn send_zero(node: &mut DoraNode) -> Result<()> {
    node.send_output("cmd_vel".into(), Default::default(), vec![0.0_f64, 0.0])?;
    Ok(())
}

fn main() -> Result<()> {
    let scenario = Scenario::load_from_env()?;
    let (mut node, mut events) = DoraNode::init_from_env()?;
    let started = Instant::now();
    let mut pose: Option<(Pose, Instant)> = None;
    let mut route: Option<Route> = None;
    let mut outcome: Option<&'static str> = None;
    let mut running_reported = false;
    while let Some(event) = events.recv() {
        match event {
            Event::Input { id, data, .. } if id.as_str() == "pose" => {
                let values = into_vec::<f64>(&data)?;
                pose = Some((pose_from_values(&values)?, Instant::now()));
            }
            Event::Input { id, data, metadata } if id.as_str() == "path" => {
                let values = into_vec::<f64>(&data)?;
                let incoming = parse_route(&values, &metadata)?;
                if route
                    .as_ref()
                    .is_none_or(|current| current.id != incoming.id)
                {
                    route = Some(incoming);
                }
            }
            Event::Input { id, data, .. } if id.as_str() == "collision" => {
                let values = into_vec::<u8>(&data)?;
                ensure!(values.as_slice() == [1], "invalid collision event");
                if outcome.is_none() {
                    outcome = Some("collision");
                    send_status(&mut node, 3, "collision")?;
                }
                send_zero(&mut node)?;
            }
            Event::Input { id, .. } if id.as_str() == "tick" => {
                if outcome.is_some() {
                    send_zero(&mut node)?;
                    continue;
                }
                if route.is_none() && started.elapsed() > Duration::from_secs(2) {
                    outcome = Some("path_timeout");
                    send_status(&mut node, 3, "path_timeout")?;
                    send_zero(&mut node)?;
                    continue;
                }
                let Some((current, received)) = pose else {
                    send_zero(&mut node)?;
                    continue;
                };
                let Some(route) = &route else {
                    send_zero(&mut node)?;
                    continue;
                };
                if received.elapsed() > Duration::from_millis(200) {
                    outcome = Some("pose_timeout");
                    send_status(&mut node, 3, "pose_timeout")?;
                    send_zero(&mut node)?;
                    continue;
                }
                if (current.x - route.goal.x).hypot(current.y - route.goal.y) <= route.tolerance {
                    outcome = Some("goal_reached");
                    send_status(&mut node, 2, "goal_reached")?;
                    send_zero(&mut node)?;
                    continue;
                }
                if !running_reported {
                    send_status(&mut node, 1, "running")?;
                    running_reported = true;
                }
                let velocities = command(route, current, &scenario);
                finite_values(&velocities, 2)?;
                node.send_output("cmd_vel".into(), Default::default(), velocities.to_vec())?;
            }
            Event::InputClosed { id } if matches!(id.as_str(), "pose" | "path" | "collision") => {
                if outcome.is_none() {
                    send_status(&mut node, 3, "input_closed")?;
                }
                send_zero(&mut node)?;
                break;
            }
            Event::Stop(_) => {
                send_zero(&mut node)?;
                break;
            }
            _ => {}
        }
    }
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn controller_heads_toward_path() {
        let scenario = Scenario::load(concat!(
            env!("CARGO_MANIFEST_DIR"),
            "/configs/autonomous.toml"
        ))
        .unwrap();
        let route = Route {
            points: vec![Point { x: 0.0, y: 0.0 }, Point { x: 1.0, y: 0.0 }],
            goal: Point { x: 1.0, y: 0.0 },
            tolerance: 0.2,
            id: "test".into(),
        };
        let straight = command(
            &route,
            Pose {
                x: 0.0,
                y: 0.0,
                theta: 0.0,
            },
            &scenario,
        );
        assert!(straight[0] > 0.0 && straight[1].abs() < 1e-12);
        let left = command(
            &route,
            Pose {
                x: 0.0,
                y: -0.2,
                theta: 0.0,
            },
            &scenario,
        );
        assert!(left[1] > 0.0);
    }
}
