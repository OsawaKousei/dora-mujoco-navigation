use std::cmp::Ordering;
use std::collections::BinaryHeap;
use std::path::Path;

use eyre::{Result, bail, ensure};
use serde::Deserialize;

#[derive(Clone, Copy, Debug, Deserialize, PartialEq)]
pub struct Point {
    pub x: f64,
    pub y: f64,
}

#[derive(Clone, Copy, Debug, Deserialize, PartialEq)]
pub struct Pose {
    pub x: f64,
    pub y: f64,
    pub theta: f64,
}

impl Pose {
    pub fn point(self) -> Point {
        Point {
            x: self.x,
            y: self.y,
        }
    }
}

#[derive(Clone, Copy, Debug, Deserialize)]
pub struct Rect {
    pub x: f64,
    pub y: f64,
    pub half_x: f64,
    pub half_y: f64,
}

impl Rect {
    fn clearance(self, point: Point) -> f64 {
        let dx = (point.x - self.x).abs() - self.half_x;
        let dy = (point.y - self.y).abs() - self.half_y;
        dx.max(0.0).hypot(dy.max(0.0))
    }

    fn ray_distance(self, origin: Point, dx: f64, dy: f64) -> Option<f64> {
        let mut near = f64::NEG_INFINITY;
        let mut far = f64::INFINITY;
        for (coordinate, direction, min, max) in [
            (origin.x, dx, self.x - self.half_x, self.x + self.half_x),
            (origin.y, dy, self.y - self.half_y, self.y + self.half_y),
        ] {
            if direction.abs() < 1e-12 {
                if coordinate < min || coordinate > max {
                    return None;
                }
            } else {
                let a = (min - coordinate) / direction;
                let b = (max - coordinate) / direction;
                near = near.max(a.min(b));
                far = far.min(a.max(b));
            }
        }
        if far >= near && far >= 0.0 {
            Some(if near >= 0.0 { near } else { far })
        } else {
            None
        }
    }
}

#[derive(Clone, Debug, Deserialize)]
pub struct MapConfig {
    pub width: f64,
    pub height: f64,
    pub resolution: f64,
}

#[derive(Clone, Debug, Deserialize)]
pub struct RobotConfig {
    pub start: Pose,
    pub goal: Pose,
}

#[derive(Clone, Debug, Deserialize)]
pub struct NavigationConfig {
    pub robot_radius: f64,
    pub goal_tolerance: f64,
    pub lookahead_distance: f64,
    pub linear_speed: f64,
    pub angular_speed: f64,
}

#[derive(Clone, Debug, Deserialize)]
pub struct LidarConfig {
    pub ray_count: usize,
    pub min_angle: f64,
    pub max_angle: f64,
    pub max_range: f64,
    pub noise_stddev: f64,
}

#[derive(Clone, Debug, Deserialize)]
pub struct Scenario {
    pub map: MapConfig,
    pub robot: RobotConfig,
    pub navigation: NavigationConfig,
    pub lidar: LidarConfig,
    pub obstacles: Vec<Rect>,
}

impl Scenario {
    pub fn load_from_env() -> Result<Self> {
        let path = std::env::var("AUTO_DRIVE_SCENARIO").unwrap_or_else(|_| {
            concat!(env!("CARGO_MANIFEST_DIR"), "/configs/autonomous.toml").to_owned()
        });
        Self::load(path)
    }

    pub fn load(path: impl AsRef<Path>) -> Result<Self> {
        let source = std::fs::read_to_string(path)?;
        let scenario: Self = toml::from_str(&source)?;
        scenario.validate()?;
        Ok(scenario)
    }

    pub fn validate(&self) -> Result<()> {
        let m = &self.map;
        ensure!(
            m.width.is_finite()
                && m.height.is_finite()
                && m.resolution.is_finite()
                && m.width > 0.0
                && m.height > 0.0
                && m.resolution > 0.0,
            "invalid map dimensions"
        );
        ensure!(
            m.width / m.resolution <= 2000.0 && m.height / m.resolution <= 2000.0,
            "map is too large"
        );
        ensure!(
            ((m.width / m.resolution).round() - m.width / m.resolution).abs() < 1e-9
                && ((m.height / m.resolution).round() - m.height / m.resolution).abs() < 1e-9,
            "map dimensions must be integer multiples of resolution"
        );
        for pose in [self.robot.start, self.robot.goal] {
            ensure!(
                pose.x.is_finite() && pose.y.is_finite() && pose.theta.is_finite(),
                "nonfinite robot pose"
            );
        }
        let n = &self.navigation;
        ensure!(
            [
                n.robot_radius,
                n.goal_tolerance,
                n.lookahead_distance,
                n.linear_speed,
                n.angular_speed
            ]
            .iter()
            .all(|v| v.is_finite() && *v > 0.0),
            "invalid navigation parameters"
        );
        ensure!(
            n.linear_speed + n.angular_speed * crate::differential_drive::TRACK_WIDTH / 2.0
                <= crate::differential_drive::WHEEL_RADIUS
                    * crate::differential_drive::MAX_WHEEL_SPEED,
            "navigation speed exceeds wheel actuator limits"
        );
        let l = &self.lidar;
        ensure!(
            l.ray_count >= 2
                && l.ray_count <= 4096
                && l.min_angle.is_finite()
                && l.max_angle.is_finite()
                && l.max_angle > l.min_angle
                && l.max_range.is_finite()
                && l.max_range > 0.0
                && l.noise_stddev.is_finite()
                && l.noise_stddev >= 0.0,
            "invalid lidar parameters"
        );
        ensure!(!self.obstacles.is_empty(), "scenario has no obstacles");
        for r in &self.obstacles {
            ensure!(
                [r.x, r.y, r.half_x, r.half_y].iter().all(|v| v.is_finite())
                    && r.half_x > 0.0
                    && r.half_y > 0.0,
                "invalid obstacle"
            );
        }
        let grid = Grid::new(self);
        ensure!(
            grid.is_free(self.robot.start.point()),
            "start is obstructed"
        );
        ensure!(grid.is_free(self.robot.goal.point()), "goal is obstructed");
        Ok(())
    }

    pub fn lidar_angle_increment(&self) -> f64 {
        (self.lidar.max_angle - self.lidar.min_angle) / (self.lidar.ray_count - 1) as f64
    }

    pub fn raycast(&self, pose: Pose, local_angle: f64) -> f64 {
        let angle = pose.theta + local_angle;
        let (dx, dy) = (angle.cos(), angle.sin());
        self.obstacles
            .iter()
            .filter_map(|rect| rect.ray_distance(pose.point(), dx, dy))
            .fold(self.lidar.max_range, f64::min)
            .min(self.lidar.max_range)
    }

    pub fn mujoco_xml(&self) -> Result<String> {
        let mut geometry = String::new();
        for (index, rect) in self.obstacles.iter().enumerate() {
            geometry.push_str(&format!(
                "<geom name=\"nav_wall_{index}\" type=\"box\" pos=\"{} {} 0.25\" size=\"{} {} 0.25\" group=\"1\" rgba=\"0.65 0.25 0.20 1\"/>\n",
                rect.x, rect.y, rect.half_x, rect.half_y
            ));
        }
        let original = include_str!("../assets/robot.xml");
        let old_obstacle = "<geom name=\"obstacle\"\n          type=\"box\"\n          pos=\"1.2 0 0.25\"\n          size=\"0.2 0.3 0.25\"\n          group=\"1\"\n          rgba=\"1 0.3 0.3 1\"/>";
        ensure!(
            original.contains(old_obstacle),
            "robot model obstacle template changed"
        );
        let xml = original.replace(old_obstacle, &geometry);
        let initial = "<body name=\"robot\" pos=\"0 0 0.06\">";
        ensure!(xml.contains(initial), "robot model start template changed");
        Ok(xml.replace(
            initial,
            &format!(
                "<body name=\"robot\" pos=\"{} {} 0.06\" euler=\"0 0 {}\">",
                self.robot.start.x, self.robot.start.y, self.robot.start.theta
            ),
        ))
    }
}

#[derive(Clone)]
pub struct Grid {
    pub width: usize,
    pub height: usize,
    pub resolution: f64,
    radius: f64,
    obstacles: Vec<Rect>,
    free: Vec<bool>,
}

impl Grid {
    pub fn new(scenario: &Scenario) -> Self {
        let width = (scenario.map.width / scenario.map.resolution).round() as usize;
        let height = (scenario.map.height / scenario.map.resolution).round() as usize;
        let resolution = scenario.map.resolution;
        let radius = scenario.navigation.robot_radius;
        let obstacles = scenario.obstacles.clone();
        let mut free = Vec::with_capacity(width * height);
        for y in 0..height {
            for x in 0..width {
                let point = Point {
                    x: (x as f64 + 0.5) * resolution,
                    y: (y as f64 + 0.5) * resolution,
                };
                free.push(Self::clear(point, radius, &obstacles));
            }
        }
        Self {
            width,
            height,
            resolution,
            radius,
            obstacles,
            free,
        }
    }

    fn clear(point: Point, radius: f64, obstacles: &[Rect]) -> bool {
        obstacles.iter().all(|r| r.clearance(point) > radius)
    }

    pub fn is_free(&self, point: Point) -> bool {
        point.x >= 0.0
            && point.y >= 0.0
            && point.x < self.width as f64 * self.resolution
            && point.y < self.height as f64 * self.resolution
            && Self::clear(point, self.radius, &self.obstacles)
    }

    fn cell(&self, point: Point) -> Option<usize> {
        if !self.is_free(point) {
            return None;
        }
        let x = (point.x / self.resolution).floor() as usize;
        let y = (point.y / self.resolution).floor() as usize;
        let index = y * self.width + x;
        self.free[index].then_some(index)
    }

    fn center(&self, index: usize) -> Point {
        Point {
            x: (index % self.width) as f64 * self.resolution + self.resolution / 2.0,
            y: (index / self.width) as f64 * self.resolution + self.resolution / 2.0,
        }
    }

    fn segment_free(&self, a: Point, b: Point) -> bool {
        let distance = (b.x - a.x).hypot(b.y - a.y);
        let steps = (distance / (self.resolution * 0.5)).ceil() as usize;
        (0..=steps).all(|index| {
            let t = index as f64 / steps.max(1) as f64;
            self.is_free(Point {
                x: a.x + t * (b.x - a.x),
                y: a.y + t * (b.y - a.y),
            })
        })
    }

    pub fn plan(&self, start: Point, goal: Point) -> Result<Vec<Point>> {
        let start_index = self
            .cell(start)
            .ok_or_else(|| eyre::eyre!("start outside free map"))?;
        let goal_index = self
            .cell(goal)
            .ok_or_else(|| eyre::eyre!("goal outside free map"))?;
        if start_index == goal_index {
            return Ok(vec![start, goal]);
        }
        let mut cost = vec![f64::INFINITY; self.free.len()];
        let mut previous = vec![None; self.free.len()];
        let mut queue = BinaryHeap::new();
        cost[start_index] = 0.0;
        queue.push(SearchNode {
            index: start_index,
            estimate: 0.0,
        });
        const MOVES: [(isize, isize); 8] = [
            (1, 0),
            (-1, 0),
            (0, 1),
            (0, -1),
            (1, 1),
            (1, -1),
            (-1, 1),
            (-1, -1),
        ];
        while let Some(current) = queue.pop() {
            if current.index == goal_index {
                break;
            }
            let x = current.index % self.width;
            let y = current.index / self.width;
            let heuristic = |index: usize| {
                let a = self.center(index);
                let b = self.center(goal_index);
                (a.x - b.x).hypot(a.y - b.y)
            };
            if current.estimate > cost[current.index] + heuristic(current.index) + 1e-9 {
                continue;
            }
            for (dx, dy) in MOVES {
                let nx = x as isize + dx;
                let ny = y as isize + dy;
                if nx < 0 || ny < 0 || nx >= self.width as isize || ny >= self.height as isize {
                    continue;
                }
                let index = ny as usize * self.width + nx as usize;
                if !self.free[index] {
                    continue;
                }
                if dx != 0
                    && dy != 0
                    && (!self.free[y * self.width + nx as usize]
                        || !self.free[ny as usize * self.width + x])
                {
                    continue;
                }
                let step_cost = if dx != 0 && dy != 0 {
                    std::f64::consts::SQRT_2
                } else {
                    1.0
                };
                let candidate = cost[current.index] + step_cost * self.resolution;
                if candidate < cost[index] {
                    cost[index] = candidate;
                    previous[index] = Some(current.index);
                    queue.push(SearchNode {
                        index,
                        estimate: candidate + heuristic(index),
                    });
                }
            }
        }
        if previous[goal_index].is_none() {
            bail!("no path to goal");
        }
        let mut indices = vec![goal_index];
        while *indices.last().unwrap() != start_index {
            indices.push(
                previous[*indices.last().unwrap()].ok_or_else(|| eyre::eyre!("broken path"))?,
            );
        }
        indices.reverse();
        let mut raw = vec![start];
        raw.extend(indices.into_iter().skip(1).map(|index| self.center(index)));
        raw.push(goal);
        let mut smoothed = vec![start];
        let mut current = 0;
        while current < raw.len() - 1 {
            let next = ((current + 1)..raw.len())
                .rev()
                .find(|&index| self.segment_free(raw[current], raw[index]))
                .ok_or_else(|| eyre::eyre!("path smoothing failed"))?;
            smoothed.push(raw[next]);
            current = next;
        }
        Ok(smoothed)
    }
}

#[derive(Clone, Copy)]
struct SearchNode {
    index: usize,
    estimate: f64,
}

impl PartialEq for SearchNode {
    fn eq(&self, other: &Self) -> bool {
        self.index == other.index && self.estimate == other.estimate
    }
}
impl Eq for SearchNode {}
impl PartialOrd for SearchNode {
    fn partial_cmp(&self, other: &Self) -> Option<Ordering> {
        Some(self.cmp(other))
    }
}
impl Ord for SearchNode {
    fn cmp(&self, other: &Self) -> Ordering {
        other
            .estimate
            .total_cmp(&self.estimate)
            .then_with(|| self.index.cmp(&other.index))
    }
}

pub fn normalize_angle(angle: f64) -> f64 {
    (angle + std::f64::consts::PI).rem_euclid(std::f64::consts::TAU) - std::f64::consts::PI
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn path_avoids_configured_obstacle() {
        let scenario = Scenario::load(concat!(
            env!("CARGO_MANIFEST_DIR"),
            "/configs/autonomous.toml"
        ))
        .unwrap();
        let grid = Grid::new(&scenario);
        let path = grid
            .plan(scenario.robot.start.point(), scenario.robot.goal.point())
            .unwrap();
        assert!(path.len() >= 4, "slalom route has too few turns: {path:?}");
        for pair in path.windows(2) {
            assert!(grid.segment_free(pair[0], pair[1]));
        }
        assert!(path.iter().any(|point| point.y > 1.8));
        assert!(path.iter().any(|point| point.y < 1.2));
        assert!((scenario.raycast(scenario.robot.start, 0.0) - 0.7).abs() < 1e-9);
    }
}
