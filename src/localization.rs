use crate::navigation::{Pose, Scenario, normalize_angle};

#[derive(Clone, Copy, Debug)]
pub struct EkfState {
    pub pose: Pose,
    pub covariance: [[f64; 3]; 3],
}

impl EkfState {
    pub fn new(pose: Pose) -> Self {
        Self {
            pose,
            covariance: [[0.01, 0.0, 0.0], [0.0, 0.01, 0.0], [0.0, 0.0, 0.02]],
        }
    }

    pub fn predict(&mut self, delta: [f64; 3]) {
        let [forward, lateral, turn] = delta;
        let (sin, cos) = self.pose.theta.sin_cos();
        let fx = -forward * sin - lateral * cos;
        let fy = forward * cos - lateral * sin;
        self.pose.x += forward * cos - lateral * sin;
        self.pose.y += forward * sin + lateral * cos;
        self.pose.theta = normalize_angle(self.pose.theta + turn);
        let f = [[1.0, 0.0, fx], [0.0, 1.0, fy], [0.0, 0.0, 1.0]];
        let mut next = [[0.0; 3]; 3];
        for i in 0..3 {
            for j in 0..3 {
                for k in 0..3 {
                    for l in 0..3 {
                        next[i][j] += f[i][k] * self.covariance[k][l] * f[j][l];
                    }
                }
            }
        }
        let travel = forward.hypot(lateral);
        next[0][0] += 0.00001 + 0.005 * travel;
        next[1][1] += 0.00001 + 0.005 * travel;
        next[2][2] += 0.00001 + 0.005 * turn.abs();
        self.covariance = next;
    }

    pub fn update_scan(&mut self, ranges: &[f64], scenario: &Scenario) -> usize {
        let lidar = &scenario.lidar;
        let increment = scenario.lidar_angle_increment();
        let stride = (lidar.ray_count / 24).max(1);
        let mut accepted = 0;
        for index in (0..ranges.len()).step_by(stride) {
            let observed = ranges[index];
            if !observed.is_finite() || observed <= 0.0 || observed >= lidar.max_range - 0.05 {
                continue;
            }
            let angle = lidar.min_angle + index as f64 * increment;
            let expected = scenario.raycast(self.pose, angle);
            let residual = observed - expected;
            if expected >= lidar.max_range - 0.05 || residual.abs() > 0.35 {
                continue;
            }
            let epsilon = 0.005;
            let mut h = [0.0; 3];
            for (axis, derivative) in h.iter_mut().enumerate() {
                let mut shifted = self.pose;
                match axis {
                    0 => shifted.x += epsilon,
                    1 => shifted.y += epsilon,
                    _ => shifted.theta += epsilon,
                }
                *derivative = (scenario.raycast(shifted, angle) - expected) / epsilon;
            }
            if h.iter().any(|v| !v.is_finite() || v.abs() > 20.0) {
                continue;
            }
            let p = self.covariance;
            let mut ph = [0.0; 3];
            for i in 0..3 {
                for j in 0..3 {
                    ph[i] += p[i][j] * h[j];
                }
            }
            let measurement_variance = (lidar.noise_stddev * lidar.noise_stddev).max(0.0004);
            let innovation_variance =
                measurement_variance + (0..3).map(|i| h[i] * ph[i]).sum::<f64>();
            if !innovation_variance.is_finite() || innovation_variance <= 0.0 {
                continue;
            }
            if residual * residual > 9.0 * innovation_variance {
                continue;
            }
            let gain = ph.map(|value| value / innovation_variance);
            self.pose.x += gain[0] * residual;
            self.pose.y += gain[1] * residual;
            self.pose.theta = normalize_angle(self.pose.theta + gain[2] * residual);
            // Joseph form preserves covariance symmetry and positive definiteness.
            let mut a = [[0.0; 3]; 3];
            for i in 0..3 {
                for j in 0..3 {
                    a[i][j] = f64::from(i == j) - gain[i] * h[j];
                }
            }
            let mut updated = [[0.0; 3]; 3];
            for i in 0..3 {
                for j in 0..3 {
                    for k in 0..3 {
                        for l in 0..3 {
                            updated[i][j] += a[i][k] * p[k][l] * a[j][l];
                        }
                    }
                    updated[i][j] += gain[i] * measurement_variance * gain[j];
                }
            }
            self.covariance = updated;
            accepted += 1;
        }
        accepted
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn prediction_and_scan_reduce_position_error() {
        let scenario = Scenario::load(concat!(
            env!("CARGO_MANIFEST_DIR"),
            "/configs/autonomous.toml"
        ))
        .unwrap();
        let true_pose = scenario.robot.start;
        let ranges: Vec<_> = (0..scenario.lidar.ray_count)
            .map(|i| {
                scenario.raycast(
                    true_pose,
                    scenario.lidar.min_angle + i as f64 * scenario.lidar_angle_increment(),
                )
            })
            .collect();
        let mut state = EkfState::new(Pose {
            x: true_pose.x + 0.05,
            y: true_pose.y - 0.04,
            theta: 0.02,
        });
        let before = (state.pose.x - true_pose.x).hypot(state.pose.y - true_pose.y);
        assert!(state.update_scan(&ranges, &scenario) >= 3);
        let after = (state.pose.x - true_pose.x).hypot(state.pose.y - true_pose.y);
        assert!(
            after < before,
            "scan did not improve position: {before} -> {after}"
        );
    }
}
