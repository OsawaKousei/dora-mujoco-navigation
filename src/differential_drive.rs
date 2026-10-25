pub const WHEEL_RADIUS: f64 = 0.06;
pub const TRACK_WIDTH: f64 = 0.24;
pub const MAX_WHEEL_SPEED: f64 = 12.0;

#[derive(Debug, PartialEq)]
pub struct WheelSpeeds {
    pub left: f64,
    pub right: f64,
}

pub fn wheel_speeds(linear: f64, angular: f64) -> WheelSpeeds {
    let half_track = TRACK_WIDTH / 2.0;
    WheelSpeeds {
        left: ((linear - angular * half_track) / WHEEL_RADIUS)
            .clamp(-MAX_WHEEL_SPEED, MAX_WHEEL_SPEED),
        right: ((linear + angular * half_track) / WHEEL_RADIUS)
            .clamp(-MAX_WHEEL_SPEED, MAX_WHEEL_SPEED),
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn differential_drive_directions_and_limits() {
        assert_eq!(
            wheel_speeds(0.3, 0.0),
            WheelSpeeds {
                left: 5.0,
                right: 5.0
            }
        );
        assert_eq!(
            wheel_speeds(0.0, 1.0),
            WheelSpeeds {
                left: -2.0,
                right: 2.0
            }
        );
        assert_eq!(
            wheel_speeds(10.0, 0.0),
            WheelSpeeds {
                left: 12.0,
                right: 12.0
            }
        );
    }

    #[test]
    fn reverse_turn_combines_wheels_and_clamps_each_side() {
        assert_eq!(
            wheel_speeds(-0.3, 1.0),
            WheelSpeeds {
                left: -7.0,
                right: -3.0
            }
        );
        assert_eq!(
            wheel_speeds(0.6, 3.0),
            WheelSpeeds {
                left: 4.0,
                right: 12.0
            }
        );
    }
}
