pub const WHEEL_RADIUS: f64 = 0.06;
pub const TRACK_WIDTH: f64 = 0.24;
pub const MAX_WHEEL_SPEED: f64 = 12.0;

pub fn wheel_speeds(linear: f64, angular: f64) -> [f64; 2] {
    let half_track = TRACK_WIDTH / 2.0;
    [
        ((linear - angular * half_track) / WHEEL_RADIUS).clamp(-MAX_WHEEL_SPEED, MAX_WHEEL_SPEED),
        ((linear + angular * half_track) / WHEEL_RADIUS).clamp(-MAX_WHEEL_SPEED, MAX_WHEEL_SPEED),
    ]
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn differential_drive_directions_and_limits() {
        assert_eq!(wheel_speeds(0.3, 0.0), [5.0, 5.0]);
        assert_eq!(wheel_speeds(0.0, 1.0), [-2.0, 2.0]);
        assert_eq!(wheel_speeds(10.0, 0.0), [12.0, 12.0]);
    }
}
