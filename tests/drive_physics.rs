use dora_mujoco_navigation::wheel_speeds;
use mujoco_rs::prelude::{MjModel, MjtObj};

fn run_command(linear: f64, angular: f64) -> (f64, f64, f64) {
    let model = MjModel::from_xml(concat!(env!("CARGO_MANIFEST_DIR"), "/assets/robot.xml"))
        .expect("robot model should load");
    let robot = model.name_to_id(MjtObj::mjOBJ_BODY, "robot").unwrap();
    let left = model
        .name_to_id(MjtObj::mjOBJ_ACTUATOR, "left_motor")
        .unwrap();
    let right = model
        .name_to_id(MjtObj::mjOBJ_ACTUATOR, "right_motor")
        .unwrap();
    let mut data = model.make_data();
    let speeds = wheel_speeds(linear, angular);
    data.ctrl_mut()[left] = speeds[0];
    data.ctrl_mut()[right] = speeds[1];
    for _ in 0..400 {
        data.step();
    }
    let [x, y, _] = data.xpos()[robot];
    let [w, qx, qy, qz] = data.xquat()[robot];
    let yaw = (2.0 * (w * qz + qx * qy)).atan2(1.0 - 2.0 * (qy * qy + qz * qz));
    (x, y, yaw)
}

#[test]
fn positive_linear_command_moves_robot_forward() {
    let (x, _, _) = run_command(0.24, 0.0);
    assert!(x > 0.1, "robot did not move forward: x={x}");
}

#[test]
fn positive_angular_command_turns_robot_left() {
    let (_, _, yaw) = run_command(0.0, 1.0);
    assert!(yaw > 0.3, "robot did not turn left: yaw={yaw}");
}
