#![cfg(unix)]

use std::fs;
use std::path::Path;
use std::process::{Command, Stdio};
use std::thread;
use std::time::{Duration, Instant, SystemTime, UNIX_EPOCH};

use dora_mujoco_navigation::navigation::Scenario;
use dora_node_api::{DoraNode, Event, into_vec};
use eyre::{Result, ensure};
use serde::{Deserialize, Serialize};

#[derive(Serialize, Deserialize)]
struct Observation {
    truth: Vec<[f64; 3]>,
    estimate: Option<[f64; 3]>,
    statuses: Vec<u8>,
    collision: bool,
}

#[test]
#[ignore = "requires dora CLI and local MuJoCo; run scripts/test-all.sh"]
fn reaches_goal_without_collision() -> Result<()> {
    let repo = Path::new(env!("CARGO_MANIFEST_DIR"));
    let unique = SystemTime::now().duration_since(UNIX_EPOCH)?.as_nanos();
    let root = std::env::temp_dir().join(format!("autonomous-e2e-{}-{unique}", std::process::id()));
    fs::create_dir(&root)?;
    let result_path = root.join("observation.json");
    let plot_path = root.join("trajectory.png");
    let graph_path = root.join("dataflow.yml");
    let mut graph: serde_yaml::Value =
        serde_yaml::from_str(&fs::read_to_string(repo.join("dataflow.yml"))?)?;
    let nodes = graph["nodes"]
        .as_sequence_mut()
        .ok_or_else(|| eyre::eyre!("missing nodes"))?;
    for node in nodes.iter_mut() {
        let path = match node["id"].as_str() {
            Some("simulator") => env!("CARGO_BIN_EXE_simulator"),
            Some("odometry") => env!("CARGO_BIN_EXE_odometry"),
            Some("lidar") => env!("CARGO_BIN_EXE_lidar"),
            Some("localization") => env!("CARGO_BIN_EXE_localization"),
            Some("planner") => env!("CARGO_BIN_EXE_planner"),
            Some("controller") => env!("CARGO_BIN_EXE_controller"),
            Some("trajectory_plot") => env!("CARGO_BIN_EXE_trajectory_plot"),
            other => eyre::bail!("unexpected node: {other:?}"),
        };
        node["path"] = serde_yaml::Value::String(path.to_owned());
    }
    nodes.push(serde_yaml::to_value(serde_json::json!({
        "id": "probe", "path": std::env::current_exe()?,
        "args": "--ignored --exact probe_node --nocapture",
        "inputs": {
            "truth": {"source": "simulator/robot_pose", "queue_size": 1, "queue_policy": "drop_oldest"},
            "estimate": {"source": "localization/pose", "queue_size": 1, "queue_policy": "drop_oldest"},
            "status": "controller/status",
            "collision": "simulator/collision"
        }
    }))?);
    fs::write(&graph_path, serde_yaml::to_string(&graph)?)?;

    let lib_dir = std::env::var_os("MUJOCO_DYNAMIC_LINK_DIR")
        .map(std::path::PathBuf::from)
        .unwrap_or_else(|| repo.join("target/mujoco-3.12.0/lib"));
    let library_path = format!(
        "{}:{}",
        lib_dir.display(),
        std::env::var("LD_LIBRARY_PATH").unwrap_or_default()
    );
    let stdout_path = root.join("dora.stdout.log");
    let stderr_path = root.join("dora.stderr.log");
    let mut run = Command::new("dora")
        .args(["run", "--stop-after", "35s"])
        .arg(&graph_path)
        .current_dir(repo)
        .env("MUJOCO_HEADLESS", "1")
        .env("AUTO_DRIVE_SCENARIO", repo.join("configs/autonomous.toml"))
        .env("MUJOCO_DYNAMIC_LINK_DIR", &lib_dir)
        .env("LD_LIBRARY_PATH", library_path)
        .env("AUTONOMOUS_E2E_RESULTS", &result_path)
        .env("TRAJECTORY_PLOT_PATH", &plot_path)
        .stdout(Stdio::from(fs::File::create(&stdout_path)?))
        .stderr(Stdio::from(fs::File::create(&stderr_path)?))
        .spawn()?;
    let started = Instant::now();
    let status = loop {
        if let Some(status) = run.try_wait()? {
            break status;
        }
        if started.elapsed() > Duration::from_secs(43) {
            run.kill()?;
            run.wait()?;
            eyre::bail!("dora run timed out; logs at {}", root.display());
        }
        thread::sleep(Duration::from_millis(50));
    };
    ensure!(
        status.success(),
        "dora failed; logs at {}:\n{}\n{}",
        root.display(),
        fs::read_to_string(&stdout_path)?,
        fs::read_to_string(&stderr_path)?
    );
    let observation: Observation = serde_json::from_slice(&fs::read(&result_path)?)?;
    ensure!(
        observation.statuses.contains(&2),
        "goal_reached was not published; logs at {}",
        root.display()
    );
    ensure!(
        !observation.statuses.contains(&3),
        "controller reported failure; logs at {}",
        root.display()
    );
    ensure!(
        !observation.collision,
        "collision was reported; logs at {}",
        root.display()
    );
    ensure!(observation.truth.len() > 100, "too few pose samples");
    let last = observation.truth.last().unwrap();
    let scenario = Scenario::load(repo.join("configs/autonomous.toml"))?;
    let goal_distance = (last[0] - scenario.robot.goal.x).hypot(last[1] - scenario.robot.goal.y);
    ensure!(
        goal_distance <= 0.23,
        "true pose missed goal by {goal_distance:.3} m: {last:?}"
    );
    let earlier = observation.truth[observation.truth.len() - 80];
    ensure!(
        (last[0] - earlier[0]).hypot(last[1] - earlier[1]) < 0.03,
        "robot moved after stopping: {earlier:?} -> {last:?}"
    );
    let estimate = observation
        .estimate
        .ok_or_else(|| eyre::eyre!("missing localization estimate"))?;
    ensure!(
        (last[0] - estimate[0]).hypot(last[1] - estimate[1]) < 0.25,
        "localization drifted: true={last:?}, estimated={estimate:?}"
    );
    let png = fs::read(&plot_path)?;
    ensure!(
        png.starts_with(b"\x89PNG\r\n\x1a\n") && png.len() > 3000,
        "trajectory PNG was not generated correctly; logs at {}",
        root.display()
    );
    fs::remove_dir_all(root)?;
    Ok(())
}

#[test]
#[ignore = "probe process launched by reaches_goal_without_collision"]
fn probe_node() -> Result<()> {
    let Some(path) = std::env::var_os("AUTONOMOUS_E2E_RESULTS") else {
        return Ok(());
    };
    let (_node, mut events) = DoraNode::init_from_env()?;
    let mut observation = Observation {
        truth: Vec::new(),
        estimate: None,
        statuses: Vec::new(),
        collision: false,
    };
    while let Some(event) = events.recv() {
        match event {
            Event::Input { id, data, .. } if id.as_str() == "truth" => {
                let values = into_vec::<f64>(&data)?;
                if let [x, y, yaw] = values.as_slice() {
                    observation.truth.push([*x, *y, *yaw]);
                }
            }
            Event::Input { id, data, .. } if id.as_str() == "estimate" => {
                let values = into_vec::<f64>(&data)?;
                if let [x, y, yaw] = values.as_slice() {
                    observation.estimate = Some([*x, *y, *yaw]);
                }
            }
            Event::Input { id, data, .. } if id.as_str() == "status" => {
                observation.statuses.extend(into_vec::<u8>(&data)?);
            }
            Event::Input { id, .. } if id.as_str() == "collision" => observation.collision = true,
            Event::Stop(_) => break,
            _ => {}
        }
    }
    fs::write(path, serde_json::to_vec(&observation)?)?;
    Ok(())
}
