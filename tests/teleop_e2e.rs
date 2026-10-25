#![cfg(unix)]

use std::fs;
use std::io::Write;
use std::os::fd::{AsRawFd, FromRawFd};
use std::os::unix::fs::PermissionsExt;
use std::path::Path;
use std::process::{Child, Command, Stdio};
use std::thread;
use std::time::{Duration, Instant, SystemTime, UNIX_EPOCH};

use dora_node_api::{DoraNode, Event, into_vec};
use eyre::{Result, ensure};

struct ChildGuard(Child);

impl Drop for ChildGuard {
    fn drop(&mut self) {
        let _ = self.0.kill();
        let _ = self.0.wait();
    }
}

#[test]
#[ignore = "requires dora CLI and a local MuJoCo library; run scripts/test-all.sh"]
fn keyboard_to_robot_pose() -> Result<()> {
    let repo = Path::new(env!("CARGO_MANIFEST_DIR"));
    let unique = SystemTime::now().duration_since(UNIX_EPOCH)?.as_nanos();
    let root = std::env::temp_dir().join(format!("teleop-e2e-{}-{unique}", std::process::id()));
    fs::create_dir(&root)?;
    let results = root.join("poses.json");
    let graph_path = root.join("dataflow.yml");
    let terminal_path = root.join("terminal.sh");
    fs::write(
        &terminal_path,
        "#!/bin/sh\nset -eu\n[ \"$1\" = \"-e\" ]\nexport DORA_E2E_HELPER=\"$2\"\nexport DORA_E2E_SOCKET=\"$3\"\nexec \"$DORA_E2E_TEST_BINARY\" --ignored --exact terminal_node --nocapture\n",
    )?;
    fs::set_permissions(&terminal_path, fs::Permissions::from_mode(0o700))?;
    let test_binary = std::env::current_exe()?;
    let mut graph: serde_yaml::Value =
        serde_yaml::from_str(&fs::read_to_string(repo.join("dataflow.teleop.yml"))?)?;
    let nodes = graph["nodes"]
        .as_sequence_mut()
        .ok_or_else(|| eyre::eyre!("dataflow.teleop.yml has no nodes"))?;
    for node in nodes.iter_mut() {
        let path = match node["id"].as_str() {
            Some("keyboard") => env!("CARGO_BIN_EXE_keyboard"),
            Some("teleop") => env!("CARGO_BIN_EXE_teleop"),
            Some("simulator") => env!("CARGO_BIN_EXE_simulator"),
            other => eyre::bail!("unexpected node in teleop dataflow: {other:?}"),
        };
        node["path"] = serde_yaml::Value::String(path.to_owned());
    }
    nodes.push(serde_yaml::to_value(serde_json::json!({
        "id": "probe", "path": &test_binary,
        "args": "--ignored --exact pose_probe_node --nocapture",
        "inputs": {"pose": "simulator/robot_pose"}
    }))?);
    fs::write(&graph_path, serde_yaml::to_string(&graph)?)?;

    let lib_dir = std::env::var_os("MUJOCO_DYNAMIC_LINK_DIR")
        .map(std::path::PathBuf::from)
        .unwrap_or_else(|| repo.join("target/mujoco-3.12.0/lib"));
    let existing_path = std::env::var_os("LD_LIBRARY_PATH").unwrap_or_default();
    let library_path = format!("{}:{}", lib_dir.display(), existing_path.to_string_lossy());
    let stdout_path = root.join("dora.stdout.log");
    let stderr_path = root.join("dora.stderr.log");
    let mut run = Command::new("dora")
        .args(["run", "--stop-after", "4s"])
        .arg(&graph_path)
        .current_dir(repo)
        .env("MUJOCO_HEADLESS", "1")
        .env("MUJOCO_DYNAMIC_LINK_DIR", &lib_dir)
        .env("LD_LIBRARY_PATH", library_path)
        .env("TELEOP_TERMINAL", &terminal_path)
        .env("DORA_E2E_TEST_BINARY", &test_binary)
        .env("DORA_E2E_RESULTS", &results)
        .stdout(Stdio::from(fs::File::create(&stdout_path)?))
        .stderr(Stdio::from(fs::File::create(&stderr_path)?))
        .spawn()?;
    let started = Instant::now();
    let status = loop {
        if let Some(status) = run.try_wait()? {
            break status;
        }
        if started.elapsed() > Duration::from_secs(10) {
            run.kill()?;
            run.wait()?;
            eyre::bail!("dora run exceeded 10 seconds; logs at {}", root.display());
        }
        thread::sleep(Duration::from_millis(50));
    };
    ensure!(
        status.success(),
        "dora run failed; logs at {}:\n{}\n{}",
        root.display(),
        fs::read_to_string(&stdout_path)?,
        fs::read_to_string(&stderr_path)?
    );

    let samples: Vec<[f64; 4]> = serde_json::from_slice(&fs::read(&results)?)?;
    ensure!(
        samples.len() > 30,
        "too few pose samples: {}",
        samples.len()
    );
    let first_x = samples[0][1];
    let max_x = samples
        .iter()
        .map(|sample| sample[1])
        .fold(first_x, f64::max);
    ensure!(
        max_x - first_x > 0.05,
        "forward key did not move robot: {samples:?}"
    );
    let last = samples.last().unwrap();
    let late = samples
        .iter()
        .find(|sample| sample[0] >= last[0] - 0.5)
        .unwrap();
    ensure!(
        (last[1] - late[1]).abs() < 0.03,
        "robot kept moving after stop: late={late:?}, last={last:?}"
    );
    ensure!(
        last[2].abs() < 0.05 && last[3].abs() < 0.1,
        "unexpected sideways motion: {last:?}"
    );
    fs::remove_dir_all(root)?;
    Ok(())
}

#[test]
#[ignore = "helper process launched by the E2E terminal wrapper"]
fn terminal_node() -> Result<()> {
    let Some(helper) = std::env::var_os("DORA_E2E_HELPER") else {
        return Ok(());
    };
    let socket = std::env::var_os("DORA_E2E_SOCKET")
        .ok_or_else(|| eyre::eyre!("DORA_E2E_SOCKET is missing"))?;
    let (mut master_fd, mut slave_fd) = (-1, -1);
    if unsafe {
        libc::openpty(
            &mut master_fd,
            &mut slave_fd,
            std::ptr::null_mut(),
            std::ptr::null_mut(),
            std::ptr::null_mut(),
        )
    } != 0
    {
        return Err(std::io::Error::last_os_error().into());
    }
    let mut master = unsafe { fs::File::from_raw_fd(master_fd) };
    let slave = unsafe { fs::File::from_raw_fd(slave_fd) };
    let mut child = ChildGuard(
        Command::new(helper)
            .arg(socket)
            .stdin(Stdio::from(slave.try_clone()?))
            .stdout(Stdio::null())
            .stderr(Stdio::null())
            .spawn()?,
    );

    let deadline = Instant::now() + Duration::from_secs(3);
    loop {
        let mut settings = unsafe { std::mem::zeroed() };
        if unsafe { libc::tcgetattr(slave.as_raw_fd(), &mut settings) } != 0 {
            return Err(std::io::Error::last_os_error().into());
        }
        if settings.c_lflag & libc::ICANON == 0 {
            break;
        }
        ensure!(
            Instant::now() < deadline,
            "keyboard_tty did not enter raw mode"
        );
        thread::sleep(Duration::from_millis(10));
    }

    thread::sleep(Duration::from_millis(300));
    master.write_all(b"\x1b[A")?;
    thread::sleep(Duration::from_millis(1500));
    master.write_all(b" ")?;
    thread::sleep(Duration::from_millis(200));
    master.write_all(b"q")?;

    let deadline = Instant::now() + Duration::from_secs(3);
    loop {
        if let Some(status) = child.0.try_wait()? {
            ensure!(status.success(), "keyboard_tty exited with {status}");
            break;
        }
        ensure!(Instant::now() < deadline, "keyboard_tty did not exit on Q");
        thread::sleep(Duration::from_millis(10));
    }
    Ok(())
}

#[test]
#[ignore = "helper process launched by the E2E dataflow"]
fn pose_probe_node() -> Result<()> {
    if std::env::var_os("DORA_NODE_CONFIG").is_none() {
        return Ok(());
    }
    let (_node, mut events) = DoraNode::init_from_env()?;
    let started = Instant::now();
    let mut samples = Vec::<[f64; 4]>::new();
    while let Some(event) = events.recv() {
        match event {
            Event::Input { id, data, .. } if id.as_str() == "pose" => {
                let pose = into_vec::<f64>(&data)?;
                ensure!(pose.len() == 3 && pose.iter().all(|v| v.is_finite()));
                samples.push([started.elapsed().as_secs_f64(), pose[0], pose[1], pose[2]]);
            }
            Event::Stop(_) => break,
            _ => {}
        }
    }
    fs::write(
        std::env::var("DORA_E2E_RESULTS")?,
        serde_json::to_vec(&samples)?,
    )?;
    Ok(())
}
