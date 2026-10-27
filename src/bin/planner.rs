use dora_mujoco_navigation::navigation::{Grid, Scenario};
use dora_node_api::{DoraNode, Event, MetadataParameters, Parameter};
use eyre::Result;

fn main() -> Result<()> {
    let scenario = Scenario::load_from_env()?;
    let grid = Grid::new(&scenario);
    let path = grid.plan(scenario.robot.start.point(), scenario.robot.goal.point())?;
    eprintln!("planner: planned {} waypoints", path.len());
    let coordinates: Vec<f64> = path.iter().flat_map(|point| [point.x, point.y]).collect();
    let (mut node, mut events) = DoraNode::init_from_env()?;
    let mut publish = true;
    while publish {
        let mut params = MetadataParameters::default();
        params.insert(
            "plan_id".into(),
            Parameter::String("autonomous_static_v1".into()),
        );
        params.insert("goal_x".into(), Parameter::Float(scenario.robot.goal.x));
        params.insert("goal_y".into(), Parameter::Float(scenario.robot.goal.y));
        params.insert(
            "goal_tolerance".into(),
            Parameter::Float(scenario.navigation.goal_tolerance),
        );
        node.send_output("path".into(), params, coordinates.clone())?;
        loop {
            match events.recv() {
                Some(Event::Input { id, .. }) if id.as_str() == "tick" => break,
                Some(Event::Stop(_)) | None => {
                    publish = false;
                    break;
                }
                _ => {}
            }
        }
    }
    Ok(())
}
