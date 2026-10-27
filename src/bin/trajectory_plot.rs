use std::path::{Path, PathBuf};

use dora_mujoco_navigation::nav_messages::pose_from_values;
use dora_mujoco_navigation::navigation::{Point, Scenario};
use dora_node_api::{DoraNode, Event, into_vec};
use eyre::{Result, ensure, eyre};
use plotters::prelude::*;

fn path_points(values: &[f64]) -> Result<Vec<Point>> {
    ensure!(
        values.len() >= 4 && values.len().is_multiple_of(2),
        "invalid planned path"
    );
    ensure!(
        values.iter().all(|value| value.is_finite()),
        "nonfinite planned path"
    );
    Ok(values
        .chunks_exact(2)
        .map(|pair| Point {
            x: pair[0],
            y: pair[1],
        })
        .collect())
}

fn plot(
    scenario: &Scenario,
    planned: &[Point],
    actual: &[Point],
    status: Option<u8>,
    output: &Path,
) -> Result<()> {
    if planned.len() < 2 || actual.is_empty() {
        return Ok(());
    }
    if let Some(parent) = output
        .parent()
        .filter(|parent| !parent.as_os_str().is_empty())
    {
        std::fs::create_dir_all(parent)?;
    }
    let temporary = output.with_extension("tmp.png");
    {
        let drawing = BitMapBackend::new(&temporary, (1100, 760)).into_drawing_area();
        drawing
            .fill(&WHITE)
            .map_err(|error| eyre!("plot background: {error:?}"))?;
        let caption = match status {
            Some(2) => "Autonomous navigation: goal reached",
            Some(3) => "Autonomous navigation: stopped with failure",
            _ => "Autonomous navigation: running",
        };
        let mut chart = ChartBuilder::on(&drawing)
            .caption(caption, ("sans-serif", 30))
            .margin(24)
            .x_label_area_size(48)
            .y_label_area_size(48)
            .build_cartesian_2d(0.0..scenario.map.width, 0.0..scenario.map.height)
            .map_err(|error| eyre!("plot axes: {error:?}"))?;
        chart
            .configure_mesh()
            .x_desc("X [m]")
            .y_desc("Y [m]")
            .x_labels(11)
            .y_labels(7)
            .light_line_style(WHITE)
            .bold_line_style(RGBColor(225, 225, 225))
            .draw()
            .map_err(|error| eyre!("plot grid: {error:?}"))?;
        chart
            .draw_series(scenario.obstacles.iter().map(|obstacle| {
                Rectangle::new(
                    [
                        (obstacle.x - obstacle.half_x, obstacle.y - obstacle.half_y),
                        (obstacle.x + obstacle.half_x, obstacle.y + obstacle.half_y),
                    ],
                    RGBColor(115, 120, 130).mix(0.65).filled(),
                )
            }))
            .map_err(|error| eyre!("plot obstacles: {error:?}"))?;
        chart
            .draw_series(LineSeries::new(
                planned.iter().map(|point| (point.x, point.y)),
                BLUE.stroke_width(3),
            ))
            .map_err(|error| eyre!("plot planned path: {error:?}"))?
            .label("Planned path")
            .legend(|(x, y)| PathElement::new(vec![(x, y), (x + 28, y)], BLUE.stroke_width(3)));
        chart
            .draw_series(LineSeries::new(
                actual.iter().map(|point| (point.x, point.y)),
                RED.stroke_width(3),
            ))
            .map_err(|error| eyre!("plot actual trajectory: {error:?}"))?
            .label("Actual trajectory")
            .legend(|(x, y)| PathElement::new(vec![(x, y), (x + 28, y)], RED.stroke_width(3)));
        chart
            .draw_series([
                Circle::new(
                    (scenario.robot.start.x, scenario.robot.start.y),
                    7,
                    GREEN.filled(),
                ),
                Circle::new(
                    (scenario.robot.goal.x, scenario.robot.goal.y),
                    7,
                    MAGENTA.filled(),
                ),
            ])
            .map_err(|error| eyre!("plot endpoints: {error:?}"))?;
        chart
            .configure_series_labels()
            .position(SeriesLabelPosition::UpperRight)
            .background_style(WHITE.mix(0.9))
            .border_style(BLACK)
            .draw()
            .map_err(|error| eyre!("plot legend: {error:?}"))?;
        drawing
            .present()
            .map_err(|error| eyre!("save plot: {error:?}"))?;
    }
    std::fs::rename(&temporary, output)?;
    Ok(())
}

fn main() -> Result<()> {
    let scenario = Scenario::load_from_env()?;
    let output = std::env::var_os("TRAJECTORY_PLOT_PATH")
        .map(PathBuf::from)
        .unwrap_or_else(|| PathBuf::from("out/autonomous-path.png"));
    let (_node, mut events) = DoraNode::init_from_env()?;
    let mut planned = Vec::new();
    let mut actual = Vec::new();
    let mut status = None;
    let mut dirty = false;
    let mut finished = false;
    while let Some(event) = events.recv() {
        match event {
            Event::Input { id, data, .. } if id.as_str() == "path" => {
                let values = into_vec::<f64>(&data)?;
                let points = path_points(&values)?;
                if planned != points {
                    planned = points;
                    dirty = true;
                }
            }
            Event::Input { id, data, .. } if id.as_str() == "robot_pose" && !finished => {
                let values = into_vec::<f64>(&data)?;
                actual.push(pose_from_values(&values)?.point());
                dirty = true;
            }
            Event::Input { id, data, .. } if id.as_str() == "status" => {
                let values = into_vec::<u8>(&data)?;
                if let [code] = values.as_slice() {
                    status = Some(*code);
                    if *code == 2 || *code == 3 {
                        plot(&scenario, &planned, &actual, status, &output)?;
                        eprintln!("trajectory_plot: saved {}", output.display());
                        finished = true;
                        dirty = false;
                    }
                }
            }
            Event::Input { id, .. } if id.as_str() == "tick" && dirty && !finished => {
                plot(&scenario, &planned, &actual, status, &output)?;
                dirty = false;
            }
            Event::Stop(_) => {
                if dirty && !finished {
                    plot(&scenario, &planned, &actual, status, &output)?;
                }
                break;
            }
            _ => {}
        }
    }
    Ok(())
}
