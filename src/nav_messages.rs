use dora_node_api::{Metadata, MetadataParameters, Parameter};
use eyre::{Result, ensure, eyre};

use crate::navigation::Pose;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Stamp {
    pub step: i64,
    pub sim_time_ns: i64,
}

pub fn stamped_parameters(stamp: Stamp) -> MetadataParameters {
    let mut params = MetadataParameters::default();
    params.insert("step".into(), Parameter::Integer(stamp.step));
    params.insert("sim_time_ns".into(), Parameter::Integer(stamp.sim_time_ns));
    params
}

pub fn stamp_from_metadata(metadata: &Metadata) -> Result<Stamp> {
    let integer = |name: &str| match metadata.parameters.get(name) {
        Some(Parameter::Integer(value)) if *value >= 0 => Ok(*value),
        _ => Err(eyre!("missing or invalid {name} metadata")),
    };
    Ok(Stamp {
        step: integer("step")?,
        sim_time_ns: integer("sim_time_ns")?,
    })
}

pub fn float_parameter(metadata: &Metadata, name: &str) -> Result<f64> {
    match metadata.parameters.get(name) {
        Some(Parameter::Float(value)) if value.is_finite() => Ok(*value),
        _ => Err(eyre!("missing or invalid {name} metadata")),
    }
}

pub fn finite_values(values: &[f64], expected: usize) -> Result<()> {
    ensure!(
        values.len() == expected,
        "expected {expected} values, got {}",
        values.len()
    );
    ensure!(
        values.iter().all(|value| value.is_finite()),
        "nonfinite message value"
    );
    Ok(())
}

pub fn pose_from_values(values: &[f64]) -> Result<Pose> {
    finite_values(values, 3)?;
    Ok(Pose {
        x: values[0],
        y: values[1],
        theta: values[2],
    })
}

pub fn pose_values(pose: Pose) -> Vec<f64> {
    vec![pose.x, pose.y, pose.theta]
}
