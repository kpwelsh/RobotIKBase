mod utils;
use utils::*;

use std::error::Error;
use once_cell::sync::Lazy;
use std::sync::Mutex;

extern crate k;
use k::nalgebra as na;
use na::{UnitQuaternion, Vector3, Quaternion};
use optimization_engine::{constraints::*, panoc::*, *};
use urdf_rs;

type MaybeError<T> = Result<T, Box<dyn Error>>;
type Robot = k::Chain<f64>;

#[derive(Debug, Clone)]
pub struct IKError {
    pub code : i32
}

struct IKState {
    pub robot : Robot,
    pub cache : PANOCCache
}

static STATE: Lazy<Mutex<Vec<IKState>>> = Lazy::new(|| Mutex::new(Vec::new()));

pub fn init(urdf: &str, dof: usize) -> MaybeError<usize> {
    let robot : Robot = urdf_rs::read_from_string(urdf)?.into();
    let cache = PANOCCache::new(dof, 1e-3, 100);
    let state = IKState {robot, cache};
    let mut cache = STATE.lock().unwrap();
    cache.push(state);
    Ok(cache.len() - 1)
}

pub fn dof(robot_id: usize, frame: &str) -> usize {
    let mut cache = STATE.lock().unwrap();
    if robot_id >= cache.len() {
        return 0;
    }
    let state = &mut cache[robot_id];
    let ee = state.robot.find(frame).unwrap();
    let arm = k::SerialChain::from_end(ee);
    return arm.dof();
}

pub fn solve_32(robot_id: usize, current_q: &Vec<f32>, frame: &str, x: &Vector3<f32>, rot: &UnitQuaternion<f32>) -> Result<Vec<f32>, IKError> {
    let current_q_64 = current_q.iter().map(|&v| v as f64).collect();
    let x = Vector3::new(x[0] as f64, x[1] as f64, x[2] as f64);
    let rot = UnitQuaternion::from_quaternion(Quaternion::new(rot.w as f64, rot.i as f64, rot.j as f64, rot.k as f64));

    let result = solve(robot_id, &current_q_64, frame, &x, &rot);
    result.map(|r| r.iter().map(|&v| v as f32).collect())
}


pub fn solve(robot_id: usize, current_q: &Vec<f64>, frame: &str, x: &Vector3<f64>, rot: &UnitQuaternion<f64>) -> Result<Vec<f64>, IKError> {
    let mut cache = STATE.lock().unwrap();
    if cache.len() <= robot_id {
        return Err(IKError{code:15});
    }
    let state = &mut cache[robot_id];

    let ee = state.robot.find(frame).ok_or(IKError{code: 11})?;
    let arm = k::SerialChain::from_end(ee);
    

    let cost = |u: &[f64], c: &mut f64| {
        arm.set_joint_positions_clamped(u);
        arm.update_transforms();
        *c = 0.0;

        let trans = arm.find(frame).ok_or(SolverError::Cost)?.world_transform().ok_or(SolverError::Cost)?;

        *c += position_cost(&trans.translation.vector, x);
        *c += rotation_cost(&trans.rotation, rot);
        Ok(())
    };

    let dcost = |u: &[f64], grad: &mut [f64]| {
        finite_difference(&cost, u, grad)
    };

    let mut u = current_q.clone();
    
    let mut lb: Vec<f64> = current_q.iter().map(|_| {-1000.0}).collect();
    let mut ub: Vec<f64> = current_q.iter().map(|_| {1000.0}).collect();

    for (i, joint) in arm.iter_joints().enumerate() {
        lb[i] = joint.limits.ok_or(IKError{code:12})?.min + 0.1;
        ub[i] = joint.limits.ok_or(IKError{code:12})?.max - 0.1;
    }

    let bounds = Rectangle::new(Some(&lb[..]), Some(&ub[..]));
    let problem = Problem::new(
        &bounds,
        dcost, 
        cost
    );
    bounds.project(&mut u);
    let mut panoc = PANOCOptimizer::new(problem, &mut state.cache)
                        .with_max_iter(100);
    
    if panoc.solve(&mut u).is_err() {
        return Err(IKError{code:13});
    }
    Ok(u)
}
