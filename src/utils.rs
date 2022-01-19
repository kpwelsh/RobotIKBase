
extern crate k;
use k::nalgebra as na;
use na::{UnitQuaternion, Vector3};
use optimization_engine::{SolverError};


fn groove_loss(x_val: f64, t: f64, d: i32, c: f64, f: f64, g: i32) -> f64 {
    -( (-(x_val - t).powi(d)) / (2.0 * c.powi(2) ) ).exp() + f * (x_val - t).powi(g)
}


pub fn finite_difference(f: &dyn Fn(&[f64], &mut f64) -> Result<(), SolverError>, u: &[f64], grad: &mut [f64]) -> Result<(), SolverError> {
    let h = 1e-6;
    let mut f0 = 0.0;
    f(u, &mut f0).unwrap();

    let mut x = [0.0,0.0,0.0,0.0,0.0,0.0,0.0];
    for i in 0..7 {
        x[i] = u[i];
    }

    for i in 0..7 {
        let mut fi = 0.0;
        x[i] += h;
        f(&x, &mut fi).unwrap();
        grad[i] = (fi - f0) / h;
        x[i] -= h;
    }

    Ok(())
}

pub fn position_cost(current_position: &Vector3<f64>, desired_position: &Vector3<f64>) -> f64 {
    2.0 * groove_loss((current_position - desired_position).norm(), 0., 2, 0.1, 10.0, 2)
}

pub fn rotation_cost(current_rotation: &UnitQuaternion<f64>, desired_rotation: &UnitQuaternion<f64>) -> f64 {
    groove_loss(current_rotation.angle_to(desired_rotation), 0., 2, 0.1, 10.0, 2)
}
