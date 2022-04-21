mod utils;
use utils::*;

use std::time::Duration;
extern crate k;
use k::{Isometry3};
use k::{Chain, SerialChain};
use optimization_engine::{constraints::*, panoc::*, *};
use urdf_rs;
use std::fs::read_to_string;

/// Struct for holding persistant state between ik solving
pub struct IKSolver {
    pub robot : Chain<f64>,
    pub arm : Option<SerialChain<f64>>,
    pub cache : Option<PANOCCache>,
    pub self_collision: bool,
    pub maxiter: Option<usize>,
    pub maxtime: Option<Duration>
}

impl IKSolver {
    pub fn from_robot(robot: Chain<f64>) -> IKSolver {
        IKSolver {
            robot: robot,
            arm : None,
            cache : None,
            self_collision: false,
            maxiter: None,
            maxtime: None
        }
    }

    pub fn from_urdf_str(urdf: &str) -> IKSolver {
        let robot = urdf_rs::read_from_string(urdf).unwrap().into();
        IKSolver::from_robot(robot)
    }

    pub fn from_urdf_fp(fp: &str) -> IKSolver {
        IKSolver::from_urdf_str(&read_to_string(fp).unwrap())
    }

    pub fn set_ee(&mut self, frame: &str) {
        let ee = self.robot.find(frame).unwrap();
        self.arm = Some(SerialChain::from_end(ee));
        self.cache = self.arm.as_ref().and_then(|arm| Some(PANOCCache::new(arm.dof(), 1e-3, 10)));
    }

    pub fn dof(&self) -> i32 {
        match self.arm.as_ref() {
            Some(arm) => {
                arm.dof() as i32
            },
            None => {
                -1
            }
        }
    }

    pub fn solve(&mut self, current_q: &Vec<f64>, pose: &Isometry3<f64>) -> Result<Vec<f64>, SolverError> {
        if let (Some(arm), Some(mut cache)) = (self.arm.as_ref(), self.cache.as_mut()) {
            let mut collision_links = Vec::new();
            for link in arm.iter() {
                // This is not generally right.
                // Assumes that there is exactly 1 capsule link per actual robot link.
                if link.link().is_some() {
                    collision_links.push(vec![link]);
                }
            }
            let use_self_collision = self.self_collision;
            
            let cost = |u: &[f64], c: &mut f64| {
                arm.set_joint_positions_clamped(&u);
                arm.update_transforms();
                *c = 0.0;
        
                let trans = arm.end_transform();
        
                *c += position_cost(&trans.translation.vector, &pose.translation.vector);
                *c += rotation_cost(&trans.rotation, &pose.rotation);

                if use_self_collision {
                    arm.update_link_transforms();
                    *c += self_collision_cost(&collision_links);
                }
                Ok(())
            };
        
            let dcost = |u: &[f64], grad: &mut [f64]| {
                finite_difference(&cost, u, grad)
            };
        
            let mut u = current_q.clone();
            
            let mut lb: Vec<f64> = current_q.iter().map(|_| {f64::NEG_INFINITY}).collect();
            let mut ub: Vec<f64> = current_q.iter().map(|_| {f64::INFINITY}).collect();
        
            for (i, joint) in arm.iter_joints().enumerate() {
                match joint.limits {
                    Some(range) => {
                        lb[i] = range.min;
                        ub[i] = range.max;
                    },
                    None => { }
                }
            }
            let bounds = Rectangle::new(Some(&lb[..]), Some(&ub[..]));
            let problem = Problem::new(
                &bounds,
                dcost, 
                cost
            );
            bounds.project(&mut u);

            let mut opt = PANOCOptimizer::new(problem, &mut cache);
            if let Some(maxiter) = self.maxiter {
                opt = opt.with_max_iter(maxiter);
            }
            if let Some(maxtime) = self.maxtime {
                opt = opt.with_max_duration(maxtime);
            }

            let status = opt.solve(&mut u);
            println!("{:?}", status);
            return status.and_then(|_status| Ok(u));
        }

        Err(SolverError::Cost)
    }
}


#[cfg(test)]
mod tests {
    use crate::IKSolver;

    #[test]
    fn it_runs() {
        let mut solver = IKSolver::from_urdf_fp("test/test.urdf");

        solver.set_ee("panda_joint8");
        
        let q = solver.arm.as_ref().unwrap().joint_positions();
        solver.arm.as_ref().unwrap().set_joint_positions_clamped(&q);
        let pose = solver.arm.as_ref().unwrap().end_transform();
        let mut q = solver.arm.as_ref().unwrap().joint_positions();
        q[1] += 0.5;
        let result = solver.solve(&q, &pose);
        assert!(result.is_ok());

        solver.arm.as_ref().unwrap().set_joint_positions(&result.unwrap()).unwrap();
        let found_pose = solver.arm.as_ref().unwrap().end_transform();
        
        assert!((found_pose.translation.vector - pose.translation.vector).magnitude() <= 1e-4);
    }

    
    #[test]
    fn it_runs_withcollision() {
        let mut solver = IKSolver::from_urdf_fp("test/test.urdf");

        solver.set_ee("panda_joint8");
        solver.self_collision = true;
        
        let mut q = solver.arm.as_ref().unwrap().joint_positions();
        q[2] = 1.;
        solver.arm.as_ref().unwrap().set_joint_positions_clamped(&q);
        let q = solver.arm.as_ref().unwrap().joint_positions();
        let pose = solver.arm.as_ref().unwrap().end_transform();
        let result = solver.solve(&q, &pose);
        assert!(result.is_ok());

        solver.arm.as_ref().unwrap().set_joint_positions(&result.unwrap()).unwrap();
        let found_pose = solver.arm.as_ref().unwrap().end_transform();
        
        assert!((found_pose.translation.vector - pose.translation.vector).magnitude() <= 1e-4);
    }
}