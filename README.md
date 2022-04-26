# Installation

This Rust package is not registered on [crates.io](https://crates.io/). If you want to install this, you will need to install it from this repo with either

```
cargo install --git https://github.com/kpwelsh/RobotIKBase.git
```

to install it locally, or by adding

```
[dependencies]
robot_ik_base = {git = "https://github.com/kpwelsh/RobotIKBase.git" }
```

to your Cargo.toml file to add it as a dependency. (Note that specifying remote git dependencies will prevent you from publishing your package)


# Usage

This package exposes a single interface intended to solve the robot arm inverse kinematics problem via non-linear optimization. For details on the mathematical approach and implementation, see [RobotIKBase](https://kpwelsh.github.io/RobotIKBase/). 

The ```IKSolver``` interface makes use of both the [k](https://github.com/openrr/k) library for kinematics and the [OptimizationEngine](https://github.com/alphaville/optimization-engine) non-linear optimizer to provide a simple interface for determining the necessary joint angles for a robot arm to reach a certain end-effector pose. There is *some* support for self-collision avoidance, but you may need to modify the cost function code yourself to achieve decent outcomes.

# Example

The following example can be found as a library test case.

```rust
// Create a solver instance from a URDF file
let mut solver = IKSolver::from_urdf_fp("test/test.urdf");
// Set the desired end-effector frame (k only likes this to be joints)
solver.set_ee("panda_joint8");

// Pick some pose to achieve. 
// This methods works fine for a generic test pose,
// but you will probably have an external pose source.
let q = solver.arm.as_ref().unwrap().joint_positions();
solver.arm.as_ref().unwrap().set_joint_positions_clamped(&q);
let pose = solver.arm.as_ref().unwrap().end_transform();
let mut q = solver.arm.as_ref().unwrap().joint_positions();
q[1] += 0.5;

// Ask it to solve the IK.
let result = solver.solve(&q, &pose);
assert!(result.is_ok());

// Here we just double check that it found an OK answer.
solver.arm.as_ref().unwrap().set_joint_positions(&result.unwrap()).unwrap();
let found_pose = solver.arm.as_ref().unwrap().end_transform();

assert!((found_pose.translation.vector - pose.translation.vector).magnitude() <= 1e-4);
```