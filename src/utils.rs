
extern crate k;
use k::{nalgebra as na, link::Geometry};
use na::{UnitQuaternion, Vector3, Vector2};
use optimization_engine::{SolverError};

fn groove_loss(x_val: f64, t: f64, d: i32, c: f64, f: f64, g: i32) -> f64 {
    -( (-(x_val - t).powi(d)) / (2.0 * c.powi(2) ) ).exp() + f * (x_val - t).powi(g)
}

pub fn finite_difference(f: &dyn Fn(&[f64], &mut f64) -> Result<(), SolverError>, u: &[f64], grad: &mut [f64]) -> Result<(), SolverError> {
    let h = 1e-6;
    let mut f0 = 0.0;
    f(u, &mut f0).unwrap();

    let mut x : Vec<f64> = u.iter().map(|&v| v).collect();
    for i in 0..x.len() {
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

fn closest_point_ls_p(a: &Vector2<f64>, b: &Vector2<f64>, p: &Vector2<f64>) -> Vector2<f64> {
    let pa = p - a;
    let ba = b - a;
    let h = pa.dot(&ba).clamp(0., 1.);

    a + ba * h
}

pub struct Capsule {
    pub a: Vector3<f64>,
    pub b: Vector3<f64>,
    pub r: f64
}

// This is specifically oriented to go counter clockwise around a rectangle.
static DOMAIN_VERTICES : [Vector2<f64>; 4] = [
    Vector2::<f64>::new(0., 0.),
    Vector2::<f64>::new(1., 0.),
    Vector2::<f64>::new(1., 1.),
    Vector2::<f64>::new(0., 1.),
];

static Z : Vector3<f64> = Vector3::new(0., 0., 1.);

impl  Capsule {

    pub fn s(&self) -> Vector3<f64> {
        self.b - self.a
    }

    pub fn distance(&self, other: &Capsule) -> f64 {
        // Mohammad Safeea, Pedro Neto, Richard Bearee. Efficient Calculation of Minimum Distance Between Capsules and Its Use in Robotics. 
        // IEEE Access, IEEE, 2018, 7, pp.5368-5373. ff10.1109/ACCESS.2018.2889311ff. ffhal-02050431f

        let mut a_mat = na::Matrix3x2::zeros();
        a_mat.set_column(0, &other.s());
        a_mat.set_column(1, &-self.s());


        let qr = a_mat.qr();
        let q = qr.q();
        let r = qr.r();

        let y = other.a - self.a;

        let mut umin= Vector2::new(f64::INFINITY, f64::INFINITY);
        let origin = Vector2::<f64>::zeros();
        
        let mut contains_origin = true;
        let domain_offset = q.transpose() * y;
        for i in 0..4 {
            let a = r * DOMAIN_VERTICES[i] + domain_offset;
            let b = r * DOMAIN_VERTICES[(i + 1) % (DOMAIN_VERTICES.len())] + domain_offset;

            let p = closest_point_ls_p(&a, &b, &origin);

            if p.magnitude_squared() < umin.magnitude_squared() {
                umin = p;
            }

            contains_origin = contains_origin && {
                let ba = b - a;
                let perp = Vector2::new(-ba[1], ba[0]);
                perp.dot(&-a) >= 0.
            };
        }

        if contains_origin {
            umin = Vector2::<f64>::zeros();
        }
        
        (umin.magnitude_squared() + y.magnitude_squared() - (y.transpose() * q * q.transpose() * y)[0]).sqrt() - self.r - other.r
    }
}


fn link_to_capsules(link: &k::Link<f64>) -> Vec<Capsule> {
    let mut capsules = Vec::new();
    for collision in link.collisions.iter() {
        if let Geometry::Capsule { radius: r, length: l } = collision.geometry {
            let origin = collision.world_transform().unwrap();
            
            capsules.push(
                Capsule { 
                    a: origin.rotation * &(-Z * l / 2.) + origin.translation.vector, 
                    b: origin.rotation * &(Z * l / 2.) + origin.translation.vector, 
                    r: r 
                }
            )
        }
    }

    capsules
}

fn link_group_to_capsules(link_group: &Vec<&k::Node<f64>>) -> Vec<Capsule> {
    link_group.iter()
        .map(|&node| node.link())
        .map(|link| link_to_capsules(link.as_ref().unwrap()))
        .flatten()
        .collect()
}

fn capsule_group_dist(capsules_a: &Vec<Capsule>, capsules_b: &Vec<Capsule>) -> f64 {
    let mut dist = f64::INFINITY;
    for a in capsules_a.iter() {
        for b in capsules_b.iter() {
            dist = dist.min(a.distance(b));
        }
    }
    dist
}

fn bounded_reciprocal_cost(x: f64, lb: f64, ub: f64) -> f64 {
    let x = x.clamp(lb, ub);
    let p = 2;
    x.powi(-p) - ub.powi(-p) + (x.powi(p) - ub.powi(p)) * ub.powi(-(p+1))
}

pub fn self_collision_cost(collision_links: &Vec<Vec<&k::Node<f64>>>) -> f64 {
    let mut cost = 0.;

    for i in 0..collision_links.len() {
        let capsules_a : Vec<Capsule> = link_group_to_capsules(&collision_links[i]);
        for j in (i+2)..collision_links.len() {
            let capsules_b : Vec<Capsule> = link_group_to_capsules(&collision_links[j]);
            
            let d = capsule_group_dist(&capsules_a, &capsules_b);
            cost += bounded_reciprocal_cost(d, 1e-6, 0.05);
        }
    }

    cost
}


#[cfg(test)]
mod tests {

    use crate::Capsule;
    use crate::k::nalgebra as na;

    use na::{Vector3};

    #[test]
    fn intersecting() {
        let a = Vector3::new(1., 1., 1.);
        let b = Vector3::new(1., 1., -1.);

        let cap_a = Capsule {a: -a, b: a, r: 0.};
        let cap_b = Capsule {a: -b, b: b, r: 0.};

        assert!((cap_a.distance(&cap_b) - 0.0).abs() <= 1e-6);
    }
    
    #[test]
    fn intersecting_nonzero() {
        let offset = Vector3::new(5., 10., 20.);
        let a = Vector3::new(1., 1., 1.) + offset;
        let b = Vector3::new(1., 1., -1.) + offset;

        let cap_a = Capsule {a: -a, b: a, r: 0.};
        let cap_b = Capsule {a: -b, b: b, r: 0.};

        assert!((cap_a.distance(&cap_b) - 0.0).abs() <= 1e-6);
    }

    
    #[test]
    fn close() {
        let offset = Vector3::new(5., 10., 20.);
        let a = Vector3::new(1., 1., 1.) + offset;
        let b = Vector3::new(1., 1., 1.) + offset;

        let rel_offset = Vector3::new(1.5, 0., 0.);

        let cap_a = Capsule {a: (-a + rel_offset), b: (a + rel_offset), r: 0.};
        let cap_b = Capsule {a: -b, b: b, r: 0.};

        assert!((cap_a.distance(&cap_b) - rel_offset.magnitude()).abs() <= 1e-6);
    }
}
