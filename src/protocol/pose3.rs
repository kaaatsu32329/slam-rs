use crate::*;
use nalgebra as na;

// TODO: Change method etc.; Or use the `nalgebra` type as it is with the Type alias.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Pose3 {
    /// [m]
    x: f64,
    /// [m]
    y: f64,
    /// [m]
    z: f64,
    /// Quaternion
    q: na::UnitQuaternion<f64>,
}

impl Pose3 {
    pub fn new(x: f64, y: f64, z: f64, q: na::UnitQuaternion<f64>) -> Self {
        Self { x, y, z, q }
    }

    pub fn x(&self) -> f64 {
        self.x
    }

    pub fn y(&self) -> f64 {
        self.y
    }

    pub fn z(&self) -> f64 {
        self.z
    }

    pub fn q(&self) -> na::UnitQuaternion<f64> {
        self.q
    }

    pub fn set_x(&mut self, x: f64) {
        self.x = x;
    }

    pub fn set_y(&mut self, y: f64) {
        self.y = y;
    }

    pub fn set_z(&mut self, z: f64) {
        self.z = z;
    }

    pub fn set_q(&mut self, q: na::UnitQuaternion<f64>) {
        self.q = q;
    }
}

impl Point for Pose3 {
    fn distance(&self, other: &Self) -> f64 {
        ((self.x - other.x).powi(2)
            + (self.y - other.y).powi(2)
            + (self.z - other.z).powi(2)
            + (self.q.angle_to(&other.q)).powi(2))
        .sqrt()
    }

    fn distance_squared(&self, other: &Self) -> f64 {
        (self.x - other.x).powi(2)
            + (self.y - other.y).powi(2)
            + (self.z - other.z).powi(2)
            + (self.q.angle_to(&other.q)).powi(2)
    }
}

impl From<na::Isometry3<f64>> for Pose3 {
    fn from(isometry: na::Isometry3<f64>) -> Self {
        let translation = isometry.translation.vector;
        let rotation = isometry.rotation;
        Self::new(translation.x, translation.y, translation.z, rotation)
    }
}

impl From<Pose3> for na::Isometry3<f64> {
    fn from(p: Pose3) -> Self {
        let translation = na::Vector3::new(p.x, p.y, p.z);
        let rotation = p.q;
        na::Isometry3::from_parts(na::Translation3::from(translation), rotation)
    }
}
