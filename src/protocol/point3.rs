use crate::*;
use nalgebra as na;

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Point3 {
    /// [m]
    x: f64,
    /// [m]
    y: f64,
    /// [m]
    z: f64,
}

impl Point3 {
    pub fn new(x: f64, y: f64, z: f64) -> Self {
        Self { x, y, z }
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

    pub fn set_x(&mut self, x: f64) {
        self.x = x;
    }

    pub fn set_y(&mut self, y: f64) {
        self.y = y;
    }

    pub fn set_z(&mut self, z: f64) {
        self.z = z;
    }
}

impl Point for Point3 {
    fn distance(&self, other: &Self) -> f64 {
        ((self.x - other.x).powi(2) + (self.y - other.y).powi(2) + (self.z - other.z).powi(2))
            .sqrt()
    }

    fn distance_squared(&self, other: &Self) -> f64 {
        (self.x - other.x).powi(2) + (self.y - other.y).powi(2) + (self.z - other.z).powi(2)
    }
}

impl From<na::Vector3<f64>> for Point3 {
    fn from(v: na::Vector3<f64>) -> Self {
        Self::new(v.x, v.y, v.z)
    }
}

impl From<Point3> for na::Vector3<f64> {
    fn from(p: Point3) -> Self {
        na::Vector3::new(p.x, p.y, p.z)
    }
}

impl From<na::Translation3<f64>> for Point3 {
    fn from(t: na::Translation3<f64>) -> Self {
        Self::new(t.vector.x, t.vector.y, t.vector.z)
    }
}

impl From<Point3> for na::Translation3<f64> {
    fn from(p: Point3) -> Self {
        na::Translation3::new(p.x, p.y, p.z)
    }
}
