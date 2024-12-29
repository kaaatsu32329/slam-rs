use nalgebra as na;

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Point {
    /// [m]
    x: f64,
    /// [m]
    y: f64,
}

impl Point {
    pub fn new(x: f64, y: f64) -> Self {
        Self { x, y }
    }

    pub fn x(&self) -> f64 {
        self.x
    }

    pub fn y(&self) -> f64 {
        self.y
    }

    pub fn set_x(&mut self, x: f64) {
        self.x = x;
    }

    pub fn set_y(&mut self, y: f64) {
        self.y = y;
    }

    pub fn distance(&self, other: &Self) -> f64 {
        ((self.x - other.x).powi(2) + (self.y - other.y).powi(2)).sqrt()
    }
}

impl From<na::Vector2<f64>> for Point {
    fn from(v: na::Vector2<f64>) -> Self {
        Self::new(v.x, v.y)
    }
}

impl From<Point> for na::Vector2<f64> {
    fn from(p: Point) -> Self {
        na::Vector2::new(p.x, p.y)
    }
}

impl From<na::Translation2<f64>> for Point {
    fn from(t: na::Translation2<f64>) -> Self {
        Self::new(t.vector.x, t.vector.y)
    }
}

impl From<Point> for na::Translation2<f64> {
    fn from(p: Point) -> Self {
        na::Translation2::new(p.x, p.y)
    }
}
