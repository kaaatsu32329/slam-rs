use nalgebra as na;

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Pose {
    /// [m]
    x: f64,
    /// [m]
    y: f64,
    /// [rad]
    theta: f64,
}

impl Pose {
    pub fn new(x: f64, y: f64, theta: f64) -> Self {
        Self { x, y, theta }
    }

    pub fn x(&self) -> f64 {
        self.x
    }

    pub fn y(&self) -> f64 {
        self.y
    }

    pub fn theta(&self) -> f64 {
        self.theta
    }

    pub fn set_x(&mut self, x: f64) {
        self.x = x;
    }

    pub fn set_y(&mut self, y: f64) {
        self.y = y;
    }

    pub fn set_theta(&mut self, theta: f64) {
        self.theta = theta;
    }

    pub fn distance(&self, other: &Self) -> f64 {
        ((self.x - other.x).powi(2) + (self.y - other.y).powi(2)).sqrt()
    }
}

impl From<na::Isometry2<f64>> for Pose {
    fn from(isometry: na::Isometry2<f64>) -> Self {
        let translation = isometry.translation.vector;
        let rotation = isometry.rotation.angle();
        Self::new(translation.x, translation.y, rotation)
    }
}

impl From<Pose> for na::Isometry2<f64> {
    fn from(pose: Pose) -> Self {
        na::Isometry2::new(na::Vector2::new(pose.x, pose.y), pose.theta)
    }
}

// TODO: Check following implementations are correct
// NOTE: These implementations are required for argmin crate
impl argmin_math::ArgminMul<f64, Pose> for Pose {
    fn mul(&self, other: &f64) -> Pose {
        Pose {
            x: self.x * other,
            y: self.y * other,
            theta: self.theta * other,
        }
    }
}

impl argmin_math::ArgminAdd<Pose, Pose> for Pose {
    fn add(&self, other: &Pose) -> Pose {
        Pose {
            x: self.x + other.x,
            y: self.y + other.y,
            theta: self.theta + other.theta,
        }
    }
}

impl argmin_math::ArgminScaledAdd<Pose, f64, Pose> for Pose {
    fn scaled_add(&self, factor: &f64, other: &Pose) -> Pose {
        Pose {
            x: self.x + factor * other.x,
            y: self.y + factor * other.y,
            theta: self.theta + factor * other.theta,
        }
    }
}

impl argmin_math::ArgminSub<Pose, Pose> for Pose {
    fn sub(&self, other: &Pose) -> Pose {
        Pose {
            x: self.x - other.x,
            y: self.y - other.y,
            theta: self.theta - other.theta,
        }
    }
}

impl argmin_math::ArgminDot<Pose, f64> for Pose {
    fn dot(&self, other: &Pose) -> f64 {
        self.x * other.x + self.y * other.y + self.theta * other.theta
    }
}

impl argmin_math::ArgminZero for Pose {
    fn zero() -> Pose {
        Pose {
            x: 0.0,
            y: 0.0,
            theta: 0.0,
        }
    }
}
