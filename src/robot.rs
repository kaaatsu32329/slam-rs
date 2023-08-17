use nalgebra as na;

pub type Pose2d = na::Isometry2<f64>;
pub type Velocity2d = na::Vector2<f64>;

#[derive(Debug, Clone, Copy)]
pub struct Robot {
    pose: Pose2d,
    velocity: Velocity2d,
}

impl Robot {
    pub fn new(pose: Pose2d, velocity: Velocity2d) -> Self {
        Self { pose, velocity }
    }

    pub fn pose(&self) -> &Pose2d {
        &self.pose
    }

    pub fn velocity(&self) -> &Velocity2d {
        &self.velocity
    }
}
