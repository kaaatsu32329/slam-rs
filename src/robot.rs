use nalgebra as na;

pub type Pose2d = na::Isometry2<f64>;
pub type Velocity2d = na::Isometry2<f64>;

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

    pub fn update_from_pose(&mut self, pose: Pose2d) {
        self.pose = pose;
    }

    pub fn update_from_velocity(&mut self, velocity: Velocity2d, delta_time: f64) {
        self.velocity = velocity;
        self.pose.translation.x += (self.pose.rotation.angle().cos() * self.velocity.translation.x
            - self.pose.rotation.angle().sin() * self.velocity.translation.y)
            * delta_time;
        self.pose.translation.y += (self.pose.rotation.angle().sin() * self.velocity.translation.x
            + self.pose.rotation.angle().cos() * self.velocity.translation.y)
            * delta_time;
        self.pose.rotation = na::UnitComplex::new(
            self.pose.rotation.angle() + self.velocity.rotation.angle() * delta_time,
        );
    }
}
