use crate::*;
use bevy::prelude::*;

#[derive(Debug, Resource)]
pub struct Slam {
    robot: Robot,
    current_point_cloud: PointCloud,
}

impl Slam {
    pub fn new(robot: Robot) -> Self {
        Self {
            robot,
            current_point_cloud: PointCloud::default(),
        }
    }

    pub fn sensor_update(&mut self, point_cloud: PointCloud) {
        self.current_point_cloud = point_cloud;
    }

    pub fn robot(&self) -> &Robot {
        &self.robot
    }

    pub fn current_point_cloud(&self) -> &PointCloud {
        &self.current_point_cloud
    }
}
