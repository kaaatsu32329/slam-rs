use crate::*;
use bevy::prelude::*;

#[derive(Debug, Resource)]
pub struct Slam {
    robot: Robot,
    current_point_cloud: PointCloud,
    map: Map,
    slam_executor: SlamExecutor,
}

impl Slam {
    pub fn new(robot: Robot) -> Self {
        Self {
            robot,
            current_point_cloud: PointCloud::default(),
            map: Map::default(),
            slam_executor: SlamExecutor::default(),
        }
    }

    pub fn sensor_update(&mut self, points: Vec<Point2d>) {
        *self.current_point_cloud.center_mut() = self.robot.pose().clone();
        self.current_point_cloud.update_points(points);

        self.slam_executor
            .update(&self.current_point_cloud, &mut self.map);
    }

    pub fn odometry_update(&mut self, velocity: Velocity2d, delta_time: f64) {
        self.robot.update_from_velocity(velocity, delta_time);
        *self.current_point_cloud.center_mut() = self.robot.pose().clone();
    }

    pub fn robot(&self) -> &Robot {
        &self.robot
    }

    pub fn current_point_cloud(&self) -> &PointCloud {
        &self.current_point_cloud
    }

    pub fn map(&self) -> &Map {
        &self.map
    }
}
