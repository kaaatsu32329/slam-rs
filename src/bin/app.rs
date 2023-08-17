use std::f64::consts::FRAC_PI_3;

use nalgebra::Vector2;
use slam_rs::{PointCloud, Pose2d, Robot, Slam, SlamViewerApp};

fn main() {
    let mut app = SlamViewerApp::new();

    let robot = Robot::new(
        Pose2d::new(Vector2::new(1., 2.), FRAC_PI_3),
        Vector2::new(0., 0.),
    );
    let mut slam = Slam::new(robot);

    let point_cloud = PointCloud::new_from_csv("dataset/circle.csv").unwrap();
    slam.sensor_update(point_cloud);

    app.setup(slam);
    app.run();
}
