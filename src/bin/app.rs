use nalgebra::Vector2;
use parking_lot::Mutex;
use slam_rs::{PointCloud, Pose2d, Robot, Slam, SlamViewerApp, Velocity2d};
use std::sync::Arc;

fn main() {
    let mut app = SlamViewerApp::new();

    let robot = Robot::new(
        Pose2d::new(Vector2::new(1., 2.), std::f64::consts::FRAC_PI_3),
        Velocity2d::new(Vector2::new(0., 0.), 0.),
    );

    let mut slam = Slam::new(robot);
    let point_cloud = PointCloud::new_from_csv("dataset/circle.csv").unwrap();
    slam.sensor_update(point_cloud.points().clone());

    let slam = Arc::new(Mutex::new(slam));

    let cloned_slam = slam.clone();
    let bevy_cloned_slam = slam.clone();

    let velocity = Velocity2d::new(Vector2::new(1., 0.), 0.2);
    let delta_time = 0.05;

    std::thread::spawn(move || {
        for _ in 0..200 {
            cloned_slam.lock().odometry_update(velocity, delta_time);

            std::thread::sleep(std::time::Duration::from_millis(50));
        }
    });

    app.setup(bevy_cloned_slam);
    app.run();
}
