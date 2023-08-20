use nalgebra::Vector2;
use parking_lot::Mutex;
use slam_rs::{PointCloud, Pose2d, Robot, Slam, SlamViewerApp, Velocity2d};
use std::sync::Arc;

const SAMPLE_DATASET_PATH: &str = "dataset/sample-gen/output/sample";

fn main() {
    let mut app = SlamViewerApp::new();

    let robot = Robot::new(
        Pose2d::new(Vector2::new(-2., -2.), 0_f64.to_radians()),
        Velocity2d::new(Vector2::new(0., 0.), 0.),
    );

    let slam = Arc::new(Mutex::new(Slam::new(robot)));

    let cloned_slam = slam.clone();
    let bevy_cloned_slam = slam.clone();

    let velocity = Velocity2d::new(Vector2::new(1., 0.), 0.);
    let delta_time = 0.1;

    std::thread::spawn(move || {
        for i in 0..=40 {
            let point_cloud =
                PointCloud::new_from_csv(&format!("{SAMPLE_DATASET_PATH}{i}.csv")).unwrap();

            cloned_slam.lock().odometry_update(velocity, delta_time);
            cloned_slam
                .lock()
                .sensor_update(point_cloud.points().clone());

            std::thread::sleep(std::time::Duration::from_millis(100));
        }
    });

    app.setup(bevy_cloned_slam);
    app.run();
}
