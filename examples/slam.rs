use grid_map::Position;
use slam::*;

const INIT_ROBOT_POSE_X: f64 = -2.0;
const INIT_ROBOT_POSE_Y: f64 = 0.5;
const INIT_ROBOT_POSE_THETA: f64 = 0.0;

fn main() {
    let scan_log_file_name = "sample/ros2_scan_log.yaml";
    let odom_log_file_name = "sample/ros2_odom_log.yaml";
    let scan_log_path = format!("{}/{}", env!("CARGO_MANIFEST_DIR"), scan_log_file_name);
    let odom_log_path = format!("{}/{}", env!("CARGO_MANIFEST_DIR"), odom_log_file_name);

    let mut data_loader = DebuggerYaml::new(&scan_log_path, &odom_log_path);

    let min_point = Position::new(-5.0, -5.0);
    let max_point = Position::new(5.0, 5.0);
    let resolution = 0.02;

    let mapping = Mapping::new(
        min_point,
        max_point,
        resolution,
        DEFAULT_PROBABILITY_FREE_SPACE,
        DEFAULT_PROBABILITY_OCCUPIED_SPACE,
    );
    let init_pose = Pose2::new(INIT_ROBOT_POSE_X, INIT_ROBOT_POSE_Y, INIT_ROBOT_POSE_THETA);
    let mut slam_runner = SlamRunner::new(mapping, init_pose);

    let mut map_viz = MapViz2::new();

    while let Some((laser_scan, _)) = data_loader.next_scan_2d() {
        slam_runner.update(&laser_scan, &Odometry::default());
        map_viz.update(slam_runner.mapping(), slam_runner.robot_pose());

        std::thread::sleep(std::time::Duration::from_millis(25));
    }
}
