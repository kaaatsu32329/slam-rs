use grid_map::Position;
use slam::*;

fn main() {
    let scan_log_file_name = "sample/ros2_scan_log.yaml";
    let odom_log_file_name = "sample/ros2_odom_log.yaml";
    let scan_log_path = format!("{}/{}", env!("CARGO_MANIFEST_DIR"), scan_log_file_name);
    let odom_log_path = format!("{}/{}", env!("CARGO_MANIFEST_DIR"), odom_log_file_name);

    let mut data_loader = DebuggerYaml::new(&scan_log_path, &odom_log_path);

    let min_point = Position::new(-5.0, -5.0);
    let max_point = Position::new(5.0, 5.0);
    let resolution = 0.02;

    let mut mapping = Mapping::new(
        min_point,
        max_point,
        resolution,
        DEFAULT_PROBABILITY_FREE_SPACE,
        DEFAULT_PROBABILITY_OCCUPIED_SPACE,
    );

    let mut map_viz = MapViz::new();

    while let Some((laser_scan, current_position)) = data_loader.next_scan_2d() {
        mapping.update(&current_position, &laser_scan);

        let current_pose = Pose::new(
            current_position.translation.vector.x,
            current_position.translation.vector.y,
            current_position.rotation.angle(),
        );
        map_viz.update(&mapping, current_pose);
        std::thread::sleep(std::time::Duration::from_millis(25));
    }
}
