use std::io::Write;

use sample_gen::*;

const ROBOT_POSE_FILE_PATH: &str = "config/robot_pose.csv";
const LINEAR_WALL_FILE_PATH: &str = "config/linear.csv";
const CIRCLE_WALL_FILE_PATH: &str = "config/circle.csv";

const OUTPUT_FILE_PATH: &str = "output";

fn main() {
    let lidar_info = get_lidar_data(
        ROBOT_POSE_FILE_PATH,
        LINEAR_WALL_FILE_PATH,
        CIRCLE_WALL_FILE_PATH,
    );

    for (idx, info) in lidar_info.iter().enumerate() {
        let mut file =
            std::fs::File::create(format!("{OUTPUT_FILE_PATH}/sample{idx}.csv")).unwrap();
        let inner = lidar_info_to_csv_format(info);

        file.write_all(inner.as_bytes()).unwrap();
        file.flush().unwrap();
    }
}
