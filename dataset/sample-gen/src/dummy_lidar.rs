use crate::{
    circle_info::*, csv_reader::*, lidar_info::LidarInfo, linear_info::*,
    robot_pose_time_stamped::*,
};

const ANGLE_RESOLUTION: f64 = 1.0;
const DISTANCE_MAX_LIMIT: f64 = 12.0;

pub fn get_lidar_data(
    robot_pose_file_path: &str,
    linear_wall_file_path: &str,
    circle_wall_file_path: &str,
) -> Vec<Vec<LidarInfo>> {
    let mut lidar_data = vec![];

    let steps = (360.0 / ANGLE_RESOLUTION) as usize;

    let robot_pose_time_stamped = robot_pose_from_csv(robot_pose_file_path);

    let linear_info = linear_info_from_csv(linear_wall_file_path);
    let circle_info = circle_info_from_csv(circle_wall_file_path);

    for pose in robot_pose_time_stamped {
        let mut data = vec![];
        for step in 0..steps {
            let angle = step as f64 * ANGLE_RESOLUTION;
            let mut distance = DISTANCE_MAX_LIMIT;

            for linear in &linear_info {
                if let Some(d) = distance_robot_pose_to_linear(&pose, &linear, angle) {
                    distance = distance.min(d);
                }
            }
            for circle in &circle_info {
                if let Some(d) = distance_robot_pose_to_circle(&pose, &circle, angle) {
                    distance = distance.min(d);
                }
            }
            data.push(LidarInfo {
                angle_deg: angle,
                distance,
            });
        }
        lidar_data.push(data);
    }

    lidar_data
}

fn distance_robot_pose_to_linear(
    robot_pose: &RobotPoseTimeStamped,
    linear_info: &LinearInfo,
    angle: f64,
) -> Option<f64> {
    let target_linear_info = robot_pose.to_linear_info_from_angle(angle);

    let intersect_point = linear_info.get_intersection(&target_linear_info);

    match intersect_point {
        Some((x, y)) => Some(robot_pose.get_distance_to_point(x, y)),
        None => None,
    }
}

fn distance_robot_pose_to_circle(
    robot_pose: &RobotPoseTimeStamped,
    circle_info: &CircleInfo,
    angle: f64,
) -> Option<f64> {
    let target_linear_info = robot_pose.to_linear_info_from_angle(angle);

    let intersect_point = circle_info.get_intersection(&target_linear_info);

    match intersect_point {
        Some(points) => {
            let mut distance = f64::MAX;
            for p in points {
                distance = robot_pose.get_distance_to_point(p.0, p.1).min(distance);
            }
            Some(distance)
        }
        None => None,
    }
}

#[cfg(test)]
mod test {
    use super::*;

    const _ROBOT_POSE_FILE_PATH: &str = "config/robot_pose.csv";
    const _LINEAR_WALL_FILE_PATH: &str = "config/linear.csv";
    const _CIRCLE_WALL_FILE_PATH: &str = "config/circle.csv";

    #[test]
    fn test_get_lidar_data() {}

    #[test]
    fn test_distance_robot_pose_to_linear() {
        let robot_pose = RobotPoseTimeStamped {
            x: 1.0,
            y: 2.0,
            theta: 180.0,
            time_stamp: 0.0,
        };

        let linear_info1 = LinearInfo {
            a: 3.0,
            b: 4.0,
            c: 5.0,
            x_min: None,
            x_max: None,
            y_min: None,
            y_max: None,
        };

        let angle = 90.0;

        let distance1 = distance_robot_pose_to_linear(&robot_pose, &linear_info1, angle);

        assert_eq!(distance1, Some(4.));
    }
}
