use crate::{circle_info::*, linear_info::*, robot_pose_time_stamped::*};

pub fn robot_pose_from_csv(robot_pose_file_path: &str) -> Vec<RobotPoseTimeStamped> {
    let mut robot_pose_time_stamped = vec![];

    let mut rdr_robot_pose = csv::Reader::from_path(robot_pose_file_path).unwrap();

    for result in rdr_robot_pose.records() {
        let record = result.unwrap();

        let x = record[0].parse::<f64>().unwrap();
        let y = record[1].parse::<f64>().unwrap();
        let theta = record[2].parse::<f64>().unwrap();
        let time = record[3].parse::<f64>().unwrap();

        robot_pose_time_stamped.push(RobotPoseTimeStamped {
            x,
            y,
            theta,
            time_stamp: time,
        });
    }

    robot_pose_time_stamped
}

pub fn linear_info_from_csv(linear_wall_file_path: &str) -> Vec<LinearInfo> {
    let mut linear_info = vec![];

    let mut rdr_linear_wall = csv::Reader::from_path(linear_wall_file_path).unwrap();

    for result in rdr_linear_wall.records() {
        let record = result.unwrap();

        let a = record[0].parse::<f64>().unwrap();
        let b = record[1].parse::<f64>().unwrap();
        let c = record[2].parse::<f64>().unwrap();
        let x_min = record[3].parse::<f64>().ok();
        let x_max = record[4].parse::<f64>().ok();
        let y_min = record[5].parse::<f64>().ok();
        let y_max = record[6].parse::<f64>().ok();

        linear_info.push(LinearInfo {
            a,
            b,
            c,
            x_min,
            x_max,
            y_min,
            y_max,
        });
    }

    linear_info
}

pub fn circle_info_from_csv(circle_wall_file_path: &str) -> Vec<CircleInfo> {
    let mut circle_info = vec![];

    let mut rdr_circle_wall = csv::Reader::from_path(circle_wall_file_path).unwrap();

    for result in rdr_circle_wall.records() {
        let record = result.unwrap();

        let x = record[0].parse::<f64>().unwrap();
        let y = record[1].parse::<f64>().unwrap();
        let radius = record[2].parse::<f64>().unwrap();
        let theta_min = record[3].parse::<f64>().unwrap();
        let theta_max = record[4].parse::<f64>().unwrap();

        circle_info.push(CircleInfo {
            x,
            y,
            radius,
            theta_min,
            theta_max,
        });
    }

    circle_info
}

#[cfg(test)]
mod test {
    use super::*;

    #[test]
    fn test_robot_pose_from_csv() {
        let robot_pose_file_path = "config/robot_pose.csv";

        let robot_pose_time_stamped = robot_pose_from_csv(robot_pose_file_path);

        robot_pose_time_stamped.iter().for_each(|p| {
            println!("robot_pose_time_stamped = {:?}", p);
        });
    }

    #[test]
    fn test_linear_info_from_csv() {
        let linear_info_file_path = "config/linear.csv";

        let linear_info = linear_info_from_csv(linear_info_file_path);

        linear_info.iter().for_each(|l| {
            println!("linear_info = {:?}", l);
        });
    }

    #[test]
    fn test_circle_info_from_csv() {
        let circle_info_file_path = "config/circle.csv";

        let circle_info = circle_info_from_csv(circle_info_file_path);

        circle_info.iter().for_each(|c| {
            println!("circle_info = {:?}", c);
        });
    }
}
