use crate::*;

pub struct SlamRunner {
    mapping: Mapping,
    icp_client: IterativeClosestPoint2,
    latest_pointcloud: Option<Pointcloud2>,
    robot_pose: Pose2,
}

impl SlamRunner {
    pub fn new(mapping: Mapping, init_pose: Pose2) -> Self {
        Self {
            mapping,
            icp_client: IterativeClosestPoint2::new(&init_pose),
            latest_pointcloud: None,
            robot_pose: init_pose,
        }
    }

    pub fn update(&mut self, laser_scan: &LaserScan, odom: &Odometry) {
        let latest_pointcloud = self
            .latest_pointcloud
            .clone()
            .unwrap_or(laser_scan.clone().into());

        let preprocessed_pointcloud =
            Self::pointcloud_preprocessing(&laser_scan.clone().into(), 0.1);

        self.icp_client
            .set_data(&preprocessed_pointcloud, &latest_pointcloud);
        let robot_pose_with_icp = self.icp_client.scan_matching(10);

        let aaa = 1.00;
        let bbb = 0.00;

        self.robot_pose = Pose2::new(
            aaa * odom.pose().translation.x + bbb * robot_pose_with_icp.x(),
            aaa * odom.pose().translation.y + bbb * robot_pose_with_icp.y(),
            aaa * odom.pose().rotation.angle() + bbb * robot_pose_with_icp.theta(),
        );

        self.mapping.update(&self.robot_pose, laser_scan);

        self.latest_pointcloud = Some(preprocessed_pointcloud);
    }

    pub fn mapping(&self) -> &Mapping {
        &self.mapping
    }

    pub fn robot_pose(&self) -> Pose2 {
        self.robot_pose
    }

    fn pointcloud_preprocessing(pointcloud: &Pointcloud2, interval: f64) -> Pointcloud2 {
        let lines = ransac_algorithm(pointcloud, 100, 0.05);
        let mut preprocessed_points = Vec::new();
        let mut remove_points_idxs = Vec::new();

        for (idxs, line) in lines {
            remove_points_idxs.extend_from_slice(&idxs);
            let (lower_point, upper_point) = line.edge();
            let center_point = Point2::new(
                (lower_point.x() + upper_point.x()) / 2.0,
                (lower_point.y() + upper_point.y()) / 2.0,
            );
            let angle =
                (upper_point.y() - lower_point.y()).atan2(upper_point.x() - lower_point.x());
            let half_edge_distance = center_point.distance(&upper_point);

            let mut current_upper_points = center_point.clone();
            let mut current_lower_points = center_point.clone();
            while current_upper_points.distance(&center_point) < half_edge_distance {
                current_upper_points = Point2::new(
                    current_upper_points.x() + interval * angle.cos(),
                    current_upper_points.y() + interval * angle.sin(),
                );
                current_lower_points = Point2::new(
                    current_lower_points.x() - interval * angle.cos(),
                    current_lower_points.y() - interval * angle.sin(),
                );
                preprocessed_points.push(current_upper_points);
                preprocessed_points.push(current_lower_points);
            }
        }
        preprocessed_points.extend_from_slice(
            &pointcloud
                .clone()
                .points()
                .iter()
                .enumerate()
                .filter(|(i, _)| !remove_points_idxs.contains(i))
                .map(|(_, p)| *p)
                .collect::<Vec<Point2>>(),
        );

        Pointcloud2::new(preprocessed_points)
    }
}
