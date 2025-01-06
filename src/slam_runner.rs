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

        self.icp_client
            .set_data(&laser_scan.clone(), &latest_pointcloud);
        let robot_pose_with_icp = self.icp_client.scan_matching(10);

        let aaa = 0.99;
        let bbb = 0.01;

        self.robot_pose = Pose2::new(
            aaa * odom.pose().translation.x + bbb * robot_pose_with_icp.x(),
            aaa * odom.pose().translation.y + bbb * robot_pose_with_icp.y(),
            aaa * odom.pose().rotation.angle() + bbb * robot_pose_with_icp.theta(),
        );

        self.mapping.update(&self.robot_pose, laser_scan);

        self.latest_pointcloud = Some(laser_scan.clone().into());
    }

    pub fn mapping(&self) -> &Mapping {
        &self.mapping
    }

    pub fn robot_pose(&self) -> Pose2 {
        self.robot_pose
    }

    fn pointcloud_pre_processing(pointcloud: &Pointcloud2) -> Pointcloud2 {
        let lines = ransac_algorithm(pointcloud, 100, 0.1);

        todo!()
    }
}
