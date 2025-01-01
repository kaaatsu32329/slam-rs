use crate::*;

pub struct SlamRunner {
    mapping: Mapping,
    icp_client: IterativeClosestPoint2,
    latest_pointcloud: Option<Pointcloud2>,
}

impl SlamRunner {
    pub fn new(mapping: Mapping, init_pose: Pose2) -> Self {
        Self {
            mapping,
            icp_client: IterativeClosestPoint2::new(&init_pose),
            latest_pointcloud: None,
        }
    }

    pub fn update(&mut self, laser_scan: &LaserScan, odom: &Odometry) {
        let latest_pointcloud = self
            .latest_pointcloud
            .clone()
            .unwrap_or(laser_scan.clone().into());

        self.icp_client
            .set_data(&laser_scan.clone().into(), &latest_pointcloud);
        let robot_pose = self.icp_client.scan_matching(10);

        self.mapping.update(&robot_pose, laser_scan);

        self.latest_pointcloud = Some(laser_scan.clone().into());
    }

    pub fn mapping(&self) -> &Mapping {
        &self.mapping
    }

    pub fn robot_pose(&self) -> Pose2 {
        self.icp_client.robot_pose()
    }
}
