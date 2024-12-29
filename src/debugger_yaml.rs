// TODO: Remove or move to tests
use crate::*;
use nalgebra as na;

pub struct DebuggerYaml {
    laser_scan: Vec<LaserScan>,
    odometry: Vec<Odometry>,
    counter: usize,
    odom_counter: usize,
}

impl DebuggerYaml {
    pub fn new(scan_log_path: &str, odom_log_path: &str) -> Self {
        let scan_yaml = std::fs::read_to_string(scan_log_path).unwrap();
        let odom_yaml = std::fs::read_to_string(odom_log_path).unwrap();

        let laser_scan = load_laser_scan_from_yaml(&scan_yaml);
        let odometry = load_odometry_from_yaml(&odom_yaml);

        Self {
            laser_scan,
            odometry,
            counter: 0,
            odom_counter: 0,
        }
    }

    pub fn reset_count(&mut self) {
        self.counter = 0;
        self.odom_counter = 0;
    }

    pub fn next_scan_2d(&mut self) -> Option<(LaserScan, na::Isometry2<f64>)> {
        if self.counter >= self.laser_scan.len() {
            return None;
        }

        let scan = self.laser_scan[self.counter].clone();
        let scan_time = scan.header().stamp().sec_as_f64();

        let (idx, odom) = self
            .odometry
            .iter()
            .skip(self.odom_counter)
            .enumerate()
            .find(|(_, odom)| odom.header().stamp().sec_as_f64() >= scan_time)
            .unwrap();
        let odom = odom.clone();
        if idx + 1 >= self.odometry.len() {
            return None;
        }
        let next_odom = self.odometry[idx + 1].clone();

        let current_odom = odom.linear_interpolation(&next_odom, scan_time);

        let angle = current_odom.pose().rotation.angle();
        let x = current_odom.pose().translation.vector.x;
        let y = current_odom.pose().translation.vector.y;
        let current_position = na::Isometry2::new(na::Vector2::new(x, y), angle);

        self.counter += 1;
        self.odom_counter = idx;

        Some((scan, current_position))
    }
}
