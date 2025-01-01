use crate::*;
use yaml_rust2::YamlLoader;

#[derive(Debug, Clone, PartialEq)]
pub struct LaserScan {
    header: Header,
    /// [m]
    range_max: f64,
    /// [m]
    range_min: f64,
    /// [rad]
    angle_increment: f64,
    /// [rad]
    angle_min: f64,
    /// [rad]
    angle_max: f64,
    /// [sec]
    time_increment: f64,
    /// [m]
    ranges: Vec<f64>,
}

impl LaserScan {
    pub fn new(
        range_max: f64,
        range_min: f64,
        angle_increment: f64,
        angle_min: f64,
        angle_max: f64,
        time_increment: f64,
        ranges: Vec<f64>,
    ) -> Self {
        Self {
            header: Header::new(Time::new(0, 0), "".to_string()),
            range_max,
            range_min,
            angle_increment,
            angle_min,
            angle_max,
            time_increment,
            ranges,
        }
    }

    #[allow(clippy::too_many_arguments)]
    pub fn new_with_header(
        header: Header,
        range_max: f64,
        range_min: f64,
        angle_increment: f64,
        angle_min: f64,
        angle_max: f64,
        time_increment: f64,
        ranges: Vec<f64>,
    ) -> Self {
        Self {
            header,
            range_max,
            range_min,
            angle_increment,
            angle_min,
            angle_max,
            time_increment,
            ranges,
        }
    }

    pub fn header(&self) -> &Header {
        &self.header
    }

    pub fn range_max(&self) -> f64 {
        self.range_max
    }

    pub fn range_min(&self) -> f64 {
        self.range_min
    }

    pub fn angle_increment(&self) -> f64 {
        self.angle_increment
    }

    pub fn angle_min(&self) -> f64 {
        self.angle_min
    }

    pub fn angle_max(&self) -> f64 {
        self.angle_max
    }

    pub fn time_increment(&self) -> f64 {
        self.time_increment
    }

    pub fn ranges(&self) -> &Vec<f64> {
        &self.ranges
    }
}

impl From<LaserScan> for Pointcloud2 {
    fn from(laser_scan: LaserScan) -> Self {
        let header = laser_scan.header;
        let mut points = Vec::new();
        for (i, range) in laser_scan.ranges.iter().enumerate() {
            if *range < laser_scan.range_min || *range > laser_scan.range_max {
                continue;
            }
            let angle = laser_scan.angle_min + (i as f64) * laser_scan.angle_increment;
            let x = angle.cos() * range;
            let y = angle.sin() * range;
            points.push(Point2::new(x, y));
        }
        Self::new_with_header(header, points)
    }
}

pub fn load_laser_scan_from_yaml(yaml: &str) -> Vec<LaserScan> {
    let docs = YamlLoader::load_from_str(yaml).unwrap();

    let mut laser_scans = Vec::new();

    for scan in docs {
        let header = Header::new(
            Time::new(
                scan["header"]["stamp"]["sec"].as_i64().unwrap() as u64,
                scan["header"]["stamp"]["nanosec"].as_i64().unwrap() as u64,
            ),
            scan["header"]["frame_id"].as_str().unwrap().to_string(),
        );
        let range_max = scan["range_max"].as_f64().unwrap();
        let range_min = scan["range_min"].as_f64().unwrap();
        let angle_increment = scan["angle_increment"].as_f64().unwrap();
        let angle_min = scan["angle_min"].as_f64().unwrap();
        let angle_max = scan["angle_max"].as_f64().unwrap();
        let time_increment = scan["time_increment"].as_f64().unwrap_or(0.0); // TODO: Do not unwrap_or(0.0). Handle this case properly
        let ranges = scan["ranges"]
            .as_vec()
            .unwrap()
            .iter()
            .map(|r| r.as_f64().unwrap())
            .collect();
        laser_scans.push(LaserScan::new_with_header(
            header,
            range_max,
            range_min,
            angle_increment,
            angle_min,
            angle_max,
            time_increment,
            ranges,
        ));
    }

    laser_scans
}

#[cfg(test)]
mod test {
    use super::*;

    #[test]
    fn test_load_laser_scan_from_yaml() {
        let file_name = "sample/ros2_scan_sample.yaml";
        let path = format!("{}/{}", env!("CARGO_MANIFEST_DIR"), file_name);

        let yaml = std::fs::read_to_string(path).unwrap();
        let laser_scans = load_laser_scan_from_yaml(&yaml);

        println!("{:?}", laser_scans[0]);
    }
}
