use crate::*;
use nalgebra as na;
use yaml_rust2::YamlLoader;

#[derive(Debug, Clone, PartialEq)]
pub struct Odometry {
    header: Header,
    pose: na::Isometry3<f64>,
    // pose_covariance: [f64; 36],
    twist: na::Vector6<f64>,
    // twist_covariance: [f64; 36],
}

impl Odometry {
    pub fn new(
        pose: na::Isometry3<f64>,
        // pose_covariance: [f64; 36],
        twist: na::Vector6<f64>,
        // twist_covariance: [f64; 36],
    ) -> Self {
        Self {
            header: Header::new(Time::new(0, 0), "".to_string()),
            pose,
            // pose_covariance,
            twist,
            // twist_covariance,
        }
    }

    pub fn new_with_header(
        header: Header,
        pose: na::Isometry3<f64>,
        // pose_covariance: [f64; 36],
        twist: na::Vector6<f64>,
        // twist_covariance: [f64; 36],
    ) -> Self {
        Self {
            header,
            pose,
            // pose_covariance,
            twist,
            // twist_covariance,
        }
    }

    pub fn header(&self) -> &Header {
        &self.header
    }

    pub fn pose(&self) -> &na::Isometry3<f64> {
        &self.pose
    }

    // pub fn pose_covariance(&self) -> &[f64; 36] {
    //     &self.pose_covariance
    // }

    pub fn twist(&self) -> &na::Vector6<f64> {
        &self.twist
    }

    // pub fn twist_covariance(&self) -> &[f64; 36] {
    //     &self.twist_covariance
    // }

    pub fn linear_interpolation(&self, other: &Self, time: f64) -> Self {
        let time0 = self.header().stamp().sec_as_f64();
        let time1 = other.header().stamp().sec_as_f64();

        let ratio = (time - time0) / (time1 - time0);

        let pose = na::Isometry3::from_parts(
            na::Translation3::from(
                self.pose().translation.vector
                    + (other.pose().translation.vector - self.pose().translation.vector) * ratio,
            ),
            self.pose().rotation.slerp(&other.pose().rotation, ratio),
        );

        let twist = na::Vector6::new(
            self.twist().x + (other.twist().x - self.twist().x) * ratio,
            self.twist().y + (other.twist().y - self.twist().y) * ratio,
            self.twist().z + (other.twist().z - self.twist().z) * ratio,
            self.twist().w + (other.twist().w - self.twist().w) * ratio,
            self.twist().a + (other.twist().a - self.twist().a) * ratio,
            self.twist().b + (other.twist().b - self.twist().b) * ratio,
        );

        Odometry::new_with_header(
            Header::new(
                Time::new(
                    (time0 + (time1 - time0) * ratio) as u64,
                    ((time0 + (time1 - time0) * ratio) % 1.0 * 1e9) as u64,
                ),
                self.header().frame_id().to_string(),
            ),
            pose,
            twist,
        )
    }
}

pub fn load_odometry_from_yaml(yaml: &str) -> Vec<Odometry> {
    let docs = YamlLoader::load_from_str(yaml).unwrap();

    let mut odometries = Vec::new();

    for odom in docs {
        let header = Header::new(
            Time::new(
                odom["header"]["stamp"]["sec"].as_i64().unwrap() as u64,
                odom["header"]["stamp"]["nanosec"].as_i64().unwrap() as u64,
            ),
            odom["header"]["frame_id"].as_str().unwrap().to_string(),
        );
        let pose = na::Isometry3::from_parts(
            na::Translation3::new(
                odom["pose"]["pose"]["position"]["x"].as_f64().unwrap(),
                odom["pose"]["pose"]["position"]["y"].as_f64().unwrap(),
                odom["pose"]["pose"]["position"]["z"].as_f64().unwrap(),
            ),
            na::UnitQuaternion::from_quaternion(na::Quaternion::new(
                odom["pose"]["pose"]["orientation"]["w"].as_f64().unwrap(),
                odom["pose"]["pose"]["orientation"]["x"].as_f64().unwrap(),
                odom["pose"]["pose"]["orientation"]["y"].as_f64().unwrap(),
                odom["pose"]["pose"]["orientation"]["z"].as_f64().unwrap(),
            )),
        );
        let twist = na::Vector6::new(
            odom["twist"]["twist"]["linear"]["x"].as_f64().unwrap(),
            odom["twist"]["twist"]["linear"]["y"].as_f64().unwrap(),
            odom["twist"]["twist"]["linear"]["z"]
                .as_f64()
                .unwrap_or(0.0), // TODO: Do not unwrap_or(0.0). Handle this case properly
            odom["twist"]["twist"]["angular"]["x"]
                .as_f64()
                .unwrap_or(0.0),
            odom["twist"]["twist"]["angular"]["y"]
                .as_f64()
                .unwrap_or(0.0),
            odom["twist"]["twist"]["angular"]["z"].as_f64().unwrap(),
        );
        odometries.push(Odometry::new_with_header(header, pose, twist));
    }

    odometries
}

#[cfg(test)]
mod test {
    use super::*;
    use assert_approx_eq::assert_approx_eq;

    #[test]
    fn test_load_odometry_from_yaml() {
        let file_name = "sample/ros2_odom_sample.yaml";
        let path = format!("{}/{}", env!("CARGO_MANIFEST_DIR"), file_name);

        let yaml = std::fs::read_to_string(path).unwrap();
        let odometries = load_odometry_from_yaml(&yaml);

        println!("{:?}", odometries[0]);
    }

    #[test]
    fn test_linear_interpolation() {
        let odom0 = Odometry::new_with_header(
            Header::new(Time::new(0, 0), "".to_string()),
            na::Isometry3::new(na::Vector3::new(0.0, 0.0, 0.0), na::zero()),
            na::Vector6::new(0.0, 0.0, 0.0, 0.0, 0.0, 0.0),
        );
        let odom1 = Odometry::new_with_header(
            Header::new(Time::new(1, 0), "".to_string()),
            na::Isometry3::new(na::Vector3::new(1.0, 1.0, 1.0), na::zero()),
            na::Vector6::new(1.0, 1.0, 1.0, 1.0, 1.0, 1.0),
        );

        let odom = odom0.linear_interpolation(&odom1, 0.5);

        assert_approx_eq!(odom.header().stamp().sec_as_f64(), 0.5);
        assert_approx_eq!(odom.pose().translation.vector.x, 0.5);
        assert_approx_eq!(odom.pose().translation.vector.y, 0.5);
        assert_approx_eq!(odom.pose().translation.vector.z, 0.5);
        assert_approx_eq!(odom.twist().x, 0.5);
        assert_approx_eq!(odom.twist().y, 0.5);
        assert_approx_eq!(odom.twist().z, 0.5);
        assert_approx_eq!(odom.twist().w, 0.5);
        assert_approx_eq!(odom.twist().a, 0.5);
        assert_approx_eq!(odom.twist().b, 0.5);
    }
}
