pub struct SingleRangingInfo {
    pub angle_deg: f64,
    pub distance: f64,
}

impl SingleRangingInfo {
    pub fn to_csv_raw_string(&self) -> String {
        format!("{:.4},{:.4}\n", self.angle_deg, self.distance)
    }
}

pub fn lidar_info_to_csv_format(lidar_info: &[SingleRangingInfo]) -> String {
    let mut csv_string = String::from("angle,distance\n");

    for lidar in lidar_info {
        csv_string.push_str(&lidar.to_csv_raw_string());
    }

    csv_string
}

#[cfg(test)]
mod test {
    use super::*;

    #[test]
    fn test_to_csv_raw_string() {
        let lidar_info = SingleRangingInfo {
            angle_deg: 1.0,
            distance: 2.0,
        };

        let csv_string = lidar_info.to_csv_raw_string();

        assert_eq!(csv_string, "1.0000,2.0000\n");
    }

    #[test]
    fn test_lidar_info_to_csv_format() {
        let lidar_info = vec![
            SingleRangingInfo {
                angle_deg: 1.0,
                distance: 2.0,
            },
            SingleRangingInfo {
                angle_deg: 3.0,
                distance: 4.0,
            },
        ];

        let csv_string = lidar_info_to_csv_format(&lidar_info);

        assert_eq!(csv_string, "angle,distance\n1.0000,2.0000\n3.0000,4.0000\n");
    }
}
