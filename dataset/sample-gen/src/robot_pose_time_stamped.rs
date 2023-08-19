use crate::linear_info::*;

#[derive(Debug, Clone, Copy)]
pub struct RobotPoseTimeStamped {
    pub x: f64,
    pub y: f64,
    pub theta: f64,
    pub time_stamp: f64,
}

impl RobotPoseTimeStamped {
    pub fn get_distance_to_point(&self, x: f64, y: f64) -> f64 {
        ((self.x - x).powi(2) + (self.y - y).powi(2)).sqrt()
    }

    pub fn to_linear_info_from_angle(&self, angle: f64) -> LinearInfo {
        let total_angle = angle + self.theta;

        let is_x_expand_direction_positive = total_angle.to_radians().cos().is_sign_positive();
        let is_y_expand_direction_positive = total_angle.to_radians().sin().is_sign_positive();

        LinearInfo {
            a: total_angle.to_radians().tan(),
            b: -1.0,
            c: self.y - total_angle.to_radians().tan() * self.x,
            x_min: if is_x_expand_direction_positive {
                Some(self.x)
            } else {
                None
            },
            x_max: if !is_x_expand_direction_positive {
                Some(self.x)
            } else {
                None
            },
            y_min: if is_y_expand_direction_positive {
                Some(self.y)
            } else {
                None
            },
            y_max: if !is_y_expand_direction_positive {
                Some(self.y)
            } else {
                None
            },
        }
    }
}

#[cfg(test)]
mod test {
    use super::*;

    use assert_approx_eq::assert_approx_eq;

    #[test]
    fn test_get_distance_to_point() {
        let robot_pose = RobotPoseTimeStamped {
            x: 1.0,
            y: 2.0,
            theta: 3.0,
            time_stamp: 4.0,
        };
        let point = (5.0, 6.0);

        let distance = robot_pose.get_distance_to_point(point.0, point.1);

        assert_approx_eq!(distance, 5.656854249);
    }
}
