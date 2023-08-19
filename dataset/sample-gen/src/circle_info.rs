use crate::linear_info::*;

#[derive(Debug, Clone, Copy)]
pub struct CircleInfo {
    pub x: f64,
    pub y: f64,
    pub radius: f64,
    pub theta_min: f64,
    pub theta_max: f64,
}

impl CircleInfo {
    pub fn get_distance_to_point(&self, x: f64, y: f64) -> f64 {
        (((self.x - x).powi(2) + (self.y - y).powi(2)).sqrt() - self.radius).abs()
    }

    pub fn get_intersection(&self, linear: &LinearInfo) -> Option<Vec<(f64, f64)>> {
        let distance_to_center = linear.get_distance_to_point(self.x, self.y);

        // TODO: Implement range check for theta_min and theta_max

        if (distance_to_center - self.radius).is_sign_positive() {
            None
        } else if (distance_to_center - self.radius).abs() < 1e-10 {
            let norm = (linear.a.powi(2) + linear.b.powi(2)).sqrt();

            let x = linear.a * self.radius / norm + self.x;
            let y = linear.b * self.radius / norm + self.y;

            Some(vec![(x, y)])
        } else {
            let d = (linear.a * self.x + linear.b * self.y + linear.c).abs();
            let square_norm = linear.a.powi(2) + linear.b.powi(2);
            let x_a = (linear.a * d
                - linear.b * (square_norm * self.radius.powi(2) - d.powi(2)).sqrt())
                / square_norm
                + self.x;
            let y_a = (linear.b * d
                + linear.a * (square_norm * self.radius.powi(2) - d.powi(2)).sqrt())
                / square_norm
                + self.y;
            let x_b = (linear.a * d
                + linear.b * (square_norm * self.radius.powi(2) - d.powi(2)).sqrt())
                / square_norm
                + self.x;
            let y_b = (linear.b * d
                - linear.a * (square_norm * self.radius.powi(2) - d.powi(2)).sqrt())
                / square_norm
                + self.y;
            Some(vec![(x_a, y_a), (x_b, y_b)])
        }
    }
}

#[cfg(test)]
mod test {
    use std::f64::consts::SQRT_2;

    use super::*;

    use assert_approx_eq::assert_approx_eq;

    #[test]
    fn test_get_distance_to_point() {
        let point1 = (1.0, 2.0);
        let point2 = (-2.0, 9.0);
        let circle_info = CircleInfo {
            x: 3.0,
            y: 4.0,
            radius: 5.0,
            theta_min: 0.0,
            theta_max: 360.0,
        };

        let distance1 = circle_info.get_distance_to_point(point1.0, point1.1);
        let distance2 = circle_info.get_distance_to_point(point2.0, point2.1);

        assert_approx_eq!(distance1, 5. - 2. * SQRT_2);
        assert_approx_eq!(distance2, 5. * SQRT_2 - 5.);
    }

    #[test]
    fn test_get_intersection() {
        let linear_info1 = LinearInfo {
            a: 1.0,
            b: 2.0,
            c: 3.0,
            x_min: None,
            x_max: None,
            y_min: None,
            y_max: None,
        };
        let linear_info2 = LinearInfo {
            a: 1.0,
            b: -2.0,
            c: 3.0,
            x_min: None,
            x_max: None,
            y_min: None,
            y_max: None,
        };
        let circle_info = CircleInfo {
            x: 4.0,
            y: 5.0,
            radius: 6.0,
            theta_min: 0.0,
            theta_max: 360.0,
        };

        let intersect1 = circle_info.get_intersection(&linear_info1);
        let intersect2 = circle_info.get_intersection(&linear_info2).unwrap();

        let ideal_intersect2 = vec![(-0.6306787, 1.1846606), (9.8306787, 6.4153393)];

        assert!(intersect1.is_none());

        assert!(
            (intersect2[0].0 - ideal_intersect2[0].0).abs() < 1e-6
                || (intersect2[0].0 - ideal_intersect2[1].0).abs() < 1e-6
        );
        assert!(
            (intersect2[0].1 - ideal_intersect2[0].1).abs() < 1e-6
                || (intersect2[0].1 - ideal_intersect2[1].1).abs() < 1e-6
        );
        assert!(
            (intersect2[1].0 - ideal_intersect2[0].0).abs() < 1e-6
                || (intersect2[1].0 - ideal_intersect2[1].0).abs() < 1e-6
        );
        assert!(
            (intersect2[1].1 - ideal_intersect2[0].1).abs() < 1e-6
                || (intersect2[1].1 - ideal_intersect2[1].1).abs() < 1e-6
        );
    }
}
