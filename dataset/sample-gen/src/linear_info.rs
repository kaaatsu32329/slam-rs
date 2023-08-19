#[derive(Debug, Clone, Copy)]
pub struct LinearInfo {
    pub a: f64,
    pub b: f64,
    pub c: f64,
    pub x_min: Option<f64>,
    pub x_max: Option<f64>,
    pub y_min: Option<f64>,
    pub y_max: Option<f64>,
}

impl LinearInfo {
    pub fn get_distance_to_point(&self, x: f64, y: f64) -> f64 {
        (self.a * x + self.b * y + self.c).abs() / (self.a.powi(2) + self.b.powi(2)).sqrt()
    }

    pub fn get_intersection(&self, other: &LinearInfo) -> Option<(f64, f64)> {
        if (self.a * other.b - other.a * self.b).abs() < 1e-10 {
            None
        } else {
            let x = (self.b * other.c - other.b * self.c) / (self.a * other.b - other.a * self.b);
            let y = (other.a * self.c - self.a * other.c) / (self.a * other.b - other.a * self.b);

            let x_min = self
                .x_min
                .unwrap_or(std::f64::MIN)
                .max(other.x_min.unwrap_or(std::f64::MIN));
            let x_max = self
                .x_max
                .unwrap_or(std::f64::MAX)
                .min(other.x_max.unwrap_or(std::f64::MAX));
            let y_min = self
                .y_min
                .unwrap_or(std::f64::MIN)
                .max(other.y_min.unwrap_or(std::f64::MIN));
            let y_max = self
                .y_max
                .unwrap_or(std::f64::MAX)
                .min(other.y_max.unwrap_or(std::f64::MAX));

            if x_min <= x && x <= x_max && y_min <= y && y <= y_max {
                Some((x, y))
            } else {
                None
            }
        }
    }
}

#[cfg(test)]
mod test {
    use super::*;

    use assert_approx_eq::assert_approx_eq;

    #[test]
    fn test_get_distance_to_point() {
        let point = (1.0, 2.0);
        let linear_info = LinearInfo {
            a: 3.0,
            b: 4.0,
            c: 5.0,
            x_min: None,
            x_max: None,
            y_min: None,
            y_max: None,
        };

        let distance = linear_info.get_distance_to_point(point.0, point.1);

        assert_approx_eq!(distance, 3.2);
    }

    #[test]
    fn test_get_intersection() {
        let linear_info_1 = LinearInfo {
            a: 1.0,
            b: 2.0,
            c: 3.0,
            x_min: None,
            x_max: None,
            y_min: None,
            y_max: None,
        };
        let linear_info_2 = LinearInfo {
            a: 4.0,
            b: 5.0,
            c: 6.0,
            x_min: None,
            x_max: None,
            y_min: None,
            y_max: None,
        };

        let intersect_point = linear_info_1.get_intersection(&linear_info_2);

        assert_approx_eq!(intersect_point.unwrap().0, 1.0);
        assert_approx_eq!(intersect_point.unwrap().1, -2.0);
    }
}
