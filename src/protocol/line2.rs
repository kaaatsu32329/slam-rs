use crate::*;

const LINE_EQUAL_THRESHOLD: f64 = 1e-1;

/// Line parameters.
/// ax + by + c = 0
#[derive(Debug, Clone, PartialEq, Default)]
struct Param {
    pub a: f64,
    pub b: f64,
    pub c: f64,
}

#[derive(Debug, Clone, Default)]
pub struct Line2 {
    points: Vec<Point2>,
    param: Param,
    edge: (Point2, Point2),
}

impl Line2 {
    pub fn new(points: &[impl Into<Point2> + Clone]) -> Self {
        if points.len() != 2 {
            panic!("The number of points must be 2. TODO: support more points.");
        }
        let mut line = Line2::default();
        line.points = points.iter().map(|p| p.clone().into()).collect();
        line.param = Param {
            a: line.points[1].y() - line.points[0].y(),
            b: line.points[0].x() - line.points[1].x(),
            c: line.points[1].x() * line.points[0].y() - line.points[0].x() * line.points[1].y(),
        };
        line.edge = (line.points[0], line.points[1]);

        line
    }

    /// Get the parameters of the line.
    /// Return (a, b, c), ax + by + c = 0
    pub fn param(&self) -> (f64, f64, f64) {
        (self.param.a, self.param.b, self.param.c)
    }

    pub fn x(&self, y: f64) -> Option<f64> {
        if self.param.a.abs() < f64::EPSILON {
            Some(-(self.param.c / self.param.b))
        } else if self.param.b.abs() < f64::EPSILON {
            None
        } else {
            Some(-(self.param.b * y + self.param.c) / self.param.a)
        }
    }

    pub fn y(&self, x: f64) -> Option<f64> {
        if self.param.a.abs() < f64::EPSILON {
            None
        } else if self.param.b.abs() < f64::EPSILON {
            Some(-(self.param.c / self.param.a))
        } else {
            Some(-(self.param.a * x + self.param.c) / self.param.b)
        }
    }

    /// Get the edge of the line.
    /// Return (min_point, max_point)
    pub fn edge(&self) -> (Point2, Point2) {
        self.edge
    }

    pub fn distance(&self, points: &[impl Into<Point2> + Clone]) -> Vec<f64> {
        let mut results = Vec::new();

        for point in points {
            let point = point.clone().into();
            results.push(
                (self.param.a * point.x() + self.param.b * point.y() + self.param.c).abs()
                    / (self.param.a * self.param.a + self.param.b * self.param.b).sqrt(),
            );
        }

        results
    }

    pub fn add_points(&mut self, points: &[impl Into<Point2> + Clone]) {
        for point in points {
            self.points.push(point.clone().into());
        }

        self.update_param();
        self.update_segment_edge();
    }

    pub fn closest_point(&self, points: &[impl Into<Point2> + Clone]) -> Vec<Point2> {
        let mut results = Vec::new();

        if self.param.a.abs() < f64::EPSILON {
            let y = -self.param.c / self.param.b;
            points
                .iter()
                .for_each(|p| results.push(Point2::new(p.clone().into().x(), y)));
        } else if self.param.b.abs() < f64::EPSILON {
            let x = -self.param.c / self.param.a;
            points
                .iter()
                .for_each(|p| results.push(Point2::new(x, p.clone().into().y())));
        } else {
            points
                .iter()
                .for_each(|p| results.push(self.get_closest_point(&p.clone().into())));
        }

        results
    }

    pub fn split_lines(&self, threshold: f64) -> Vec<Vec<Point2>> {
        let sorted_points = {
            let mut points = self
                .closest_point(&self.points)
                .clone()
                .iter()
                .enumerate()
                .map(|(idx, &point)| (idx, point))
                .collect::<Vec<_>>();
            if self.param.b.abs() < f64::EPSILON {
                points.sort_by(|(_, a), (_, b)| {
                    let closest_a = self.get_closest_point(a);
                    let closest_b = self.get_closest_point(b);
                    closest_a.y().partial_cmp(&closest_b.y()).unwrap()
                });
            } else {
                points.sort_by(|(_, a), (_, b)| {
                    let closest_a = self.get_closest_point(a);
                    let closest_b = self.get_closest_point(b);
                    closest_a.x().partial_cmp(&closest_b.x()).unwrap()
                });
            }
            points
        };

        let mut point_groups = vec![vec![]];
        for w in sorted_points.windows(2) {
            let (idx1, p1) = w[0];
            let (_, p2) = w[1];
            let distance = p1.distance(&p2);
            point_groups.last_mut().unwrap().push(idx1);
            if distance > threshold {
                point_groups.push(vec![]);
            }
        }
        point_groups
            .last_mut()
            .unwrap()
            .push(sorted_points.last().unwrap().0);

        let mut splited_points = vec![];
        for group in point_groups {
            let mut points = vec![];
            for idx in group {
                points.push(self.points[idx]);
            }
            splited_points.push(points);
        }

        splited_points
    }

    pub fn points_len(&self) -> usize {
        self.points.len()
    }

    fn update_param(&mut self) {
        let num = self.points.len() as f64;
        let sum_x = self.points.iter().map(|p| p.x()).sum::<f64>();
        let sum_y = self.points.iter().map(|p| p.y()).sum::<f64>();
        let sum_x2 = self.points.iter().map(|p| p.x() * p.x()).sum::<f64>();
        let sum_xy = self.points.iter().map(|p| p.x() * p.y()).sum::<f64>();
        let sum_y2 = self.points.iter().map(|p| p.y() * p.y()).sum::<f64>();

        self.param = if (num * sum_x2 - sum_x * sum_x).abs() < f64::EPSILON {
            // x = py + q
            let p = (num * sum_xy - sum_x * sum_y) / (num * sum_y2 - sum_y * sum_y);
            let q = (sum_y2 * sum_x - sum_y * sum_xy) / (num * sum_y2 - sum_y * sum_y);
            Param {
                a: -1.0,
                b: p,
                c: q,
            }
        } else {
            // y = px + q
            let p = (num * sum_xy - sum_x * sum_y) / (num * sum_x2 - sum_x * sum_x);
            let q = (sum_x2 * sum_y - sum_x * sum_xy) / (num * sum_x2 - sum_x * sum_x);
            Param {
                a: p,
                b: -1.0,
                c: q,
            }
        };
    }

    fn update_segment_edge(&mut self) {
        let mut min_point = Point2::new(f64::MAX, f64::MAX);
        let mut max_point = Point2::new(f64::MIN, f64::MIN);
        if self.param.a.abs() < f64::EPSILON {
            let y = -self.param.c / self.param.b;
            for p in self.points.iter() {
                if p.y() < min_point.y() {
                    min_point = Point2::new(p.x(), y);
                }
                if p.y() > max_point.y() {
                    max_point = Point2::new(p.x(), y);
                }
            }
        } else if self.param.b.abs() < f64::EPSILON {
            let x = -self.param.c / self.param.a;
            for p in self.points.iter() {
                if p.x() < min_point.x() {
                    min_point = Point2::new(x, p.y());
                }
                if p.x() > max_point.x() {
                    max_point = Point2::new(x, p.y());
                }
            }
        } else {
            for p in self.points.iter() {
                let closest_point = self.get_closest_point(p);
                if closest_point.x() < min_point.x() {
                    min_point = closest_point;
                }
                if closest_point.x() > max_point.x() {
                    max_point = closest_point;
                }
            }
        }

        self.edge = (min_point, max_point);
    }

    fn get_closest_point(&self, point: &Point2) -> Point2 {
        let denominator = self.param.a * self.param.a + self.param.b * self.param.b;
        let ab = self.param.a * self.param.b;
        let ac = self.param.a * self.param.c;
        let bc = self.param.b * self.param.c;
        let a2 = self.param.a * self.param.a;
        let b2 = self.param.b * self.param.b;
        let x = (-ab * point.y() + b2 * point.x() - ac) / denominator;
        let y = (-ab * point.x() + a2 * point.y() - bc) / denominator;

        Point2::new(x, y)
    }
}

impl PartialEq for Line2 {
    fn eq(&self, other: &Self) -> bool {
        let self_theta =
            ((-self.param.a).atan2(self.param.b) + std::f64::consts::TAU) % std::f64::consts::PI;
        let other_theta =
            ((-other.param.a).atan2(other.param.b) + std::f64::consts::TAU) % std::f64::consts::PI;

        (self_theta - other_theta).abs() < LINE_EQUAL_THRESHOLD
            && (self.param.c - other.param.c).abs() < LINE_EQUAL_THRESHOLD
    }
}

#[cfg(test)]
mod test {
    use super::*;
    use assert_approx_eq::assert_approx_eq;

    #[test]
    fn test_line2() {
        let mut line = data_gen();

        assert_eq!(line.param(), (1.0, -1.0, 1.0));

        let new_points = vec![Point2::new(2.0, 3.0), Point2::new(3.0, 4.0)];
        line.add_points(&new_points);

        assert_eq!(line.param(), (1.0, -1.0, 1.0));
    }

    #[test]
    fn test_distance() {
        let line = data_gen();

        let target_points = vec![Point2::new(1.0, 1.0), Point2::new(2.0, 2.0)];

        let results = line.distance(&target_points);

        assert_approx_eq!(results[0], std::f64::consts::FRAC_1_SQRT_2);
        assert_approx_eq!(results[1], std::f64::consts::FRAC_1_SQRT_2);
    }

    #[test]
    fn test_edge() {
        let mut line = data_gen();
        let added_points = added_points();
        line.add_points(&added_points);

        let (min_point, max_point) = line.edge();

        assert_eq!(min_point, Point2::new(0.0, 1.0));
        assert_eq!(max_point, Point2::new(22.0, 23.0));
    }

    #[test]
    fn test_split_lines() {
        let mut line = data_gen();
        let added_points = added_points();
        line.add_points(&added_points);

        let results = line.split_lines(3.0);

        assert_eq!(results[0].len(), 5);
        assert_eq!(results[1].len(), 3);
        assert_eq!(results[2].len(), 3);
    }

    fn data_gen() -> Line2 {
        let points = vec![Point2::new(0.0, 1.0), Point2::new(1.0, 2.0)];

        Line2::new(&points)
    }

    fn added_points() -> Vec<Point2> {
        vec![
            Point2::new(2.0, 3.0),
            Point2::new(3.0, 4.0),
            Point2::new(4.0, 5.0),
            Point2::new(10.0, 11.0),
            Point2::new(11.0, 12.0),
            Point2::new(12.0, 13.0),
            Point2::new(20.0, 21.0),
            Point2::new(21.0, 22.0),
            Point2::new(22.0, 23.0),
        ]
    }
}
