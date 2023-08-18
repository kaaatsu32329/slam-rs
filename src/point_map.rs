use crate::Point2d;

#[derive(Debug, Clone, Default)]
pub struct PointMap {
    points: Vec<Vec<Point2d>>,
}

impl PointMap {
    pub fn new() -> Self {
        Self { points: Vec::new() }
    }

    pub fn points(&self) -> &Vec<Vec<Point2d>> {
        &self.points
    }

    pub fn add_points(&mut self, points: Vec<Point2d>) {
        self.points.push(points);
    }
}
