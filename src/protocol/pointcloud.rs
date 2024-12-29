use crate::*;

#[derive(Debug, Clone, PartialEq)]
pub struct Pointcloud2 {
    header: Header,
    /// [m]
    points: Vec<Point>,
}

impl Pointcloud2 {
    pub fn new(points: Vec<Point>) -> Self {
        Self {
            header: Header::new(Time::new(0, 0), "".to_string()),
            points,
        }
    }

    pub fn new_with_header(header: Header, points: Vec<Point>) -> Self {
        Self { header, points }
    }

    pub fn header(&self) -> &Header {
        &self.header
    }

    pub fn points(&self) -> &Vec<Point> {
        &self.points
    }

    pub fn set_points(&mut self, points: Vec<Point>) {
        self.points = points;
    }
}
