use crate::*;

#[derive(Debug, Clone, PartialEq)]
pub struct Pointcloud2 {
    header: Header,
    /// [m]
    points: Vec<Point2>,
}

impl Pointcloud2 {
    pub fn new(points: Vec<Point2>) -> Self {
        Self {
            header: Header::new(Time::new(0, 0), "".to_string()),
            points,
        }
    }

    pub fn new_with_header(header: Header, points: Vec<Point2>) -> Self {
        Self { header, points }
    }

    pub fn header(&self) -> &Header {
        &self.header
    }

    pub fn points(&self) -> &Vec<Point2> {
        &self.points
    }

    pub fn set_points(&mut self, points: Vec<Point2>) {
        self.points = points;
    }
}

#[derive(Debug, Clone, PartialEq)]
pub struct Pointcloud3 {
    header: Header,
    /// [m]
    points: Vec<Point3>,
}

impl Pointcloud3 {
    pub fn new(points: Vec<Point3>) -> Self {
        Self {
            header: Header::new(Time::new(0, 0), "".to_string()),
            points,
        }
    }

    pub fn new_with_header(header: Header, points: Vec<Point3>) -> Self {
        Self { header, points }
    }

    pub fn header(&self) -> &Header {
        &self.header
    }

    pub fn points(&self) -> &Vec<Point3> {
        &self.points
    }

    pub fn set_points(&mut self, points: Vec<Point3>) {
        self.points = points;
    }
}
