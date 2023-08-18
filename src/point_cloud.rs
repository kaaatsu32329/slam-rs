use crate::{Error, Pose2d};
use na::Vector2;
use nalgebra as na;

pub type Point2d = na::Point2<f64>;

#[derive(Debug, Clone, Default)]
pub struct PointCloud {
    center: Pose2d,
    points: Vec<Point2d>,
}

impl PointCloud {
    pub fn new(center: Pose2d, points: Vec<Point2d>) -> Self {
        Self { center, points }
    }

    pub fn new_from_csv(path: &str) -> Result<Self, Error> {
        let mut rdr = match csv::Reader::from_path(path) {
            Ok(rdr) => rdr,
            Err(_) => return Err(Error::CsvError),
        };
        let mut points = Vec::new();

        for result in rdr.records() {
            let record = result.unwrap();
            let x = record[0].parse::<f64>().unwrap().cos() * record[1].parse::<f64>().unwrap();
            let y = record[0].parse::<f64>().unwrap().sin() * record[1].parse::<f64>().unwrap();
            points.push(Point2d {
                coords: Vector2::new(x, y),
            });
        }
        let center = Pose2d::default();

        Ok(Self { center, points })
    }

    pub fn center(&self) -> &Pose2d {
        &self.center
    }

    pub fn center_mut(&mut self) -> &mut Pose2d {
        &mut self.center
    }

    pub fn points(&self) -> &Vec<Point2d> {
        &self.points
    }

    pub fn update_points(&mut self, points: Vec<Point2d>) {
        self.points = points;
    }
}
