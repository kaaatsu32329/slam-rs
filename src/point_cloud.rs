use crate::Error;

#[derive(Debug, Clone, Copy, Default)]
pub struct Point2d {
    pub x: f64,
    pub y: f64,
}

#[derive(Debug, Clone, Default)]
pub struct PointCloud {
    pub center: Point2d,
    pub points: Vec<Point2d>,
}

impl PointCloud {
    pub fn new(center: Point2d, points: Vec<Point2d>) -> Self {
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
            points.push(Point2d { x, y });
        }
        let center = Point2d { x: 0., y: 0. };

        Ok(Self { center, points })
    }

    pub fn points(&self) -> &Vec<Point2d> {
        &self.points
    }
}
