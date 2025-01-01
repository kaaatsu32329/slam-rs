pub trait Point {
    fn distance(&self, other: &Self) -> f64;
    fn distance_squared(&self, other: &Self) -> f64;
}
