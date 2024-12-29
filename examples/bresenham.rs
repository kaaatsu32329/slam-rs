use grid_map::{GridMap, Position};
use nalgebra as na;
use slam::*;

fn main() {
    let min_point = Position::new(-10.0, -10.0);
    let max_point = Position::new(15.0, 15.0);
    let resolution = 0.5;
    let grid_map: GridMap<f64> = GridMap::new(min_point, max_point, resolution);

    let start = na::Translation2::new(2.7, 3.2);
    let end = na::Translation2::new(10.9, 7.4);

    let path_grids = bresenham_algorithm(&start, &end, grid_map.resolution(), grid_map.min_point());

    for grid in path_grids {
        println!("{:?}", grid);
    }
}
