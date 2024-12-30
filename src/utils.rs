use grid_map::{Grid, Position};
use nalgebra as na;

pub fn coordinate_transformation(
    current_position: &(impl Into<na::Isometry2<f64>> + Clone),
    target_points: &[(impl Into<na::Translation2<f64>> + Clone)],
) -> Vec<na::Translation2<f64>> {
    let mut points = Vec::new();
    for target_point in target_points {
        let c: na::Isometry2<f64> = current_position.clone().into();
        let p: na::Translation2<f64> = target_point.clone().into();
        let isometry = c * p;
        points.push(isometry.translation);
    }

    points
}

// TODO: Update to derive a grid that accurately traces the line segment between two points.
pub fn bresenham_algorithm(
    start: &na::Translation2<f64>,
    end: &na::Translation2<f64>,
    resolution: f64,
    min_point: &Position,
) -> Vec<Grid> {
    let mut idx_x0 = ((start.x - min_point.x) / resolution).round() as i32;
    let mut idx_y0 = ((start.y - min_point.y) / resolution).round() as i32;
    let idx_x1 = ((end.x - min_point.x) / resolution).round() as i32;
    let idx_y1 = ((end.y - min_point.y) / resolution).round() as i32;

    let mut grids = Vec::new();
    grids.push(Grid::new(idx_x0 as usize, idx_y0 as usize));

    let delta_x = (idx_x1 - idx_x0).abs();
    let delta_y = (idx_y1 - idx_y0).abs();

    let step_x = if idx_x0 < idx_x1 { 1 } else { -1 };
    let step_y = if idx_y0 < idx_y1 { 1 } else { -1 };

    let mut error = delta_x - delta_y;

    while idx_x0 != idx_x1 || idx_y0 != idx_y1 {
        let error_x2 = 2 * error;
        if error_x2 > -delta_y {
            error -= delta_y;
            idx_x0 += step_x;
        }
        if error_x2 < delta_x {
            error += delta_x;
            idx_y0 += step_y;
        }
        grids.push(Grid::new(idx_x0 as usize, idx_y0 as usize));
    }

    grids
}

pub fn linear_interpolation(
    time0: f64,
    value0: f64,
    time1: f64,
    value1: f64,
    current_time: f64,
) -> f64 {
    let slope = (value1 - value0) / (time1 - time0);
    slope * (current_time - time0) + value0
}

#[cfg(test)]
mod test {
    use super::*;
    use assert_approx_eq::assert_approx_eq;

    #[test]
    fn test_linear_interpolation() {
        let time0 = 0.0;
        let value0 = 0.0;
        let time1 = 1.0;
        let value1 = 1.0;

        let current_time = 0.5;
        let future_time = 2.0;

        let curret_value = linear_interpolation(time0, value0, time1, value1, current_time);
        let future_value = linear_interpolation(time0, value0, time1, value1, future_time);

        assert_approx_eq!(curret_value, 0.5);
        assert_approx_eq!(future_value, 2.0);
    }
}
