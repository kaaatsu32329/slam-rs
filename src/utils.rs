use crate::*;
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

/// Detects the best line that fits the given points using the RANSAC algorithm.
/// Actually, some parts of the algorithm differ slightly from the actual algorithm.
pub fn ransac_algorithm(
    pointcloud: &(impl Into<Pointcloud2> + Clone),
    max_iters: usize,
    threshold_distance: f64,
) -> Vec<(Vec<usize>, Line2)> {
    let pointcloud: Pointcloud2 = pointcloud.clone().into();
    let min_sample = 2;
    let least_points = 2; // TODO: Move to the parameter.
    let mut lines = Vec::new();

    for _ in 0..max_iters {
        let random_idxs = rand::seq::index::sample(
            &mut rand::thread_rng(),
            pointcloud.points().len(),
            min_sample,
        )
        .into_vec();

        let mut point_idx = vec![random_idxs[0], random_idxs[1]];
        let mut line = Line2::new(&[
            pointcloud.points()[random_idxs[0]],
            pointcloud.points()[random_idxs[1]],
        ]);
        let mut added_points = Vec::new();

        for (idx, point) in pointcloud
            .points()
            .iter()
            .enumerate()
            .filter(|(i, _)| *i != random_idxs[0] && *i != random_idxs[1])
        {
            let distance = line.distance(&[*point])[0];
            if distance < threshold_distance {
                point_idx.push(idx);
                added_points.push(*point);
            }
        }

        if added_points.len() > least_points {
            line.add_points(&added_points);

            if !lines.iter().any(|(_, l)| *l == line) {
                lines.push((point_idx, line));
            } else {
                let (idx, (_, exist_line)) = lines
                    .iter()
                    .enumerate()
                    .find(|(_, (_, l))| *l == line)
                    .unwrap();
                if exist_line.points_len() < line.points_len() {
                    lines[idx] = (point_idx, line);
                }
            }
        }
    }

    lines
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

    #[test]
    fn test_ransac_algorithm() {
        let mut pointcloud_inner = vec![Point2::new(0.0, 0.0)];
        let resolution = 20.0;
        for i in 1..(resolution as i32) {
            pointcloud_inner.push(Point2::new(0.0, 0.0 + 10.0 / resolution * i as f64));
            pointcloud_inner.push(Point2::new(0.0 + 10.0 / resolution * i as f64, 0.0));
        }
        let pointcloud = Pointcloud2::new(pointcloud_inner);

        let lines = ransac_algorithm(&pointcloud, 100, 0.05);

        for line in lines.iter().map(|(_, l)| l) {
            println!("{:?}", line);
        }

        assert_eq!(lines.len(), 2);
    }
}
