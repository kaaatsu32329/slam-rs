use crate::*;
use bevy_egui::egui::{
    plot::{PlotPoints, Points, Polygon},
    Color32,
};
use grid_map::*;
use nalgebra as na;

pub fn convert_robot_to_egui_point(robot: &Robot, color: Color32, scale: f64) -> Polygon {
    let robot_size = scale * 0.04;

    let vertices = vec![
        na::Vector2::new(2. * robot_size, 0.),
        na::Vector2::new(-robot_size, robot_size),
        na::Vector2::new(-robot_size, -robot_size),
    ];

    let plot_point: PlotPoints = PlotPoints::new(
        vertices
            .into_iter()
            .map(|v| {
                let p = robot.pose().rotation * v + robot.pose().translation.vector;
                [p[0], p[1]]
            })
            .collect(),
    );

    Polygon::new(plot_point).color(color).fill_alpha(1.0)
}

pub fn convert_point_cloud_to_egui_points(
    point_cloud: &PointCloud,
    color: Color32,
    scale: f32,
) -> Points {
    let c = point_cloud.center();
    let plot_point: PlotPoints = PlotPoints::new(
        point_cloud
            .points()
            .iter()
            .map(|p| {
                let p = c.rotation * na::Vector2::new(p.x, p.y) + c.translation.vector;
                [p.x, p.y]
            })
            .collect(),
    );

    Points::new(plot_point).color(color).radius(scale)
}

pub fn convert_point_map_to_egui_points(
    point_map: &PointMap,
    color: Color32,
    scale: f32,
) -> Points {
    let plot_point: PlotPoints = PlotPoints::new(
        point_map
            .points()
            .iter()
            .flatten()
            .map(|p| [p.x, p.y])
            .collect(),
    );

    Points::new(plot_point).color(color).radius(scale)
}

pub fn convert_grid_map_to_egui_polygon(grid_map: &GridMap<u8>) -> Vec<Polygon> {
    let min_point = grid_map.min_point();
    let resolution = grid_map.resolution();
    let width = grid_map.width();
    let cells = grid_map.cells();

    let mut polygons = Vec::<Polygon>::new();

    for (i, cell_i) in cells.iter().enumerate() {
        let order = [0, 1, 3, 2];
        let plot_points: PlotPoints = order
            .iter()
            .map(|j| {
                let x = min_point.x + (i % width + j % 2) as f64 * resolution;
                let y = min_point.y + (i / width + j / 2) as f64 * resolution;
                [x, y]
            })
            .collect();
        let polygon = match cell_i {
            Cell::Unknown => Polygon::new(plot_points)
                .color(Color32::from_gray(250))
                .fill_alpha(1.0),
            Cell::Obstacle => Polygon::new(plot_points)
                .color(Color32::from_gray(30))
                .fill_alpha(1.0),
            Cell::Value(_) => Polygon::new(plot_points)
                .color(Color32::from_gray(220))
                .fill_alpha(1.0),
            Cell::Uninitialized => Polygon::new(plot_points)
                .color(Color32::from_gray(200))
                .fill_alpha(1.0),
        };
        polygons.push(polygon);
    }

    polygons
}
