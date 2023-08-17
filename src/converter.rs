use crate::Robot;
use bevy_egui::egui::{
    plot::{PlotPoints, Points, Polygon},
    Color32,
};
use nalgebra as na;

pub fn convert_robot_to_egui_point(robot: &Robot, color: Color32, scale: f64) -> Polygon {
    let robot_size = scale * 0.2;

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
    point_cloud: &crate::PointCloud,
    color: Color32,
    scale: f32,
) -> Points {
    let plot_point: PlotPoints =
        PlotPoints::new(point_cloud.points().iter().map(|p| [p.x, p.y]).collect());

    Points::new(plot_point).color(color).radius(scale)
}
