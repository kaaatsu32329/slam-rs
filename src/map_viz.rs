use crate::*;
use grid_map::Position;

pub struct MapViz {
    recording_stream: rerun::RecordingStream,
}

impl MapViz {
    pub fn new() -> Self {
        let recording_stream = rerun::RecordingStreamBuilder::new("map_viz")
            .spawn()
            .unwrap();
        Self { recording_stream }
    }

    pub fn update(&mut self, mapping: &Mapping, current_pose: Pose) {
        let map_frame_min_point = (mapping.min_point().x as f32, mapping.min_point().y as f32);
        let map_size = mapping.map_size();
        let resolution = mapping.resolution();
        let map_frame_size = (
            map_size.0 as f32 * resolution as f32,
            map_size.1 as f32 * resolution as f32,
        );

        let explored_grids = mapping.get_explored_grids_positions();
        let occupied_grids = mapping.get_occupied_grids_positions();

        let map_frame_box =
            rerun::Boxes2D::from_mins_and_sizes([map_frame_min_point], [map_frame_size])
                .with_colors([rerun::Color::from_rgb(250, 250, 250)]);
        let explored_boxes =
            position_to_rerun_boxes2d(&explored_grids, resolution, (200, 210, 230));
        let occupied_boxes = position_to_rerun_boxes2d(&occupied_grids, resolution, (20, 60, 240));
        let robot = rerun::Points2D::new([(current_pose.x() as f32, current_pose.y() as f32)])
            .with_radii([0.05])
            .with_colors([rerun::Color::from_rgb(200, 50, 50)]);

        self.recording_stream
            .log("slam/00_map_frame", &map_frame_box)
            .unwrap();
        self.recording_stream
            .log("slam/01_explored", &explored_boxes)
            .unwrap();
        self.recording_stream
            .log("slam/02_occupied", &occupied_boxes)
            .unwrap();
        self.recording_stream.log("slam/03_robot", &robot).unwrap();
    }
}

fn position_to_rerun_boxes2d(
    grid_positions: &[Position],
    resolution: f64,
    color_rgb: (u8, u8, u8),
) -> rerun::Boxes2D {
    let length = grid_positions.len();

    let mins = grid_positions
        .iter()
        .map(|position| (position.x as f32, position.y as f32));

    let sizes = std::iter::repeat((resolution as f32, resolution as f32)).take(length);

    rerun::Boxes2D::from_mins_and_sizes(mins, sizes).with_colors([rerun::Color::from_rgb(
        color_rgb.0,
        color_rgb.1,
        color_rgb.2,
    )])
}

impl Default for MapViz {
    fn default() -> Self {
        Self::new()
    }
}
