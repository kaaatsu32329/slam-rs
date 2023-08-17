use crate::*;
use bevy::{
    app::*,
    prelude::*,
    window::{Window, WindowPlugin},
};
use bevy_egui::{
    egui::{self, plot::Plot, Color32},
    EguiContexts, EguiPlugin,
};
use parking_lot::Mutex;
use std::sync::Arc;

#[derive(Resource)]
pub struct SharedSlam(Arc<Mutex<Slam>>);

pub struct SlamViewerApp {
    app: App,
}

impl SlamViewerApp {
    pub fn new() -> Self {
        Self { app: App::new() }
    }

    pub fn setup(&mut self, slam: Arc<Mutex<Slam>>) {
        let user_plugin = DefaultPlugins.set(WindowPlugin {
            primary_window: Some(Window {
                title: "SLAM".to_owned(),
                resolution: (1080., 720.).into(),
                ..Default::default()
            }),
            ..Default::default()
        });

        self.app
            .add_plugins(user_plugin)
            .add_plugin(EguiPlugin)
            .insert_resource(SharedSlam(slam))
            .add_system(update_system);
    }

    pub fn run(&mut self) {
        self.app.run();
    }
}

fn update_system(mut contexts: EguiContexts, slam: Res<SharedSlam>) {
    let ctx = contexts.ctx_mut();

    egui::CentralPanel::default().show(ctx, |ui| {
        Plot::new("point_cloud")
            .data_aspect(1.)
            .show(ui, |plot_ui| {
                let locked_slam = slam.0.lock();
                plot_ui.polygon(convert_robot_to_egui_point(
                    locked_slam.robot(),
                    Color32::LIGHT_BLUE,
                    1.,
                ));
                plot_ui.points(convert_point_cloud_to_egui_points(
                    locked_slam.current_point_cloud(),
                    Color32::YELLOW,
                    1.,
                ));
            });
    });
}
