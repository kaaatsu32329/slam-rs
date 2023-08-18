use crate::{PointCloud, PointMap};

#[derive(Debug, Clone, Default)]
pub struct SlamExecutor {}

impl SlamExecutor {
    pub fn update(&self, point_cloud: &PointCloud, point_map: &mut PointMap) {
        // TODO: Improve this algorithm
        let center = point_cloud.center();
        point_map.add_points(
            point_cloud
                .points()
                .iter()
                .map(|p| center.rotation * p + center.translation.vector)
                .collect::<Vec<_>>(),
        );
    }
}
