use crate::*;

#[derive(Debug, Clone, Default)]
pub struct SlamExecutor {}

impl SlamExecutor {
    pub fn update(&self, point_cloud: &PointCloud, map: &mut Map) {
        // TODO: Improve this algorithm
        let center = point_cloud.center();
        map.point_map.add_points(
            point_cloud
                .points()
                .iter()
                .map(|p| center.rotation * p + center.translation.vector)
                .collect::<Vec<_>>(),
        );
    }
}
