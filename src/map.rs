use crate::PointMap;
use grid_map::{GridMap, Position};

#[derive(Debug, Clone)]
pub struct Map {
    pub grid_map: GridMap<u8>,
    pub point_map: PointMap,
}

impl Map {
    pub fn new(grid_map: GridMap<u8>, point_map: PointMap) -> Self {
        Self {
            grid_map,
            point_map,
        }
    }
}

impl Default for Map {
    fn default() -> Self {
        Self {
            grid_map: GridMap::new(
                Position { x: -10., y: -10. },
                Position { x: 10., y: 10. },
                0.05,
            ),
            point_map: PointMap::default(),
        }
    }
}
