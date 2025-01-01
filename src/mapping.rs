/// Occupancy grid mapping
use crate::*;
use grid_map::{Cell, Grid, GridMap, Position};
use nalgebra as na;

pub const DEFAULT_PROBABILITY_FREE_SPACE: f64 = 0.2;
pub const DEFAULT_PROBABILITY_OCCUPIED_SPACE: f64 = 0.8;

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct MapElement {
    /// Log-odds of the probability of occupancy.
    pub log_odds: f64,
    /// Probability of occupancy.
    pub probability: f64,
}

impl MapElement {
    pub fn is_occupied(&self) -> bool {
        self.probability > 0.5
    }
}

impl Default for MapElement {
    fn default() -> Self {
        let probability: f64 = 0.5;
        Self {
            log_odds: probability.ln() - (1.0 - probability).ln(),
            probability,
        }
    }
}

/// Occupancy grid mapping
pub struct Mapping {
    /// Elements in the grid map are the log-odds of the probability of occupancy.
    grid_map: GridMap<MapElement>,
    probability_free_space: f64,
    probability_occupied_space: f64,
}

impl Mapping {
    pub fn new(
        min_point: Position,
        max_point: Position,
        resolution: f64,
        probability_free_space: f64,
        probability_occupied_space: f64,
    ) -> Self {
        let mut grid_map = GridMap::new(min_point, max_point, resolution);
        for cell in grid_map.cells_mut() {
            *cell = Cell::Uninitialized;
        }
        Self {
            grid_map,
            probability_free_space,
            probability_occupied_space,
        }
    }

    pub fn init(&mut self) {
        for cell in self.grid_map.cells_mut() {
            *cell = Cell::Uninitialized;
        }
    }

    pub fn update(
        &mut self,
        current_position: &(impl Into<na::Isometry2<f64>> + Clone),
        laser_scan: &LaserScan,
    ) {
        let points: Pointcloud2 = laser_scan.clone().into();
        let points = coordinate_transformation(current_position, points.points());

        let current_position = current_position.clone().into();
        let current_position_translation = current_position.translation;

        for point in points {
            let grids = bresenham_algorithm(
                &current_position_translation,
                &point,
                self.grid_map.resolution(),
                self.grid_map.min_point(),
            );

            for grid in grids.iter().take(grids.len() - 1) {
                let cell = self.grid_map.cell_mut(grid).unwrap();
                if cell.is_uninitialized() {
                    *cell = Cell::from_value(MapElement::default());
                }
                let log_odds = cell.value().unwrap().log_odds + self.probability_free_space.ln()
                    - self.probability_occupied_space.ln();
                let probability = 1.0 / (1.0 + (-log_odds).exp());
                *cell = Cell::from_value(MapElement {
                    log_odds,
                    probability,
                });
            }
            let last_grid = grids.last().unwrap();
            let cell = self.grid_map.cell_mut(last_grid).unwrap();
            if cell.is_uninitialized() {
                *cell = Cell::from_value(MapElement::default());
            }
            let log_odds = cell.value().unwrap().log_odds + self.probability_occupied_space.ln()
                - self.probability_free_space.ln();
            let probability = 1.0 / (1.0 + (-log_odds).exp());
            *cell = Cell::from_value(MapElement {
                log_odds,
                probability,
            });
        }

        // TODO: Return updated grid list
    }

    pub fn resolution(&self) -> f64 {
        self.grid_map.resolution()
    }

    pub fn map_size(&self) -> (usize, usize) {
        (self.grid_map.width(), self.grid_map.height())
    }

    pub fn min_point(&self) -> &Position {
        self.grid_map.min_point()
    }

    pub fn get_explored_grids_positions(&self) -> Vec<Position> {
        let mut explored_grids = Vec::new();
        let width = self.grid_map.width();
        let height = self.grid_map.height();
        for h in 0..height {
            for w in 0..width {
                let grid = Grid::new(w, h);
                if let Some(cell) = self.grid_map.cell(&grid) {
                    if !cell.is_uninitialized() {
                        let x = self.grid_map.min_point().x + w as f64 * self.grid_map.resolution();
                        let y = self.grid_map.min_point().y + h as f64 * self.grid_map.resolution();
                        explored_grids.push(Position::new(x, y));
                    }
                }
            }
        }
        explored_grids
    }

    pub fn get_occupied_grids_positions(&self) -> Vec<Position> {
        let mut occupied_grids = Vec::new();
        let width = self.grid_map.width();
        let height = self.grid_map.height();
        for h in 0..height {
            for w in 0..width {
                let grid = Grid::new(w, h);
                if let Some(cell) = self.grid_map.cell(&grid) {
                    if cell.has_value() && cell.value().unwrap().is_occupied() {
                        let x = self.grid_map.min_point().x + w as f64 * self.grid_map.resolution();
                        let y = self.grid_map.min_point().y + h as f64 * self.grid_map.resolution();
                        occupied_grids.push(Position::new(x, y));
                    }
                }
            }
        }
        occupied_grids
    }
}
