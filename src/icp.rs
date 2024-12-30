use crate::*;
use argmin::{
    core::{CostFunction, Executor, Gradient},
    solver::{gradientdescent::SteepestDescent, linesearch::MoreThuenteLineSearch},
};

#[derive(Debug, Clone)]
pub struct IterativeClosestPoint {
    /// Robot coordinates of the scan points.
    scan_points: Pointcloud2,
    /// World coordinates of the reference points.
    reference_points: Pointcloud2,
    /// Estimated robot pose.
    robot_pose: Pose,
    /// Correspondences between scan points and reference points.
    correspondences: Vec<usize>,
}

impl IterativeClosestPoint {
    pub fn new(
        scan_points: &(impl Into<Pointcloud2> + Clone),
        reference_points: &(impl Into<Pointcloud2> + Clone),
        robot_pose: &(impl Into<Pose> + Clone),
    ) -> Self {
        Self {
            scan_points: (*scan_points).clone().into(),
            reference_points: (*reference_points).clone().into(),
            robot_pose: (*robot_pose).clone().into(),
            correspondences: vec![0; scan_points.clone().into().points().len()],
        }
    }

    fn data_correspondences(&mut self) {
        let scan_points_transformed =
            coordinate_transformation(&self.robot_pose, self.scan_points.points())
                .iter()
                .map(|p| Point::from(*p))
                .collect::<Vec<Point>>();
        for (i, scan_point) in scan_points_transformed.iter().enumerate() {
            let mut min_distance = f64::MAX;
            let mut min_idx = 0;
            for (j, reference_point) in self.reference_points.points().iter().enumerate() {
                let distance = scan_point.distance(reference_point);
                if distance < min_distance {
                    min_distance = distance;
                    min_idx = j;
                }
            }
            self.correspondences[i] = min_idx;
        }
    }

    fn distance_between_correspondences(&self, pose: &Pose) -> f64 {
        let mut distance = 0.0;
        let scan_points_transformed = coordinate_transformation(pose, self.scan_points.points())
            .iter()
            .map(|p| Point::from(*p))
            .collect::<Vec<Point>>();
        for (i, j) in self.correspondences.iter().enumerate() {
            distance +=
                scan_points_transformed[i].distance_squared(&self.reference_points.points()[*j]);
        }
        distance / self.correspondences.len() as f64
    }

    pub fn optimize_once(&mut self) {
        self.data_correspondences();

        let init_pose = self.robot_pose;
        let linesearch = MoreThuenteLineSearch::new();
        let solver = SteepestDescent::new(linesearch);

        let result = Executor::new(self.clone(), solver)
            .configure(|state| state.param(init_pose).max_iters(10).target_cost(0.0))
            .run()
            .unwrap();

        let best = result.state.best_param.unwrap();

        self.robot_pose = best;
    }

    pub fn scan_matching(&mut self, max_iterations: usize) {
        for _ in 0..max_iterations {
            self.optimize_once();
        }
    }
}

impl CostFunction for IterativeClosestPoint {
    type Param = Pose;

    type Output = f64;

    fn cost(&self, param: &Self::Param) -> Result<Self::Output, argmin::core::Error> {
        Ok(self.distance_between_correspondences(param))
    }
}

impl Gradient for IterativeClosestPoint {
    type Param = Pose;

    type Gradient = Pose;

    fn gradient(&self, param: &Self::Param) -> Result<Self::Gradient, argmin::core::Error> {
        let epsilon = 1e-6;
        let grad_x = (self.cost(&Pose::new(param.x() + epsilon, param.y(), param.theta()))?
            - self.cost(&Pose::new(param.x() - epsilon, param.y(), param.theta()))?)
            / (2.0 * epsilon);
        let grad_y = (self.cost(&Pose::new(param.x(), param.y() + epsilon, param.theta()))?
            - self.cost(&Pose::new(param.x(), param.y() - epsilon, param.theta()))?)
            / (2.0 * epsilon);
        let grad_theta = (self.cost(&Pose::new(param.x(), param.y(), param.theta() + epsilon))?
            - self.cost(&Pose::new(param.x(), param.y(), param.theta() - epsilon))?)
            / (2.0 * epsilon);
        Ok(Pose::new(grad_x, grad_y, grad_theta))
    }
}

#[cfg(test)]
mod test {
    use super::*;
    use assert_approx_eq::assert_approx_eq;

    #[test]
    fn test_icp() {
        let mut icp_client = data_gen();
        icp_client.scan_matching(5);

        let expected_pose = Pose::new(0.1, 0.1, 0.1);

        println!("Estimated: {:?}", icp_client.robot_pose);
        println!("Expected: {:?}", expected_pose);

        assert_approx_eq!(icp_client.robot_pose.x(), expected_pose.x());
        assert_approx_eq!(icp_client.robot_pose.y(), expected_pose.y());
        assert_approx_eq!(
            icp_client.robot_pose.theta() % std::f64::consts::TAU,
            expected_pose.theta() % std::f64::consts::TAU
        );
    }

    fn data_gen() -> IterativeClosestPoint {
        let init_pose = Pose::new(0.0, 0.0, 0.0);
        let mut scan_points_inner = vec![Point::new(init_pose.x(), init_pose.y())];
        let wall_length = 1.0;
        let resolution = 20.0;
        for i in 1..(resolution as i32) {
            scan_points_inner.push(Point::new(
                init_pose.x(),
                init_pose.y() + wall_length / resolution * i as f64,
            ));
            scan_points_inner.push(Point::new(
                init_pose.x() + wall_length / resolution * i as f64,
                init_pose.y(),
            ));
        }

        let expected_pose = Pose::new(0.1, 0.1, 0.1);
        let reference_points_inner = coordinate_transformation(&expected_pose, &scan_points_inner)
            .iter()
            .map(|p| Point::from(*p))
            .collect::<Vec<Point>>();

        let scan_points = Pointcloud2::new(scan_points_inner);
        let reference_points = Pointcloud2::new(reference_points_inner);

        IterativeClosestPoint::new(&scan_points, &reference_points, &init_pose)
    }
}
