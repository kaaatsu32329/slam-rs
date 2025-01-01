use slam::*;

fn main() {
    let current_position = Pose2::new(1.0, 2.0, std::f64::consts::FRAC_PI_2);

    let target_points = (0..8)
        .map(|i| {
            let angle = (i as f64) * std::f64::consts::FRAC_PI_4;
            Point2::new(angle.cos(), angle.sin())
        })
        .collect::<Vec<Point2>>();

    let target_points = coordinate_transformation(&current_position, &target_points);

    println!("Current position: {:?}", current_position);
    println!("Target points: {:?}", target_points);
}
