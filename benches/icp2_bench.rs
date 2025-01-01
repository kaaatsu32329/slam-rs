use criterion::{criterion_group, criterion_main, Criterion};
use slam::*;

fn icp2_bench(c: &mut Criterion) {
    let icp_client = data_gen();
    c.bench_function("icp2_bench", |b| {
        b.iter(|| {
            let mut icp_client = icp_client.clone();
            icp_client.scan_matching(5);
        });
    });
}

fn data_gen() -> IterativeClosestPoint2 {
    let init_pose = Pose2::new(0.0, 0.0, 0.0);
    let mut scan_points_inner = vec![Point2::new(init_pose.x(), init_pose.y())];
    let wall_length = 1.0;
    let resolution = 20.0;
    for i in 1..(resolution as i32) {
        scan_points_inner.push(Point2::new(
            init_pose.x(),
            init_pose.y() + wall_length / resolution * i as f64,
        ));
        scan_points_inner.push(Point2::new(
            init_pose.x() + wall_length / resolution * i as f64,
            init_pose.y(),
        ));
    }

    let expected_pose = Pose2::new(0.1, 0.1, 0.1);
    let reference_points_inner = coordinate_transformation(&expected_pose, &scan_points_inner)
        .iter()
        .map(|p| Point2::from(*p))
        .collect::<Vec<Point2>>();

    let scan_points = Pointcloud2::new(scan_points_inner);
    let reference_points = Pointcloud2::new(reference_points_inner);

    IterativeClosestPoint2::new(&scan_points, &reference_points, &init_pose)
}

criterion_group!(benches, icp2_bench);
criterion_main!(benches);
