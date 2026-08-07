use nalgebra::Point2;

pub trait Path {
    fn at_t(&self, t: f32) -> Point2<f32>;
    /// Returns a point and it's t, on path closest to p and t.
    /// New t > old t
    fn next_closest_tp(&self, p: Point2<f32>, t: f32) -> (Point2<f32>, f32);
}

pub struct LinesPath<const N: usize> {
    point_len: [(Point2<f32>, f32); N],
}

impl<const N: usize> Path for LinesPath<N> {
    fn at_t(&self, t: f32) -> Point2<f32> {
        unimplemented!()
    }
    fn next_closest_tp(&self, p: Point2<f32>, t: f32) -> (Point2<f32>, f32) {
        unimplemented!()
    }
}
