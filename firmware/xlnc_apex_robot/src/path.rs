use nalgebra::Point2;

pub trait Path {
    fn at_t(&mut self, t: f32) -> Point2<f32>;
    /// Returns a point and it's t, on path closest to p and t.
    /// New t > old t
    fn next_closest_tp(&mut self, p: Point2<f32>, t: f32) -> (Point2<f32>, f32);
}

pub struct LinesPath<const N: usize> {
    /// Points
    p: [Point2<f32>; N],
    i: usize,
    t_l: f32,
    t_r: f32,
}

impl<const N: usize> Path for LinesPath<N> {
    fn at_t(&mut self, t: f32) -> Point2<f32> {
        self.go_until_t(t);
        self.p[self.i].lerp(&self.p[self.i + 1], t - self.t_l)
    }
    fn next_closest_tp(&mut self, _p: Point2<f32>, _t: f32) -> (Point2<f32>, f32) {
        unimplemented!()
    }
}

impl<const N: usize> LinesPath<N> {
    pub fn new(points: [Point2<f32>; N]) -> Self {
        LinesPath {
            p: points,
            i: 0,
            t_r: (points[1] - points[0]).magnitude(),
            t_l: 0.0,
        }
    }

    fn go_until_t(&mut self, t: f32) {
        while self.t_r <= t {
            self.t_l = self.t_r;
            self.t_r += (self.p[self.i + 1] - self.p[self.i]).magnitude();
            self.i += 1;
        }
    }
}
