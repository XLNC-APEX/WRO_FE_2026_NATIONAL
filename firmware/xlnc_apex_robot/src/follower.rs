use defmt::{dbg, trace};
use heapless::Vec;
use libm::{atan2f, atanf, cosf, sincosf, sinf, sqrtf, tanf};
use nalgebra::{Point2, Vector2};
use sparkfun_otos::driver::otos::Pose;

use crate::follower::IntersectionError::{NoIntr, OutOfSegment};
#[cfg(target_os = "none")]
use crate::target::beep;

pub trait Car {
    fn steer(&mut self, pos: f32);
    fn get_pos_vel(&mut self) -> impl Future<Output = [Pose; 2]> + Send;
    fn reset(&mut self) -> impl Future<Output = ()> + Send;
}

pub struct PurePursuitConfig {
    /// lookahead coefficient
    pub kl: f32,
    pub min_l: f32,
    pub max_l: f32,
    // drive length(front, rear axles dist)
    pub l_drv: f32,
    // absolute max steer in radians
    pub max_steer: f32,
}
pub struct PurePursuit<T: Car> {
    car: T,
    path: &'static [Point2<f32>],
    n: usize,
    config: PurePursuitConfig,
}

impl<T: Car> PurePursuit<T> {
    pub fn new(car: T, path: &'static [Point2<f32>], config: PurePursuitConfig) -> Self {
        Self {
            car,
            path,
            n: 0,
            config,
        }
    }

    /// Updates steering angle
    pub async fn update(&mut self) {
        let [pos, vel] = self.car.get_pos_vel().await;
        dbg!(pos);
        let ld = self.get_lookahead_radius(vel.into());
        dbg!(ld);
        let tp = self.get_target_point(ld, pos.into());
        dbg!(tp, self.n);
        let a = atan2f(tp.y, tp.x) - pos.h;
        dbg!(a);
        let steer = atanf((2.0 * self.config.l_drv * sinf(a)) / ld);
        dbg!(steer);
        self.car
            .steer(steer.clamp(-self.config.max_steer, self.config.max_steer));
    }

    // TP is relative: as if pos is coords origin
    fn get_target_point(&mut self, r: f32, pos: Point2<f32>) -> Point2<f32> {
        while (self.n + 1) < self.path.len() {
            trace!("In loop");
            dbg!(self.n);
            let s = self.path[self.n] - pos;
            let e = self.path[self.n + 1] - pos;
            match Self::find_intersection(s, e, r) {
                Err(NoIntr) => {
                    trace!("No intr");
                    // Check intr with next segment
                    if (self.n + 2) < self.path.len() {
                        let s = self.path[self.n + 1] - pos;
                        let e = self.path[self.n + 2] - pos;
                        if Self::find_intersection(s, e, r).is_ok() {
                            trace!("Found intersection on the next segment, n++");
                            self.n += 1;
                            #[cfg(target_os = "none")]
                            beep();
                            continue;
                        }
                    }
                    // This is the last segment
                    trace!("Going to segment end");
                    return e.into();
                }
                Err(OutOfSegment) => {
                    trace!("Out of segment");
                    // Check intr with next segment
                    if (self.n + 2) < self.path.len() {
                        let s = self.path[self.n + 1] - pos;
                        let e = self.path[self.n + 2] - pos;
                        if Self::find_intersection(s, e, r).is_ok() {
                            trace!("Found intersection on the next segment, n++");
                            self.n += 1;
                            #[cfg(target_os = "none")]
                            beep();
                            continue;
                        }
                    }
                    // This is the last segment
                    trace!("Going to segment end");
                    return e.into();
                }
                Ok(p) => return p,
            }
        }
        // If path ended, return last point of path
        (self.path.last().unwrap() - pos).into()
    }

    fn find_intersection(
        s: Vector2<f32>,
        e: Vector2<f32>,
        r: f32,
    ) -> Result<Point2<f32>, IntersectionError> {
        let m = s + e;
        let a = m.x * m.x + m.y * m.y;
        let b = -2.0 * (s.x * m.x + s.y * m.y);
        let c = s.norm_squared() - (r * r);

        let d = b * b - 4.0 * a * c;
        if d < 0.0 {
            return Err(NoIntr);
        }
        let sqrt_d = sqrtf(d);
        // TODO: what if a == 0? Can it be?
        let t1 = (-b + sqrt_d) / (2.0 * a);
        let t2 = (-b - sqrt_d) / (2.0 * a);
        let mut ts = Vec::<f32, 2>::new();
        for t in [t1, t2] {
            if (0.0..=1.0).contains(&t) {
                ts.push(t).unwrap(); // Should never fail, since ts has 2 len.
            }
        }
        if ts.is_empty() {
            return Err(OutOfSegment);
        }
        let t = *ts.iter().max_by(|a, b| a.partial_cmp(b).unwrap()).unwrap();
        let p = -(s - (m * t));
        Ok(p.into())
    }

    fn get_lookahead_radius(&self, vel: Vector2<f32>) -> f32 {
        (vel.norm() * self.config.kl).clamp(self.config.min_l, self.config.max_l)
    }

    // TODO: make numerical stable. NaN at steer == 0.0;
    fn predict_pos(dt: f32, v: f32, l: f32, steer: f32, h: f32) -> Vector2<f32> {
        let r = l / tanf(steer);
        let y = (v * dt) / r;
        let a = y * 2.0;

        let z = r * sqrtf(2.0 * (1.0 - cosf(y)));
        let (s, c) = sincosf(a + h);
        Vector2::new(z * c, z * s)
    }
}

#[cfg(test)]
mod tests {
    use core::f32::consts::{FRAC_PI_2, FRAC_PI_6};

    use sparkfun_otos::Pose;
    extern crate std;

    use crate::{follower::Car, follower::PurePursuit};

    struct MockCar;

    impl Car for MockCar {
        fn steer(&mut self, _pos: f32) {}
        async fn reset(&mut self) {}
        async fn get_pos_vel(&mut self) -> [sparkfun_otos::Pose; 2] {
            [Pose::new(0.0, 0.0, 0.0); 2]
        }
    }

    #[test]
    fn plot_predict_pos() {
        let pos = PurePursuit::<MockCar>::predict_pos(1.0, 1.0, 0.096, 0.0001, FRAC_PI_2);
        std::dbg!(pos);
    }
}

enum IntersectionError {
    /// Negative discriminant
    NoIntr,
    /// No positive t
    OutOfSegment,
}
