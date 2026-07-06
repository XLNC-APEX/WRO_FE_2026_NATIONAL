use defmt::dbg;
use heapless::Vec;
use libm::{atan2f, sinf, sqrtf};
use nalgebra::{Point2, Vector2};
use sparkfun_otos::driver::otos::Pose;

pub trait Car {
    fn steer_deg(&mut self, pos: f32);
    fn steer_rad(&mut self, pos: f32);
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
    // absolute max steer in degrees
    pub max_steer_rad: f32,
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
        dbg!(tp);
        let a = atan2f(tp.y, tp.x) - pos.h;
        dbg!(a);
        let steer = atan2f(ld, 2.0 * self.config.l_drv * sinf(a));
        dbg!(steer);
        self.car
            .steer_rad(steer.clamp(-self.config.max_steer_rad, self.config.max_steer_rad));
    }

    // TP is relative: as if pos is coords origin
    fn get_target_point(&mut self, r: f32, pos: Point2<f32>) -> Point2<f32> {
        while (self.n + 1) < self.path.len() {
            let s = self.path[self.n] - pos;
            let e = self.path[self.n + 1] - pos; //TODO: no out of bounds, make sure
            let m = s + e;
            let a = m.x * m.x + m.y * m.y;
            let b = -2.0 * (s.x * m.x + s.y * m.y);
            let c = s.norm_squared() - (r * r);

            let d = b * b - 4.0 * a * c;
            // No intersection
            if d < 0.0 {
                // Proceed to next segment
                self.n += 1;
                continue;
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
            // No intersection within segment
            if ts.is_empty() {
                // Proceed to next segment
                self.n += 1;
                continue;
            }
            // Select the inter. closest to end
            // if d == 0.0, still should work.

            // TODO: make pretty code
            // will it work when ts len is 1?
            let t = *ts.iter().max_by(|a, b| a.partial_cmp(b).unwrap()).unwrap();
            let p = s - (m * t);
            return p.into();
        }
        // If path ended, return last point of path
        (self.path.last().unwrap() - pos).into()
    }

    fn get_lookahead_radius(&self, vel: Vector2<f32>) -> f32 {
        (vel.norm() * self.config.kl).clamp(self.config.min_l, self.config.max_l)
    }
}
