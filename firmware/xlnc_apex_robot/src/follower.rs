use defmt::dbg;
use libm::{atan2f, atanf, cosf, sinf};
use nalgebra::{Point2, Rotation2, Vector2};
use sparkfun_otos::driver::otos::Pose;

use crate::path::Path;

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
    /// drive length(front, rear axles dist)
    pub l_drv: f32,
    /// absolute max steer in radians
    pub max_steer: f32,
}
pub struct PurePursuit<T: Car, P: Path> {
    car: T,
    path: P,
    t: f32,
    steer: f32,
    config: PurePursuitConfig,
}

impl<T: Car, P: Path> PurePursuit<T, P> {
    pub fn new(car: T, path: P, config: PurePursuitConfig) -> Self {
        Self {
            car,
            path,
            t: 0.0,
            steer: 0.0,
            config,
        }
    }

    /// Updates steering angle
    pub async fn update(&mut self) {
        let [pos, vel] = self.car.get_pos_vel().await;
        dbg!(vel);
        let tp_rel = self.get_target_point(pos, vel.into()) - Point2::<f32>::from(pos);
        dbg!(tp_rel, self.t);
        self.steer = self.get_steer_to(tp_rel, pos.h);
        dbg!(self.steer);
        self.car.steer(self.steer);
    }

    fn get_steer_to(&self, tp: Vector2<f32>, h: f32) -> f32 {
        let a = atan2f(tp.y, tp.x) - h;
        dbg!(a);
        atanf((2.0 * self.config.l_drv * sinf(a)) / tp.magnitude())
            .clamp(-self.config.max_steer, self.config.max_steer)
    }

    fn get_target_point(&mut self, pos: Pose, vel: Vector2<f32>) -> Point2<f32> {
        // Note, vel does not need to be rotated by h, because we only need magnitude.
        let p = Point2::<f32>::from(pos)
            + Self::predict_pos(0.1, vel, self.config.l_drv, self.steer, pos.h);
        let (tp, _) = self.path.next_closest_tp(p, 0.0);
        tp
        // TODO: move tp a bit along the path: tp = self.path.at_t(t+dt)
    }

    // TODO: test correctness
    fn predict_pos(dt: f32, v: Vector2<f32>, l: f32, steer: f32, h: f32) -> Vector2<f32> {
        let v = v.magnitude();
        // Equation of rotation speed.
        // I did it intuitively for rear drive bicycle model
        let w = (v * sinf(2.0 * steer)) / (2.0 * l);
        // Correction for fns discontinuity at w = 0
        // Threshold may be tuned?
        let (x, y) = if w.abs() > 1e-4 {
            ((v * sinf(w * dt)) / w, (v * (1.0 - cosf(w * dt))) / w)
        } else {
            (v * dt, 0.0)
        };
        Rotation2::new(h) * Vector2::new(x, y)
    }
}

#[cfg(test)]
mod tests {
    use core::f32::consts::FRAC_PI_6;

    use nalgebra::Vector2;
    use sparkfun_otos::Pose;
    extern crate std;

    use crate::{follower::Car, follower::PurePursuit};
    use gnuplot::{AutoOption::Fix, AxesCommon, Figure};

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
        let mut fg = Figure::new();
        fg.set_terminal("png size 512, 512", "plot_predict_pos.png");
        const N: usize = 100;
        let mut x = [0f32; N];
        let mut y = [0f32; N];
        let mut t = 0.0;
        let ax = fg.axes2d();
        ax.set_x_grid(true);
        ax.set_y_grid(true);
        ax.set_aspect_ratio(Fix(-1.0));
        ax.set_x_range(Fix(-0.5), Fix(0.5));
        ax.set_y_range(Fix(-0.5), Fix(0.5));
        ax.points([0.0], [0.0], &[]);
        for i in 0..N {
            let pos = PurePursuit::<MockCar>::_predict_pos(
                t,
                Vector2::new(1.0, 0.0),
                0.096,
                -FRAC_PI_6,
                0.0,
            );
            // For FL(x forward, y left) coordinate system conversion:
            x[i] = -pos.y;
            y[i] = pos.x;
            t += 0.01;
            std::dbg!(pos);
        }
        ax.lines_points(x, y, &[]);
        fg.set_title("Path of car");
        fg.show().unwrap();
    }
}
