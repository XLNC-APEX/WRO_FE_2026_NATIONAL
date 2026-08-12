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
        let p = pos + predict_pos(0.1, vel, self.config.l_drv, self.steer, pos.h);
        let (tp, _) = self.path.next_closest_tp(p.into(), 0.0);
        tp
        // TODO: move tp a bit along the path: tp = self.path.at_t(t+dt)
    }
}

// TODO: test correctness
fn predict_pos(dt: f32, v: Vector2<f32>, l: f32, steer: f32, h: f32) -> Pose {
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
    let mut pos: Pose = (Rotation2::new(h) * Vector2::new(x, y)).into();
    pos.h = w * dt;
    pos
}

#[cfg(test)]
mod tests {
    use core::f32::consts::FRAC_PI_6;

    use nalgebra::{Point2, Vector2};
    use sparkfun_otos::Pose;
    extern crate std;

    use crate::follower::{Car, PurePursuit, PurePursuitConfig, predict_pos};
    use crate::path::LinesPath;
    use gnuplot::{AutoOption::Fix, AxesCommon, Figure};

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
            let pos = predict_pos(t, Vector2::new(1.0, 0.0), 0.096, -FRAC_PI_6, 0.0);
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

    #[derive(Debug)]
    struct SimCar {
        pub pos: Pose,
        pub vel: Pose,
        pub steer: f32,
    }

    impl Car for SimCar {
        fn steer(&mut self, pos: f32) {
            self.steer = pos;
        }
        async fn reset(&mut self) {}
        async fn get_pos_vel(&mut self) -> [sparkfun_otos::Pose; 2] {
            [self.pos, self.vel]
        }
    }

    impl SimCar {
        fn new() -> Self {
            SimCar {
                pos: Pose::new(0.0, -0.1, 0.0),
                vel: Pose::new(0.1, 0.0, 0.0),
                steer: 0.0,
            }
        }
    }

    #[tokio::test]
    async fn plot_sim_car() {
        use std::io::Write;
        use std::process::{Command, Stdio};

        let mut process = Command::new("gnuplot")
            .arg("-p")
            .stdin(Stdio::piped())
            .spawn()
            .expect("gnuplot executable not found");

        let gp = process.stdin.as_mut().unwrap();
        writeln!(
            gp,
            "set terminal svg background rgb 'white' size 1024, 1024"
        )
        .unwrap();
        writeln!(gp, "set output 'plot_sim_car.svg'").unwrap();
        writeln!(gp, "set grid").unwrap();
        writeln!(gp, "set size ratio -1").unwrap();
        writeln!(gp, "set xrange [-2:2]").unwrap();
        writeln!(gp, "set yrange [-2:2]").unwrap();
        // x y dx dy
        writeln!(
            gp,
            "plot '-' with linespoints notitle, '-' with linespoints notitle"
        )
        .unwrap();

        let car = SimCar::new();
        let path_points = [
            Point2::new(0.0, 0.0),
            Point2::new(1.0, 0.0),
            Point2::new(1.0, 1.0),
            Point2::new(2.0, 1.0),
        ];
        let path = LinesPath::new(path_points);

        let config = PurePursuitConfig {
            kl: 1.0,
            min_l: 0.1,
            max_l: 0.5,
            l_drv: 0.096,
            max_steer: FRAC_PI_6,
        };
        let mut pp = PurePursuit::new(car, path, config);
        const SIM_T: f32 = 32.0;
        const DT: f32 = 0.1;
        const N: usize = (SIM_T / DT) as usize;
        for _ in 0..N {
            writeln!(gp, "{} {}", -pp.car.pos.y, pp.car.pos.x).unwrap();
            pp.update().await;
            pp.car.pos += predict_pos(
                DT,
                pp.car.vel.into(),
                pp.config.l_drv,
                pp.car.steer,
                pp.car.pos.h,
            );
        }
        writeln!(gp, "e").unwrap();
        for p in path_points {
            writeln!(gp, "{} {}", -p.y, p.x).unwrap();
        }
        writeln!(gp, "e").unwrap();
        process.wait().unwrap();
    }
}
