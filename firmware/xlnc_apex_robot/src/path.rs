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
        self.p[self.i].lerp(&self.p[self.i + 1], (t - self.t_l) / (self.t_r - self.t_l))
    }
    fn next_closest_tp(&mut self, p: Point2<f32>, _t: f32) -> (Point2<f32>, f32) {
        // Naive, t independent, global mapping.
        let segs = self.p.array_windows::<2>().skip(self.i);
        let ps = segs.map(|s| closest_on_seg(&p, &s[0], &s[1]));
        let mut closest = Point2::origin(); // Works on plots.
        let mut min_d = f32::MAX;
        for cp in ps {
            let d = (p - cp).magnitude_squared();
            if d < min_d {
                min_d = d;
                closest = cp;
            }
        }
        (closest, 0.0)
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

pub fn closest_on_seg(p: &Point2<f32>, p0: &Point2<f32>, p1: &Point2<f32>) -> Point2<f32> {
    // segment vector
    let s = p1 - p0;
    // If segment len is too small, return p0.
    // This avoids division by zero
    let s_mg2 = s.magnitude_squared();
    if s_mg2 < 1e-6 {
        return *p0;
    }
    // vector to p, from first point of segment
    let p = p - p0;
    // ratio of projection/segment vectors
    // clamped to be within segment
    let r = (s.dot(&p) / s_mg2).clamp(0.0, 1.0);
    // projected p vector
    p0 + r * s
}

#[cfg(test)]
mod tests {
    extern crate std;
    use gnuplot::{AutoOption::Fix, AxesCommon, Figure};
    use nalgebra::Point2;

    use crate::path::{LinesPath, Path, closest_on_seg};

    #[test]
    fn plot_at_t() {
        let path = [
            Point2::new(0.0, 0.0),
            Point2::new(1.0, 0.0),
            Point2::new(1.0, 1.0),
        ];
        let mut path = LinesPath::new(path);
        let mut fg = Figure::new();
        fg.set_terminal(
            "svg size 1024, 1024 background rgb 'white'",
            "plot_at_t.svg",
        );
        const N: usize = 100;
        const T_MAX: f32 = 2.0;
        let mut x = [0f32; N];
        let mut y = [0f32; N];
        let mut t = 0.0;
        let ax = fg.axes2d();
        ax.set_x_grid(true);
        ax.set_y_grid(true);
        ax.set_aspect_ratio(Fix(-1.0));
        ax.set_x_range(Fix(-1.0), Fix(1.0));
        ax.set_y_range(Fix(-1.0), Fix(1.0));
        for i in 0..N {
            let p = path.at_t(t);
            x[i] = -p.y;
            y[i] = p.x;
            t += T_MAX / N as f32;
        }
        ax.lines_points(x, y, &[]);
        fg.set_title("Path");
        fg.show().unwrap();
    }

    use std::io::Write;
    use std::process::{Command, Stdio};
    #[test]
    fn plot_closest_on_seg() {
        let path = [
            Point2::new(0.0, 0.0),
            Point2::new(1.0, 0.0),
            Point2::new(1.0, 1.0),
        ];
        let path = LinesPath::new(path);
        const N: usize = 16;
        const D_MAX: f32 = 2.0;
        const DV: f32 = D_MAX / N as f32;

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
        writeln!(gp, "set output 'plot_closest_on_seg.svg'").unwrap();
        writeln!(gp, "set grid").unwrap();
        writeln!(gp, "set size ratio -1").unwrap();
        writeln!(gp, "set xrange [-2:2]").unwrap();
        writeln!(gp, "set yrange [-2:2]").unwrap();
        // x y dx dy
        writeln!(gp, "plot '-' using 1:2:3:4 with vectors notitle").unwrap();

        for yi in 0..(2 * N) {
            let y = (N as f32 * -DV) + yi as f32 * DV;
            for xi in 0..(2 * N) {
                let x = (N as f32 * -DV) + xi as f32 * DV;
                let p = Point2::new(x, y);
                let dest = closest_on_seg(&p, &path.p[0], &path.p[1]);
                let a = (dest - p).normalize() * 0.05;
                writeln!(gp, "{} {} {} {}", -p.y, p.x, -a.y, a.x).unwrap();
            }
        }
        writeln!(gp, "e").unwrap();
        process.wait().unwrap();
    }

    #[test]
    fn plot_next_closest_tp() {
        let path_points = [
            Point2::new(0.0, 0.0),
            Point2::new(1.0, 0.0),
            Point2::new(1.0, 1.0),
            Point2::new(2.0, -1.0),
        ];
        let mut path = LinesPath::new(path_points);
        const N: usize = 32;
        const D_MAX: f32 = 2.0;
        const DV: f32 = D_MAX / N as f32;

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
        writeln!(gp, "set output 'plot_next_closest_tp.svg'").unwrap();
        writeln!(gp, "set grid").unwrap();
        writeln!(gp, "set size ratio -1").unwrap();
        writeln!(gp, "set xrange [-2:2]").unwrap();
        writeln!(gp, "set yrange [-2:2]").unwrap();
        // 2 plots: direction space(x y dx dy), path
        writeln!(
            gp,
            "plot '-' using 1:2:3:4 with vectors notitle, '-' with linespoints notitle"
        )
        .unwrap();
        // Direction space
        for yi in 0..(2 * N) {
            let y = (N as f32 * -DV) + yi as f32 * DV;
            for xi in 0..(2 * N) {
                let x = (N as f32 * -DV) + xi as f32 * DV;
                let p = Point2::new(x, y);
                let (dest, _) = path.next_closest_tp(p, 0.0);
                let a = (dest - p).normalize() * 0.05;
                writeln!(gp, "{} {} {} {}", -p.y, p.x, -a.y, a.x).unwrap();
            }
        }
        writeln!(gp, "e").unwrap();
        // Plot path
        for p in path_points {
            writeln!(gp, "{} {}", -p.y, p.x).unwrap();
        }
        writeln!(gp, "e").unwrap();
        process.wait().unwrap();
    }
}
