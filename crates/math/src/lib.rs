//! # gta-math
//!
//! Bounded context: **mathematics**.
//!
//! Minimal, dependency-free linear algebra (column-major 4x4 matrices, right-handed,
//! +Y up, -Z forward) plus the deterministic hash-noise toolkit every other crate
//! builds on. Everything here is pure: no I/O, no globals, no allocation.

#![allow(clippy::too_many_arguments)]

pub mod noise;

pub mod aabb;
pub mod rng;

pub use noise::{
    fade, fbm2, fbm3, hash11, hash2, hash3, lattice2, lattice3, perlin2, perlin3, ridged2, smoothstep, unit, value2,
    value3, Noise2, Noise3,
};

pub use aabb::Aabb;
pub use rng::Rng;

pub const PI: f32 = core::f32::consts::PI;
pub const TAU: f32 = core::f32::consts::TAU;
pub const FRAC_PI_2: f32 = core::f32::consts::FRAC_PI_2;
pub const DEG_TO_RAD: f32 = core::f32::consts::PI / 180.0;
pub const RAD_TO_DEG: f32 = 180.0 / core::f32::consts::PI;

#[inline]
pub fn clamp(v: f32, lo: f32, hi: f32) -> f32 {
    if v < lo {
        lo
    } else if v > hi {
        hi
    } else {
        v
    }
}

#[inline]
pub fn lerpc(a: f32, b: f32, t: f32) -> f32 {
    a + (b - a) * t
}

/// Wraps an angle into `[-PI, PI)`.
pub fn wrap_angle(a: f32) -> f32 {
    let mut x = (a + PI) % TAU;
    if x < 0.0 {
        x += TAU;
    }
    x - PI
}

/// Frame-rate independent exponential smoothing towards a target.
#[inline]
pub fn damp(a: f32, b: f32, rate: f32, dt: f32) -> f32 {
    let t = 1.0 - (-rate * dt).exp();
    lerpc(a, b, t)
}

macro_rules! impl_common {
    ($t:ty, $n:expr) => {
        impl $t {
            #[inline]
            pub fn splat(v: f32) -> Self {
                let mut o = Self::ZERO;
                for i in 0..$n {
                    o.as_mut()[i] = v;
                }
                o
            }
            #[inline]
            pub fn dot(self, rhs: Self) -> f32 {
                let mut s = 0.0;
                for i in 0..$n {
                    s += self.as_ref()[i] * rhs.as_ref()[i];
                }
                s
            }
            #[inline]
            pub fn length_sq(self) -> f32 {
                self.dot(self)
            }
            #[inline]
            pub fn length(self) -> f32 {
                self.length_sq().sqrt()
            }
            #[inline]
            pub fn normalize(self) -> Self {
                let l = self.length();
                if l > 1e-12 {
                    self * (1.0 / l)
                } else {
                    self
                }
            }
            #[inline]
            pub fn lerp(self, rhs: Self, t: f32) -> Self {
                self + (rhs - self) * t
            }
            #[inline]
            pub fn min(self, rhs: Self) -> Self {
                let mut o = self;
                for i in 0..$n {
                    o.as_mut()[i] = o.as_ref()[i].min(rhs.as_ref()[i]);
                }
                o
            }
            #[inline]
            pub fn max(self, rhs: Self) -> Self {
                let mut o = self;
                for i in 0..$n {
                    o.as_mut()[i] = o.as_ref()[i].max(rhs.as_ref()[i]);
                }
                o
            }
            #[inline]
            pub fn map(self, f: impl Fn(f32) -> f32) -> Self {
                let mut o = self;
                for i in 0..$n {
                    o.as_mut()[i] = f(o.as_ref()[i]);
                }
                o
            }
        }
        impl core::ops::Add for $t {
            type Output = Self;
            #[inline]
            fn add(self, r: Self) -> Self {
                let mut o = self;
                for i in 0..$n {
                    o.as_mut()[i] += r.as_ref()[i];
                }
                o
            }
        }
        impl core::ops::Sub for $t {
            type Output = Self;
            #[inline]
            fn sub(self, r: Self) -> Self {
                let mut o = self;
                for i in 0..$n {
                    o.as_mut()[i] -= r.as_ref()[i];
                }
                o
            }
        }
        impl core::ops::Mul<f32> for $t {
            type Output = Self;
            #[inline]
            fn mul(self, r: f32) -> Self {
                self.map(|v| v * r)
            }
        }
        impl core::ops::Div<f32> for $t {
            type Output = Self;
            #[inline]
            fn div(self, r: f32) -> Self {
                self.map(|v| v / r)
            }
        }
        impl core::ops::Neg for $t {
            type Output = Self;
            #[inline]
            fn neg(self) -> Self {
                self * -1.0
            }
        }
        impl core::ops::AddAssign for $t {
            #[inline]
            fn add_assign(&mut self, r: Self) {
                *self = *self + r;
            }
        }
        impl core::ops::SubAssign for $t {
            #[inline]
            fn sub_assign(&mut self, r: Self) {
                *self = *self - r;
            }
        }
        impl core::ops::MulAssign<f32> for $t {
            #[inline]
            fn mul_assign(&mut self, r: f32) {
                *self = *self * r;
            }
        }
    };
}

macro_rules! impl_slice_ops {
    ($t:ty) => {
        impl AsRef<[f32]> for $t {
            #[inline]
            fn as_ref(&self) -> &[f32] {
                unsafe {
                    core::slice::from_raw_parts(self as *const Self as *const f32, core::mem::size_of::<$t>() / 4)
                }
            }
        }
        impl AsMut<[f32]> for $t {
            #[inline]
            fn as_mut(&mut self) -> &mut [f32] {
                unsafe {
                    core::slice::from_raw_parts_mut(self as *mut Self as *mut f32, core::mem::size_of::<$t>() / 4)
                }
            }
        }
    };
}

#[derive(Clone, Copy, Debug, Default, PartialEq)]
#[repr(C)]
pub struct Vec2 {
    pub x: f32,
    pub y: f32,
}
impl_common!(Vec2, 2);
impl_slice_ops!(Vec2);

impl Vec2 {
    pub const ZERO: Vec2 = Vec2 { x: 0.0, y: 0.0 };
    #[inline]
    pub fn new(x: f32, y: f32) -> Self {
        Self { x, y }
    }
    /// Perpendicular (rotate 90 deg in the plane).
    #[inline]
    pub fn perp(self) -> Self {
        Self { x: -self.y, y: self.x }
    }
    #[inline]
    pub fn from_vec3(v: Vec3) -> Self {
        Self { x: v.x, y: v.z }
    }
    #[inline]
    pub fn to_vec3(self, y: f32) -> Vec3 {
        Vec3 { x: self.x, y, z: self.y }
    }
    #[inline]
    pub fn floor(self) -> Self {
        self.map(|v| v.floor())
    }
    #[inline]
    pub fn distance(self, r: Self) -> f32 {
        (self - r).length()
    }
    #[inline]
    pub fn distance_sq(self, r: Self) -> f32 {
        (self - r).length_sq()
    }
    /// Squared distance from `p` to the segment `a`-`b`, plus the parametric position
    /// of the closest point (0..1). Parking stalls, frontages and lane checks use this.
    #[inline]
    pub fn segment_distance_sq(a: Self, b: Self, p: Self, t: &mut f32) -> f32 {
        let ab = b - a;
        let len2 = ab.length_sq();
        if len2 < 1e-9 {
            *t = 0.0;
            return (p - a).length_sq();
        }
        let u = ((p - a).dot(ab) / len2).clamp(0.0, 1.0);
        *t = u;
        (p - (a + ab * u)).length_sq()
    }
    /// Closest point on segment `a`-`b`.
    #[inline]
    pub fn closest_on_segment(a: Self, b: Self, p: Self) -> Self {
        let mut t = 0.0;
        let _ = Self::segment_distance_sq(a, b, p, &mut t);
        a + (b - a) * t
    }
    /// Rotates `self` by `ang` radians in the plane.
    #[inline]
    pub fn rot(self, a: f32) -> Self {
        let (s, c) = a.sin_cos();
        Self::new(self.x * c - self.y * s, self.x * s + self.y * c)
    }
}

#[derive(Clone, Copy, Debug, Default, PartialEq)]
#[repr(C)]
pub struct Vec3 {
    pub x: f32,
    pub y: f32,
    pub z: f32,
}
impl_common!(Vec3, 3);
impl_slice_ops!(Vec3);

impl Vec3 {
    pub const ZERO: Vec3 = Vec3 { x: 0.0, y: 0.0, z: 0.0 };
    pub const X: Vec3 = Vec3 { x: 1.0, y: 0.0, z: 0.0 };
    pub const Y: Vec3 = Vec3 { x: 0.0, y: 1.0, z: 0.0 };
    pub const Z: Vec3 = Vec3 { x: 0.0, y: 0.0, z: 1.0 };
    #[inline]
    pub fn new(x: f32, y: f32, z: f32) -> Self {
        Self { x, y, z }
    }
    #[inline]
    pub fn cross(self, r: Self) -> Self {
        Self {
            x: self.y * r.z - self.z * r.y,
            y: self.z * r.x - self.x * r.z,
            z: self.x * r.y - self.y * r.x,
        }
    }
    #[inline]
    pub fn xz(self) -> Vec2 {
        Vec2 { x: self.x, y: self.z }
    }
    #[inline]
    pub fn with_y(self, y: f32) -> Self {
        Self { x: self.x, y, z: self.z }
    }
    #[inline]
    pub fn length_xz(self) -> f32 {
        (self.x * self.x + self.z * self.z).sqrt()
    }
    #[inline]
    pub fn floor(self) -> Self {
        self.map(|v| v.floor())
    }
    /// Yaw (rotation about +Y) that this vector points along. 0 => +Z.
    #[inline]
    pub fn yaw(self) -> f32 {
        self.x.atan2(self.z)
    }
    #[inline]
    pub fn from_yaw(yaw: f32) -> Self {
        let (s, c) = yaw.sin_cos();
        Self { x: s, y: 0.0, z: c }
    }
    #[inline]
    pub fn from_arr(a: [f32; 3]) -> Self {
        Self::new(a[0], a[1], a[2])
    }

    pub fn to_arr(self) -> [f32; 3] {
        [self.x, self.y, self.z]
    }
}

#[derive(Clone, Copy, Debug, Default, PartialEq)]
#[repr(C)]
pub struct Vec4 {
    pub x: f32,
    pub y: f32,
    pub z: f32,
    pub w: f32,
}
impl_common!(Vec4, 4);
impl_slice_ops!(Vec4);

impl Vec4 {
    pub const ZERO: Vec4 = Vec4 { x: 0.0, y: 0.0, z: 0.0, w: 0.0 };
    #[inline]
    pub fn new(x: f32, y: f32, z: f32, w: f32) -> Self {
        Self { x, y, z, w }
    }
    #[inline]
    pub fn xyz(self) -> Vec3 {
        Vec3 { x: self.x, y: self.y, z: self.z }
    }
    #[inline]
    pub fn from_xyz(xyz: Vec3, w: f32) -> Self {
        Self { x: xyz.x, y: xyz.y, z: xyz.z, w }
    }
    /// 0..1 rgb to 8-bit bytes (clamped) — used for vertex colours.
    #[inline]
    pub fn rgb8(self) -> [u8; 3] {
        [
            (clamp(self.x, 0.0, 1.0) * 255.0) as u8,
            (clamp(self.y, 0.0, 1.0) * 255.0) as u8,
            (clamp(self.z, 0.0, 1.0) * 255.0) as u8,
        ]
    }
}

#[derive(Clone, Copy, Debug, Default, PartialEq)]
#[repr(C)]
pub struct Quat {
    pub x: f32,
    pub y: f32,
    pub z: f32,
    pub w: f32,
}

impl Quat {
    pub const IDENTITY: Quat = Quat { x: 0.0, y: 0.0, z: 0.0, w: 1.0 };

    #[inline]
    pub fn from_axis_angle(axis: Vec3, angle: f32) -> Self {
        let a = axis.normalize();
        let (s, c) = (angle * 0.5).sin_cos();
        Self { x: a.x * s, y: a.y * s, z: a.z * s, w: c }
    }
    #[inline]
    pub fn from_yaw(yaw: f32) -> Self {
        Self::from_axis_angle(Vec3::Y, yaw)
    }
    #[inline]
    pub fn mul(self, r: Self) -> Self {
        Self {
            x: self.w * r.x + self.x * r.w + self.y * r.z - self.z * r.y,
            y: self.w * r.y - self.x * r.z + self.y * r.w + self.z * r.x,
            z: self.w * r.z + self.x * r.y - self.y * r.x + self.z * r.w,
            w: self.w * r.w - self.x * r.x - self.y * r.y - self.z * r.z,
        }
    }
    #[inline]
    pub fn rotate(self, v: Vec3) -> Vec3 {
        let tx = 2.0 * (self.y * v.z - self.z * v.y);
        let ty = 2.0 * (self.z * v.x - self.x * v.z);
        let tz = 2.0 * (self.x * v.y - self.y * v.x);
        Vec3 {
            x: v.x + self.w * tx + (self.y * tz - self.z * ty),
            y: v.y + self.w * ty + (self.z * tx - self.x * tz),
            z: v.z + self.w * tz + (self.x * ty - self.y * tx),
        }
    }
    #[inline]
    pub fn normalize(self) -> Self {
        let l = (self.x * self.x + self.y * self.y + self.z * self.z + self.w * self.w).sqrt();
        if l > 1e-12 {
            Self { x: self.x / l, y: self.y / l, z: self.z / l, w: self.w / l }
        } else {
            Self::IDENTITY
        }
    }
    pub fn slerp(self, r: Self, t: f32) -> Self {
        let mut d = self.x * r.x + self.y * r.y + self.z * r.z + self.w * r.w;
        let (a, b) = if d < 0.0 {
            d = -d;
            (self, Self { x: -r.x, y: -r.y, z: -r.z, w: -r.w })
        } else {
            (self, r)
        };
        if d > 0.9995 {
            let o = Self {
                x: a.x + (b.x - a.x) * t,
                y: a.y + (b.y - a.y) * t,
                z: a.z + (b.z - a.z) * t,
                w: a.w + (b.w - a.w) * t,
            };
            return o.normalize();
        }
        let theta = d.clamp(-1.0, 1.0).acos();
        let st = theta.sin();
        let s0 = ((1.0 - t) * theta).sin() / st;
        let s1 = (theta * t).sin() / st;
        Self {
            x: a.x * s0 + b.x * s1,
            y: a.y * s0 + b.y * s1,
            z: a.z * s0 + b.z * s1,
            w: a.w * s0 + b.w * s1,
        }
    }
}

/// Column-major 4x4 matrix, `c[col][row]`.
#[derive(Clone, Copy, Debug, PartialEq)]
#[repr(C)]
pub struct Mat4 {
    pub c: [[f32; 4]; 4],
}

impl Default for Mat4 {
    #[inline]
    fn default() -> Self {
        Self::IDENTITY
    }
}

impl Mat4 {
    pub const IDENTITY: Mat4 = Mat4::from_cols(
        [1.0, 0.0, 0.0, 0.0],
        [0.0, 1.0, 0.0, 0.0],
        [0.0, 0.0, 1.0, 0.0],
        [0.0, 0.0, 0.0, 1.0],
    );

    #[inline]
    pub const fn from_cols(c0: [f32; 4], c1: [f32; 4], c2: [f32; 4], c3: [f32; 4]) -> Self {
        Mat4 { c: [c0, c1, c2, c3] }
    }

    /// Column-major floats, ready for a uniform buffer.
    #[inline]
    pub fn as_slice(&self) -> &[f32; 16] {
        unsafe { &*(self as *const Mat4 as *const [f32; 16]) }
    }

    #[inline]
    pub fn translate(t: Vec3) -> Self {
        Self::from_cols(
            [1.0, 0.0, 0.0, 0.0],
            [0.0, 1.0, 0.0, 0.0],
            [0.0, 0.0, 1.0, 0.0],
            [t.x, t.y, t.z, 1.0],
        )
    }
    #[inline]
    pub fn scale(s: Vec3) -> Self {
        Self::from_cols(
            [s.x, 0.0, 0.0, 0.0],
            [0.0, s.y, 0.0, 0.0],
            [0.0, 0.0, s.z, 0.0],
            [0.0, 0.0, 0.0, 1.0],
        )
    }
    #[inline]
    pub fn rot_y(a: f32) -> Self {
        let (s, c) = a.sin_cos();
        Self::from_cols([c, 0.0, -s, 0.0], [0.0, 1.0, 0.0, 0.0], [s, 0.0, c, 0.0], [0.0, 0.0, 0.0, 1.0])
    }

    #[inline]
    pub fn rot_x(a: f32) -> Self {
        let (s, c) = a.sin_cos();
        Self::from_cols(
            [1.0, 0.0, 0.0, 0.0],
            [0.0, c, s, 0.0],
            [0.0, -s, c, 0.0],
            [0.0, 0.0, 0.0, 1.0],
        )
    }

    /// `T * R(yaw) * S` — the bread-and-butter transform for city props.
    #[inline]
    pub fn compose(pos: Vec3, yaw: f32, scale: Vec3) -> Self {
        let (s, c) = yaw.sin_cos();
        Self::from_cols(
            [c * scale.x, 0.0, -s * scale.x, 0.0],
            [0.0, scale.y, 0.0, 0.0],
            [s * scale.z, 0.0, c * scale.z, 0.0],
            [pos.x, pos.y, pos.z, 1.0],
        )
    }

    #[inline]
    pub fn mul(&self, r: &Mat4) -> Mat4 {
        let mut out = [[0.0f32; 4]; 4];
        for (col, o) in out.iter_mut().enumerate() {
            for (row, v) in o.iter_mut().enumerate() {
                *v = self.c[0][row] * r.c[col][0]
                    + self.c[1][row] * r.c[col][1]
                    + self.c[2][row] * r.c[col][2]
                    + self.c[3][row] * r.c[col][3];
            }
        }
        Mat4 { c: out }
    }

    #[inline]
    pub fn transform_point(&self, p: Vec3) -> Vec3 {
        let m = self;
        Vec3 {
            x: m.c[0][0] * p.x + m.c[1][0] * p.y + m.c[2][0] * p.z + m.c[3][0],
            y: m.c[0][1] * p.x + m.c[1][1] * p.y + m.c[2][1] * p.z + m.c[3][1],
            z: m.c[0][2] * p.x + m.c[1][2] * p.y + m.c[2][2] * p.z + m.c[3][2],
        }
    }
    #[inline]
    pub fn transform_vector(&self, p: Vec3) -> Vec3 {
        let m = self;
        Vec3 {
            x: m.c[0][0] * p.x + m.c[1][0] * p.y + m.c[2][0] * p.z,
            y: m.c[0][1] * p.x + m.c[1][1] * p.y + m.c[2][1] * p.z,
            z: m.c[0][2] * p.x + m.c[1][2] * p.y + m.c[2][2] * p.z,
        }
    }
    #[inline]
    pub fn transform_normal(&self, n: Vec3) -> Vec3 {
        self.transform_vector(n).normalize()
    }

    /// Right-handed perspective (eye at origin, looking down -Z), depth 0..1 (wgpu).
    pub fn perspective(fov_y: f32, aspect: f32, near: f32, far: f32) -> Self {
        let f = 1.0 / (fov_y * 0.5).tan();
        let nf = 1.0 / (near - far);
        Self::from_cols(
            [f / aspect, 0.0, 0.0, 0.0],
            [0.0, f, 0.0, 0.0],
            [0.0, 0.0, (far + near) * nf, -1.0],
            [0.0, 0.0, 2.0 * far * near * nf, 0.0],
        )
    }

    /// Right-handed view matrix (camera at `eye` looking at `target`).
    pub fn look_at(eye: Vec3, target: Vec3, up: Vec3) -> Self {
        let f = (target - eye).normalize();
        let s = up.cross(f).normalize();
        let u = f.cross(s);
        // Rows: right / up / -forward, so the camera looks down view-space -Z.
        Self::from_cols(
            [s.x, u.x, -f.x, 0.0],
            [s.y, u.y, -f.y, 0.0],
            [s.z, u.z, -f.z, 0.0],
            [-s.dot(eye), -u.dot(eye), f.dot(eye), 1.0],
        )
    }

    /// Inverts an affine transform (rotation/scale + translation), which is all this
    /// project ever builds. Returns `None` when the 3x3 part is degenerate.
    pub fn invert(&self) -> Option<Mat4> {
        let a = self.as_slice();
        // Read the 3x3 part row-major: element(row, col) == a[row * 4 + col].
        let (r00, r01, r02) = (a[0], a[4], a[8]);
        let (r10, r11, r12) = (a[1], a[5], a[9]);
        let (r20, r21, r22) = (a[2], a[6], a[10]);
        let c00 = r11 * r22 - r12 * r21;
        let c01 = r12 * r20 - r10 * r22;
        let c02 = r10 * r21 - r11 * r20;
        let c10 = r02 * r21 - r01 * r22;
        let c11 = r00 * r22 - r02 * r20;
        let c12 = r01 * r20 - r00 * r21;
        let c20 = r01 * r12 - r02 * r11;
        let c21 = r02 * r10 - r00 * r12;
        let c22 = r00 * r11 - r01 * r10;
        let det = r00 * c00 + r01 * c01 + r02 * c02;
        if det.abs() < 1e-12 {
            return None;
        }
        let id = 1.0 / det;
        // inverse = adjugate/det (adjugate = transpose of cofactors)
        let o00 = c00 * id;
        let o01 = c10 * id;
        let o02 = c20 * id;
        let o10 = c01 * id;
        let o11 = c11 * id;
        let o12 = c21 * id;
        let o20 = c02 * id;
        let o21 = c12 * id;
        let o22 = c22 * id;
        // new translation = -inv3 * t
        let (tx, ty, tz) = (a[12], a[13], a[14]);
        let px = -(o00 * tx + o01 * ty + o02 * tz);
        let py = -(o10 * tx + o11 * ty + o12 * tz);
        let pz = -(o20 * tx + o21 * ty + o22 * tz);
        Some(Mat4::from_cols(
            [o00, o10, o20, 0.0],
            [o01, o11, o21, 0.0],
            [o02, o12, o22, 0.0],
            [px, py, pz, 1.0],
        ))
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const EPS: f32 = 1e-4;

    #[test]
    fn vec_arith() {
        let a = Vec3::new(1.0, 2.0, 3.0);
        let b = Vec3::new(4.0, 5.0, 6.0);
        assert_eq!((a + b).to_arr(), [5.0, 7.0, 9.0]);
        assert_eq!((b - a).to_arr(), [3.0, 3.0, 3.0]);
        assert_eq!(a.dot(b), 32.0);
        assert_eq!(Vec3::X.cross(Vec3::Y), Vec3::Z);
        assert!((Vec3::new(3.0, 0.0, 4.0).length() - 5.0).abs() < EPS);
        assert!((Vec3::new(0.0, 7.0, 0.0).normalize() - Vec3::Y).length() < EPS);
        assert_eq!(Vec2::new(2.0, 3.0).perp(), Vec2::new(-3.0, 2.0));
    }

    #[test]
    fn yaw_roundtrip() {
        for a in [-3.0f32, -1.7, -0.3, 0.0, 0.7, 2.5] {
            let v = Vec3::from_yaw(a);
            assert!(wrap_angle(v.yaw() - a).abs() < 1e-5, "yaw {a}");
        }
    }

    #[test]
    fn compose_matches_trs() {
        let p = Vec3::new(11.0, 4.0, -2.0);
        let yaw = 0.7;
        let s = Vec3::new(2.0, 1.5, 2.5);
        let c = Mat4::compose(p, yaw, s);
        // local +Z scaled then rotated then translated
        let local = Vec3::new(0.0, 0.0, 1.0);
        let expect = Mat4::translate(p).mul(&Mat4::rot_y(yaw)).mul(&Mat4::scale(s)).transform_point(Vec3::Z);
        assert!((c.transform_point(Vec3::Z) - expect).length() < 1e-3, "{yaw}");
        let _ = local;
    }

    #[test]
    fn rot_y_directions() {
        // rot_y(+90deg) maps +Z onto +X
        let v = Mat4::rot_y(FRAC_PI_2).transform_vector(Vec3::Z);
        assert!((v - Vec3::X).length() < EPS, "{v:?}");
    }

    #[test]
    fn quat_matches_mat() {
        let q = Quat::from_yaw(0.9);
        let m = Mat4::rot_y(0.9);
        assert!((q.rotate(Vec3::Z) - m.transform_vector(Vec3::Z)).length() < EPS);
    }

    #[test]
    fn slerp_endpoints_and_mid() {
        let a = Quat::from_yaw(-0.8);
        let b = Quat::from_yaw(0.6);
        let s0 = a.slerp(b, 0.0);
        let s1 = a.slerp(b, 1.0);
        assert!((wrap_angle(yaw_of(s0) - yaw_of(a))).abs() < 1e-4);
        assert!((wrap_angle(yaw_of(s1) - yaw_of(b))).abs() < 1e-4);
        let mid = a.slerp(b, 0.5);
        assert!((yaw_of(mid) - (-0.1)).abs() < 1e-4, "{}", yaw_of(mid));
    }

    fn yaw_of(q: Quat) -> f32 {
        q.rotate(Vec3::Z).yaw()
    }

    #[test]
    fn look_at_maps_eye_to_origin_and_target_to_minus_z() {
        let eye = Vec3::new(3.0, 2.0, -5.0);
        let target = Vec3::new(0.0, 1.0, 4.0);
        let v = Mat4::look_at(eye, target, Vec3::Y);
        let o = v.transform_point(eye);
        assert!(o.length() < 1e-3, "eye should map to origin, got {o:?}");
        let t = v.transform_point(target);
        assert!(t.x.abs() < 1e-3 && t.y.abs() < 1e-3, "target on the view axis: {t:?}");
        assert!(t.z < 0.0, "target should be in front (-Z): {t:?}");
        // A point directly above the eye stays up in view space.
        let above = v.transform_point(eye + Vec3::new(0.0, 5.0, 0.0));
        assert!(above.y > 0.0, "world up should be view up: {above:?}");
        // Distance along the view axis is preserved.
        let d = (target - eye).length();
        assert!((t.z + d).abs() < 1e-2, "depth {} vs {}", t.z, d);
    }

    #[test]
    fn invert_roundtrip() {
        let m = Mat4::compose(Vec3::new(1.0, 2.0, 3.0), 0.4, Vec3::new(2.0, 3.0, 4.0));
        let inv = m_invert(&m).unwrap();
        let p = Vec3::new(1.5, -2.0, 0.25);
        let back = inv.transform_point(m.transform_point(p));
        assert!((back - p).length() < 1e-2, "{back:?} vs {p:?}");
    }

    fn m_invert(m: &Mat4) -> Option<Mat4> {
        m.invert()
    }
}
