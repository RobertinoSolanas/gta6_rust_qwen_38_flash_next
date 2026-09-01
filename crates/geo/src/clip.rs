//! Bounded context: **2D polygon clipping**.
//!
//! Sutherland–Hodgman clipping of a convex polygon against an axis-aligned
//! rectangle. The city uses this to cut building footprints to their lot and to
//! derive corner-lot sidewalk shapes. Output stays convex so triangulation is a fan.

use gta_math::Vec2;

/// Axis-aligned rectangle in the xz plane (struct `.y` == world `z`).
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct Rect {
    pub min: Vec2,
    pub max: Vec2,
}

impl Rect {
    #[inline]
    pub fn new(min: Vec2, max: Vec2) -> Self {
        Rect { min, max }
    }
    #[inline]
    pub fn centered(c: Vec2, w: f32, d: f32) -> Self {
        Rect {
            min: Vec2::new(c.x - 0.5 * w, c.y - 0.5 * d),
            max: Vec2::new(c.x + 0.5 * w, c.y + 0.5 * d),
        }
    }
    #[inline]
    pub fn width(&self) -> f32 {
        self.max.x - self.min.x
    }
    #[inline]
    pub fn depth(&self) -> f32 {
        self.max.y - self.min.y
    }
    #[inline]
    pub fn contains(&self, p: Vec2) -> bool {
        p.x >= self.min.x && p.x <= self.max.x && p.y >= self.min.y && p.y <= self.max.y
    }
    #[inline]
    pub fn inflated(&self, m: f32) -> Rect {
        Rect { min: Vec2::new(self.min.x - m, self.min.y - m), max: Vec2::new(self.max.x + m, self.max.y + m) }
    }
}

#[inline]
fn lerp2(a: Vec2, b: Vec2, t: f32) -> Vec2 {
    Vec2::new(a.x + (b.x - a.x) * t, a.y + (b.y - a.y) * t)
}

/// One Sutherland–Hodgman pass against the half-plane `inside(p)`.
///
/// `inside` must be a half-plane test; the intersection parameter is computed with
/// the matching axis denominator, so `inside` and the intersection formula must agree.
fn clip_half_plane(poly: &[Vec2], inside: impl Fn(Vec2) -> bool, inter: impl Fn(Vec2, Vec2) -> Vec2) -> Vec<Vec2> {
    let n = poly.len();
    let mut out = Vec::with_capacity(n + 2);
    for i in 0..n {
        let a = poly[i];
        let b = poly[(i + 1) % n];
        let (ai, bi) = (inside(a), inside(b));
        if ai {
            out.push(a);
            if !bi {
                out.push(inter(a, b));
            }
        } else if bi {
            out.push(inter(a, b));
        }
    }
    out
}

/// Clips a convex polygon to `rect`, dropping slivers below `min_area`.
pub fn clip_polygon(subject: &[Vec2], rect: Rect) -> Vec<Vec2> {
    let mut poly: Vec<Vec2> = subject.to_vec();
    if poly.len() < 3 {
        return Vec::new();
    }
    poly = clip_half_plane(&poly, |p| p.x >= rect.min.x, |a, b| lerp2(a, b, (rect.min.x - a.x) / (b.x - a.x)));
    poly = clip_half_plane(&poly, |p| p.x <= rect.max.x, |a, b| lerp2(a, b, (rect.max.x - a.x) / (b.x - a.x)));
    poly = clip_half_plane(&poly, |p| p.y >= rect.min.y, |a, b| lerp2(a, b, (rect.min.y - a.y) / (b.y - a.y)));
    poly = clip_half_plane(&poly, |p| p.y <= rect.max.y, |a, b| lerp2(a, b, (rect.max.y - a.y) / (b.y - a.y)));
    if poly.len() < 3 {
        return Vec::new();
    }
    poly
}

/// Signed area (positive when CCW).
pub fn polygon_area(poly: &[Vec2]) -> f32 {
    if poly.len() < 3 {
        return 0.0;
    }
    let mut a = 0.0f32;
    for i in 0..poly.len() {
        let p = poly[i];
        let q = poly[(i + 1) % poly.len()];
        a += p.x * q.y - q.x * p.y;
    }
    0.5 * a
}

#[inline]
pub fn area_abs(poly: &[Vec2]) -> f32 {
    polygon_area(poly).abs()
}

/// Area-weighted centroid (falls back to vertex average for degenerate polygons).
pub fn centroid(poly: &[Vec2]) -> Vec2 {
    let n = poly.len();
    if n == 0 {
        return Vec2::ZERO;
    }
    let mut a = 0.0f32;
    let mut cx = 0.0f32;
    let mut cy = 0.0f32;
    for i in 0..n {
        let p = poly[i];
        let q = poly[(i + 1) % n];
        let cross = p.x * q.y - q.x * p.y;
        a += cross;
        cx += (p.x + q.x) * cross;
        cy += (p.y + q.y) * cross;
    }
    if a.abs() < 1e-9 {
        let s = poly.iter().fold(Vec2::ZERO, |acc, p| acc + *p);
        return s / n as f32;
    }
    let f = 1.0 / (3.0 * a);
    Vec2::new(cx * f, cy * f)
}

/// Axis-aligned bounds of a polygon.
pub fn bounds(poly: &[Vec2]) -> Rect {
    let mut r = Rect { min: Vec2::splat(f32::MAX), max: Vec2::splat(f32::MIN) };
    for p in poly {
        r.min = r.min.min(*p);
        r.max = r.max.max(*p);
    }
    r
}

#[cfg(test)]
mod tests {
    use super::*;

    fn square(c: Vec2, h: f32) -> Vec<Vec2> {
        vec![
            Vec2::new(c.x - h, c.y - h),
            Vec2::new(c.x + h, c.y - h),
            Vec2::new(c.x + h, c.y + h),
            Vec2::new(c.x - h, c.y + h),
        ]
    }

    #[test]
    fn area_and_centroid_of_unit_square() {
        let s = square(Vec2::new(1.0, 2.0), 1.0);
        assert!((area_abs(&s) - 4.0).abs() < 1e-5);
        let c = centroid(&s);
        assert!((c.x - 1.0).abs() < 1e-5 && (c.y - 2.0).abs() < 1e-5, "{c:?}");
    }

    #[test]
    fn clip_fully_inside_is_identity() {
        let s = square(Vec2::ZERO, 1.0);
        let c = clip_polygon(&s, Rect::centered(Vec2::ZERO, 10.0, 10.0));
        assert_eq!(c.len(), 4);
        assert!((area_abs(&c) - 4.0).abs() < 1e-4);
    }

    #[test]
    fn clip_fully_outside_yields_nothing() {
        let s = square(Vec2::new(50.0, 50.0), 1.0);
        let c = clip_polygon(&s, Rect::centered(Vec2::ZERO, 10.0, 10.0));
        assert!(c.len() < 3);
    }

    #[test]
    fn clip_half_of_a_square() {
        let s = square(Vec2::ZERO, 2.0);
        let c = clip_polygon(&s, Rect::new(Vec2::new(-1.0, -1.0), Vec2::new(10.0, 10.0)));
        // x >= -1 and y >= -1 inside a 4x4 square -> 3x3 = 9
        assert!((area_abs(&c) - 9.0).abs() < 1e-4, "{} {:?}", area_abs(&c), c);
    }

    #[test]
    fn clip_corner_lot() {
        let s = square(Vec2::new(5.0, 5.0), 3.0); // spans 2..8
        let c = clip_polygon(&s, Rect::new(Vec2::new(0.0, 0.0), Vec2::new(4.0, 4.0)));
        assert!((area_abs(&c) - 4.0).abs() < 1e-4, "{}", area_abs(&c));
        for p in &c {
            assert!(p.x <= 4.0001 && p.y <= 4.0001 && p.x >= -0.0001 && p.y >= -0.0001, "{p:?}");
        }
    }

    #[test]
    fn bounds_cover_points() {
        let s = square(Vec2::new(3.0, -2.0), 1.5);
        let r = bounds(&s);
        assert!((r.min.x - 1.5).abs() < 1e-5);
        assert!((r.max.y + 0.5).abs() < 1e-5);
        for p in &s {
            assert!(r.contains(*p));
        }
    }
}
