//! Bounded context: **bounding volumes**.
//!
//! Axis-aligned boxes used for collision queries, spatial-index buckets and frustum
//! culling. Kept tiny so every other crate can depend on it.

use crate::{Vec2, Vec3};

/// Axis-aligned bounding box.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct Aabb {
    pub min: Vec3,
    pub max: Vec3,
}

impl Aabb {
    /// Empty box — useful as an accumulator seed.
    pub const INVALID: Aabb = Aabb {
        min: Vec3 { x: f32::MAX, y: f32::MAX, z: f32::MAX },
        max: Vec3 { x: f32::MIN, y: f32::MIN, z: f32::MIN },
    };

    #[inline]
    pub fn new(min: Vec3, max: Vec3) -> Self {
        Aabb { min, max }
    }

    #[inline]
    pub fn from_center_size(center: Vec3, size: Vec3) -> Self {
        let half = size * 0.5;
        Aabb { min: center - half, max: center + half }
    }

    /// Box on the ground plane: footprint centre (xz), width/depth and a height.
    #[inline]
    pub fn footprint(center_xz: Vec2, width: f32, depth: f32, height: f32) -> Self {
        Aabb {
            min: Vec3::new(center_xz.x - width * 0.5, 0.0, center_xz.y - depth * 0.5),
            max: Vec3::new(center_xz.x + width * 0.5, height, center_xz.y + depth * 0.5),
        }
    }

    #[inline]
    pub fn center(&self) -> Vec3 {
        (self.min + self.max) * 0.5
    }

    #[inline]
    pub fn half(&self) -> Vec3 {
        (self.max - self.min) * 0.5
    }

    #[inline]
    pub fn size(&self) -> Vec3 {
        self.max - self.min
    }

    #[inline]
    pub fn radius(&self) -> f32 {
        self.half().length()
    }

    #[inline]
    pub fn is_valid(&self) -> bool {
        self.min.x <= self.max.x && self.min.y <= self.max.y && self.min.z <= self.max.z
    }

    #[inline]
    pub fn contains_point(&self, p: Vec3) -> bool {
        p.x >= self.min.x && p.x <= self.max.x && p.y >= self.min.y && p.y <= self.max.y && p.z >= self.min.z && p.z <= self.max.z
    }

    /// 2D (xz) overlap test — the plane we care about for city layout.
    #[inline]
    pub fn overlaps_xz(&self, o: &Aabb) -> bool {
        self.min.x <= o.max.x && self.max.x >= o.min.x && self.min.z <= o.max.z && self.max.z >= o.min.z
    }

    /// 3D overlap test.
    #[inline]
    pub fn intersects(&self, o: &Aabb) -> bool {
        self.overlaps_xz(o) && self.min.y <= o.max.y && self.max.y >= o.min.y
    }

    #[inline]
    pub fn expanded(&self, m: f32) -> Aabb {
        let v = Vec3::splat(m);
        Aabb { min: self.min - v, max: self.max + v }
    }

    /// Grows the box in place to include `p`.
    pub fn grow(&mut self, p: Vec3) {
        self.min = self.min.min(p);
        self.max = self.max.max(p);
    }

    /// Merged bounds of two boxes.
    pub fn merged(&self, o: &Aabb) -> Aabb {
        Aabb { min: self.min.min(o.min), max: self.max.max(o.max) }
    }

    /// Distance from a point to the box surface (0 when inside).
    pub fn distance_point(&self, p: Vec3) -> f32 {
        let dx = (self.min.x - p.x).max(0.0).max(p.x - self.max.x);
        let dy = (self.min.y - p.y).max(0.0).max(p.y - self.max.y);
        let dz = (self.min.z - p.z).max(0.0).max(p.z - self.max.z);
        (dx * dx + dy * dy + dz * dz).sqrt()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn basics() {
        let b = Aabb::from_center_size(Vec3::new(1.0, 2.0, 3.0), Vec3::new(2.0, 4.0, 6.0));
        assert_eq!(b_center(&b), Vec3::new(1.0, 2.0, 3.0));
        assert_eq!(b_size(&b), Vec3::new(2.0, 4.0, 6.0));
        assert!(!Aabb::INVALID.is_valid());
        assert!(b.is_valid());
    }

    #[test]
    fn contains_and_overlap() {
        let a = Aabb::new(Vec3::new(0.0, 0.0, 0.0), Vec3::new(2.0, 1.0, 2.0));
        assert!(a.contains_point(Vec3::new(1.0, 0.5, 1.0)));
        assert!(!a.contains_point(Vec3::new(2.5, 0.5, 1.0)));
        let b = Aabb::new(Vec3::new(1.0, 0.0, 1.0), Vec3::new(4.0, 1.0, 3.0));
        assert!(a.intersects(&b));
        let c = Aabb::new(Vec3::new(5.0, 0.0, 5.0), Vec3::new(6.0, 1.0, 6.0));
        assert!(!a.intersects(&c));
        assert!(!a.overlaps_xz(&c));
    }

    #[test]
    fn grow_and_expand() {
        let mut b = Aabb::new(Vec3::ZERO, Vec3::new(1.0, 1.0, 1.0));
        b.grow(Vec3::new(-3.0, 0.0, 3.0));
        assert_eq!(b.min.x, -3.0);
        assert_eq!(b.max.z, 3.0);
        let e = b.expanded(0.5);
        assert_eq!(e.min.x, -3.5);
        let m = b.merged(&Aabb::new(Vec3::new(-10.0, 0.0, 0.0), Vec3::new(0.0, 1.0, 1.0)));
        assert_eq!(m.min.x, -10.0);
    }

    #[test]
    fn distance_outside_and_inside() {
        let b = Aabb::new(Vec3::ZERO, Vec3::new(2.0, 2.0, 2.0));
        assert_eq!(b.distance_point(Vec3::new(1.0, 1.0, 1.0)), 0.0);
        let d = b.distance_point(Vec3::new(3.0, 1.0, 1.0));
        assert!((d - 1.0).abs() < 1e-6, "{d}");
        let d2 = b.distance_point(Vec3::new(3.0, 3.0, 1.0));
        assert!((d2 - (2.0f32).sqrt()).abs() < 1e-5, "{d2}");
    }

    // Small wrappers keep the assertions readable.
    fn b_center(b: &Aabb) -> Vec3 {
        b.center()
    }
    fn b_size(b: &Aabb) -> Vec3 {
        b.size()
    }
}
