//! Bounded context: **procedural primitives**.
//!
//! The shape vocabulary shared by the city, the vehicles and the characters. These are
//! deliberately not just boxes: extruded polygons, tapered cylinders, profile
//! extrusions, wheels, UV spheres and capsules are what make the city read as
//! *designed* geometry rather than a debug scene.
//!
//! Conventions: `+Y` up, metres, outward normals. `base` = bottom of a solid,
//! `centre` = middle of a volume. Because [`MeshBuilder::tri_n`] /
//! [`MeshBuilder::quad_n`] auto-correct winding, callers only have to get the *normal*
//! right.

use gta_math::{Mat4, Quat, Vec2, Vec3, TAU};

use crate::clip::{centroid, Rect};
use crate::tri::{face_normal, MeshBuilder, Paint};

/// Closed box (all six faces) resting on `base`.
pub fn box_solid(mb: &mut MeshBuilder, base: Vec3, size: Vec3, paint: &Paint) {
    let (x0, y0, z0) = (base.x, base.y, base.z);
    let (x1, y1, z1) = (base.x + size.x, base.y + size.y, base.z + size.z);
    let p = |x: f32, y: f32, z: f32| Vec3::new(x, y, z);
    let up = Vec3::Y;
    let down = -Vec3::Y;
    mb.quad_n(p(x0, y0, z1), p(x0, y0, z0), p(x0, y1, z0), p(x0, y1, z1), Some(-Vec3::X), paint);
    mb.quad_n(p(x1, y0, z0), p(x1, y0, z1), p(x1, y1, z1), p(x1, y1, z0), Some(Vec3::X), paint);
    mb.quad_n(p(x1, y0, z0), p(x0, y0, z0), p(x0, y1, z0), p(x1, y1, z0), Some(-Vec3::Z), paint);
    mb.quad_n(p(x0, y0, z1), p(x1, y0, z1), p(x1, y1, z1), p(x0, y1, z1), Some(Vec3::Z), paint);
    mb.quad_n(p(x0, y1, z0), p(x1, y1, z0), p(x1, y1, z1), p(x0, y1, z1), Some(up), paint);
    mb.quad_n(p(x0, y0, z0), p(x1, y0, z0), p(x1, y0, z1), p(x0, y0, z1), Some(down), paint);
}

/// The four side walls of a box, no caps (roofs get their own material).
pub fn box_walls(mb: &mut MeshBuilder, base: Vec3, size: Vec3, paint: &Paint) {
    let (x0, y0, z0) = (base.x, base.y, base.z);
    let (x1, y1, z1) = (base.x + size.x, base.y + size.y, base.z + size.z);
    let p = |x: f32, y: f32, z: f32| Vec3::new(x, y, z);
    mb.quad_n(p(x0, y0, z1), p(x0, y0, z0), p(x0, y1, z0), p(x0, y1, z1), Some(-Vec3::X), paint);
    mb.quad_n(p(x1, y0, z0), p(x1, y0, z1), p(x1, y1, z1), p(x1, y1, z0), Some(Vec3::X), paint);
    mb.quad_n(p(x1, y0, z0), p(x0, y0, z0), p(x0, y1, z0), p(x1, y1, z0), Some(-Vec3::Z), paint);
    mb.quad_n(p(x0, y0, z1), p(x1, y0, z1), p(x1, y1, z1), p(x0, y1, z1), Some(Vec3::Z), paint);
}

/// One axis-aligned quad in the XZ plane at height `y`, normal +Y.
pub fn ground_quad(mb: &mut MeshBuilder, r: Rect, y: f32, paint: &Paint) {
    let p0 = Vec3::new(r.min.x, y, r.min.y);
    let p1 = Vec3::new(r.max.x, y, r.min.y);
    let p2 = Vec3::new(r.max.x, y, r.max.y);
    let p3 = Vec3::new(r.min.x, y, r.max.y);
    mb.quad_n(p0, p1, p2, p3, Some(Vec3::Y), paint);
}

/// Caps a convex polygon with a triangle fan. `up` selects the normal direction.
pub fn cap_polygon(mb: &mut MeshBuilder, poly: &[Vec2], y: f32, up: bool, paint: &Paint) {
    if poly.len() < 3 {
        return;
    }
    let n = poly.len();
    let c = centroid(poly);
    let c3 = Vec3::new(c.x, y, c.y);
    let nrm = if up { Vec3::Y } else { -Vec3::Y };
    for i in 0..poly.len() {
        let a = poly[i];
        let b = poly[(i + 1) % n];
        let va = Vec3::new(a.x, y, a.y);
        let vb = Vec3::new(b.x, y, b.y);
        mb.tri_n(c3, va, vb, nrm, paint);
    }
}

/// Flat convex polygon slab (normal +Y).
#[inline]
pub fn flat_polygon(mb: &mut MeshBuilder, poly: &[Vec2], y: f32, paint: &Paint) {
    cap_polygon(mb, poly, y, true, paint);
}

/// Extrudes a convex polygon upward from `base_y` by `h`.
///
/// The workhorse for blocks, plazas, sidewalk pads, podiums and L-shaped footprints.
pub fn extrude_polygon(
    mb: &mut MeshBuilder,
    poly: &[Vec2],
    base_y: f32,
    h: f32,
    paint: &Paint,
    cap_top: bool,
    cap_bottom: bool,
) {
    if poly.len() < 3 || h <= 0.0 {
        return;
    }
    let n = poly.len();
    for i in 0..poly.len() {
        let a = poly[i];
        let b = poly[(i + 1) % n];
        let (ex, ez) = (b.x - a.x, b.y - a.y);
        let nn = Vec3::new(ez, 0.0, -ex).normalize();
        if nn.length_sq() < 1e-8 {
            continue;
        }
        let pa = Vec3::new(a.x, base_y, a.y);
        let pb = Vec3::new(b.x, base_y, b.y);
        let pb_top = Vec3::new(b.x, base_y + h, b.y);
        let pa_top = Vec3::new(a.x, base_y + h, a.y);
        mb.quad_n(pa, pb, pb_top, pa_top, Some(nn), paint);
    }
    if cap_top {
        cap_polygon(mb, poly, base_y + h, true, paint);
    }
    if cap_bottom {
        cap_polygon(mb, poly, base_y, false, paint);
    }
}

/// Straight cylinder with `segs` sides and both caps.
#[inline]
pub fn cylinder(mb: &mut MeshBuilder, base: Vec3, radius: f32, height: f32, segs: usize, paint: &Paint) {
    tapered_cylinder(mb, base, radius, radius, height, segs, paint, true);
}

/// Cylinder tapering from `r0` (bottom) to `r1` (top); `r1 == 0` makes a cone.
pub fn tapered_cylinder(
    mb: &mut MeshBuilder,
    base: Vec3,
    r0: f32,
    r1: f32,
    height: f32,
    segs: usize,
    paint: &Paint,
    caps: bool,
) {
    let segs = segs.max(3);
    let top = base.y + height;
    let mut ring0 = Vec::with_capacity(segs);
    let mut ring1 = Vec::with_capacity(segs);
    for i in 0..segs {
        let a = (i as f32 / segs as f32) * TAU;
        let (s, c) = a.sin_cos();
        ring0.push(Vec3::new(base.x + c * r0, base.y, base.z + s * r0));
        ring1.push(Vec3::new(base.x + c * r1, top, base.z + s * r1));
    }
    for i in 0..segs {
        let j = (i + 1) % segs;
        let a = ring0[i];
        let b = ring0[j];
        let ct = ring1[j];
        let d = ring1[i];
        let na = Vec3::new(a.x - base.x, 0.0, a.z - base.z).normalize();
        let nb = Vec3::new(b.x - base.x, 0.0, b.z - base.z).normalize();
        let n = (na + nb) * 0.5;
        if r1 > 1e-5 {
            mb.quad_n(a, b, ct, d, Some(n), paint);
        } else {
            mb.tri_n(a, b, d, n, paint);
        }
    }
    if caps {
        let ct = Vec3::new(base.x, top, base.z);
        let cb = Vec3::new(base.x, base.y, base.z);
        for i in 0..segs {
            let j = (i + 1) % segs;
            if r1 > 1e-5 {
                mb.tri_n(ct, ring1[i], ring1[j], Vec3::Y, paint);
            }
            if r0 > 1e-5 {
                mb.tri_n(cb, ring0[j], ring0[i], -Vec3::Y, paint);
            }
        }
    }
}

/// Cone with apex at `base.y + height`.
#[inline]
pub fn cone(mb: &mut MeshBuilder, base: Vec3, radius: f32, height: f32, segs: usize, paint: &Paint) {
    tapered_cylinder(mb, base, radius, 0.0, height, segs, paint, true);
}

/// Ellipsoid with per-triangle outward normals.
pub fn ellipsoid(mb: &mut MeshBuilder, centre: Vec3, radii: Vec3, seg: usize, rings: usize, paint: &Paint) {
    let seg = seg.max(4);
    let rings = rings.max(2);
    let pt = |i: usize, j: usize| -> Vec3 {
        let v = j as f32 / rings as f32;
        let u = i as f32 / seg as f32;
        let phi = v * std::f32::consts::PI;
        let theta = u * TAU;
        let (sp, cp) = phi.sin_cos();
        let (st, ct) = theta.sin_cos();
        Vec3::new(
            centre.x + radii.x * sp * ct,
            centre.y + radii.y * cp,
            centre.z + radii.z * sp * st,
        )
    };
    for j in 0..rings {
        for i in 0..seg {
            let a = pt(i, j);
            let b = pt(i + 1, j);
            let c = pt(i + 1, j + 1);
            let d = pt(i, j + 1);
            for (p, q, r) in [(a, b, c), (a, c, d)] {
                let n = ((p + q + r) * (1.0 / 3.0) - centre).normalize();
                mb.tri_n(p, q, r, n, paint);
            }
        }
    }
}

/// UV sphere.
#[inline]
pub fn sphere(mb: &mut MeshBuilder, centre: Vec3, radius: f32, seg: usize, rings: usize, paint: &Paint) {
    ellipsoid(mb, centre, Vec3::splat(radius), seg, rings, paint);
}

/// A squashed ellipsoid — bushes, hedges, bushes and shrubs.
#[inline]
pub fn blob(mb: &mut MeshBuilder, centre: Vec3, radius: f32, squash: f32, seg: usize, rings: usize, paint: &Paint) {
    ellipsoid(mb, centre, Vec3::new(radius, radius * squash, radius), seg, rings, paint);
}

/// Rotation taking unit `a` onto unit `b`.
pub fn quat_from_to(a: Vec3, b: Vec3) -> Quat {
    let a = a.normalize();
    let b = b.normalize();
    let d = a.dot(b);
    if d > 0.999999 {
        return Quat::IDENTITY;
    }
    if d < -0.999999 {
        let axis = Vec3::X.cross(a);
        let axis = if axis.length_sq() < 1e-6 { Vec3::Z.cross(a) } else { axis };
        return Quat::from_axis_angle(axis, gta_math::PI);
    }
    let axis = a.cross(b);
    Quat { x: axis.x, y: axis.y, z: axis.z, w: 1.0 + d }.normalize()
}

/// Capsule between `a` and `b`: a cylinder shaft plus two domed ends. Limbs and torsos.
pub fn capsule(mb: &mut MeshBuilder, a: Vec3, b: Vec3, radius: f32, seg: usize, paint: &Paint) {
    let seg = seg.max(6);
    let axis = b - a;
    let len = axis.length();
    if len < 1e-5 {
        sphere(mb, a, radius, seg, seg / 2, paint);
        return;
    }
    let mut local = MeshBuilder::new();
    tapered_cylinder(&mut local, Vec3::ZERO, radius, radius, len, seg, paint, false);
    for i in 0..seg {
        let a0 = (i as f32 / seg as f32) * TAU;
        let a1 = ((i + 1) as f32 / seg as f32) * TAU;
        let (s0, c0) = a0.sin_cos();
        let (s1, c1) = a1.sin_cos();
        for (base_y, sign) in [(0.0f32, -1.0f32), (len, 1.0f32)] {
            let p0 = Vec3::new(c0 * radius, base_y, s0 * radius);
            let p1 = Vec3::new(c1 * radius, base_y, s1 * radius);
            let apex = Vec3::new(0.0, base_y + sign * radius * 0.85, 0.0);
            let n = face_normal(apex, p0, p1);
            local.tri_n(apex, p0, p1, n, paint);
        }
    }
    let q = quat_from_to(Vec3::Y, axis * (1.0 / len));
    local.apply_rot(&q);
    mb.merge_offset(&local, a);
}

/// A box beam oriented along `a -> b` (rails, lamp arms, girders).
pub fn beam(mb: &mut MeshBuilder, a: Vec3, b: Vec3, thickness: f32, paint: &Paint) {
    let d = b - a;
    let len = d.length();
    if len < 1e-5 {
        return;
    }
    let mut local = MeshBuilder::new();
    box_solid(
        &mut local,
        Vec3::new(-thickness * 0.5, -thickness * 0.5, 0.0),
        Vec3::new(thickness, thickness, len),
        paint,
    );
    let yaw = d.yaw();
    let pitch = -(d.y / len.max(1e-6)).asin();
    let m = Mat4::translate((a + b) * 0.5)
        .mul(&Mat4::rot_y(yaw))
        .mul(&Mat4::rot_x(pitch));
    mb.merge_mat(&local, &m);
}

/// A road wheel: tyre barrel, rim discs and five spokes, axis along local +X.
///
/// `roll` spins the rim (wheel-spin animation); `centre` is the hub centre.
pub fn wheel(
    mb: &mut MeshBuilder,
    centre: Vec3,
    radius: f32,
    width: f32,
    segs: usize,
    tyre: &Paint,
    rim: &Paint,
    roll: f32,
) {
    let segs = segs.max(8);
    let half = width * 0.5;
    for i in 0..segs {
        let a0 = (i as f32 / segs as f32) * TAU;
        let a1 = ((i + 1) as f32 / segs as f32) * TAU;
        let (s0, c0) = a0.sin_cos();
        let (s1, c1) = a1.sin_cos();
        let n = (Vec3::new(0.0, c0, s0) + Vec3::new(0.0, c1, s1)) * 0.5;
        let q0 = Vec3::new(centre.x - half, centre.y + radius * c0, centre.z + radius * s0);
        let q1 = Vec3::new(centre.x + width * 0.5, centre.y + radius * c0, centre.z + radius * s0);
        let q2 = Vec3::new(centre.x + width * 0.5, centre.y + radius * c1, centre.z + radius * s1);
        let q3 = Vec3::new(centre.x - width * 0.5, centre.y + radius * c1, centre.z + radius * s1);
        mb.quad_n(q0, q1, q2, q3, Some(n), tyre);
    }
    for side in [-1.0f32, 1.0] {
        let x = centre.x + side * (half - 0.004);
        let rr = radius * 0.62;
        let c = Vec3::new(x, centre.y, centre.z);
        let nrm = Vec3::new(side, 0.0, 0.0);
        for i in 0..segs {
            let a0 = (i as f32 / segs as f32) * TAU + roll;
            let a1 = ((i + 1) as f32 / segs as f32) * TAU + roll;
            let p0 = Vec3::new(x, centre.y + rr * a0.cos(), centre.z + rr * a0.sin());
            let p1 = Vec3::new(x, centre.y + rr * a1.cos(), centre.z + rr * a1.sin());
            mb.tri_n(c, p0, p1, nrm, rim);
        }
        for k in 0..5 {
            let a = (k as f32 / 5.0) * TAU + roll;
            let (s, c) = a.sin_cos();
            let u = Vec3::new(0.0, -s, c);
            let (w0, w1) = (radius * 0.14, radius * 0.07);
            let b0 = Vec3::new(x, centre.y + s * radius * 0.2, centre.z + c * radius * 0.2);
            let b1 = Vec3::new(x, centre.y + s * rr, centre.z + c * rr);
            mb.quad_n(b0 + u * w0, b1 + u * w1, b1 - u * w1, b0 - u * w0, Some(nrm), rim);
        }
    }
}

/// Extrudes a 2D silhouette along the local **Z** axis, then places it with `yaw`.
///
/// `profile` points live in the (y = height, x = depth) plane, CCW. This is how cars,
/// benches and kerbstones get real silhouettes instead of stacked boxes.
pub fn extrude_profile(
    mb: &mut MeshBuilder,
    centre: Vec3,
    half_length: f32,
    profile: &[Vec2],
    yaw: f32,
    paint: &Paint,
) {
    if profile.len() < 3 {
        return;
    }
    let n = profile.len();
    let mut local = MeshBuilder::new();
    // Side walls: sweep each profile edge along the local Z axis.
    for i in 0..n {
        let a = profile[i];
        let b = profile[(i + 1) % n];
        // Outward normal of a CCW edge in the XY profile plane: perp of (ex, ey).
        let (ex, ey) = (b.x - a.x, b.y - a.y);
        let nn = Vec3::new(ey, -ex, 0.0).normalize();
        if nn.length_sq() < 1e-8 {
            continue;
        }
        let a0 = Vec3::new(a.x, a.y, -half_length);
        let a1 = Vec3::new(a.x, a.y, half_length);
        let b0 = Vec3::new(b.x, b.y, half_length);
        let b1 = Vec3::new(b.x, b.y, -half_length);
        local.quad_n(a0, a1, b0, b1, Some(nn), paint);
    }
    // End caps at z = ±half_length.
    let c2 = centroid(profile);
    for side in [-1.0f32, 1.0] {
        let z = side * half_length;
        let c3 = Vec3::new(c2.x, c2.y, z);
        let nrm = Vec3::new(0.0, 0.0, side);
        for i in 0..n {
            let a = profile[i];
            let b = profile[(i + 1) % n];
            let va = Vec3::new(a.x, a.y, z);
            let vb = Vec3::new(b.x, b.y, z);
            local.tri_n(c3, va, vb, nrm, paint);
        }
    }
    local.rotate_y(yaw);
    mb.merge_offset(&local, centre);
}

/// Shrinks a convex polygon toward its centroid by `d` metres.
pub fn inset(poly: &[Vec2], d: f32) -> Vec<Vec2> {
    if poly.is_empty() {
        return Vec::new();
    }
    let c = centroid(poly);
    poly.iter()
        .map(|p| {
            let v = *p - c;
            let l = v.length();
            if l < 1e-6 {
                c
            } else {
                c + v * ((l - d) / l)
            }
        })
        .collect()
}

/// An octagon centred on `centre` with corner cuts of `chamfer`.
pub fn octagon(centre: Vec2, w: f32, d: f32, chamfer: f32) -> Vec<Vec2> {
    let k = chamfer.min(w * 0.5).min(d * 0.5);
    let (hx, hz) = (w * 0.5, d * 0.5);
    vec![
        Vec2::new(centre.x - hx + k, centre.y - hz),
        Vec2::new(centre.x + hx - k, centre.y - hz),
        Vec2::new(centre.x + hx, centre.y - hz + k),
        Vec2::new(centre.x + hx, centre.y + hz - k),
        Vec2::new(centre.x + hx - k, centre.y + hz),
        Vec2::new(centre.x - hx + k, centre.y + hz),
        Vec2::new(centre.x - hx, centre.y + hz - k),
        Vec2::new(centre.x - hx, centre.y - hz + k),
    ]
}

/// A box with chamfered top edges resting on `base`, so rooftops read as designed caps
/// rather than razor-sharp box lids.
pub fn chamfered_box(mb: &mut MeshBuilder, base: Vec3, size: Vec3, chamfer: f32, wall: &Paint, roof: &Paint) {
    let c = chamfer.min(size.x * 0.25).min(size.z * 0.25);
    let top = base.y + size.y;
    if c < 0.05 {
        box_walls(mb, base, size, wall);
        ground_quad(mb, rect_xz(base, size), top, roof);
        return;
    }
    let mid = Vec2::new(base.x + size.x * 0.5, base.z + size.z * 0.5);
    let outer = octagon(mid, size.x, size.z, c);
    extrude_polygon(mb, &outer, base.y, size.y, wall, false, false);
    let inner = inset(&outer, c * 0.9);
    let rise = c * 0.55;
    flat_polygon(mb, &inner, top + rise, roof);
    let n = outer.len();
    for i in 0..n {
        let a = outer[i];
        let b = outer[(i + 1) % n];
        let ia = inner[i];
        let ib = inner[(i + 1) % n];
        let a0 = Vec3::new(a.x, top, a.y);
        let b0 = Vec3::new(b.x, top, b.y);
        let a1 = Vec3::new(ia.x, top + rise, ia.y);
        let b1 = Vec3::new(ib.x, top + rise, ib.y);
        let nn = face_normal(a0, b0, a1);
        mb.quad_n(a0, b0, b1, a1, Some(nn), wall);
    }
}

/// An axis-aligned rectangle covering a box's footprint.
#[inline]
pub fn rect_xz(base: Vec3, size: Vec3) -> Rect {
    Rect {
        min: Vec2::new(base.x, base.z),
        max: Vec2::new(base.x + size.x, base.z + size.z),
    }
}
