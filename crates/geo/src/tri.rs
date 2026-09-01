//! Bounded context: **vertex format + mesh assembly**.
//!
//! The single vertex layout used by every baked mesh and every instanced prop, plus
//! [`MeshBuilder`] for assembling triangle soup with flat shading, merging and
//! transforms.
//!
//! 48-byte stride, `#[repr(C)]`, uploaded verbatim to the GPU. Nothing here is a
//! conventional material: `color.a` carries specular strength, `emissive.a` an
//! emissive gain, and `params` drives the procedural facade shader (window grid,
//! glazing ratio, night-lit probability). One vertex layout, one shader, every
//! material — asphalt, glass, chrome, skin, neon.

use gta_math::{Aabb, Mat4, Vec3};

/// 64-byte vertex. Keep in sync with the shader's `VertexIn`.
#[repr(C)]
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Vertex {
    pub pos: [f32; 3],
    pub normal: [f32; 3],
    /// albedo rgb; `a` = specular strength (0..255).
    pub color: [u8; 4],
    /// emissive rgb; `a` = emissive gain (0..255 maps 0..8x).
    pub emissive: [u8; 4],
    /// Procedural facade parameters.
    ///
    /// `x` = facade kind (`FacadeKind::as_f32`, 0 = plain surface), `y` = bay width,
    /// `z` = storey height, `w` = probability a window is lit at night. Encoding the
    /// recipe per-vertex lets one shader draw every facade in the city.
    pub params: [f32; 4],
    /// Surface coordinates in metres: `x` runs along the face, `y` is up (or, for
    /// ground planes, the world Z). Facades, road markings and ground patterns all
    /// read this rather than tripping over object-space coordinates.
    pub uv: [f32; 2],
    /// Padding so the stride is a clean 64 bytes (a multiple of both 16 and 4 for
    /// the vertex-buffer binding), and so future attributes have somewhere to live.
    pub _pad: [f32; 2],
}

const _: () = assert!(core::mem::size_of::<Vertex>() == 64);

/// Byte stride of [`Vertex`] — the value used in the GPU vertex buffer layout.
pub const VERT_STRIDE: u32 = 64;

/// Facade recipes understood by the procedural window shader.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum FacadeKind {
    /// Plain shaded surface, no windows.
    None,
    /// Curtain wall: near-continuous glazing on a fine grid.
    CurtainWall,
    /// Concrete frame with punched windows and spandrels.
    FrameGrid,
    /// Brick/stucco with sash windows and sills.
    Masonry,
    /// Shopfront: tall glazing, awning band, brightest at night.
    Shopfront,
    /// Multi-storey car park: open deck slots.
    CarPark,
    /// House: small windows in solid walls.
    House,
}

impl FacadeKind {
    /// Value written into `params.x`.
    #[inline]
    pub fn as_f32(self) -> f32 {
        match self {
            FacadeKind::None => 0.0,
            FacadeKind::CurtainWall => 1.0,
            FacadeKind::FrameGrid => 2.0,
            FacadeKind::Masonry => 3.0,
            FacadeKind::Shopfront => 4.0,
            FacadeKind::CarPark => 5.0,
            FacadeKind::House => 6.0,
        }
    }
}

impl Default for Vertex {
    #[inline]
    fn default() -> Self {
        Vertex {
            pos: [0.0; 3],
            normal: [0.0, 1.0, 0.0],
            color: [255, 255, 255, 0],
            emissive: [0, 0, 0, 0],
            params: [0.0; 4],
            uv: [0.0; 2],
            _pad: [0.0; 2],
        }
    }
}

/// Surface character, packed into `color.a`.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Finish {
    Matte,
    Satin,
    Gloss,
    Chrome,
    Wet,
    Glass,
}

impl Finish {
    /// Value written into `color.a`.
    #[inline]
    pub fn spec_byte(self) -> u8 {
        match self {
            Finish::Matte => 12,
            Finish::Satin => 64,
            Finish::Gloss => 160,
            Finish::Chrome => 235,
            Finish::Wet => 200,
            Finish::Glass => 224,
        }
    }
}

/// A material while building: albedo + finish + optional emission.
#[derive(Clone, Copy, Debug)]
pub struct Paint {
    pub rgb: [f32; 3],
    pub finish: Finish,
    pub glow: [f32; 3],
    pub glow_strength: f32,
    pub facade: FacadeKind,
    /// Bay width in metres (window grid pitch along the wall).
    pub bay: f32,
    /// Storey height in metres.
    pub storey: f32,
    /// Probability a window is lit at night.
    pub lit: f32,
}

impl Default for Paint {
    fn default() -> Self {
        Paint {
            rgb: [0.7, 0.7, 0.72],
            finish: Finish::Satin,
            glow: [0.0, 0.0, 0.0],
            glow_strength: 0.0,
            facade: FacadeKind::None,
            bay: 3.0,
            storey: 3.2,
            lit: 0.0,
        }
    }
}

impl PartialEq for Paint {
    fn eq(&self, o: &Self) -> bool {
        self.rgb == o.rgb
            && self.finish == o.finish
            && self.glow == o.glow
            && self.glow_strength == o.glow_strength
            && self.facade == o.facade
            && self.bay == o.bay
            && self.storey == o.storey
            && self.lit == o.lit
    }
}

#[inline]
fn q8(v: f32) -> u8 {
    (v.clamp(0.0, 1.0) * 255.0) as u8
}

impl Paint {
    #[inline]
    pub fn rgb(r: f32, g: f32, b: f32) -> Self {
        Paint {
            rgb: [r, g, b],
            finish: Finish::Satin,
            glow: [0.0, 0.0, 0.0],
            glow_strength: 0.0,
            facade: FacadeKind::None,
            bay: 3.0,
            storey: 3.2,
            lit: 0.0,
        }
    }
    #[inline]
    pub fn grey(v: f32) -> Self {
        Self::rgb(v, v, v)
    }
    #[inline]
    pub fn finish(mut self, f: Finish) -> Self {
        self.finish = f;
        self
    }
    #[inline]
    pub fn glow(mut self, rgb: [f32; 3], strength: f32) -> Self {
        self.glow = rgb;
        self.glow_strength = strength;
        self
    }
    #[inline]
    pub fn lit_chance(mut self, p: f32) -> Self {
        self.lit = p;
        self
    }
    /// Multiplies albedo (per-instance tinting).
    #[inline]
    pub fn scaled(mut self, s: f32) -> Self {
        self.rgb = [self.rgb[0] * s, self.rgb[1] * s, self.rgb[2] * s];
        self
    }
    /// Sets the procedural facade recipe (bay width, storey height, night-lit odds).
    #[inline]
    pub fn facade(mut self, kind: FacadeKind, bay: f32, storey: f32, lit: f32) -> Self {
        self.facade = kind;
        self.bay = bay;
        self.storey = storey;
        self.lit = lit;
        self
    }
    /// Linear blend towards `o`.
    pub fn mix(&self, o: &Paint, t: f32) -> Paint {
        let l = |a: f32, b: f32| a + (b - a) * t;
        Paint {
            rgb: [l(self.rgb[0], o.rgb[0]), l(self.rgb[1], o.rgb[1]), l(self.rgb[2], o.rgb[2])],
            finish: if t < 0.5 { self.finish } else { o.finish },
            glow: [
                l(self.glow[0], o.glow[0]),
                l(self.glow[1], o.glow[1]),
                l(self.glow[2], o.glow[2]),
            ],
            glow_strength: l(self.glow_strength, o.glow_strength),
            facade: if t < 0.5 { self.facade } else { o.facade },
            bay: l(self.bay, o.bay),
            storey: l(self.storey, o.storey),
            lit: l(self.lit, o.lit),
        }
    }
    #[inline]
    pub fn encode_params(&self) -> [f32; 4] {
        [self.facade.as_f32(), self.bay, self.storey, self.lit]
    }
    #[inline]
    pub fn encode_color(&self) -> [u8; 4] {
        [q8(self.rgb[0]), q8(self.rgb[1]), q8(self.rgb[2]), self.finish.spec_byte()]
    }
    #[inline]
    pub fn encode_emissive(&self) -> [u8; 4] {
        [
            q8(self.glow[0]),
            q8(self.glow[1]),
            q8(self.glow[2]),
            ((self.glow_strength.clamp(0.0, 8.0) / 8.0) * 255.0) as u8,
        ]
    }
}

/// CCW face normal of a triangle.
#[inline]
pub fn face_normal(a: Vec3, b: Vec3, c: Vec3) -> Vec3 {
    (b - a).cross(c - a).normalize()
}

/// Planar ("box") projection of `p` onto the dominant plane of normal `n`, in metres.
///
/// Cheap and seam-free for axis-aligned architecture: horizontal faces get (x, z) and
/// vertical faces get (distance along the wall, height), which is exactly the space the
/// facade shader wants.
pub fn box_uv(p: Vec3, n: Vec3) -> [f32; 2] {
    let ax = n.x.abs();
    let ay = n.y.abs();
    let az = n.z.abs();
    if ay >= ax && ay >= az {
        [p.x, p.z]
    } else if ax >= az {
        [p.z, p.y]
    } else {
        [p.x, p.y]
    }
}

/// Triangle soup under construction.
#[derive(Clone, Debug, Default)]
pub struct MeshBuilder {
    pub verts: Vec<Vertex>,
    pub idx: Vec<u32>,
}

impl MeshBuilder {
    #[inline]
    pub fn new() -> Self {
        MeshBuilder { verts: Vec::new(), idx: Vec::new() }
    }

    #[inline]
    pub fn with_capacity(v: usize, i: usize) -> Self {
        MeshBuilder { verts: Vec::with_capacity(v), idx: Vec::with_capacity(i) }
    }

    #[inline]
    pub fn vertex_count(&self) -> usize {
        self.verts.len()
    }

    #[inline]
    pub fn triangle_count(&self) -> usize {
        self.idx.len() / 3
    }

    #[inline]
    pub fn is_empty(&self) -> bool {
        self.idx.is_empty()
    }

    /// One flat-shaded triangle.
    #[inline]
    pub fn tri(&mut self, a: Vec3, b: Vec3, c: Vec3, paint: &Paint) {
        let n = face_normal(a, b, c);
        self.tri_n(a, b, c, n, paint)
    }

    /// Flat triangle with an explicit normal.
    ///
    /// Winding is auto-corrected so the geometric normal agrees with `n`: callers can
    /// hand over whichever order is convenient and still get correct back-face
    /// culling. This removed a whole class of "invisible wall" bugs in the props.
    pub fn tri_n(&mut self, a: Vec3, b: Vec3, c: Vec3, n: Vec3, paint: &Paint) {
        let (a, b, c) = if face_normal(a, b, c).dot(n) >= 0.0 {
            (a, b, c)
        } else {
            (a, c, b)
        };
        let col = paint.encode_color();
        let emi = paint.encode_emissive();
        let prm = paint.encode_params();
        let base = self.verts.len() as u32;
        for p in [a, b, c] {
            let uv = box_uv(p, n);
            self.verts.push(Vertex {
                pos: [p.x, p.y, p.z],
                normal: [n.x, n.y, n.z],
                color: col,
                emissive: emi,
                params: prm,
                uv,
                _pad: [0.0; 2],
            });
        }
        self.idx.extend_from_slice(&[base, base + 1, base + 2]);
    }

    /// Flat quad, corners in CCW order.
    #[inline]
    pub fn quad(&mut self, p0: Vec3, p1: Vec3, p2: Vec3, p3: Vec3, paint: &Paint) {
        self.quad_n(p0, p1, p2, p3, None, paint)
    }

    /// Quad with an optional explicit normal (winding auto-corrected to match).
    pub fn quad_n(&mut self, p0: Vec3, p1: Vec3, p2: Vec3, p3: Vec3, n: Option<Vec3>, paint: &Paint) {
        let (p0, p1, p2, p3) = match n {
            Some(n) => {
                if face_normal(p0, p1, p2).dot(n) >= 0.0 {
                    (p0, p1, p2, p3)
                } else {
                    (p3, p2, p1, p0)
                }
            }
            None => (p0, p1, p2, p3),
        };
        let n = n.unwrap_or_else(|| face_normal(p0, p1, p2));
        let col = paint.encode_color();
        let emi = paint.encode_emissive();
        let prm = paint.encode_params();
        let base = self.verts.len() as u32;
        for p in [p0, p1, p2, p3] {
            let uv = box_uv(p, n);
            self.verts.push(Vertex {
                pos: [p.x, p.y, p.z],
                normal: [n.x, n.y, n.z],
                color: col,
                emissive: emi,
                params: prm,
                uv,
                _pad: [0.0; 2],
            });
        }
        self.idx.extend_from_slice(&[base, base + 1, base + 2, base, base + 2, base + 3]);
    }

    /// Appends another builder's geometry, offset by `delta`.
    pub fn merge_offset(&mut self, other: &MeshBuilder, delta: Vec3) {
        let base = self.verts.len() as u32;
        self.verts.reserve(other.verts.len());
        for v in &other.verts {
            let mut v = *v;
            v.pos = [v.pos[0] + delta.x, v.pos[1] + delta.y, v.pos[2] + delta.z];
            self.verts.push(v);
        }
        self.idx.reserve(other.idx.len());
        for i in &other.idx {
            self.idx.push(base + *i);
        }
    }

    /// Appends another builder transformed by `m`.
    pub fn merge_mat(&mut self, other: &MeshBuilder, m: &Mat4) {
        let base = self.verts.len() as u32;
        self.verts.reserve(other.verts.len());
        for v in &other.verts {
            let p = m.transform_point(Vec3::from_arr(v.pos));
            let n = m.transform_normal(Vec3::from_arr(v.normal));
            self.verts.push(Vertex {
                pos: p.to_arr(),
                normal: n.to_arr(),
                color: v.color,
                emissive: v.emissive,
                params: v.params,
                uv: v.uv,
                _pad: [0.0; 2],
            });
        }
        self.idx.reserve(other.idx.len());
        for i in &other.idx {
            self.idx.push(base + *i);
        }
    }

    /// Transforms all positions and normals in place.
    pub fn apply(&mut self, m: &Mat4) {
        for v in &mut self.verts {
            let p = m.transform_point(Vec3::from_arr(v.pos));
            let n = m.transform_normal(Vec3::from_arr(v.normal));
            v.pos = p.to_arr();
            v.normal = n.to_arr();
        }
    }

    #[inline]
    pub fn translated(mut self, d: Vec3) -> Self {
        self.translate(d);
        self
    }

    /// Rotates all geometry about the local origin by a quaternion.
    pub fn apply_rot(&mut self, q: &gta_math::Quat) {
        for v in &mut self.verts {
            let p = q.rotate(Vec3::from_arr(v.pos));
            let n = q.rotate(Vec3::from_arr(v.normal));
            v.pos = p.to_arr();
            v.normal = n.to_arr();
        }
    }

    /// Rotates all geometry about the local Y axis by `yaw`.
    pub fn rotate_y(&mut self, yaw: f32) {
        self.apply(&Mat4::rot_y(yaw));
    }

    pub fn translate(&mut self, d: Vec3) {
        for v in &mut self.verts {
            v.pos[0] += d.x;
            v.pos[1] += d.y;
            v.pos[2] += d.z;
        }
    }

    /// Axis-aligned bounds of the geometry (INVALID when empty).
    pub fn bounds(&self) -> Aabb {
        let mut b = Aabb::INVALID;
        for v in &self.verts {
            b.grow(Vec3::from_arr(v.pos));
        }
        b
    }

    /// Vertex data as bytes for `Queue::write_buffer`.
    pub fn vertex_bytes(&self) -> &[u8] {
        unsafe {
            core::slice::from_raw_parts(
                self.verts.as_ptr() as *const u8,
                self.verts.len() * core::mem::size_of::<Vertex>(),
            )
        }
    }

    /// Index data as little-endian u32 bytes.
    pub fn index_bytes(&self) -> Vec<u8> {
        let mut out = Vec::with_capacity(self.idx.len() * 4);
        for i in &self.idx {
            out.extend_from_slice(&i.to_le_bytes());
        }
        out
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn vertex_is_gpu_ready() {
        assert_eq!(core::mem::size_of::<Vertex>(), 64);
        assert_eq!(core::mem::align_of::<Vertex>(), 4);
        assert_eq!(VERT_STRIDE, core::mem::size_of::<Vertex>() as u32);
    }

    #[test]
    fn paint_encodes_material_hints() {
        let p = Paint::rgb(0.2, 0.4, 0.6).finish(Finish::Chrome);
        let c = p.encode_color();
        assert_eq!(c[3], Finish::Chrome.spec_byte());
        assert_eq!(c[0], 51);
        let g = Paint::grey(0.5).glow([1.0, 0.5, 0.0], 4.0).encode_emissive();
        assert_eq!(g[0], 255);
        assert_eq!(g[3], 127);
    }

    #[test]
    fn paint_mix_and_scale() {
        let a = Paint::rgb(0.0, 1.0, 0.5);
        let b = Paint::rgb(1.0, 0.0, 0.0);
        let m = a.mix(&b, 0.5);
        assert_eq!(m.rgb, [0.5, 0.5, 0.25]);
        assert_eq!(a.mix(&b, 0.0), a);
        assert_eq!(Paint::grey(0.5).scaled(2.0).rgb, [1.0, 1.0, 1.0]);
    }

    #[test]
    fn flat_triangle_normal() {
        let mut m = MeshBuilder::new();
        // CCW when viewed from +Y
        m.tri(Vec3::new(0.0, 0.0, 0.0), Vec3::new(0.0, 0.0, 1.0), Vec3::new(1.0, 0.0, 0.0), &Paint::grey(0.5));
        assert_eq!(m.verts[0].normal, [0.0, 1.0, 0.0]);
        assert_eq!(m.triangle_count(), 1);
    }

    #[test]
    fn box_uv_projects_onto_dominant_plane() {
        // Vertical wall facing +X: uv = (z, y).
        assert_eq!(box_uv(Vec3::new(1.0, 2.0, 3.0), Vec3::X), [3.0, 2.0]);
        // Horizontal ground: uv = (x, z).
        assert_eq!(box_uv(Vec3::new(1.0, 2.0, 3.0), Vec3::Y), [1.0, 3.0]);
        // Wall facing +Z: uv = (x, y).
        assert_eq!(box_uv(Vec3::new(1.0, 2.0, 3.0), Vec3::Z), [1.0, 2.0]);
    }

    #[test]
    fn winding_follows_the_normal() {
        let mut m = MeshBuilder::new();
        let p = Paint::grey(0.5);
        // Deliberately CW when viewed from +Y, but we ask for a +Y normal.
        m.tri(
            Vec3::new(0.0, 0.0, 0.0),
            Vec3::new(1.0, 0.0, 0.0),
            Vec3::new(0.0, 0.0, 1.0),
            &p,
        );
        // tri() derives the normal from winding, so it stays as given.
        assert_eq!(m.verts[0].normal, [0.0, -1.0, 0.0]);
        // tri_n must flip the winding so the geometric normal matches.
        let mut m2 = MeshBuilder::new();
        m2.tri_n(
            Vec3::new(0.0, 0.0, 0.0),
            Vec3::new(1.0, 0.0, 0.0),
            Vec3::new(0.0, 0.0, 1.0),
            Vec3::Y,
            &p,
        );
        let a = Vec3::from_arr(m2.verts[0].pos);
        let b = Vec3::from_arr(m2.verts[1].pos);
        let c = Vec3::from_arr(m2.verts[2].pos);
        assert!(face_normal(a, b, c).dot(Vec3::Y) > 0.0);
    }

    #[test]
    fn quad_is_six_indices() {
        let mut m = MeshBuilder::new();
        m.quad(
            Vec3::new(0.0, 0.0, 0.0),
            Vec3::new(1.0, 0.0, 0.0),
            Vec3::new(1.0, 0.0, 1.0),
            Vec3::new(0.0, 0.0, 1.0),
            &Paint::grey(0.5),
        );
        assert_eq!(m.verts.len(), 4);
        assert_eq!(m.idx.len(), 6);
        assert_eq!(m.idx, vec![0, 1, 2, 0, 2, 3]);
    }

    #[test]
    fn merge_offsets_indices() {
        let mut a = MeshBuilder::new();
        let p = Paint::grey(0.5);
        a.tri(Vec3::ZERO, Vec3::X, Vec3::Z, &p);
        let mut b = MeshBuilder::new();
        b.tri(Vec3::ZERO, Vec3::X, Vec3::Z, &p);
        a.merge_offset(&b, Vec3::new(10.0, 0.0, 0.0));
        assert_eq!(a.verts.len(), 6);
        assert_eq!(a.idx.len(), 6);
        assert_eq!(a.idx[3], 3);
        assert_eq!(a.verts[3].pos[0], 10.0);
    }

    #[test]
    fn merge_mat_rotates_and_renormals() {
        let mut unit = MeshBuilder::new();
        let p = Paint::grey(0.5);
        unit.tri(Vec3::ZERO, Vec3::new(0.0, 0.0, 1.0), Vec3::new(1.0, 0.0, 0.0), &p);
        let mut out = MeshBuilder::new();
        let m = Mat4::compose(Vec3::new(5.0, 0.0, 0.0), std::f32::consts::FRAC_PI_2, Vec3::splat(2.0));
        out.merge_mat(&unit, &m);
        assert_eq!(out.verts.len(), 3);
        assert_eq!(out.triangle_count(), 1);
    }

    #[test]
    fn bounds_and_bytes() {
        let mut m = MeshBuilder::new();
        let p = Paint::grey(0.5);
        m.quad(
            Vec3::ZERO,
            Vec3::new(2.0, 0.0, 0.0),
            Vec3::new(2.0, 0.0, 2.0),
            Vec3::new(0.0, 0.0, 2.0),
            &p,
        );
        let b = m.bounds();
        assert_eq!(b.min, Vec3::ZERO);
        assert_eq!(b.max, Vec3::new(2.0, 0.0, 2.0));
        assert_eq!(m.vertex_bytes().len(), 4 * 64);
        assert_eq!(m.index_bytes().len(), 6 * 4);
        assert!(MeshBuilder::new().bounds().min.x > 0.0);
    }
}
