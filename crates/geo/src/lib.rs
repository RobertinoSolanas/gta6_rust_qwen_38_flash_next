//! Bounded context: **geometry primitives**.
//!
//! The shape vocabulary shared by the city, the vehicles and the characters:
//!
//! * [`MeshBuilder`] — flat/smooth shaded triangles with a packed 32-byte vertex,
//!   an emissive channel and byte accessors ready for GPU upload.
//! * [`primitives`] — boxes, extruded polygons, tapered cylinders, cones, spheres,
//!   capsules, beams, wheels, chamfered boxes and profile extrusions. Real, bevelled
//!   geometry rather than debug cubes.
//! * [`GridIndex`] — a uniform spatial hash for proximity queries.
//! * [`clip`] — rectangle/polygon helpers (Sutherland–Hodgman clipping) used for
//!   corner lots and sidewalk pads.
//!
//! Conventions: `+Y` up, metres, outward-facing normals. `base` is the bottom of a
//! solid, `centre` is the middle of a volume. Because [`MeshBuilder::tri_n`] and
//! [`MeshBuilder::quad_n`] auto-correct winding to agree with the supplied normal,
//! callers only have to get the *normal* right.

pub mod clip;
pub mod grid;
pub mod primitives;
pub mod tri;

pub use clip::{area_abs, bounds, centroid, clip_polygon, polygon_area, Rect};
pub use grid::GridIndex;
pub use primitives::{
    beam, blob, box_solid, box_walls, capsule, cap_polygon, chamfered_box, cone, cylinder, ellipsoid,
    extrude_polygon, extrude_profile, flat_polygon, ground_quad, inset, octagon, quat_from_to, rect_xz,
    sphere, tapered_cylinder, wheel,
};
pub use tri::{face_normal, FacadeKind, Finish, MeshBuilder, Paint, Vertex, VERT_STRIDE};
