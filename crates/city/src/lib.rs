//! # gta-city
//!
//! Bounded context: **the city**.
//!
//! The simulation-level model of a city: blocks, zones, roads, lots, buildings and the
//! walkable graph. This crate knows nothing about triangles or GPUs — it answers questions
//! like *"what is at this position?"*, *"where may a pedestrian walk?"* and *"which lane do
//! I drive in?"*. The `scene` crate turns it into triangles.
//!
//! Everything is a pure function of [`CityParams`] and a seed: generation runs once, then
//! the city is read-only while the sim runs. Entry point: [`generate`], which builds the
//! street [`Network`], the zoned [`Block`]s and the [`WalkGraph`] in one pass.

pub mod blocks;
pub mod graph;
pub mod params;
pub mod roads;
pub mod zone;

pub use blocks::{Block, BlockKind, Building, Lamp, LampKind, RoofKind, Tree, TreeKind};
pub use graph::{WalkGraph, WalkId, WalkKind};
pub use params::CityParams;
pub use roads::{Axis, Intersection, Network, Road};
pub use zone::{Facade, Zone, ZonePlanner};

use gta_math::Vec2;

/// Everything generation produces; read-only while the sim runs.
pub struct City {
    pub params: CityParams,
    pub network: Network,
    pub walk: WalkGraph,
    pub blocks: Vec<Block>,
}

impl City {
    /// Block by lattice cell indices (`ix + iz * grid`).
    pub fn block(&self, ix: usize, iz: usize) -> Option<&Block> {
        if ix >= self.params.grid || iz >= self.params.grid {
            return None;
        }
        self.blocks.get(ix + iz * self.params.grid)
    }

    /// The block whose kerb-to-kerb rect contains `p`, if any.
    pub fn block_at(&self, p: Vec2) -> Option<&Block> {
        self.blocks.iter().find(|b| b.rect.contains(p))
    }

    /// True when `p` sits in a carriageway (`pad` widens the test) — where pedestrians
    /// should not be.
    pub fn on_road(&self, p: Vec2, pad: f32) -> bool {
        self.network.on_road(p, pad)
    }

    /// Widest carriageway in the city.
    pub fn widest_street(&self) -> f32 {
        self.params.avenue.max(self.params.road)
    }

    /// Carriageway width of the street running along lattice line `i`.
    #[inline]
    pub fn width_of_line(&self, i: usize) -> f32 {
        self.params.width(i)
    }

    /// Half-width of the junction box at lattice line `line`, measured along the street
    /// that runs *through* it: the box is bounded by the kerbs of the street sitting on
    /// `line`, plus a metre of give for the stop line.
    #[inline]
    pub fn junction_half(&self, line: f32) -> f32 {
        0.5 * self.params.width(self.network.nearest_line(line)) + 1.0
    }
}

/// Generates the whole city deterministically from `params.seed`.
pub fn generate(params: CityParams) -> City {
    let lines = params.lines();
    let half = params.half();
    let network = Network::build(
        &lines,
        |i| params.width(i),
        |i| params.is_avenue(i),
        |i| if params.is_avenue(i) { 2 } else { 1 },
        half,
    );

    // Separate RNG streams per subsystem: adding a random consumer in one subsystem can
    // never perturb another's output.
    let mut zone_rng = params.rng(0x20_1F);
    let mut planner = ZonePlanner::new(half, &mut zone_rng);
    let mut blocks: Vec<Block> = Vec::with_capacity(params.grid * params.grid);
    let mut index = 0u32;
    for iz in 0..params.grid {
        for ix in 0..params.grid {
            // Kerb-to-kerb cell, *not* the midpoint of the centre lines: where an avenue
            // meets a normal street the kerb midpoint sits (w_i - w_i+1)/4 off the line
            // midpoint, and blocks must sit exactly between their two kerbs.
            let centre = blocks::cell_centre(&lines, ix, iz, &params);
            let zone = planner.zone_of(centre, index);
            // One RNG stream per block: a change in one block can never ripple into
            // another block's output.
            let mut rng = params.rng(0xB10C_5EED ^ (index as u64).wrapping_mul(0x9E37_79B9_7F4A_7C15));
            blocks.push(blocks::build(ix, iz, centre, zone, &mut planner, &params, &mut rng));
            index += 1;
        }
    }

    // Sidewalk rings first, then zebra crossings between facing kerb nodes.
    let mut walk = WalkGraph::new(params.extent());
    for b in &blocks {
        let ids = b.add_walk_nodes(&mut walk, params.sidewalk);
        walk.link_ring(&ids);
    }
    blocks::add_crossings(&mut walk, &lines, widest_of(&params), params.sidewalk);

    City { params, network, walk, blocks }
}

#[inline]
fn widest_of(p: &CityParams) -> f32 {
    p.avenue.max(p.road)
}

#[cfg(test)]
mod tests {
    use super::*;

    fn city(grid: usize) -> City {
        generate(CityParams { grid, ..Default::default() })
    }

    #[test]
    fn block_count_matches_grid() {
        let c = city(7);
        assert_eq!(c.blocks.len(), 49);
        // One corridor per lattice line per axis, and there are `grid + 1` lines.
        assert_eq!(c.network.road_count(), 2 * (7 + 1));
    }

    #[test]
    fn blocks_sit_between_their_kerbs() {
        let c = city(7);
        let ls = c.params.lines();
        for b in &c.blocks {
            let left = ls[b.ix] + 0.5 * c.params.width(b.ix);
            let right = ls[b.ix + 1] - 0.5 * c.params.width(b.ix + 1);
            let near = ls[b.iz] + 0.5 * c.params.width(b.iz);
            let far = ls[b.iz + 1] - 0.5 * c.params.width(b.iz + 1);
            assert!((b.rect.min.x - left).abs() < 1e-3, "block {:?} left", b.centre);
            assert!((b.rect.max.x - right).abs() < 1e-3, "block {:?} right", b.centre);
            assert!((b.rect.min.y - near).abs() < 1e-3);
            assert!((b.rect.max.y - far).abs() < 1e-3);
            assert!((b.rect.width() - c.params.block).abs() < 1e-2);
        }
        assert!(c.block(0, 0).is_some());
        assert!(c.block(7, 0).is_none());
    }

    #[test]
    fn block_at_finds_the_right_block() {
        let c = city(7);
        for b in &c.blocks {
            assert_eq!(c.block_at(b.centre).map(|x| x.ix), Some(b.ix));
            assert_eq!(c.block_at(b.centre).map(|x| x.iz), Some(b.iz));
        }
        // Outside the grid there is nothing.
        let far = Vec2::new(c.params.half() * 3.0, 0.0);
        assert!(c.block_at(far).is_none());
    }

    #[test]
    fn generation_is_deterministic() {
        let a = generate(CityParams::new(0xABCD));
        let b = generate(CityParams::new(0xABCD));
        assert_eq!(a.blocks.len(), b.blocks.len());
        for (x, y) in a.blocks.iter().zip(b.blocks.iter()) {
            assert_eq!(x.zone, y.zone);
            assert_eq!(x.kind, y.kind);
            assert_eq!(x.buildings.len(), y.buildings.len());
            assert_eq!(x.trees.len(), y.trees.len());
            assert_eq!(x.lamps.len(), y.lamps.len());
            for (p, q) in x.buildings.iter().zip(y.buildings.iter()) {
                assert_eq!(p.rect, q.rect);
                assert_eq!(p.height.to_bits(), q.height.to_bits());
                assert_eq!(p.facade, q.facade);
            }
            for (p, q) in x.trees.iter().zip(y.trees.iter()) {
                assert_eq!(p.pos, q.pos);
            }
        }
        assert_eq!(a.walk.len(), b.walk.len());
        assert_eq!(a.walk.edge_count(), b.walk.edge_count());
    }

    #[test]
    fn different_seeds_give_different_cities() {
        let a = generate(CityParams::new(1));
        let b = generate(CityParams::new(4242));
        let za: Vec<Zone> = a.blocks.iter().map(|b| b.zone).collect();
        let zb: Vec<Zone> = b.blocks.iter().map(|b| b.zone).collect();
        assert_ne!(za, zb);
    }

    #[test]
    fn buildings_stay_inside_their_block() {
        let c = city(9);
        for b in &c.blocks {
            if b.kind != BlockKind::Building {
                continue;
            }
            assert!(!b.buildings.is_empty(), "building block with no buildings");
            for bd in &b.buildings {
                assert!(bd.rect.min.x >= b.rect.min.x - 0.01, "{:?}", bd.rect);
                assert!(bd.rect.max.x <= b.rect.max.x + 0.01);
                assert!(bd.rect.min.y >= b.rect.min.y - 0.01);
                assert!(bd.rect.max.y <= b.rect.max.y + 0.01);
                assert!(bd.height > 2.0);
                assert!(bd.rect.width() > 3.0 && bd.rect.depth() > 3.0);
            }
        }
    }

    #[test]
    fn buildings_do_not_overlap() {
        let c = city(9);
        for b in &c.blocks {
            for i in 0..b.buildings.len() {
                for j in (i + 1)..b.buildings.len() {
                    let a = &b.buildings[i].rect;
                    let o = &b.buildings[j].rect;
                    let separated = a.max.x < o.min.x
                        || o.max.x < a.min.x
                        || a.max.y < o.min.y
                        || o.max.y < a.min.y;
                    assert!(separated, "overlap in block {:?}", b.centre);
                }
            }
        }
    }

    #[test]
    fn props_never_land_on_a_carriageway() {
        let c = city(7);
        for b in &c.blocks {
            for t in &b.trees {
                assert!(!c.on_road(t.pos, 0.5), "tree at {:?}", t.pos);
            }
            for l in &b.lamps {
                assert!(!c.on_road(l.pos, -0.5), "lamp off the kerb at {:?}", l.pos);
            }
        }
    }

    #[test]
    fn walk_graph_is_connected() {
        let c = city(7);
        let start = c.walk.nearest(Vec2::ZERO, 1e9).expect("some node");
        let orphans = c.walk.unreachable_from(start);
        assert_eq!(orphans, 0, "unreachable walk nodes");
    }

    #[test]
    fn sidewalk_nodes_are_off_the_carriageway() {
        let c = city(7);
        for id in 0..c.walk.len() as u32 {
            if c.walk.kind(id) == WalkKind::Sidewalk {
                let p = c.walk.pos(id);
                assert!(!c.on_road(p, -0.3), "walk node on the carriageway: {p:?}");
            }
        }
    }

    #[test]
    fn crossings_cross_a_single_street() {
        let c = city(7);
        let widest = c.widest_street();
        let mut n = 0usize;
        for id in 0..c.walk.len() as u32 {
            if c.walk.kind(id) != WalkKind::Crossing {
                continue;
            }
            let p = c.walk.pos(id);
            assert!(c.on_road(p, 0.0), "crossing {p:?} is not on a street");
            // The two kerb nodes it joins must be within one street width apart.
            let nb = c.walk.neighbours(id);
            assert_eq!(nb.len(), 2);
            let span = (c.walk.pos(nb[0]) - c.walk.pos(nb[1])).length();
            assert!(span < widest + 2.0 * c.params.sidewalk + 6.0, "crossing span {span}");
            n += 1;
        }
        assert!(n > 20, "only {n} crossings generated");
    }

    #[test]
    fn downtown_is_taller_than_the_suburbs() {
        let c = city(11);
        let tallest = |z: Zone| {
            c.blocks
                .iter()
                .filter(|b| b.zone == z)
                .flat_map(|b| b.buildings.iter())
                .map(|b| b.height)
                .fold(0.0f32, f32::max)
        };
        let (dt, res) = (tallest(Zone::Downtown), tallest(Zone::Residential));
        assert!(dt > res, "downtown {dt} should tower over residential {res}");
    }

    #[test]
    fn every_block_has_kerb_lamps() {
        let c = city(5);
        for b in &c.blocks {
            assert!(b.lamps.len() >= 4, "block {:?} has {} lamps", b.centre, b.lamps.len());
        }
    }

    #[test]
    fn parks_are_green_and_buildings_free() {
        let c = city(9);
        let mut parks = 0;
        for b in &c.blocks {
            if b.kind == BlockKind::Park {
                parks += 1;
                assert!(b.buildings.is_empty());
                assert!(!b.trees.is_empty(), "park without trees at {:?}", b.centre);
            }
        }
        assert!(parks > 0, "no parks generated");
    }
}
