//! Bounded context: **blocks, lots and massing**.
//!
//! Turns one lattice cell into a place: a buildable site, a subdivision into lots, a
//! building per lot with zone-appropriate height and facade, plus green and civic
//! furniture (trees, bushes, lamp posts, the odd pond) and the sidewalk ring that feeds
//! the [`WalkGraph`](crate::WalkGraph).
//!
//! Everything is a pure function of the block's centre, its [`Zone`] and an RNG stream, so
//! a block can be inspected or regenerated in isolation — and so a whole city compares
//! byte-for-byte between runs (the tests rely on that).

use gta_geo::Rect;
use gta_math::{clamp, fbm2, smoothstep, Rng, Vec2};

use crate::graph::{WalkGraph, WalkId, WalkKind};
use crate::params::CityParams;
use crate::zone::{Facade, Zone, ZonePlanner};

/// Sidewalk-ring sample spacing in metres.
pub const RING_STEP: f32 = 6.0;

/// Put a crossing at every Nth ring sample — roughly every 18 m, about the spacing of
/// signalised crossings on a real grid.
pub const CROSS_EVERY: usize = 3;

/// What occupies the block interior.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum BlockKind {
    /// Lots with buildings.
    Building,
    /// Parkland: lawn, trees, sometimes a pond.
    Park,
    /// Open civic paving.
    Plaza,
    /// Surface car park.
    Parking,
}

/// Foliage flavour — drives silhouette and palette in the scene crate.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum TreeKind {
    /// Round broadleaf canopy on a trunk.
    Canopy,
    /// Low dense blob.
    Bush,
    /// Slim conifer.
    Conifer,
}

/// A tree, bush or hedge.
#[derive(Clone, Copy, Debug)]
pub struct Tree {
    pub pos: Vec2,
    /// Canopy radius in metres.
    pub radius: f32,
    pub height: f32,
    pub kind: TreeKind,
    /// 0..1 palette variation.
    pub tint: f32,
}

/// Where a lamp belongs.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum LampKind {
    /// Kerbside street light.
    Street,
    /// Lower park / plaza globe light.
    Park,
}

/// A lamp post.
#[derive(Clone, Copy, Debug)]
pub struct Lamp {
    pub pos: Vec2,
    pub kind: LampKind,
}

/// Roof treatment.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum RoofKind {
    /// Parapet ring over a flat deck.
    Flat,
    /// Gable roof (houses).
    Pitched,
    /// Chamfered cap (mid- and high-rise).
    Chamfer,
}

/// One building footprint.
#[derive(Clone, Copy, Debug)]
pub struct Building {
    /// Footprint in the XZ plane, inside the block's site.
    pub rect: Rect,
    pub floors: usize,
    /// Eave height in metres (`floors × zone floor height`).
    pub height: f32,
    pub facade: Facade,
    pub roof: RoofKind,
    /// 0..1 albedo variation so identical facades do not read as clones.
    pub tint: f32,
    /// Probability that a window is lit at night, scaled from the facade's night life.
    pub lit: f32,
    /// Rooftop stair/plant box width in metres (0 = none).
    pub rooftop: f32,
}

impl Building {
    /// Ridge rise above `height` for a pitched roof, 0 otherwise.
    pub fn roof_rise(&self) -> f32 {
        match self.roof {
            RoofKind::Pitched => (self.rect.depth().min(self.rect.width()) * 0.34).min(4.5),
            RoofKind::Chamfer | RoofKind::Flat => 0.0,
        }
    }

    pub fn centre(&self) -> Vec2 {
        Vec2::new(0.5 * (self.rect.min.x + self.rect.max.x), 0.5 * (self.rect.min.y + self.rect.max.y))
    }
}

/// One city block.
#[derive(Clone, Debug)]
pub struct Block {
    pub ix: usize,
    pub iz: usize,
    pub centre: Vec2,
    /// Kerb-to-kerb footprint.
    pub rect: Rect,
    pub zone: Zone,
    pub kind: BlockKind,
    pub buildings: Vec<Building>,
    pub trees: Vec<Tree>,
    pub lamps: Vec<Lamp>,
    /// Park pond as (centre, radius).
    pub pond: Option<(Vec2, f32)>,
}

impl Block {
    /// The buildable interior: the block minus the sidewalk band.
    pub fn site(&self, sidewalk: f32) -> Rect {
        let m = sidewalk + 0.4;
        Rect::centered(self.centre, self.rect.width() - 2.0 * m, self.rect.depth() - 2.0 * m)
    }

    /// Sidewalk ring: points about [`RING_STEP`] metres apart along the block border,
    /// inset half a sidewalk from the kerb. The same ring builds the walk graph *and* the
    /// paving geometry, so simulation and picture can never disagree.
    pub fn walk_ring(&self, sidewalk: f32) -> Vec<Vec2> {
        edge_inset(self.rect, sidewalk * 0.5, RING_STEP)
    }

    /// Adds this block's sidewalk ring to `g`, returning the node ids in the walk order
    /// documented on [`edge_inset`] so [`link_ring`](WalkGraph::link_ring) can close the
    /// loop and [`add_crossings`] can address ring slots arithmetically.
    pub fn add_walk_nodes(&self, g: &mut WalkGraph, sidewalk: f32) -> Vec<WalkId> {
        let kind = match self.kind {
            BlockKind::Park => WalkKind::Park,
            BlockKind::Plaza => WalkKind::Plaza,
            BlockKind::Parking => WalkKind::Sidewalk,
            BlockKind::Building => WalkKind::Sidewalk,
        };
        self.walk_ring(sidewalk).iter().map(|p| g.add(*p, kind)).collect()
    }
}

// ---------------------------------------------------------------------------
// Generation
// ---------------------------------------------------------------------------

/// Centre of lattice cell `(ix, iz)`: the midpoint of the kerb-to-kerb rectangle.
///
/// Note this is *not* the midpoint of the two centre lines. Where an avenue meets a normal
/// street the kerb midpoint sits `(w_i - w_i+1)/4` off the line midpoint, and a block must
/// sit exactly between its two kerbs or it would overlap a carriageway on one side.
#[inline]
pub fn cell_centre(lines: &[f32], ix: usize, iz: usize, p: &CityParams) -> Vec2 {
    Vec2::new(kerb_mid(lines, ix, p), kerb_mid(lines, iz, p))
}

/// Midpoint between the kerbs that bound cell `i`: half-way between the inner edge of
/// street `i` and the kerb of street `i + 1`.
#[inline]
fn kerb_mid(lines: &[f32], i: usize, p: &CityParams) -> f32 {
    let left = lines[i] + 0.5 * p.width(i);
    let right = lines[i + 1] - 0.5 * p.width(i + 1);
    0.5 * (left + right)
}

/// Builds one block. `planner` supplies the facade mix; `rng` is this block's stream.
pub fn build(
    ix: usize,
    iz: usize,
    centre: Vec2,
    zone: Zone,
    planner: &mut ZonePlanner,
    p: &CityParams,
    rng: &mut Rng,
) -> Block {
    let rect = Rect::centered(centre, p.block, p.block);
    let kind = match zone {
        Zone::Park => BlockKind::Park,
        Zone::Plaza => BlockKind::Plaza,
        Zone::Parking => BlockKind::Parking,
        _ => BlockKind::Building,
    };
    let mut b = Block {
        ix,
        iz,
        centre,
        rect,
        zone,
        kind,
        buildings: Vec::new(),
        trees: Vec::new(),
        lamps: Vec::new(),
        pond: None,
    };

    match kind {
        BlockKind::Building => massing(&mut b, p, planner, rng),
        BlockKind::Park => pond_layout(&mut b, rng),
        BlockKind::Plaza | BlockKind::Parking => {}
    }

    scatter_trees(&mut b, p, rng);
    add_lamps(&mut b, p);
    b
}

/// Storeys for a lot: the zone envelope, biased by a low-frequency noise field so towers
/// gather into a skyline instead of every plot in a district being equally tall.
fn floors_for(zone: Zone, centre: Vec2, rng: &mut Rng) -> usize {
    let (lo, hi) = zone.floors_envelope();
    let n = clamp(fbm2(centre.x * 0.012, centre.y * 0.012, 3), -1.0, 1.0);
    let bump = 0.5 + 0.5 * smoothstep(clamp(0.5 + 0.5 * n, 0.0, 1.0));
    let t = clamp(0.55 * bump + 0.45 * rng.f32(), 0.0, 1.0);
    (lo as f32 + (hi - lo) as f32 * t).round() as usize
}

fn massing(b: &mut Block, p: &CityParams, planner: &mut ZonePlanner, rng: &mut Rng) {
    let site = b.site(p.sidewalk);
    let mut lots: Vec<Rect> = Vec::new();
    let target = (site.width() * site.depth() * b.zone.site_coverage()).max(300.0);
    let depth = if b.zone == Zone::Downtown { 2 } else { 3 };
    subdivide(site, target, depth, rng, &mut lots);

    for lot in lots {
        let lc = Vec2::new(0.5 * (lot.min.x + lot.max.x), 0.5 * (lot.min.y + lot.max.y));
        let floors = floors_for(b.zone, lc, rng);
        let facade = planner.facade_of(b.zone, floors);
        let height = b.zone.floor_height() * floors as f32;
        let d = rng.range(0.4, 1.3);
        let rect = Rect {
            min: Vec2::new(lot.min.x + d, lot.min.y + d),
            max: Vec2::new(lot.max.x - d, lot.max.y - d),
        };
        if rect.width() < 7.0 || rect.depth() < 6.0 {
            continue;
        }
        let roof = match facade {
            Facade::House => {
                if rng.chance(0.85) {
                    RoofKind::Pitched
                } else {
                    RoofKind::Flat
                }
            }
            _ => {
                if floors > 4 {
                    RoofKind::Chamfer
                } else {
                    RoofKind::Flat
                }
            }
        };
        b.buildings.push(Building {
            rect,
            floors,
            height,
            facade,
            roof,
            tint: rng.f32(),
            lit: facade.night_life() * (0.45 + 0.5 * rng.f32()),
            rooftop: if floors > 6 && rng.chance(0.7) { rng.range(2.5, 5.5) } else { 0.0 },
        });
    }
}

/// Recursive binary lot subdivision: stops at `depth`, once a lot is small enough, or
/// when a further cut would leave a lot too narrow to build on.
fn subdivide(rect: Rect, target: f32, depth: usize, rng: &mut Rng, out: &mut Vec<Rect>) {
    /// Narrowest lot worth building on.
    const MIN_LOT: f32 = 13.0;

    let (w, d) = (rect.width(), rect.depth());
    if depth == 0 || w * d <= target || (w < 2.0 * MIN_LOT && d < 2.0 * MIN_LOT) {
        out.push(rect);
        return;
    }
    if w >= d {
        if w < 2.0 * MIN_LOT {
            out.push(rect);
            return;
        }
        let x = rect.min.x + w * rng.range(0.4, 0.6);
        subdivide(Rect { min: rect.min, max: Vec2::new(x, rect.max.y) }, target, depth - 1, rng, out);
        subdivide(Rect { min: Vec2::new(x, rect.min.y), max: rect.max }, target, depth - 1, rng, out);
    } else {
        if d < 2.0 * MIN_LOT {
            out.push(rect);
            return;
        }
        let z = rect.min.y + d * rng.range(0.4, 0.6);
        subdivide(Rect { min: rect.min, max: Vec2::new(rect.max.x, z) }, target, depth - 1, rng, out);
        subdivide(Rect { min: Vec2::new(rect.min.x, z), max: rect.max }, target, depth - 1, rng, out);
    }
}

/// A pond in some parks, kept clear of the sidewalks.
fn pond_layout(b: &mut Block, rng: &mut Rng) {
    if !rng.chance(0.45) {
        return;
    }
    let r = rng.range(5.0, 9.0);
    if b.rect.width() < 2.0 * (r + 5.0) || b.rect.depth() < 2.0 * (r + 5.0) {
        return;
    }
    let c = Vec2::new(
        rng.range(b.rect.min.x + r + 5.0, b.rect.max.x - r - 5.0),
        rng.range(b.rect.min.y + r + 5.0, b.rect.max.y - r - 5.0),
    );
    b.pond = Some((c, r));
}

/// Trees, bushes and hedges: density by zone, rejected when they would clash with a
/// building, a pond or an already-placed plant.
fn scatter_trees(b: &mut Block, p: &CityParams, rng: &mut Rng) {
    let site = b.site(p.sidewalk);
    let density = match b.zone {
        Zone::Park => 0.0075,
        Zone::Plaza => 0.0032,
        Zone::Parking => 0.0012,
        Zone::Residential => 0.004,
        _ => 0.0022,
    };
    let want = (((site.width() * site.depth()) * density) as usize).min(80);
    let mut tries = 0usize;
    while b.trees.len() < want && tries < 4 * want + 60 {
        tries += 1;
        let pos = Vec2::new(rng.range(site.min.x, site.max.x), rng.range(site.min.y, site.max.y));
        if !clear_of_buildings(b, pos) {
            continue;
        }
        if let Some((c, r)) = b.pond {
            if (pos - c).length() < r + 1.2 {
                continue;
            }
        }
        if b.trees.iter().any(|t| (t.pos - pos).length_sq() < 1.44) {
            continue;
        }
        let kind = if b.zone == Zone::Park {
            if rng.chance(0.14) {
                TreeKind::Conifer
            } else if rng.chance(0.2) {
                TreeKind::Bush
            } else {
                TreeKind::Canopy
            }
        } else if rng.chance(0.3) {
            TreeKind::Bush
        } else {
            TreeKind::Canopy
        };
        let (radius, height) = match kind {
            TreeKind::Canopy => (rng.range(1.7, 3.1), rng.range(4.5, 7.5)),
            TreeKind::Bush => (rng.range(0.7, 1.3), rng.range(0.9, 1.6)),
            TreeKind::Conifer => (rng.range(1.0, 1.8), rng.range(6.0, 10.0)),
        };
        b.trees.push(Tree { pos, radius, height, kind, tint: rng.f32() });
    }
}

fn clear_of_buildings(b: &Block, pos: Vec2) -> bool {
    !b.buildings.iter().any(|bd| {
        pos.x > bd.rect.min.x - 1.2
            && pos.x < bd.rect.max.x + 1.2
            && pos.y > bd.rect.min.y - 1.2
            && pos.y < bd.rect.max.y + 1.2
    })
}

/// Four kerb-side lamps at the block corners, plus masts along one edge of big open
/// blocks. Always on the sidewalk band, never in the carriageway.
fn add_lamps(b: &mut Block, p: &CityParams) {
    let m = p.sidewalk * 0.5;
    let kind = if b.kind == BlockKind::Building { LampKind::Street } else { LampKind::Park };
    let corners = [
        Vec2::new(b.rect.min.x + m, b.rect.min.y + m),
        Vec2::new(b.rect.max.x - m, b.rect.min.y + m),
        Vec2::new(b.rect.max.x - m, b.rect.max.y - m),
        Vec2::new(b.rect.min.x + m, b.rect.max.y - m),
    ];
    for pos in corners {
        b.lamps.push(Lamp { pos, kind });
    }
    if b.kind == BlockKind::Parking || b.kind == BlockKind::Plaza {
        for i in 1..3 {
            let t = i as f32 / 3.0;
            b.lamps.push(Lamp {
                pos: Vec2::new(b.rect.min.x + t * b.rect.width(), b.rect.min.y + m),
                kind,
            });
        }
    }
}

// ---------------------------------------------------------------------------
// Ring geometry & pedestrian crossings
// ---------------------------------------------------------------------------

/// Samples a closed polyline inset `m` metres inside `rect`, spacing about `step`.
///
/// Walk order: bottom edge (x ascending), right edge (z ascending), top edge (x
/// descending), left edge (z descending).
///
/// The property that matters: two blocks facing each other across a street share their
/// centre coordinate on the axis along that street, so their ring samples line up almost
/// exactly. That lets [`add_crossings`] pair kerb nodes with one cheap local query instead
/// of a search.
pub fn edge_inset(rect: Rect, m: f32, step: f32) -> Vec<Vec2> {
    let w = (rect.width() - 2.0 * m).max(1.0);
    let d = (rect.depth() - 2.0 * m).max(1.0);
    let c = Vec2::new(0.5 * (rect.min.x + rect.max.x), 0.5 * (rect.min.y + rect.max.y));
    let r = Rect::centered(c, w, d);
    let nx = slots(w, step);
    let nz = slots(d, step);
    let mut out = Vec::with_capacity(2 * (nx + nz));
    for i in 0..nx {
        out.push(Vec2::new(r.min.x + w * (i as f32 / nx as f32), r.min.y));
    }
    for i in 0..nz {
        out.push(Vec2::new(r.max.x, r.min.y + d * (i as f32 / nz as f32)));
    }
    for i in 0..nx {
        out.push(Vec2::new(r.max.x - w * (i as f32 / nx as f32), r.max.y));
    }
    for i in 0..nz {
        out.push(Vec2::new(r.min.x, r.max.y - d * (i as f32 / nz as f32)));
    }
    out
}

/// Ring slot count for one edge of the given length — mirrors [`edge_inset`].
#[inline]
pub fn slots(len: f32, step: f32) -> usize {
    ((len.max(1.0) / step.max(0.5)) as usize).max(2)
}

/// The lattice line strictly between two coordinates on one axis, if any.
fn line_between(lines: &[f32], a: f32, b: f32) -> Option<f32> {
    let (lo, hi) = if a < b { (a, b) } else { (b, a) };
    lines.iter().copied().find(|&l| l > lo + 0.05 && l < hi - 0.05)
}

/// One candidate pedestrian crossing.
#[derive(Clone, Copy, Debug)]
struct Crossing {
    /// `true` when the crossed street has constant `x` (runs north–south).
    vertical: bool,
    /// Centre line of the street being crossed — part of the crossing's identity, so two
    /// zebrae on *different* streets never compare equal.
    line: f32,
    /// Along-street coordinate of the zebra.
    along: f32,
    pos: Vec2,
    a: WalkId,
    b: WalkId,
}

/// Adds a [`WalkKind::Crossing`] node in the middle of the carriageway between facing
/// sidewalk nodes, and links it to both.
///
/// Two sidewalk nodes "face" each other when the segment joining them is perpendicular to
/// a street and straddles exactly one lattice line: a Z-running street (constant `x`) is
/// crossed when the nodes differ in `x` but not in `z`, and vice versa.
///
/// Candidates are sorted by street and along-street position, duplicates dropped (facing
/// ring samples generate the same crossing twice), and then thinned to every
/// [`CROSS_EVERY`]th one — about 18 m, the spacing of signalised crossings on a real grid.
/// Anything that would land inside a junction box is skipped.
pub fn add_crossings(g: &mut WalkGraph, lines: &[f32], widest: f32, sidewalk: f32) {
    if lines.len() < 2 {
        return;
    }
    // Kerb to kerb across one street plus slack; any farther is two streets away.
    let reach = widest + 2.0 * sidewalk + 3.0;
    let junction = 0.5 * widest + 2.0;
    let clear_of_junction = |v: f32| lines.iter().all(|&l| (l - v).abs() > junction);

    let mut found: Vec<Crossing> = Vec::new();
    let mut hits: Vec<WalkId> = Vec::new();
    for a in 0..g.len() as u32 {
        if g.kind(a) == WalkKind::Crossing {
            continue;
        }
        let pa = g.pos(a);
        hits.clear();
        g.within(pa, reach, &mut hits);
        for i in 0..hits.len() {
            let b = hits[i];
            if b <= a || g.kind(b) == WalkKind::Crossing {
                continue;
            }
            let pb = g.pos(b);
            let (dx, dz) = ((pa.x - pb.x).abs(), (pa.y - pb.y).abs());
            // Vertical street (constant x): differ in x, aligned in z.
            if dx > 1.0 && dx <= reach && dz < 1.2 {
                if let Some(l) = line_between(lines, pa.x, pb.x) {
                    let mid = (pa + pb) * 0.5;
                    if clear_of_junction(mid.y) {
                        found.push(Crossing { vertical: true, line: l, along: mid.y, pos: mid, a, b });
                    }
                }
            }
            // Horizontal street (constant z): differ in z, aligned in x.
            if dz > 1.0 && dz <= reach && dx < 1.2 {
                if let Some(l) = line_between(lines, pa.y, pb.y) {
                    let mid = (pa + pb) * 0.5;
                    if clear_of_junction(mid.x) {
                        found.push(Crossing { vertical: false, line: l, along: mid.x, pos: mid, a, b });
                    }
                }
            }
        }
    }

    // Group by street, then by position along it: that is the order in which "every third
    // crossing" thins sensibly, and the order in which duplicates sit adjacent.
    found.sort_by(|p, q| {
        p.vertical
            .cmp(&q.vertical)
            .then(p.line.total_cmp(&q.line))
            .then(p.along.total_cmp(&q.along))
            .then(p.a.cmp(&q.a))
    });

    let mut last: Option<(bool, f32, f32)> = None;
    let mut rank = 0usize;
    for c in found {
        // Same zebra seen from the other side of the street (both facing ring nodes generate
        // it): keep the first, drop the rest.
        if let Some((lv, ll, la)) = last {
            if lv == c.vertical && (ll - c.line).abs() < 0.05 && (la - c.along).abs() < 2.0 {
                continue;
            }
        }
        // New street? restart the every-third counter.
        rank = match last {
            Some((lv, ll, _)) if lv == c.vertical && (ll - c.line).abs() < 0.05 => rank + 1,
            _ => 0,
        };
        last = Some((c.vertical, c.line, c.along));
        if rank % CROSS_EVERY != 0 {
            continue;
        }
        let id = g.add(c.pos, WalkKind::Crossing);
        g.link(c.a, id);
        g.link(c.b, id);
    }
}
