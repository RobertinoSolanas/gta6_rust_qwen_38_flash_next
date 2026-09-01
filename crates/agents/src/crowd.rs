//! Bounded context: **pedestrians**.
//!
//! Pedestrians walk the [`WalkGraph`]: sidewalk rings, zebra crossings, plazas and park
//! paths. The model is the classic steering pair — hold a goal node, follow the shortest
//! path to it, and steer around whoever is in the way.
//!
//! The graph is built so crossings are the *only* links between opposite kerbs, which makes
//! jaywalking impossible by construction: a walker that will not step into traffic simply
//! waits at the kerb until its crossing becomes traversable, because that is the only edge
//! that leads where it wants to go.

use gta_city::{Axis, City, Road, WalkId, WalkKind};
use gta_geo::GridIndex;
use gta_math::{clamp, wrap_angle, Rng, Vec2};

use crate::traffic::Traffic;

/// Reference walking speed, m/s.
pub const WALK_SPEED: f32 = 1.4;
/// Minimum seconds spent idling at a destination.
pub const IDLE_MIN: f32 = 0.6;
/// Shoulder-to-shoulder radius used for separation.
pub const SHOULDER: f32 = 0.45;
/// How far ahead walkers look for a new destination, metres.
pub const HORIZON: f32 = 70.0;

/// What the renderer needs for one pedestrian.
#[derive(Clone, Copy, Debug)]
pub struct PedPose {
    pub pos: Vec2,
    /// Heading in the [`crate::traffic::yaw_of`] convention (0 => `+Z`).
    pub yaw: f32,
    pub speed: f32,
    /// Walk-cycle phase 0..1 — the renderer swings the legs from this.
    pub stride: f32,
    /// 0..1 clothing colour variation.
    pub tint: f32,
    /// Height in metres.
    pub height: f32,
}

/// A pedestrian.
#[derive(Clone, Copy, Debug)]
pub struct Ped {
    /// Node we are standing on, or the one we just left.
    pub at: WalkId,
    /// Next node on the path.
    pub next: Option<WalkId>,
    /// Destination.
    pub goal: WalkId,
    pub pos: Vec2,
    pub yaw: f32,
    pub speed: f32,
    /// This walker's preferred pace, m/s.
    pub pace: f32,
    /// Seconds left standing still.
    pub rest: f32,
    pub tint: f32,
    pub height: f32,
    pub stride: f32,
    /// Mid-carriageway this frame.
    pub crossing: bool,
}

/// The pedestrian population.
pub struct Crowd {
    pub peds: Vec<Ped>,
    index: GridIndex<u32>,
}

impl Crowd {
    /// Spawns `count` pedestrians on random walkable nodes.
    pub fn spawn(city: &City, count: usize, rng: &mut Rng) -> Crowd {
        let mut peds = Vec::with_capacity(count);
        let n = city.walk.len();
        if n == 0 {
            return Crowd { peds, index: GridIndex::new(8.0, city.params.extent()) };
        }
        for _ in 0..count {
            let start = rng.below(n as u32);
            peds.push(new_ped(city, rng, start));
        }
        let mut c = Crowd { peds, index: GridIndex::new(8.0, city.params.extent()) };
        c.reindex();
        c
    }

    #[inline]
    pub fn len(&self) -> usize {
        self.peds.len()
    }

    #[inline]
    pub fn is_empty(&self) -> bool {
        self.peds.is_empty()
    }

    /// Advances the crowd by `dt`. `clock` is the signal clock from [`Traffic`], which
    /// decides when a walker may step into a crossing.
    pub fn step(&mut self, city: &City, clock: f32, dt: f32, rng: &mut Rng) {
        self.reindex();
        for i in 0..self.peds.len() {
            self.step_ped(city, clock, i, dt, rng);
        }
    }

    /// Renderer view of pedestrian `i`.
    pub fn pose(&self, i: usize) -> PedPose {
        let p = &self.peds[i];
        PedPose { pos: p.pos, yaw: p.yaw, speed: p.speed, stride: p.stride, tint: p.tint, height: p.height }
    }

    /// True when any pedestrian stands in the carriageway of the junction of `road` at
    /// crossing line `line`. Cars yield to this.
    pub fn in_junction(&self, city: &City, road: &Road, line: f32) -> bool {
        let half = city.junction_half(road.axis.other(), line);
        let centre = road.axis.place(line, road.c);
        let mut hits: Vec<u32> = Vec::new();
        self.index.query_ids(centre, half + 4.0, &mut hits);
        for k in 0..hits.len() {
            let p = &self.peds[hits[k] as usize];
            let d = p.pos - centre;
            if d.length_sq() <= half * half && city.on_road(p.pos, -0.4) {
                return true;
            }
        }
        false
    }

    fn step_ped(&mut self, city: &City, clock: f32, i: usize, dt: f32, rng: &mut Rng) {
        let mut ped = self.peds[i];

        // Resting at a destination: stand still, then pick somewhere new.
        if ped.next.is_none() {
            ped.rest -= dt;
            ped.speed = 0.0;
            ped.crossing = false;
            if ped.rest <= 0.0 {
                ped.goal = pick_goal(city, rng, ped.at);
                ped.next = first_step(city, ped.at, ped.goal);
            }
            self.peds[i] = ped;
            return;
        }

        let next = ped.next.unwrap();
        let target = city.walk.pos(next);

        // Arrived at the intermediate node: adopt it and take the following hop.
        if ped.pos.distance_sq(target) < 0.36 {
            ped.at = next;
            ped.next = None;
            let arrived = ped.at == ped.goal;
            // Occasionally divert, which stops the crowd running the same loops.
            if arrived || rng.chance(0.03) {
                ped.goal = pick_goal(city, rng, ped.at);
            }
            ped.next = first_step(city, ped.at, ped.goal);
            if ped.next.is_none() {
                ped.rest = rng.range(IDLE_MIN, 3.0);
                self.peds[i] = ped;
                return;
            }
        }

        // Held at a red man: wait on the kerb. The graph guarantees the crossing is the
        // only way on, so waiting is the whole behaviour — no deadlock, no jaywalking.
        if blocked(city, clock, &ped) {
            ped.speed = 0.0;
            ped.crossing = false;
            self.peds[i] = ped;
            return;
        }

        // Steer: seek the node, push away from close neighbours.
        let to = target - ped.pos;
        let mut want = to.normalize();
        let mut hits: Vec<u32> = Vec::new();
        self.index.query_ids(ped.pos, 2.4, &mut hits);
        for k in 0..hits.len() {
            let j = hits[k] as usize;
            if j == i {
                continue;
            }
            let o = &self.peds[j];
            let d = ped.pos - o.pos;
            let dl = d.length();
            let reach = 2.0 * SHOULDER + 0.7;
            if dl > 0.05 && dl < reach {
                let w = 1.0 - dl / reach;
                ped.pos = ped.pos + d.normalize() * (w * w * 0.5 * dt * 6.0);
                want = want + d.normalize() * (w * 0.8);
            }
        }
        if want.length_sq() > 1e-8 {
            let want = want.normalize();
            let delta = wrap_angle(yaw_of(want) - ped.yaw);
            ped.yaw += clamp(delta, -4.0 * dt, 4.0 * dt);
            // Only make ground in the direction we are actually facing.
            let align = clamp(want.dot(dir_of(ped.yaw)), 0.0, 1.0);
            ped.speed = ped.pace * align;
            ped.pos = ped.pos + dir_of(ped.yaw) * ped.speed * dt;
        } else {
            ped.speed = 0.0;
        }

        ped.crossing = city.on_road(ped.pos, -0.4);
        ped.stride = (ped.stride + ped.speed * dt * 0.5) % 1.0;
        self.peds[i] = ped;
    }

    /// Rebuilds the position index (the grid has no removal, so a fresh one per step).
    fn reindex(&mut self) {
        let extent = self
            .peds
            .first()
            .map(|_| 0.0)
            .unwrap_or(0.0);
        let _ = extent;
        let mut g = GridIndex::new(8.0, self.extent_hint());
        for i in 0..self.peds.len() {
            g.insert(self.peds[i].pos, SHOULDER, i as u32);
        }
        self.index = g;
    }

    #[inline]
    fn extent(&self) -> f32 {
        self.extent_hint()
    }

    #[inline]
    fn extent_hint(&self) -> f32 {
        self.index.extent()
    }
}

fn new_ped(city: &City, rng: &mut Rng, start: WalkId) -> Ped {
    let pos = city.walk.pos(start);
    Ped {
        at: start,
        next: None,
        goal: start,
        pos,
        yaw: rng.range(0.0, gta_math::TAU),
        speed: 0.0,
        pace: WALK_SPEED * rng.range(0.8, 1.25),
        rest: rng.range(0.0, 1.5),
        tint: rng.f32(),
        height: rng.range(1.62, 1.92),
        stride: rng.f32(),
        crossing: false,
    }
}

/// Picks a destination a block or two away: far enough to be worth walking, near enough
/// that the path search stays cheap.
fn pick_goal(city: &City, rng: &mut Rng, from: WalkId) -> WalkId {
    let origin = city.walk.pos(from);
    let mut hits: Vec<WalkId> = Vec::new();
    city.walk.within(origin, HORIZON, &mut hits);
    let mut best: Option<(WalkId, f32)> = None;
    for k in 0..hits.len() {
        let id = hits[k];
        if id == from {
            continue;
        }
        // Favour the far side of the window: nobody crosses the road for three metres.
        let d = city.walk.pos(id).distance_sq(origin);
        if d < 20.0 * 20.0 {
            continue;
        }
        if best.map_or(true, |(_, bd)| d > bd) && d < HORIZON * HORIZON {
            best = Some((id, d));
        }
    }
    best.map(|(id, _)| id).unwrap_or(from)
}

/// First hop on the shortest path `from` -> `to`.
fn first_step(city: &City, from: WalkId, to: WalkId) -> Option<WalkId> {
    city.walk.next_step(from, to).or_else(|| {
        // No route (goal unreachable right now): wander to any neighbour rather than freeze.
        let nb = city.walk.neighbours(from);
        nb.first().copied()
    })
}

/// True when `ped` wants to step onto a crossing whose signal is currently held.
fn blocked(city: &City, clock: f32, ped: &Ped) -> bool {
    let next = match ped.next {
        Some(n) => n,
        None => return false,
    };
    if city.walk.kind(next) != WalkKind::Crossing {
        return false;
    }
    // The crossing joins two kerb nodes; the street it spans runs perpendicular to the
    // kerb-to-kerb vector. Traffic on that street is what we must wait for, so we go when
    // the *crossing* street's traffic has green.
    let a = city.walk.pos(ped.at);
    let b = city.walk.pos(next);
    let spans_x = (a.x - b.x).abs() > (a.y - b.y).abs();
    let waiting_on = if spans_x { Axis::X } else { Axis::Z };
    let (line_i, phase) = match city.network.light_near(if spans_x { a } else { b }, if spans_x { b.y } else { b.x }) {
        Some(its) => (0, its.phase),
        None => return true,
    };
    let _ = line_i;
    !Traffic::green(waiting_on, clock, phase)
}

/// Heading of a planar direction (kept local: identical to [`crate::traffic::yaw_of`]).
#[inline]
fn yaw_of(d: Vec2) -> f32 {
    d.x.atan2(d.y)
}

/// Unit planar direction for a yaw angle.
#[inline]
fn dir_of(yaw: f32) -> Vec2 {
    Vec2::new(yaw.sin(), yaw.cos())
}
