//! Bounded context: **traffic**.
//!
//! Cars on the lane model: lane following, queueing behind the car ahead, halting at red
//! lights, yielding to pedestrians in the junction ahead, and turning at junctions.
//!
//! Deliberately not a physics sim. A car is a point mass with a length, a heading and a
//! speed. Steering is "aim at a look-ahead point on my lane and turn towards it at a
//! limited yaw rate", which is stable at any time step, keeps cars inside their lane and
//! sweeps them through the arc of a turn without any special-case junction geometry.
//!
//! ## Conventions
//!
//! [`gta_math::Vec2`] holds `(x, z)`; heading follows [`gta_math::Vec3::from_yaw`], i.e.
//! yaw `0` points along `+Z` and increases towards `+X`. Use [`yaw_of`] / [`dir_from_yaw`]
//! to convert, both of which agree with `Vec3::from_yaw` so the renderer can rotate a car
//! mesh with the same number.

use gta_city::{Axis, City, Road};
use gta_geo::GridIndex;
use gta_math::{clamp, wrap_angle, Rng, Vec2};

/// Seconds of green for one phase.
pub const GREEN_TIME: f32 = 9.0;
/// Seconds of all-red between phases.
pub const ALL_RED: f32 = 1.5;
/// Full signal cycle: X green, clearance, Z green, clearance.
pub const CYCLE: f32 = 2.0 * GREEN_TIME + 2.0 * ALL_RED;
/// Standstill stand-off a driver keeps to the car ahead, metres.
pub const BUMPER: f32 = 2.4;
/// Yaw rate ceiling, rad/s (scaled down as speed rises).
pub const YAW_RATE: f32 = 2.0;
/// Hard braking deceleration, m/s^2.
pub const BRAKE: f32 = 9.0;

/// A vehicle.
#[derive(Clone, Copy, Debug)]
pub struct Car {
    /// Road this car is driving on.
    pub road: u32,
    /// Direction of travel along that road.
    pub forward: bool,
    /// Lane index within its direction of travel.
    pub lane: usize,
    /// World position in the XZ plane.
    pub pos: Vec2,
    /// Heading in the [`gta_math::Vec3::from_yaw`] convention (0 => `+Z`).
    pub yaw: f32,
    /// Speed, m/s.
    pub speed: f32,
    /// This driver's preferred speed, m/s.
    pub limit: f32,
    /// Body length / width, metres.
    pub length: f32,
    pub width: f32,
    /// 0..1 body colour variation.
    pub tint: f32,
    /// Vans and buses: longer, taller, lazier off the line.
    pub heavy: bool,
    /// Brake lights are on this frame.
    pub braking: bool,
    /// Manoeuvre picked at the last junction, executed at its centre.
    pub turn: Option<Turn>,
    /// Lattice line of the junction already decided about, so the decision is made once
    /// per junction instead of every frame. `NAN` = none.
    pub handled: f32,
}

/// A junction manoeuvre.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Turn {
    /// Keep straight on.
    Straight,
    /// Turn to the driver's right.
    Right,
    /// Turn across oncoming traffic to the driver's left.
    Left,
}

/// Everything the renderer needs for one car.
#[derive(Clone, Copy, Debug)]
pub struct CarPose {
    pub pos: Vec2,
    pub yaw: f32,
    pub speed: f32,
    pub braking: bool,
    pub heavy: bool,
    pub tint: f32,
    pub length: f32,
    pub width: f32,
}

/// The traffic population.
pub struct Traffic {
    pub cars: Vec<Car>,
    /// Positions for "is there a car in front of me" queries; rebuilt each step because
    /// [`gta_geo::GridIndex`] has no removal.
    index: GridIndex<u32>,
    /// Signal clock, seconds.
    clock: f32,
}

impl Traffic {
    /// Spawns `count` cars spread over the network.
    pub fn spawn(city: &City, count: usize, rng: &mut Rng) -> Traffic {
        let mut cars = Vec::with_capacity(count);
        let nlines = city.network.lines.len();
        for _ in 0..count {
            let axis = if rng.chance(0.5) { Axis::X } else { Axis::Z };
            let line = rng.below(nlines as u32) as usize;
            let id = city.network.road_id(axis, line);
            let road = city.network.road(id);
            cars.push(make_car(rng, road, rng.chance(0.5), rng.below(road.lanes_per_dir as u32) as usize, rng.range(road.from + 6.0, road.to - 6.0)));
        }
        let mut t = Traffic { cars, index: GridIndex::new(20.0, city.params.extent()), clock: 0.0 };
        t.reindex(city);
        t
    }

    #[inline]
    pub fn len(&self) -> usize {
        self.cars.len()
    }

    #[inline]
    pub fn is_empty(&self) -> bool {
        self.cars.is_empty()
    }

    /// Signal clock, seconds.
    #[inline]
    pub fn clock(&self) -> f32 {
        self.clock
    }

    /// Green for traffic running on `axis` at a junction with light `phase` (that
    /// intersection's offset in the cycle)?
    ///
    /// The cycle runs X green, clearance, Z green, clearance. Staggering `phase` per
    /// intersection turns an avenue into a green wave for through traffic.
    #[inline]
    pub fn green(axis: Axis, t: f32, phase: f32) -> bool {
        let u = (t / CYCLE + phase).rem_euclid(1.0) * CYCLE;
        match axis {
            Axis::X => u < GREEN_TIME,
            Axis::Z => u >= GREEN_TIME + ALL_RED && u < 2.0 * GREEN_TIME + ALL_RED,
        }
    }

    /// Renderer view of car `i`.
    pub fn pose(&self, i: usize) -> CarPose {
        let c = &self.cars[i];
        CarPose {
            pos: c.pos,
            yaw: c.yaw,
            speed: c.speed,
            braking: c.braking,
            heavy: c.heavy,
            tint: c.tint,
            length: c.length,
            width: c.width,
        }
    }

    /// Advances traffic by `dt`. `crowd` supplies the pedestrians a car must yield to.
    pub fn step(&mut self, city: &City, crowd: &Crowd, dt: f32, rng: &mut Rng) {
        self.clock += dt;
        self.reindex(city);
        for i in 0..self.cars.len() {
            self.step_car(city, crowd, i, dt, rng);
        }
    }

    fn step_car(&mut self, city: &City, crowd: &Crowd, i: usize, dt: f32, rng: &mut Rng) {
        // Borrowed by value: `Car` is small and `self.cars` must stay free to mutate.
        let car = self.cars[i];
        let road = city.network.road(car.road);
        let sign = if car.forward { 1.0 } else { -1.0 };
        let along = road.axis.along_of(car.pos);

        // Drifted off the end of the grid? Recycle the car elsewhere: the population is
        // fixed, so traffic leaves at one edge and re-enters at another.
        if along > road.to + 6.0 || along < road.from - 6.0 {
            self.respawn(city, i, rng);
            return;
        }

        let next = road.next_crossing(along, car.forward, &city.network.lines);
        let mut want = car.limit;

        // Queue behind the car ahead: `room` is the clear space in front of my bumper.
        if let Some(gap) = self.gap_ahead(i, road, car.forward, car.lane, along) {
            let room = gap - BUMPER;
            if room <= 0.0 {
                want = 0.0;
            } else {
                // Ease speed with the clear space: cruise when far clear, crawl when tight.
                want = want.min(room * 0.9);
            }
        }

        // Signal: the cross-street has the green, so hold at the box.
        let mut holding = false;
        if let Some(l) = next {
            let phase = city
                .network
                .intersection_at(road, l)
                .map(|its| its.phase)
                .unwrap_or(0.0);
            if !Traffic::green(road.axis.other(), self.clock, phase) {
                let to_stop = (l - along).abs() - city.junction_half(l);
                want = if to_stop > 3.0 { (to_stop * 1.1).min(want) } else { 0.0 };
                holding = true;
            }
        }

        // Yield to pedestrians standing in the junction ahead, green or not.
        if !holding {
            if let Some(l) = next {
                let d = (l - along).abs();
                if d < 20.0 && crowd.in_junction(city, road, l) {
                    want = want.min(if d > 7.0 { 1.6 } else { 0.0 });
                }
            }
        }

        // Longitudinal integration.
        let accel = if car.heavy { 2.4 } else { 4.6 };
        let braking = want < car.speed - 0.3;
        let speed = if braking {
            (car.speed - BRAKE * dt).max(want.max(0.0))
        } else {
            (car.speed + accel * dt).min(want.max(0.0))
        };

        // Decide a manoeuvre once per junction, then commit at the centre of the box.
        let mut turn = car.turn;
        let mut handled = car.handled;
        if let Some(l) = next {
            if (handled - l).abs() > 0.5 && (l - along).abs() < 22.0 {
                handled = l;
                // Most drivers drive through; of the rest, right turns beat left ones
                // because turning across oncoming traffic is impolite.
                turn = Some(if rng.chance(0.6) {
                    Turn::Straight
                } else if rng.chance(0.62) {
                    Turn::Right
                } else {
                    Turn::Left
                });
            }
            let passed = (l - along) * sign >= 0.0;
            if passed {
                if let Some(t) = car.turn {
                    if t != Turn::Straight {
                        self.enter_turn(city, i, road, l, t);
                    }
                    turn = None;
                }
            }
        }

        // Steering: aim at a look-ahead point on the lane and turn towards it. The yaw
        // rate falls off with speed, so a car crawling out of a junction can still complete
        // the arc while fast traffic holds a straight line.
        let look = 5.0 + speed * 0.6;
        let target = road.lane_point(along + sign * look, car.forward, car.lane);
        let delta = wrap_angle(yaw_of(target - car.pos) - car.yaw);
        let max_turn = YAW_RATE * dt * clamp(1.3 - speed * 0.05, 0.35, 1.0);
        let yaw = car.yaw + clamp(delta, -max_turn, max_turn);
        let pos = car.pos + dir_from_yaw(yaw) * speed * dt;

        let c = &mut self.cars[i];
        c.pos = pos;
        c.yaw = yaw;
        c.speed = speed;
        c.braking = braking;
        c.turn = turn;
        c.handled = handled;
    }

    /// Switches car `i` onto the street it is turning into at crossing line `line`.
    ///
    /// Only the route changes; position and heading are untouched, so the car continues
    /// through the junction on its current momentum while lane-following steering pulls it
    /// into the new lane. No snapping, no junction geometry.
    fn enter_turn(&mut self, city: &City, i: usize, from: &Road, line: f32, right: bool) {
        let axis = from.axis.other();
        let forward = turned_forward(from.axis, from.forward, right);
        let new_line = city.network.nearest_line(line);
        let id = city.network.road_id(axis, new_line);
        let road = city.network.road(id);
        // Right turns hug the kerb; left turns come out in the inner lane.
        let lane = if right { 0 } else { road.lanes_per_dir - 1 };
        let c = &mut self.cars[i];
        c.road = id;
        c.forward = forward;
        c.lane = lane;
        c.turn = None;
    }

    /// Puts car `i` back on a random street at a random spot, out of other cars' way.
    fn respawn(&mut self, city: &City, i: usize, rng: &mut Rng) {
        let nlines = city.network.lines.len();
        for _ in 0..12 {
            let axis = if rng.chance(0.5) { Axis::X } else { Axis::Z };
            let line = rng.below(nlines as u32) as usize;
            let id = city.network.road_id(axis, line);
            let road = city.network.road(id);
            let forward = rng.chance(0.5);
            let lane = rng.below(road.lanes_per_dir as u32) as usize;
            let along = rng.range(road.from + 6.0, road.to - 6.0);
            let p = road.lane_point(along, forward, lane);
            if !self.occupied(p, 11.0) {
                let c = &mut self.cars[i];
                c.road = id;
                c.forward = forward;
                c.lane = lane;
                c.pos = p;
                c.yaw = yaw_of(axis.dir(forward));
                c.speed = c.limit * 0.4;
                c.turn = None;
                c.handled = f32::NAN;
                return;
            }
        }
        // Nowhere clear: park the car at the very end of a street.
        let id = city.network.road_id(Axis::X, 0);
        let road = city.network.road(id);
        let c = &mut self.cars[i];
        c.road = id;
        c.forward = true;
        c.lane = 0;
        c.pos = road.lane_point(road.from + 2.0, true, 0);
        c.yaw = yaw_of(Axis::X.dir(true));
        c.speed = 0.0;
        c.turn = None;
        c.handled = f32::NAN;
    }

    /// Gap in metres from my bumper to the rear of the nearest car ahead in my lane.
    fn gap_ahead(&self, me: usize, road: &Road, forward: bool, lane: usize, along: f32) -> Option<f32> {
        let mut hits: Vec<u32> = Vec::new();
        let mine = self.cars[me];
        self.index.query_ids(mine.pos, 26.0, &mut hits);
        let mut best: Option<f32> = None;
        for k in 0..hits.len() {
            let j = hits[k] as usize;
            if j == me {
                continue;
            }
            let o = &self.cars[j];
            if o.road != mine.road || o.forward != forward || o.lane != lane {
                continue;
            }
            let d = (road.axis.along_of(o.pos) - along) * if forward { 1.0 } else { -1.0 };
            if d <= 0.0 {
                continue;
            }
            let room = d - o.length;
            if best.map_or(true, |b: f32| room < b) {
                best = Some(room);
            }
        }
        best
    }

    /// True when another car already occupies `p` — keeps respawns out of live traffic.
    fn occupied(&self, p: Vec2, radius: f32) -> bool {
        let mut hits: Vec<u32> = Vec::new();
        self.index.query_ids(p, radius, &mut hits);
        hits.iter()
            .any(|&id| self.cars[id as usize].pos.distance_sq(p) < radius * radius)
    }

    fn reindex(&mut self, city: &City) {
        let mut g = GridIndex::new(20.0, city.params.extent());
        for i in 0..self.cars.len() {
            let c = &self.cars[i];
            g.insert(c.pos, 0.5, i as u32);
        }
        self.index = g;
    }
}

/// Builds a car on `road` bound for `forward` in `lane` at along-street `along`.
pub fn make_car(rng: &mut Rng, road: &Road, forward: bool, lane: usize, along: f32) -> Car {
    let heavy = rng.chance(0.14);
    let base = if road.avenue { 13.5 } else { 9.5 };
    let (length, width) = if heavy {
        (rng.range(5.4, 7.6), rng.range(2.1, 2.45))
    } else {
        (rng.range(3.9, 4.8), rng.range(1.7, 1.95))
    };
    Car {
        road: road.id,
        forward,
        lane,
        pos: road.lane_point(along, forward, lane),
        yaw: yaw_of(road.axis.dir(forward)),
        speed: base * 0.5,
        limit: base * rng.range(0.85, 1.12),
        length,
        width,
        tint: rng.f32(),
        heavy,
        braking: false,
        turn: None,
        handled: f32::NAN,
    }
}

/// Heading of a planar direction, in the [`gta_math::Vec3::from_yaw`] convention.
#[inline]
pub fn yaw_of(d: Vec2) -> f32 {
    d.x.atan2(d.y)
}

/// Unit planar direction for a yaw angle; the inverse of [`yaw_of`].
#[inline]
pub fn dir_from_yaw(yaw: f32) -> Vec2 {
    Vec2::new(yaw.sin(), yaw.cos())
}

/// Heading after turning at a junction (right-hand traffic).
///
/// The driver's right is [`Vec2::perp`] of the direction of travel, so from `+X` a right
/// turn heads `+Z`, from `+Z` it heads `-X`, and so on.
pub fn turned_forward(axis: Axis, forward: bool, right: bool) -> bool {
    let d = axis.dir(forward);
    let r = if right { d.perp() } else { -d.perp() };
    match axis.other() {
        Axis::X => r.x >= 0.0,
        Axis::Z => r.y >= 0.0,
    }
}
