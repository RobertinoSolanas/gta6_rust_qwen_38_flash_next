//! Bounded context: **streets and lanes**.
//!
//! The carriageway model. Streets are axis-aligned corridors sitting on the lattice
//! lines; each carries a fixed number of lanes per direction and they cross at
//! intersections which own a traffic-light phase.
//!
//! ## Direction convention
//!
//! `Vec2` holds `(x, z)` and `+Y` is up. A driver's *right* hand side is [`Vec2::perp`]
//! of the direction of travel, so traffic heading `+X` occupies the `+z` half of its
//! corridor and traffic heading `-X` the `-z` half. Opposite directions therefore always
//! sit in opposite halves — two-way streets never overlap.

use gta_math::{Vec2, Vec3};

/// Which way a street runs.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Axis {
    /// Runs east–west; centre line at constant `z`.
    X,
    /// Runs north–south; centre line at constant `x`.
    Z,
}

impl Axis {
    #[inline]
    pub fn other(self) -> Axis {
        match self {
            Axis::X => Axis::Z,
            Axis::Z => Axis::X,
        }
    }

    /// Unit direction of travel.
    #[inline]
    pub fn dir(self, forward: bool) -> Vec2 {
        match (self, forward) {
            (Axis::X, true) => Vec2::new(1.0, 0.0),
            (Axis::X, false) => Vec2::new(-1.0, 0.0),
            (Axis::Z, true) => Vec2::new(0.0, 1.0),
            (Axis::Z, false) => Vec2::new(0.0, -1.0),
        }
    }

    /// The constant coordinate of a point on this axis' street.
    #[inline]
    pub fn cross_of(self, p: Vec2) -> f32 {
        match self {
            Axis::X => p.y,
            Axis::Z => p.x,
        }
    }

    /// The along-street coordinate.
    #[inline]
    pub fn along_of(self, p: Vec2) -> f32 {
        match self {
            Axis::X => p.x,
            Axis::Z => p.y,
        }
    }

    /// Builds a world point from (along, cross).
    #[inline]
    pub fn place(self, along: f32, cross: f32) -> Vec2 {
        match self {
            Axis::X => Vec2::new(along, cross),
            Axis::Z => Vec2::new(cross, along),
        }
    }

    /// Unit vector to the driver's right.
    #[inline]
    pub fn right_of(self, forward: bool) -> Vec2 {
        self.dir(forward).perp()
    }
}

/// Residual kerb margin kept clear at the outside of the carriageway.
const KERB_MARGIN: f32 = 1.0;
/// Half-width of the central reservation on avenues.
const MEDIAN_HALF: f32 = 1.6;

/// One street corridor.
#[derive(Clone, Copy, Debug)]
pub struct Road {
    pub id: u32,
    pub axis: Axis,
    /// Lattice line index.
    pub line: usize,
    /// Constant coordinate of the centre line.
    pub c: f32,
    /// Along-street span.
    pub from: f32,
    pub to: f32,
    /// Carriageway width.
    pub width: f32,
    pub avenue: bool,
    pub lanes_per_dir: usize,
}

impl Road {
    /// Usable carriageway half-width: carriageway half-width less kerb margin and, on
    /// avenues, the central reservation.
    #[inline]
    fn half_usable(&self) -> f32 {
        let median = if self.avenue { MEDIAN_HALF } else { 0.0 };
        (self.width * 0.5 - KERB_MARGIN - median).max(0.5)
    }

    /// Width of one lane on this street.
    #[inline]
    pub fn lane_width(&self) -> f32 {
        (self.half_usable() / self.lanes_per_dir.max(1) as f32).max(1.0)
    }

    /// Signed lateral offset of lane `lane` for `forward` traffic (positive = the
    /// driver's right). Always inside the carriageway.
    pub fn lane_offset(&self, forward: bool, lane: usize) -> f32 {
        let median = if self.avenue { MEDIAN_HALF } else { 0.0 };
        let lanes = self.lanes_per_dir.max(1);
        let lane_w = self.half_usable() / lanes as f32;
        let k = lane.min(lanes - 1) as f32;
        let off = median + lane_w * (k + 0.5);
        if forward {
            off
        } else {
            -off
        }
    }

    /// World centre-line position of a lane.
    #[inline]
    pub fn lane_point(&self, along: f32, forward: bool, lane: usize) -> Vec2 {
        let off = self.lane_offset(forward, lane);
        self.axis.place(along, self.c + off)
    }

    /// Lane position in 3D at height `y`.
    #[inline]
    pub fn lane_point3(&self, along: f32, forward: bool, lane: usize, y: f32) -> Vec3 {
        self.lane_point(along, forward, lane).to_vec3(y)
    }

    /// Direction a vehicle at `p` is travelling in (right-hand side of the corridor).
    #[inline]
    pub fn forward_at(&self, p: Vec2) -> bool {
        self.off_of(p) >= 0.0
    }

    /// Signed lateral offset of `p` from the centre line, positive = driver's right.
    #[inline]
    pub fn off_of(&self, p: Vec2) -> f32 {
        self.axis.cross_of(p) - self.c
    }

    /// Is `p` inside the carriageway (plus `pad`)?
    #[inline]
    pub fn covers(&self, p: Vec2, pad: f32) -> bool {
        self.off_of(p).abs() <= self.width * 0.5 + pad
    }

    /// The next crossing line ahead of `along` (the nearest lattice line that is not this
    /// street's own centre line), or `None` at the far end of the grid.
    pub fn next_crossing(&self, along: f32, forward: bool, lines: &[f32]) -> Option<f32> {
        let mut best: Option<f32> = None;
        for &l in lines {
            if (l - self.c).abs() < 1e-3 || !ahead_of(along, l, forward) {
                continue;
            }
            if best.map_or(true, |b| (l - along).abs() < (b - along).abs()) {
                best = Some(l);
            }
        }
        best
    }

    /// Midpoint of the street's span — handy for placing a test vehicle.
    #[inline]
    pub fn mid(&self) -> f32 {
        0.5 * (self.from + self.to)
    }
}

#[inline]
fn ahead_of(from: f32, target: f32, forward: bool) -> bool {
    if forward {
        target > from
    } else {
        target < from
    }
}

/// Crossing of an X-street and a Z-street.
#[derive(Clone, Copy, Debug)]
pub struct Intersection {
    pub id: u32,
    pub ix: usize,
    pub iz: usize,
    pub pos: Vec2,
    /// Half extents of the junction box.
    pub half_x: f32,
    pub half_z: f32,
    /// Light-cycle offset in seconds (green-wave stagger).
    pub phase: f32,
}

impl Intersection {
    /// Junction radius: the inscribed half-size, used for "stop before the box".
    #[inline]
    pub fn radius(&self) -> f32 {
        self.half_x.max(self.half_z)
    }
}

/// The street network.
#[derive(Clone, Debug)]
pub struct Network {
    pub roads: Vec<Road>,
    pub intersections: Vec<Intersection>,
    /// Shared lattice lines for both axes.
    pub lines: Vec<f32>,
    half: f32,
}

impl Network {
    /// Builds the lattice. Kept decoupled from `CityParams` — it only needs the numbers.
    pub fn build(
        lines: &[f32],
        width_of: impl Fn(usize) -> f32,
        avenue_of: impl Fn(usize) -> bool,
        lanes_of: impl Fn(usize) -> usize,
        half: f32,
    ) -> Network {
        let n = lines.len();
        assert!(n >= 2, "need at least two lattice lines");
        let from = lines[0] - 0.5 * width_of(0);
        let to = lines[n - 1] + 0.5 * width_of(n - 1);

        let mut roads = Vec::with_capacity(n * 2);
        for i in 0..n {
            let (w, av, lanes) = (width_of(i), avenue_of(i), lanes_of(i).max(1));
            roads.push(Road {
                id: (i * 2) as u32,
                axis: Axis::X,
                line: i,
                c: lines[i],
                from,
                to,
                width: w,
                avenue: av,
                lanes_per_dir: lanes,
            });
            roads.push(Road {
                id: (i * 2 + 1) as u32,
                axis: Axis::Z,
                line: i,
                c: lines[i],
                from,
                to,
                width: w,
                avenue: av,
                lanes_per_dir: lanes,
            });
        }

        let mut intersections = Vec::with_capacity(n * n);
        for iz in 0..n {
            for ix in 0..n {
                intersections.push(Intersection {
                    id: intersections.len() as u32,
                    ix,
                    iz,
                    pos: Vec2::new(lines[ix], lines[iz]),
                    half_x: 0.5 * width_of(ix),
                    half_z: 0.5 * width_of(iz),
                    phase: 0.0,
                });
            }
        }
        // Green-wave stagger: phase follows position so the central avenues mostly
        // read green to through traffic.
        for its in &mut intersections {
            let d = (its.pos.x + its.pos.y) * 0.015;
            its.phase = (d - d.floor()).abs();
        }

        Network { roads, intersections, lines: lines.to_vec(), half }
    }

    #[inline]
    pub fn road(&self, id: u32) -> &Road {
        &self.roads[id as usize]
    }

    #[inline]
    pub fn road_count(&self) -> usize {
        self.roads.len()
    }

    #[inline]
    pub fn intersection(&self, ix: usize, iz: usize) -> Option<&Intersection> {
        let n = self.lines.len();
        if ix >= n || iz >= n {
            return None;
        }
        self.intersections.get(ix + iz * n)
    }

    /// Road id for a given axis and lattice line.
    #[inline]
    pub fn road_id(&self, axis: Axis, line: usize) -> u32 {
        (line * 2 + if axis == Axis::X { 0 } else { 1 }) as u32
    }

    /// The two roads (X-running, Z-running) nearest `p`.
    #[inline]
    pub fn streets_at(&self, p: Vec2) -> (u32, u32) {
        (
            self.road_id(Axis::Z, self.nearest_line(p.x)),
            self.road_id(Axis::X, self.nearest_line(p.y)),
        )
    }

    /// Nearest lattice line index to coordinate `v`.
    pub fn nearest_line(&self, v: f32) -> usize {
        let mut best = 0usize;
        let mut bd = f32::MAX;
        for (i, &l) in self.lines.iter().enumerate() {
            let d = (l - v).abs();
            if d < bd {
                bd = d;
                best = i;
            }
        }
        best
    }

    /// The junction where `road` meets the street on lattice line `line`.
    pub fn intersection_at(&self, road: &Road, line: f32) -> Option<&Intersection> {
        let i = self.nearest_line(line);
        match road.axis {
            // Running east-west: the crossing street fixes x, this road fixes z.
            Axis::X => self.intersection(i, road.line),
            Axis::Z => self.intersection(road.line, i),
        }
    }

    /// Junction nearest `p` — borrows a signal phase when the exact road pair is unknown
    /// (a pedestrian mid-crossing, a HUD readout).
    pub fn intersection_near(&self, p: Vec2) -> Option<&Intersection> {
        self.intersection(self.nearest_line(p.x), self.nearest_line(p.y))
    }

    /// Crossing lines (lattice lines other than the road's own) — what a driver on this
    /// street will meet.
    pub fn crossing_lines(&self, road: &Road) -> Vec<f32> {
        self.lines
            .iter()
            .filter(|&&l| (l - road.c).abs() > 1e-3)
            .copied()
            .collect()
    }

    #[inline]
    pub fn half(&self) -> f32 {
        self.half
    }

    /// True when `p` is on any carriageway — used to keep trees, trees and spawn
    /// points out of the road.
    pub fn on_road(&self, p: Vec2, pad: f32) -> bool {
        self.roads.iter().any(|r| r.covers(p, pad))
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::params::CityParams;

    fn net() -> Network {
        let p = CityParams { grid: 5, block: 40.0, road: 12.0, avenue: 20.0, avenue_every: 2, ..Default::default() };
        let lines = p.lines();
        Network::build(&lines, |i| p.width(i), |i| p.is_avenue(i), |i| if p.is_avenue(i) { 2 } else { 1 }, p.half())
    }

    #[test]
    fn axis_helpers_agree_with_perp() {
        assert_eq!(Axis::X.right_of(true), Vec2::new(0.0, 1.0));
        assert_eq!(Axis::X.right_of(false), Vec2::new(0.0, -1.0));
        assert_eq!(Axis::Z.right_of(true), Vec2::new(-1.0, 0.0));
        assert_eq!(Axis::X.place(3.0, 7.0), Vec2::new(3.0, 7.0));
        assert_eq!(Axis::Z.place(3.0, 7.0), Vec2::new(7.0, 3.0));
        assert_eq!(Axis::X.along_of(Vec2::new(4.0, 9.0)), 4.0);
        assert_eq!(Axis::X.cross_of(Vec2::new(4.0, 9.0)), 9.0);
    }

    #[test]
    fn opposite_directions_use_opposite_halves() {
        let n = net();
        let r = n.road(n.road_id(Axis::X, 1));
        let a = r.lane_point(0.0, true, 0);
        let b = r.lane_point(0.0, false, 0);
        let (off_a, off_b) = (r.off_of(a), r.off_of(b));
        assert!(off_a > 0.0 && off_b < 0.0, "a={off_a} b={off_b}");
        assert!((off_a + off_b).abs() < 1e-5, "lanes must mirror about the centre line");
    }

    #[test]
    fn lanes_stay_inside_the_corridor() {
        let n = net();
        for r in &n.roads {
            for &f in &[true, false] {
                for lane in 0..r.lanes_per_dir {
                    let off = r.lane_offset(f, lane).abs();
                    let lane_w = r.lane_width();
                    assert!(
                        off + lane_w * 0.5 <= r.width * 0.5 + 1e-3,
                        "road {} lane {lane} spills: off {off} lane_w {lane_w}",
                        r.width
                    );
                    assert!(off >= lane_w * 0.5 - 1e-3);
                }
            }
        }
    }

    #[test]
    fn lane_point_matches_place_and_offset() {
        let n = net();
        let r = n.road(n.road_id(Axis::Z, 2));
        let p = r.lane_point(11.0, true, 0);
        let off = r.lane_offset(true, 0);
        assert_eq!(p, Vec2::new(r.c + off, 11.0));
        assert!((r.axis.along_of(p) - 11.0).abs() < 1e-5);
    }

    #[test]
    fn forward_at_reads_the_side_of_the_road() {
        let n = net();
        let r = n.road(n.road_id(Axis::X, 0));
        let up = r.lane_point(0.0, true, 0);
        let down = r.lane_point(0.0, false, 0);
        assert!(r.forward_at(up));
        assert!(!r.forward_at(down));
        assert!(r.covers(up, 0.0) && r.covers(down, 0.0));
        assert!(!r.covers(r.axis.place(0.0, r.c + r.width), 0.0));
    }

    #[test]
    fn network_has_a_road_per_line_per_axis() {
        let n = net();
        assert_eq!(n.road_count(), 2 * n.lines.len());
        assert_eq!(n.intersections.len(), n.lines.len() * n.lines.len());
        assert!(n.intersection(0, 0).is_some());
        assert!(n.intersection(0, n.lines.len()).is_none());
    }

    #[test]
    fn nearest_line_rounds_correctly() {
        let n = net();
        for (i, &l) in n.lines.iter().enumerate() {
            assert_eq!(n.nearest_line(l), i, "line {i}");
            assert_eq!(n.nearest_line(l + 0.3), i);
        }
    }

    #[test]
    fn crossings_exclude_own_line() {
        let n = net();
        let r = n.road(n.road_id(Axis::X, 2));
        let cs = n.crossing_lines(r);
        assert!(!cs.contains(&r.c));
        assert_eq!(cs.len(), n.lines.len() - 1);
    }

    #[test]
    fn next_crossing_is_ahead() {
        let n = net();
        let r = n.road(n.road_id(Axis::X, 0));
        let cs = n.crossing_lines(r);
        let mid = r.mid();
        let ahead = r.next_crossing(mid, true, &cs).expect("crossing ahead");
        assert!(ahead > mid, "ahead {ahead} from {mid}");
        let back = r.next_crossing(mid, false, &cs).expect("crossing behind");
        assert!(back < mid);
        // Nothing lies ahead of the last crossing.
        assert_eq!(r.next_crossing(*cs.last().unwrap(), true, &cs), None);
        // And the answer is never the street's own centre line.
        assert!((ahead - r.c).abs() > 1e-3);
    }

    #[test]
    fn on_road_matches_covers() {
        let n = net();
        let r = n.road(n.road_id(Axis::Z, 3));
        assert!(n.on_road(r.lane_point(0.0, true, 0), 0.0));
        assert!(!n.on_road(Vec2::new(r.c + r.width * 2.0, 0.0), 0.0));
    }

    #[test]
    fn phases_are_in_unit_range() {
        let n = net();
        for i in &n.intersections {
            assert!((0.0..1.0).contains(&i.phase), "phase {}", i.phase);
        }
    }
}
