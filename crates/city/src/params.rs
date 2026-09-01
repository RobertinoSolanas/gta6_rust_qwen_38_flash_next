//! Bounded context: **city layout parameters**.
//!
//! Every knob that shapes the city lives here, so generation is a pure function of
//! [`CityParams`] plus a seed. Units are metres.
//!
//! ## Street geometry
//!
//! The city is a lattice of `grid x grid` square blocks crossed by `grid + 1` streets
//! in each direction. A street is a *corridor* `road` metres wide (or `avenue` for the
//! boulevards). To keep every block exactly `block` metres across whatever the corridor
//! widths are, consecutive centre lines are spaced by
//!
//! ```text
//! gap_i = block + (w_i + w_{i+1}) / 2
//! ```
//!
//! i.e. the block plus half of each adjacent corridor, so the kerb-to-kerb buildable
//! distance is always exactly `block`. [`CityParams::lines`] accumulates those gaps and
//! recentres the lattice on the origin.

use gta_math::Rng;

/// Everything needed to regenerate the same city, byte for byte.
#[derive(Clone, Debug)]
pub struct CityParams {
    pub seed: u64,
    /// Blocks per side; the city is `grid x grid` blocks.
    pub grid: usize,
    /// Edge length of one city block (lot area, kerbs excluded).
    pub block: f32,
    /// Carriageway width of a normal street.
    pub road: f32,
    /// Carriageway width of an avenue.
    pub avenue: f32,
    /// Streets this many blocks from the centre are avenues.
    pub avenue_every: usize,
    /// Sidewalk band drawn inside the block outline.
    pub sidewalk: f32,
    /// Pavement height above street level.
    pub kerb: f32,
}

impl Default for CityParams {
    fn default() -> Self {
        CityParams {
            seed: 0x5EED_1234,
            grid: 11,
            block: 46.0,
            road: 12.0,
            avenue: 20.0,
            avenue_every: 4,
            sidewalk: 3.4,
            kerb: 0.14,
        }
    }
}

impl CityParams {
    #[inline]
    pub fn new(seed: u64) -> Self {
        CityParams { seed, ..Default::default() }
    }

    /// True when street line `i` (of `grid + 1`) is an avenue. Symmetric about the
    /// middle, which always is one, so the city has a grand central boulevard.
    #[inline]
    pub fn is_avenue(&self, i: usize) -> bool {
        let step = self.avenue_every.max(1);
        let m = self.grid / 2;
        let d = if i > m { i - m } else { m - i };
        d % step == 0
    }

    /// Carriageway width of street line `i`.
    #[inline]
    pub fn width(&self, i: usize) -> f32 {
        if self.is_avenue(i) {
            self.avenue
        } else {
            self.road
        }
    }

    /// Distance between street centre lines `i` and `i + 1`.
    #[inline]
    pub fn gap(&self, i: usize) -> f32 {
        self.block + 0.5 * (self.width(i) + self.width(i + 1))
    }

    /// Centre-line coordinates of all `grid + 1` streets, centred on the origin.
    pub fn lines(&self) -> Vec<f32> {
        let gaps: Vec<f32> = (0..self.grid).map(|i| self.gap(i)).collect();
        let total: f32 = gaps.iter().sum();
        let mut x = -0.5 * total;
        let mut out = Vec::with_capacity(self.grid + 1);
        for g in &gaps {
            out.push(x);
            x += *g;
        }
        out.push(x);
        out
    }

    /// Half the total grid span: origin to the outermost centre line.
    pub fn half(&self) -> f32 {
        0.5 * (0..self.grid).map(|i| self.gap(i)).sum::<f32>()
    }

    /// Centre-line coordinate of street `i`.
    #[inline]
    pub fn line(&self, i: usize) -> f32 {
        let i = i.clamp(0, self.grid);
        let before: f32 = (0..i).map(|k| self.gap(k)).sum();
        -self.half() + before
    }

    /// World extent used to size spatial hashes and set the far plane.
    #[inline]
    pub fn extent(&self) -> f32 {
        self.half() * 2.0 + 80.0
    }

    /// Deterministic sub-stream for one subsystem, so adding a new random consumer
    /// elsewhere cannot perturb an existing subsystem's output.
    pub fn rng(&self, salt: u64) -> Rng {
        Rng::new(self.seed.wrapping_mul(0x9E37_79B9_7F4A_7C15) ^ salt)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn lines_are_centred_and_monotonic() {
        let p = CityParams::default();
        let ls = p.lines();
        assert_eq!(ls.len(), p.grid + 1);
        assert!(ls[0] < 0.0 && *ls.last().unwrap() > 0.0);
        for w in ls.windows(2) {
            assert!(w[1] > w[0], "lines not increasing");
        }
        // Symmetric about the origin.
        assert!((ls[0] + *ls.last().unwrap()).abs() < 1e-2);
    }

    #[test]
    fn line_matches_lines() {
        let p = CityParams { grid: 8, ..Default::default() };
        let ls = p.lines();
        for i in 0..=p.grid {
            assert!((p.line(i) - ls[i]).abs() < 1e-2, "line {i}");
        }
    }

    #[test]
    fn kerb_to_kerb_is_exactly_one_block() {
        let p = CityParams { grid: 7, block: 40.0, road: 12.0, avenue: 20.0, avenue_every: 3, ..Default::default() };
        let ls = p.lines();
        for i in 0..p.grid {
            let inner_a = ls[i] + 0.5 * p.width(i);
            let inner_b = ls[i + 1] - 0.5 * p.width(i + 1);
            assert!((inner_b - inner_a - p.block).abs() < 1e-3, "block {i}");
        }
    }

    #[test]
    fn avenues_are_symmetric_and_include_the_centre() {
        let p = CityParams { grid: 11, avenue_every: 4, ..Default::default() };
        let m = p.grid / 2;
        assert!(p.is_avenue(m));
        for i in 0..=p.grid {
            let d = (i as isize - m as isize).unsigned_abs();
            assert_eq!(p.is_avenue(i), d % 4 == 0, "line {i}");
        }
        assert!(p.width(m) > p.width(m + 1));
    }

    #[test]
    fn extent_wraps_the_grid() {
        let p = CityParams::default();
        assert!(p.half() >= p.line(p.grid) - 1e-3);
        assert!(p.extent() > p.line(p.grid) * 2.0);
    }

    #[test]
    fn rng_streams_are_independent_per_salt() {
        let p = CityParams::new(1234);
        let a = p.rng(1).f32();
        let b = p.rng(2).f32();
        assert_ne!(a, b);
        assert_eq!(a, p.rng(1).f32());
    }
}
