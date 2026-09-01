//! Bounded context: **zoning**.
//!
//! What a block *is*: downtown towers, low-rise housing, a park, a transit plaza. The
//! zone drives building massing, materials and where trees go; it is deliberately a
//! small vocabulary so the city reads as a planned place rather than random boxes.

use gta_math::{Rng, Vec2};

/// Land-use character of a block.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub enum Zone {
    /// 12–45 storey towers, tight setbacks, glass and steel.
    Downtown,
    /// 5–12 storey mixed-use: shops at street level, flats above.
    MidRise,
    /// 2–4 storey terraced housing and small workshops.
    LowRise,
    /// Suburban-feeling detached houses with gardens.
    Residential,
    /// Parkland: lawn, trees, ponds, winding paths.
    Park,
    /// Open civic plaza: paving, fountains, flags, no buildings.
    Plaza,
    /// Surface car park with marked stalls and light masts.
    Parking,
}

impl Zone {
    /// Probability that a block of this zone becomes green space / civic open space.
    #[inline]
    pub fn openness(self) -> f32 {
        match self {
            Zone::Downtown => 0.12,
            Zone::MidRise => 0.2,
            Zone::LowRise => 0.3,
            Zone::Residential => 0.36,
            _ => 1.0,
        }
    }

    /// Typical ground-floor height in metres.
    #[inline]
    pub fn floor_height(self) -> f32 {
        match self {
            Zone::Downtown => 3.9,
            Zone::MidRise => 3.35,
            Zone::LowRise => 3.0,
            Zone::Residential => 2.85,
            _ => 3.0,
        }
    }

    /// Storey-count envelope `(min, max)` before noise.
    #[inline]
    pub fn floors_envelope(self) -> (usize, usize) {
        match self {
            Zone::Downtown => (8, 42),
            Zone::MidRise => (3, 12),
            Zone::LowRise => (1, 4),
            Zone::Residential => (1, 3),
            _ => (0, 0),
        }
    }

    /// How much of the lot a building may cover (footprint ratio).
    #[inline]
    pub fn site_coverage(self) -> f32 {
        match self {
            Zone::Downtown => 0.74,
            Zone::MidRise => 0.8,
            Zone::LowRise => 0.84,
            Zone::Residential => 0.55,
            _ => 0.05,
        }
    }

    /// Density of street trees along this zone's frontage.
    #[inline]
    pub fn tree_density(self) -> f32 {
        match self {
            Zone::Park => 1.0,
            Zone::Residential => 0.75,
            Zone::LowRise => 0.55,
            Zone::MidRise => 0.4,
            Zone::Downtown => 0.3,
            Zone::Plaza => 0.35,
            Zone::Parking => 0.15,
        }
    }

    #[inline]
    pub fn is_open(self) -> bool {
        matches!(self, Zone::Park | Zone::Plaza | Zone::Parking)
    }
}

/// How a building's facade is drawn. Drives the facade generator in `scene`.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Facade {
    /// Curtain wall: continuous glazing, thin mullions.
    CurtainWall,
    /// Concrete frame with punched glazing and stone spandrels.
    FrameGrid,
    /// Brick / stucco with regular sash windows and balconies.
    Masonry,
    /// Retail base: large glazing, awning, signage band.
    Shopfront,
    /// Multi-storey car park: open decks with ribbed concrete.
    CarPark,
    /// House: solid walls, pitched roof, chimney.
    House,
}

impl Facade {
    /// Window bay width in metres.
    #[inline]
    pub fn bay(self) -> f32 {
        match self {
            Facade::CurtainWall => 2.6,
            Facade::FrameGrid => 3.4,
            Facade::Shopfront => 4.6,
            Facade::CarPark => 2.4,
            Facade::Masonry => 3.0,
            Facade::House => 2.4,
        }
    }

    /// Fraction of a storey that is glazing (rest is spandrel).
    #[inline]
    pub fn glazing(self) -> f32 {
        match self {
            Facade::CurtainWall => 0.82,
            Facade::FrameGrid => 0.56,
            Facade::Masonry => 0.46,
            Facade::Shopfront => 0.62,
            Facade::CarPark => 0.12,
            Facade::House => 0.4,
        }
    }

    /// How strongly lit windows glow at night (0..1).
    #[inline]
    pub fn night_life(self) -> f32 {
        match self {
            Facade::CurtainWall => 0.55,
            Facade::FrameGrid => 0.75,
            Facade::Masonry => 0.6,
            Facade::Shopfront => 1.0,
            Facade::CarPark => 0.35,
            Facade::House => 0.5,
        }
    }
}

/// Chooses zones and massing from a seed. Kept separate from generation so the
/// "feel" of the city can be tuned (and tested) on its own.
pub struct ZonePlanner<'a> {
    half: f32,
    rng: &'a mut Rng,
}

impl<'a> ZonePlanner<'a> {
    pub fn new(half: f32, rng: &'a mut Rng) -> Self {
        ZonePlanner { half, rng }
    }

    /// Radial zoning with noise so districts are irregular, plus occasional open
    /// space injected where it does the most good.
    pub fn zone_of(&mut self, centre: Vec2, index: u32) -> Zone {
        let d = centre.length() / self.half.max(1.0);
        let core = gta_math::smoothstep(gta_math::clamp(1.0 - d * 1.25, 0.0, 1.0));
        // A second, weaker downtown off-centre makes the skyline two-nucleus,
        // which reads far more like a real city than one perfect cone.
        let off = Vec2::new(self.half * 0.34, -self.half * 0.28);
        let d2 = (centre - off).length() / self.half.max(1.0);
        let core2 = gta_math::smoothstep(gta_math::clamp(1.0 - d2 * 1.7, 0.0, 1.0)) * 0.6;
        let core = core.max(core2);

        let base = if core > 0.72 {
            Zone::Downtown
        } else if core > 0.4 {
            Zone::MidRise
        } else if core > 0.18 {
            Zone::LowRise
        } else {
            Zone::Residential
        };

        // Open space: parks get pushed to the middle ring, plazas near the core.
        let roll = self.rng.f32();
        let threshold = base.openness();
        if roll < threshold * 0.55 {
            return Zone::Park;
        }
        if roll < threshold * 0.8 {
            return if core > 0.45 { Zone::Plaza } else { Zone::Parking };
        }
        // Every so often a surface car park wedges itself between towers.
        let wedge = 0.1 + self.rng.f32() * 0.05;
        if base == Zone::Downtown && self.rng.chance(wedge) {
            return Zone::Parking;
        }
        let _ = index;
        base
    }

    /// Picks a facade style for a building of `floors` storeys.
    pub fn facade_of(&mut self, zone: Zone, floors: usize) -> Facade {
        let r = self.rng.f32();
        match zone {
            Zone::Downtown => {
                if floors > 18 {
                    if r < 0.62 {
                        Facade::CurtainWall
                    } else {
                        Facade::FrameGrid
                    }
                } else if r < 0.3 {
                    Facade::Shopfront
                } else if r < 0.75 {
                    Facade::FrameGrid
                } else {
                    Facade::CurtainWall
                }
            }
            Zone::MidRise => {
                if r < 0.3 {
                    Facade::Shopfront
                } else if r < 0.62 {
                    Facade::Masonry
                } else {
                    Facade::FrameGrid
                }
            }
            Zone::LowRise => {
                if r < 0.45 {
                    Facade::Masonry
                } else if r < 0.8 {
                    Facade::Shopfront
                } else {
                    Facade::FrameGrid
                }
            }
            Zone::Residential => {
                if r < 0.82 {
                    Facade::House
                } else {
                    Facade::Masonry
                }
            }
            Zone::Parking => Facade::CarPark,
            _ => Facade::House,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn planner(seed: u64) -> (f32, Rng) {
        (200.0, Rng::new(seed))
    }

    #[test]
    fn zoning_is_radial() {
        let (h, mut rng) = planner(1);
        let mut p = ZonePlanner::new(h, &mut rng);
        let core = p.zone_of(Vec2::ZERO, 0);
        assert_eq!(core, Zone::Downtown, "city centre must be downtown");
        let edge = p.zone_of(Vec2::new(h * 0.95, h * 0.95), 1);
        assert!(
            matches!(edge, Zone::Residential | Zone::LowRise | Zone::Park | Zone::Parking),
            "far block was {edge:?}"
        );
    }

    #[test]
    fn zoning_is_deterministic() {
        let (h, mut r1) = planner(7);
        let (h2, mut r2) = planner(7);
        let a: Vec<Zone> = (0..64)
            .map(|i| ZonePlanner::new(h, &mut r1).zone_of(Vec2::new(i as f32 * 3.0, i as f32 * -2.0), i))
            .collect();
        let b: Vec<Zone> = (0..64)
            .map(|i| ZonePlanner::new(h2, &mut r2).zone_of(Vec2::new(i as f32 * 3.0, i as f32 * -2.0), i))
            .collect();
        assert_eq!(a, b);
    }

    #[test]
    fn every_zone_reachable() {
        let (h, mut rng) = planner(3);
        let mut p = ZonePlanner::new(h, &mut rng);
        let mut seen = std::collections::HashSet::new();
        for i in 0..4000u32 {
            let a = (i as f32 * 0.31).sin() * h * 0.9;
            let b = (i as f32 * 0.17).cos() * h * 0.9;
            seen.insert(p.zone_of(Vec2::new(a, b), i));
        }
        for z in [Zone::Downtown, Zone::MidRise, Zone::LowRise, Zone::Residential, Zone::Park] {
            assert!(seen.contains(&z), "{z:?} never generated");
        }
    }

    #[test]
    fn facades_match_zone() {
        let (h, mut rng) = planner(4);
        let mut p = ZonePlanner::new(h, &mut rng);
        assert_eq!(p.facade_of(Zone::Parking, 4), Facade::CarPark);
        for _ in 0..200 {
            let f = p.facade_of(Zone::Residential, 2);
            assert!(matches!(f, Facade::House | Facade::Masonry));
        }
    }

    #[test]
    fn envelopes_are_sane() {
        for z in [Zone::Downtown, Zone::MidRise, Zone::LowRise, Zone::Residential] {
            let (lo, hi) = z.floors_envelope();
            assert!(lo >= 1 && lo <= hi, "{z:?} envelope");
            assert!((0.0..=1.0).contains(&z.site_coverage()));
            assert!(z.floor_height() > 2.0 && z.floor_height() < 5.0);
            assert!(z.floor_height() * hi as f32 > 3.0);
        }
        assert!(Zone::Park.is_open());
        assert!(!Zone::MidRise.is_open());
    }
}
