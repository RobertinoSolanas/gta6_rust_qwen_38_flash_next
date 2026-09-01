//! Bounded context: **procedural noise**.
//!
//! Deterministic, allocation-free hash noise: integer hashes, value noise, gradient
//! (Perlin) noise, fBm and ridged variants. Determinism matters — the whole city is
//! generated from a single seed and must be reproducible (tests rely on it).

use crate::{lerpc, Vec2, Vec3};

#[inline(always)]
fn mut32(mut x: u32) -> u32 {
    x ^= x >> 16;
    x = x.wrapping_mul(0x7feb_352d);
    x ^= x >> 15;
    x = x.wrapping_mul(0x846c_a68b);
    x ^= x >> 16;
    x
}

/// Hash of a single integer.
#[inline]
pub fn hash11(x: i32) -> u32 {
    mut32(x as u32)
}

/// splitmix64 finaliser — a strong 64-bit avalanche.
#[inline]
fn mix64(mut z: u64) -> u64 {
    z = (z ^ (z >> 30)).wrapping_mul(0xBF58_476D_1CE4_E5B9);
    z = (z ^ (z >> 27)).wrapping_mul(0x94D0_49BB_1331_11EB);
    z ^ (z >> 31)
}

/// Packs (x, y) into a 64-bit key and runs two rounds of splitmix64 so the result
/// has a full 64-bit avalanche.
#[inline]
pub fn hash2_64(x: i32, y: i32) -> u64 {
    let key = (x as u32 as u64) | ((y as u32 as u64) << 32);
    mix64(mix64(key).wrapping_mul(0x9E37_79B9_7F4A_7C15))
}

/// Hash of a 2D lattice point (low 32 bits of [`hash2_64`]).
#[inline]
pub fn hash2(x: i32, y: i32) -> u32 {
    hash2_64(x, y) as u32
}

/// Hash of a 3D lattice point.
#[inline]
pub fn hash3(x: i32, y: i32, z: i32) -> u32 {
    hash3_64(x, y, z) as u32
}

/// 64-bit 3D lattice hash.
#[inline]
pub fn hash3_64(x: i32, y: i32, z: i32) -> u64 {
    mix64(hash2_64(x, y) ^ ((z as u64).wrapping_mul(0xD6E8_F962_FD41_7C0D) | 1))
}

/// Hash -> float in `0..1`.
#[inline]
pub fn unit(h: u32) -> f32 {
    (h >> 8) as f32 * (1.0 / (1u32 << 24) as f32)
}

/// 2D lattice random in `0..1`.
#[inline]
pub fn lattice2(x: i32, y: i32) -> f32 {
    unit(hash2(x, y))
}

/// 3D lattice random in `0..1`.
#[inline]
pub fn lattice3(x: i32, y: i32, z: i32) -> f32 {
    unit(hash3(x, y, z))
}

/// Smoothstep curve.
#[inline]
pub fn smoothstep(t: f32) -> f32 {
    t * t * (3.0 - 2.0 * t)
}

/// Perlin's quintic fade curve.
#[inline]
pub fn fade(t: f32) -> f32 {
    t * t * t * (t * (t * 6.0 - 15.0) + 10.0)
}

#[inline]
fn fi(v: f32) -> i32 {
    v.floor() as i32
}

/// 2D value noise, output `0..1`.
pub fn value2(p: Vec2) -> f32 {
    let i = p.floor();
    let f = p - i;
    let (x0, y0) = (fi(i.x), fi(i.y));
    let u = smoothstep(f.x);
    let v = smoothstep(f.y);
    let a = lattice2(x0, y0);
    let b = lattice2(x0 + 1, y0);
    let c = lattice2(x0, y0 + 1);
    let d = lattice2(x0 + 1, y0 + 1);
    lerpc(lerpc(a, b, u), lerpc(c, d, u), v)
}

/// 3D value noise, output `0..1`.
pub fn value3(p: Vec3) -> f32 {
    let i = p.floor();
    let f = p - i;
    let (x0, y0, z0) = (fi(i.x), fi(i.y), fi(i.z));
    let u = fade(f.x);
    let v = fade(f.y);
    let w = fade(f.z);
    let a = lerpc(lattice3(x0, y0, z0), lattice3(x0 + 1, y0, z0), u);
    let b = lerpc(lattice3(x0, y0 + 1, z0), lattice3(x0 + 1, y0 + 1, z0), u);
    let c = lerpc(lattice3(x0, y0, z0 + 1), lattice3(x0 + 1, y0, z0 + 1), u);
    let d = lerpc(lattice3(x0, y0 + 1, z0 + 1), lattice3(x0 + 1, y0 + 1, z0 + 1), u);
    lerpc(lerpc(a, b, v), lerpc(c, d, v), w)
}

/// 16 evenly spaced gradient directions selected by the hash.
#[inline]
fn grad2(h: u32, fx: f32, fy: f32) -> f32 {
    let a = (h >> 12) as f32 * (crate::TAU / 16.0);
    let (s, c) = a.sin_cos();
    fx * c + fy * s
}

/// Gustavson-style gradient selection (12 cube-edge directions).
#[inline]
fn grad3(h: u32, fx: f32, fy: f32, fz: f32) -> f32 {
    let h = h >> 10;
    let u = if h < 8 { fx } else { fy };
    let v = if h < 4 { fy } else if h == 12 || h == 14 { fx } else { fz };
    (if h & 1 == 0 { u } else { -u }) + (if h & 2 == 0 { v } else { -v })
}

/// 2D Perlin gradient noise, roughly `-1..1` (exactly 0 on lattice points).
pub fn perlin2(p: Vec2) -> f32 {
    let i = p.floor();
    let f = p - i;
    let (x0, y0) = (fi(i.x), fi(i.y));
    let u = fade(f.x);
    let v = fade(f.y);
    let a = grad2(hash2(x0, y0), f.x, f.y);
    let b = grad2(hash2(x0 + 1, y0), f.x - 1.0, f.y);
    let c = grad2(hash2(x0, y0 + 1), f.x, f.y - 1.0);
    let d = grad2(hash2(x0 + 1, y0 + 1), f.x - 1.0, f.y - 1.0);
    lerpc(lerpc(a, b, u), lerpc(c, d, u), v)
}

/// 3D Perlin gradient noise, roughly `-1..1`.
pub fn perlin3(p: Vec3) -> f32 {
    let i = p.floor();
    let f = p - i;
    let (x0, y0, z0) = (fi(i.x), fi(i.y), fi(i.z));
    let u = fade(f.x);
    let v = fade(f.y);
    let w = fade(f.z);
    let a = lerpc(
        grad3(hash3(x0, y0, z0), f.x, f.y, f.z),
        grad3(hash3(x0 + 1, y0, z0), f.x - 1.0, f.y, f.z),
        u,
    );
    let b = lerpc(
        grad3(hash3(x0, y0 + 1, z0), f.x, f.y - 1.0, f.z),
        grad3(hash3(x0 + 1, y0 + 1, z0), f.x - 1.0, f.y - 1.0, f.z),
        u,
    );
    let c = lerpc(
        grad3(hash3(x0, y0, z0 + 1), f.x, f.y, f.z - 1.0),
        grad3(hash3(x0 + 1, y0, z0 + 1), f.x - 1.0, f.y, f.z - 1.0),
        u,
    );
    let d = lerpc(
        grad3(hash3(x0, y0 + 1, z0 + 1), f.x, f.y - 1.0, f.z - 1.0),
        grad3(hash3(x0 + 1, y0 + 1, z0 + 1), f.x - 1.0, f.y - 1.0, f.z - 1.0),
        u,
    );
    lerpc(lerpc(a, b, v), lerpc(c, d, v), w)
}

/// Fractal Brownian motion over 2D value noise, normalised to `0..1`.
pub fn fbm2(x: f32, y: f32, octaves: usize) -> f32 {
    let mut sum = 0.0f32;
    let mut amp = 0.5f32;
    let mut freq = 1.0f32;
    let mut norm = 0.0f32;
    for _ in 0..octaves.max(1) {
        sum += amp * value2(Vec2::new(x * freq, y * freq));
        norm += amp;
        amp *= 0.5;
        freq *= 2.02;
    }
    sum / norm
}

/// Fractal Brownian motion over 3D value noise, normalised to `0..1`.
pub fn fbm3(p: Vec3, octaves: usize) -> f32 {
    let mut sum = 0.0f32;
    let mut amp = 0.5f32;
    let mut freq = 1.0f32;
    let mut norm = 0.0f32;
    for _ in 0..octaves.max(1) {
        sum += amp * value3(p * freq);
        norm += amp;
        amp *= 0.5;
        freq *= 2.02;
    }
    sum / norm
}

/// Ridged multifractal (ridgelines, facade grime), output `0..1`.
pub fn ridged2(x: f32, y: f32, octaves: usize) -> f32 {
    let mut sum = 0.0f32;
    let mut amp = 0.5f32;
    let mut freq = 1.0f32;
    let mut norm = 0.0f32;
    for _ in 0..octaves.max(1) {
        let n = value2(Vec2::new(x * freq, y * freq));
        let r = 1.0 - (n * 2.0 - 1.0).abs();
        sum += amp * r * r;
        norm += amp;
        amp *= 0.5;
        freq *= 2.07;
    }
    sum / norm
}

/// Integral coordinate offset for a seed/lane pair — decorrelates seeds while
/// keeping the lattice aligned.
#[inline]
fn seed_off(seed: u32, lane: u32) -> f32 {
    mut32(seed ^ lane.wrapping_mul(0x9e37_79b9)) as f32 % 4096.0
}

/// A seeded 2D noise field. Cheap to clone; carries only the seed.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct Noise2 {
    pub seed: u32,
}

impl Noise2 {
    #[inline]
    pub fn new(seed: u32) -> Self {
        Self { seed }
    }
    /// Value noise in `0..1`.
    #[inline]
    pub fn value(&self, x: f32, y: f32) -> f32 {
        value2(Vec2::new(x + seed_off(self.seed, 1), y + seed_off(self.seed, 2)))
    }
    /// Perlin noise, roughly `-1..1`.
    #[inline]
    pub fn perlin(&self, x: f32, y: f32) -> f32 {
        perlin2(Vec2::new(x + seed_off(self.seed, 3), y + seed_off(self.seed, 4)))
    }
    /// fBm in `0..1`.
    #[inline]
    pub fn fbm(&self, x: f32, y: f32, octaves: usize) -> f32 {
        fbm2(x + seed_off(self.seed, 5), y + seed_off(self.seed, 6), octaves)
    }
    /// Ridged fBm in `0..1`.
    #[inline]
    pub fn ridged(&self, x: f32, y: f32, octaves: usize) -> f32 {
        ridged2(x + seed_off(self.seed, 7), y + seed_off(self.seed, 8), octaves)
    }
    /// Deterministic random in `0..1` keyed by an integer id.
    #[inline]
    pub fn index(&self, i: u32) -> f32 {
        unit(hash11((i ^ self.seed) as i32))
    }
}

/// A seeded 3D noise field.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct Noise3 {
    pub seed: u32,
}

impl Noise3 {
    #[inline]
    pub fn new(seed: u32) -> Self {
        Self { seed }
    }
    #[inline]
    pub fn value(&self, p: Vec3) -> f32 {
        value3(p + Vec3::new(seed_off(self.seed, 1), seed_off(self.seed, 2), seed_off(self.seed, 3)))
    }
    #[inline]
    pub fn perlin(&self, p: Vec3) -> f32 {
        perlin3(p + Vec3::new(seed_off(self.seed, 4), seed_off(self.seed, 5), seed_off(self.seed, 6)))
    }
    #[inline]
    pub fn fbm(&self, p: Vec3, octaves: usize) -> f32 {
        fbm3(p + Vec3::new(seed_off(self.seed, 7), seed_off(self.seed, 8), seed_off(self.seed, 9)), octaves)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn lattice_hashes_are_injective_over_a_city_block() {
        // 64-bit hash: no collisions at all over a big lattice patch.
        let mut seen = std::collections::HashSet::new();
        for i in -300..400i32 {
            for j in -300..400i32 {
                assert!(seen.insert(hash2_64(i, j)), "hash2_64 collision at {i},{j}");
            }
        }
        let mut seen3 = std::collections::HashSet::new();
        for i in 0..120i32 {
            for j in 0..120i32 {
                for k in 0..120i32 {
                    assert!(seen3.insert(hash3_64(i, j, k)), "hash3_64 collision at {i},{j},{k}");
                }
            }
        }
    }

    #[test]
    fn hash32_is_well_distributed() {
        // Truncating to 32 bits must not introduce structure: birthday collisions for
        // a random function are ~n^2/2^33, so we only assert the rate stays sane and
        // that the float output is uniform-ish.
        let mut seen = std::collections::HashSet::new();
        let mut collisions = 0usize;
        let mut sum = 0.0f64;
        let mut buckets = [0u32; 16];
        for i in 0..400i32 {
            for j in 0..400i32 {
                let h = hash2(i, j);
                if !seen.insert(h) {
                    collisions += 1;
                }
                let v = unit(h);
                sum += v as f64;
                buckets[((v * 16.0) as usize).min(15)] += 1;
            }
        }
        let total = seen.len() + collisions;
        let expected = (total * total) as f64 / (2.0 * u32::MAX as f64);
        assert!((collisions as f64) < expected * 4.0 + 1.0, "too many collisions {collisions} vs {expected}");
        let mean = sum / total as f64;
        assert!((mean - 0.5).abs() < 0.02, "mean {mean}");
        let expected_per_bucket = total as f64 / 16.0;
        for b in buckets {
            assert!((b as f64 - expected_per_bucket).abs() < expected_per_bucket * 0.25, "bucket {b}");
        }
    }

    #[test]
    fn hash_is_deterministic() {
        assert_eq!(hash2(3, 7), hash2(3, 7));
        assert_ne!(hash2(3, 7), hash2(7, 3));
        assert_ne!(hash3(1, 2, 3), hash3(1, 2, 4));
    }

    #[test]
    fn value_noise_range_and_continuity() {
        for i in 0..400 {
            let x = i as f32 * 0.113;
            let y = i as f32 * 0.317;
            let a = value2(Vec2::new(x, y));
            assert!((0.0..=1.0).contains(&a), "value2 range {a}");
            let b = value2(Vec2::new(x + 1e-4, y));
            assert!((a - b).abs() < 1e-3, "value2 continuity {a} {b}");
            let p = Vec3::new(x, y, x * 0.5);
            let v = value3(p);
            assert!((0.0..=1.0).contains(&v));
            assert!((v - value3(p + Vec3::new(1e-4, 0.0, 0.0))).abs() < 1e-3, "value3 continuity");
        }
    }

    #[test]
    fn perlin_zero_on_lattice_and_bounded() {
        for i in -4..4i32 {
            for j in -4..4i32 {
                assert!(perlin2(Vec2::new(i as f32, j as f32)).abs() < 1e-6);
            }
        }
        let mut mn = f32::MAX;
        let mut mx = f32::MIN;
        for i in 0..2000 {
            let v = perlin2(Vec2::new(i as f32 * 0.0731, i as f32 * 0.917));
            assert!(v.abs() <= 1.2, "perlin2 range {v}");
            mn = mn.min(v);
            mx = mx.max(v);
        }
        assert!(mx - mn > 0.3, "perlin2 flat: {mn}..{mx}");
    }

    #[test]
    fn fbm_and_ridged_normalised() {
        for i in 0..1000 {
            let x = i as f32 * 0.113;
            let y = i as f32 * 0.419;
            let v = fbm2(x, y, 5);
            assert!((0.0..=1.0).contains(&v), "fbm2 {v}");
            let r = ridged2(x, y, 4);
            assert!((0.0..=1.0).contains(&r), "ridged {r}");
            let w = fbm3(Vec3::new(x, y, x * 0.5), 4);
            assert!((0.0..=1.0).contains(&w), "fbm3 {w}");
        }
    }

    #[test]
    fn seeds_decorrelate_and_replay() {
        let a = Noise2::new(1);
        let b = Noise2::new(2);
        let mut diff = 0;
        for i in 0..60 {
            let x = i as f32 * 0.4;
            if (a.value(x, 1.5) - b.value(x, 1.5)).abs() > 1e-5 {
                diff += 1;
            }
        }
        assert!(diff > 45, "seeds should decorrelate, got {diff}");
        assert_eq!(Noise2::new(7).value(3.5, -2.5), Noise2::new(7).value(3.5, -2.5));
        assert_eq!(
            Noise3::new(9).perlin(Vec3::new(1.5, 2.5, 0.5)),
            Noise3::new(9).perlin(Vec3::new(1.5, 2.5, 0.5))
        );
        assert_ne!(
            Noise3::new(9).perlin(Vec3::new(1.5, 2.5, 0.5)),
            Noise3::new(10).perlin(Vec3::new(1.5, 2.5, 0.5))
        );
    }
}
