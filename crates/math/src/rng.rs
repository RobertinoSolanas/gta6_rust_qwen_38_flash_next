//! Bounded context: **deterministic randomness**.
//!
//! A tiny splitmix64-based PRNG. The whole city is generated from one seed, so *every*
//! random decision — lot splits, facade palettes, traffic, pedestrian wardrobes — has
//! to be reproducible from that seed. Floating-point output is produced by consuming
//! the top 32 bits of a 64-bit stream, which keeps output identical on every platform
//! that has IEEE-754 `f32` (i.e. wasm32 and x86/AArch64 alike).

/// Splitmix64 generator: fast, statistically decent, 64 bits of state.
#[derive(Clone, Debug)]
pub struct Rng {
    state: u64,
}

impl Rng {
    #[inline]
    pub fn new(seed: u64) -> Self {
        Rng { state: seed ^ 0x9E37_79B9_7F4A_7C15 }
    }

    #[inline]
    fn next_u64(&mut self) -> u64 {
        self.state = self.state.wrapping_add(0x9E37_79B9_7F4A_7C15);
        let mut z = self.state;
        z = (z ^ (z >> 30)).wrapping_mul(0xBF58_476D_1CE4_E5B9);
        z = (z ^ (z >> 27)).wrapping_mul(0x94D0_49BB_1331_11EB);
        z ^ (z >> 31)
    }

    /// Uniform in `[0, 1)`.
    #[inline]
    pub fn f32(&mut self) -> f32 {
        // 24 bits of mantissa is exactly what f32 can represent.
        ((self.next_u64() >> 40) as f32) * (1.0 / 16_777_216.0)
    }

    #[inline]
    pub fn range(&mut self, lo: f32, hi: f32) -> f32 {
        lo + (hi - lo) * self.f32()
    }

    #[inline]
    pub fn signed(&mut self) -> f32 {
        self.f32() * 2.0 - 1.0
    }

    #[inline]
    pub fn below(&mut self, n: u32) -> u32 {
        if n == 0 {
            0
        } else {
            (self.next_u64() % n as u64) as u32
        }
    }

    #[inline]
    pub fn index(&mut self, n: usize) -> usize {
        self.below(n as u32) as usize
    }

    #[inline]
    pub fn chance(&mut self, p: f32) -> bool {
        self.f32() < p
    }

    /// Integer in `lo..=hi`.
    #[inline]
    pub fn int_range(&mut self, lo: i32, hi: i32) -> i32 {
        if hi <= lo {
            lo
        } else {
            lo + self.below((hi - lo + 1) as u32) as i32
        }
    }

    /// Picks one element, if the slice is non-empty.
    #[inline]
    pub fn pick<'a, T>(&mut self, xs: &'a [T]) -> Option<&'a T> {
        if xs.is_empty() {
            None
        } else {
            xs.get(self.index(xs.len()))
        }
    }

    /// Fisher–Yates shuffle in place.
    pub fn shuffle<T>(&mut self, xs: &mut [T]) {
        for i in (1..xs.len()).rev() {
            let j = self.index(i + 1);
            xs.swap(i, j);
        }
    }

    /// Derives an independent sub-stream (so adding a new random consumer elsewhere
    /// cannot perturb an existing subsystem's output).
    #[inline]
    pub fn sub(&mut self, salt: u64) -> Rng {
        Rng::new(self.state.wrapping_mul(0x2545_F491_4F6C_DD1D) ^ salt)
    }
}

impl Default for Rng {
    fn default() -> Self {
        Rng::new(0x1234_ABCD)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn same_seed_same_stream() {
        let mut a = Rng::new(7);
        let mut b = Rng::new(7);
        for _ in 0..64 {
            assert_eq!(a.f32(), b.f32());
        }
    }

    #[test]
    fn different_seeds_differ() {
        let a: f32 = Rng::new(1).f32();
        let b: f32 = Rng::new(2).f32();
        assert!((a - b).abs() > 1e-4);
    }

    #[test]
    fn output_in_range() {
        let mut r = Rng::new(99);
        let mut sum = 0.0;
        for _ in 0..100_000 {
            let v = r.f32();
            assert!((0.0..1.0).contains(&v), "{v} out of range");
            sum += v;
        }
        let mean = sum / 100_000.0;
        assert!((mean - 0.5).abs() < 0.01, "mean {mean} far from 0.5");
    }

    #[test]
    fn range_and_int_range_respect_bounds() {
        let mut r = Rng::new(5);
        for _ in 0..1000 {
            let v = r.range(-2.0, 3.0);
            assert!((-2.0..3.0).contains(&v));
            let i = r.int_range(3, 6);
            assert!((3..=6).contains(&i));
        }
    }

    #[test]
    fn shuffle_is_a_permutation() {
        let mut v: Vec<i32> = (0..64).collect();
        let mut r = Rng::new(3);
        r.shuffle(&mut v);
        v.sort_unstable();
        assert_eq!(v, (0..64).collect::<Vec<_>>());
    }

    #[test]
    fn sub_streams_are_independent() {
        let mut a = Rng::new(11);
        let mut b = Rng::new(11);
        let _ = a.f32();
        let mut s1 = a.sub(1);
        let _ = b.f32();
        let mut s2 = b.sub(2);
        assert_ne!(s1.f32(), s2.f32());
    }

    #[test]
    fn pick_returns_member_or_none() {
        let mut r = Rng::new(5);
        let xs = [1u8, 2, 3];
        for _ in 0..20 {
            assert!(xs.contains(&r.pick(&xs).unwrap()));
        }
        assert!(r.pick::<u8>(&[]).is_none());
    }
}
