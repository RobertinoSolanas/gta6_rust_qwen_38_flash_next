//! Bounded context: **spatial index**.
//!
//! A uniform-grid spatial hash. Every runtime query in the sim is local ("what is
//! near me?"), so a uniform grid beats anything fancier for this access pattern and
//! needs no rebuild while the agent populations move around.

use gta_math::Vec2;

/// Uniform spatial hash over a square world region centred on the origin.
#[derive(Clone, Debug)]
pub struct GridIndex<T> {
    cell: f32,
    inv: f32,
    half: f32,
    cols: i32,
    buckets: Vec<Vec<u32>>,
    items: Vec<Item<T>>,
}

#[derive(Clone, Copy, Debug)]
struct Item<T> {
    x: f32,
    z: f32,
    r: f32,
    item: T,
}

impl<T> GridIndex<T> {
    /// `cell` — bucket edge length in metres. `extent` — world width covered
    /// (from `-extent/2` to `+extent/2`). Items outside are clamped into the grid so
    /// nothing is ever silently dropped.
    pub fn new(cell: f32, extent: f32) -> Self {
        let cell = cell.max(0.5);
        let half = extent.max(cell) * 0.5;
        let cols = ((extent / cell).ceil() as i32 + 1).max(1);
        GridIndex {
            cell,
            inv: 1.0 / cell,
            half,
            cols,
            buckets: vec![Vec::new(); (cols as usize) * (cols as usize)],
            items: Vec::new(),
        }
    }

    #[inline]
    fn clamp_i(&self, v: i32) -> i32 {
        v.clamp(0, self.cols - 1)
    }

    #[inline]
    fn cell_of(&self, x: f32, z: f32) -> (i32, i32) {
        let cx = ((x + self.half) * self.inv).floor() as i32;
        let cz = ((z + self.half) * self.inv).floor() as i32;
        (cx.clamp(0, self.cols - 1), cz.clamp(0, self.cols - 1))
    }

    #[inline]
    fn bucket_index(&self, cx: i32, cz: i32) -> usize {
        (cz * self.cols + cx) as usize
    }

    /// Stores `item` at `pos` with a query `radius`. Returns its id.
    pub fn insert(&mut self, pos: Vec2, radius: f32, item: T) -> u32 {
        let id = self.items.len() as u32;
        self.items.push(Item { x: pos.x, z: pos.y, r: radius.max(0.0), item });
        let (cx0, cz0) = self.cell_of(pos.x - radius, pos.y - radius);
        let (cx1, cz1) = self.cell_of(pos.x + radius, pos.y + radius);
        let cols = self.cols;
        for cz in cz0..=cz1 {
            for cx in cx0..=cx1 {
                let b = (cz * cols + cx) as usize;
                self.buckets[b].push(id);
            }
        }
        id
    }

    #[inline]
    pub fn len(&self) -> usize {
        self.items.len()
    }

    #[inline]
    pub fn is_empty(&self) -> bool {
        self.items.is_empty()
    }

    #[inline]
    pub fn get(&self, id: u32) -> Option<&T> {
        self.items.get(id as usize).map(|i| &i.item)
    }

    /// Ids whose *bucket* overlaps the query circle. Superset of the true answer —
    /// callers that need exact distances filter afterwards.
    pub fn query_ids(&self, center: Vec2, radius: f32, out: &mut Vec<u32>) {
        let (cx0, cz0) = self.cell_of(center.x - radius, center.y - radius);
        let (cx1, cz1) = self.cell_of(center.x + radius, center.y + radius);
        for cz in cz0..=cz1 {
            for cx in cx0..=cx1 {
                out.extend_from_slice(&self.buckets[self.bucket_index(cx, cz)]);
            }
        }
    }

    /// Exact circle query: only items whose stored radius overlaps `radius`.
    pub fn query_exact(&self, center: Vec2, radius: f32, out: &mut Vec<u32>) {
        out.clear();
        self.query_ids(center, radius, out);
        out.retain(|&id| {
            let it = &self.items[id as usize];
            let dx = it.x - center.x;
            let dz = it.z - center.y;
            let rr = radius + it.r;
            dx * dx + dz * dz <= rr * rr
        });
    }

    /// Borrowed items inside the circle (allocates a Vec of references).
    pub fn query_items(&self, center: Vec2, radius: f32) -> Vec<&T> {
        let mut ids = Vec::new();
        self.query_exact(center, radius, &mut ids);
        ids.iter().filter_map(|&id| self.get(id)).collect()
    }

    /// Nearest stored item to `center` within `max_dist`, if any.
    pub fn nearest(&self, center: Vec2, max_dist: f32) -> Option<(u32, f32, &T)> {
        let mut ids = Vec::new();
        self.query_ids(center, max_dist, &mut ids);
        let mut best: Option<(u32, f32, &T)> = None;
        for id in ids {
            let it = &self.items[id as usize];
            let d = ((it.x - center.x).powi(2) + (it.z - center.y).powi(2)).sqrt();
            if d <= max_dist && best.map_or(true, |b: (u32, f32, &T)| d < b.1) {
                best = Some((id as u32, d, &it.item));
            }
        }
        best
    }

    /// Mean bucket occupancy — used by tests to prove the distribution is sane.
    pub fn max_bucket(&self) -> usize {
        self.buckets.iter().map(|b| b.len()).max().unwrap_or(0)
    }

    #[inline]
    pub fn cell_size(&self) -> f32 {
        self.cell
    }
}

impl<T> Default for GridIndex<T> {
    fn default() -> Self {
        Self::new(16.0, 1024.0)
    }
}
