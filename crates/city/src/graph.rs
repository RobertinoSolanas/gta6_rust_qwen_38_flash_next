//! Bounded context: **walkable graph**.
//!
//! Where pedestrians may walk: sidewalk rings, crossings, plazas and park paths,
//! collapsed into an undirected graph. Agents steer along it, so they never clip
//! through a building or idle in a live carriageway.

use gta_geo::GridIndex;
use gta_math::Vec2;

/// Node id.
pub type WalkId = u32;

/// What kind of place a node belongs to — drives behaviour and idle posing.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum WalkKind {
    /// Kerbside sidewalk.
    Sidewalk,
    /// Zebra crossing between two sidewalk nodes.
    Crossing,
    /// Open civic paving.
    Plaza,
    /// Park path.
    Park,
    /// Building frontage / entrance apron.
    Frontage,
}

#[derive(Clone, Copy, Debug)]
struct WalkNode {
    pos: Vec2,
    kind: WalkKind,
}

/// Undirected walk graph with a uniform-grid index over its nodes.
#[derive(Clone, Debug)]
pub struct WalkGraph {
    nodes: Vec<WalkNode>,
    adj: Vec<Vec<u32>>,
    index: GridIndex<u32>,
}

impl Default for WalkGraph {
    fn default() -> Self {
        Self::new(400.0)
    }
}

impl WalkGraph {
    pub fn new(extent: f32) -> Self {
        WalkGraph { nodes: Vec::new(), adj: Vec::new(), index: GridIndex::new(10.0, extent) }
    }

    /// Adds a node, returning its id.
    pub fn add(&mut self, pos: Vec2, kind: WalkKind) -> WalkId {
        let id = self.nodes.len() as WalkId;
        self.nodes.push(WalkNode { pos, kind });
        self.adj.push(Vec::new());
        self.index.insert(pos, 1.2, id);
        id
    }

    #[inline]
    pub fn len(&self) -> usize {
        self.nodes.len()
    }

    #[inline]
    pub fn is_empty(&self) -> bool {
        self.nodes.is_empty()
    }

    #[inline]
    pub fn pos(&self, id: WalkId) -> Vec2 {
        self.nodes[id as usize].pos
    }

    #[inline]
    pub fn kind(&self, id: WalkId) -> WalkKind {
        self.nodes[id as usize].kind
    }

    #[inline]
    pub fn neighbours(&self, id: WalkId) -> &[u32] {
        &self.adj[id as usize]
    }

    #[inline]
    pub fn edge_count(&self) -> usize {
        self.adj.iter().map(|a| a.len()).sum::<usize>() / 2
    }

    /// Connects two nodes. Symmetric, idempotent, no self-links.
    pub fn link(&mut self, a: WalkId, b: WalkId) {
        let (na, nb) = (a as usize, b as usize);
        if na == nb || na >= self.nodes.len() || nb >= self.nodes.len() {
            return;
        }
        if !self.adj[na].contains(&b) {
            self.adj[na].push(b);
        }
        if !self.adj[nb].contains(&a) {
            self.adj[nb].push(a);
        }
    }

    /// Wires every node to all nodes within `radius`. Called once per block to close
    /// sidewalk rings, and again to stitch rings to crossings.
    pub fn connect_within(&mut self, radius: f32) {
        let mut hits: Vec<u32> = Vec::new();
        let n = self.nodes.len() as WalkId;
        // Collect first, mutate after — the grid is borrowed by the query.
        let mut pending: Vec<(WalkId, WalkId)> = Vec::new();
        for i in 0..n {
            let p = self.pos(i);
            hits.clear();
            self.index.query_exact(p, radius, &mut hits);
            for h in hits.drain(..) {
                if h != i {
                    pending.push((i, h));
                }
            }
        }
        for (a, b) in pending {
            self.link(a, b);
        }
    }

    /// Nearest node to `p` within `max_dist`.
    pub fn nearest(&self, p: Vec2, max_dist: f32) -> Option<WalkId> {
        self.index.nearest(p, max_dist).map(|(id, _, _)| id)
    }

    /// Wires `ids` into a closed loop in the given order. Used for sidewalk rings, where
    /// the caller already holds the points in walk order.
    pub fn link_ring(&mut self, ids: &[WalkId]) {
        if ids.len() < 2 {
            return;
        }
        for w in ids.windows(2) {
            self.link(w[0], w[1]);
        }
        let first = ids[0];
        let last = ids[ids.len() - 1];
        self.link(first, last);
    }

    /// Nearest node whose kind is not `skip` — "somewhere to stand, but not on a
    /// crossing".
    pub fn nearest_except(&self, p: Vec2, max_dist: f32, skip: WalkKind) -> Option<WalkId> {
        let mut ids: Vec<u32> = Vec::new();
        self.index.query_ids(p, max_dist, &mut ids);
        let mut best: Option<(WalkId, f32)> = None;
        for id in ids {
            let n = &self.nodes[id as usize];
            if n.kind == skip {
                continue;
            }
            let d = n.pos.distance_sq(p);
            match best {
                Some((_, bd)) if bd <= d => {}
                _ => best = Some((id, d)),
            }
        }
        best.map(|(i, _)| i)
    }

    /// True when `p` is within `pad` of a walk node — the cheap spawner test that
    /// rejects positions in the middle of a carriageway.
    pub fn on_walkable(&self, p: Vec2, pad: f32) -> bool {
        self.nearest(p, pad).is_some()
    }

    /// Node ids within `radius` of `p`.
    pub fn within(&self, p: Vec2, radius: f32, out: &mut Vec<WalkId>) {
        self.index.query_ids(p, radius, out);
    }

    /// Next hop from `from` towards `to` along a shortest path, or `None` when `to` is
    /// unreachable within `max_hops` (or already reached). Breadth-first, so the returned
    /// hop is always one edge closer to the goal than `from`.
    ///
    /// Runs a full BFS over the reachable component — fine at this node count and the
    /// access pattern (one call per agent per decision, not per frame).
    pub fn next_step(&self, from: WalkId, to: WalkId, max_hops: usize) -> Option<WalkId> {
        if from == to || from as usize >= self.nodes.len() || to as usize >= self.nodes.len() {
            return None;
        }
        let n = self.nodes.len();
        // `prev[i]` is the node BFS reached `i` from; `u32::MAX` marks "not yet seen".
        // Seeding the root with itself makes the walk-back below terminate cleanly.
        let mut prev = vec![u32::MAX; n];
        prev[from as usize] = from;
        let mut frontier = vec![from];
        let mut hops = 0usize;
        while prev[to as usize] == u32::MAX && hops < max_hops && !frontier.is_empty() {
            let mut next = Vec::new();
            for &cur in &frontier {
                for &nb in self.neighbours(cur) {
                    if prev[nb as usize] == u32::MAX {
                        prev[nb as usize] = cur;
                        next.push(nb);
                    }
                }
            }
            frontier = next;
            hops += 1;
        }
        if prev[to as usize] == u32::MAX {
            return None;
        }
        // Walk back from the goal to the node whose predecessor is `from`: that is the hop
        // to take now.
        let mut cur = to as usize;
        for _ in 0..n {
            let p = prev[cur] as usize;
            if p == from as usize {
                return Some(cur as WalkId);
            }
            if p == cur || p == u32::MAX as usize {
                return None;
            }
            cur = p;
        }
        None
    }

    /// Breadth-first hop count from `from` to `to`, capped at `max_depth`.
    pub fn bfs_depth(&self, from: WalkId, to: WalkId, max_depth: usize) -> Option<usize> {
        if from == to {
            return Some(0);
        }
        let n = self.nodes.len();
        let mut seen = vec![false; n];
        let mut frontier = vec![from];
        seen[from as usize] = true;
        for depth in 1..=max_depth {
            let mut next = Vec::new();
            for &cur in &frontier {
                for &nb in self.neighbours(cur) {
                    if seen[nb as usize] {
                        continue;
                    }
                    if nb == to {
                        return Some(depth);
                    }
                    seen[nb as usize] = true;
                    next.push(nb);
                }
            }
            if next.is_empty() {
                return None;
            }
            frontier = next;
        }
        None
    }

    /// Nodes unreachable from `from`. Generation asserts this is zero — the single
    /// best smoke test that pedestrians are not trapped inside a block.
    pub fn unreachable_from(&self, from: WalkId) -> usize {
        let n = self.nodes.len();
        if n == 0 {
            return 0;
        }
        let mut seen = vec![false; n];
        let mut stack = vec![from];
        seen[from as usize] = true;
        let mut count = 1;
        while let Some(cur) = stack.pop() {
            for &nb in self.neighbours(cur) {
                if !seen[nb as usize] {
                    seen[nb as usize] = true;
                    count += 1;
                    stack.push(nb);
                }
            }
        }
        n - count
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn add_and_nearest() {
        let mut g = WalkGraph::new(100.0);
        let a = g.add(Vec2::new(0.0, 0.0), WalkKind::Sidewalk);
        let b = g.add(Vec2::new(5.0, 0.0), WalkKind::Sidewalk);
        g.add(Vec2::new(50.0, 50.0), WalkKind::Plaza);
        assert_eq!(g.len(), 3);
        assert_eq!(g.nearest(Vec2::new(0.4, 0.0), 3.0), Some(a));
        assert_eq!(g.nearest(Vec2::new(5.2, 0.2), 1.0), Some(b));
        assert_eq!(g.nearest(Vec2::new(20.0, 50.0), 2.0), None);
    }

    #[test]
    fn links_are_symmetric_and_deduped() {
        let mut g = WalkGraph::new(50.0);
        let a = g.add(Vec2::ZERO, WalkKind::Sidewalk);
        let b = g.add(Vec2::new(3.0, 0.0), WalkKind::Crossing);
        g.link(a, b);
        g.link(b, a);
        g.link(a, a);
        assert_eq!(g.neighbours(a), &[b]);
        assert_eq!(g.neighbours(b), &[a]);
        assert_eq!(g.edge_count(), 1);
    }

    #[test]
    fn connect_within_closes_rings() {
        let mut g = WalkGraph::new(100.0);
        let pts = [
            Vec2::new(0.0, 0.0),
            Vec2::new(6.0, 0.0),
            Vec2::new(6.0, 6.0),
            Vec2::new(0.0, 6.0),
        ];
        let ids: Vec<_> = pts.iter().map(|p| g.add(*p, WalkKind::Sidewalk)).collect();
        g.connect_within(7.0);
        assert_eq!(g.unreachable_from(ids[0]), 0);
        let far = g.add(Vec2::new(40.0, 40.0), WalkKind::Plaza);
        assert!(g.neighbours(far).is_empty());
        assert_eq!(g.unreachable_from(ids[0]), 1);
    }

    #[test]
    fn nearest_except_skips_kind() {
        let mut g = WalkGraph::new(50.0);
        let c = g.add(Vec2::new(0.5, 0.0), WalkKind::Crossing);
        let s = g.add(Vec2::new(2.0, 0.0), WalkKind::Sidewalk);
        assert_eq!(g.nearest(Vec2::ZERO, 5.0), Some(c));
        assert_eq!(g.nearest_except(Vec2::ZERO, 5.0, WalkKind::Crossing), Some(s));
    }

    #[test]
    fn bfs_finds_short_path() {
        let mut g = WalkGraph::new(100.0);
        let ids: Vec<_> = (0..6).map(|i| g.add(Vec2::new(i as f32 * 4.0, 0.0), WalkKind::Sidewalk)).collect();
        for w in ids.windows(2) {
            g.link(w[0], w[1]);
        }
        assert_eq!(g.bfs_depth(ids[0], ids[3], 10), Some(3));
        assert_eq!(g.bfs_depth(ids[0], ids[5], 2), None);
        assert_eq!(g.bfs_depth(ids[2], ids[2], 4), Some(0));
    }

    #[test]
    fn on_walkable_matches_nearest() {
        let mut g = WalkGraph::new(50.0);
        g.add(Vec2::new(1.0, 1.0), WalkKind::Frontage);
        assert!(g.on_walkable(Vec2::new(1.2, 0.9), 1.0));
        assert!(!g.on_walkable(Vec2::new(9.0, 9.0), 1.0));
    }
}
