# TODO

Trackable work list for `gta6_rust_qwen_38_flash_next`. Generated from a full repo audit.

## How to use this file

Every task is a markdown checkbox with a stable ID, so a later session can resume cold:

- `- [ ]` = open · `- [x]` = done · `[-]` = won't do / superseded
- Never delete an item — mark it done and leave the line, history lives in git.
- On completion: tick the box, fill `Done:` with a date + commit SHA, and paste the output of
  its **Verify** command in the `Evidence:` block.
- Work top to bottom within a priority block. `Depends:` must be closed first.

`Status:` lines are free-form notes for whoever picks the item up next.

```bash
# Session-start sanity check — paste this first, before anything else
cargo test --workspace 2>&1 | grep -E 'test result'
cargo clippy --workspace --all-targets 2>&1 | grep -c '^warning'
```

**Baseline at time of writing:** 97/97 tests pass · clippy ~10 distinct warnings ·
`cargo fmt --check` reports **115 diffs across 16 files** · git has 1 commit containing only
`.gitignore`.

---

## P0 — Broken / misleading right now

### [ ] TODO-01 — `gta-agents` never compiles its own code
**Severity:** 🔴 blocking · **Effort:** ~15 min · **Depends:** —

`crates/agents/src/lib.rs` is exactly one line, `//! placeholder`, with no `mod` declarations.
`traffic.rs` (435 L) and `crowd.rs` (323 L) are therefore **not in the build graph**: never
type-checked, never tested, never dead-code-warned. `cargo build -p gta-agents` succeeds while
testing nothing, and the crate reports `running 0 tests`.

- [ ] Add `pub mod crowd;` and `pub mod traffic;` to `crates/agents/src/lib.rs`
- [ ] Add re-exports in the style of the other crates (`Traffic`, `Car`, `CarPose`, `Turn`,
      `Crowd`, `Ped`, `PedPose`, `yaw_of`, `dir_from_yaw`, `make_car`)
- [ ] Replace the `//! placeholder` doc header with a real crate-level doc in the house style
- [ ] Fix whatever fails to compile (these 758 lines have never been through rustc)
- [ ] Add `gta-agents` to the workspace description list if any crate description is missing

**Verify:** `cargo test -p gta-agents` reports > 0 tests, and
`cargo build -p gta-agents 2>&1 | grep -c error` is `0`.
**Evidence:**
```
```
**Done:**
**Status:** Both files import `gta_geo::GridIndex` — the only real consumer of the spatial
hash in the repo. Enabling this module may surface unused-import and borrow errors.

---

### [ ] TODO-02 — Vertex stride documented three different ways
**Severity:** 🔴 high · **Effort:** 5 min · **Depends:** —

The single most load-bearing constant in the rendering contract is documented three ways and
only one is right.

| location | claim |
|---|---|
| `crates/geo/src/lib.rs:5` | "a packed **32-byte** vertex" |
| `crates/geo/src/tri.rs:7` | "48-byte stride" |
| `crates/geo/src/tri.rs` (`VERT_STRIDE`, compile-time assert) | **64** ✅ |

- [ ] Correct `geo/src/lib.rs:5` to 64-byte
- [ ] Correct `crates/geo/src/tri.rs:7` header
- [ ] Grep for any other stale size claim: `grep -rn "32-byte\|48-byte" crates/`

**Verify:** `grep -rn "32-byte\|48-byte" crates/` returns nothing;
`cargo test -p gta-geo` still green (the `size_of` assert is the real source of truth).
**Evidence:**
```
```
**Done:**
**Status:**

---

### [ ] TODO-03 — Commit the actual source tree
**Severity:** 🔴 high · **Effort:** 10 min · **Depends:** —

`git log` has a single commit (`8160672 Initial commit`) that contains **only `.gitignore`**.
6.4k lines of Rust, `Cargo.toml` and `Cargo.lock` are all untracked:

```
 M .gitignore
?? Cargo.lock
?? Cargo.toml
?? crates/
```

The working-copy `.gitignore` was also **rewritten** relative to HEAD: the committed Rust
template (Cargo, `**/*.rs.bk`, `*.pdb`, `mutants.out*`, RustRover) was replaced by a shorter,
web/wasm-flavoured list (`target/ dist/ pkg/ node_modules/ *.log .DS_Store .webassets-cache/`).

- [ ] Decide `.gitignore` policy for `Cargo.lock` (currently **untracked**; for an
      application-bound workspace it should be committed)
- [ ] Decide whether the `.gitignore` rewrite was intentional; if not, restore the dropped Rust
      entries (`**/*.rs.bk`, `*.pdb`, `mutants.out*/`, RustRover notes)
- [ ] `git add` the workspace and commit in reviewable chunks (per-crate is natural)
- [ ] Add a `.gitignore` entry for `.pi/` if it should stay local

**Verify:** `git status --short` is empty; `git ls-files crates | wc -l` ≥ 23.
**Evidence:**
```
```
**Done:**
**Status:** `git diff .gitignore` shows the full template → short-list replacement; confirm
intent with the author before restoring anything.

---

## P1 — Hygiene / cheap correctness

### [ ] TODO-04 — Run `cargo fmt` (115 diffs, 16 files)
**Severity:** 🟡 medium · **Effort:** 5 min · **Depends:** TODO-01, TODO-02 (format last, so
the format commit doesn't collide with content edits)

`cargo fmt --check` reports **115 hunks across all 16 source files** — the project has never
been formatted. Affects every crate including `math`, `geo`, `city`, `sky`.

- [ ] Land all source edits first
- [ ] `cargo fmt --all`
- [ ] Commit the format pass **on its own**, so blame can skip it
- [ ] Add a CI step to prevent regression (see TODO-08)

**Verify:** `cargo fmt --check` exits 0 with no output.
**Evidence:**
```
```
**Done:**
**Status:** Consider whether a `rustfmt.toml` (e.g. `max_width = 110`, matching the existing
hand-wrapped style) is wanted before running, otherwise the diff is larger than necessary.

### [ ] TODO-05 — Clear clippy warnings
**Severity:** 🟡 medium · **Effort:** 30–60 min · **Depends:** TODO-01 (module must be compiled
to lint `agents`)

`cargo clippy --workspace --all-targets` reports ~10 distinct warnings:

- [ ] `method mul can be confused for the standard trait method` (`Mat4::mul` — deliberate,
      decide: add an `#[allow]` with a comment, or implement `core::ops::Mul`)
- [ ] `this function has too many arguments (8/7)` ×2
- [ ] `this map_or can be simplified` ×2
- [ ] `manual implementation of .is_multiple_of()` ×2
- [ ] `manual RangeInclusive::contains implementation`
- [ ] `manual absolute difference pattern without using abs_diff`
- [ ] `the loop variable i is used to index <ls|lines|hits>` ×3
- [ ] `casting to the same type is unnecessary (u32 -> u32)`
- [ ] `this expression creates a reference which is immediately dereferenced`
- [ ] `method clamp_i is never used` → see TODO-06
- [ ] Re-run with `agents` enabled and clean up whatever it newly surfaces

**Verify:** `cargo clippy --workspace --all-targets -- -D warnings` succeeds.
**Evidence:**
```
```
**Done:**
**Status:**

### [ ] TODO-06 — `GridIndex::clamp_i` is dead
**Severity:** 🟢 low · **Effort:** 5 min · **Depends:** TODO-01

The only dead-code warning in the workspace, and its only plausible consumer (`agents`) isn't
compiled. Resolve after TODO-01: either make it `pub` and use it, or delete it.

- [ ] Re-check usage after enabling `gta-agents`
- [ ] Make `pub` or delete

**Verify:** `cargo build --workspace 2>&1 | grep dead_code` is empty.
**Done:**
**Status:**

### [ ] TODO-07 — Repo metadata missing
**Severity:** 🟢 low · **Effort:** 10 min · **Depends:** —

- [ ] `LICENSE` file — every `Cargo.toml` declares `license = "MIT"` but there is no `LICENSE`
- [ ] `rust-toolchain.toml` — no toolchain pin despite `edition = "2021"`
- [ ] `rustfmt.toml` (decide before TODO-04)
- [ ] Fill in missing `description` fields: `gta-avatar`, `gta-scene`, `gta-render`, `gta-app`
      are all `null` in `cargo metadata`
- [ ] Remove or populate the empty `.pi/` directory

**Verify:** `cargo metadata --no-deps --format-version 1 | jq '.packages[] | {name, description}'`
has no `null` descriptions.
**Done:**
**Status:**

---

## P2 — Architecture / next real features

### [ ] TODO-08 — CI pipeline
**Severity:** 🟡 medium · **Depends:** TODO-04, TODO-05 (get green before enforcing)

No CI config of any kind exists. The 97 invariant tests are the project's actual
specification and nothing currently runs them.

- [ ] Add workflow: `cargo fmt --check`, `cargo clippy --workspace --all-targets -- -D warnings`,
      `cargo test --workspace`, `cargo build --workspace --release`
- [ ] Add a wasm32-unknown-unknown build job — determinism docs explicitly promise identical
      output on wasm32, and nothing verifies it compiles there
- [ ] Optional: seed-stability test as a guard so refactors can't silently change generation

**Verify:** badge/job runs on every push and is green.
**Done:**
**Status:**

### [ ] TODO-09 — `gta-scene`: turn `City` into triangles
**Severity:** 🔴 the missing link · **Effort:** large · **Depends:** TODO-01

`gta-scene` is `//! placeholder`. It is the crate the whole architecture is aiming at:
everything upstream already documents `scene` as "the thing that turns `City` into triangles".
Without it there is no path from the sim to pixels.

- [ ] Design: baked-per-block meshes vs. instanced props (the API in `geo` supports both —
      `MeshBuilder::merge_offset` / `merge_mat` for baking, `Mat4::compose` for instances)
- [ ] Ground plane + carriageway + sidewalk pads from `Block::site()` / `walk_ring()`
- [ ] Buildings: `extrude_polygon` / `chamfered_box`, `RoofKind` → `roof_rise()`,
      `Facade` → `Paint::facade(kind, bay, storey, lit)`
- [ ] Props: trees (`blob`/`tapered_cylinder`), lamps (`beam` + emissive), parking stalls,
      pond, zebra crossings
- [ ] Emit `Vec<Vertex>` + index buffer per block; per-block `Aabb` for culling
- [ ] Tests: triangle counts > 0, every building produces a closed solid, no degenerate
      triangles, emissive only on `night`-relevant paint

**Verify:** `cargo test -p gta-scene` with real assertions; a headless dump of one block to a
readable format for eyeballing.
**Done:**
**Status:**

### [ ] TODO-10 — `gta-render`: consume `Vertex` and `Sky`
**Severity:** 🔴 the missing link · **Depends:** TODO-09

`gta-render` is `//! placeholder` and has **no dependencies declared at all**. `tri.rs` and
`sky` both already reference it in doc comments as the consumer.

- [ ] Pick the API: `wgpu` (strongly implied — `Mat4::perspective` is written for wgpu's 0..1
      depth range, and `Vertex` is described as "uploaded verbatim")
- [ ] Add deps to `crates/render/Cargo.toml` (currently empty `[dependencies]`)
- [ ] Single vertex layout at `VERT_STRIDE = 64`, single shader, `params` driving the
      procedural facade — that is the contract `Vertex` was designed around
- [ ] `Sky` → uniform block (`sun_dir`, `sun_color`, `sun_intensity`, ambient, `fog_density`,
      `night`, `stars`, `exposure`)
- [ ] Camera: `Mat4::perspective` + `look_at` already exist in `gta-math`
- [ ] Frustum culling hook using `Aabb`

**Verify:** a window draws on native; wasm build works.
**Done:**
**Status:**

### [ ] TODO-11 — `gta-app`: entry point
**Severity:** 🟡 medium · **Depends:** TODO-09, TODO-10

`gta-app` is `//! placeholder` with no dependencies and **no binary target at all** — every
crate in the workspace is `crate-type = ["lib"]`, so there is nothing to run today.

- [ ] Add a `[[bin]]` target
- [ ] Wire it up: `generate(CityParams)` → `scene` build → `Traffic::spawn` / `Crowd::spawn` →
      fixed-step `Sky::at_time` + `Traffic::step` + `Crowd::step` → render
- [ ] Decide the sim tick rate and whether `Traffic::step`/`Crowd::step` are fixed or variable
      `dt` (traffic steering is documented as "stable at any time step" — worth a test)
- [ ] Add a `headless` example/bench so CI can measure sim throughput without a GPU

**Verify:** `cargo run` opens a window; `cargo run --release -- --headless` reports ms/tick.
**Done:**
**Status:**

### [ ] TODO-12 — `gta-avatar`: character geometry is entirely absent
**Severity:** 🟡 medium · **Depends:** TODO-09

`gta-avatar` is `//! placeholder`, yet the surrounding design already assumes it:
`PedPose { stride, tint, height }` is documented as *"Walk-cycle phase 0..1 — the renderer
swings the legs from this"*, and `Vertex` docs claim one shader draws every material including
"skin". There is no body to swing.

- [ ] Decide: procedural low-poly humanoid from existing primitives (`capsule`, `capsule`
      limbs, `blob` head) vs. a rigged mesh asset
- [ ] Body parts as separate instanced transforms driven by `PedPose.stride`
- [ ] `tint` → wardrobe palette from a deterministic RNG stream
- [ ] `height` scaling contract with `Ped.height`

**Verify:** `cargo test -p gta-avatar` with a pose-driven mesh test.
**Done:**
**Status:**

---

## P3 — Testing gaps

### [ ] TODO-13 — Cover the untested modules
**Severity:** 🟡 medium · **Depends:** TODO-01

Test distribution is very uneven — 42 of 97 tests live in `city`. Modules with **zero** tests:

| file | LOC | notes |
|---|---|---|
| `agents/traffic.rs` | 435 | **0 tests**, and not compiled — the whole car model is unverified |
| `agents/crowd.rs` | 323 | **0 tests**, same |
| `geo/primitives.rs` | 464 | 21 pub builders, 0 tests — no degenerate-triangle guard |
| `geo/grid.rs` | 156 | 0 tests — spatial hash correctness unproven |
| `math/aabb.rs` | 164 | 4 tests only |

Specific assertions worth adding:
- [ ] `GridIndex`: item spanning multiple cells is found by `query_exact`; `nearest` returns
      true nearest, not nearest-in-first-cell
- [ ] `primitives`: every builder yields `triangle_count() > 0` and no zero-area faces
- [ ] `traffic`: a car halts before a red light and never passes the stop line
- [ ] `traffic`: queueing produces non-decreasing gaps (no interpenetration over N steps)
- [ ] `crowd`: **jaywalking-impossible-by-construction** is currently only an architectural
      claim — assert no `Ped` ever reaches a non-crossing node while mid-carriageway
- [ ] `crowd`: pedestrians never end up inside a building footprint

**Verify:** `cargo test --workspace` and per-crate counts rise for `agents`, `geo`.
**Done:**
**Status:**

### [ ] TODO-14 — Property / fuzz testing for the determinism invariant
**Severity:** 🟢 low · **Depends:** TODO-08

`generation_is_deterministic` compares two runs of the same seed today. Worth hardening:

- [ ] Golden-file golden test (hash of building rects/heights) committed to the repo
- [ ] Cross-target determinism check (host vs wasm32) once CI exists
- [ ] Property test: changing block `i`'s content never changes any other block's output
      (this is the reason the per-block salt scheme exists)

**Verify:** `cargo test --workspace` includes the golden/property tests and they pass.
**Done:**
**Status:**

---

## Deferred / nice to have

- [ ] TODO-15 — `docs/` or `cargo doc` pass: fix dangling intra-doc links to `render`/`scene`
      (`tri.rs` and `sky/src/lib.rs` currently reference crates that are empty)
- [ ] TODO-16 — Batch generation for large cities: `city::block_at` is a linear scan over all
      blocks (`blocks.iter().find(...)`) — fine at 11×11, wrong for a real map; `GridIndex` exists
- [ ] TODO-17 — Parallelize `generate()`'s per-block loop (blocks already have independent RNG
      streams, so they are embarrassingly parallel by design)
- [ ] TODO-18 — Traffic-light coverage: verify every intersection is green on some phase
      (no permanently-red approach)
- [ ] TODO-19 — Turns are decided "once per junction" via a `NAN` sentinel in `handled`;
      document/replace the magic float sentinel with an explicit type
- [ ] TODO-20 — `avatar` / `scene` / `render` / `app`: add crate-level docs now, in house style,
      so intent is recorded even while empty

---

## Closed

*(nothing closed yet — move items here with `Done:` date + commit SHA)*
