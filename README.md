# jolt_sys

Raw `extern "C"` bindings to Jolt Physics. Builds the Jolt static library
from the vendored checkout and exposes a flat C ABI over a C++ shim
(`wrapper/wrapper.cpp`). No Bevy, no game logic, no opinions.

## Layout

- `vendor/JoltPhysics/` — upstream Jolt source, untouched.
- `wrapper/wrapper.cpp` — the only hand-written C++. One `BJoltWorld`
  struct owns everything (world, job system, registries, listeners);
  every `bjolt_*` function takes the world pointer first.
- `src/lib.rs` — `unsafe extern "C"` declarations mirroring the wrapper.
  If you add a wrapper function, declare it here with identical types.

## Rules for agents touching this crate

1. **No header file.** `wrapper.cpp` compiles directly via cmake — there is
   no `wrapper.h` to update. Includes live at the top of the cpp.
2. **New Jolt class = new include.** Every shape/constraint/listener header
   must be included explicitly (e.g. `ConvexHullShape.h`,
   `BodyActivationListener.h`). A missing include surfaces as
   `undefined class` / `unidentified identifier`, never as a Rust error —
   check the C++ log first when `cargo check -p jolt_sys` fails.
3. **Signatures must match exactly** across cpp definition ↔ `lib.rs`
   declaration ↔ bevy_jolt import. A dropped parameter in any one of the
   three reads as "not found in scope" downstream, far from the cause.
   After editing the import block in bevy_jolt, re-read the whole block:
   adjacent lines get eaten silently.
4. **Bad ids must be safe.** Every function validates or no-ops on stale
   body/constraint ids (`nullptr` checks, `HasError()` on shape cooks,
   early `return 0`). Rust callers hold raw `u32` ids across frames;
   despawn races are normal, crashes are not. Cook failures return 0 and
   Rust asserts with a message naming the shape.
   The one exception is destroy: `bjolt_body_remove_destroy` returns false
   when the id names no live body, and bevy_jolt panics on that. A stale id
   at destroy is a bug rather than a race, and Jolt's `DestroyBodies` indexes
   `mBodies[id.GetIndex()]` with no bounds check and no sequence check (its
   asserts are compiled out), so passing it through frees whatever body owns
   that slot. No-op'ing instead would leave a live body nobody owns.
   Destroying is always two calls: `RemoveBody` then `DestroyBody`. Jolt's
   `RemoveBodyInternal` requires the body out of the broadphase and inactive
   — asserts that this build compiles out — so `DestroyBody` alone on an
   active body frees its memory while leaving its id in `mActiveBodies` for
   the next `PhysicsSystem::Update` to walk. That is exactly how soft-body
   teardown used to segfault (`bjolt_soft_destroy` skipped the first call).
5. **No allocation under solver locks.** Listener callbacks
   (`ContactListener`, `BodyActivationListener`) record into fixed-cap
   arrays and return. Overflow drops the newest. Rust drains after the step
   and resets the counts. Never call back into Bevy, never allocate, never
   lock inside a callback.
6. **Motion codes are 0/1/2** (static/kinematic/dynamic), matching
   `JoltMotion` order. Comment the mapping at every site that encodes it.
7. **Activation discipline:** non-static bodies bake `Activate`;
   statics bake `DontActivate`. Kinematic bodies set
   `mAllowSleeping = false` (a sleeping kinematic stops producing
   `MoveKinematic` velocity and riders fall off). `MoveKinematic`
   re-activates explicitly — the derived velocity vanishes at sine
   extremes.
8. **Mesh/heightfield are static-only.** The wrapper ignores motion for
   them; Rust asserts before calling. Single-sided faces: winding decides
   which side collides.

## Listener inventory

| Listener | Records | Drained by |
|---|---|---|
| `BevyJoltContactListener` | begin/end pairs + sensor overlaps (cap 256) | `bjolt_drain_contact_added/removed` |
| `BevyJoltActivationListener` | slept/woke body ids (cap 64) | `bjolt_drain_slept/woke` |

Both are created with the world and destroyed with it. Both fire for manual
*and* solver-driven transitions (activate/deactivate funnel through
`BodyManager`, which notifies unconditionally).

FFI correctness is proven through `bevy_jolt/tests/` (hull/mesh/heightfield
cooks, sleep/wake round-trips).
Build check is `cargo check -p jolt_sys --offline`. If the C++ changed but
cargo reports success anyway, the build fingerprint is stale — `build.rs`
only watches `wrapper/wrapper.cpp`, and some editors preserve mtimes:
`touch wrapper/wrapper.cpp src/lib.rs` forces a real rebuild.
