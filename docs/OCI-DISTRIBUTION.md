# OCI distribution + wasm.directory

The verified flight component (`falcon:flight`, the `falcon-flight-vX.wasm`
artifact) is published to **ghcr.io as an OCI 1.1 artifact** on every tagged
release, in addition to the cosign-signed GitHub Release. This makes it
`wkg oci pull`-able and indexable by [wasm.directory](https://wasm.directory) —
a meta-registry that indexes OCI registries rather than hosting packages itself.

## What the release does automatically

`.github/workflows/release.yml` (flight-component job) publishes **each verified
cascade stage as its own cosign-signed OCI entity** (jess#167 decision 5) — the
fine-grained components jess `wkg oci pull`s and fuses/lowers/places per core:

```
ghcr.io/pulseengine/falcon-flight:<full-version>     # runnable demo (sealed loop)
ghcr.io/pulseengine/falcon-iekf:<full-version>       # invariant-EKF estimator
ghcr.io/pulseengine/falcon-position:<full-version>   # position/velocity controller
ghcr.io/pulseengine/falcon-attitude:<full-version>   # geometric SO(3) attitude
ghcr.io/pulseengine/falcon-rate:<full-version>       # body-rate PID
ghcr.io/pulseengine/falcon-mixer:<full-version>      # control allocator
#   ...each also tagged :latest
```

Each is built **WASI-free** (`--no-default-features`, so it lowers to bare metal
— see `wasm/cm/*`) and pushed with `wkg` (wasm-pkg-tools) so it carries the
Component-Model OCI media type wasm.directory expects — not a generic blob — and
cosign-signed keyless by immutable digest, like the release bundle. The step is
**non-blocking** (`continue-on-error`): a registry hiccup must never fail the
primary signed GitHub Release. It will be tightened to a hard failure once it has
proven itself across a few tags. Every published component carries the full
embedded metadata contract (below) so its wasm.directory listing is real.

Pull + verify:

```sh
wkg oci pull ghcr.io/pulseengine/falcon-flight:1.127.0 -o falcon-flight.wasm
cosign verify ghcr.io/pulseengine/falcon-flight:1.127.0 \
  --certificate-identity-regexp \
    'https://github.com/pulseengine/relay/.github/workflows/release.yml@.*' \
  --certificate-oidc-issuer 'https://token.actions.githubusercontent.com'
```

## One-time manual steps (org owner)

1. **Make the ghcr package public.** ghcr packages are created *private* by
   default; wasm.directory can only index a public package. After the first
   tagged release populates it, in the org's package settings set
   `pulseengine/falcon-flight` visibility to **public** and link it to the
   `relay` repo. (One-time; subsequent pushes inherit the setting.)
2. **Register the namespace with wasm.directory.** Publishing the bytes (the
   `wkg oci push` above) and *indexing* in the meta-registry are two separate
   steps — the push does not list the package in the searchable index. Indexing
   is a GitHub issue on the index repo (`yoshuawuyts/wasm.directory`), *not* a
   config file you push. Two ways to file it, same endpoint:

   - **Tool path** (`component-cli`): from a project with a `[package]` section
     in a `wasm.toml`, run `component registry publish` — it reads
     `name`/`kind`/`registry`, checks whether the package is already indexed,
     and opens the **Registry entry** issue prefilled (`--no-open` prints the
     URL instead).
   - **By hand**: open the repo's **`registry-entry`** issue template with:

     - **kind**: `component`
     - **namespace**: `pulseengine`
     - **package name**: `falcon-flight`
     - **repository**: `falcon-flight`
     - **OCI registry** (new namespaces only): `ghcr.io/pulseengine`

   Submitting it auto-opens a PR adding `registry/pulseengine.toml`:

   ```toml
   [namespace]
   name = "pulseengine"
   registry = "ghcr.io/pulseengine"

   [[component]]
   name = "falcon-flight"
   repository = "falcon-flight"
   ```

   A **new namespace** is flagged for maintainer review, so the ghcr package must
   already exist and be **public** (step 1) before submitting — an unpullable
   entry is rejected. Do this only after a real release has populated the package.

   Adding a component to an **existing** namespace auto-merges (no maintainer
   review) — only a brand-new namespace is held for review. Registered so far:

   | component | wasm.directory | note |
   |---|---|---|
   | `falcon-flight` | ✅ indexed (#466 → #467) | |
   | `falcon-iekf` | filed #474 → PR #478 | |
   | `falcon-position` | filed #475 → PR #479 | |
   | `falcon-attitude` | filed #476 → PR #480 | |
   | `falcon-mixer` | filed #477 → PR #481 | |
   | `falcon-rate` | **HELD — do not register yet** | published 1.129.0 is a CORE MODULE, not a component; register after v1.130 republishes it |

   **Verify the payload before registering.** The OCI `config.mediaType` reads
   `application/vnd.wasm.config.v0+json` whether or not the bytes are actually a
   component, so metadata cannot be trusted here — pull the blob and check the
   8-byte header (`0061736d0d000100` = component, `0061736d01000000` = core
   module). This is how `falcon-rate:1.129.0` was caught.

   > Status: **LIVE** — the `pulseengine` namespace was accepted
   > ([wasm.directory#466](https://github.com/yoshuawuyts/wasm.directory/issues/466)
   > → [PR #467](https://github.com/yoshuawuyts/wasm.directory/pull/467) merged);
   > indexed at <https://wasm.directory/pulseengine/falcon-flight/1.128.0>.

   wasm.directory is **alpha** ("indexed data may be incomplete or reset without
   notice; don't depend on it in production") — treat the listing as a visibility
   channel, not a dependency. The durable, signed artifacts remain the GitHub
   Release and the ghcr OCI ref.

## Two build paths — and why they still differ

The components are built **twice**, and the paths do not yet agree:

| | Bazel (`rust_wasm_component_bindgen`) | `scripts/build-components.sh` |
|---|---|---|
| used by | the cascade / `wac_plug` / `meld_fuse` / `synth_compile` graph | the **release** (what we publish) |
| variant | **std** (carries WASI imports) | **no_std, WASI-free** |
| on build error | fails the build | fails the build (since v1.130; it used to `|| true`) |

The published artifact is the WASI-free one, and only the script produces it today.
The feature flags are now properly independent — `bazel-bindings` selects the
bindings source, `std` selects std-vs-no_std — but the Bazel target must still
opt into `std`, because building it `no_std` needs two things it does not have:

1. `lol_alloc` in the Bazel `deps` (Bazel does not read `Cargo.toml` deps), and
2. a genuine no_std rules_rust configuration — otherwise std is still linked and
   the crate's own `#[panic_handler]` collides ("duplicate lang item `panic_impl`").

**Converging them is the open follow-up**: once Bazel can build the no_std
variant, the release should consume Bazel outputs and this script becomes a thin
bundler. That removes the whole class of bug that shipped `falcon-rate:1.129.0`
as a core module (silent build failure + untyped artifact selection), because a
Bazel action failure cannot be swallowed and its outputs are declared.

## Component metadata contract (what wasm.directory renders)

wasm.directory shows a component's **WIT interface** (with its `///` doc comments)
plus the **embedded package metadata** — `description`, `licenses`, `source`,
`authors`, `version` — which `cargo-component` bakes into the wasm
`package-metadata` section from the crate's `Cargo.toml`. Empty metadata → a bare
listing. So **every published relay component MUST carry**, in its (standalone)
`Cargo.toml` — it can't inherit the workspace root:

```toml
description = "<one sentence a stranger understands — lead with what it IS>"
license = "Apache-2.0"
repository = "https://github.com/pulseengine/relay"
documentation = "<link to the docs for this component>"
authors = ["PulseEngine"]
keywords = ["flight-control", "verified", "drone", "embedded", "wasm"]
categories = ["aerospace", "science::robotics"]
```

…and a WIT world/interface with real `///` doc comments (the interface docs come
straight from the WIT). This is the metadata pattern the per-component publishing
(the `falcon-rate`/estimator/mixer components, jess#167) inherits — a listing is
only as informative as the metadata the component carries.

## Scope note

`falcon:flight`'s current world (`flight-demo`) exports two runnable smoke-test
functions (`run-stabilization`, `run-position-hold`) — it is a *runnable*
verified component, not yet a reusable library exposing a typed control
interface. Publishing it is a provenance/visibility play (a formally-verified
flight component, signed, in the public component ecosystem). A richer typed
`falcon:flight` interface others could import is a separate follow-on.
