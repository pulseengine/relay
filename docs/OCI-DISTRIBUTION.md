# OCI distribution + wasm.directory

The verified flight component (`falcon:flight`, the `falcon-flight-vX.wasm`
artifact) is published to **ghcr.io as an OCI 1.1 artifact** on every tagged
release, in addition to the cosign-signed GitHub Release. This makes it
`wkg oci pull`-able and indexable by [wasm.directory](https://wasm.directory) —
a meta-registry that indexes OCI registries rather than hosting packages itself.

## What the release does automatically

`.github/workflows/release.yml` (flight-component job) pushes the exact wasm the
release attached:

```
ghcr.io/pulseengine/falcon-flight:<full-version>   # e.g. 1.127.0
ghcr.io/pulseengine/falcon-flight:latest
```

pushed with `wkg` (wasm-pkg-tools) so it carries the Component-Model OCI media
type wasm.directory expects — not a generic blob — and cosign-signed keyless,
like the release bundle. The step is **non-blocking** (`continue-on-error`): a
registry hiccup must never fail the primary signed GitHub Release. It will be
tightened to a hard failure once it has proven itself across a few tags.

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
2. **Register the namespace with wasm.directory.** Per its publishing guide,
   add the `pulseengine` namespace → `ghcr.io` mapping to the wasm.directory
   registry config (or your account's namespace list), then it indexes the
   package. wasm.directory is **alpha** ("indexed data may be incomplete or
   reset without notice; don't depend on it in production") — treat the listing
   as a visibility channel, not a dependency. The durable, signed artifacts
   remain the GitHub Release and the ghcr OCI ref.

## Scope note

`falcon:flight`'s current world (`flight-demo`) exports two runnable smoke-test
functions (`run-stabilization`, `run-position-hold`) — it is a *runnable*
verified component, not yet a reusable library exposing a typed control
interface. Publishing it is a provenance/visibility play (a formally-verified
flight component, signed, in the public component ecosystem). A richer typed
`falcon:flight` interface others could import is a separate follow-on.
