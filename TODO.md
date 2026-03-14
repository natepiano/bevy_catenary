# Current TODO

## Steady State

- Treat `cargo mend` as part of both the local pre-push checklist and CI.
- Keep CI uniform across owned repositories and only vary it for real repo-specific extras, such as benchmarks.
- When a new CI optimization or fix is discovered, backport it to every already-standardized owned repo before moving on.
- Commit `Cargo.lock` for CI determinism unless a repo has a specific reason not to.
- Update this file after every 5 successful activity completions so it stays current.

## Canonical CI

- Use `actions/checkout@v6`.
- Set top-level workflow env:
  - `CARGO_TERM_COLOR: always`
  - `CARGO_TERM_PROGRESS_WHEN: always`
  - `CARGO_TERM_PROGRESS_WIDTH: 80`
  - `FORCE_JAVASCRIPT_ACTIONS_TO_NODE24: true`
  - `TAPLO_VERSION: 0.10.0`
- Keep `Taplo Check` as a separate job.
- Install Taplo from a pinned GitHub release binary via `taiki-e/install-action`.
- Use `cargo nextest run --all-features --workspace --tests` as the standard test command.
- Keep `cargo mend --fail-on-warn` in CI.

## Cache Policy

- Keep `Swatinem/rust-cache@v2` for now because its cache-pruning behavior is better than a naive direct cache replacement.
- Accept the current `actions/cache/restore@v4` warning while `rust-cache@v2` remains the best practical option.
- Upgrade the cache path as soon as a Node-24-safe replacement is ready without regressing cache size or effectiveness.

## Local Validation

- Canonical helper script: `~/.claude/scripts/validate_ci.sh`
- Standard repos should validate:
  - formatting
  - Taplo
  - clippy
  - build
  - nextest
  - `cargo mend`

## Notes

- `hana` is functionally green on `feat/box-select`, but its CI build path is still unusually slow and may need later performance work.
- `bevy_liminal` is the intended replacement for `bevy_mesh_outline`.
