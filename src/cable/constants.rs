//! Tuning thresholds for cable observers.

// Alignment
/// Dot-product threshold above which the alignment observer skips writing back
/// to `Transform`. Prevents an infinite recompute cycle of geometry → alignment
/// → geometry.
pub(super) const ALIGNMENT_FEEDBACK_GUARD: f32 = 0.9999;
