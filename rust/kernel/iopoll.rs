// SPDX-License-Identifier: GPL-2.0

//! Functions to poll an address until a condition is met or a timeout occurs.

use crate::{error::code::*, bindings, Result};
use core::time::Duration;

const NANOS_PER_SEC: u64 = 1_000_000_000;

/// Return duration in nanoseconds, saturating up to U64_MAX.
fn duration_nanos(duration: Duration) -> u64 {
    let seconds_as_nanos = duration.as_secs().saturating_mul(NANOS_PER_SEC);
    seconds_as_nanos.saturating_add(duration.subsec_nanos() as u64)
}

/// Periodically poll an address until the 'done' predicate returns is true or a timeout occurs.
pub fn read_poll_timeout<F>(done: F, sleep: Duration, timeout: Duration) -> Result<()>
where
    F: Fn() -> bool
{
    let start_ns = unsafe { bindings::ktime_get() } as u64;
    let deadline_ns = start_ns.saturating_add(duration_nanos(timeout));
    // TODO(q3k): take seconds into account
    let sleep_us = sleep.subsec_micros();

    // TODO: might_sleep_if(duration)
    loop {
        let d = done();
        if d {
            return Ok(());
        }

        if !timeout.is_zero() {
            let now = unsafe { bindings::ktime_get() } as u64;
            if now > deadline_ns {
                return Err(ETIMEDOUT);
            }
        }

        if sleep_us != 0 {
            unsafe {
                bindings::usleep_range_state((sleep_us >> 2) + 1, sleep_us, bindings::TASK_UNINTERRUPTIBLE);
            }
        }
    }
}