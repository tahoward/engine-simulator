//! How many threads the simulation runs on, found by trying counts on the engine as it is and keeping
//! the fastest.
//!
//! The best count is not a property of the machine alone: each thread costs a hand-off per solver step,
//! so a small exhaust is fastest on one or two, a big one on more, and a user can add pipe to any
//! engine. Nor does it fall smoothly with the count, as the ducts may split evenly across four threads
//! and badly across three. So the tuner times the render thread's own blocks at every count and keeps
//! the fastest. It visits the counts in turn, a short burst each, several times over, so a cost that
//! drifts as the engine warms up weighs on every count alike; and after the first pass it drops any
//! count plainly slower than the best. It tries again after an edit, and every `RETUNE` in case the
//! machine's load has changed. The sound is the same at any count, so trying one is never heard.

use std::time::{Duration, Instant};

/// Blocks to let a count settle before timing it: its threads' caches fill with their ducts.
const WARM_BLOCKS: usize = 24;
/// Blocks timed per visit to a count, and visits per count. A count's time is the lower quartile of
/// all its blocks: the system delays some blocks, by moving a thread or slowing its core, and a count
/// is judged by what it does when it is not held up.
const TIMED_BLOCKS: usize = 24;
const PASSES: usize = 3;
/// After the first pass, a count this much slower than the best is tried no more.
const DROP_SLOWER: f64 = 1.3;
/// How near the fastest a count must be to count as fast as it.
const MARGIN: f64 = 0.03;
/// How long a settled count is kept before the tuner tries every count again.
const RETUNE: Duration = Duration::from_secs(30);

pub struct ThreadTuner {
    /// The most threads there are, and the most this engine can use.
    cap: usize,
    max: usize,
    current: usize,
    /// The count in use when this round of trying began.
    start: usize,
    /// The counts still being tried, the pass the round is on, and the next of them to visit.
    order: Vec<usize>,
    pass: usize,
    next: usize,
    /// Every block timed this round, by count.
    times: Vec<Vec<Duration>>,
    warm: usize,
    timed: usize,
    /// Settled until this time, or `None` while trying counts.
    settled_until: Option<Instant>,
}

impl ThreadTuner {
    /// Choose between 1 and `cap` threads, starting at `start`.
    pub fn new(cap: usize, start: usize) -> ThreadTuner {
        let cap = cap.max(1);
        let start = start.clamp(1, cap);
        let mut t = ThreadTuner {
            cap,
            max: cap,
            current: start,
            start,
            order: Vec::with_capacity(cap),
            pass: 0,
            next: 0,
            times: (0..=cap).map(|_| Vec::with_capacity(TIMED_BLOCKS * PASSES)).collect(),
            warm: 0,
            timed: 0,
            settled_until: None,
        };
        t.retune(cap);
        t
    }

    /// The count to run on now.
    pub fn threads(&self) -> usize {
        self.current
    }

    /// Start trying every count again, up to the `max` the engine can use, as after an edit to it.
    pub fn retune(&mut self, max: usize) {
        self.max = max.clamp(1, self.cap);
        self.current = self.current.min(self.max);
        if self.max == 1 {
            self.settled_until = Some(Instant::now() + RETUNE);
            return;
        }
        self.start = self.current;
        // The one in use first: nothing changes until the first burst is done.
        let start = self.start;
        self.order.clear();
        self.order.extend(1..=self.max);
        self.order.sort_by_key(|&n| (n.abs_diff(start), n));
        for t in self.times.iter_mut() {
            t.clear();
        }
        self.pass = 0;
        self.next = 0;
        self.settled_until = None;
        self.visit_next();
    }

    fn visit_next(&mut self) {
        self.current = self.order[self.next];
        self.next += 1;
        self.warm = 0;
        self.timed = 0;
    }

    /// Take how long a block took to render, at `now`. Returns the new count to run on if it changes.
    pub fn record(&mut self, took: Duration, now: Instant) -> Option<usize> {
        let before = self.current;
        if let Some(until) = self.settled_until {
            if now >= until {
                self.retune(self.max);
            }
            return self.changed(before);
        }
        if self.warm < WARM_BLOCKS {
            self.warm += 1;
            return None;
        }
        self.times[self.current].push(took);
        self.timed += 1;
        if self.timed < TIMED_BLOCKS {
            return None;
        }
        if self.next == self.order.len() {
            self.pass += 1;
            self.next = 0;
            if self.pass == 1 {
                let best = self.best().1;
                let times = &mut self.times;
                self.order.retain(|&n| quartile(&mut times[n]) <= best.mul_f64(DROP_SLOWER));
            }
            if self.pass == PASSES || self.order.len() == 1 {
                self.settle(now);
                return self.changed(before);
            }
        }
        self.visit_next();
        self.changed(before)
    }

    /// The fastest count still being tried, and its time.
    fn best(&mut self) -> (usize, Duration) {
        let mut best = (self.start, Duration::MAX);
        for &n in &self.order {
            if self.times[n].is_empty() {
                continue;
            }
            let m = quartile(&mut self.times[n]);
            if m < best.1 {
                best = (n, m);
            }
        }
        best
    }

    fn changed(&self, before: usize) -> Option<usize> {
        (self.current != before).then_some(self.current)
    }

    /// Settle on the count in use if it is within `MARGIN` of the fastest, and otherwise on the fewest
    /// threads that are: no more cores busy than buy a real gain.
    fn settle(&mut self, now: Instant) {
        let limit = self.best().1.as_secs_f64() * (1.0 + MARGIN);
        let mut within = Vec::with_capacity(self.order.len());
        for &n in &self.order {
            if quartile(&mut self.times[n]).as_secs_f64() <= limit {
                within.push(n);
            }
        }
        self.current = if within.contains(&self.start) { self.start } else { within.into_iter().min().unwrap() };
        self.settled_until = Some(now + RETUNE);
    }
}

fn quartile(times: &mut [Duration]) -> Duration {
    times.sort_unstable();
    times[times.len() / 4]
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Run the tuner against block times given by `cost(threads)`, until it settles; the count it
    /// settles on.
    fn settle_on(max: usize, start: usize, cost: impl Fn(usize) -> f64) -> usize {
        let mut t = ThreadTuner::new(max, start);
        let now = Instant::now();
        for _ in 0..100_000 {
            if t.settled_until.is_some() {
                return t.threads();
            }
            t.record(Duration::from_secs_f64(cost(t.threads()) * 1e-6), now);
        }
        panic!("never settled");
    }

    #[test]
    fn finds_the_fastest_count() {
        // Fastest at 4: work shared out, plus a hand-off cost per thread.
        let cost = |n: usize| 100.0 / n as f64 + 6.0 * n as f64;
        assert_eq!(settle_on(6, 2, cost), 4);
    }

    #[test]
    fn finds_it_past_a_count_that_splits_badly() {
        let cost = |n: usize| match n {
            1 => 100.0,
            2 => 80.0,
            3 => 78.0,
            4 => 69.0,
            _ => 75.0,
        };
        assert_eq!(settle_on(6, 2, cost), 4);
    }

    #[test]
    fn comes_down_when_more_is_slower() {
        let cost = |n: usize| 10.0 + 8.0 * n as f64;
        assert_eq!(settle_on(6, 3, cost), 1);
    }

    #[test]
    fn stays_put_at_the_best_count() {
        let cost = |n: usize| 100.0 / n as f64 + 25.0 * n as f64;
        assert_eq!(settle_on(6, 2, cost), 2);
    }

    #[test]
    fn uses_no_more_than_it_may() {
        let cost = |n: usize| 100.0 / n as f64;
        assert_eq!(settle_on(3, 1, cost), 3);
    }

    #[test]
    fn stops_trying_a_count_plainly_slower() {
        let mut t = ThreadTuner::new(3, 2);
        let now = Instant::now();
        let mut visits = [0; 4];
        while t.settled_until.is_none() {
            let us = if t.threads() == 1 { 100 } else { 50 };
            if t.record(Duration::from_micros(us), now).is_some() || t.warm == 0 {
                visits[t.threads()] += 1;
            }
        }
        assert_eq!(visits[1], 1, "one slow count tried once");
    }

    #[test]
    fn a_drift_while_trying_does_not_pick_a_count() {
        // Every block slower than the last, as an engine warming up; all counts alike otherwise.
        let mut t = ThreadTuner::new(4, 2);
        let now = Instant::now();
        let mut us = 50.0;
        while t.settled_until.is_none() {
            t.record(Duration::from_secs_f64(us * 1e-6), now);
            us *= 1.002;
        }
        assert_eq!(t.threads(), 2);
    }

    #[test]
    fn a_count_barely_faster_is_not_worth_a_change() {
        let cost = |n: usize| if n == 3 { 99.0 } else { 100.0 };
        assert_eq!(settle_on(6, 2, cost), 2);
    }

    #[test]
    fn of_counts_as_fast_takes_the_fewest() {
        let cost = |n: usize| if n == 1 { 200.0 } else { 100.0 - n as f64 * 0.1 };
        assert_eq!(settle_on(6, 1, cost), 2);
    }

    #[test]
    fn an_engine_that_can_use_one_thread_gets_one() {
        let mut t = ThreadTuner::new(6, 3);
        t.retune(1);
        assert_eq!(t.threads(), 1);
        assert_eq!(t.record(Duration::from_micros(100), Instant::now()), None);
    }

    #[test]
    fn tries_again_after_retune_and_after_a_while() {
        let mut t = ThreadTuner::new(4, 1);
        let now = Instant::now();
        while t.settled_until.is_none() {
            t.record(Duration::from_micros(100), now);
        }
        assert_eq!(t.record(Duration::from_micros(100), now + RETUNE / 2), None);
        t.record(Duration::from_micros(100), now + RETUNE * 2);
        assert!(t.settled_until.is_none(), "trying counts again");
    }
}
