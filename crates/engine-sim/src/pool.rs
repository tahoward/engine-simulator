//! A small pool of spinning worker threads, for work split finer than an operating system can
//! schedule: the exhaust's ducts, two hand-offs per solver step, hundreds of thousands a second.
//!
//! A worker waiting for its next job spins rather than sleeps, as waking a sleeping thread costs more
//! than a step of the solver. It goes to sleep after `IDLE` without a job of its own, so a paused
//! engine, or one given fewer threads than the pool has, does not hold cores busy, and the next job
//! that includes it wakes it.
//!
//! The pool needs threads, so in Wasm it has no workers and runs every job on the caller's thread.

use std::cell::UnsafeCell;
use std::marker::PhantomData;
use std::sync::Arc;
use std::sync::atomic::{AtomicBool, AtomicU64, AtomicUsize, Ordering};
use std::thread::{self, JoinHandle};
use std::time::{Duration, Instant};

/// How long a worker spins without a job before it sleeps.
const IDLE: Duration = Duration::from_millis(5);

/// Bits of `Shared::generation` that hold the participant count.
const ACTIVE_BITS: u32 = 16;

/// A job: called once with each participant's index, 0 for the caller.
type Job = dyn Fn(usize) + Sync;

/// On its own cache line, so the threads polling one counter do not slow the threads writing another.
#[repr(align(128))]
struct Padded<T>(T);

/// A value on cache lines of its own, for one of a row of results each written from its own thread.
#[derive(Clone, Copy, Debug, Default)]
#[repr(align(128))]
pub struct CachePadded<T>(pub T);

/// A slice handed out to a job's threads an element at a time: each element to one thread only.
pub struct Disjoint<'a, T> {
    ptr: *mut T,
    len: usize,
    _slice: PhantomData<&'a mut [T]>,
}

// Each element is reached only through `get`, by the one thread it was given to.
unsafe impl<T: Send> Sync for Disjoint<'_, T> {}
unsafe impl<T: Send> Send for Disjoint<'_, T> {}

impl<'a, T> Disjoint<'a, T> {
    pub fn new(slice: &'a mut [T]) -> Disjoint<'a, T> {
        Disjoint { ptr: slice.as_mut_ptr(), len: slice.len(), _slice: PhantomData }
    }

    /// Element `i`.
    ///
    /// # Safety
    ///
    /// No other thread may hold element `i` while the result is alive.
    #[allow(clippy::mut_from_ref)]
    pub unsafe fn get(&self, i: usize) -> &mut T {
        assert!(i < self.len);
        unsafe { &mut *self.ptr.add(i) }
    }

    /// A copy of element `i`, for any thread to read once the one that writes it is done with it.
    ///
    /// # Safety
    ///
    /// No thread may be writing element `i` while it is read.
    pub unsafe fn read(&self, i: usize) -> T
    where
        T: Copy,
    {
        assert!(i < self.len);
        unsafe { std::ptr::read(self.ptr.add(i)) }
    }
}

struct Shared {
    /// Bumped once per job, in the bits above `ACTIVE_BITS`, with the job's participant count, the
    /// caller included, in the bits below: one word, so a worker never reads one job's count with
    /// another's generation. A worker runs a job when it sees a generation it has not.
    generation: Padded<AtomicU64>,
    /// Workers still running the current job.
    remaining: Padded<AtomicUsize>,
    /// Per worker, by index, whether it is asleep or on its way to sleep. Index 0 is the caller's.
    asleep: Vec<Padded<AtomicBool>>,
    job: UnsafeCell<*const Job>,
    stop: AtomicBool,
}

// `job` is written only by the caller, before it publishes the generation, and read only by workers
// after they see it.
unsafe impl Sync for Shared {}
unsafe impl Send for Shared {}

pub struct ThreadPool {
    shared: Arc<Shared>,
    workers: Vec<JoinHandle<()>>,
}

impl ThreadPool {
    /// A pool that runs jobs on `threads` threads: the caller's and `threads - 1` workers. Each worker
    /// calls `on_start` first, where the caller can raise its priority.
    pub fn new(threads: usize, on_start: Option<Arc<dyn Fn() + Send + Sync>>) -> ThreadPool {
        let shared = Arc::new(Shared {
            generation: Padded(AtomicU64::new(0)),
            remaining: Padded(AtomicUsize::new(0)),
            asleep: (0..threads.max(1)).map(|_| Padded(AtomicBool::new(false))).collect(),
            job: UnsafeCell::new(std::ptr::null::<fn(usize)>() as *const Job),
            stop: AtomicBool::new(false),
        });
        let spawned = if cfg!(target_arch = "wasm32") { 0 } else { threads.saturating_sub(1) };
        let workers = (1..=spawned)
            .map(|index| {
                let shared = shared.clone();
                let on_start = on_start.clone();
                thread::Builder::new()
                    .name(format!("engine-sim worker {index}"))
                    .spawn(move || {
                        if let Some(f) = on_start {
                            f();
                        }
                        work(&shared, index);
                    })
                    .expect("a worker thread starts")
            })
            .collect();
        ThreadPool { shared, workers }
    }

    /// Threads a job can run on, the caller's included.
    pub fn threads(&self) -> usize {
        self.workers.len() + 1
    }

    /// Run `job` on the first `participants` threads at once, each with its own index, the caller's
    /// thread being 0, and return when every one has finished.
    pub fn run(&self, participants: usize, job: &(dyn Fn(usize) + Sync + '_)) {
        let participants = participants.clamp(1, self.threads().min(1 << ACTIVE_BITS));
        if participants == 1 {
            job(0);
            return;
        }
        let s = &*self.shared;
        // The workers see the job only through this pointer, and only until `remaining` reaches zero,
        // which `Wait` holds this call for, so the lifetime it is stretched past is never used.
        unsafe { *s.job.get() = std::mem::transmute::<&(dyn Fn(usize) + Sync + '_), &'static Job>(job) as *const Job };
        s.remaining.0.store(participants - 1, Ordering::Relaxed);
        let next = ((s.generation.0.load(Ordering::Relaxed) >> ACTIVE_BITS) + 1) << ACTIVE_BITS;
        s.generation.0.store(next | participants as u64, Ordering::SeqCst);
        for (w, worker) in self.workers.iter().enumerate().take(participants - 1) {
            if s.asleep[w + 1].0.load(Ordering::SeqCst) {
                worker.thread().unpark();
            }
        }
        let _wait = Wait(s);
        job(0);
    }
}

/// Waits for the workers to finish the current job, even if the caller's share panics.
struct Wait<'a>(&'a Shared);

impl Drop for Wait<'_> {
    fn drop(&mut self) {
        while self.0.remaining.0.load(Ordering::Acquire) != 0 {
            std::hint::spin_loop();
        }
    }
}

impl Drop for ThreadPool {
    fn drop(&mut self) {
        self.shared.stop.store(true, Ordering::SeqCst);
        self.shared.generation.0.fetch_add(1 << ACTIVE_BITS, Ordering::SeqCst);
        for w in self.workers.drain(..) {
            w.thread().unpark();
            let _ = w.join();
        }
    }
}

fn work(s: &Shared, index: usize) {
    let includes = |g: u64| (index as u64) < g & ((1 << ACTIVE_BITS) - 1);
    // The generation the pool starts at, which no job has: a worker that starts after its first job
    // was published still runs it.
    let mut seen = 0;
    let mut last_job = Instant::now();
    let mut spins = 0u32;
    loop {
        let g = s.generation.0.load(Ordering::Acquire);
        if g != seen {
            seen = g;
            if s.stop.load(Ordering::Acquire) {
                return;
            }
            if includes(g) {
                let job = unsafe { &**s.job.get() };
                job(index);
                s.remaining.0.fetch_sub(1, Ordering::Release);
                last_job = Instant::now();
            }
            continue;
        }
        std::hint::spin_loop();
        spins = spins.wrapping_add(1);
        if spins.is_multiple_of(1024) && last_job.elapsed() > IDLE {
            // Announced before the generation is checked again, and the caller publishes a job before
            // it checks who is asleep: one of the two sees the other. Jobs that leave this worker
            // out do not wake it.
            s.asleep[index].0.store(true, Ordering::SeqCst);
            loop {
                if s.stop.load(Ordering::SeqCst) {
                    return;
                }
                let g = s.generation.0.load(Ordering::SeqCst);
                if g != seen && includes(g) {
                    break;
                }
                thread::park();
            }
            s.asleep[index].0.store(false, Ordering::SeqCst);
            last_job = Instant::now();
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::sync::atomic::AtomicUsize;

    #[test]
    fn every_participant_runs_once_per_job() {
        let pool = ThreadPool::new(4, None);
        let counts: Vec<AtomicUsize> = (0..4).map(|_| AtomicUsize::new(0)).collect();
        for _ in 0..10_000 {
            pool.run(3, &|w| {
                counts[w].fetch_add(1, Ordering::Relaxed);
            });
        }
        let got: Vec<usize> = counts.iter().map(|c| c.load(Ordering::Relaxed)).collect();
        assert_eq!(got, vec![10_000, 10_000, 10_000, 0]);
    }

    #[test]
    fn a_worker_left_out_sleeps_and_is_woken_when_wanted() {
        let pool = ThreadPool::new(3, None);
        let n = AtomicUsize::new(0);
        let start = Instant::now();
        while start.elapsed() < IDLE * 4 {
            pool.run(2, &|_| {
                n.fetch_add(1, Ordering::Relaxed);
            });
        }
        assert!(pool.shared.asleep[2].0.load(Ordering::SeqCst), "the third thread was never needed");
        assert!(!pool.shared.asleep[1].0.load(Ordering::SeqCst));
        let before = n.load(Ordering::Relaxed);
        pool.run(3, &|_| {
            n.fetch_add(1, Ordering::Relaxed);
        });
        assert_eq!(n.load(Ordering::Relaxed), before + 3);
    }

    #[test]
    fn wakes_after_sleeping() {
        let pool = ThreadPool::new(2, None);
        let n = AtomicUsize::new(0);
        pool.run(2, &|_| {
            n.fetch_add(1, Ordering::Relaxed);
        });
        thread::sleep(IDLE * 3);
        pool.run(2, &|_| {
            n.fetch_add(1, Ordering::Relaxed);
        });
        assert_eq!(n.load(Ordering::Relaxed), 4);
    }
}
