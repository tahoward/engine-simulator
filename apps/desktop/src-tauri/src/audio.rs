//! The simulation running natively, and its sound going out through the system's audio device.
//!
//! Three threads, so that nothing the audio device waits on can be held up:
//!
//! ```text
//!   render thread                 device callback              frame thread
//!   ─────────────────             ───────────────              ────────────
//!   owns EngineSim                drains the ring,             sends snapshots and
//!   renders blocks ahead ──ring──▸ copies to every channel     lag reports to the UI
//!   into a ring buffer            counts underruns
//!   takes snapshots ─────────────────────────────────────────▸
//! ```
//!
//! The render thread keeps the ring filled a couple of device buffers ahead, so a slow block, a
//! rebuild of the exhaust after an edit, or the operating system briefly scheduling something else,
//! is absorbed rather than heard. The device callback does nothing but copy: it takes no lock and
//! allocates nothing. Commands from the UI reach the render thread over a channel, so everything
//! that allocates (a new exhaust, a new engine) happens there.
//!
//! Unlike an AudioWorklet, the render thread can time itself, so it reports when the simulation is
//! not keeping up with real time from its own measurements, and from the underruns the callback saw.
//!
//! The render thread also has a pool of worker threads to step the simulation across, and times its
//! own blocks to choose how many of them to use (see `tuner`).

use std::sync::Arc;
use std::sync::atomic::{AtomicBool, AtomicU64, Ordering};
use std::sync::mpsc::{self, Receiver, Sender, TryRecvError};
use std::thread::{self, JoinHandle};
use std::time::{Duration, Instant};

use cpal::traits::{DeviceTrait, HostTrait, StreamTrait};
use engine_sim::exhaust_graph::ExhaustGraph;
use engine_sim::listener::SoundSources;
use engine_sim::pool::ThreadPool;
use engine_sim::{EngineConfig, EngineSim, LaunchConfig};
use serde::{Deserialize, Serialize};

use crate::tuner::ThreadTuner;

/// Samples the render thread produces at a time.
const BLOCK: usize = 128;
/// Output samples kept for the scope's waveform and spectrum: the web app's analyser size.
const WAVEFORM: usize = 2048;
/// How long a window the load is judged over, and the share of real time a window may take before
/// it counts as late. The same rule the web app applies to its audio clock.
const LAG_WINDOW: Duration = Duration::from_secs(2);
const LAG_RATIO: f64 = 0.98;
const LAG_WINDOWS_BAD: u32 = 2;
const LAG_WINDOWS_GOOD: u32 = 3;
/// Most threads the simulation is stepped on, the render thread's included: past this, each one's
/// hand-off costs more than it takes on.
const MAX_THREADS: usize = 6;

/// A change from the UI, as the web app's worklet messages carry it, plus what the web app sets
/// through AudioParams and its AudioContext.
#[derive(Debug, Deserialize)]
#[serde(tag = "type", rename_all = "camelCase")]
pub enum Command {
    /// The `EngineSpec` fields to change.
    Engine {
        engine: serde_json::Value,
    },
    Graph {
        graph: Option<ExhaustGraph>,
    },
    Launch {
        config: Option<LaunchConfig>,
    },
    /// Where the engine makes its sound.
    Sources {
        sources: SoundSources,
    },
    /// Where the listener's ear is, m; `None` where it stands by default.
    Listener {
        position: Option<[f64; 3]>,
    },
    SnapshotRate {
        hz: f64,
    },
    /// Share of real time the simulation runs at: 1 is real time, less is slow motion.
    TimeScale {
        scale: f64,
    },
    Controls {
        throttle: f64,
        load: f64,
    },
    /// Off, the engine coasts to a standstill with the stream still playing.
    Ignition {
        on: bool,
    },
    Suspend,
    Resume,
}

/// What the stream is running at, for the UI.
#[derive(Clone, Copy, Debug, Serialize)]
#[serde(rename_all = "camelCase")]
pub struct StreamInfo {
    pub sample_rate: u32,
    /// Frames per device buffer, where the device reports it.
    pub buffer_frames: Option<u32>,
    /// Samples the render thread keeps ready ahead of the device.
    pub lead_frames: u32,
}

/// A message for the UI, packed as the desktop frontend reads it: a kind byte, three bytes of
/// padding, the length of a JSON document and the document, padded to four bytes, then the waveform
/// as little-endian `f32`s.
pub enum Frame {
    Snapshot { json: Vec<u8>, waveform: Vec<f32> },
    Lag { behind: bool },
}

impl Frame {
    pub fn pack(&self) -> Vec<u8> {
        let (kind, json, wave): (u8, Vec<u8>, &[f32]) = match self {
            Frame::Snapshot { json, waveform } => (0, json.clone(), waveform),
            Frame::Lag { behind } => (1, format!("{{\"behind\":{behind}}}").into_bytes(), &[]),
        };
        let pad = (4 - json.len() % 4) % 4;
        let mut out = Vec::with_capacity(8 + json.len() + pad + wave.len() * 4);
        out.extend_from_slice(&[kind, 0, 0, 0]);
        out.extend_from_slice(&(json.len() as u32).to_le_bytes());
        out.extend_from_slice(&json);
        out.extend(std::iter::repeat_n(0u8, pad));
        for s in wave {
            out.extend_from_slice(&s.to_le_bytes());
        }
        out
    }
}

/// The running audio: the stream, the render thread, and the way to both.
pub struct Audio {
    commands: Sender<Command>,
    stop: Arc<AtomicBool>,
    render: Option<JoinHandle<()>>,
    host: Option<JoinHandle<()>>,
    host_stop: Sender<()>,
    pub info: StreamInfo,
}

impl Audio {
    /// Start the simulation from `config` and play it on the default output device, at
    /// `sample_rate` if it supports that and its own rate otherwise, with a device buffer of
    /// `buffer_frames` where it allows one. Every snapshot and lag report goes to `frames`.
    pub fn start(
        config: EngineConfig,
        sample_rate: Option<u32>,
        buffer_frames: Option<u32>,
        frames: Sender<Frame>,
    ) -> Result<Audio, String> {
        let (commands, command_rx) = mpsc::channel::<Command>();
        let stop = Arc::new(AtomicBool::new(false));
        let underruns = Arc::new(AtomicU64::new(0));
        let (info_tx, info_rx) = mpsc::channel::<Result<(StreamInfo, rtrb::Producer<f32>, thread::Thread), String>>();
        let (host_stop, host_stop_rx) = mpsc::channel::<()>();
        let (render_thread_tx, render_thread_rx) = mpsc::channel::<thread::Thread>();

        // The stream is made and kept on a thread of its own: on some platforms it may not move
        // between threads, and this way nothing else ever holds it.
        let host = {
            let underruns = underruns.clone();
            thread::Builder::new()
                .name("audio-host".into())
                .spawn(move || {
                    let render_thread = match render_thread_rx.recv() {
                        Ok(t) => t,
                        Err(_) => return,
                    };
                    match open_stream(sample_rate, buffer_frames, underruns, render_thread.clone()) {
                        Ok((stream, info, producer)) => {
                            let _ = info_tx.send(Ok((info, producer, render_thread)));
                            if let Err(e) = stream.play() {
                                eprintln!("audio: the stream would not start: {e}");
                            }
                            // Held until told to stop, then dropped here, which closes it.
                            let _ = host_stop_rx.recv();
                            drop(stream);
                        }
                        Err(e) => {
                            let _ = info_tx.send(Err(e));
                        }
                    }
                })
                .map_err(|e| e.to_string())?
        };

        let (ready_tx, ready_rx) = mpsc::channel::<Result<StreamInfo, String>>();
        let render = {
            let stop = stop.clone();
            thread::Builder::new()
                .name("engine-render".into())
                .spawn(move || {
                    let (info, producer, _) = match info_rx.recv() {
                        Ok(Ok(v)) => v,
                        Ok(Err(e)) => {
                            let _ = ready_tx.send(Err(e));
                            return;
                        }
                        Err(_) => return,
                    };
                    let _ = ready_tx.send(Ok(info));
                    render_loop(config, info, producer, command_rx, frames, stop, underruns);
                })
                .map_err(|e| e.to_string())?
        };
        let _ = render_thread_tx.send(render.thread().clone());

        let info = ready_rx.recv().map_err(|_| "the audio thread stopped before starting".to_string())??;
        Ok(Audio { commands, stop, render: Some(render), host: Some(host), host_stop, info })
    }

    pub fn send(&self, command: Command) -> Result<(), String> {
        self.commands.send(command).map_err(|_| "the audio has stopped".to_string())?;
        if let Some(t) = &self.render {
            t.thread().unpark();
        }
        Ok(())
    }
}

impl Drop for Audio {
    fn drop(&mut self) {
        self.stop.store(true, Ordering::Release);
        if let Some(t) = self.render.take() {
            t.thread().unpark();
            let _ = t.join();
        }
        let _ = self.host_stop.send(());
        if let Some(t) = self.host.take() {
            let _ = t.join();
        }
    }
}

/// The default output device's stream, and the ring its callback drains.
fn open_stream(
    sample_rate: Option<u32>,
    buffer_frames: Option<u32>,
    underruns: Arc<AtomicU64>,
    render_thread: thread::Thread,
) -> Result<(cpal::Stream, StreamInfo, rtrb::Producer<f32>), String> {
    let host = cpal::default_host();
    let device = host.default_output_device().ok_or("there is no audio output device")?;
    let default = device.default_output_config().map_err(|e| e.to_string())?;

    // The rate asked for, if the device offers it with float samples; otherwise its own.
    let chosen = sample_rate
        .and_then(|rate| {
            device.supported_output_configs().ok()?.find_map(|range| {
                if range.sample_format() != cpal::SampleFormat::F32 {
                    return None;
                }
                range.try_with_sample_rate(rate)
            })
        })
        .unwrap_or(default);
    if chosen.sample_format() != cpal::SampleFormat::F32 {
        return Err(format!("the output device wants {:?} samples; only float is supported", chosen.sample_format()));
    }
    let channels = chosen.channels() as usize;
    let rate = chosen.sample_rate();
    let buffer = match (buffer_frames, chosen.buffer_size()) {
        (Some(n), cpal::SupportedBufferSize::Range { min, max }) => Some(n.clamp(*min, *max)),
        _ => None,
    };
    let config = cpal::StreamConfig {
        channels: chosen.channels(),
        sample_rate: rate,
        buffer_size: match buffer {
            Some(n) => cpal::BufferSize::Fixed(n),
            None => cpal::BufferSize::Default,
        },
    };

    // Two device buffers and a block ahead, or 10 ms where the device does not say.
    let device_frames = buffer.unwrap_or((rate as f64 * 0.005) as u32).max(BLOCK as u32);
    let lead = 2 * device_frames + BLOCK as u32;
    let (producer, mut consumer) = rtrb::RingBuffer::<f32>::new((lead as usize).next_power_of_two() * 2);

    let stream = device
        .build_output_stream::<f32, _, _>(
            config,
            move |data: &mut [f32], _| {
                let frames = data.len() / channels;
                let available = consumer.slots();
                let take = frames.min(available);
                if let Ok(chunk) = consumer.read_chunk(take) {
                    let (a, b) = chunk.as_slices();
                    for (frame, &s) in data.chunks_mut(channels).zip(a.iter().chain(b)) {
                        frame.fill(s);
                    }
                    chunk.commit_all();
                }
                if take < frames {
                    data[take * channels..].fill(0.0);
                    underruns.fetch_add(1, Ordering::Relaxed);
                }
                render_thread.unpark();
            },
            |err| eprintln!("audio: {err}"),
            None,
        )
        .map_err(|e| e.to_string())?;
    let info = StreamInfo { sample_rate: rate, buffer_frames: buffer, lead_frames: lead };
    Ok((stream, info, producer))
}

/// Keeps the ring filled with the simulation's output, applies commands, and reports.
fn render_loop(
    config: EngineConfig,
    info: StreamInfo,
    mut producer: rtrb::Producer<f32>,
    commands: Receiver<Command>,
    frames: Sender<Frame>,
    stop: Arc<AtomicBool>,
    underruns: Arc<AtomicU64>,
) {
    let fs = info.sample_rate as f64;
    // Real-time priority, where the platform grants it, sized to the blocks this thread renders.
    let _priority = audio_thread_priority::promote_current_thread_to_real_time(BLOCK as u32, info.sample_rate).ok();

    let mut sim = EngineSim::new(fs, &config);
    // Two cores left for the device callback and the interface.
    let cores = thread::available_parallelism().map_or(1, |n| n.get());
    let max_threads = cores.saturating_sub(2).clamp(1, MAX_THREADS);
    if max_threads > 1 {
        sim.set_pool(Some(Arc::new(ThreadPool::new(max_threads, Some(Arc::new(worker_priority))))));
    }
    sim.set_budget_scale(budget_scale(fs, max_threads));
    let mut tuner = ThreadTuner::new(max_threads, 2);
    tuner.retune(sim.useful_threads());
    sim.set_max_threads(tuner.threads());
    let mut time_scale = 1.0;
    let mut block = vec![0.0f32; BLOCK];
    let mut waveform = vec![0.0f32; WAVEFORM];
    let mut wave_at = 0;
    let mut snapshot_interval = (fs / 60.0).round() as usize;
    let mut since_snapshot = 0;
    let mut suspended = false;
    let lead = info.lead_frames as usize;

    let mut window_start = Instant::now();
    let mut window_busy = Duration::ZERO;
    let mut window_samples = 0usize;
    let mut window_underruns = underruns.load(Ordering::Relaxed);
    let mut behind = false;
    let mut streak = 0;

    while !stop.load(Ordering::Acquire) {
        loop {
            match commands.try_recv() {
                Ok(command) => {
                    if apply(&mut sim, command, &mut snapshot_interval, &mut suspended, &mut time_scale, fs) {
                        tuner.retune(sim.useful_threads());
                        sim.set_max_threads(tuner.threads());
                    }
                }
                Err(TryRecvError::Empty) => break,
                Err(TryRecvError::Disconnected) => return,
            }
        }
        if suspended {
            // Silence goes out from the callback's underrun path; the simulation stands still.
            thread::park_timeout(Duration::from_millis(20));
            window_start = Instant::now();
            window_busy = Duration::ZERO;
            window_samples = 0;
            window_underruns = underruns.load(Ordering::Relaxed);
            continue;
        }
        let buffered = producer.buffer().capacity() - producer.slots();
        if buffered >= lead || producer.slots() < BLOCK {
            thread::park_timeout(Duration::from_millis(2));
            continue;
        }

        let t0 = Instant::now();
        sim.render_into(&mut block);
        let took = t0.elapsed();
        window_busy += took;
        // In slow motion a block holds few steps, too few to tell the counts apart by.
        if time_scale >= 1.0
            && let Some(threads) = tuner.record(took, Instant::now())
        {
            sim.set_max_threads(threads);
        }
        window_samples += BLOCK;
        if let Ok(mut chunk) = producer.write_chunk_uninit(BLOCK) {
            let (a, b) = chunk.as_mut_slices();
            for (dst, &s) in a.iter_mut().chain(b.iter_mut()).zip(&block) {
                dst.write(s);
            }
            unsafe { chunk.commit_all() };
        }
        for &s in &block {
            waveform[wave_at] = s;
            wave_at = (wave_at + 1) % WAVEFORM;
        }

        since_snapshot += BLOCK;
        if since_snapshot >= snapshot_interval {
            since_snapshot = 0;
            let snapshot = sim.snapshot();
            let json = serde_json::to_vec(&snapshot).expect("a snapshot serialises");
            let mut ordered = Vec::with_capacity(WAVEFORM);
            ordered.extend_from_slice(&waveform[wave_at..]);
            ordered.extend_from_slice(&waveform[..wave_at]);
            let _ = frames.send(Frame::Snapshot { json, waveform: ordered });
        }

        // Keeping up: the simulation's own time against the audio it made, and whether the device
        // ever found the ring empty.
        let elapsed = window_start.elapsed();
        if elapsed >= LAG_WINDOW && window_samples > 0 {
            let audio = window_samples as f64 / fs;
            let now_underruns = underruns.load(Ordering::Relaxed);
            let late = window_busy.as_secs_f64() > audio * LAG_RATIO || now_underruns > window_underruns;
            if late == behind {
                streak = 0;
            } else {
                streak += 1;
                if streak >= if late { LAG_WINDOWS_BAD } else { LAG_WINDOWS_GOOD } {
                    streak = 0;
                    behind = late;
                    let _ = frames.send(Frame::Lag { behind });
                }
            }
            window_start = Instant::now();
            window_busy = Duration::ZERO;
            window_samples = 0;
            window_underruns = now_underruns;
        }
    }
}

/// Apply a command; true if it changed the engine or its exhaust, which can change the best thread count.
fn apply(
    sim: &mut EngineSim,
    command: Command,
    snapshot_interval: &mut usize,
    suspended: &mut bool,
    time_scale: &mut f64,
    fs: f64,
) -> bool {
    match command {
        Command::Engine { engine } => {
            if let Err(e) = sim.set_engine_json(&engine) {
                eprintln!("audio: engine change rejected: {e}");
            }
            return true;
        }
        Command::Graph { graph } => {
            sim.set_graph(graph);
            return true;
        }
        Command::Sources { sources } => sim.set_sources(sources),
        Command::Listener { position } => sim.set_listener(position),
        Command::Launch { config } => match config {
            Some(c) => sim.start_launch(c),
            None => sim.stop_launch(),
        },
        Command::SnapshotRate { hz } => *snapshot_interval = ((fs / hz).round() as usize).max(1),
        Command::TimeScale { scale } => {
            sim.set_time_scale(scale);
            *time_scale = scale;
        }
        Command::Controls { throttle, load } => sim.set_controls(throttle, load),
        Command::Ignition { on } => sim.set_ignition(on),
        Command::Suspend => *suspended = true,
        Command::Resume => *suspended = false,
    }
    false
}

/// How many times a browser's solver budget the simulation may spend on finer cells, at sample rate
/// `fs`, Hz, on up to `threads` threads. The budget is what one browser thread affords at 48 kHz; a cell
/// costs in proportion to the sample rate, and each thread past the first is worth about a quarter of
/// one, as they share the work only as finely as the ducts split, up to the most that pays.
fn budget_scale(fs: f64, threads: usize) -> f64 {
    let parallel = (1.0 + 0.25 * (threads.max(1) - 1) as f64).min(2.25);
    parallel * 48000.0 / fs
}

/// A worker thread's priority: on macOS, the class that keeps it on a performance core. Not real-time,
/// as the render thread is: a worker spins between jobs, and a real-time thread that never yields is
/// demoted on macOS and may be killed on Linux.
fn worker_priority() {
    #[cfg(target_os = "macos")]
    {
        const QOS_CLASS_USER_INTERACTIVE: u32 = 0x21;
        unsafe extern "C" {
            fn pthread_set_qos_class_self_np(class: u32, relative_priority: i32) -> i32;
        }
        unsafe { pthread_set_qos_class_self_np(QOS_CLASS_USER_INTERACTIVE, 0) };
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn the_budget_grows_with_threads_and_shrinks_with_the_sample_rate() {
        assert_eq!(budget_scale(48000.0, 1), 1.0);
        assert_eq!(budget_scale(96000.0, 1), 0.5);
        assert_eq!(budget_scale(48000.0, 5), 2.0);
        assert_eq!(budget_scale(96000.0, 6), 1.125);
        assert_eq!(budget_scale(96000.0, 12), 1.125);
    }

    #[test]
    fn a_frame_packs_its_json_padded_then_its_waveform() {
        let frame = Frame::Snapshot { json: b"{\"a\":1}".to_vec(), waveform: vec![1.0, -2.0] };
        let bytes = frame.pack();
        assert_eq!(bytes[0], 0);
        assert_eq!(u32::from_le_bytes(bytes[4..8].try_into().unwrap()), 7);
        assert_eq!(&bytes[8..15], b"{\"a\":1}");
        assert_eq!(bytes.len(), 8 + 8 + 8);
        assert_eq!(f32::from_le_bytes(bytes[16..20].try_into().unwrap()), 1.0);
        assert_eq!(f32::from_le_bytes(bytes[20..24].try_into().unwrap()), -2.0);
    }

    /// The render thread, with this test standing in for the device: it keeps the ring topped up
    /// ahead, takes commands, and sends snapshots with the waveform at the snapshot rate.
    #[test]
    fn the_render_thread_keeps_the_ring_ahead_and_reports() {
        let path = concat!(env!("CARGO_MANIFEST_DIR"), "/../../../crates/engine-sim/tests/fixtures/presets.json");
        let presets: serde_json::Value = serde_json::from_str(&std::fs::read_to_string(path).unwrap()).unwrap();
        let config: EngineConfig = serde_json::from_value(presets["defaultConfig"].clone()).unwrap();
        let info = StreamInfo { sample_rate: 48000, buffer_frames: Some(256), lead_frames: 2 * 256 + BLOCK as u32 };
        let (producer, mut consumer) = rtrb::RingBuffer::<f32>::new(2048);
        let (commands, command_rx) = mpsc::channel();
        let (frames_tx, frames) = mpsc::channel();
        let stop = Arc::new(AtomicBool::new(false));
        let underruns = Arc::new(AtomicU64::new(0));
        let render = {
            let stop = stop.clone();
            thread::spawn(move || render_loop(config, info, producer, command_rx, frames_tx, stop, underruns))
        };
        commands.send(Command::Controls { throttle: 0.9, load: 0.0 }).unwrap();

        // A device taking 256 frames every 5.3 ms, for half a second of audio.
        let mut heard = Vec::new();
        let mut short = 0;
        while heard.len() < 24000 {
            thread::sleep(Duration::from_micros(5333));
            render.thread().unpark();
            let n = consumer.slots().min(256);
            if n < 256 {
                short += 1;
            }
            let chunk = consumer.read_chunk(n).unwrap();
            let (a, b) = chunk.as_slices();
            heard.extend_from_slice(a);
            heard.extend_from_slice(b);
            chunk.commit_all();
        }
        stop.store(true, Ordering::Release);
        render.thread().unpark();
        render.join().unwrap();

        assert!(short <= 1, "the device found the ring short {short} times");
        assert!(heard.iter().any(|&s| s.abs() > 1e-4), "silence");
        let snapshots: Vec<Frame> = frames.try_iter().collect();
        assert!(snapshots.len() >= 25, "{} snapshots in half a second", snapshots.len());
        match &snapshots[snapshots.len() - 1] {
            Frame::Snapshot { json, waveform } => {
                assert_eq!(waveform.len(), WAVEFORM);
                let v: serde_json::Value = serde_json::from_slice(json).unwrap();
                assert!(v["rpm"].as_f64().unwrap() > 0.0);
            }
            Frame::Lag { .. } => panic!("a lag report where a snapshot was expected"),
        }
    }

    #[test]
    fn commands_parse_from_the_ui() {
        let c: Command = serde_json::from_str(r#"{"type":"controls","throttle":0.5,"load":0}"#).unwrap();
        assert!(matches!(c, Command::Controls { .. }));
        let c: Command = serde_json::from_str(r#"{"type":"graph","graph":null}"#).unwrap();
        assert!(matches!(c, Command::Graph { graph: None }));
        let c: Command = serde_json::from_str(r#"{"type":"engine","engine":{"rpm":3000}}"#).unwrap();
        assert!(matches!(c, Command::Engine { .. }));
        let c: Command = serde_json::from_str(r#"{"type":"timeScale","scale":0.01}"#).unwrap();
        assert!(matches!(c, Command::TimeScale { scale } if scale == 0.01));
        let c: Command =
            serde_json::from_str(r#"{"type":"sources","sources":{"mouths":[{"duct":"a","position":[0,0,1]}]}}"#)
                .unwrap();
        assert!(matches!(c, Command::Sources { sources } if sources.mouths.len() == 1));
        let c: Command = serde_json::from_str(r#"{"type":"listener","position":[1,1.2,2]}"#).unwrap();
        assert!(matches!(c, Command::Listener { position: Some(p) } if p == [1.0, 1.2, 2.0]));
    }
}
