//! The engine simulator as a desktop app.
//!
//! The interface is the web app's, built for the desktop (`npm run build:desktop` in apps/web), in
//! the system webview. The simulation is `engine-sim`, compiled natively and run on a real-time
//! render thread (`audio`) rather than in an AudioWorklet. The frontend drives it with these
//! commands and hears back through a channel of packed frames: see `audio::Frame`.

#![cfg_attr(not(debug_assertions), windows_subsystem = "windows")]

mod audio;
mod tuner;

use std::sync::Mutex;
use std::sync::mpsc;

use audio::{Audio, Command, Frame, StreamInfo};
use engine_sim::EngineConfig;
use tauri::ipc::{Channel, InvokeResponseBody};
use tauri_plugin_dialog::DialogExt;

#[derive(Default)]
struct State {
    audio: Mutex<Option<Audio>>,
}

/// Start the audio from `config`, replacing any already running. Frames arrive on `frames`.
#[tauri::command]
fn audio_start(
    state: tauri::State<'_, State>,
    config: EngineConfig,
    sample_rate: Option<u32>,
    buffer_frames: Option<u32>,
    frames: Channel<InvokeResponseBody>,
) -> Result<StreamInfo, String> {
    let mut slot = state.audio.lock().map_err(|e| e.to_string())?;
    // Any running stream closes before the new one opens.
    *slot = None;
    let (tx, rx) = mpsc::channel::<Frame>();
    std::thread::Builder::new()
        .name("engine-frames".into())
        .spawn(move || {
            // Ends when the render thread drops its sender.
            while let Ok(frame) = rx.recv() {
                if frames.send(InvokeResponseBody::Raw(frame.pack())).is_err() {
                    break;
                }
            }
        })
        .map_err(|e| e.to_string())?;
    let audio = Audio::start(config, sample_rate, buffer_frames, tx)?;
    let info = audio.info;
    *slot = Some(audio);
    Ok(info)
}

/// A change for the running simulation. Ignored if nothing is running.
#[tauri::command]
fn audio_command(state: tauri::State<'_, State>, command: Command) -> Result<(), String> {
    let slot = state.audio.lock().map_err(|e| e.to_string())?;
    match slot.as_ref() {
        Some(audio) => audio.send(command),
        None => Ok(()),
    }
}

/// Stop the audio and close the stream.
#[tauri::command]
fn audio_stop(state: tauri::State<'_, State>) -> Result<(), String> {
    *state.audio.lock().map_err(|e| e.to_string())? = None;
    Ok(())
}

/// Ask where to save an exported engine, suggesting `name`, and write `contents` there. False if the
/// user cancels.
///
/// The webview will not save a download itself, so the file goes through a native save dialog. Async,
/// so it runs off the main thread, which the dialog needs free while this waits on it.
#[tauri::command]
async fn save_engine_file(app: tauri::AppHandle, name: String, contents: String) -> Result<bool, String> {
    let Some(path) = app.dialog().file().set_file_name(&name).add_filter("Engine", &["json"]).blocking_save_file()
    else {
        return Ok(false);
    };
    let path = path.into_path().map_err(|e| e.to_string())?;
    std::fs::write(&path, contents).map_err(|e| format!("could not write {}: {e}", path.display()))?;
    Ok(true)
}

fn main() {
    tauri::Builder::default()
        .plugin(tauri_plugin_dialog::init())
        .manage(State::default())
        .invoke_handler(tauri::generate_handler![audio_start, audio_command, audio_stop, save_engine_file])
        .run(tauri::generate_context!())
        .expect("the app failed to start");
}
