# Deployment

The site is published to GitHub Pages by
[`.github/workflows/deploy.yml`](https://github.com/tahoward/engine-simulator/blob/main/.github/workflows/deploy.yml).
It runs on every push to `main`. You can also start it by hand with `workflow_dispatch`.

## Layout

**These docs are the root of the site. The simulator lives under `app/`.**

```
https://tahoward.github.io/engine-simulator/       → these docs
https://tahoward.github.io/engine-simulator/app/   → the simulator
```

One Pages deployment serves both. The workflow builds each one separately, then puts them together
into one upload:

1. **Build the docs.** `mkdocs build --strict` writes to `site/`.
2. **Build the app.** `npm run build` checks types and writes to `dist/`.
3. **Put them together.** `dist/` is moved to `site/app/`, and `site/` is uploaded to Pages.

Neither build reads the other's files. They only meet in step 3.

## The app's base path

The app needs no special settings for Pages. There is one Vite config, and its `base` is `'./'`.
That makes every URL in the build relative:

- `index.html` loads `./assets/…`.
- The [AudioWorklet](glossary.md#audioworklet) code is loaded with `new URL(…, import.meta.url)`, relative to the file that
  loads it.

So the same `dist/` works at `/`, at `/engine-simulator/app/`, or anywhere else. You can rename the
repository or move the app without rebuilding.

!!! warning "Keep the base relative"

    An absolute `base` would tie the build to one path. The workflow's `mv` destination would then
    have to match it. If they didn't match, the page would load, but every asset would fail with a
    404, including the audio worklet.

The page and the worklet talk with [`postMessage`, not `SharedArrayBuffer`](glossary.md#postmessage-and-sharedarraybuffer). This is so that no
cross-origin-isolation headers are needed, because GitHub Pages can't set them.

## Building locally

To build the same thing CI publishes:

```bash
# Docs to site/
mkdocs build --strict

# App to dist/
npm run build

# Put them together
mv dist site/app

# Serve the result
python3 -m http.server -d site 8080
```

The docs are then at <http://localhost:8080/> and the simulator at <http://localhost:8080/app/>.
The paths are relative, so the app works there just as it will on Pages. One catch: the docs'
**Launch** button links straight to the live app, so it takes you away from your local copy.

## Pages configuration

Set the repository's Pages source to **GitHub Actions**, not a branch
(**Settings → Pages → Build and deployment → Source**).

The workflow uses `actions/configure-pages`, `actions/upload-pages-artifact` and
`actions/deploy-pages`. It has `pages: write` and `id-token: write` permissions. It also uses a
`pages` concurrency group, so a new run waits for the current one instead of cancelling it.

!!! note "Don't use `mkdocs gh-deploy`"

    `mkdocs gh-deploy` pushes to a `gh-pages` branch, which is a *different* Pages source. It
    would clash with the Actions deployment and overwrite the app. Only use the workflow.

## What the workflow does not run

The deploy doesn't run the tests. Each test renders seconds of audio, so the suite takes a few
minutes. The tests are there to protect the physics, not the publishing. Run `npm test` yourself
before pushing. The build does check types, so a type error still stops a deploy.

## Cache and pinning

- Node is pinned to 22, with npm caching, via `actions/setup-node`. Vite 8 and Vitest 5 need at
  least 20.19.
- Python is pinned to 3.12, with pip caching, via `actions/setup-python`.
- The docs tools are pinned to minor versions in
  [`requirements-docs.txt`](https://github.com/tahoward/engine-simulator/blob/main/requirements-docs.txt).
  So a CI run months from now builds the same site you checked locally.
- The built Wasm kernel is committed as `src/audio/worklet/kernelWasm.ts`, so CI doesn't need
  AssemblyScript.

## Build outputs are not committed

`dist/` and `site/` are both in `.gitignore`. Nothing built is checked in, except the kernel above.
Only the workflow produces what gets published.
