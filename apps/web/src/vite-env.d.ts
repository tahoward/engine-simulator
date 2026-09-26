/// <reference types="vite/client" />

interface ImportMetaEnv {
  /** `desktop` in the desktop app's build (`npm run build:desktop`), absent in the web app's. */
  readonly VITE_TARGET?: 'desktop';
}

interface ImportMeta {
  readonly env: ImportMetaEnv;
}
