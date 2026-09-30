/**
 * Icon buttons, each with a tip beside it on hover: the view's toolbar of tools that build the
 * exhaust, and the rail that picks the panel's section.
 *
 * The panel owns what each button does and whether it is on; this only draws the buttons.
 */

const svg = (body: string): string =>
  `<svg viewBox="0 0 24 24" width="20" height="20" fill="none" stroke="currentColor" stroke-width="1.8" ` +
  `stroke-linecap="round" stroke-linejoin="round" aria-hidden="true">${body}</svg>`;

export const TOOL_ICONS = {
  /** A route of straights, with a point at each corner. */
  draw: svg(
    '<path d="M4 19 9 10l6 4 5-9"/><circle cx="4" cy="19" r="1.6"/><circle cx="9" cy="10" r="1.6"/>' +
      '<circle cx="15" cy="14" r="1.6"/><circle cx="20" cy="5" r="1.6"/>',
  ),
  /** A straight tube, and a plus. */
  placePipe: svg('<rect x="2.5" y="11" width="13" height="6" rx="1"/><path d="M13 13.5v1M19 3.5v7M15.5 7h7"/>'),
  /** An elbow. */
  bend: svg('<path d="M4 21v-6A11 11 0 0 1 15 4h6"/><path d="M9.5 21v-6a5.5 5.5 0 0 1 5.5-5.5h6"/>'),
  /** Four pipes coming together into one. */
  header: svg(
    '<path d="M3 4c6 0 7 8 12 8M3 9.3c6 0 7 2.7 12 2.7M3 14.7c6 0 7-2.7 12-2.7M3 20c6 0 7-8 12-8M15 12h6"/>',
  ),
  /** A turbine's scroll, and its outlet. */
  turbo: svg(
    '<path d="M12 12a1 1 0 0 1 2 0 2 2 0 0 1-4 0 3 3 0 0 1 6 0 4 4 0 0 1-8 0 5 5 0 0 1 10 0"/><path d="M13 7h8"/>',
  ),
};

/** The panel's sections, one icon each in the rail down its edge. */
export const SECTION_ICONS = {
  /** Play. */
  transport: svg('<path d="M8 5v14l11-7z"/>'),
  /** A dial and its needle. */
  operatingPoint: svg('<path d="M4 17a8 8 0 1 1 16 0"/><path d="m12 17 4-5"/><circle cx="12" cy="17" r="1.3"/>'),
  /** A flag. */
  launch: svg('<path d="M5 21V4h12l-2.5 4L17 12H5"/>'),
  /** A power curve on its axes. */
  dyno: svg('<path d="M4 4v16h16"/><path d="M7 16c3-1 4-8 7-8s3 4 5 5"/>'),
  /** Three bores in a row. */
  layout: svg('<rect x="2.5" y="7" width="5" height="10" rx="1"/><rect x="9.5" y="7" width="5" height="10" rx="1"/><rect x="16.5" y="7" width="5" height="10" rx="1"/>'),
  /** A piston on its rod. */
  geometry: svg('<rect x="7" y="3" width="10" height="7" rx="1"/><path d="M12 10v5"/><circle cx="12" cy="18" r="3"/>'),
  /** A poppet valve. */
  valves: svg('<path d="M12 3v12"/><path d="M5 20c2-3.5 4.5-5 7-5s5 1.5 7 5z"/>'),
  /** Air drawn in. */
  intake: svg('<path d="M3 8h10a3 3 0 1 0-3-3"/><path d="M3 12h15a3 3 0 1 1-3 3"/><path d="M3 16h7"/>'),
  /** A flame. */
  combustion: svg('<path d="M12 3c1 4 6 6 6 11a6 6 0 0 1-12 0c0-3 2-4 3-7 1 2 2 3 3 3 0-2-1-4 0-7z"/>'),
  /** Headphones. */
  listener: svg('<path d="M4 15v-3a8 8 0 0 1 16 0v3"/><rect x="3" y="14" width="4" height="6" rx="1.5"/><rect x="17" y="14" width="4" height="6" rx="1.5"/>'),
  /** An eye. */
  view: svg('<path d="M2 12s3.5-6 10-6 10 6 10 6-3.5 6-10 6S2 12 2 12z"/><circle cx="12" cy="12" r="2.5"/>'),
};

/** Add a tool's button to `parent`: `icon`, and a tip naming the tool, `name`, and saying what it does. */
export function toolButton(parent: HTMLElement, icon: string, name: string, tip: string): HTMLButtonElement {
  const button = document.createElement('button');
  button.className = 'tool-btn';
  button.setAttribute('aria-label', name);
  button.innerHTML = icon;
  const tipEl = document.createElement('span');
  tipEl.className = 'tool-tip';
  const head = document.createElement('strong');
  head.textContent = name;
  const body = document.createElement('span');
  body.textContent = tip;
  tipEl.append(head, body);
  button.append(tipEl);
  parent.append(button);
  return button;
}
