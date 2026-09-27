/**
 * The view's toolbar: the tools that build the exhaust, each an icon with a tip beside it on hover.
 *
 * The panel owns what each tool does and whether it is on; this only draws the buttons.
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
