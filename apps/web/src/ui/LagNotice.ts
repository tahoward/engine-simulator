/**
 * The pop-up shown while the audio thread cannot keep up with real time.
 *
 * Falling behind is not always crackle: on some browsers and devices the output goes silent, and
 * the app then looks as if it never started. So the notice says both, and offers the one change that
 * reliably helps, a lower sample rate, as a button.
 */

import { SAMPLE_RATES } from './Panel.js';

export class LagNotice {
  private readonly card: HTMLElement;
  private readonly text: HTMLElement;
  private readonly lowerBtn: HTMLButtonElement;
  /** The rate the button switches to, Hz; `null` when there is none lower. */
  private lower: number | null = null;
  /** Dismissed during this spell of lag; shown again only once the audio has caught up and fallen behind anew. */
  private dismissed = false;

  constructor(
    parent: HTMLElement,
    private readonly onSampleRate: (hz: number) => void,
  ) {
    this.card = document.createElement('div');
    this.card.className = 'lag-card hidden';
    this.card.setAttribute('role', 'alert');
    const title = document.createElement('h2');
    title.textContent = 'Audio can’t keep up';
    this.text = document.createElement('p');
    const buttons = document.createElement('div');
    buttons.className = 'row buttons';
    this.lowerBtn = document.createElement('button');
    this.lowerBtn.className = 'primary';
    this.lowerBtn.addEventListener('click', () => {
      if (this.lower !== null) this.onSampleRate(this.lower);
    });
    const dismissBtn = document.createElement('button');
    dismissBtn.textContent = 'Dismiss';
    dismissBtn.addEventListener('click', () => {
      this.dismissed = true;
      this.card.classList.add('hidden');
    });
    buttons.append(this.lowerBtn, dismissBtn);
    this.card.append(title, this.text, buttons);
    parent.appendChild(this.card);
  }

  /** Show or hide the notice, for audio running at `rate` Hz. */
  update(behind: boolean, rate: number): void {
    if (!behind) {
      this.dismissed = false;
      this.card.classList.add('hidden');
      return;
    }
    if (this.dismissed) return;
    const lower = SAMPLE_RATES.map(([hz]) => hz).filter((hz) => hz < rate);
    this.lower = lower.length > 0 ? Math.max(...lower) : null;
    const kHz = (hz: number) => `${hz / 1000} kHz`;
    this.text.textContent =
      this.lower !== null
        ? `This device can’t run the simulation in real time at ${kHz(rate)}, so the sound may ` +
          'crackle, stutter, or not play at all. A lower sample rate needs much less CPU.'
        : 'This device can’t run the simulation in real time even at the lowest sample rate, so the ' +
          'sound may crackle, stutter, or not play at all. Try an engine with fewer cylinders, or a ' +
          'coarser Solver resolution.';
    this.lowerBtn.classList.toggle('hidden', this.lower === null);
    if (this.lower !== null) this.lowerBtn.textContent = `Switch to ${kHz(this.lower)}`;
    this.card.classList.remove('hidden');
  }
}
