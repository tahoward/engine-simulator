/**
 * Undo and redo, as whole states rather than a record of each edit.
 *
 * Every edit in the app ends in the configuration as it then stands, which is already what is saved to
 * the link, so a snapshot of it undoes any edit the same way, however many pipes, junctions and turbos
 * the edit touched. States are kept as their JSON, which is also what tells two of them apart.
 */
export class UndoHistory {
  private readonly past: string[] = [];
  private readonly future: string[] = [];

  /** `current`: the state the app starts in. At most `limit` steps are kept back from it. */
  constructor(
    private current: string,
    private readonly limit = 100,
  ) {}

  get canUndo(): boolean {
    return this.past.length > 0;
  }

  get canRedo(): boolean {
    return this.future.length > 0;
  }

  /**
   * The state after an edit: a step to undo, unless it is the state already current. A new step
   * drops whatever had been undone, which can no longer be redone. Whether it made a step.
   */
  record(state: string): boolean {
    if (state === this.current) return false;
    this.past.push(this.current);
    if (this.past.length > this.limit) this.past.shift();
    this.current = state;
    this.future.length = 0;
    return true;
  }

  /**
   * Put `state` in place of the current one without making a step: for the state an undo or redo
   * comes out as once it is loaded, which can differ a little from the one stored.
   */
  replace(state: string): void {
    this.current = state;
  }

  /** The state before the current one, now current, or `null` with nothing to undo. */
  undo(): string | null {
    const state = this.past.pop();
    if (state === undefined) return null;
    this.future.push(this.current);
    this.current = state;
    return state;
  }

  /** The state last undone, now current again, or `null` with nothing to redo. */
  redo(): string | null {
    const state = this.future.pop();
    if (state === undefined) return null;
    this.past.push(this.current);
    this.current = state;
    return state;
  }
}
