/**
 * Undo and redo over whole states: each recorded state a step, undone and redone in order, and a new
 * edit after an undo dropping what could have been redone.
 */

import { describe, expect, it } from 'vitest';

import { UndoHistory } from '../src/model/undoHistory.js';

describe('UndoHistory', () => {
  it('starts with nothing to undo or redo', () => {
    const h = new UndoHistory('a');
    expect([h.canUndo, h.canRedo]).toEqual([false, false]);
    expect(h.undo()).toBeNull();
    expect(h.redo()).toBeNull();
  });

  it('undoes and redoes each recorded state in order', () => {
    const h = new UndoHistory('a');
    h.record('b');
    h.record('c');
    expect(h.undo()).toBe('b');
    expect(h.undo()).toBe('a');
    expect(h.undo()).toBeNull();
    expect(h.redo()).toBe('b');
    expect(h.redo()).toBe('c');
    expect(h.redo()).toBeNull();
  });

  it('makes no step of the state already current', () => {
    const h = new UndoHistory('a');
    expect(h.record('a')).toBe(false);
    expect(h.canUndo).toBe(false);
    expect(h.record('b')).toBe(true);
    expect(h.record('b')).toBe(false);
    expect(h.undo()).toBe('a');
    expect(h.canUndo).toBe(false);
  });

  it('drops what was undone once a new edit is made', () => {
    const h = new UndoHistory('a');
    h.record('b');
    h.undo();
    h.record('c');
    expect(h.canRedo).toBe(false);
    expect(h.undo()).toBe('a');
  });

  it('takes a replaced state as current without making a step', () => {
    const h = new UndoHistory('a');
    h.record('b');
    h.undo();
    h.replace('a2');
    expect(h.record('a2')).toBe(false);
    expect(h.redo()).toBe('b');
    expect(h.undo()).toBe('a2');
  });

  it('keeps at most its limit of steps back', () => {
    const h = new UndoHistory('0', 3);
    for (let i = 1; i <= 5; i++) h.record(String(i));
    expect([h.undo(), h.undo(), h.undo(), h.undo()]).toEqual(['4', '3', '2', null]);
  });
});
