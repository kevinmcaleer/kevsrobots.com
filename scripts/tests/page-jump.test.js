// node --test scripts/tests/*.test.js  — the "On this page" jump keeps
// correcting until the target stays put (it used to creep and stop short).
const test = require('node:test');
const assert = require('node:assert/strict');
const path = require('node:path');

const { settle, FRAMES } = require(path.join(__dirname, '../../web/assets/js/page-jump.js'));

// Frames run when `flush` is called, like requestAnimationFrame.
function frames() {
  let queue = [];
  return {
    nextFrame: fn => queue.push(fn),
    flush(n = Infinity) {
      while (queue.length && n-- > 0) {
        const run = queue;
        queue = [];
        run.forEach(fn => fn());
      }
    },
  };
}

test('jumps again while the target is off, then stops correcting', () => {
  // The target moves 300px, then 120px, as pictures above it load.
  const offsets = [300, 120, 0, 0, 0];
  let at = 0;
  let corrections = 0;
  let done = false;
  const f = frames();
  settle({
    distance: () => offsets[Math.min(at, offsets.length - 1)],
    correct: () => { corrections++; at++; },
    nextFrame: f.nextFrame,
    done: () => { done = true; },
  });
  f.flush();
  assert.equal(corrections, 2);
  assert.ok(done);
});

test('a target that keeps moving gets FRAMES tries, not forever', () => {
  let corrections = 0;
  const f = frames();
  settle({ distance: () => 500, correct: () => corrections++, nextFrame: f.nextFrame });
  f.flush();
  assert.equal(corrections, FRAMES);
});

test('stopping (the reader scrolls) ends the corrections at once', () => {
  let corrections = 0;
  let done = 0;
  const f = frames();
  const stop = settle({ distance: () => 500, correct: () => corrections++, nextFrame: f.nextFrame, done: () => done++ });
  f.flush(3);
  stop();
  stop();
  f.flush();
  assert.equal(corrections, 3);
  assert.equal(done, 1);
});

test('a target the editor cannot place yet (no coordinates) is left alone', () => {
  let corrections = 0;
  const f = frames();
  settle({ distance: () => null, correct: () => corrections++, nextFrame: f.nextFrame });
  f.flush();
  assert.equal(corrections, 0);
});
