// Jump the page to a heading in the project editor, or to a section of the
// page, from the "On this page" nav, and make it stay there.
//
// Two things stopped the nav's links short (a few lines' worth towards the
// target, then nothing):
//
// - Bootstrap 5 turns on `scroll-behavior: smooth` for the whole page, so
//   CodeMirror's own scrolling (it sets the window's scrollTop, reads it
//   back, and corrects) becomes an animation that each correction
//   restarts from where it has got to.
// - The editor guesses the height of text it hasn't drawn. As a smooth
//   scroll passes through it, it draws that text and keeps the lines in
//   view still by moving the scroll position, which cancels the smooth
//   scroll part way.
//
// So the jump is instant (smooth scrolling is off while it runs), and for
// a few frames afterwards it checks the target is still where it should be
// and jumps again if pictures or tables above it changed size. Scrolling
// with the mouse, a finger or the keyboard stops the checking.
//
// `settle` has no DOM (tests: node --test scripts/tests/*.test.js).
(function (root) {
  const FRAMES = 30;      // about half a second of checks
  const TOLERANCE = 2;    // px

  // Call `correct` while `distance()` is more than TOLERANCE px from 0, for
  // up to `frames` frames; then `done`. Returns a function that stops early.
  function settle({ distance, correct, frames = FRAMES, nextFrame, done }) {
    let left = frames;
    let stopped = false;
    const stop = () => {
      if (stopped) return;
      stopped = true;
      if (done) done();
    };
    const tick = () => {
      if (stopped) return;
      const d = distance();
      if (d !== null && Math.abs(d) > TOLERANCE) correct();
      if (--left <= 0) stop();
      else nextFrame(tick);
    };
    nextFrame(tick);
    return stop;
  }

  const USER_SCROLL = ['wheel', 'touchstart', 'keydown', 'mousedown'];
  let current = null;

  // Instant scrolling (no CSS smooth) until `settle` is done, and stop if
  // the reader scrolls themselves.
  function run(jump, distance) {
    if (current) current();
    const html = document.documentElement;
    const before = html.style.scrollBehavior;
    html.style.scrollBehavior = 'auto';
    jump();
    const stopOnInput = () => stop();
    const stop = settle({
      distance,
      correct: jump,
      nextFrame: fn => requestAnimationFrame(fn),
      done() {
        html.style.scrollBehavior = before;
        USER_SCROLL.forEach(ev => window.removeEventListener(ev, stopOnInput, true));
        if (current === stop) current = null;
      },
    });
    USER_SCROLL.forEach(ev => window.addEventListener(ev, stopOnInput, { capture: true, passive: true }));
    current = stop;
    return stop;
  }

  // An element's top `offset` px below the top of the window.
  function toElement(el, { offset = 80 } = {}) {
    if (!el) return null;
    const distance = () => el.getBoundingClientRect().top - offset;
    return run(() => window.scrollTo(0, window.scrollY + distance()), distance);
  }

  // Line `line` (0-based) of a MarkdownEditor `offset` px below the top of
  // the window, with the cursor on it.
  function toEditorLine(editor, line, { offset = 80 } = {}) {
    const view = editor && editor.view;
    if (!view) return null;
    const doc = view.state.doc;
    const pos = doc.line(Math.max(1, Math.min(line + 1, doc.lines))).from;
    // view.constructor is CodeMirror's EditorView.
    const jump = () => view.dispatch({
      effects: view.constructor.scrollIntoView(pos, { y: 'start', yMargin: offset }),
    });
    const distance = () => {
      const at = view.coordsAtPos(pos);
      return at ? at.top - offset : null;
    };
    view.dispatch({ selection: { anchor: pos } });
    const stop = run(jump, distance);
    view.focus();
    return stop;
  }

  const api = { settle, toElement, toEditorLine, FRAMES, TOLERANCE };
  if (typeof module === 'object' && module.exports) module.exports = api;
  else root.PageJump = api;
})(typeof window !== 'undefined' ? window : globalThis);
