// node --test scripts/tests/*.test.js  — the wiring workspace's Bill of Materials helpers.
const test = require('node:test');
const assert = require('node:assert/strict');
const path = require('node:path');

const W = require(path.join(__dirname, '../../web/assets/js/wiring-bom.js'));

const pins = n => [{ edge: 'left', pins: Array.from({ length: n }, (_, i) => ({ name: 'P' + i, type: 'io' })) }];
const pico = { id: 'pico', name: 'Pico', family: 'Microcontroller', headers: pins(4) };
const led = { id: 'led', name: 'LED', headers: pins(2) };
const motor = { id: 'tt-motor', name: 'TT Motor', headers: [] };

test('placeholder ids come from the row name', () => {
  assert.equal(W.placeholderId('M3 Screws (x4)'), 'row-m3-screws-x4');
  assert.equal(W.placeholderId('Café Résumé'), 'row-cafe-resume');
  assert.equal(W.placeholderId('***'), 'row-part');
});

test('a placeholder part has no pins and draws its name', () => {
  const p = W.placeholderPart('Big <red> button');
  assert.equal(p.id, 'row-big-red-button');
  assert.equal(W.hasPins(p), false);
  const svg = decodeURIComponent(p.imageData.replace(/^data:image\/svg\+xml;charset=utf-8,/, ''));
  assert.match(svg, /Big &lt;red&gt; button/);
  assert.match(svg, /Not linked/);
});

test('a catalog part with no pins and no photo gets the card', () => {
  const m = W.withFallbackImage(motor);
  assert.notEqual(m, motor);
  assert.match(decodeURIComponent(m.imageData), /No pins in the catalog yet/);
  assert.equal(W.withFallbackImage(led), led);
  const photo = { ...motor, imageData: 'https://x/y.png' };
  assert.equal(W.withFallbackImage(photo), photo);
});

test('entries: every row once, with its state', () => {
  const rows = [
    { name: 'Pico', slug: 'pico' },
    { name: 'LED', slug: 'led' },
    { name: 'LED again', slug: 'led' },
    { name: 'TT Motor', slug: 'tt-motor' },
    { name: 'Gaffer tape', slug: null },
    { name: 'Gone', slug: 'not-in-catalog' },
  ];
  const robot = { board: 'pico', parts: [{ id: 'led', lib: 'kevsrobots', part: 'led' }, { id: 'row-gaffer-tape', lib: W.BOM_LIB, part: 'row-gaffer-tape' }], connections: [] };
  const defs = new Map([['pico', pico], ['led', led], ['tt-motor', motor]]);
  const e = W.entries(rows, robot, defs);
  assert.deepEqual(e.map(x => x.name), ['Pico', 'LED', 'TT Motor', 'Gaffer tape', 'Gone']);
  assert.equal(e[0].isTheBoard, true);
  assert.equal(e[0].onCanvas, true);
  assert.equal(e[1].onCanvas, true);
  assert.equal(e[2].pins, false);
  assert.equal(e[2].onCanvas, false);
  assert.equal(e[3].linked, false);
  assert.equal(e[3].lib, W.BOM_LIB);
  assert.equal(e[3].onCanvas, true);
  assert.equal(e[4].def, null);
});

test('relink swaps a placeholder in place and drops its wires', () => {
  const robot = {
    parts: [
      { id: 'row-motor', lib: W.BOM_LIB, part: 'row-motor', label: 'Motor', x: 10, y: 20 },
      { id: 'led', lib: 'kevsrobots', part: 'led' },
    ],
    connections: [{ id: 'w1', from: 'led.P0#0', to: 'row-motor.X#0' }, { id: 'w2', from: 'led.P1#1', to: 'board.GND#3' }],
  };
  const next = W.relink(robot, W.BOM_LIB, 'row-motor', 'kevsrobots', 'tt-motor', 'TT Motor');
  assert.deepEqual(next.parts[0], { id: 'row-motor', lib: 'kevsrobots', part: 'tt-motor', label: 'TT Motor', x: 10, y: 20 });
  assert.equal(next.parts[1], robot.parts[1]);
  assert.deepEqual(next.connections.map(c => c.id), ['w2']);
  assert.equal(W.relink(robot, W.BOM_LIB, 'row-nothing', 'kevsrobots', 'x'), robot);
});

test('linkedSince finds placeholders whose row is linked now', () => {
  const robot = { parts: [
    { id: 'a', lib: W.BOM_LIB, part: 'row-tt-motor' },
    { id: 'b', lib: W.BOM_LIB, part: 'row-tt-motor' },
    { id: 'c', lib: W.BOM_LIB, part: 'row-tape' },
  ], connections: [] };
  const rows = [{ name: 'TT motor', slug: 'tt-motor' }, { name: 'Tape', slug: null }];
  assert.deepEqual(W.linkedSince(robot, rows), [{ from: 'row-tt-motor', slug: 'tt-motor', name: 'TT motor' }]);
});

test('a pin-less part takes its photo’s shape', () => {
  const holder = { id: 'holder', name: '18650 holder', headers: [], imageData: 'x.jpg' };
  const wide = W.fitToPhoto(holder, 800, 400);
  assert.equal(wide.aspect, 2);
  assert.equal(wide.dimensions, undefined);
  // A real width is kept, the height follows the photo.
  const sized = W.fitToPhoto(Object.assign({}, holder, { dimensions: { width: 77, height: 77 } }), 800, 400);
  assert.deepEqual(sized.dimensions, { width: 77, height: 38.5 });
  assert.equal(sized.aspect, 2);
  // No size read, or pins placed already: unchanged.
  assert.equal(W.fitToPhoto(holder, 0, 0), holder);
  const led = { id: 'led', headers: pins(2), aspect: 0.6 };
  assert.equal(W.fitToPhoto(led, 800, 400), led);
});
