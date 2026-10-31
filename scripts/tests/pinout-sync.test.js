// node --test scripts/tests/*.test.js  — the pinout table's marker and stale check.
const test = require('node:test');
const assert = require('node:assert/strict');
const path = require('node:path');

const PinoutSync = require(path.join(__dirname, '../../web/assets/js/pinout-sync.js'));

const generated = [
  '# Robot — Pinouts',
  '',
  '| MCU Pin | Part | Part Pin | Net |',
  '| --- | --- | --- | --- |',
  '| GP0 | Servo | SIG | GP0 |',
  '| GND | Servo | GND | GND |',
  '',
].join('\n');
const block = PinoutSync.fromGenerated(generated);
const doc = body => '# Wiring\n\n![Wiring diagram](https://projects.kevsrobots.com/api/projects/3/wiring.svg?v=1)\n\n' + body + '\n\n# Code\n\nSome code.\n';

test('fromGenerated drops the heading and keeps the table', () => {
  assert.ok(block.startsWith('| MCU Pin |'));
  assert.ok(!block.includes('Pinouts'));
  assert.equal(PinoutSync.fromGenerated('# X — Pinouts\n\n_No connections in this project yet._\n'), '');
});

test('a fresh, unedited table is neither stale nor edited', () => {
  const md = doc(PinoutSync.withMarker(block));
  const found = PinoutSync.find(md);
  assert.equal(found.hash, PinoutSync.hash(block));
  assert.equal(found.block, block);
  assert.equal(md.slice(found.from, found.to), PinoutSync.withMarker(block));
  assert.deepEqual(PinoutSync.compare(found, block), { stale: false, edited: false });
});

test('re-spacing the table is not an edit', () => {
  const spaced = block.replace('| GP0 | Servo |', '|   GP0   |  Servo  |').replace('| --- | --- | --- | --- |', '|-----|-----|-----|-----|');
  const found = PinoutSync.find(doc(PinoutSync.marker(block) + '\n' + spaced));
  assert.deepEqual(PinoutSync.compare(found, block), { stale: false, edited: false });
});

test('a changed diagram makes the table stale; editing it shows as edited', () => {
  const newer = block.replace('GP0 | Servo | SIG | GP0', 'GP2 | Servo | SIG | GP2');
  const edited = block.replace('| SIG |', '| Signal (orange) |');
  assert.deepEqual(PinoutSync.compare(PinoutSync.find(doc(PinoutSync.withMarker(block))), newer), { stale: true, edited: false });
  const found = PinoutSync.find(doc(PinoutSync.marker(block) + '\n' + edited));
  assert.deepEqual(PinoutSync.compare(found, newer), { stale: true, edited: true });
  assert.deepEqual(PinoutSync.compare(found, block), { stale: false, edited: true });
});

test('an older table without a marker is found by its header', () => {
  const found = PinoutSync.find(doc(block));
  assert.equal(found.hash, null);
  assert.equal(found.block, block);
  assert.deepEqual(PinoutSync.compare(found, block), { stale: false, edited: null });
  assert.equal(PinoutSync.compare(found, block.replace('GP0', 'GP1')).stale, true);
});

test('the Other connections table is part of the block', () => {
  const both = block + '\n\n## Other connections\n\n| From | To | Net |\n| --- | --- | --- |\n| LED.A | R1.1 | N1 |';
  const md = doc(PinoutSync.withMarker(both));
  const found = PinoutSync.find(md);
  assert.equal(found.block, both);
  assert.ok(md.slice(found.to).startsWith('\n\n# Code'));
});

test('a marker in a code block, or with no table under it, is ignored', () => {
  assert.equal(PinoutSync.find('```\n' + PinoutSync.withMarker(block) + '\n```\n'), null);
  assert.equal(PinoutSync.find(PinoutSync.marker(block) + '\n\nText.\n'), null);
});

test('escaped pipes stay in their cell', () => {
  assert.deepEqual(PinoutSync.cells('| a \\| b | c |'), ['a \\| b', 'c']);
});
