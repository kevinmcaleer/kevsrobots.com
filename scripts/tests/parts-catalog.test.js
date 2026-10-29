// node --test scripts/tests/*.test.js  — the parts catalog's filters (issue #265).
const test = require('node:test');
const assert = require('node:assert/strict');
const path = require('node:path');

const C = require(path.join(__dirname, '../../web/assets/js/parts-catalog.js'));

const parts = [
  { slug: 'pico', name: 'Raspberry Pi Pico', category: 'Microcontroller', family: 'RP2040', manufacturer: 'Raspberry Pi', tags: ['rp2040', 'raspberry pi', 'usb'], status: 'verified', has_price: true },
  { slug: 'pico-w', name: 'Pico W', category: 'Microcontroller', family: 'RP2040', manufacturer: 'Raspberry Pi', tags: ['rp2040', 'wifi'], status: 'draft', has_price: false },
  { slug: 'esp32', name: 'ESP32 DevKit', category: 'Microcontroller', family: 'ESP32', manufacturer: 'Espressif', tags: ['wifi', 'usb'], status: 'disputed', has_price: true },
  { slug: 'sg90', name: 'SG90 Micro Servo', category: 'Motor', family: null, manufacturer: '', tags: ['servo'], status: 'draft', has_price: false },
  { slug: 'tape', name: 'Tape', category: null, tags: [], status: 'under_review', has_price: false },
];
const sel = () => C.emptySelection();

test('OR within a facet, AND across, AND with the query', () => {
  const s = sel();
  s.families.add('RP2040');
  s.families.add('ESP32');
  assert.deepEqual(C.applyFilters(parts, s, '').map(p => p.slug), ['pico', 'pico-w', 'esp32']);
  s.tags.add('wifi');
  assert.deepEqual(C.applyFilters(parts, s, '').map(p => p.slug), ['pico-w', 'esp32']);
  assert.deepEqual(C.applyFilters(parts, s, 'espressif').map(p => p.slug), ['esp32']);
  s.priced = true;
  assert.deepEqual(C.applyFilters(parts, s, '').map(p => p.slug), ['esp32']);
});

test('a facet counts against the other filters, not itself', () => {
  const s = sel();
  s.manufacturers.add('Espressif');
  const f = C.facets(parts, s, '');
  const m = Object.fromEntries(f.manufacturers.map(v => [v.value, v.count]));
  assert.deepEqual(m, { 'Raspberry Pi': 2, Espressif: 1, '—': 2 });
  const fam = Object.fromEntries(f.families.map(v => [v.value, v.count]));
  assert.deepEqual(fam, { ESP32: 1, RP2040: 0, '—': 0 });
  assert.equal(f.priced, 1);
});

test('tags restating a family or maker go, and one-part tags go last', () => {
  const f = C.facets(parts, sel(), '');
  const tags = f.tags.map(t => t.value + (t.rare ? '*' : ''));
  assert.deepEqual(tags, ['usb', 'wifi', 'servo*']);
});

test('status: unknown values count as draft', () => {
  const f = C.facets(parts, sel(), '');
  assert.deepEqual(f.statuses.map(v => [v.value, v.count]), [['verified', 1], ['draft', 3], ['disputed', 1]]);
  const s = sel();
  s.statuses.add('draft');
  assert.deepEqual(C.applyFilters(parts, s, '').map(p => p.slug), ['pico-w', 'sg90', 'tape']);
});

test('shelves: biggest first, uncategorised last, and ?category= by slug', () => {
  assert.deepEqual(C.shelves(parts).map(s => s.category + ':' + s.parts.length), ['Microcontroller:3', 'Motor:1', 'Other parts:1']);
  const s = sel();
  s.category = 'microcontroller';
  assert.equal(C.applyFilters(parts, s, '').length, 3);
});
