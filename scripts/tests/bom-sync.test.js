// node --test scripts/tests/*.test.js  — the Bill of Materials table parser (issue #264).
const test = require('node:test');
const assert = require('node:assert/strict');
const path = require('node:path');

globalThis.window = globalThis;
require(path.join(__dirname, '../../web/assets/js/project-sections.js'));
const BomSync = require(path.join(__dirname, '../../web/assets/js/bom-sync.js'));

const doc = table => '# Overview\n\nA robot.\n\n# Bill of Materials\n\n' + table + '\n\n# Wiring\n\n| From | To |\n| --- | --- |\n| a | b |\n';

test('reads parts, quantities, links and prices', () => {
  const out = BomSync.parse(doc([
    '| Part | Qty | Price | Total | Notes |',
    '| --- | --- | --- | --- | --- |',
    '| [Raspberry Pi Pico](/parts/view.html?slug=pico) | 1 | £4.00 | £4.00 | brain |',
    '| **SG90** servo | 2 | 2.50 EUR | 5.00 EUR |  |',
    '| [Wheels](https://example.com/wheels) | 4 | $1,200.50 |  |  |',
    '| Jumper wires | lots | ~ |  |  |',
    '| **Total** |  |  | **£9.00** |  |',
  ].join('\n')));
  assert.equal(out.hasPrice, true);
  assert.deepEqual(out.rows, [
    { name: 'Raspberry Pi Pico', quantity: 1, part_slug: 'pico', unit_cost: 4, currency_code: 'GBP' },
    { name: 'SG90 servo', quantity: 2, part_slug: null, unit_cost: 2.5, currency_code: 'EUR' },
    { name: 'Wheels', quantity: 4, part_slug: null, unit_cost: 1200.5, currency_code: 'USD' },
    { name: 'Jumper wires', quantity: 1, part_slug: null, unit_cost: null, currency_code: null },
  ]);
});

test('header variants and column order', () => {
  const out = BomSync.parse(doc([
    '| Quantity | Component | Unit cost |',
    '| :--- | --- | ---: |',
    '| 3 | [Motor \\| driver](https://www.kevsrobots.com/parts/view.html?slug=L298N) | CAD 7 |',
  ].join('\n')));
  assert.deepEqual(out.rows, [{ name: 'Motor | driver', quantity: 3, part_slug: 'l298n', unit_cost: 7, currency_code: 'CAD' }]);
});

test('missing columns', () => {
  const out = BomSync.parse(doc('| Thing | Notes |\n| --- | --- |\n| Battery | AA |\n|  | blank |'));
  assert.equal(out.hasPrice, false);
  // No Part column: the first; no Qty: one each; no Price: no unit_cost at all.
  assert.deepEqual(out.rows, [{ name: 'Battery', quantity: 1, part_slug: null }]);
});

test('no Bill of Materials table', () => {
  assert.equal(BomSync.parse('# Overview\n\nText.\n'), null);
  assert.equal(BomSync.parse('# Bill of Materials\n\nJust a list:\n- Pico\n'), null);
  // An empty table still counts: its rows are gone.
  assert.deepEqual(BomSync.parse(doc('| Part | Qty |\n| --- | --- |')).rows, []);
});

test('a table from stored rows', () => {
  const md = BomSync.tableFromRows([
    { name: 'Pico [W]', quantity: 2, part_slug: 'pico-w', unit_cost: 6, currency_code: 'GBP' },
    { name: 'Odd|name', quantity: 1, part_slug: null, unit_cost: 3, currency_code: 'CHF' },
    { name: 'Tape', quantity: 1, part_slug: null, unit_cost: null, currency_code: null },
  ]);
  const back = BomSync.parse(md);
  assert.deepEqual(back.rows, [
    { name: 'Pico [W]', quantity: 2, part_slug: 'pico-w', unit_cost: 6, currency_code: 'GBP' },
    { name: 'Odd|name', quantity: 1, part_slug: null, unit_cost: 3, currency_code: 'CHF' },
    { name: 'Tape', quantity: 1, part_slug: null, unit_cost: null, currency_code: null },
  ]);
});
