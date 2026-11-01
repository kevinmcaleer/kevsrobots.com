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

test('adds a Total column and a total row', () => {
  const md = doc([
    '| Part | Qty | Price |',
    '| --- | --- | --- |',
    '| Pico | 1 | £4.00 |',
    '| Servo | 2 | £2.50 |',
  ].join('\n'));
  assert.equal(BomSync.withTotals(md), doc([
    '| Part | Qty | Price | Total |',
    '| --- | --- | --- | --- |',
    '| Pico | 1 | £4.00 | £4.00 |',
    '| Servo | 2 | £2.50 | £5.00 |',
    '| **Total** |  |  | **£9.00** |',
  ].join('\n')));
});

test('leaves a table whose totals are right as it is', () => {
  const md = doc([
    '| Part | Qty | Price | Total |',
    '| --- | --- | --- | --- |',
    '| Pico | 1 | £4.00 | £4.00 |',
    '| **Total** |  |  | **£4.00** |',
  ].join('\n'));
  assert.deepEqual(BomSync.totalsChanges(md), []);
  assert.equal(BomSync.withTotals(md), md);
});

test('a table without Qty and Price gets no totals', () => {
  const md = doc('| Part | Notes |\n| --- | --- |\n| Pico | brain |');
  assert.equal(BomSync.withTotals(md), md);
});

test('BOM, Parts and Parts list headings count as the Bill of Materials', () => {
  const table = '| Part | Qty | Price |\n| --- | --- | --- |\n| Cell | 2 | 5 |\n';
  ['## BOM', '# parts', '### Parts List'].forEach(h => {
    const out = BomSync.withTotals('# Intro\n\n' + h + '\n\n' + table);
    assert.match(out, /\| Part \| Qty \| Price \| Total \|/, h);
    assert.match(out, /\| \*\*Total\*\* \|  \|  \| \*\*10\.00\*\* \|/, h);
  });
});

test('the first Bill of Materials heading with a table counts', () => {
  const md = '# Bill of Materials\n\nSee below.\n\n# Assembly instructions\n\n## BOM\n\n' +
    '| | Part | link | Qty | Price |\n| --- | --- | --- | --- | --- |\n| Power | Cell | x | 2 | 5 |\n';
  const out = BomSync.parse(md);
  assert.deepEqual(out.rows.map(r => [r.name, r.quantity]), [['Cell', 2]]);
  assert.match(BomSync.withTotals(md), /\| \*\*Total\*\* \|  \|  \|  \|  \| \*\*10\.00\*\* \|/);
});

test('a currency in the Price header formats every price and total', () => {
  const md = doc([
    '| Part | Qty | Price (£) |',
    '| --- | --- | --- |',
    '| Pico | 1 | 4 |',
    '| Servo | 2 | $2.5 |',
  ].join('\n'));
  assert.equal(BomSync.withTotals(md), doc([
    '| Part | Qty | Price (£) | Total |',
    '| --- | --- | --- | --- |',
    '| Pico | 1 | £4.00 | £4.00 |',
    '| Servo | 2 | £2.50 | £5.00 |',
    '| **Total** |  |  | **£9.00** |',
  ].join('\n')));
});

test('yen has no decimals, and a code works in the header', () => {
  const md = doc('| Part | Qty | Price (JPY) |\n| --- | --- | --- |\n| Pico | 2 | 650 |');
  assert.match(BomSync.withTotals(md), /\| Pico \| 2 \| ¥650 \| ¥1300 \|\n\| \*\*Total\*\* \|  \|  \| \*\*¥1300\*\* \|/);
  const chf = doc('| Part | Qty | Unit price (CHF) |\n| --- | --- | --- |\n| Pico | 1 | 4 |');
  assert.match(BomSync.withTotals(chf), /\| Pico \| 1 \| 4\.00 CHF \| 4\.00 CHF \|/);
});

test('the price being typed is left alone until the caret moves on', () => {
  const md = doc('| Part | Qty | Price (€) | Total |\n| --- | --- | --- | --- |\n| Pico | 2 | 4 |  |');
  const line = md.split('\n').findIndex(l => l.startsWith('| Pico'));
  const changes = BomSync.totalsChanges(md, { typingLine: line });
  assert.equal(changes.find(c => c.line === line).text, '| Pico | 2 | 4 | €8.00 |');
  assert.equal(BomSync.totalsChanges(md).find(c => c.line === line).text, '| Pico | 2 | €4.00 | €8.00 |');
});

test('choosing a currency rewrites the header, prices and totals', () => {
  const rows = [
    ['Part', 'Qty', 'Unit price', 'Total', 'Notes'],
    ['Pico', '1', '£4', '£4.00', 'brain'],
    ['Wires', 'lots', '~', '', ''],
    ['**Total**', '', '', '**£4.00**', ''],
  ];
  assert.deepEqual(BomSync.withCurrency(rows, 'USD'), [
    ['Part', 'Qty', 'Unit price ($)', 'Total', 'Notes'],
    ['Pico', '1', '$4.00', '$4.00', 'brain'],
    ['Wires', 'lots', '~', '', ''],
    ['**Total**', '', '', '**$4.00**', ''],
  ]);
  const back = BomSync.withCurrency(BomSync.withCurrency(rows, 'USD'), null);
  assert.deepEqual(back[0], ['Part', 'Qty', 'Unit price', 'Total', 'Notes']);
  assert.deepEqual(back[1], ['Pico', '1', '4.00', '4.00', 'brain']);
});

test('the table currency comes from the header, else the first price', () => {
  assert.equal(BomSync.tableCurrency([['Part', 'Price (€)'], ['Pico', '£4']]), 'EUR');
  assert.equal(BomSync.tableCurrency([['Part', 'Price'], ['Pico', '4'], ['Servo', '£4']]), 'GBP');
  assert.equal(BomSync.tableCurrency([['Part', 'Price'], ['Pico', '4']]), null);
});

test('a bare price takes the header currency when synced', () => {
  const out = BomSync.parse(doc('| Part | Qty | Price (€) |\n| --- | --- | --- |\n| Pico | 1 | 4 |'));
  assert.deepEqual(out.rows[0], { name: 'Pico', quantity: 1, part_slug: null, unit_cost: 4, currency_code: 'EUR' });
});

test('linkRow links the row the wiring workspace names, keeping its wording', () => {
  const md = doc([
    '| Part | Qty | Notes |',
    '| --- | --- | --- |',
    '| [Pico](/parts/view.html?slug=pico) | 1 |  |',
    '| **SG90** servo | 2 |  |',
    '| Arm [left] | 1 | |',
    '| **Total** |  |  |',
  ].join('\n'));
  const lines = md.split('\n');
  const sg = BomSync.linkRow(md, 'sg90 SERVO', 'sg90');
  assert.equal(lines[sg.line], '| **SG90** servo | 2 |  |');
  assert.equal(sg.text, '| [**SG90** servo](/parts/view.html?slug=sg90) | 2 |  |');
  const arm = BomSync.linkRow(md, 'Arm [left]', 'arm');
  assert.equal(arm.text, '| [Arm \\[left\\]](/parts/view.html?slug=arm) | 1 | |');
  lines[arm.line] = arm.text;
  assert.equal(BomSync.parse(lines.join('\n')).rows.find(r => r.name === 'Arm [left]').part_slug, 'arm');
  assert.equal(BomSync.linkRow(md, 'Pico', 'other'), null, 'a row linked to a part is left alone');
  assert.equal(BomSync.linkRow(md, 'Total', 'x'), null);
  assert.equal(BomSync.linkRow(md, 'Motor', 'motor'), null);
});

test('linkRow moves a shop link on the name into the Notes column', () => {
  const md = doc([
    '| Part | Qty | Notes |',
    '| --- | --- | --- |',
    '| [Wheels](https://example.com/w) | 4 |  |',
    '| [Tyres](https://example.com/t) | 4 | grippy |',
  ].join('\n'));
  assert.equal(BomSync.linkRow(md, 'Wheels', 'wheel').text, '| [Wheels](/parts/view.html?slug=wheel) | 4 | [Buy](https://example.com/w) |');
  const tyre = BomSync.linkRow(md, 'tyres', 'tyre');
  assert.equal(tyre.text, '| [Tyres](/parts/view.html?slug=tyre) | 4 | grippy · [Buy](https://example.com/t) |');
  const lines = md.split('\n');
  lines[tyre.line] = tyre.text;
  assert.deepEqual(BomSync.parse(lines.join('\n')).rows[1], { name: 'Tyres', quantity: 4, part_slug: 'tyre' });
  const bare = doc('| Part | Qty |\n| --- | --- |\n| [Wheels](https://example.com/w) | 4 |');
  assert.equal(BomSync.linkRow(bare, 'Wheels', 'wheel'), null, 'no Notes column: the shop link is not dropped');
});

test('rowDetails reads a row for the New part dialog', () => {
  const md = doc([
    '| Part | Qty | Price (£) | Notes |',
    '| --- | --- | --- | --- |',
    '| [UBEC](https://shop.example.com/ubec) | 1 | £6.50 | 5V regulator |',
    '| SG90 servo | 2 | 2.50 EUR | small · [Buy](https://shop.example.com/sg90) |',
    '| Wire | 1 |  |  |',
    '| **Total** |  |  |  |',
  ].join('\n'));
  assert.deepEqual(BomSync.rowDetails(md, 'ubec'), { name: 'UBEC', shopUrl: 'https://shop.example.com/ubec', price: 6.5, currency: 'GBP', notes: '5V regulator' });
  assert.deepEqual(BomSync.rowDetails(md, 'SG90 servo'), { name: 'SG90 servo', shopUrl: 'https://shop.example.com/sg90', price: 2.5, currency: 'EUR', notes: 'small' });
  assert.deepEqual(BomSync.rowDetails(md, 'Wire'), { name: 'Wire', shopUrl: null, price: null, currency: 'GBP', notes: null });
  assert.equal(BomSync.rowDetails(md, 'Nope'), null);
});

test('linkRow with dropShopLink links a shop-linked row with no Notes column', () => {
  const md = doc(['| Part | Qty |', '| --- | --- |', '| [UBEC](https://shop.example.com/ubec) | 1 |'].join('\n'));
  assert.equal(BomSync.linkRow(md, 'UBEC', 'ubec'), null);
  const edit = BomSync.linkRow(md, 'UBEC', 'ubec', { dropShopLink: true });
  assert.equal(edit.text, '| [UBEC](/parts/view.html?slug=ubec) | 1 |');
});

// --- Catalog prices ---

const pricedDoc = (head = 'Price') => doc([
  '| Part | Qty | ' + head + ' | Total | Notes |',
  '| --- | --- | --- | --- | --- |',
  '| [Raspberry Pi Pico](/parts/view.html?slug=pico) | 1 |  |  |  |',
  '| [SG90](/parts/view.html?slug=sg90) | 2 | £3.00 | £6.00 | spare |',
  '| Wires | 1 | £1.00 | £1.00 |  |',
  '| **Total** |  |  | **£7.00** |  |',
].join('\n'));

test('linkedRows lists the rows that link a catalog part, with their prices', () => {
  const out = BomSync.linkedRows(pricedDoc('Price (£)'));
  assert.equal(out.hasPrice, true);
  assert.equal(out.currency, 'GBP');
  assert.deepEqual(out.rows.map(r => [r.slug, r.name, r.price, r.priceText]), [
    ['pico', 'Raspberry Pi Pico', null, ''],
    ['sg90', 'SG90', { amount: 3, code: 'GBP' }, '£3.00'],
  ]);
  assert.equal(BomSync.linkedRows('# Overview\n\nNo table.'), null);
});

test('catalogPrice prefers a shop in the table currency, then the cheapest', () => {
  const shops = [
    { name: 'Adafruit', unit_cost: 4, currency_code: 'USD', kind: 'shop' },
    { name: 'Pimoroni', unit_cost: 4.8, currency_code: 'GBP', kind: 'shop' },
    { name: 'The Pi Hut', unit_cost: 4.2, currency_code: 'GBP', kind: 'shop' },
    { name: 'Gone', unit_cost: 1, currency_code: 'GBP', kind: 'shop', unavailable_at: '2026-10-01' },
    { name: 'Broken', unit_cost: 1, currency_code: 'GBP', kind: 'shop', is_broken: true },
    { name: 'Datasheet', unit_cost: 0.5, currency_code: 'GBP', kind: 'datasheet' },
    { name: 'No price', unit_cost: null, currency_code: 'GBP' },
  ];
  assert.deepEqual(BomSync.catalogPrice(shops, 'GBP'), { amount: 4.2, code: 'GBP', shop: 'The Pi Hut' });
  assert.deepEqual(BomSync.catalogPrice(shops, ['EUR', 'GBP']), { amount: 4.2, code: 'GBP', shop: 'The Pi Hut' });
  // Neither: the first shop's currency, not a cheaper number in another.
  assert.deepEqual(BomSync.catalogPrice(shops, 'EUR'), { amount: 4, code: 'USD', shop: 'Adafruit' });
  assert.deepEqual(BomSync.catalogPrice(shops.slice(1), null), { amount: 4.2, code: 'GBP', shop: 'The Pi Hut' });
  assert.equal(BomSync.catalogPrice([], 'GBP'), null);
  assert.equal(BomSync.catalogPrice(shops.slice(3), 'GBP'), null);
});

test('setRowQty writes the Qty cell of the linked row', () => {
  const md = pricedDoc();
  const lines = md.split('\n');
  const sg = lines.findIndex(l => l.includes('slug=sg90'));
  assert.deepEqual(BomSync.setRowQty(md, 'sg90', 4), {
    line: sg, text: lines[sg].replace('| 2 |', '| 4 |'),
  });
  assert.equal(BomSync.setRowQty(md, 'nope', 4), null);
  assert.equal(BomSync.setRowQty(doc('| Part | Price |\n| --- | --- |\n| [Pico](/parts/view.html?slug=pico) | £4 |'), 'pico', 2), null);
  assert.equal(BomSync.setRowQty(doc('| Part | Quantity |\n| --- | --- |\n| [Pico](/parts/view.html?slug=pico) |  |'), 'pico', 2).text,
    '| [Pico](/parts/view.html?slug=pico) | 2 |');
});

test('setRowPrice writes the price cell and adds a note', () => {
  const md = pricedDoc();
  const lines = md.split('\n');
  const pico = lines.findIndex(l => l.includes('slug=pico'));
  assert.deepEqual(BomSync.setRowPrice(md, 'pico', '£4.20'), {
    line: pico, text: '| [Raspberry Pi Pico](/parts/view.html?slug=pico) | 1 | £4.20 |  |  |',
  });
  assert.deepEqual(BomSync.setRowPrice(md, 'pico', '£3.30', 'from $4.00'), {
    line: pico, text: '| [Raspberry Pi Pico](/parts/view.html?slug=pico) | 1 | £3.30 |  | from $4.00 |',
  });
  const sg = BomSync.setRowPrice(md, 'SG90', '£2.50', 'from $3.10');
  assert.equal(sg.text, '| [SG90](/parts/view.html?slug=sg90) | 2 | £2.50 | £6.00 | spare · from $3.10 |');
  assert.equal(BomSync.setRowPrice(md, 'nope', '£1'), null);
  // No closing pipe, no Notes column.
  const open = doc('| Part | Price\n| --- | ---\n| [Pico](/parts/view.html?slug=pico) |');
  assert.equal(BomSync.setRowPrice(open, 'pico', '£4.00', 'note'), null);
  const open2 = doc('| Part | Price\n| --- | ---\n| [Pico](/parts/view.html?slug=pico) | 1');
  assert.equal(BomSync.setRowPrice(open2, 'pico', '£4.00', 'note').text, '| [Pico](/parts/view.html?slug=pico) | £4.00 ');
  // No Price column.
  assert.equal(BomSync.setRowPrice(doc('| Part | Qty |\n| --- | --- |\n| [Pico](/parts/view.html?slug=pico) | 1 |'), 'pico', '£4'), null);
});
