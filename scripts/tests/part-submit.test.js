// node --test scripts/tests/*.test.js  — the "New part" dialog's pure helpers.
const test = require('node:test');
const assert = require('node:assert/strict');
const path = require('node:path');

const PartSubmit = require(path.join(__dirname, '../../web/assets/js/part-submit.js'));

test('payload sends only what was filled in, the shop link with its price', () => {
  assert.deepEqual(PartSubmit.payload({ name: '  SG90 servo ', shopUrl: '', price: '', tags: '' }), { name: 'SG90 servo', links: [], tags: [] });
  assert.deepEqual(PartSubmit.payload({
    name: 'UBEC', shopUrl: 'https://www.example.com/ubec', price: '£6.50', currency: 'EUR',
    description: '5V regulator', category: 'Power', datasheetUrl: 'https://example.com/d.pdf',
    manufacturer: ' HENGE ', mpn: '', voltage: '7-25V', package: '', tags: 'power, bec ,',
  }), {
    name: 'UBEC',
    links: [{ url: 'https://www.example.com/ubec', kind: 'shop', name: 'example.com', unit_cost: 6.5, currency_code: 'EUR' }],
    tags: ['power', 'bec'],
    description_md: '5V regulator', category: 'Power', manufacturer: 'HENGE', voltage: '7-25V',
    datasheet_url: 'https://example.com/d.pdf',
  });
  // A link without a price sends no currency.
  assert.deepEqual(PartSubmit.payload({ name: 'Pico', shopUrl: 'https://s.example/pico', currency: 'GBP' }).links[0].currency_code, null);
});

test('payload explains what is wrong', () => {
  assert.throws(() => PartSubmit.payload({ name: ' ' }), /name/);
  assert.throws(() => PartSubmit.payload({ name: 'Pico', shopUrl: 'example.com' }), /shop link/);
  assert.throws(() => PartSubmit.payload({ name: 'Pico', datasheetUrl: 'ftp://x' }), /datasheet/);
});

test('imageLinks finds pictures in a drop or paste from another page', () => {
  assert.deepEqual(PartSubmit.imageLinks('https://a.example/x.JPG\nhttps://a.example/page.html https://a.example/y.webp?w=200'),
    ['https://a.example/x.JPG', 'https://a.example/y.webp?w=200']);
  assert.deepEqual(PartSubmit.imageLinks(''), []);
});

test('errorText reads the API detail', () => {
  assert.equal(PartSubmit.errorText({ detail: 'Too many' }, 429), 'Too many');
  assert.equal(PartSubmit.errorText({ detail: [{ msg: 'bad name' }] }, 422), 'bad name');
  assert.match(PartSubmit.errorText({}, 401), /Sign in/);
});
