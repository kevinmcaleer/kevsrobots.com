// node --test scripts/tests/*.test.js  — the suite switcher's app list and
// the account button's helpers (kevsrobots.com#581). The elements themselves
// are tested in Snakie, which bundles a copy (test/krSuite.test.ts).
const test = require('node:test');
const assert = require('node:assert/strict');
const path = require('node:path');
const { pathToFileURL } = require('node:url');

const load = () => import(pathToFileURL(path.join(__dirname, '../../web/assets/suite/kr-suite.js')).href);

test('lists the four apps in switcher order, each on https', async () => {
  const { KR_APPS } = await load();
  assert.deepEqual(KR_APPS.map(a => a.id), ['learn', 'projects', 'snakie', 'bettercad']);
  for (const a of KR_APPS) assert.match(a.url, /^https:\/\//);
});

test('initials come from the first and last name, else the username', async () => {
  const { initials } = await load();
  assert.equal(initials({ name: 'Kevin McAleer' }), 'KM');
  assert.equal(initials({ name: 'Kevin Mc Aleer' }), 'KA');
  assert.equal(initials({ name: 'Kev' }), 'KE');
  assert.equal(initials({ name: '', username: 'kev' }), 'KE');
  assert.equal(initials(null), '');
});

test('shows the name, falling back to the username', async () => {
  const { displayName } = await load();
  assert.equal(displayName({ name: ' Kev ', username: 'kev' }), 'Kev');
  assert.equal(displayName({ username: 'kev' }), 'kev');
});

test('keeps the card on screen', async () => {
  const { placeCard } = await load();
  const viewport = { width: 400, height: 600 };
  const card = { width: 320, height: 300 };
  // Under the button, lined up with its left edge.
  assert.deepEqual(placeCard({ left: 10, right: 42, top: 8, bottom: 40 }, card, viewport), { left: 10, top: 46, width: 320 });
  // Near the right edge it lines up with the button's right edge instead.
  assert.equal(placeCard({ left: 360, right: 392, top: 8, bottom: 40 }, card, viewport).left, 72);
  // Near the bottom it opens upwards.
  assert.equal(placeCard({ left: 10, right: 42, top: 500, bottom: 532 }, card, viewport).top, 194);
  // Never wider than the screen.
  assert.equal(placeCard({ left: 0, right: 32, top: 0, bottom: 32 }, card, { width: 300, height: 600 }).width, 284);
});
