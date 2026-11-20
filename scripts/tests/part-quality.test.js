// node --test scripts/tests/*.test.js  — how complete a catalog part is, and better matches.
const test = require('node:test');
const assert = require('node:assert/strict');
const path = require('node:path');

const Q = require(path.join(__dirname, '../../web/assets/js/part-quality.js'));

const towerPro = { slug: 'tower-pro-sg90', name: 'Tower Pro SG90 Micro Servo', has_mesh: true, pin_count: 3, curated_photo: true, status: 'verified', usage_count: 12 };
const homeMade = { slug: 'sg90-servo', name: 'SG90 Servo', has_mesh: false, pin_count: 0, curated_photo: true, status: 'draft', usage_count: 1 };
const mg90 = { slug: 'mg90s', name: 'MG90S Metal Gear Servo', has_mesh: true, pin_count: 3, curated_photo: true, status: 'verified' };
const clone = { slug: 'sg-90-clone', name: 'SG-90 9g servo', has_mesh: false, pin_count: 3, curated_photo: false, status: 'draft' };
const disputed = { slug: 'sg90-bad', name: 'SG90', has_mesh: true, pin_count: 3, curated_photo: true, status: 'disputed' };
const catalog = [homeMade, towerPro, mg90, clone, disputed];

test('quality counts the 3D model, pins and curated picture', () => {
  assert.deepEqual(Q.quality(towerPro), { model: true, pins: true, photo: true, verified: true, score: 3, complete: true, missing: [] });
  const h = Q.quality(homeMade);
  assert.equal(h.score, 1);
  assert.deepEqual(h.missing, ['3D model', 'pins']);
  assert.equal(Q.quality(null).score, 0);
  assert.equal(Q.describe(towerPro), 'a 3D model, 3 pins and a curated picture');
  assert.equal(Q.describe({}), 'no 3D model, pins or curated picture yet');
  assert.deepEqual(Q.marks(homeMade).map(m => m.on), [false, false, true]);
});

test('similar names share a model number', () => {
  assert.deepEqual(Q.words('SG-90 Micro Servo (x2)'), ['sg90', 'servo']);
  assert.ok(Q.similarity('SG90 Servo', 'Tower Pro SG90 Micro Servo') >= 0.6);
  assert.ok(Q.similarity('SG-90 9g servo', 'SG90 Servo') >= 0.6);
  assert.equal(Q.similarity('SG90 Servo', 'MG90S Metal Gear Servo'), 0);
  assert.equal(Q.similarity('', 'x'), 0);
  // No model numbers: the words in common.
  assert.equal(Q.similarity('Small power switch', 'Power switch'), 1);
  assert.ok(Q.similarity('Cable sheath', 'Power switch') < 0.6);
});

test('suggest points a row at a more complete part with a matching name', () => {
  const s = Q.suggest({ name: 'SG90 Servo', slug: 'sg90-servo' }, catalog);
  assert.equal(s.part.slug, 'tower-pro-sg90');
  assert.equal(s.quality.complete, true);
  // An unlinked row is matched too.
  assert.equal(Q.suggest({ name: 'sg90 servo', slug: null }, catalog).part.slug, 'tower-pro-sg90');
  // Already the best: nothing to suggest.
  assert.equal(Q.suggest({ name: 'Tower Pro SG90 Micro Servo', slug: 'tower-pro-sg90' }, catalog), null);
  // Nothing alike.
  assert.equal(Q.suggest({ name: 'Cable sheath', slug: null }, catalog), null);
  // Same marks, but verified beats a draft.
  const draftTwin = { ...towerPro, slug: 'twin', status: 'draft' };
  assert.equal(Q.suggest({ name: 'Tower Pro SG90', slug: 'twin' }, [draftTwin, towerPro]).part.slug, 'tower-pro-sg90');
});

test('searchQuery looks for the model number', () => {
  assert.equal(Q.searchQuery('SG90 Servo'), 'sg90');
  assert.equal(Q.searchQuery('BNO055 9-DOF IMU'), 'bno055');
  assert.equal(Q.searchQuery('Small power switch'), 'Small power switch');
});
