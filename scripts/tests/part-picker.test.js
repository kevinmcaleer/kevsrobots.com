// node --test scripts/tests/*.test.js  — where /part offers only the catalog.
const test = require('node:test');
const assert = require('node:assert/strict');
const path = require('node:path');

globalThis.window = globalThis;
require(path.join(__dirname, '../../web/assets/js/project-sections.js'));
require(path.join(__dirname, '../../web/assets/js/project-slash-commands.js'));
const { inBomTable } = window.ProjectSlashCommands;

const doc = [
  '# Bill of Materials', //    0
  '',                    //    1
  '| Part | Qty |',       //    2
  '| --- | --- |',       //    3
  '| Servo | 2 |',       //    4
  '| Pico | 1 |',        //    5
  '',                    //    6
  '# Wiring',            //    7
  '',                    //    8
  '| From | To |',       //    9
  '| --- | --- |',       //   10
  '| a | b |',           //   11
].join('\n');

test('the Bill of Materials table, header to last row', () => {
  assert.deepEqual([0, 1, 2, 3, 4, 5, 6].map(l => inBomTable(doc, l)), [false, false, true, true, true, true, false]);
});

test('other tables and documents without a BOM', () => {
  assert.equal(inBomTable(doc, 11), false);
  assert.equal(inBomTable('# Wiring\n\n| a | b |\n| --- | --- |\n| 1 | 2 |', 4), false);
});

test('a table with only its header', () => {
  const d = '# Bill of Materials\n\n| Part | Qty |\n| --- | --- |\n\nText';
  assert.deepEqual([2, 3, 4].map(l => inBomTable(d, l)), [true, true, false]);
});
