// node --test scripts/tests/*.test.js — dragging a heading in the editor's
// "On this page" moves its whole section.
const test = require('node:test');
const assert = require('node:assert/strict');
const path = require('node:path');

const { move } = require(path.join(__dirname, '../../web/assets/js/project-section-move.js'));

function headings(md) {
  const out = [];
  let fence = false;
  md.split('\n').forEach((line, i) => {
    if (/^```/.test(line)) { fence = !fence; return; }
    const m = !fence && line.match(/^(#{1,6})\s+(.+)$/);
    if (m) out.push({ level: m[1].length, text: m[2], line: i });
  });
  return out;
}
const line = (md, text) => headings(md).find(h => h.text === text).line;
const run = (md, from, to, after) => move(md, headings(md), line(md, from), line(md, to), after);

const DOC = [
  'Intro text',
  '',
  '# Overview',
  'About it',
  '',
  '## Parts',
  'list',
  '',
  '# Wiring',
  '```',
  '# not a heading',
  '```',
  '',
  '# Code',
  'print(1)',
].join('\n');

test('moves a section, with its sub-headings, before another', () => {
  const r = run(DOC, 'Overview', 'Code', false);
  assert.equal(r.doc, [
    'Intro text', '', '# Wiring', '```', '# not a heading', '```', '',
    '# Overview', 'About it', '', '## Parts', 'list', '', '# Code', 'print(1)',
  ].join('\n'));
  assert.equal(r.doc.split('\n')[r.line], '# Overview');
});

test('moves the last section to the top, keeping blank lines between', () => {
  const r = run(DOC, 'Code', 'Overview', false);
  assert.equal(r.doc, [
    'Intro text', '', '# Code', 'print(1)', '', '# Overview', 'About it', '', '## Parts', 'list', '',
    '# Wiring', '```', '# not a heading', '```',
  ].join('\n'));
  assert.equal(r.doc.split('\n')[r.line], '# Code');
});

test('moves a section to the end', () => {
  const r = run(DOC + '\n', 'Overview', 'Code', true);
  assert.equal(r.doc, [
    'Intro text', '', '# Wiring', '```', '# not a heading', '```', '',
    '# Code', 'print(1)', '', '# Overview', 'About it', '', '## Parts', 'list', '',
  ].join('\n'));
});

test('a sub-heading moves within or across its parents', () => {
  const r = run(DOC, 'Parts', 'Code', false);
  assert.ok(r.doc.includes('```\n\n## Parts\nlist\n\n# Code'));
  assert.ok(r.doc.startsWith('Intro text\n\n# Overview\nAbout it\n\n# Wiring'));
});

test('dropping where it already is does nothing', () => {
  assert.equal(run(DOC, 'Overview', 'Wiring', false), null);
  assert.equal(run(DOC, 'Wiring', 'Overview', true), null);
  assert.equal(run(DOC, 'Overview', 'Overview', true), null);
});
