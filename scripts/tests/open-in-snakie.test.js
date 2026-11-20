// node --test scripts/tests/*.test.js  — "Open in Snakie" on lesson code
// (#581): the links it builds, and that Snakie can read the code back out.
const test = require('node:test');
const assert = require('node:assert/strict');
const path = require('node:path');
const zlib = require('node:zlib');

const { fileName, links, isProgram, encodeCode, runPlan, whyNot, imports } = require(path.join(__dirname, '../../web/assets/js/open-in-snakie.js'));

// Snakie's decodeCode (src/renderer/src/code-embed/share.ts): base64url, raw inflate.
function decode(payload) {
  const b64 = payload.replace(/-/g, '+').replace(/_/g, '/');
  return zlib.inflateRawSync(Buffer.from(b64, 'base64')).toString('utf8');
}

test('names the file after the lesson, numbering later blocks', () => {
  assert.equal(fileName('/learn/micropython/06_our_first_program.html', 1), '06_our_first_program.py');
  assert.equal(fileName('/learn/micropython/06_our_first_program.html', 3), '06_our_first_program_3.py');
  assert.equal(fileName('/learn/micropython/', 1), 'micropython.py');
  assert.equal(fileName('/', 1), 'main.py');
  assert.equal(fileName('/learn/x/index.html', 1), 'main.py');
  assert.equal(fileName('/learn/a b/c%20d.html', 1), 'c_d.py');
});

test('the compressed code round-trips into Snakie, in the hash', async () => {
  const code = 'from machine import Pin\nled = Pin("LED", Pin.OUT)\n# café ✓\nled.toggle()';
  const payload = await encodeCode(code);
  assert.match(payload, /^[A-Za-z0-9_-]+$/);
  assert.equal(decode(payload), code);
  const l = links(code, 'blink.py', payload);
  assert.equal(l.web, 'https://app.snakie.org/?name=blink.py#src=' + payload);
  assert.equal(l.desktop, 'snakie://open?name=blink.py#src=' + payload);
});

test('a large program still encodes', async () => {
  const code = Array.from({ length: 5000 }, (_, i) => `print(${i})`).join('\n');
  assert.equal(decode(await encodeCode(code)), code);
});

test('without compression it sends the plain code', () => {
  const l = links('print("hi")', 'a.py', null);
  assert.equal(l.web, 'https://app.snakie.org/?name=a.py&code=print(%22hi%22)');
  assert.equal(new URL(l.web).searchParams.get('code'), 'print("hi")');
});

test('REPL sessions and empty blocks are not programs', () => {
  assert.equal(isProgram('print("hi")'), true);
  assert.equal(isProgram('\n\nimport time\n'), true);
  assert.equal(isProgram('>>> 1 + 1\n2'), false);
  assert.equal(isProgram('$ pip install x'), false);
  assert.equal(isProgram('   \n'), false);
});

test('reads the modules a program imports', () => {
  assert.deepEqual(imports('import time, machine as m  # two\nfrom machine import Pin\nimport os.path\nx = 1'), ['time', 'machine', 'machine', 'os']);
  assert.deepEqual(imports('print("import this")'), []);
});

test('Run here is offered only for code a browser can run', () => {
  assert.deepEqual(runPlan('from machine import Pin\nled = Pin("LED", Pin.OUT)\nled.toggle()'), { engine: 'micropython' });
  assert.deepEqual(runPlan('import numpy as np\nprint(np.arange(3))'), { engine: 'pyodide' });
  assert.equal(runPlan('name = input("Name? ")\nprint(name)'), null);
  assert.equal(runPlan('import network\nwlan = network.WLAN()'), null);
  assert.equal(runPlan('import numpy\nimport requests'), null);
  assert.equal(runPlan('>>> print(1)\n1'), null);
  assert.equal(runPlan(''), null);
});

test('play on code a browser can\'t run says why, instead of being switched off', () => {
  assert.equal(whyNot('print("hi")'), null);
  assert.match(whyNot('name = input("Name? ")'), /type/);
  assert.match(whyNot('import network\nwlan = network.WLAN()'), /network/);
  // whyNot() and runPlan() agree on every block.
  ['print(1)', 'import numpy', 'input()', 'import socket', '>>> 1', 'from machine import Pin'].forEach((code) => {
    assert.equal(whyNot(code) === null, runPlan(code) !== null, code);
  });
});
