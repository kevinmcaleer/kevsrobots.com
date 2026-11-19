// node --test scripts/tests/*.test.js  — live project pages (#590): what is
// wired to each pin, wire names, and where code and wiring disagree.
const test = require('node:test');
const assert = require('node:assert/strict');
const path = require('node:path');

const L = require(path.join(__dirname, '../../web/assets/js/project-live.js'));

const robot = {
  board: 'pico2w',
  parts: [
    { id: 'servo1', lib: 'kev', part: 'sg90', label: 'Servo 1' },
    { id: 'dist', lib: 'kev', part: 'hcsr04', label: 'Range finder' },
    { id: 'buzz', lib: 'kev', part: 'buzzer' },
  ],
  connections: [
    { id: 'w1', from: 'board.GP15', to: 'servo1.SIG' },
    { id: 'w2', from: 'servo1.GND', to: 'board.GND' },
    { id: 'w3', from: 'board.GP2', to: 'dist.TRIG' },
    { id: 'w4', from: 'dist.TRIG', to: 'buzz.IN' },
  ],
};

test('reads what each board pin reaches, through joined wires', () => {
  const wired = L.boardPins(robot);
  assert.deepEqual(wired.GP15, { parts: ['Servo 1'], ends: ['Servo 1 SIG'], wires: ['w1'] });
  assert.deepEqual(wired.GP2.parts, ['Range finder', 'buzzer']);
  assert.deepEqual(wired.GP2.wires, ['w3', 'w4']);
  assert.equal(wired.GP4, undefined);
  assert.deepEqual(L.boardPins(null), {});
  assert.deepEqual(L.boardPins({ parts: [], connections: [{ id: 'x', from: 'junk', to: '' }] }), {});
});

test('a wire on the carrier the board sits in is a wire on the board pin', () => {
  const wired = L.boardPins({
    boardMountedOn: 'base',
    parts: [{ id: 'base', part: 'explorer' }, { id: 'led', part: 'led', label: 'Red LED' }],
    connections: [{ id: 'w1', from: 'base.GP7', to: 'led.A' }],
  });
  assert.deepEqual(wired.GP7.parts, ['Red LED']);
});

test('names a wire board end first', () => {
  assert.equal(L.wireLabel(robot, robot.connections[0]), 'GP15 to Servo 1 SIG');
  assert.equal(L.wireLabel(robot, robot.connections[1]), 'GND to Servo 1 GND');
  assert.equal(L.wireLabel(robot, robot.connections[3]), 'Range finder TRIG to buzzer IN');
});

test('pin names match however they are written', () => {
  assert.equal(L.pinKey('GP15'), L.pinKey('15'));
  assert.equal(L.pinKey('gpio15'), L.pinKey('IO15'));
  assert.notEqual(L.pinKey('GP15'), L.pinKey('GP1'));
  assert.equal(L.pinKey('led'), 'LED');
});

test('says which pins the code uses that nothing is wired to', () => {
  const wired = L.boardPins(robot);
  const pins = [{ id: 'GP15', onBoard: true }, { id: 'GP14', onBoard: true }, { id: 'LED', onBoard: true }, { id: 'GP99', onBoard: false }];
  assert.deepEqual(L.mismatches('main.py', pins, wired), ['main.py uses GP14, but nothing is wired to GP14.']);
  assert.deepEqual(L.unusedPins([{ id: 'GP15' }], wired), ['GP2']);
});

test('reads a GitHub repository link, with a branch and folder', () => {
  assert.deepEqual(L.parseRepo('https://github.com/kevinmcaleer/burgerbot'), { owner: 'kevinmcaleer', name: 'burgerbot', ref: null, dir: '' });
  assert.deepEqual(L.parseRepo('https://github.com/kev/bot.git'), { owner: 'kev', name: 'bot', ref: null, dir: '' });
  assert.deepEqual(L.parseRepo('https://github.com/kev/bot/tree/dev/src/robot/'), { owner: 'kev', name: 'bot', ref: 'dev', dir: 'src/robot' });
  assert.equal(L.parseRepo('https://gitlab.com/kev/bot'), null);
  assert.equal(L.parseRepo('https://github.com/kev/bot/tree/dev/../x'), null);
});

test('picks the Python files to show, the program a board runs first', () => {
  const listing = [
    { type: 'file', name: 'README.md', path: 'README.md' },
    { type: 'file', name: 'servo.py', path: 'servo.py' },
    { type: 'dir', name: 'lib.py', path: 'lib.py' },
    { type: 'file', name: 'main.py', path: 'main.py' },
    { type: 'file', name: 'arm.py', path: 'arm.py' },
  ];
  assert.deepEqual(L.pickFiles(listing, 5), ['main.py', 'arm.py', 'servo.py']);
  assert.deepEqual(L.pickFiles(listing, 2), ['main.py', 'arm.py']);
  assert.deepEqual(L.pickFiles({ message: 'rate limited' }, 5), []);
});

test('knows a project with a robot model among its files', () => {
  assert.equal(L.hasRobot([{ filename: 'base.stl' }, { filename: 'Arm.URDF' }]), true);
  assert.equal(L.hasRobot([{ filename: 'base.stl' }, { filename: 'urdf-notes.md' }]), false);
  assert.equal(L.hasRobot(null), false);
});
