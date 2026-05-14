/**
 * Circuit Component Definitions for the KevBot Circuit Editor
 * Issue #104 — Simple diagram tool for kevsrobots.com projects
 *
 * Research notes:
 * - Mermaid.js supports flowcharts, sequence, state, gantt — but NOT circuit diagrams
 * - KiCad uses .kicad_sch (S-expression text), Fritzing uses XML (.fzz),
 *   CircuitJS uses proprietary text format — none are web-native
 * - SVG-based approach chosen: components are SVG groups with defined pins,
 *   placed on a grid-snapped canvas. JSON storage for serialization.
 * - Each component defines its SVG paths, pin positions, default label,
 *   and editable properties.
 *
 * Adding a new component:
 *   1. Add an entry to CIRCUIT_COMPONENTS below
 *   2. Provide svg (drawn centered on 0,0), pins array, and default props
 *   3. The editor picks it up automatically in the palette
 */

const CIRCUIT_COMPONENTS = {
  resistor: {
    name: 'Resistor',
    category: 'passive',
    icon: 'R',
    width: 80,
    height: 30,
    pins: [
      { id: 'a', x: -40, y: 0, label: '1' },
      { id: 'b', x: 40, y: 0, label: '2' },
    ],
    defaultProps: { resistance: '10kΩ' },
    svg: function (props) {
      return `
        <line x1="-40" y1="0" x2="-25" y2="0" stroke="currentColor" stroke-width="2"/>
        <polyline points="-25,0 -20,-10 -10,10 0,-10 10,10 20,-10 25,0"
          fill="none" stroke="currentColor" stroke-width="2"/>
        <line x1="25" y1="0" x2="40" y2="0" stroke="currentColor" stroke-width="2"/>
        <text x="0" y="-16" text-anchor="middle" font-size="10" fill="currentColor"
          class="comp-label">${props.resistance || '10kΩ'}</text>
      `;
    },
  },

  capacitor: {
    name: 'Capacitor',
    category: 'passive',
    icon: 'C',
    width: 60,
    height: 30,
    pins: [
      { id: 'a', x: -30, y: 0, label: '+' },
      { id: 'b', x: 30, y: 0, label: '-' },
    ],
    defaultProps: { capacitance: '100µF' },
    svg: function (props) {
      return `
        <line x1="-30" y1="0" x2="-5" y2="0" stroke="currentColor" stroke-width="2"/>
        <line x1="-5" y1="-12" x2="-5" y2="12" stroke="currentColor" stroke-width="2"/>
        <path d="M5,-12 Q8,0 5,12" fill="none" stroke="currentColor" stroke-width="2"/>
        <line x1="5" y1="0" x2="30" y2="0" stroke="currentColor" stroke-width="2"/>
        <text x="0" y="-18" text-anchor="middle" font-size="10" fill="currentColor"
          class="comp-label">${props.capacitance || '100µF'}</text>
      `;
    },
  },

  led: {
    name: 'LED',
    category: 'active',
    icon: 'LED',
    width: 60,
    height: 40,
    pins: [
      { id: 'anode', x: -30, y: 0, label: 'A' },
      { id: 'cathode', x: 30, y: 0, label: 'K' },
    ],
    defaultProps: { colour: 'Red', voltage: '2.0V' },
    svg: function (props) {
      return `
        <line x1="-30" y1="0" x2="-10" y2="0" stroke="currentColor" stroke-width="2"/>
        <polygon points="-10,-10 -10,10 8,0" fill="none" stroke="currentColor" stroke-width="2"/>
        <line x1="8" y1="-10" x2="8" y2="10" stroke="currentColor" stroke-width="2"/>
        <line x1="8" y1="0" x2="30" y2="0" stroke="currentColor" stroke-width="2"/>
        <line x1="2" y1="-14" x2="8" y2="-20" stroke="currentColor" stroke-width="1.5"/>
        <line x1="5" y1="-19" x2="8" y2="-20" stroke="currentColor" stroke-width="1.5"/>
        <line x1="8" y1="-20" x2="7" y2="-17" stroke="currentColor" stroke-width="1.5"/>
        <line x1="7" y1="-12" x2="13" y2="-18" stroke="currentColor" stroke-width="1.5"/>
        <line x1="10" y1="-17" x2="13" y2="-18" stroke="currentColor" stroke-width="1.5"/>
        <line x1="13" y1="-18" x2="12" y2="-15" stroke="currentColor" stroke-width="1.5"/>
        <text x="0" y="22" text-anchor="middle" font-size="10" fill="currentColor"
          class="comp-label">${props.colour || 'Red'}</text>
      `;
    },
  },

  battery: {
    name: 'Battery',
    category: 'power',
    icon: 'B',
    width: 60,
    height: 40,
    pins: [
      { id: 'pos', x: -30, y: 0, label: '+' },
      { id: 'neg', x: 30, y: 0, label: '-' },
    ],
    defaultProps: { voltage: '3.3V' },
    svg: function (props) {
      return `
        <line x1="-30" y1="0" x2="-8" y2="0" stroke="currentColor" stroke-width="2"/>
        <line x1="-8" y1="-14" x2="-8" y2="14" stroke="currentColor" stroke-width="2"/>
        <line x1="-2" y1="-8" x2="-2" y2="8" stroke="currentColor" stroke-width="2"/>
        <line x1="4" y1="-14" x2="4" y2="14" stroke="currentColor" stroke-width="2"/>
        <line x1="10" y1="-8" x2="10" y2="8" stroke="currentColor" stroke-width="2"/>
        <line x1="10" y1="0" x2="30" y2="0" stroke="currentColor" stroke-width="2"/>
        <text x="-14" y="-18" text-anchor="middle" font-size="9" fill="currentColor">+</text>
        <text x="16" y="-18" text-anchor="middle" font-size="9" fill="currentColor">-</text>
        <text x="0" y="24" text-anchor="middle" font-size="10" fill="currentColor"
          class="comp-label">${props.voltage || '3.3V'}</text>
      `;
    },
  },

  switch_spst: {
    name: 'Switch',
    category: 'passive',
    icon: 'SW',
    width: 60,
    height: 30,
    pins: [
      { id: 'a', x: -30, y: 0, label: '1' },
      { id: 'b', x: 30, y: 0, label: '2' },
    ],
    defaultProps: { label: 'S1' },
    svg: function (props) {
      return `
        <line x1="-30" y1="0" x2="-12" y2="0" stroke="currentColor" stroke-width="2"/>
        <circle cx="-12" cy="0" r="3" fill="currentColor"/>
        <line x1="-12" y1="0" x2="12" y2="-12" stroke="currentColor" stroke-width="2"/>
        <circle cx="12" cy="0" r="3" fill="currentColor"/>
        <line x1="12" y1="0" x2="30" y2="0" stroke="currentColor" stroke-width="2"/>
        <text x="0" y="-20" text-anchor="middle" font-size="10" fill="currentColor"
          class="comp-label">${props.label || 'S1'}</text>
      `;
    },
  },

  ground: {
    name: 'Ground',
    category: 'power',
    icon: 'GND',
    width: 30,
    height: 40,
    pins: [
      { id: 'a', x: 0, y: -20, label: '' },
    ],
    defaultProps: {},
    svg: function () {
      return `
        <line x1="0" y1="-20" x2="0" y2="0" stroke="currentColor" stroke-width="2"/>
        <line x1="-14" y1="0" x2="14" y2="0" stroke="currentColor" stroke-width="2"/>
        <line x1="-9" y1="5" x2="9" y2="5" stroke="currentColor" stroke-width="2"/>
        <line x1="-4" y1="10" x2="4" y2="10" stroke="currentColor" stroke-width="2"/>
      `;
    },
  },

  motor: {
    name: 'Motor',
    category: 'active',
    icon: 'M',
    width: 60,
    height: 50,
    pins: [
      { id: 'a', x: -30, y: 0, label: '+' },
      { id: 'b', x: 30, y: 0, label: '-' },
    ],
    defaultProps: { label: 'M1' },
    svg: function (props) {
      return `
        <line x1="-30" y1="0" x2="-18" y2="0" stroke="currentColor" stroke-width="2"/>
        <circle cx="0" cy="0" r="18" fill="none" stroke="currentColor" stroke-width="2"/>
        <text x="0" y="5" text-anchor="middle" font-size="14" font-weight="bold"
          fill="currentColor">M</text>
        <line x1="18" y1="0" x2="30" y2="0" stroke="currentColor" stroke-width="2"/>
        <text x="0" y="-24" text-anchor="middle" font-size="10" fill="currentColor"
          class="comp-label">${props.label || 'M1'}</text>
      `;
    },
  },

  ic_chip: {
    name: 'IC Chip',
    category: 'active',
    icon: 'IC',
    width: 80,
    height: 80,
    pins: [
      { id: 'p1', x: -40, y: -20, label: '1' },
      { id: 'p2', x: -40, y: 0, label: '2' },
      { id: 'p3', x: -40, y: 20, label: '3' },
      { id: 'p4', x: 40, y: -20, label: '4' },
      { id: 'p5', x: 40, y: 0, label: '5' },
      { id: 'p6', x: 40, y: 20, label: '6' },
    ],
    defaultProps: { label: 'U1', part: 'ATmega328' },
    svg: function (props) {
      return `
        <rect x="-25" y="-30" width="50" height="60" fill="none" stroke="currentColor" stroke-width="2" rx="2"/>
        <line x1="-40" y1="-20" x2="-25" y2="-20" stroke="currentColor" stroke-width="2"/>
        <line x1="-40" y1="0" x2="-25" y2="0" stroke="currentColor" stroke-width="2"/>
        <line x1="-40" y1="20" x2="-25" y2="20" stroke="currentColor" stroke-width="2"/>
        <line x1="25" y1="-20" x2="40" y2="-20" stroke="currentColor" stroke-width="2"/>
        <line x1="25" y1="0" x2="40" y2="0" stroke="currentColor" stroke-width="2"/>
        <line x1="25" y1="20" x2="40" y2="20" stroke="currentColor" stroke-width="2"/>
        <path d="M-10,-30 A10,10 0 0,1 10,-30" fill="none" stroke="currentColor" stroke-width="1.5"/>
        <text x="0" y="5" text-anchor="middle" font-size="9" fill="currentColor"
          class="comp-label">${props.part || 'IC'}</text>
        <text x="0" y="-36" text-anchor="middle" font-size="10" fill="currentColor"
          class="comp-label">${props.label || 'U1'}</text>
      `;
    },
  },

  pico: {
    name: 'Pico',
    category: 'boards',
    icon: 'Pi',
    width: 100,
    height: 120,
    pins: [
      { id: 'gp0', x: -50, y: -40, label: 'GP0' },
      { id: 'gp1', x: -50, y: -20, label: 'GP1' },
      { id: 'gp2', x: -50, y: 0, label: 'GP2' },
      { id: 'gnd_l', x: -50, y: 20, label: 'GND' },
      { id: '3v3', x: -50, y: 40, label: '3V3' },
      { id: 'gp16', x: 50, y: -40, label: 'GP16' },
      { id: 'gp17', x: 50, y: -20, label: 'GP17' },
      { id: 'gp18', x: 50, y: 0, label: 'GP18' },
      { id: 'gnd_r', x: 50, y: 20, label: 'GND' },
      { id: 'vsys', x: 50, y: 40, label: 'VSYS' },
    ],
    defaultProps: { label: 'Pico' },
    svg: function (props) {
      return `
        <rect x="-35" y="-50" width="70" height="100" fill="none" stroke="currentColor" stroke-width="2" rx="4"/>
        <rect x="-12" y="-50" width="24" height="10" fill="none" stroke="currentColor" stroke-width="1.5" rx="2"/>
        <line x1="-50" y1="-40" x2="-35" y2="-40" stroke="currentColor" stroke-width="2"/>
        <line x1="-50" y1="-20" x2="-35" y2="-20" stroke="currentColor" stroke-width="2"/>
        <line x1="-50" y1="0" x2="-35" y2="0" stroke="currentColor" stroke-width="2"/>
        <line x1="-50" y1="20" x2="-35" y2="20" stroke="currentColor" stroke-width="2"/>
        <line x1="-50" y1="40" x2="-35" y2="40" stroke="currentColor" stroke-width="2"/>
        <line x1="35" y1="-40" x2="50" y2="-40" stroke="currentColor" stroke-width="2"/>
        <line x1="35" y1="-20" x2="50" y2="-20" stroke="currentColor" stroke-width="2"/>
        <line x1="35" y1="0" x2="50" y2="0" stroke="currentColor" stroke-width="2"/>
        <line x1="35" y1="20" x2="50" y2="20" stroke="currentColor" stroke-width="2"/>
        <line x1="35" y1="40" x2="50" y2="40" stroke="currentColor" stroke-width="2"/>
        <text x="0" y="5" text-anchor="middle" font-size="11" font-weight="bold"
          fill="currentColor">${props.label || 'Pico'}</text>
      `;
    },
  },

  wire_node: {
    name: 'Junction',
    category: 'wiring',
    icon: '•',
    width: 10,
    height: 10,
    pins: [
      { id: 'a', x: 0, y: 0, label: '' },
    ],
    defaultProps: {},
    svg: function () {
      return `<circle cx="0" cy="0" r="4" fill="currentColor"/>`;
    },
  },
};

/* Category display order and labels */
const COMPONENT_CATEGORIES = [
  { id: 'passive', label: 'Passive' },
  { id: 'active', label: 'Active' },
  { id: 'power', label: 'Power' },
  { id: 'boards', label: 'Boards' },
  { id: 'wiring', label: 'Wiring' },
];

/* Expose globally */
if (typeof window !== 'undefined') {
  window.CIRCUIT_COMPONENTS = CIRCUIT_COMPONENTS;
  window.COMPONENT_CATEGORIES = COMPONENT_CATEGORIES;
}
