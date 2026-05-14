/**
 * Circuit Diagram Editor — MVP for kevsrobots.com Projects Hub
 * Issue #104 — Simple drag-and-drop circuit diagram tool
 *
 * Depends on: circuit-components.js (loaded first)
 *
 * Architecture:
 *   - All rendering via SVG (no Canvas) for easy export
 *   - Components placed on a 20px grid
 *   - Wires connect between component pins
 *   - State stored as JSON: {components:[...], wires:[...]}
 *   - Export to SVG string or inline into markdown
 */

(function () {
  'use strict';

  const GRID = 20;

  /* ---- Helpers ---- */
  function snap(v) { return Math.round(v / GRID) * GRID; }
  function uid() { return '_' + Math.random().toString(36).slice(2, 9); }

  function svgNS(tag, attrs) {
    const el = document.createElementNS('http://www.w3.org/2000/svg', tag);
    if (attrs) Object.entries(attrs).forEach(([k, v]) => el.setAttribute(k, v));
    return el;
  }

  /* ---- CircuitEditor class ---- */
  class CircuitEditor {
    constructor() {
      this.components = [];   // {id, type, x, y, rotation, props}
      this.wires = [];        // {id, from:{component,pin}, to:{component,pin}}
      this.selected = null;   // component id or wire id
      this.selectedType = null; // 'component' | 'wire'
      this.wiringFrom = null; // {componentId, pinId, x, y} while drawing wire
      this.dragState = null;  // {componentId, offsetX, offsetY}
      this.el = null;
      this.svg = null;
      this.onInsert = null;   // callback(svgString, json) when user clicks Insert
    }

    /* Open the editor as a modal */
    open(options) {
      options = options || {};
      this.onInsert = options.onInsert || null;

      // Load existing diagram
      if (options.json) {
        this.components = JSON.parse(JSON.stringify(options.json.components || []));
        this.wires = JSON.parse(JSON.stringify(options.json.wires || []));
      } else {
        this.components = [];
        this.wires = [];
      }

      this._buildUI();
      document.body.appendChild(this.el);
      document.body.style.overflow = 'hidden';
      this._render();

      // Close on Escape
      this._keyHandler = (e) => {
        if (e.key === 'Escape') {
          if (this.wiringFrom) {
            this._cancelWiring();
          } else {
            this.close();
          }
        }
        if (e.key === 'Delete' || e.key === 'Backspace') {
          if (document.activeElement && document.activeElement.tagName === 'INPUT') return;
          this._deleteSelected();
        }
        if (e.key === 'r' || e.key === 'R') {
          if (document.activeElement && document.activeElement.tagName === 'INPUT') return;
          this._rotateSelected();
        }
      };
      document.addEventListener('keydown', this._keyHandler);
    }

    close() {
      if (this.el && this.el.parentNode) {
        this.el.parentNode.removeChild(this.el);
      }
      document.body.style.overflow = '';
      document.removeEventListener('keydown', this._keyHandler);
    }

    /* ---- Build the modal UI ---- */
    _buildUI() {
      const modal = document.createElement('div');
      modal.className = 'circuit-editor-modal';

      const wrap = document.createElement('div');
      wrap.className = 'circuit-editor-wrap';

      // Top bar
      const topbar = document.createElement('div');
      topbar.className = 'circuit-editor-topbar';
      topbar.innerHTML = `
        <span class="ce-title"><i class="fas fa-microchip"></i> Circuit Diagram Editor</span>
        <div>
          <button class="ce-btn-clear ce-btn-danger" title="Clear all"><i class="fas fa-trash-alt"></i> Clear</button>
          <button class="ce-btn-export" title="Export SVG"><i class="fas fa-download"></i> Export SVG</button>
          <button class="ce-btn-insert ce-btn-primary" title="Insert into project"><i class="fas fa-check"></i> Insert</button>
          <button class="ce-btn-close" title="Close"><i class="fas fa-times"></i></button>
        </div>
      `;
      wrap.appendChild(topbar);

      // Body: palette | canvas | props
      const body = document.createElement('div');
      body.className = 'circuit-editor-body';

      // Palette
      const palette = document.createElement('div');
      palette.className = 'circuit-palette';
      this._buildPalette(palette);
      body.appendChild(palette);

      // Canvas area
      const canvasArea = document.createElement('div');
      canvasArea.className = 'circuit-canvas-area';
      body.appendChild(canvasArea);

      // Props panel
      const props = document.createElement('div');
      props.className = 'circuit-props';
      props.innerHTML = '<p class="prop-hint">Select a component to edit its properties.</p>';
      body.appendChild(props);

      wrap.appendChild(body);

      // Status bar
      const statusbar = document.createElement('div');
      statusbar.className = 'circuit-statusbar';
      statusbar.innerHTML = '<span>Components: 0</span><span>Wires: 0</span><span>Tip: drag from palette, click pins to wire, R to rotate, Del to delete</span>';
      wrap.appendChild(statusbar);

      modal.appendChild(wrap);
      this.el = modal;
      this.canvasArea = canvasArea;
      this.propsPanel = props;
      this.statusbar = statusbar;

      // Events on top bar buttons
      topbar.querySelector('.ce-btn-close').addEventListener('click', () => this.close());
      topbar.querySelector('.ce-btn-clear').addEventListener('click', () => {
        if (this.components.length === 0 && this.wires.length === 0) return;
        if (confirm('Clear the entire diagram?')) {
          this.components = [];
          this.wires = [];
          this.selected = null;
          this._render();
          this._updateProps();
        }
      });
      topbar.querySelector('.ce-btn-export').addEventListener('click', () => this._exportSVG());
      topbar.querySelector('.ce-btn-insert').addEventListener('click', () => this._insertDiagram());

      // Prevent clicks on modal background from closing (only close button does)
      modal.addEventListener('click', (e) => {
        if (e.target === modal) this.close();
      });

      this._setupCanvas();
    }

    /* Build the component palette */
    _buildPalette(palette) {
      const cats = window.COMPONENT_CATEGORIES || [];
      const comps = window.CIRCUIT_COMPONENTS || {};

      cats.forEach(cat => {
        const items = Object.entries(comps).filter(([, c]) => c.category === cat.id);
        if (items.length === 0) return;

        const catLabel = document.createElement('div');
        catLabel.className = 'palette-category';
        catLabel.textContent = cat.label;
        palette.appendChild(catLabel);

        items.forEach(([key, comp]) => {
          const item = document.createElement('div');
          item.className = 'palette-item';
          item.draggable = true;
          item.dataset.componentType = key;
          item.innerHTML = `<span class="palette-icon">${comp.icon}</span><span>${comp.name}</span>`;

          item.addEventListener('dragstart', (e) => {
            e.dataTransfer.setData('text/plain', key);
            e.dataTransfer.effectAllowed = 'copy';
          });

          // Double-click to place in center
          item.addEventListener('dblclick', () => {
            const rect = this.canvasArea.getBoundingClientRect();
            this._addComponent(key, snap(rect.width / 2), snap(rect.height / 2));
          });

          palette.appendChild(item);
        });
      });
    }

    /* Set up SVG canvas */
    _setupCanvas() {
      const svg = svgNS('svg');
      svg.setAttribute('xmlns', 'http://www.w3.org/2000/svg');
      this.canvasArea.appendChild(svg);
      this.svg = svg;

      // Drop handler
      this.canvasArea.addEventListener('dragover', (e) => {
        e.preventDefault();
        e.dataTransfer.dropEffect = 'copy';
      });

      this.canvasArea.addEventListener('drop', (e) => {
        e.preventDefault();
        const type = e.dataTransfer.getData('text/plain');
        if (!type || !CIRCUIT_COMPONENTS[type]) return;
        const rect = this.canvasArea.getBoundingClientRect();
        const x = snap(e.clientX - rect.left);
        const y = snap(e.clientY - rect.top);
        this._addComponent(type, x, y);
      });

      // Click on canvas background to deselect
      svg.addEventListener('click', (e) => {
        if (e.target === svg || e.target.classList.contains('grid-bg')) {
          this.selected = null;
          this.selectedType = null;
          this._render();
          this._updateProps();
        }
      });

      // Mouse move for wiring preview
      svg.addEventListener('mousemove', (e) => {
        if (!this.wiringFrom) return;
        const rect = this.canvasArea.getBoundingClientRect();
        const mx = e.clientX - rect.left;
        const my = e.clientY - rect.top;
        const preview = this.svg.querySelector('.wire-preview');
        if (preview) {
          preview.setAttribute('x2', mx);
          preview.setAttribute('y2', my);
        }
      });
    }

    /* ---- Component management ---- */
    _addComponent(type, x, y) {
      const def = CIRCUIT_COMPONENTS[type];
      if (!def) return;
      const comp = {
        id: uid(),
        type: type,
        x: x,
        y: y,
        rotation: 0,
        props: JSON.parse(JSON.stringify(def.defaultProps)),
      };
      this.components.push(comp);
      this.selected = comp.id;
      this.selectedType = 'component';
      this._render();
      this._updateProps();
    }

    _deleteSelected() {
      if (!this.selected) return;
      if (this.selectedType === 'component') {
        // Remove component and its wires
        this.wires = this.wires.filter(w =>
          w.from.component !== this.selected && w.to.component !== this.selected
        );
        this.components = this.components.filter(c => c.id !== this.selected);
      } else if (this.selectedType === 'wire') {
        this.wires = this.wires.filter(w => w.id !== this.selected);
      }
      this.selected = null;
      this.selectedType = null;
      this._render();
      this._updateProps();
    }

    _rotateSelected() {
      if (!this.selected || this.selectedType !== 'component') return;
      const comp = this.components.find(c => c.id === this.selected);
      if (!comp) return;
      comp.rotation = ((comp.rotation || 0) + 90) % 360;
      this._render();
    }

    /* ---- Wiring ---- */
    _startWiring(componentId, pinId) {
      const comp = this.components.find(c => c.id === componentId);
      if (!comp) return;
      const def = CIRCUIT_COMPONENTS[comp.type];
      const pin = def.pins.find(p => p.id === pinId);
      if (!pin) return;

      const pos = this._getPinWorldPos(comp, pin);

      if (!this.wiringFrom) {
        // Start wiring
        this.wiringFrom = { componentId, pinId, x: pos.x, y: pos.y };

        // Add preview line
        const preview = svgNS('line', {
          x1: pos.x, y1: pos.y, x2: pos.x, y2: pos.y,
          stroke: '#4A90D9', 'stroke-width': 2, 'stroke-dasharray': '5 3',
          class: 'wire-preview',
        });
        this.svg.appendChild(preview);
      } else {
        // Finish wiring — don't connect a pin to itself
        if (this.wiringFrom.componentId === componentId && this.wiringFrom.pinId === pinId) {
          this._cancelWiring();
          return;
        }

        // Check for duplicate wire
        const dup = this.wires.find(w =>
          (w.from.component === this.wiringFrom.componentId && w.from.pin === this.wiringFrom.pinId &&
           w.to.component === componentId && w.to.pin === pinId) ||
          (w.to.component === this.wiringFrom.componentId && w.to.pin === this.wiringFrom.pinId &&
           w.from.component === componentId && w.from.pin === pinId)
        );

        if (!dup) {
          this.wires.push({
            id: uid(),
            from: { component: this.wiringFrom.componentId, pin: this.wiringFrom.pinId },
            to: { component: componentId, pin: pinId },
          });
        }

        this.wiringFrom = null;
        this._render();
      }
    }

    _cancelWiring() {
      this.wiringFrom = null;
      const preview = this.svg.querySelector('.wire-preview');
      if (preview) preview.remove();
    }

    _getPinWorldPos(comp, pin) {
      const rad = (comp.rotation || 0) * Math.PI / 180;
      const rx = pin.x * Math.cos(rad) - pin.y * Math.sin(rad);
      const ry = pin.x * Math.sin(rad) + pin.y * Math.cos(rad);
      return { x: comp.x + rx, y: comp.y + ry };
    }

    /* ---- Rendering ---- */
    _render() {
      const svg = this.svg;
      const w = this.canvasArea.clientWidth;
      const h = this.canvasArea.clientHeight;
      svg.setAttribute('viewBox', `0 0 ${w} ${h}`);
      svg.innerHTML = '';

      // Defs
      const defs = svgNS('defs');
      // Grid pattern
      const minorPattern = svgNS('pattern', {
        id: 'grid-minor', width: GRID, height: GRID,
        patternUnits: 'userSpaceOnUse',
      });
      minorPattern.appendChild(svgNS('line', {
        x1: 0, y1: 0, x2: GRID, y2: 0,
        class: 'circuit-grid-line',
      }));
      minorPattern.appendChild(svgNS('line', {
        x1: 0, y1: 0, x2: 0, y2: GRID,
        class: 'circuit-grid-line',
      }));
      defs.appendChild(minorPattern);

      const majorPattern = svgNS('pattern', {
        id: 'grid-major', width: GRID * 5, height: GRID * 5,
        patternUnits: 'userSpaceOnUse',
      });
      majorPattern.appendChild(svgNS('rect', {
        width: GRID * 5, height: GRID * 5, fill: 'url(#grid-minor)',
      }));
      majorPattern.appendChild(svgNS('line', {
        x1: 0, y1: 0, x2: GRID * 5, y2: 0,
        class: 'circuit-grid-line-major',
      }));
      majorPattern.appendChild(svgNS('line', {
        x1: 0, y1: 0, x2: 0, y2: GRID * 5,
        class: 'circuit-grid-line-major',
      }));
      defs.appendChild(majorPattern);
      svg.appendChild(defs);

      // Grid background
      const bg = svgNS('rect', {
        width: w, height: h, fill: 'url(#grid-major)', class: 'grid-bg',
      });
      svg.appendChild(bg);

      // Wires
      this.wires.forEach(wire => {
        this._renderWire(wire);
      });

      // Components
      this.components.forEach(comp => {
        this._renderComponent(comp);
      });

      this._updateStatus();
    }

    _renderComponent(comp) {
      const def = CIRCUIT_COMPONENTS[comp.type];
      if (!def) return;

      const g = svgNS('g', {
        transform: `translate(${comp.x}, ${comp.y}) rotate(${comp.rotation || 0})`,
        class: 'circuit-component' + (this.selected === comp.id ? ' selected' : ''),
        'data-id': comp.id,
      });

      // Selection box
      const pad = 6;
      g.appendChild(svgNS('rect', {
        x: -def.width / 2 - pad,
        y: -def.height / 2 - pad,
        width: def.width + pad * 2,
        height: def.height + pad * 2,
        class: 'selection-box',
      }));

      // Component SVG
      const inner = svgNS('g');
      inner.innerHTML = def.svg(comp.props || {});
      g.appendChild(inner);

      // Pins
      def.pins.forEach(pin => {
        const pg = svgNS('g', {
          class: 'circuit-pin' + (this.wiringFrom && this.wiringFrom.componentId === comp.id && this.wiringFrom.pinId === pin.id ? ' wiring' : ''),
          'data-component': comp.id,
          'data-pin': pin.id,
        });
        pg.appendChild(svgNS('circle', {
          cx: pin.x, cy: pin.y, r: 4,
        }));
        // Larger invisible hit target
        const hit = svgNS('circle', {
          cx: pin.x, cy: pin.y, r: 10, fill: 'transparent', stroke: 'none',
        });
        pg.appendChild(hit);

        pg.addEventListener('click', (e) => {
          e.stopPropagation();
          this._startWiring(comp.id, pin.id);
        });

        g.appendChild(pg);
      });

      // Drag handling
      let dragStarted = false;
      g.addEventListener('mousedown', (e) => {
        if (e.target.closest('.circuit-pin')) return;
        e.preventDefault();
        e.stopPropagation();

        this.selected = comp.id;
        this.selectedType = 'component';
        this._render();
        this._updateProps();

        const rect = this.canvasArea.getBoundingClientRect();
        const offsetX = e.clientX - rect.left - comp.x;
        const offsetY = e.clientY - rect.top - comp.y;
        dragStarted = false;

        const onMove = (ev) => {
          dragStarted = true;
          const nx = snap(ev.clientX - rect.left - offsetX);
          const ny = snap(ev.clientY - rect.top - offsetY);
          comp.x = Math.max(0, nx);
          comp.y = Math.max(0, ny);
          this._render();
          this._updateProps();
        };

        const onUp = () => {
          document.removeEventListener('mousemove', onMove);
          document.removeEventListener('mouseup', onUp);
        };

        document.addEventListener('mousemove', onMove);
        document.addEventListener('mouseup', onUp);
      });

      this.svg.appendChild(g);
    }

    _renderWire(wire) {
      const fromComp = this.components.find(c => c.id === wire.from.component);
      const toComp = this.components.find(c => c.id === wire.to.component);
      if (!fromComp || !toComp) return;

      const fromDef = CIRCUIT_COMPONENTS[fromComp.type];
      const toDef = CIRCUIT_COMPONENTS[toComp.type];
      const fromPin = fromDef.pins.find(p => p.id === wire.from.pin);
      const toPin = toDef.pins.find(p => p.id === wire.to.pin);
      if (!fromPin || !toPin) return;

      const p1 = this._getPinWorldPos(fromComp, fromPin);
      const p2 = this._getPinWorldPos(toComp, toPin);

      // Right-angle wire routing: horizontal then vertical
      const mid = svgNS('polyline', {
        points: `${p1.x},${p1.y} ${p2.x},${p1.y} ${p2.x},${p2.y}`,
        class: 'circuit-wire' + (this.selected === wire.id ? ' selected' : ''),
        'data-id': wire.id,
      });

      mid.addEventListener('click', (e) => {
        e.stopPropagation();
        this.selected = wire.id;
        this.selectedType = 'wire';
        this._render();
        this._updateProps();
      });

      this.svg.appendChild(mid);
    }

    /* ---- Properties panel ---- */
    _updateProps() {
      const panel = this.propsPanel;
      if (!this.selected) {
        panel.innerHTML = '<p class="prop-hint">Select a component to edit its properties.<br><br>Keyboard shortcuts:<br>R = rotate<br>Del = delete<br>Esc = cancel wire / close</p>';
        return;
      }

      if (this.selectedType === 'wire') {
        panel.innerHTML = '<h6>Wire</h6><p class="prop-hint">Press Delete to remove this wire.</p>';
        return;
      }

      const comp = this.components.find(c => c.id === this.selected);
      if (!comp) return;
      const def = CIRCUIT_COMPONENTS[comp.type];

      let html = `<h6>${def.name}</h6>`;

      // Position (read-only display)
      html += `<div class="prop-row"><label>Position</label><span style="font-size:0.78rem;color:#666">(${comp.x}, ${comp.y})</span></div>`;

      // Rotation
      html += `<div class="prop-row"><label>Rotation</label><button class="rotate-btn" data-action="rotate"><i class="fas fa-redo"></i> ${comp.rotation || 0}&deg;</button></div>`;

      // Editable props
      Object.entries(comp.props || {}).forEach(([key, val]) => {
        html += `<div class="prop-row"><label>${key}</label><input type="text" data-prop="${key}" value="${val}"></div>`;
      });

      html += '<p class="prop-hint">Press R to rotate, Del to delete</p>';
      panel.innerHTML = html;

      // Bind rotate button
      const rotBtn = panel.querySelector('[data-action="rotate"]');
      if (rotBtn) {
        rotBtn.addEventListener('click', () => {
          this._rotateSelected();
          this._updateProps();
        });
      }

      // Bind prop inputs
      panel.querySelectorAll('input[data-prop]').forEach(input => {
        input.addEventListener('change', () => {
          comp.props[input.dataset.prop] = input.value;
          this._render();
        });
        input.addEventListener('keydown', (e) => {
          if (e.key === 'Delete' || e.key === 'Backspace') e.stopPropagation();
        });
      });
    }

    _updateStatus() {
      if (!this.statusbar) return;
      this.statusbar.innerHTML = `
        <span>Components: ${this.components.length}</span>
        <span>Wires: ${this.wires.length}</span>
        <span>Tip: drag from palette, click pins to wire, R to rotate, Del to delete</span>
      `;
    }

    /* ---- Export ---- */
    _getExportSVG() {
      // Clone SVG, remove grid, remove interactive bits
      const clone = this.svg.cloneNode(true);

      // Remove grid background and patterns
      const gridBg = clone.querySelector('.grid-bg');
      if (gridBg) gridBg.remove();

      // Remove pin circles and selection boxes
      clone.querySelectorAll('.circuit-pin circle, .selection-box').forEach(el => el.remove());

      // Compute bounding box
      let minX = Infinity, minY = Infinity, maxX = -Infinity, maxY = -Infinity;
      this.components.forEach(comp => {
        const def = CIRCUIT_COMPONENTS[comp.type];
        const pad = 30;
        minX = Math.min(minX, comp.x - def.width / 2 - pad);
        minY = Math.min(minY, comp.y - def.height / 2 - pad);
        maxX = Math.max(maxX, comp.x + def.width / 2 + pad);
        maxY = Math.max(maxY, comp.y + def.height / 2 + pad);
      });

      if (this.components.length === 0) {
        minX = 0; minY = 0; maxX = 200; maxY = 200;
      }

      const w = maxX - minX;
      const h = maxY - minY;
      clone.setAttribute('viewBox', `${minX} ${minY} ${w} ${h}`);
      clone.setAttribute('width', w);
      clone.setAttribute('height', h);
      clone.removeAttribute('class');

      // Add white background
      const bg = svgNS('rect', {
        x: minX, y: minY, width: w, height: h, fill: '#ffffff',
      });
      clone.insertBefore(bg, clone.firstChild);

      return new XMLSerializer().serializeToString(clone);
    }

    _exportSVG() {
      const svgStr = this._getExportSVG();
      const blob = new Blob([svgStr], { type: 'image/svg+xml' });
      const url = URL.createObjectURL(blob);
      const a = document.createElement('a');
      a.href = url;
      a.download = 'circuit-diagram.svg';
      a.click();
      URL.revokeObjectURL(url);
    }

    _getJSON() {
      return {
        components: this.components.map(c => ({
          type: c.type,
          x: c.x,
          y: c.y,
          rotation: c.rotation || 0,
          props: { ...c.props },
        })),
        wires: this.wires.map(w => ({
          from: { component: w.from.component, pin: w.from.pin },
          to: { component: w.to.component, pin: w.to.pin },
        })),
      };
    }

    _insertDiagram() {
      if (this.onInsert) {
        const svgStr = this._getExportSVG();
        const json = this._getJSON();
        this.onInsert(svgStr, json);
      }
      this.close();
    }
  }

  /* Expose globally */
  window.CircuitEditor = CircuitEditor;
})();
