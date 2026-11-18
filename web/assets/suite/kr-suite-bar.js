/*
 * <kr-suite-bar>: the kevsrobots suite bar (@kevsrobots/suite, kevsrobots.com #581).
 * =============================================================================
 *
 * One bar for every app in the suite: the kevsrobots mark, the app switcher,
 * the app's name, and optionally search and the account menu. No framework and
 * no dependencies, so Snakie and BetterCAD can bundle it for offline and
 * desktop use and kevsrobots.com can load it with one <script>.
 *
 *   <script src="https://www.kevsrobots.com/assets/suite/kr-suite-bar.js" defer></script>
 *   <kr-suite-bar app="snakie" context="burgerbot / main.py"
 *                 search="https://www.kevsrobots.com/search-results.html?query={q}&page=1">
 *     <button slot="account">KM</button>
 *   </kr-suite-bar>
 *
 * Attributes (all optional):
 *   app          learn | projects | snakie | bettercad (default learn). Sets the
 *                accent and marks the current app in the switcher.
 *   app-name     The label beside the mark (default: the app's name).
 *   context      Smaller text after the name: a lesson, a file, a project.
 *   variant      bar (default): the full 48 px bar with the accent stripe.
 *                brand: just the mark, switcher and name, inline, for a page
 *                that keeps its own navbar (kevsrobots.com does, for now).
 *   theme        light (default) | dark | auto.
 *   home         Where the mark goes (default https://www.kevsrobots.com/).
 *   mark-src     An image for the mark instead of the "KR" tile.
 *   search       A URL with {q} where the query goes. No attribute, no search.
 *   search-placeholder
 *   hide-name    Present: show the name to screen readers only.
 *
 * Slots: "account" (right-hand end: an avatar or menu), "actions" (before it).
 *
 * Events (both bubble and can be cancelled with preventDefault()):
 *   kr-suite-search  detail { query }        before search navigates
 *   kr-suite-switch  detail { app, href }    before the switcher navigates;
 *                                            a desktop app can open its own
 *                                            window instead.
 *
 * The `apps` property takes your own list ({ id, name, href, blurb, mark })
 * if an app needs other URLs, such as a local dev server.
 */
(function () {
  'use strict';

  if (typeof window === 'undefined' || !window.customElements) return;
  if (window.customElements.get('kr-suite-bar')) return;

  var APPS = [
    { id: 'learn', name: 'Learn', href: 'https://www.kevsrobots.com/', blurb: 'Courses, lessons and part guides', mark: 'KR' },
    { id: 'projects', name: 'Projects', href: 'https://www.kevsrobots.com/projects/hub', blurb: 'Your builds, steps and parts lists', mark: 'P' },
    { id: 'snakie', name: 'Snakie', href: 'https://app.snakie.org/', blurb: 'Write and run MicroPython', mark: 'S' },
    { id: 'bettercad', name: 'BetterCAD', href: 'https://bettercad.kevsrobots.com/', blurb: 'Design parts to 3D print', mark: 'B' }
  ];

  // Every colour falls back to the light token value, so the bar looks right
  // even on a page that hasn't loaded kr-tokens.css.
  var CSS = [
    ':host{',
    '  --_paper:var(--kr-paper,#F7F7F4);--_panel:var(--kr-panel,#FFFFFF);--_sunk:var(--kr-sunk,#EEF0EC);',
    '  --_ink:var(--kr-ink,#1C2230);--_soft:var(--kr-ink-soft,#5A6275);--_line:var(--kr-line,#E2E4E8);',
    '  --_shadow:var(--kr-shadow,0 1px 2px rgba(28,34,48,.06),0 8px 24px rgba(28,34,48,.08));',
    '  --_red:var(--kr-red,#C22032);--_green:var(--kr-green,#26A269);--_violet:var(--kr-violet,#7C3AED);',
    '  --_gold:var(--kr-gold,#D9A441);--_on-gold:var(--kr-on-gold,#1C2230);',
    '  --_acc:var(--_red);',
    '  --_ui:var(--kr-font-ui,"Plus Jakarta Sans",system-ui,-apple-system,"Segoe UI",Roboto,sans-serif);',
    '  --_display:var(--kr-font-display,"Oswald","Arial Narrow",sans-serif);',
    '  --_mono:var(--kr-font-mono,"IBM Plex Mono",ui-monospace,Menlo,Consolas,monospace);',
    '  display:block;position:relative;font-family:var(--_ui);color:var(--_ink);font-size:14px;line-height:1.4;',
    '  -webkit-font-smoothing:antialiased;',
    '}',
    ':host([theme="dark"]){',
    '  --_paper:#14161B;--_panel:#1C1F26;--_sunk:#181B21;--_ink:#E8EAEE;--_soft:#A2A9B8;--_line:#2E323B;',
    '  --_red:#F0596A;--_green:#4CC98C;--_violet:#A78BFA;--_gold:#E6B85C;',
    '  --_shadow:0 1px 2px rgba(0,0,0,.3),0 8px 24px rgba(0,0,0,.35);',
    '}',
    '@media (prefers-color-scheme:dark){:host([theme="auto"]){',
    '  --_paper:#14161B;--_panel:#1C1F26;--_sunk:#181B21;--_ink:#E8EAEE;--_soft:#A2A9B8;--_line:#2E323B;',
    '  --_red:#F0596A;--_green:#4CC98C;--_violet:#A78BFA;--_gold:#E6B85C;',
    '  --_shadow:0 1px 2px rgba(0,0,0,.3),0 8px 24px rgba(0,0,0,.35);',
    '}}',
    ':host([app="projects"]){--_acc:var(--_gold)}',
    ':host([app="snakie"]){--_acc:var(--_green)}',
    ':host([app="bettercad"]){--_acc:var(--_violet)}',
    ':host([variant="brand"]){display:inline-block}',
    '*{box-sizing:border-box}',
    '.bar{display:flex;align-items:center;gap:12px;min-width:0}',
    ':host(:not([variant="brand"])) .bar{height:var(--kr-suite-bar-height,48px);padding:0 12px;background:var(--_panel);',
    '  border-top:3px solid var(--_acc);border-bottom:1px solid var(--_line)}',
    'a,button{font:inherit;color:inherit}',
    ':focus-visible{outline:2px solid var(--_acc);outline-offset:2px}',
    '.mark{flex:none;display:grid;place-items:center;width:30px;height:30px;border-radius:7px;text-decoration:none;',
    '  background:var(--_red);color:#fff;font-family:var(--_display);font-weight:600;font-size:14px;letter-spacing:.02em}',
    '.mark.img{background:transparent;width:40px;height:40px;border-radius:0}',
    '.mark img{display:block;width:100%;height:100%;object-fit:contain}',
    '.switch{flex:none;width:32px;height:32px;padding:0;border-radius:8px;border:1px solid var(--_line);background:transparent;',
    '  display:grid;grid-template-columns:repeat(3,4px);gap:3px;place-content:center;cursor:pointer}',
    '.switch span{width:4px;height:4px;border-radius:1px;background:var(--_soft)}',
    '.switch:hover,.switch[aria-expanded="true"]{border-color:var(--_acc)}',
    '.switch:hover span,.switch[aria-expanded="true"] span{background:var(--_acc)}',
    '.name{display:flex;align-items:center;gap:8px;min-width:0;font-weight:700;white-space:nowrap}',
    '.name i{flex:none;width:10px;height:10px;border-radius:50%;background:var(--_acc)}',
    '.name small{font-weight:500;color:var(--_soft);font-size:13px;overflow:hidden;text-overflow:ellipsis}',
    '.name small:empty{display:none}',
    '.sr{position:absolute;width:1px;height:1px;margin:-1px;overflow:hidden;clip:rect(0 0 0 0);white-space:nowrap}',
    '.spacer{flex:1}',
    ':host([variant="brand"]) .spacer{display:none}',
    'form{display:flex;align-items:center;gap:8px;height:32px;width:240px;padding:0 10px;border:1px solid var(--_line);',
    '  border-radius:8px;background:var(--_sunk);color:var(--_soft)}',
    'form:focus-within{border-color:var(--_acc)}',
    'form input{flex:1;min-width:0;border:0;background:transparent;font:inherit;font-size:13px;color:var(--_ink);outline:none}',
    'form input::placeholder{color:var(--_soft)}',
    'kbd{font-family:var(--_mono);font-size:11px;border:1px solid var(--_line);border-radius:4px;padding:0 4px}',
    '::slotted([slot="account"]){flex:none}',
    '.menu{position:absolute;z-index:1100;top:calc(100% + 6px);left:0;width:320px;max-width:calc(100vw - 24px);',
    '  padding:8px;border:1px solid var(--_line);border-radius:12px;background:var(--_panel);box-shadow:var(--_shadow);',
    '  display:grid;gap:2px;text-align:left}',
    ':host(:not([variant="brand"])) .menu{left:12px}',
    '.menu[hidden]{display:none}',
    '.app{display:grid;grid-template-columns:36px 1fr;gap:12px;align-items:center;padding:8px;border-radius:8px;',
    '  text-decoration:none;color:var(--_ink)}',
    '.app:hover,.app:focus-visible{background:var(--_sunk);outline:none}',
    '.app[aria-current="true"]{background:var(--_sunk);box-shadow:inset 3px 0 0 var(--_acc)}',
    '.app .ic{width:36px;height:36px;border-radius:9px;display:grid;place-items:center;font-weight:700;color:#fff;',
    '  font-family:var(--_display);font-size:15px}',
    '.app b{display:block;font-size:14px}',
    '.app span{display:block;font-size:12.5px;color:var(--_soft)}',
    '.app b .by{display:inline;font-size:12.5px;font-weight:500;color:var(--_soft)}',
    '.foot{border-top:1px solid var(--_line);margin-top:6px;padding:8px 8px 2px;font-size:12px;color:var(--_soft)}',
    '.foot a{color:var(--_soft)}',
    '@media (max-width:640px){form{display:none}.name small{display:none}}'
  ].join('\n');

  var ICON_BG = { learn: '--_red', projects: '--_gold', snakie: '--_green', bettercad: '--_violet' };

  function el(tag, attrs, text) {
    var node = document.createElement(tag);
    if (attrs) {
      Object.keys(attrs).forEach(function (key) {
        if (attrs[key] !== null && attrs[key] !== undefined) node.setAttribute(key, attrs[key]);
      });
    }
    if (text) node.textContent = text;
    return node;
  }

  var uid = 0;

  class KrSuiteBar extends HTMLElement {
    static get observedAttributes() {
      return ['app', 'app-name', 'context', 'home', 'mark-src', 'search', 'search-placeholder', 'hide-name', 'variant'];
    }

    constructor() {
      super();
      this._apps = APPS.slice();
      this._id = 'kr-suite-menu-' + (++uid);
      this._onDocClick = this._onDocClick.bind(this);
      this._onKey = this._onKey.bind(this);
      this.attachShadow({ mode: 'open' });
    }

    get apps() { return this._apps.slice(); }
    set apps(list) {
      if (Array.isArray(list) && list.length) {
        this._apps = list.slice();
        this._render();
      }
    }

    connectedCallback() {
      this._render();
      document.addEventListener('keydown', this._onKey);
    }

    disconnectedCallback() {
      document.removeEventListener('keydown', this._onKey);
      document.removeEventListener('click', this._onDocClick, true);
    }

    attributeChangedCallback() {
      if (this.isConnected) this._render();
    }

    _current() {
      var id = (this.getAttribute('app') || 'learn').toLowerCase();
      for (var i = 0; i < this._apps.length; i++) {
        if (this._apps[i].id === id) return this._apps[i];
      }
      return { id: id, name: this.getAttribute('app-name') || id };
    }

    _render() {
      var root = this.shadowRoot;
      var app = this._current();
      root.innerHTML = '';
      root.appendChild(el('style', null, CSS));

      var bar = el('div', { 'class': 'bar', part: 'bar' });

      var home = this.getAttribute('home') || 'https://www.kevsrobots.com/';
      var markSrc = this.getAttribute('mark-src');
      var mark = el('a', { 'class': 'mark' + (markSrc ? ' img' : ''), href: home, part: 'mark', title: 'kevsrobots home' });
      if (markSrc) {
        mark.appendChild(el('img', { src: markSrc, alt: 'kevsrobots home' }));
      } else {
        mark.textContent = 'KR';
        mark.setAttribute('aria-label', 'kevsrobots home');
      }
      bar.appendChild(mark);

      var switcher = el('button', {
        'class': 'switch', type: 'button', part: 'switcher',
        'aria-haspopup': 'true', 'aria-expanded': 'false', 'aria-controls': this._id,
        'aria-label': 'kevsrobots apps', title: 'kevsrobots apps'
      });
      for (var d = 0; d < 9; d++) switcher.appendChild(el('span'));
      switcher.addEventListener('click', function () { this._toggle(); }.bind(this));
      bar.appendChild(switcher);

      var name = el('div', { 'class': 'name', part: 'name' });
      name.appendChild(el('i', { 'aria-hidden': 'true' }));
      var label = el('span', this.hasAttribute('hide-name') ? { 'class': 'sr' } : null, this.getAttribute('app-name') || app.name);
      name.appendChild(label);
      name.appendChild(el('small', null, this.getAttribute('context') || ''));
      bar.appendChild(name);

      bar.appendChild(el('div', { 'class': 'spacer' }));

      var search = this.getAttribute('search');
      if (search) bar.appendChild(this._searchForm(search));

      bar.appendChild(el('slot', { name: 'actions' }));
      bar.appendChild(el('slot', { name: 'account' }));

      root.appendChild(bar);
      root.appendChild(this._menu(app));
    }

    _searchForm(template) {
      var form = el('form', { role: 'search', part: 'search' });
      var input = el('input', {
        type: 'search', name: 'q', autocomplete: 'off',
        placeholder: this.getAttribute('search-placeholder') || 'Search lessons, parts, projects',
        'aria-label': 'Search kevsrobots'
      });
      form.appendChild(input);
      form.appendChild(el('kbd', { 'aria-hidden': 'true' }, '/'));
      form.addEventListener('submit', function (event) {
        event.preventDefault();
        var query = input.value.trim();
        if (!query) return;
        var go = this.dispatchEvent(new CustomEvent('kr-suite-search', {
          bubbles: true, composed: true, cancelable: true, detail: { query: query }
        }));
        if (go) window.location.href = template.replace('{q}', encodeURIComponent(query));
      }.bind(this));
      this._input = input;
      return form;
    }

    _menu(current) {
      var menu = el('div', { 'class': 'menu', id: this._id, part: 'menu', role: 'menu', 'aria-label': 'kevsrobots apps', hidden: '' });
      this._apps.forEach(function (app) {
        var link = el('a', { 'class': 'app', href: app.href, role: 'menuitem' });
        if (app.id === current.id) link.setAttribute('aria-current', 'true');
        var ic = el('span', { 'class': 'ic', 'aria-hidden': 'true' }, app.mark || app.name.charAt(0));
        ic.style.background = 'var(' + (ICON_BG[app.id] || '--_acc') + ')';
        if (app.id === 'projects') ic.style.color = 'var(--_on-gold)';
        link.appendChild(ic);
        var text = el('span');
        var title = el('b', null, app.name);
        if (app.id === 'snakie' || app.id === 'bettercad') title.appendChild(el('span', { 'class': 'by' }, ' by kevsrobots'));
        text.appendChild(title);
        if (app.blurb) text.appendChild(el('span', null, app.blurb));
        link.appendChild(text);
        link.addEventListener('click', function (event) {
          var go = this.dispatchEvent(new CustomEvent('kr-suite-switch', {
            bubbles: true, composed: true, cancelable: true, detail: { app: app.id, href: app.href }
          }));
          if (!go) event.preventDefault();
          this._close();
        }.bind(this));
        menu.appendChild(link);
      }, this);
      var foot = el('div', { 'class': 'foot' });
      foot.appendChild(el('a', { href: 'https://www.kevsrobots.com/whats-new/' }, 'What\u2019s new in the suite'));
      menu.appendChild(foot);
      return menu;
    }

    _items() {
      return Array.prototype.slice.call(this.shadowRoot.querySelectorAll('.app'));
    }

    _isOpen() {
      var menu = this.shadowRoot.getElementById(this._id);
      return !!menu && !menu.hidden;
    }

    _toggle() {
      if (this._isOpen()) this._close(); else this._open();
    }

    _open() {
      var menu = this.shadowRoot.getElementById(this._id);
      if (!menu) return;
      menu.hidden = false;
      this.shadowRoot.querySelector('.switch').setAttribute('aria-expanded', 'true');
      document.addEventListener('click', this._onDocClick, true);
      var items = this._items();
      var current = items.filter(function (a) { return a.getAttribute('aria-current') === 'true'; })[0] || items[0];
      if (current) current.focus();
    }

    _close(returnFocus) {
      var menu = this.shadowRoot.getElementById(this._id);
      if (!menu || menu.hidden) return;
      menu.hidden = true;
      var button = this.shadowRoot.querySelector('.switch');
      button.setAttribute('aria-expanded', 'false');
      document.removeEventListener('click', this._onDocClick, true);
      if (returnFocus) button.focus();
    }

    _onDocClick(event) {
      var path = event.composedPath ? event.composedPath() : [];
      if (path.indexOf(this) === -1) this._close();
    }

    _onKey(event) {
      if (this._isOpen()) {
        var items = this._items();
        var at = items.indexOf(this.shadowRoot.activeElement);
        if (event.key === 'Escape') {
          event.preventDefault();
          this._close(true);
        } else if (event.key === 'ArrowDown' || event.key === 'ArrowUp') {
          event.preventDefault();
          var step = event.key === 'ArrowDown' ? 1 : -1;
          items[(at + step + items.length) % items.length].focus();
        } else if (event.key === 'Home' || event.key === 'End') {
          event.preventDefault();
          items[event.key === 'Home' ? 0 : items.length - 1].focus();
        } else if (event.key === 'Tab') {
          this._close();
        }
        return;
      }
      // "/" jumps to search, unless the reader is already typing somewhere.
      if (event.key === '/' && this._input && this.getAttribute('search') && !event.ctrlKey && !event.metaKey && !event.altKey) {
        var target = event.composedPath ? event.composedPath()[0] : event.target;
        var tag = target && target.tagName;
        if (tag === 'INPUT' || tag === 'TEXTAREA' || tag === 'SELECT' || (target && target.isContentEditable)) return;
        event.preventDefault();
        this._input.focus();
      }
    }
  }

  window.customElements.define('kr-suite-bar', KrSuiteBar);
})();
