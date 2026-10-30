// The Bill of Materials table in a project's Markdown, read as rows for
// the structured BOM (issue #264).
//
// The project's write-up holds its Bill of Materials as a Markdown table;
// the stored rows (project_bom_items) follow it on each save, through
// PUT /api/projects/{id}/bom/sync, so cost estimates and the parts
// catalog's "used in projects" keep working. parse() is pure: the editor
// sends what it returns (tests: node --test scripts/tests/*.test.js).
(function (root) {
  const PART_COL = /^(part|parts|item|items|name|component|components)$/;
  const QTY_COL = /^(qty|quantity|count|no\.?|#)$/;
  const PRICE_COL = /^(price|unit price|price each|each|cost|unit cost)$/;
  const SLUG_RE = /^(?:https?:\/\/(?:www\.)?kevsrobots\.com)?\/parts\/view\.html\?slug=([a-z0-9][a-z0-9-]*)$/i;
  const SYMBOLS = { '£': 'GBP', '$': 'USD', 'US$': 'USD', '€': 'EUR', '¥': 'JPY', 'A$': 'AUD', 'C$': 'CAD' };
  const CODE_SYMBOLS = { GBP: '£', USD: '$', EUR: '€', JPY: '¥', AUD: 'A$', CAD: 'C$' };

  function plain(cell) {
    return String(cell || '').replace(/\\\|/g, '|').replace(/[*_`]/g, '').trim();
  }

  // "£4.50", "4.50 EUR", "USD 3", "$1,200.00" -> { amount, code }; code
  // is null when the cell names no currency we know.
  function price(cell) {
    const text = plain(cell).replace(/,/g, '');
    const m = text.match(/-?\d+(?:\.\d+)?/);
    if (!m) return null;
    const before = text.slice(0, m.index).trim();
    const after = text.slice(m.index + m[0].length).trim();
    const mark = before || after;
    let code = null;
    if (/^[A-Z]{3}$/.test(mark)) code = mark;
    else if (SYMBOLS[mark]) code = SYMBOLS[mark];
    return { amount: parseFloat(m[0]), code };
  }

  // A price as the table shows it: "£4.50", or "4.50 CHF".
  function formatPrice(amount, code) {
    if (amount == null) return '';
    const n = Number(amount).toFixed(code === 'JPY' ? 0 : 2);
    if (!code) return n;
    return CODE_SYMBOLS[code] ? CODE_SYMBOLS[code] + n : n + ' ' + code;
  }

  // The Bill of Materials table, or null when the text has none:
  //   { hasPrice, rows: [{ name, quantity, part_slug, unit_cost?, currency_code? }] }
  // A row's unit_cost is only there when the table has a Price column, so
  // the stored price of a table without one is left alone.
  function parse(markdown) {
    const table = root.ProjectSections.sectionTable(markdown, 'bill-of-materials');
    if (!table) return null;
    const header = table.header.map(plain);
    let partCol = header.findIndex(h => PART_COL.test(h));
    if (partCol < 0) partCol = 0;
    const qtyCol = header.findIndex(h => QTY_COL.test(h));
    const priceCol = header.findIndex(h => PRICE_COL.test(h));
    const rows = [];
    table.rows.forEach(cells => {
      if (/^\*\*total\*\*$/i.test((cells[0] || '').trim())) return;
      const raw = (cells[partCol] || '').trim();
      const link = raw.match(/^\[((?:\\.|[^\]\\])*)\]\(([^)\s]+)[^)]*\)$/);
      const name = plain(link ? link[1].replace(/\\([\[\]\\])/g, '$1') : raw).slice(0, 200);
      if (!name) return;
      const slug = link && link[2].match(SLUG_RE);
      const qty = qtyCol >= 0 ? parseInt(plain(cells[qtyCol]), 10) : 1;
      const row = { name, quantity: qty >= 1 ? qty : 1, part_slug: slug ? slug[1].toLowerCase() : null };
      if (priceCol >= 0) {
        const p = price(cells[priceCol]);
        row.unit_cost = p ? p.amount : null;
        row.currency_code = p ? p.code : null;
      }
      rows.push(row);
    });
    return { hasPrice: priceCol >= 0, rows };
  }

  // A Bill of Materials section made from stored rows, for a project
  // saved before the table lived in its text.
  function tableFromRows(items, heading) {
    const cell = s => String(s).replace(/\|/g, '\\|');
    const lines = ['# ' + (heading || 'Bill of Materials'), '', '| Part | Qty | Price | Total | Notes |', '| --- | --- | --- | --- | --- |'];
    items.forEach(i => {
      const label = cell(String(i.name).replace(/([\[\]])/g, '\\$1'));
      const name = i.part_slug ? '[' + label + '](/parts/view.html?slug=' + encodeURIComponent(i.part_slug) + ')' : label;
      lines.push('| ' + name + ' | ' + i.quantity + ' | ' + formatPrice(i.unit_cost, i.currency_code) + ' |  |  |');
    });
    return lines.join('\n') + '\n';
  }

  // --- Totals ---
  //
  // When the Bill of Materials table has Qty and Price columns, the editor
  // keeps a Total column (Qty × Price per row, added if missing) and a
  // "**Total**" row under the last part with the sum. They're written
  // into the Markdown, so the project page and any export show them too.
  function isTotalRow(row) {
    return /^\*\*total\*\*$/i.test((row[0] || '').trim());
  }

  const BOM_QTY = /^(qty|quantity|count)$/;
  const BOM_PRICE = /^(price|unit price|price each|each|cost|unit cost)$/;
  const BOM_TOTAL = /^(total|subtotal|line total|row total)$/;

  function parseAmount(cell) {
    const m = String(cell || '').replace(/[*_`]/g, '').replace(/,/g, '').match(/-?\d+(?:\.\d+)?/);
    return m ? parseFloat(m[0]) : null;
  }

  // "£", "$", "€"... in front of the number, or after it ("12 EUR").
  function currencyOf(cell) {
    const text = String(cell || '').replace(/[*_`]/g, '').trim();
    const m = text.match(/^([^\d\s.-]+)\s*-?\d/) || text.match(/\d\s*([^\d\s.,]+)$/);
    return m ? { symbol: m[1], before: /^[^\d\s.-]/.test(text) } : null;
  }

  function money(n, currency) {
    const amount = n.toFixed(2);
    if (!currency) return amount;
    return currency.before ? currency.symbol + amount : amount + ' ' + currency.symbol;
  }

  // The lines to replace: [{ line, text }] (line counted from 0), plus
  // where a missing total row goes. Empty when there's nothing to change.
  function totalsChanges(markdown) {
    const table = root.ProjectSections.sectionTable(markdown, 'bill-of-materials');
    if (!table) return [];
    const header = table.header;
    const qtyCol = header.findIndex(h => BOM_QTY.test(h));
    const priceCol = header.findIndex(h => BOM_PRICE.test(h));
    if (qtyCol < 0 || priceCol < 0) return [];
    const lines = markdown.split('\n');
    const rowText = cells => '| ' + cells.join(' | ') + ' |';
    const raw = line => lines[line].trim().replace(/^\|/, '').replace(/(?<!\\)\|$/, '').split(/(?<!\\)\|/).map(c => c.trim());
    const changes = [];

    let totalCol = header.findIndex(h => BOM_TOTAL.test(h));
    const adding = totalCol < 0;
    if (adding) {
      totalCol = priceCol + 1;
      const head = raw(table.headerLine);
      head.splice(totalCol, 0, 'Total');
      changes.push({ line: table.headerLine, text: rowText(head) });
      const sep = raw(table.headerLine + 1);
      sep.splice(totalCol, 0, '---');
      changes.push({ line: table.headerLine + 1, text: rowText(sep) });
    }

    let currency = null;
    let sum = 0;
    let priced = 0;
    const totalRows = [];
    table.rows.forEach((row, r) => {
      const line = table.lines[r];
      if (isTotalRow(row)) { totalRows.push(line); return; }
      const cells = raw(line);
      while (cells.length < header.length) cells.push('');
      if (adding) cells.splice(totalCol, 0, '');
      const qty = parseAmount(cells[qtyCol]);
      const price = parseAmount(cells[priceCol]);
      currency = currency || currencyOf(cells[priceCol]);
      let total = '';
      if (qty !== null && price !== null) {
        sum += qty * price;
        priced++;
        total = money(qty * price, currencyOf(cells[priceCol]) || currency);
      }
      if (adding || (cells[totalCol] || '') !== total) {
        cells[totalCol] = total;
        changes.push({ line, text: rowText(cells) });
      }
    });

    // The grand total sits under the last part; any other total row goes.
    const width = header.length + (adding ? 1 : 0);
    const lastLine = table.lines[table.lines.length - 1];
    const totalCells = new Array(width).fill('');
    totalCells[0] = '**Total**';
    totalCells[totalCol] = '**' + money(sum, currency) + '**';
    const wantText = priced ? rowText(totalCells) : null;
    totalRows.forEach(line => {
      if (line === lastLine && wantText) {
        if (lines[line].trim() !== wantText) changes.push({ line, text: wantText });
      } else {
        changes.push({ line, text: null }); // remove
      }
    });
    if (wantText && totalRows[totalRows.length - 1] !== lastLine) {
      changes.push({ line: lastLine, append: wantText });
    }
    return changes;
  }

  // The text with totalsChanges applied, as the editor writes it.
  function withTotals(markdown) {
    const changes = totalsChanges(markdown);
    if (!changes.length) return markdown;
    const lines = markdown.split('\n');
    const out = [];
    lines.forEach((text, i) => {
      // A line can be rewritten (or removed) and have the total row after it.
      let line = text;
      const appended = [];
      changes.filter(c => c.line === i).forEach(c => {
        if (c.append) appended.push(c.append);
        else line = c.text;
      });
      if (line !== null) out.push(line);
      out.push(...appended);
    });
    return out.join('\n');
  }

  const api = { parse, price, formatPrice, tableFromRows, totalsChanges, withTotals };
  if (typeof module === 'object' && module.exports) module.exports = api;
  else root.BomSync = api;
})(typeof window !== 'undefined' ? window : globalThis);
