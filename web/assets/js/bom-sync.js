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
  // Where a row's other links go: a shop link moved off its name, say.
  const NOTE_COL = /^(notes?|links?|buy|where to buy|suppliers?|source|shop|comments?)$/;
  const QTY_COL = /^(qty|quantity|count|no\.?|#)$/;
  // A Price header can name the table's currency: "Price (£)", "Price (EUR)".
  const PRICE_COL = /^(price|unit price|price each|each|cost|unit cost)(?:\s*\([^)]*\))?$/;
  const SLUG_RE = /^(?:https?:\/\/(?:www\.)?kevsrobots\.com)?\/parts\/view\.html\?slug=([a-z0-9][a-z0-9-]*)$/i;
  const SYMBOLS = { '£': 'GBP', '$': 'USD', 'US$': 'USD', '€': 'EUR', '¥': 'JPY', 'A$': 'AUD', 'C$': 'CAD', '₹': 'INR' };
  const CODE_SYMBOLS = { GBP: '£', USD: '$', EUR: '€', JPY: '¥', AUD: 'A$', CAD: 'C$', INR: '₹' };
  // The currencies offered for a Bill of Materials' Price column, and how
  // many decimals each one shows.
  const CURRENCIES = [
    { code: 'GBP', name: 'British pound' },
    { code: 'USD', name: 'US dollar' },
    { code: 'EUR', name: 'Euro' },
    { code: 'CAD', name: 'Canadian dollar' },
    { code: 'AUD', name: 'Australian dollar' },
    { code: 'JPY', name: 'Japanese yen' },
    { code: 'INR', name: 'Indian rupee' },
    { code: 'CHF', name: 'Swiss franc' },
  ];
  const DECIMALS = { JPY: 0 };
  const decimals = code => (code && DECIMALS[code] != null ? DECIMALS[code] : 2);

  function plain(cell) {
    return String(cell || '').replace(/\\\|/g, '|').replace(/[*_`]/g, '').trim();
  }

  // A currency mark ("£", "A$", "EUR") as its code, or null.
  function codeOf(mark) {
    mark = String(mark || '').trim();
    if (/^[A-Z]{3}$/i.test(mark)) return mark.toUpperCase();
    return SYMBOLS[mark] || null;
  }

  // The currency a Price header names, "Price (£)" -> 'GBP', or null.
  function headerCurrency(header) {
    const m = plain(header).match(/\(([^)]*)\)\s*$/);
    return m ? codeOf(m[1]) : null;
  }

  // The header without its currency: "Unit price (£)" -> "Unit price".
  function headerLabel(header) {
    return plain(header).replace(/\s*\([^)]*\)\s*$/, '');
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
    const code = /^[A-Z]{3}$/.test(mark) ? mark : (SYMBOLS[mark] || null);
    return { amount: parseFloat(m[0]), code };
  }

  // A price as the table shows it: "£4.50", or "4.50 CHF".
  function formatPrice(amount, code) {
    if (amount == null) return '';
    const n = Number(amount).toFixed(decimals(code));
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
    const tableCode = priceCol >= 0 ? headerCurrency(header[priceCol]) : null;
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
        row.currency_code = p ? (p.code || tableCode) : null;
      }
      rows.push(row);
    });
    return { hasPrice: priceCol >= 0, rows };
  }

  // Linking the Bill of Materials row named `name` to catalog part `slug`,
  // as the wiring workspace's Link… does: { line, text } (the 0-based line
  // and its new text), or null when no row by that name is waiting for a
  // part. Rows are named as parse() names them, so whatever the workspace
  // lists as "not linked" can be linked here. The row's wording is kept
  // as the link text. A row whose name links a shop page instead keeps
  // that link in its Notes (or Link/Buy/Supplier) column, as "Buy"; with
  // no such column, the row is left alone, so no link is lost.
  // opts.dropShopLink: with no Notes column, link the row anyway, losing
  // the shop link from the table (it's on the new part's page by then).
  function linkRow(markdown, name, slug, opts) {
    const table = root.ProjectSections.sectionTable(markdown, 'bill-of-materials');
    if (!table || !slug) return null;
    const header = table.header.map(plain);
    let partCol = header.findIndex(h => PART_COL.test(h));
    if (partCol < 0) partCol = 0;
    const noteCol = header.findIndex(h => NOTE_COL.test(h));
    const want = String(name || '').trim().toLowerCase();
    const lines = String(markdown).split('\n');
    for (let r = 0; r < table.rows.length; r++) {
      const cells = table.rows[r];
      if (/^\*\*total\*\*$/i.test((cells[0] || '').trim())) continue;
      const raw = (cells[partCol] || '').trim();
      const link = raw.match(/^\[((?:\\.|[^\]\\])*)\]\(([^)\s]+)[^)]*\)$/);
      if (link && SLUG_RE.test(link[2])) continue; // linked to a part already
      const rowName = plain(link ? link[1].replace(/\\([\[\]\\])/g, '$1') : raw).slice(0, 200);
      if (!rowName || rowName.toLowerCase() !== want) continue;
      const lineNo = table.lines[r];
      const text = lines[lineNo];
      const pipes = [...text.matchAll(/(?<!\\)\|/g)].map(m => m.index);
      const span = col => {
        if (pipes[col] === undefined) return null;
        let start = pipes[col] + 1;
        let end = pipes[col + 1] === undefined ? text.length : pipes[col + 1];
        while (start < end && /\s/.test(text[start])) start++;
        while (end > start && /\s/.test(text[end - 1])) end--;
        return { start, end };
      };
      const part = span(partCol);
      if (!part) return null;
      const label = link ? link[1] : text.slice(part.start, part.end).replace(/(?<!\\)([\[\]])/g, '\\$1');
      const edits = [{ ...part, insert: '[' + label + '](/parts/view.html?slug=' + encodeURIComponent(slug) + ')' }];
      if (link) {
        // The shop link moves to the notes rather than being lost.
        const note = noteCol >= 0 && noteCol !== partCol ? span(noteCol) : null;
        if (!note && !(opts && opts.dropShopLink)) return null;
        const had = note ? text.slice(note.start, note.end) : link[2];
        if (note && !had.includes(link[2])) {
          const buy = '[Buy](' + link[2] + ')';
          edits.push(note.start === note.end
            ? { start: pipes[noteCol] + 1, end: pipes[noteCol + 1] === undefined ? text.length : pipes[noteCol + 1], insert: ' ' + buy + ' ' }
            : { start: note.end, end: note.end, insert: ' · ' + buy });
        }
      }
      let out = text;
      edits.sort((a, b) => b.start - a.start).forEach(e => { out = out.slice(0, e.start) + e.insert + out.slice(e.end); });
      return { line: lineNo, text: out };
    }
    return null;
  }

  // What the Bill of Materials row named `name` says about its part, to
  // fill in the "New part" dialog: { name, shopUrl, price, currency, notes }
  // (each null when the row doesn't say), or null when there's no such row.
  // The shop link is the one on the name, else the first web link in the
  // Notes column; the notes are that column's text without its links.
  function rowDetails(markdown, name) {
    const table = root.ProjectSections.sectionTable(markdown, 'bill-of-materials');
    if (!table) return null;
    const header = table.header.map(plain);
    let partCol = header.findIndex(h => PART_COL.test(h.toLowerCase()));
    if (partCol < 0) partCol = 0;
    const noteCol = header.findIndex(h => NOTE_COL.test(h.toLowerCase()));
    const priceCol = header.findIndex(h => PRICE_COL.test(h.toLowerCase()));
    const tableCode = priceCol >= 0 ? headerCurrency(header[priceCol]) : null;
    const want = String(name || '').trim().toLowerCase();
    for (const cells of table.rows) {
      if (/^\*\*total\*\*$/i.test((cells[0] || '').trim())) continue;
      const raw = (cells[partCol] || '').trim();
      const link = raw.match(/^\[((?:\\.|[^\]\\])*)\]\(([^)\s]+)[^)]*\)$/);
      const rowName = plain(link ? link[1].replace(/\\([\[\]\\])/g, '$1') : raw).slice(0, 200);
      if (!rowName || rowName.toLowerCase() !== want) continue;
      let shopUrl = link && /^https?:\/\//i.test(link[2]) ? link[2] : null;
      let notes = null;
      if (noteCol >= 0 && noteCol !== partCol) {
        const note = cells[noteCol] || '';
        if (!shopUrl) {
          const m = note.match(/\]\((https?:\/\/[^)\s]+)/i) || note.match(/https?:\/\/[^\s)|]+/i);
          if (m) shopUrl = m[1] || m[0];
        }
        notes = plain(note.replace(/!?\[[^\]]*\]\([^)]*\)/g, '').replace(/https?:\/\/\S+/g, '').replace(/<br\s*\/?>/gi, ' ').replace(/\s*·\s*$/, '').replace(/^\s*·\s*/, '')) || null;
      }
      const p = priceCol >= 0 ? price(cells[priceCol]) : null;
      return {
        name: rowName,
        shopUrl,
        price: p ? p.amount : null,
        currency: p ? (p.code || tableCode) : tableCode,
        notes,
      };
    }
    return null;
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
  const BOM_PRICE = PRICE_COL;
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
  // When the Price header names a currency ("Price (£)"), every price and
  // total is written in it with its decimals: "4" becomes "£4.00". The
  // price on `options.typingLine` (the caret's row) is left as typed until
  // the caret moves on, so "4" isn't turned into "£4.00" mid-"4.5".
  function totalsChanges(markdown, options) {
    const typingLine = options && options.typingLine != null ? options.typingLine : -1;
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

    const tableCode = headerCurrency(header[priceCol]);
    const show = (n, cellCurrency) => (tableCode ? formatPrice(n, tableCode) : money(n, cellCurrency));
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
        total = show(qty * price, currencyOf(cells[priceCol]) || currency);
      }
      let changed = adding || (cells[totalCol] || '') !== total;
      cells[totalCol] = total;
      if (tableCode && price !== null && line !== typingLine) {
        const shown = formatPrice(price, tableCode);
        if (cells[priceCol] !== shown) { cells[priceCol] = shown; changed = true; }
      }
      if (changed) changes.push({ line, text: rowText(cells) });
    });

    // The grand total sits under the last part; any other total row goes.
    const width = header.length + (adding ? 1 : 0);
    const lastLine = table.lines[table.lines.length - 1];
    const totalCells = new Array(width).fill('');
    totalCells[0] = '**Total**';
    totalCells[totalCol] = '**' + show(sum, currency) + '**';
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

  // --- Currency ---

  // The table's rows (header first, each cell's Markdown) with its Price
  // column in `code` (null for none): the header names it, "Price (£)",
  // and each price and total is written with the currency's decimals.
  function withCurrency(rows, code) {
    if (!rows.length) return rows;
    const head = rows[0].map(h => plain(h).toLowerCase());
    const priceCol = head.findIndex(h => PRICE_COL.test(h));
    if (priceCol < 0) return rows;
    const totalCol = head.findIndex(h => BOM_TOTAL.test(h));
    const mark = code ? (CODE_SYMBOLS[code] || code) : null;
    const amount = cell => {
      const p = price(cell);
      return p ? formatPrice(p.amount, code) : null;
    };
    return rows.map((row, r) => row.map((cell, c) => {
      if (r === 0) return c === priceCol ? headerLabel(cell) + (mark ? ' (' + mark + ')' : '') : cell;
      if (c !== priceCol && c !== totalCol) return cell;
      if (c === priceCol && isTotalRow(row)) return cell;
      const shown = amount(cell);
      if (shown === null) return cell;
      return /^\*\*.*\*\*$/.test(cell.trim()) ? '**' + shown + '**' : shown;
    }));
  }

  // The currency a Bill of Materials table's prices are in: the Price
  // header's, else the first price that names one. Null for none.
  function tableCurrency(rows) {
    const head = (rows[0] || []).map(h => plain(h).toLowerCase());
    const priceCol = head.findIndex(h => PRICE_COL.test(h));
    if (priceCol < 0) return null;
    const named = headerCurrency(rows[0][priceCol]);
    if (named) return named;
    for (const row of rows.slice(1)) {
      const p = price(row[priceCol]);
      if (p && p.code) return p.code;
    }
    return null;
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

  const api = {
    parse, linkRow, rowDetails, price, formatPrice, tableFromRows, totalsChanges, withTotals,
    CURRENCIES, headerCurrency, withCurrency, tableCurrency, isPriceHeader: h => PRICE_COL.test(plain(h).toLowerCase()),
  };
  if (typeof module === 'object' && module.exports) module.exports = api;
  else root.BomSync = api;
})(typeof window !== 'undefined' ? window : globalThis);
