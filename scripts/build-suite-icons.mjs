#!/usr/bin/env node
/**
 * Inlines the suite's app icons (web/assets/suite/icons/<id>.svg) into
 * kr-suite.js as data URIs, so bundled copies of the switcher work offline.
 * Run after changing an icon: `node scripts/build-suite-icons.mjs`.
 */
import { readFileSync, writeFileSync } from 'node:fs'
import { dirname, join } from 'node:path'
import { fileURLToPath } from 'node:url'

const suite = join(dirname(fileURLToPath(import.meta.url)), '..', 'web', 'assets', 'suite')
const file = join(suite, 'kr-suite.js')
const ids = ['learn', 'projects', 'snakie', 'bettercad']
/** A small, gzip-friendly SVG data URI: single quotes inside, only the characters a URL needs escaped. */
const dataUri = (svg) =>
  'data:image/svg+xml,' +
  svg
    .trim()
    .replace(/\s+/g, ' ')
    .replace(/"/g, "'")
    .replace(/[%#<>{}\n]/g, (c) => encodeURIComponent(c))
const lines = ids.map((id) => `  ${id}: ${JSON.stringify(dataUri(readFileSync(join(suite, 'icons', `${id}.svg`), 'utf8')))}`).join(',\n')
const src = readFileSync(file, 'utf8')
const next = src.replace(/export const KR_ICONS = Object\.freeze\(\{\n[\s\S]*?\n\}\)/, `export const KR_ICONS = Object.freeze({\n${lines}\n})`)
if (next === src) console.log('kr-suite.js: icons already up to date')
else {
  writeFileSync(file, next)
  console.log('kr-suite.js: icons updated')
}
