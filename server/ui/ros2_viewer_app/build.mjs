/*
 * Copyright (C) 2025 wisevision
 *
 * SPDX-License-Identifier: MPL-2.0
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 */

import { readFile, writeFile } from "node:fs/promises";
import { build } from "esbuild";

const result = await build({
  entryPoints: ["src/app.tsx"],
  bundle: true,
  write: false,
  platform: "browser",
  format: "iife",
  target: ["es2020"],
  // Avoid giant single-line output; some MCP App hosts behave badly with minified blobs.
  minify: false,
  jsx: "automatic",
  jsxImportSource: "preact",
});

const js = result.outputFiles[0]?.text ?? "";
if (!js) throw new Error("esbuild produced no output");

const template = await readFile("index.template.html", "utf8");
const marker = "/*__APP_JS__*/";
if (!template.includes(marker)) {
  throw new Error(`Missing marker ${marker} in index.template.html`);
}

const out = template.replace(marker, js);
await writeFile("index.html", out, "utf8");
console.log("Wrote index.html");
