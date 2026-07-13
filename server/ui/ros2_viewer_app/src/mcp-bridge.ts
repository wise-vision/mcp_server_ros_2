/*
 * Copyright (C) 2025 wisevision
 *
 * SPDX-License-Identifier: MPL-2.0
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 */

// MCP Bridge - handles communication with MCP host

interface MCPBridge {
  callTool?: (params: { name: string; arguments: Record<string, unknown> }) => Promise<unknown>;
  tools?: {
    call?: (name: string, args: Record<string, unknown>) => Promise<unknown>;
  };
}

interface PendingRequest {
  resolve: (value: unknown) => void;
  reject: (reason: unknown) => void;
}

const pending = new Map<number, PendingRequest>();
let rpcId = 1;

function getBridge(): MCPBridge | null {
  const g = globalThis as Record<string, unknown>;
  return (g.mcp || g.MCP || g.__MCP__ || null) as MCPBridge | null;
}

// Listen for postMessage responses
if (typeof window !== "undefined") {
  window.addEventListener("message", (ev: MessageEvent) => {
    const msg = ev.data;
    if (!msg || typeof msg !== "object") return;
    if (msg.jsonrpc !== "2.0" || msg.id == null) return;
    const p = pending.get(msg.id);
    if (!p) return;
    pending.delete(msg.id);
    if (msg.error) {
      p.reject(msg.error);
    } else {
      p.resolve(msg.result);
    }
  });
}

function rpcRequest(method: string, params: Record<string, unknown>): Promise<unknown> {
  return new Promise((resolve, reject) => {
    const id = rpcId++;
    pending.set(id, { resolve, reject });
    window.parent?.postMessage({ jsonrpc: "2.0", id, method, params }, "*");
    setTimeout(() => {
      if (pending.has(id)) {
        pending.delete(id);
        reject({ message: "Timeout waiting for host response." });
      }
    }, 8000);
  });
}

export async function callTool(name: string, args: Record<string, unknown> = {}): Promise<unknown> {
  const finalArgs = args;
  const bridge = getBridge();
  if (bridge) {
    if (typeof bridge.callTool === "function") {
      return await bridge.callTool({ name, arguments: finalArgs });
    }
    if (typeof bridge.tools?.call === "function") {
      return await bridge.tools.call(name, finalArgs);
    }
  }
  return await rpcRequest("tools/call", { name, arguments: finalArgs });
}

interface ContentItem {
  type: string;
  text?: string;
}

export function extractFirstText(result: unknown): string | null {
  const content = (result as { content?: unknown })?.content ?? result;
  if (!Array.isArray(content)) return null;
  const t = content.find((c: ContentItem) => c?.type === "text");
  return t?.text ?? null;
}
