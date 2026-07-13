/*
 * Copyright (C) 2025 wisevision
 *
 * SPDX-License-Identifier: MPL-2.0
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 */

import { render, ComponentChildren } from "preact";
import { memo } from "preact/compat";
import { useState, useEffect, useRef, useCallback, useMemo } from "preact/hooks";
import { callTool, extractFirstText } from "./mcp-bridge";
import { PointCloudViewer3D, PointCloudData } from "./PointCloudViewer3D";

// Types
interface Topic {
  name: string;
  type: string;
}

interface StreamResponse {
  session_id?: string;
  error?: string;
  available?: boolean;
  seq?: number;
  message?: unknown;
  data?: string;
  mimeType?: string;
  positionsF32B64?: string;
  colorsF32B64?: string;
  pointCount?: number;
  stamp?: { sec: number; nanosec: number };
  frame_id?: string;
  rx_count?: number;
  last_rx_age_sec?: number;
  last_error?: string;
}

type Mode = "image" | "pointcloud" | "plot";

type PlotPoint = { t: number; y: number };
type PlotSeries = { path: string; color: string; points: PlotPoint[] };

interface ViewerConfig {
  autoStart?: boolean;
  preferredKind?: "image" | "pointcloud" | "plot" | "auto";
  topicName?: string;
  plotFields?: string[];
  auto_start?: boolean;
  preferred_kind?: "image" | "pointcloud" | "plot" | "auto";
  topic_name?: string;
  plot_fields?: string[];
}

function normalizeViewerConfig(config?: ViewerConfig): ViewerConfig | undefined {
  if (!config) return undefined;
  const normalized: ViewerConfig = {
    autoStart: config.autoStart ?? config.auto_start,
    preferredKind: config.preferredKind ?? config.preferred_kind,
    topicName: config.topicName ?? config.topic_name,
    plotFields: config.plotFields ?? config.plot_fields,
  };
  if (
    normalized.autoStart === undefined &&
    normalized.preferredKind === undefined &&
    !normalized.topicName
  ) {
    return undefined;
  }
  return normalized;
}

function configSignature(config?: ViewerConfig): string | null {
  const normalized = normalizeViewerConfig(config);
  if (!normalized) return null;
  const plotFields = Array.isArray(normalized.plotFields)
    ? normalized.plotFields.map((f) => String(f).trim()).filter((f) => f.length > 0)
    : undefined;
  const payload = {
    autoStart: normalized.autoStart ?? null,
    preferredKind: normalized.preferredKind ?? null,
    topicName: normalized.topicName ?? null,
    plotFields: plotFields ?? null,
  };
  return JSON.stringify(payload);
}

// Utility functions
function modeForTopicType(type: string): Mode | null {
  if (type === "sensor_msgs/msg/Image" || type === "sensor_msgs/msg/CompressedImage") {
    return "image";
  }
  if (type === "sensor_msgs/msg/PointCloud2") {
    return "pointcloud";
  }
  return null;
}

function isTopicCompatible(topic: Topic | null, mode: Mode): boolean {
  if (!topic) return false;
  if (mode === "plot") return true;
  return modeForTopicType(topic.type) === mode;
}

function b64ToArrayBuffer(b64: string): ArrayBuffer {
  const bin = atob(b64);
  const bytes = new Uint8Array(bin.length);
  for (let i = 0; i < bin.length; i++) bytes[i] = bin.charCodeAt(i);
  return bytes.buffer;
}

const PLOT_MAX_POINTS = 600;
const PLOT_MAX_POINTS_HARD = 5000;
const PLOT_MAX_FIELDS = 220;
const PLOT_MAX_ARRAY_ELEMS = 8;
const PLOT_MAX_DEPTH = 6;
const NUMERIC_TYPES = new Set([
  "bool",
  "boolean",
  "byte",
  "char",
  "float32",
  "float64",
  "double",
  "float",
  "int8",
  "uint8",
  "int16",
  "uint16",
  "int32",
  "uint32",
  "int64",
  "uint64",
]);
const ARRAY_TYPE_RE = /^(.*)\[(?:<=)?\d*\]$/;
const SEQ_TYPE_RE = /^sequence<(.+?)(?:,\s*\d+)?>$/;
const COLOR_PALETTE = [
  "#60a5fa",
  "#34d399",
  "#fbbf24",
  "#f87171",
  "#a78bfa",
  "#22d3ee",
  "#fb7185",
  "#38bdf8",
  "#f59e0b",
  "#4ade80",
];
const PATH_TOKEN_RE = /([^[.\]]+)|\[(\d+)\]/g;

function toNumber(value: unknown): number | null {
  if (typeof value === "number" && Number.isFinite(value)) return value;
  if (typeof value === "boolean") return value ? 1 : 0;
  return null;
}

function hashString(input: string): number {
  let hash = 0;
  for (let i = 0; i < input.length; i++) {
    hash = (hash * 31 + input.charCodeAt(i)) | 0;
  }
  return Math.abs(hash);
}

function colorForPath(path: string): string {
  return COLOR_PALETTE[hashString(path) % COLOR_PALETTE.length];
}

function getValueByPath(source: unknown, path: string): unknown {
  if (!path) return undefined;
  const tokens: Array<string | number> = [];
  PATH_TOKEN_RE.lastIndex = 0;
  let match: RegExpExecArray | null;
  while ((match = PATH_TOKEN_RE.exec(path)) !== null) {
    if (match[1]) tokens.push(match[1]);
    if (match[2]) tokens.push(Number(match[2]));
  }
  let cur: any = source;
  for (const token of tokens) {
    if (cur == null) return undefined;
    cur = cur[token as keyof typeof cur];
  }
  return cur;
}

function parseFieldType(typeStr: string): { base: string; isArray: boolean } {
  let t = typeStr.trim();
  let isArray = false;
  const seqMatch = SEQ_TYPE_RE.exec(t);
  if (seqMatch) {
    t = seqMatch[1].trim();
    isArray = true;
  }
  const arrMatch = ARRAY_TYPE_RE.exec(t);
  if (arrMatch) {
    t = arrMatch[1].trim();
    isArray = true;
  }
  return { base: t, isArray };
}

function isNumericType(typeStr: string): boolean {
  return NUMERIC_TYPES.has(typeStr);
}

function collectNumericPaths(
  value: unknown,
  prefix: string,
  out: string[],
  depth: number,
): void {
  if (out.length >= PLOT_MAX_FIELDS) return;
  if (depth > PLOT_MAX_DEPTH) return;
  const direct = toNumber(value);
  if (direct !== null) {
    if (prefix) out.push(prefix);
    return;
  }
  if (Array.isArray(value)) {
    const limit = Math.min(value.length, PLOT_MAX_ARRAY_ELEMS);
    for (let i = 0; i < limit; i++) {
      if (out.length >= PLOT_MAX_FIELDS) break;
      const next = value[i];
      const idxPath = prefix ? `${prefix}[${i}]` : `[${i}]`;
      const numeric = toNumber(next);
      if (numeric !== null) {
        out.push(idxPath);
        continue;
      }
      collectNumericPaths(next, idxPath, out, depth + 1);
    }
    return;
  }
  if (value && typeof value === "object") {
    for (const [key, next] of Object.entries(value as Record<string, unknown>)) {
      if (out.length >= PLOT_MAX_FIELDS) break;
      const nextPath = prefix ? `${prefix}.${key}` : key;
      collectNumericPaths(next, nextPath, out, depth + 1);
    }
  }
}

function normalizePlotValue(value: unknown, depth = 0): unknown {
  if (depth > 6) return value;
  if (Array.isArray(value)) {
    return value.map((item) => normalizePlotValue(item, depth + 1));
  }
  if (value && typeof value === "object") {
    const rec = value as Record<string, unknown>;
    const out: Record<string, unknown> = {};
    for (const [key, val] of Object.entries(rec)) {
      const clean = key.startsWith("_") ? key.slice(1) : key;
      if (!(clean in out)) {
        out[clean] = normalizePlotValue(val, depth + 1);
      }
    }
    return out;
  }
  return value;
}

function normalizePlotMessage(msg: unknown): unknown {
  if (typeof msg === "number" && Number.isFinite(msg)) {
    return { data: msg };
  }
  if (typeof msg === "boolean") {
    return { data: msg ? 1 : 0 };
  }
  return normalizePlotValue(msg);
}

function summarizePlotMessage(msg: unknown): string {
  if (msg === null) return "null";
  if (msg === undefined) return "undefined";
  if (typeof msg === "string") return `string:${msg.slice(0, 80)}`;
  if (typeof msg === "number" || typeof msg === "boolean") return String(msg);
  if (Array.isArray(msg)) return `array(len=${msg.length})`;
  if (typeof msg === "object") {
    const keys = Object.keys(msg as Record<string, unknown>).slice(0, 8);
    return `object{${keys.join(",")}${keys.length >= 8 ? ",..." : ""}}`;
  }
  return typeof msg;
}

// drawPointCloud function removed - using 3D viewer instead

// Components
const ControlPanel = memo(function ControlPanel({
  topics,
  selectedTopic,
  setSelectedTopic,
  mode,
  onModeChange,
  running,
  paused,
  onRefresh,
  onStart,
  onStop,
  onPause,
  status,
  isError,
  panelCollapsed,
  onToggleCollapsed,
  extraControls,
}: {
  topics: Topic[];
  selectedTopic: string;
  setSelectedTopic: (t: string) => void;
  mode: Mode;
  onModeChange: (m: Mode) => void;
  running: boolean;
  paused: boolean;
  onRefresh: () => void;
  onStart: () => void;
  onStop: () => void;
  onPause: () => void;
  status: string;
  isError: boolean;
  panelCollapsed: boolean;
  onToggleCollapsed: () => void;
  extraControls?: ComponentChildren;
}) {
  const filteredTopics =
    mode === "plot" ? topics : topics.filter((t) => modeForTopicType(t.type) === mode);

  return (
    <section class={`panel control-panel ${panelCollapsed ? "panel-collapsed" : ""}`}>
      <div class="panel-header">
        <span class="panel-title">Controls</span>
        <button
          class="icon-btn"
          onClick={onToggleCollapsed}
          aria-label={panelCollapsed ? "Expand panel" : "Collapse panel"}
          title={panelCollapsed ? "Expand" : "Collapse"}
        >
          {panelCollapsed ? ">" : "<"}
        </button>
      </div>
      <div class="panel-body">
        <div class="row">
          <div>
            <label>View</label>
            <select value={mode} onChange={(e) => onModeChange((e.target as HTMLSelectElement).value as Mode)}>
              <option value="image">Image</option>
              <option value="pointcloud">PointCloud</option>
              <option value="plot">Plot</option>
            </select>
          </div>
        </div>
        <div class="row">
          <div>
            <label>Topic</label>
            <select value={selectedTopic} onChange={(e) => setSelectedTopic((e.target as HTMLSelectElement).value)}>
              {filteredTopics.length === 0 && <option value="">No matching topics found</option>}
              {filteredTopics.map((t) => (
                <option key={t.name} value={t.name}>
                  {t.name} ({t.type})
                </option>
              ))}
            </select>
          </div>
        </div>

        <div class="row">
          <button class="primary" onClick={onRefresh}>Refresh topics</button>
          <button class="primary" onClick={onStart} disabled={running}>Start</button>
        </div>
        <div class="row">
          <button class="danger" onClick={onStop} disabled={!running}>Stop</button>
          <button onClick={onPause} disabled={!running}>{paused ? "Resume" : "Pause"}</button>
        </div>

        {extraControls}

        <div class="hint">
          {isError ? (
            <span class="bad">Error</span>
          ) : (
            <span class="ok ok-wrap">
              <span>OK</span>
              <span class="status-dot" aria-hidden="true" />
            </span>
          )}
        </div>
        {isError && status && <div class="status">{status}</div>}
      </div>
    </section>
  );
});

function ImageViewer({ src }: { src: string }) {
  return (
    <div id="imageView">
      <img id="img" src={src} alt="stream" />
    </div>
  );
}

function PlotControls({
  fields,
  selected,
  onToggleField,
  onAddField,
  onClearPlot,
  onReloadFields,
  loading,
  schemaError,
  maxPoints,
  onMaxPointsChange,
  timeWindowSec,
  onTimeWindowSecChange,
}: {
  fields: string[];
  selected: string[];
  onToggleField: (path: string) => void;
  onAddField: (path: string) => void;
  onClearPlot: () => void;
  onReloadFields: () => void;
  loading: boolean;
  schemaError: string | null;
  maxPoints: number;
  onMaxPointsChange: (value: number) => void;
  timeWindowSec: number;
  onTimeWindowSecChange: (value: number) => void;
}) {
  const [picked, setPicked] = useState("");
  const [openSuggest, setOpenSuggest] = useState(false);
  const query = picked.trim().toLowerCase();
  const suggestions = query
    ? fields.filter((path) => path.toLowerCase().includes(query))
    : fields;
  const visibleSuggestions = suggestions.slice(0, 200);

  const addPicked = useCallback(() => {
    if (!picked) return;
    onAddField(picked);
    setPicked("");
  }, [picked, onAddField]);

  return (
    <div class="plot-controls">
      <div class="field-meta">
        Fields: {fields.length} | Selected: {selected.length}
        {loading ? " | Loading schema..." : ""}
      </div>
      {schemaError && <div class="status">{schemaError}</div>}
      <div class="row">
        <div>
          <label>Available fields</label>
          <div class="combo">
            <input
              value={picked}
              onInput={(e) => {
                setPicked((e.target as HTMLInputElement).value);
                setOpenSuggest(true);
              }}
              onFocus={() => setOpenSuggest(true)}
              onBlur={() => setOpenSuggest(false)}
              onKeyDown={(e) => {
                if (e.key === "Enter") {
                  e.preventDefault();
                  addPicked();
                  setOpenSuggest(false);
                }
              }}
              placeholder="Type to search fields"
              aria-expanded={openSuggest}
              aria-controls="plot-field-list"
              role="combobox"
            />
            {openSuggest && (
              <div id="plot-field-list" class="field-suggest">
                {visibleSuggestions.length === 0 ? (
                  <div class="field-suggest-empty">No matches</div>
                ) : (
                  visibleSuggestions.map((path) => (
                    <button
                      type="button"
                      class={`field-suggest-item ${selected.includes(path) ? "is-selected" : ""}`}
                      key={path}
                      onMouseDown={(e) => e.preventDefault()}
                      onClick={() => {
                        setPicked(path);
                        setOpenSuggest(false);
                      }}
                    >
                      {path}
                    </button>
                  ))
                )}
              </div>
            )}
          </div>
        </div>
        <button class="auto" onClick={addPicked} disabled={!picked}>Add</button>
      </div>
      <div class="row">
        <button class="primary" onClick={onReloadFields} disabled={loading}>Reload fields</button>
      </div>
      {fields.length === 0 && (
        <div class="muted">No fields yet. Start stream or add a path manually.</div>
      )}
      <div class="field-list">
        {selected.length === 0 && <div class="muted">No fields selected yet.</div>}
        {selected.map((path) => (
          <label class="field-item" key={path}>
            <input
              type="checkbox"
              checked={selected.includes(path)}
              onChange={() => onToggleField(path)}
            />
            <span class="field-name">{path}</span>
          </label>
        ))}
      </div>
      <div class="row">
        <button class="danger" onClick={onClearPlot}>Clear plot</button>
      </div>
      <details class="advanced">
        <summary class="panel-title">Advanced</summary>
        <div class="row">
          <div>
            <label>Max points</label>
            <input
              type="number"
              min="0"
              max={PLOT_MAX_POINTS_HARD}
              value={maxPoints}
              onInput={(e) => {
                const raw = Number((e.target as HTMLInputElement).value);
                if (Number.isFinite(raw)) {
                  onMaxPointsChange(Math.max(0, Math.min(PLOT_MAX_POINTS_HARD, Math.floor(raw))));
                }
              }}
            />
          </div>
        </div>
        <div class="row">
          <div>
            <label>Time window (s)</label>
            <input
              type="number"
              min="0"
              step="0.1"
              value={timeWindowSec}
              onInput={(e) => {
                const raw = Number((e.target as HTMLInputElement).value);
                if (Number.isFinite(raw)) {
                  onTimeWindowSecChange(Math.max(0, raw));
                }
              }}
            />
            <div class="muted">0 = no limit</div>
          </div>
        </div>
      </details>
    </div>
  );
}

function PlotViewer({
  series,
  paused,
}: {
  series: PlotSeries[];
  paused: boolean;
}) {
  const canvasRef = useRef<HTMLCanvasElement | null>(null);
  const wrapRef = useRef<HTMLDivElement | null>(null);
  const [size, setSize] = useState({ w: 300, h: 200 });

  useEffect(() => {
    const el = wrapRef.current;
    if (!el) return;
    const ro = new ResizeObserver((entries) => {
      const entry = entries[0];
      if (!entry) return;
      const { width, height } = entry.contentRect;
      setSize({ w: Math.max(1, Math.floor(width)), h: Math.max(1, Math.floor(height)) });
    });
    ro.observe(el);
    return () => ro.disconnect();
  }, []);

  useEffect(() => {
    const canvas = canvasRef.current;
    if (!canvas) return;
    const ctx = canvas.getContext("2d");
    if (!ctx) return;
    const dpr = window.devicePixelRatio || 1;
    canvas.width = Math.max(1, Math.floor(size.w * dpr));
    canvas.height = Math.max(1, Math.floor(size.h * dpr));
    ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
    ctx.clearRect(0, 0, size.w, size.h);

    ctx.fillStyle = "#05070a";
    ctx.fillRect(0, 0, size.w, size.h);

    const padding = { left: 48, right: 12, top: 12, bottom: 26 };
    const plotW = Math.max(1, size.w - padding.left - padding.right);
    const plotH = Math.max(1, size.h - padding.top - padding.bottom);

    let xMin = Number.POSITIVE_INFINITY;
    let xMax = Number.NEGATIVE_INFINITY;
    let yMin = Number.POSITIVE_INFINITY;
    let yMax = Number.NEGATIVE_INFINITY;

    for (const s of series) {
      for (const p of s.points) {
        if (p.t < xMin) xMin = p.t;
        if (p.t > xMax) xMax = p.t;
        if (p.y < yMin) yMin = p.y;
        if (p.y > yMax) yMax = p.y;
      }
    }

    if (!Number.isFinite(xMin) || !Number.isFinite(yMin)) {
      ctx.fillStyle = "#94a3b8";
      ctx.font = "12px system-ui, -apple-system, Segoe UI, Roboto, sans-serif";
      ctx.fillText("No plot data yet.", padding.left, padding.top + 16);
      return;
    }

    if (xMax <= xMin) xMax = xMin + 1;
    if (yMax <= yMin) yMax = yMin + 1;

    ctx.strokeStyle = "rgba(148,163,184,0.2)";
    ctx.lineWidth = 1;
    ctx.beginPath();
    for (let i = 0; i <= 4; i++) {
      const y = padding.top + (plotH / 4) * i;
      ctx.moveTo(padding.left, y);
      ctx.lineTo(padding.left + plotW, y);
    }
    ctx.stroke();

    ctx.beginPath();
    for (let i = 0; i <= 5; i++) {
      const x = padding.left + (plotW / 5) * i;
      ctx.moveTo(x, padding.top);
      ctx.lineTo(x, padding.top + plotH);
    }
    ctx.stroke();

    for (const s of series) {
      ctx.strokeStyle = s.color;
      ctx.lineWidth = 1.6;
      if (s.points.length >= 2) {
        ctx.beginPath();
        s.points.forEach((p, idx) => {
          const x = padding.left + ((p.t - xMin) / (xMax - xMin)) * plotW;
          const y = padding.top + (1 - (p.y - yMin) / (yMax - yMin)) * plotH;
          if (idx === 0) ctx.moveTo(x, y);
          else ctx.lineTo(x, y);
        });
        ctx.stroke();
      }
      const last = s.points[s.points.length - 1];
      if (last) {
        const x = padding.left + ((last.t - xMin) / (xMax - xMin)) * plotW;
        const y = padding.top + (1 - (last.y - yMin) / (yMax - yMin)) * plotH;
        ctx.fillStyle = s.color;
        ctx.beginPath();
        ctx.arc(x, y, 3, 0, Math.PI * 2);
        ctx.fill();
      }
    }

    ctx.fillStyle = "#94a3b8";
    ctx.font = "11px system-ui, -apple-system, Segoe UI, Roboto, sans-serif";
    ctx.fillText(`${yMax.toFixed(3)}`, 8, padding.top + 10);
    ctx.fillText(`${yMin.toFixed(3)}`, 8, padding.top + plotH);
    ctx.fillText(`${xMin.toFixed(2)}s`, padding.left, size.h - 8);
    ctx.fillText(`${xMax.toFixed(2)}s`, padding.left + plotW - 42, size.h - 8);

    if (paused) {
      ctx.fillStyle = "rgba(0,0,0,0.35)";
      ctx.fillRect(0, 0, size.w, size.h);
      ctx.fillStyle = "#e5e7eb";
      ctx.font = "bold 14px system-ui, -apple-system, Segoe UI, Roboto, sans-serif";
      ctx.fillText("Paused", padding.left, padding.top + 20);
    }
  }, [series, paused, size]);

  return (
    <div id="plotView" ref={wrapRef}>
      <canvas id="plotCanvas" ref={canvasRef} />
      <div class="plot-legend">
        {series.map((s) => (
          <div class="legend-item" key={s.path}>
            <span class="legend-swatch" style={{ background: s.color }} />
            <span class="legend-label">{s.path}</span>
          </div>
        ))}
      </div>
    </div>
  );
}

// PointCloudViewer component removed - using PointCloudViewer3D instead

function App() {
  // State
  const [mode, setMode] = useState<Mode>("image");
  const [topics, setTopics] = useState<Topic[]>([]);
  const [selectedTopic, setSelectedTopic] = useState("");
  
  const [running, setRunning] = useState(false);
  const [paused, setPaused] = useState(false);
  const [status, setStatus] = useState("");
  const [isError, setIsError] = useState(false);
  const [panelCollapsed, setPanelCollapsed] = useState(false);
  const rawViewerConfig = (globalThis as { __ROS2_VIEWER_CONFIG__?: ViewerConfig }).__ROS2_VIEWER_CONFIG__;
  const embeddedConfig = normalizeViewerConfig(rawViewerConfig);
  const [remoteConfig, setRemoteConfig] = useState<ViewerConfig | undefined>(undefined);
  const viewerConfig = remoteConfig ?? embeddedConfig;
  const autoStartRef = useRef(false);
  const autoCollapseRef = useRef(false);
  const userSelectedRef = useRef(false);
  const restartInFlightRef = useRef(false);
  const currentStreamTopicRef = useRef<string>("");
  const configFetchRef = useRef(false);
  const lastConfigSigRef = useRef<string | null>(configSignature(embeddedConfig));
  
  const [imageSrc, setImageSrc] = useState("");
  const [pointCloudData, setPointCloudData] = useState<PointCloudData | null>(null);
  const [plotFields, setPlotFields] = useState<string[]>([]);
  const plotFieldSetRef = useRef<Set<string>>(new Set());
  const [selectedPaths, setSelectedPaths] = useState<string[]>([]);
  const selectedPathsRef = useRef<string[]>([]);
  const [plotSeries, setPlotSeries] = useState<PlotSeries[]>([]);
  const plotSeriesRef = useRef<PlotSeries[]>([]);
  const plotBaseTimeRef = useRef<number | null>(null);
  const plotTopicRef = useRef<string>("");
  const [plotSchemaLoading, setPlotSchemaLoading] = useState(false);
  const [plotSchemaError, setPlotSchemaError] = useState<string | null>(null);
  const schemaCacheRef = useRef<Map<string, string[]>>(new Map());
  const schemaRequestRef = useRef(0);
  const userSelectedFieldsRef = useRef(false);
  const plotHasConfigRef = useRef(false);
  const userModeRef = useRef(false);
  const [debugLines, setDebugLines] = useState<string[]>([]);
  const streamModeRef = useRef<Mode | null>(null);
  const plotLogStateRef = useRef<{ lastMs: number; count: number }>({ lastMs: 0, count: 0 });
  const plotConfigTopicRef = useRef<string | null>(null);
  const [plotMaxPoints, setPlotMaxPoints] = useState<number>(PLOT_MAX_POINTS);
  const [plotWindowSec, setPlotWindowSec] = useState<number>(0);
  const plotMaxPointsRef = useRef<number>(PLOT_MAX_POINTS);
  const plotWindowSecRef = useRef<number>(0);

  // Refs for streaming
  const sessionIdRef = useRef<string | null>(null);
  const seqRef = useRef(0);
  const pollingRef = useRef(false);
  const modeRef = useRef<Mode>(mode);

  // Keep modeRef in sync
  useEffect(() => {
    modeRef.current = mode;
  }, [mode]);
  useEffect(() => {
    selectedPathsRef.current = selectedPaths;
  }, [selectedPaths]);
  useEffect(() => {
    plotSeriesRef.current = plotSeries;
  }, [plotSeries]);
  useEffect(() => {
    plotMaxPointsRef.current = plotMaxPoints;
  }, [plotMaxPoints]);
  useEffect(() => {
    plotWindowSecRef.current = plotWindowSec;
  }, [plotWindowSec]);
  useEffect(() => {
    if (modeRef.current === "plot") return;
    if (userModeRef.current) return;
    if (!selectedTopic) return;
    const selected = topics.find((t) => t.name === selectedTopic);
    const nextMode = selected ? modeForTopicType(selected.type) : null;
    if (nextMode && nextMode !== modeRef.current) {
      setMode(nextMode);
    }
  }, [selectedTopic, topics]);

  const updateStatus = useCallback((msg: unknown, error = false) => {
    if (!error) {
      setIsError(false);
      setStatus("");
      return;
    }
    let text = "Error";
    if (msg instanceof Error) {
      text = msg.message;
    } else if (msg && typeof msg === "object") {
      const rec = msg as Record<string, unknown>;
      if (typeof rec.error === "string") {
        text = rec.error;
      } else if (typeof rec.message === "string") {
        text = rec.message;
      }
    } else if (typeof msg === "string") {
      text = msg;
    }
    setStatus(text);
    setIsError(true);
  }, []);

  const logLine = useCallback((msg: string) => {
    const stamp = new Date().toLocaleTimeString();
    setDebugLines((prev) => {
      const next = [...prev, `[${stamp}] ${msg}`];
      return next.length > 200 ? next.slice(next.length - 200) : next;
    });
  }, []);

  const applyRemoteConfig = useCallback((cfg?: ViewerConfig) => {
    const sig = configSignature(cfg);
    if (!sig) return;
    if (sig === lastConfigSigRef.current) return;
    const normalized = normalizeViewerConfig(cfg);
    if (!normalized) return;
    lastConfigSigRef.current = sig;
    setRemoteConfig(normalized);
    userModeRef.current = false;
    userSelectedRef.current = false;
    autoStartRef.current = false;
    autoCollapseRef.current = false;
    userSelectedFieldsRef.current = false;
    plotHasConfigRef.current = false;
    plotConfigTopicRef.current = null;
    plotTopicRef.current = "";
  }, []);

  const fetchViewerConfig = useCallback(async () => {
    try {
      const res = await callTool("ros2_viewer_config", {});
      const txt = extractFirstText(res);
      const parsed = txt ? JSON.parse(txt) : {};
      const cfg = normalizeViewerConfig(parsed);
      if (cfg) {
        applyRemoteConfig(cfg);
      }
    } catch {
      // Ignore missing tool / fetch errors; UI will fall back to defaults.
    }
  }, [applyRemoteConfig]);

  const clearStreamState = useCallback(() => {
    sessionIdRef.current = null;
    seqRef.current = 0;
    setRunning(false);
    setPaused(false);
    currentStreamTopicRef.current = "";
    streamModeRef.current = null;
  }, []);

  const resetPlotState = useCallback(() => {
    plotFieldSetRef.current = new Set();
    setPlotFields([]);
    setSelectedPaths([]);
    setPlotSeries([]);
    plotBaseTimeRef.current = null;
    setPlotSchemaError(null);
    userSelectedFieldsRef.current = false;
    plotHasConfigRef.current = false;
    plotConfigTopicRef.current = null;
  }, []);

  const mergePlotFields = useCallback((paths: string[]) => {
    if (!paths.length) return;
    let changed = false;
    for (const path of paths) {
      if (!plotFieldSetRef.current.has(path)) {
        plotFieldSetRef.current.add(path);
        changed = true;
      }
    }
    if (changed) {
      const next = Array.from(plotFieldSetRef.current);
      next.sort();
      setPlotFields(next);
      if (
        !userSelectedFieldsRef.current &&
        !plotHasConfigRef.current &&
        selectedPathsRef.current.length === 0 &&
        next.length > 0
      ) {
        setSelectedPaths([next[0]]);
        logLine(`Auto-selected field: ${next[0]}`);
      }
    }
  }, [logLine]);

  const handleModeChange = useCallback((next: Mode) => {
    userModeRef.current = true;
    setMode(next);
  }, []);

  const togglePlotField = useCallback((path: string) => {
    userSelectedFieldsRef.current = true;
    setSelectedPaths((prev) =>
      prev.includes(path) ? prev.filter((p) => p !== path) : [...prev, path]
    );
  }, []);

  const addPlotField = useCallback((path: string) => {
    const clean = path.trim();
    if (!clean) return;
    userSelectedFieldsRef.current = true;
    if (!plotFieldSetRef.current.has(clean)) {
      plotFieldSetRef.current.add(clean);
      const next = Array.from(plotFieldSetRef.current);
      next.sort();
      setPlotFields(next);
    }
    setSelectedPaths((prev) => (prev.includes(clean) ? prev : [...prev, clean]));
  }, []);

  const clearPlot = useCallback(() => {
    setPlotSeries((prev) => prev.map((s) => ({ ...s, points: [] })));
    plotBaseTimeRef.current = null;
  }, []);

  const updatePlotFields = useCallback((msg: unknown) => {
    const streamTopic = currentStreamTopicRef.current;
    if (streamTopic && streamTopic !== selectedTopic) {
      return;
    }
    const paths: string[] = [];
    collectNumericPaths(msg, "", paths, 0);
    mergePlotFields(paths);
  }, [mergePlotFields, selectedTopic]);

  const appendPlotPoints = useCallback((msg: unknown, stamp?: { sec: number; nanosec: number }) => {
    const selected = selectedPathsRef.current;
    if (!selected.length) return;
    const base = plotBaseTimeRef.current;
    const tAbs = stamp ? stamp.sec + stamp.nanosec * 1e-9 : performance.now() / 1000;
    if (base === null) {
      plotBaseTimeRef.current = tAbs;
    }
    const t = tAbs - (plotBaseTimeRef.current ?? tAbs);
    const nextSeries = plotSeriesRef.current.map((s) => ({
      ...s,
      points: s.points.slice(),
    }));
    const map = new Map(nextSeries.map((s) => [s.path, s]));
    for (const path of selected) {
      const raw = getValueByPath(msg, path);
      const val = toNumber(raw);
      if (val === null) continue;
      const series = map.get(path);
      if (!series) continue;
      series.points.push({ t, y: val });
      const maxPoints = Math.max(0, Math.min(PLOT_MAX_POINTS_HARD, plotMaxPointsRef.current || 0));
      if (maxPoints > 0 && series.points.length > maxPoints) {
        series.points.splice(0, series.points.length - maxPoints);
      } else if (series.points.length > PLOT_MAX_POINTS_HARD) {
        series.points.splice(0, series.points.length - PLOT_MAX_POINTS_HARD);
      }
      const windowSec = plotWindowSecRef.current || 0;
      if (windowSec > 0) {
        const cutoff = t - windowSec;
        let drop = 0;
        while (drop < series.points.length && series.points[drop].t < cutoff) drop++;
        if (drop > 0) series.points.splice(0, drop);
      }
    }
    plotSeriesRef.current = nextSeries;
    setPlotSeries(nextSeries);
  }, []);

  const loadSchemaFieldsForTopic = useCallback(async (topicName: string, messageType: string) => {
    if (!topicName || !messageType) return;
    if (schemaCacheRef.current.has(topicName)) {
      mergePlotFields(schemaCacheRef.current.get(topicName) || []);
      setPlotSchemaLoading(false);
      setPlotSchemaError(null);
      logLine(`Schema cache hit for ${topicName}`);
      return;
    }
    const reqId = ++schemaRequestRef.current;
    setPlotSchemaLoading(true);
    setPlotSchemaError(null);
    logLine(`Loading schema for ${topicName} (${messageType})`);
    const collected: string[] = [];
    const seenTypes = new Set<string>();

    const fetchFields = async (typeStr: string): Promise<Record<string, string> | null> => {
      if (!typeStr || seenTypes.has(typeStr)) return null;
      seenTypes.add(typeStr);
      try {
        const res = await callTool("ros2_get_message_fields", { message_type: typeStr });
        const txt = extractFirstText(res);
        const obj = JSON.parse(txt || "{}");
        if (!obj || typeof obj !== "object" || obj.error) {
          if (obj?.error) {
            logLine(`Schema error for ${typeStr}: ${obj.error}`);
          }
          return null;
        }
        return obj as Record<string, string>;
      } catch {
        logLine(`Schema fetch failed for ${typeStr}`);
        return null;
      }
    };

    const walk = async (typeStr: string, prefix: string, depth: number) => {
      if (collected.length >= PLOT_MAX_FIELDS) return;
      if (depth > PLOT_MAX_DEPTH) return;
      const fields = await fetchFields(typeStr);
      if (!fields) return;
      for (const [name, rawType] of Object.entries(fields)) {
        if (collected.length >= PLOT_MAX_FIELDS) break;
        const { base, isArray } = parseFieldType(rawType);
        const pathBase = prefix ? `${prefix}.${name}` : name;
        if (isNumericType(base)) {
          collected.push(isArray ? `${pathBase}[0]` : pathBase);
        } else if (base.includes("/")) {
          const nestedPrefix = isArray ? `${pathBase}[0]` : pathBase;
          await walk(base, nestedPrefix, depth + 1);
        }
      }
    };

    await walk(messageType, "", 0);
    if (reqId !== schemaRequestRef.current) {
      return;
    }
    if (plotTopicRef.current && plotTopicRef.current !== topicName) {
      return;
    }
    const unique = Array.from(new Set(collected));
    unique.sort();
    schemaCacheRef.current.set(topicName, unique);
    mergePlotFields(unique);
    setPlotSchemaLoading(false);
    if (unique.length === 0) {
      setPlotSchemaError("No numeric fields found in schema.");
    }
    logLine(`Schema fields for ${topicName}: ${unique.length}`);
  }, [mergePlotFields, logLine]);

  useEffect(() => {
    setPlotSeries((prev) => {
      const prevMap = new Map(prev.map((s) => [s.path, s]));
      const next = selectedPaths.map((path) => {
        const existing = prevMap.get(path);
        return existing ?? { path, color: colorForPath(path), points: [] };
      });
      plotSeriesRef.current = next;
      return next;
    });
  }, [selectedPaths]);

  const refreshTopics = useCallback(async () => {
    try {
      logLine("Refreshing topics...");
      const res = await callTool("ros2_topic_list", {});
      const txt = extractFirstText(res);
      const arr = JSON.parse(txt || "[]");
      const parsed: Topic[] = Array.isArray(arr)
        ? arr.map((x: Record<string, unknown>) => ({
            name: (x.topic_name || x.name || "") as string,
            type: (x.topic_type || (x.types as string[])?.[0] || x.type || "") as string,
          }))
        : [];
      setTopics(parsed);
      updateStatus("", false);
      logLine(`Topics loaded: ${parsed.length}`);
    } catch (e) {
      updateStatus(e, true);
      logLine("Failed to load topics.");
    }
  }, [updateStatus, logLine]);

  const pollOnce = useCallback(async () => {
    if (!pollingRef.current || !sessionIdRef.current) return;
    
    try {
      const res = await callTool("ros2_stream_next", {
        session_id: sessionIdRef.current,
        after_seq: seqRef.current,
      });
      const txt = extractFirstText(res);
      const obj: StreamResponse = JSON.parse(txt || "{}");
      
      if (obj.error) {
        updateStatus(obj, true);
        pollingRef.current = false;
        logLine(`Stream error: ${obj.error}`);
        if (sessionIdRef.current) {
          try {
            await callTool("ros2_stream_stop", { session_id: sessionIdRef.current });
          } catch {
            // Ignore stop errors; we'll still reset local state.
          }
        }
        clearStreamState();
        return;
      }
      
      if (!obj.available) {
        return;
      }
      
      seqRef.current = obj.seq || seqRef.current;

      if (modeRef.current === "image") {
        if (obj.data && obj.mimeType) {
          setImageSrc(`data:${obj.mimeType};base64,${obj.data}`);
        }
      } else if (modeRef.current === "pointcloud") {
        if (obj.positionsF32B64) {
          const posBuf = b64ToArrayBuffer(obj.positionsF32B64);
          const positions = new Float32Array(posBuf);
          let colors: Float32Array | undefined;
          if (obj.colorsF32B64) {
            const colorBuf = b64ToArrayBuffer(obj.colorsF32B64);
            colors = new Float32Array(colorBuf);
          }
          setPointCloudData({
            positions,
            colors,
            pointCount: obj.pointCount || positions.length / 3,
            frame_id: obj.frame_id,
          });
        }
      } else {
        if (obj.message !== undefined) {
          const normalized = normalizePlotMessage(obj.message);
          updatePlotFields(normalized);
          appendPlotPoints(normalized, obj.stamp);
          const selected = selectedPathsRef.current;
          if (selected.length > 0) {
            const now = performance.now();
            const lastState = plotLogStateRef.current;
            if (lastState.count < 5 || now - lastState.lastMs > 1000) {
              const samples = selected.slice(0, 4).map((path) => {
                const raw = getValueByPath(normalized, path);
                const num = toNumber(raw);
                if (num !== null) {
                  return `${path}=${num.toFixed(4)}`;
                }
                if (raw === null || raw === undefined) {
                  return `${path}=null`;
                }
                return `${path}=${String(raw)}`;
              });
              logLine(`Plot sample: ${samples.join(", ")}`);
              const anyNonNull = samples.some((s) => !s.endsWith("=null"));
              if (!anyNonNull) {
                logLine(`Plot raw: ${summarizePlotMessage(obj.message)}`);
              }
              plotLogStateRef.current = { lastMs: now, count: lastState.count + 1 };
            }
          }
        }
      }
    } catch (e) {
      updateStatus(e, true);
      logLine("Polling failed.");
    }
  }, [updateStatus, clearStreamState, logLine]);

  const startPolling = useCallback(() => {
    if (!sessionIdRef.current) return;
    if (pollingRef.current) return;
    pollingRef.current = true;
    const pollLoop = async () => {
      if (!pollingRef.current) return;
      await pollOnce();
      if (pollingRef.current) {
        requestAnimationFrame(pollLoop);
      }
    };
    pollLoop();
  }, [pollOnce]);

  const startStreamInternal = useCallback(async (topicName: string) => {
    if (!topicName) {
      updateStatus("Pick a topic first.", true);
      return;
    }

    const shouldAutoCollapse = autoCollapseRef.current;
    autoCollapseRef.current = false;

    try {
      logLine(`Starting stream for ${topicName} (${modeRef.current})`);
      const selected = topics.find((t) => t.name === topicName);
      const inferredMode = selected ? modeForTopicType(selected.type) : null;
      let streamKind: "image" | "pointcloud" | "message";

      if (modeRef.current === "plot") {
        streamKind = "message";
        if (plotTopicRef.current !== topicName) {
          plotTopicRef.current = topicName;
        }
      } else {
        if (!inferredMode) {
          updateStatus("Unsupported topic type for the current view.", true);
          return;
        }
        if (inferredMode !== modeRef.current) {
          if (userModeRef.current) {
            updateStatus("Selected topic does not match the chosen view mode.", true);
            return;
          }
          modeRef.current = inferredMode;
          setMode(inferredMode);
        }
        streamKind = inferredMode;
      }

      const params: Record<string, unknown> = {
        topic_name: topicName,
        kind: streamKind,
        qos_preset: "auto",
      };

      if (streamKind === "image") {
        params.jpeg_quality = 80;
      } else if (streamKind === "message") {
        params.target_fps = 20.0;
      }

      const res = await callTool("ros2_stream_start", params);
      const txt = extractFirstText(res);
      const obj: StreamResponse = JSON.parse(txt || "{}");
      
      if (obj.error) {
        updateStatus(obj, true);
        logLine(`Stream start error: ${obj.error}`);
        return;
      }

      sessionIdRef.current = obj.session_id || null;
      seqRef.current = 0;
      setRunning(true);
      setPaused(false);
      currentStreamTopicRef.current = topicName;
      streamModeRef.current = modeRef.current;
      updateStatus("Streaming.");
      logLine(`Streaming session: ${sessionIdRef.current ?? "unknown"}`);
      if (shouldAutoCollapse) {
        setPanelCollapsed(true);
      }
      startPolling();
    } catch (e) {
      updateStatus(e, true);
      logLine("Stream start failed.");
    }
  }, [topics, updateStatus, startPolling, logLine]);

  const startStream = useCallback(async () => {
    if (running) return;
    if (!selectedTopic) {
      updateStatus("Pick a topic first.", true);
      return;
    }
    await startStreamInternal(selectedTopic);
  }, [running, selectedTopic, startStreamInternal, updateStatus]);

  const stopStreamInternal = useCallback(async () => {
    if (!sessionIdRef.current) return;
    
    try {
      pollingRef.current = false;
      const res = await callTool("ros2_stream_stop", { session_id: sessionIdRef.current });
      const txt = extractFirstText(res);
      const obj = JSON.parse(txt || "{}");
      
      clearStreamState();
      updateStatus("Stopped.");
      if (obj?.error) {
        logLine(`Stop error: ${obj.error}`);
      } else {
        logLine("Stream stopped.");
      }
    } catch (e) {
      updateStatus(e, true);
      logLine("Stop failed.");
    }
  }, [updateStatus, clearStreamState, logLine]);

  const stopStream = useCallback(async () => {
    if (!running || !sessionIdRef.current) return;
    await stopStreamInternal();
  }, [running, stopStreamInternal]);

  const togglePause = useCallback(() => {
    if (!running) return;
    setPaused((p) => {
      const next = !p;
      if (next) {
        pollingRef.current = false;
      } else {
        startPolling();
      }
      return next;
    });
  }, [running, startPolling]);

  // Initial topic refresh
  useEffect(() => {
    refreshTopics();
  }, []);

  // Fetch config from the server if it wasn't embedded (covers cached UI)
  useEffect(() => {
    if (viewerConfig) return;
    if (configFetchRef.current) return;
    configFetchRef.current = true;
    const run = async () => {
      await fetchViewerConfig();
    };
    run();
  }, [viewerConfig, fetchViewerConfig]);

  // Poll for updated config (allows reusing the same UI with new MCP commands)
  useEffect(() => {
    let active = true;
    let timer: number | undefined;
    const loop = async () => {
      await fetchViewerConfig();
      if (!active) return;
      timer = window.setTimeout(loop, 1500);
    };
    loop();
    return () => {
      active = false;
      if (timer) window.clearTimeout(timer);
    };
  }, [fetchViewerConfig]);

  useEffect(() => {
    if (!viewerConfig?.preferredKind) return;
    if (viewerConfig.preferredKind === "auto") return;
    if (userModeRef.current) return;
    if (viewerConfig.preferredKind === "plot") {
      setMode("plot");
      return;
    }
    if (viewerConfig.preferredKind === "image" || viewerConfig.preferredKind === "pointcloud") {
      setMode(viewerConfig.preferredKind);
    }
  }, [viewerConfig]);

  const handleUserSelectTopic = useCallback((topic: string) => {
    userSelectedRef.current = true;
    setSelectedTopic(topic);
  }, []);

  // Keep selection on supported topics (prefer configured topic/type when empty)
  useEffect(() => {
    const selected = selectedTopic ? topics.find((t) => t.name === selectedTopic) : null;
    const selectedValid = isTopicCompatible(selected, mode);
    if (!selectedValid) {
      userSelectedRef.current = false;
    }
    if (!userSelectedRef.current && viewerConfig?.topicName) {
      const desired = topics.find((t) => t.name === viewerConfig.topicName)?.name;
      if (desired && desired !== selectedTopic) {
        setSelectedTopic(desired);
        return;
      }
    }
    if (selectedValid) {
      return;
    }
    let preferred: string | undefined;
    if (!preferred) {
      const kind = viewerConfig?.preferredKind;
      if (kind === "image") {
        preferred = topics.find((t) => modeForTopicType(t.type) === "image")?.name;
      } else if (kind === "pointcloud") {
        preferred = topics.find((t) => modeForTopicType(t.type) === "pointcloud")?.name;
      } else if (kind === "plot") {
        preferred = topics.find((t) => modeForTopicType(t.type) === null)?.name ?? topics[0]?.name;
      }
    }
    if (!preferred) {
      if (mode === "plot") {
        preferred = topics[0]?.name;
      } else {
        preferred = topics.find((t) => modeForTopicType(t.type) === mode)?.name;
      }
    }
    setSelectedTopic(preferred || "");
  }, [topics, selectedTopic, viewerConfig, mode]);

  useEffect(() => {
    if (mode !== "plot") return;
    if (!selectedTopic) return;
    if (plotTopicRef.current === selectedTopic) return;
    plotTopicRef.current = selectedTopic;
    resetPlotState();
    const topic = topics.find((t) => t.name === selectedTopic);
    if (topic?.type) {
      loadSchemaFieldsForTopic(selectedTopic, topic.type);
    } else {
      logLine("Plot: selected topic has no type info.");
    }
    const cfgFields = viewerConfig?.plotFields;
    if (cfgFields && cfgFields.length > 0 && plotConfigTopicRef.current !== selectedTopic) {
      const cleaned = Array.from(
        new Set(cfgFields.map((f) => String(f).trim()).filter((f) => f.length > 0))
      );
      if (cleaned.length > 0) {
        plotConfigTopicRef.current = selectedTopic;
        userSelectedFieldsRef.current = true;
        plotHasConfigRef.current = true;
        mergePlotFields(cleaned);
        setSelectedPaths(cleaned);
      }
    }
  }, [
    mode,
    selectedTopic,
    topics,
    resetPlotState,
    loadSchemaFieldsForTopic,
    logLine,
    viewerConfig,
    mergePlotFields,
  ]);

  // Auto-start if requested via config
  useEffect(() => {
    if (!viewerConfig?.autoStart) return;
    if (autoStartRef.current) return;
    if (!selectedTopic) return;
    if (running) {
      autoStartRef.current = true;
      return;
    }
    autoStartRef.current = true;
    autoCollapseRef.current = true;
    startStream();
  }, [viewerConfig, selectedTopic, running, startStream]);

  // Restart stream if the selected topic changes while running
  useEffect(() => {
    if (!running) return;
    if (!selectedTopic) return;
    if (selectedTopic === currentStreamTopicRef.current) return;
    if (restartInFlightRef.current) return;
    restartInFlightRef.current = true;
    const run = async () => {
      await stopStreamInternal();
      await startStreamInternal(selectedTopic);
      restartInFlightRef.current = false;
    };
    run();
  }, [selectedTopic, running, stopStreamInternal, startStreamInternal]);

  useEffect(() => {
    if (!running) return;
    if (!selectedTopic) return;
    if (restartInFlightRef.current) return;
    if (streamModeRef.current === mode) return;
    restartInFlightRef.current = true;
    const run = async () => {
      logLine(`Mode changed: restarting stream (${streamModeRef.current ?? "none"} -> ${mode})`);
      await stopStreamInternal();
      await startStreamInternal(selectedTopic);
      restartInFlightRef.current = false;
    };
    run();
  }, [mode, running, selectedTopic, stopStreamInternal, startStreamInternal, logLine]);

  const plotControls = useMemo(() => {
    if (mode !== "plot") return undefined;
    return (
      <PlotControls
        fields={plotFields}
        selected={selectedPaths}
        onToggleField={togglePlotField}
        onAddField={addPlotField}
        onClearPlot={clearPlot}
        onReloadFields={() => {
          const topic = topics.find((t) => t.name === selectedTopic);
          if (topic?.type) {
            loadSchemaFieldsForTopic(selectedTopic, topic.type);
          } else {
            setPlotSchemaError("Topic type not available.");
            logLine("Reload fields: missing topic type.");
          }
        }}
        loading={plotSchemaLoading}
        schemaError={plotSchemaError}
        maxPoints={plotMaxPoints}
        onMaxPointsChange={setPlotMaxPoints}
        timeWindowSec={plotWindowSec}
        onTimeWindowSecChange={setPlotWindowSec}
      />
    );
  }, [
    mode,
    plotFields,
    selectedPaths,
    togglePlotField,
    addPlotField,
    clearPlot,
    plotSchemaLoading,
    plotSchemaError,
    topics,
    selectedTopic,
    loadSchemaFieldsForTopic,
    logLine,
    plotMaxPoints,
    plotWindowSec,
  ]);

  return (
    <div class="wrap">
      <header>
        <h1>ROS 2 Viewer</h1>
        <span class="pill">MCP App</span>
        <span class="pill">Image / PointCloud2 / Plot</span>
      </header>
      <main class={panelCollapsed ? "is-collapsed" : ""}>
        <ControlPanel
          topics={topics}
          selectedTopic={selectedTopic}
          setSelectedTopic={handleUserSelectTopic}
          mode={mode}
          onModeChange={handleModeChange}
          running={running}
          paused={paused}
          onRefresh={refreshTopics}
          onStart={startStream}
          onStop={stopStream}
          onPause={togglePause}
          status={status}
          isError={isError}
          panelCollapsed={panelCollapsed}
          onToggleCollapsed={() => setPanelCollapsed((v) => !v)}
          extraControls={plotControls}
        />
        <section class="panel viewer">
          {mode === "image" ? (
            <ImageViewer src={imageSrc} />
          ) : mode === "pointcloud" ? (
            <PointCloudViewer3D data={pointCloudData} pointSize={0.01} />
          ) : (
            <PlotViewer series={plotSeries} paused={paused} />
          )}
        </section>
      </main>
    </div>
  );
}

// Mount app
render(<App />, document.body);
