/*
 * Copyright (C) 2025 wisevision
 *
 * SPDX-License-Identifier: MPL-2.0
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 */

import { render } from "preact";
import { useState, useEffect, useRef, useCallback } from "preact/hooks";
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

type Mode = "image" | "pointcloud";

interface ViewerConfig {
  autoStart?: boolean;
  preferredKind?: "image" | "pointcloud" | "auto";
  topicName?: string;
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

function b64ToArrayBuffer(b64: string): ArrayBuffer {
  const bin = atob(b64);
  const bytes = new Uint8Array(bin.length);
  for (let i = 0; i < bin.length; i++) bytes[i] = bin.charCodeAt(i);
  return bytes.buffer;
}

// drawPointCloud function removed - using 3D viewer instead

// Components
function ControlPanel({
  topics,
  selectedTopic,
  setSelectedTopic,
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
}: {
  topics: Topic[];
  selectedTopic: string;
  setSelectedTopic: (t: string) => void;
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
}) {
  const filteredTopics = topics.filter((t) => modeForTopicType(t.type) !== null);

  return (
    <section class={`panel ${panelCollapsed ? "panel-collapsed" : ""}`}>
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
}

function ImageViewer({ src }: { src: string }) {
  return (
    <div id="imageView">
      <img id="img" src={src} alt="stream" />
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
  const viewerConfig = (globalThis as { __ROS2_VIEWER_CONFIG__?: ViewerConfig }).__ROS2_VIEWER_CONFIG__;
  const autoStartRef = useRef(false);
  const lastAutoStartTopicRef = useRef<string>("");
  const autoCollapseRef = useRef(false);
  
  const [imageSrc, setImageSrc] = useState("");
  const [pointCloudData, setPointCloudData] = useState<PointCloudData | null>(null);

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
    if (!selectedTopic) return;
    const selected = topics.find((t) => t.name === selectedTopic);
    const nextMode = selected ? modeForTopicType(selected.type) : null;
    if (nextMode && nextMode !== mode) {
      setMode(nextMode);
    }
  }, [selectedTopic, topics, mode]);

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

  const refreshTopics = useCallback(async () => {
    try {
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
    } catch (e) {
      updateStatus(e, true);
    }
  }, [updateStatus]);

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
      } else {
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
      }
    } catch (e) {
      updateStatus(e, true);
    }
  }, [updateStatus]);

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

  const startStream = useCallback(async () => {
    if (running) return;
    if (!selectedTopic) {
      updateStatus("Pick a topic first.", true);
      return;
    }

    const shouldAutoCollapse = autoCollapseRef.current;
    autoCollapseRef.current = false;

    try {
      const selected = topics.find((t) => t.name === selectedTopic);
      const inferredMode = selected ? modeForTopicType(selected.type) : null;
      if (!inferredMode) {
        updateStatus("Unsupported topic type.", true);
        return;
      }
      if (inferredMode !== modeRef.current) {
        modeRef.current = inferredMode;
        setMode(inferredMode);
      }
      const params: Record<string, unknown> = {
        topic_name: selectedTopic,
        kind: inferredMode,
        qos_preset: "auto",
      };
      
      if (inferredMode === "image") {
        params.jpeg_quality = 80;
      }

      const res = await callTool("ros2_stream_start", params);
      const txt = extractFirstText(res);
      const obj: StreamResponse = JSON.parse(txt || "{}");
      
      if (obj.error) {
        updateStatus(obj, true);
        return;
      }

      sessionIdRef.current = obj.session_id || null;
      seqRef.current = 0;
      setRunning(true);
      setPaused(false);
      updateStatus("Streaming.");
      if (shouldAutoCollapse) {
        setPanelCollapsed(true);
      }
      startPolling();
    } catch (e) {
      updateStatus(e, true);
    }
  }, [running, selectedTopic, topics, updateStatus, startPolling]);

  const stopStream = useCallback(async () => {
    if (!running || !sessionIdRef.current) return;
    
    try {
      pollingRef.current = false;
      const res = await callTool("ros2_stream_stop", { session_id: sessionIdRef.current });
      const txt = extractFirstText(res);
      const obj = JSON.parse(txt || "{}");
      
      sessionIdRef.current = null;
      seqRef.current = 0;
      setRunning(false);
      setPaused(false);
      updateStatus("Stopped.");
    } catch (e) {
      updateStatus(e, true);
    }
  }, [running, updateStatus]);

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

  // Keep selection on supported topics (prefer configured topic/type when empty)
  useEffect(() => {
    const selected = selectedTopic ? topics.find((t) => t.name === selectedTopic) : null;
    if (selected && modeForTopicType(selected.type) !== null) {
      return;
    }
    let preferred: string | undefined;
    if (viewerConfig?.topicName) {
      preferred = topics.find((t) => t.name === viewerConfig.topicName)?.name;
    }
    if (!preferred) {
      const kind = viewerConfig?.preferredKind;
      if (kind === "image") {
        preferred = topics.find((t) => modeForTopicType(t.type) === "image")?.name;
      } else if (kind === "pointcloud") {
        preferred = topics.find((t) => modeForTopicType(t.type) === "pointcloud")?.name;
      } else {
        preferred = topics.find((t) => modeForTopicType(t.type) !== null)?.name;
      }
    }
    setSelectedTopic(preferred || "");
  }, [topics, selectedTopic]);

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

  // Auto-start when user picks a topic
  useEffect(() => {
    if (!selectedTopic) return;
    if (running) return;
    if (lastAutoStartTopicRef.current === selectedTopic) return;
    lastAutoStartTopicRef.current = selectedTopic;
    autoCollapseRef.current = true;
    startStream();
  }, [selectedTopic, running, startStream]);

  return (
    <div class="wrap">
      <header>
        <h1>ROS 2 Viewer</h1>
        <span class="pill">MCP App</span>
        <span class="pill">Image / PointCloud2</span>
      </header>
      <main class={panelCollapsed ? "is-collapsed" : ""}>
        <ControlPanel
          topics={topics}
          selectedTopic={selectedTopic}
          setSelectedTopic={setSelectedTopic}
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
        />
        <section class="panel viewer">
          {mode === "image" ? (
            <ImageViewer src={imageSrc} />
          ) : (
            <PointCloudViewer3D data={pointCloudData} pointSize={0.01} />
          )}
        </section>
      </main>
    </div>
  );
}

// Mount app
render(<App />, document.body);
