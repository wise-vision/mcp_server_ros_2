# ROS 2 Viewer App (UI)

This folder contains the **editable UI sources** (`src/*.ts(x)`) and a tiny build
step that generates a **single, self-contained** `index.html` for the MCP App
resource.

## Why
MCP Apps usually need one HTML blob (no separate asset server), so `index.html`
is the runtime artifact returned by the `ros2_viewer_app` tool.

## Edit workflow
- Edit `src/app.tsx` (and optionally add more TS/TSX files under `src/`)
- Run:
  - `npm install`
  - `npm run build`

This regenerates `index.html` from `index.template.html` + bundled JS.

