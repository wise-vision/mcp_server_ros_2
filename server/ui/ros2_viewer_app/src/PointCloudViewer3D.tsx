/*
 * Copyright (C) 2025 wisevision
 *
 * SPDX-License-Identifier: MPL-2.0
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 */

import { useRef, useEffect } from "preact/hooks";
import * as THREE from "three";
import { OrbitControls } from "three/examples/jsm/controls/OrbitControls.js";

export interface PointCloudData {
  positions: Float32Array;
  colors?: Float32Array;
  pointCount: number;
  frame_id?: string;
}

interface PointCloudViewer3DProps {
  data: PointCloudData | null;
  pointSize?: number;
}

export function PointCloudViewer3D({ data, pointSize = 3 }: PointCloudViewer3DProps) {
  const containerRef = useRef<HTMLDivElement>(null);
  const rendererRef = useRef<THREE.WebGLRenderer | null>(null);
  const sceneRef = useRef<THREE.Scene | null>(null);
  const cameraRef = useRef<THREE.PerspectiveCamera | null>(null);
  const controlsRef = useRef<OrbitControls | null>(null);
  const animationFrameRef = useRef<number | null>(null);
  const pointsMeshRef = useRef<THREE.Points | null>(null);

  // Initialize Three.js scene
  useEffect(() => {
    if (!containerRef.current) return;

    const container = containerRef.current;
    const width = container.clientWidth || 800;
    const height = container.clientHeight || 600;

    // Scene
    const scene = new THREE.Scene();
    scene.background = new THREE.Color(0x0b0f14);
    sceneRef.current = scene;

    // Camera
    const camera = new THREE.PerspectiveCamera(50, width / height, 0.1, 10000);
    camera.position.set(10, 10, 10);
    cameraRef.current = camera;

    // Renderer
    const renderer = new THREE.WebGLRenderer({ antialias: true });
    renderer.setSize(width, height);
    renderer.setPixelRatio(Math.min(window.devicePixelRatio, 2));
    container.appendChild(renderer.domElement);
    rendererRef.current = renderer;

    // Lights
    const ambientLight = new THREE.AmbientLight(0xffffff, 0.5);
    scene.add(ambientLight);

    const directionalLight = new THREE.DirectionalLight(0xffffff, 0.5);
    directionalLight.position.set(10, 10, 10);
    scene.add(directionalLight);

    // Helpers
    const axesHelper = new THREE.AxesHelper(5);
    scene.add(axesHelper);

    const gridHelper = new THREE.GridHelper(10, 10, 0x444444, 0x222222);
    scene.add(gridHelper);

    // Controls
    const controls = new OrbitControls(camera, renderer.domElement);
    controls.enableDamping = true;
    controls.dampingFactor = 0.05;
    controls.screenSpacePanning = false;
    controls.minDistance = 1;
    controls.maxDistance = 1000;
    controlsRef.current = controls;

    // Animation loop
    const animate = () => {
      animationFrameRef.current = requestAnimationFrame(animate);
      controls.update();
      renderer.render(scene, camera);
    };
    animate();

    // Handle resize
    const handleResize = () => {
      if (!containerRef.current || !rendererRef.current || !cameraRef.current) return;
      const w = containerRef.current.clientWidth;
      const h = containerRef.current.clientHeight;
      rendererRef.current.setSize(w, h);
      cameraRef.current.aspect = w / h;
      cameraRef.current.updateProjectionMatrix();
    };

    const resizeObserver = new ResizeObserver(handleResize);
    resizeObserver.observe(container);

    return () => {
      resizeObserver.disconnect();
      if (animationFrameRef.current) {
        cancelAnimationFrame(animationFrameRef.current);
      }
      controls.dispose();
      renderer.dispose();
      if (container && renderer.domElement.parentNode === container) {
        container.removeChild(renderer.domElement);
      }
    };
  }, []);

  // Update point cloud data
  useEffect(() => {
    if (!data || !sceneRef.current) return;

    const scene = sceneRef.current;

    // Remove old points
    if (pointsMeshRef.current) {
      scene.remove(pointsMeshRef.current);
      pointsMeshRef.current.geometry.dispose();
      if (pointsMeshRef.current.material instanceof THREE.Material) {
        pointsMeshRef.current.material.dispose();
      }
      pointsMeshRef.current = null;
    }

    // Create geometry
    const geometry = new THREE.BufferGeometry();
    geometry.setAttribute("position", new THREE.Float32BufferAttribute(data.positions, 3));

    // Colors
    if (data.colors && data.colors.length > 0) {
      geometry.setAttribute("color", new THREE.Float32BufferAttribute(data.colors, 3));
    } else {
      // Default to light blue color
      const colors = new Float32Array(data.positions.length);
      for (let i = 0; i < colors.length; i += 3) {
        colors[i] = 0.376;     // R (96/255)
        colors[i + 1] = 0.647; // G (165/255)
        colors[i + 2] = 0.980; // B (250/255)
      }
      geometry.setAttribute("color", new THREE.Float32BufferAttribute(colors, 3));
    }

    geometry.computeBoundingBox();

    // Shader material for round points with world-space size
    const vertexShader = `
      varying vec3 vColor;
      uniform float pointSizeMeters;
      uniform float screenHeight;
      uniform float fov;
      
      void main() {
        vColor = color;
        vec4 mvPosition = modelViewMatrix * vec4(position, 1.0);
        gl_Position = projectionMatrix * mvPosition;
        // Calculate point size in pixels based on world-space size (meters)
        float distance = -mvPosition.z;
        float fovRad = fov * 3.14159 / 180.0;
        gl_PointSize = (pointSizeMeters * screenHeight) / (2.0 * tan(fovRad / 2.0) * distance);
        gl_PointSize = clamp(gl_PointSize, 1.0, 64.0);
      }
    `;

    const fragmentShader = `
      varying vec3 vColor;
      
      void main() {
        vec2 coord = gl_PointCoord - vec2(0.5);
        if (length(coord) > 0.5) {
          discard;
        }
        gl_FragColor = vec4(vColor, 1.0);
      }
    `;

    const material = new THREE.ShaderMaterial({
      vertexShader,
      fragmentShader,
      uniforms: {
        pointSizeMeters: { value: pointSize },
        screenHeight: { value: containerRef.current?.clientHeight || 600 },
        fov: { value: 50 },
      },
      vertexColors: true,
    });

    const points = new THREE.Points(geometry, material);

    // Rotate to align ROS coordinate system with visualization
    points.rotation.x = -Math.PI / 2;

    scene.add(points);
    pointsMeshRef.current = points;

    return () => {
      if (pointsMeshRef.current && sceneRef.current) {
        sceneRef.current.remove(pointsMeshRef.current);
        pointsMeshRef.current.geometry.dispose();
        if (pointsMeshRef.current.material instanceof THREE.Material) {
          pointsMeshRef.current.material.dispose();
        }
        pointsMeshRef.current = null;
      }
    };
  }, [data, pointSize]);

  return (
    <div id="pcView" style={{ width: "100%", height: "100%", position: "relative" }}>
      <div ref={containerRef} style={{ width: "100%", height: "100%" }} />
      {data && (
        <div
          style={{
            position: "absolute",
            bottom: 10,
            left: 10,
            padding: "8px 12px",
            background: "rgba(0, 0, 0, 0.7)",
            color: "#fff",
            borderRadius: 4,
            fontSize: 11,
            fontFamily: "monospace",
          }}
        >
          <div>Points: {data.pointCount.toLocaleString()}</div>
          {data.frame_id && <div>Frame: {data.frame_id}</div>}
        </div>
      )}
    </div>
  );
}

export default PointCloudViewer3D;
