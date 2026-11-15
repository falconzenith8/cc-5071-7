// Minimal software 3D renderer on a 2D canvas: perspective projection,
// depth-sorted triangle fill and line overlay for contact diagnostics.
// No external libraries, so the contact window works fully offline.
'use strict';

class Renderer {
  constructor(canvas) {
    this.canvas = canvas;
    this.ctx = canvas.getContext('2d');
    this.yaw = -0.9;
    this.pitch = 0.42;
    this.dist = 16;
    this.focal = 760;
    this._drag = null;
    canvas.addEventListener('pointerdown', e => {
      this._drag = [e.clientX, e.clientY];
      canvas.setPointerCapture(e.pointerId);
    });
    canvas.addEventListener('pointermove', e => {
      if (!this._drag) return;
      const [x, y] = this._drag;
      this.yaw += (e.clientX - x) * 0.006;
      this.pitch = Math.max(0.08, Math.min(1.3,
        this.pitch + (e.clientY - y) * 0.005));
      this._drag = [e.clientX, e.clientY];
    });
    canvas.addEventListener('pointerup', () => { this._drag = null; });
    canvas.addEventListener('wheel', e => {
      e.preventDefault();
      this.dist = Math.max(6, Math.min(40, this.dist + e.deltaY * 0.02));
    }, {passive: false});
  }

  resize() {
    const dpr = window.devicePixelRatio || 1;
    const r = this.canvas.getBoundingClientRect();
    this.canvas.width = r.width * dpr;
    this.canvas.height = r.height * dpr;
    this.ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
    this.w = r.width; this.h = r.height;
  }

  _basis(target) {
    const cy = Math.cos(this.yaw), sy = Math.sin(this.yaw);
    const cp = Math.cos(this.pitch), sp = Math.sin(this.pitch);
    // Camera orbits the target; forward points from camera toward target.
    const offset = [this.dist * cp * sy, this.dist * sp, this.dist * cp * cy];
    const eye = [target[0] + offset[0], target[1] + offset[1],
                 target[2] + offset[2]];
    const fwd = norm3(sub3(target, eye));
    const right = norm3(cross3(fwd, [0, 1, 0]));
    const up = cross3(right, fwd);
    return {eye, fwd, right, up};
  }

  project(p, target) {
    const {eye, fwd, right, up} = this._basis(target);
    const rel = sub3(p, eye);
    const z = dot3(rel, fwd);
    if (z <= 0.1) return null;
    const x = dot3(rel, right), y = dot3(rel, up);
    const s = this.focal / z;
    return [this.w / 2 + x * s, this.h / 2 - y * s, z];
  }

  clear() {
    this.ctx.fillStyle = '#0e1420';
    this.ctx.fillRect(0, 0, this.w, this.h);
  }

  line(a, b, target, color, width = 1) {
    const pa = this.project(a, target), pb = this.project(b, target);
    if (!pa || !pb) return;
    this.ctx.strokeStyle = color;
    this.ctx.lineWidth = width;
    this.ctx.beginPath();
    this.ctx.moveTo(pa[0], pa[1]);
    this.ctx.lineTo(pb[0], pb[1]);
    this.ctx.stroke();
  }

  tri(pts, target, fill, stroke = null) {
    const proj = pts.map(p => this.project(p, target));
    if (proj.some(p => !p)) return;
    const depth = proj.reduce((s, p) => s + p[2], 0) / proj.length;
    this._tris = this._tris || [];
    this._tris.push({proj, depth, fill, stroke});
  }

  flushTris() {
    (this._tris || []).sort((a, b) => b.depth - a.depth);
    for (const t of this._tris) {
      this.ctx.fillStyle = t.fill;
      this.ctx.beginPath();
      this.ctx.moveTo(t.proj[0][0], t.proj[0][1]);
      for (let i = 1; i < t.proj.length; i++)
        this.ctx.lineTo(t.proj[i][0], t.proj[i][1]);
      this.ctx.closePath();
      this.ctx.fill();
      if (t.stroke) {
        this.ctx.strokeStyle = t.stroke;
        this.ctx.lineWidth = 0.6;
        this.ctx.stroke();
      }
    }
    this._tris = [];
  }

  drawTerrain(terrain, target, bounds, step = 1.5) {
    const [x0, x1, z0, z1] = bounds;
    for (let x = x0; x < x1; x += step) {
      for (let z = z0; z < z1; z += step) {
        const p = [[x, terrain.height(x, z), z],
                   [x + step, terrain.height(x + step, z), z],
                   [x + step, terrain.height(x + step, z + step), z + step],
                   [x, terrain.height(x, z + step), z + step]];
        const h = (p[0][1] + p[2][1]) * 0.5;
        const shade = Math.max(18, Math.min(58, 30 + h * 4));
        this.tri([p[0], p[1], p[2]], target,
                 `rgb(${shade},${shade + 18},${shade + 12})`, '#1d2c38');
        this.tri([p[0], p[2], p[3]], target,
                 `rgb(${shade + 4},${shade + 22},${shade + 16})`, '#1d2c38');
      }
    }
    this.flushTris();
  }

  drawBox(center, quat, half, target, color) {
    const M3 = window.M3;
    const corners = [];
    for (const sx of [-1, 1]) for (const sy of [-1, 1])
      for (const sz of [-1, 1])
        corners.push(M3.add(center,
          M3.bodyToWorld(quat, [sx * half[0], sy * half[1], sz * half[2]])));
    const faces = [[0, 1, 3, 2], [4, 6, 7, 5], [0, 4, 5, 1],
                   [2, 3, 7, 6], [0, 2, 6, 4], [1, 5, 7, 3]];
    for (const f of faces)
      this.tri(f.map(i => corners[i]), target, color, '#0a0f18');
    this.flushTris();
  }

  drawWheel(center, radius, steer, spin, target, grounded) {
    const M3 = window.M3;
    // Wheel axis along local z, rotated by steer about vertical.
    const axis = M3.norm([Math.sin(steer), 0, Math.cos(steer)]);
    const fwd = M3.norm([Math.cos(steer), 0, -Math.sin(steer)]);
    const spoke = M3.add(M3.scale(fwd, Math.cos(spin) * radius),
                        M3.scale([0, 1, 0], Math.sin(spin) * radius));
    const rim = grounded ? '#37e08b' : '#e0b341';
    this.line(M3.sub(center, M3.scale(axis, radius * 0.7)),
              M3.add(center, M3.scale(axis, radius * 0.7)),
              target, rim, 3);
    this.line(center, M3.add(center, spoke), target, '#dfe7f0', 2);
    this.line(center, M3.sub(center, M3.scale(spoke, 0.6)),
              target, '#8fa3b8', 1.5);
  }
}

function sub3(a, b) { return [a[0] - b[0], a[1] - b[1], a[2] - b[2]]; }
function add3(a, b) { return [a[0] + b[0], a[1] + b[1], a[2] + b[2]]; }
function dot3(a, b) { return a[0] * b[0] + a[1] * b[1] + a[2] * b[2]; }
function cross3(a, b) {
  return [a[1] * b[2] - a[2] * b[1],
          a[2] * b[0] - a[0] * b[2],
          a[0] * b[1] - a[1] * b[0]];
}
function norm3(a) {
  const n = Math.hypot(a[0], a[1], a[2]) || 1;
  return [a[0] / n, a[1] / n, a[2] / n];
}

window.Renderer = Renderer;
