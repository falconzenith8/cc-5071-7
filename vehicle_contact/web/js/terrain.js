// Analytic heightfield terrain with raycast and sphere-cast shape queries.
// Direct port of engine/terrain.py; must stay numerically identical so the
// /api/verify cross-check against the Python engine passes.
'use strict';

const M = window.M3;

class Heightfield {
  constructor(regions = []) { this.regions = regions; }

  regionHeight(kind, p, r) {
    const x = p[0], z = p[2];
    if (kind === 'base') return r.y ?? 0;
    if (kind === 'slope') {
      if (!(r.x0 <= x && x <= r.x1)) return null;
      const t = (x - r.x0) / (r.x1 - r.x0);
      return r.y0 + t * (r.y1 - r.y0);
    }
    if (kind === 'bump') {
      const dx = (x - r.cx) / r.sx, dz = (z - r.cz) / r.sz;
      const d2 = dx * dx + dz * dz;
      if (d2 > 1) return null;
      return r.base_y + r.amp * (0.5 + 0.5 * Math.cos(Math.PI * Math.sqrt(d2)));
    }
    if (kind === 'platform') {
      if (!(r.x0 <= x && x <= r.x1 && r.z0 <= z && z <= r.z1)) return null;
      return r.top_y;
    }
    throw new Error('unknown region: ' + kind);
  }

  height(x, z) {
    let h = 0;
    for (const r of this.regions) {
      const rh = this.regionHeight(r.kind, [x, 0, z], r);
      if (rh !== null && rh > h) h = rh;
    }
    return h;
  }

  normal(x, z) {
    const e = 0.05;
    const hx = (this.height(x + e, z) - this.height(x - e, z)) / (2 * e);
    const hz = (this.height(x, z + e) - this.height(x, z - e)) / (2 * e);
    let n = M.norm([-hx, 1, -hz]);
    const steep = Math.max(Math.abs(hx), Math.abs(hz));
    if (steep > 1.5) {
      const band = Math.min(1, (steep - 1.5) / 2.5);
      n = M.norm(M.lerp(n, [0, 1, 0], band));
    }
    return n;
  }

  raycast(origin, direction, maxDist) {
    const d = M.norm(direction);
    if (d[1] >= -1e-9) return null;
    const steps = 48;
    let prevT = 0;
    let prevG = origin[1] - this.height(origin[0], origin[2]);
    let hitT = null;
    for (let i = 1; i <= steps; i++) {
      const t = maxDist * i / steps;
      const pt = M.add(origin, M.scale(d, t));
      const g = pt[1] - this.height(pt[0], pt[2]);
      if (g <= 0 && 0 <= prevG) {
        let lo = prevT, hi = t;
        for (let k = 0; k < 40; k++) {
          const mid = 0.5 * (lo + hi);
          const mp = M.add(origin, M.scale(d, mid));
          const mg = mp[1] - this.height(mp[0], mp[2]);
          if (mg > 0) lo = mid; else hi = mid;
        }
        hitT = 0.5 * (lo + hi);
        break;
      }
      prevT = t; prevG = g;
    }
    if (hitT === null) return null;
    const point = M.add(origin, M.scale(d, hitT));
    return {distance: hitT, point, normal: this.normal(point[0], point[2])};
  }

  sphereCast(origin, direction, maxDist, radius) {
    const d = M.norm(direction);
    if (d[1] >= -1e-9) return null;
    const clearance = t => {
      const c = M.add(origin, M.scale(d, t));
      const n = this.normal(c[0], c[2]);
      return [c[1] - this.height(c[0], c[2]) - radius * n[1], c, n];
    };
    const steps = 48;
    let prevT = 0;
    let prevG = clearance(0)[0];
    let hitT = null;
    for (let i = 1; i <= steps; i++) {
      const t = maxDist * i / steps;
      const g = clearance(t)[0];
      if (g <= 0 && 0 <= prevG) {
        let lo = prevT, hi = t;
        for (let k = 0; k < 40; k++) {
          const mid = 0.5 * (lo + hi);
          if (clearance(mid)[0] > 0) lo = mid; else hi = mid;
        }
        hitT = 0.5 * (lo + hi);
        break;
      }
      prevT = t; prevG = g;
    }
    if (hitT === null) return null;
    const [, centre, n] = clearance(hitT);
    // Quantise horizontal coordinates so region-edge classification matches
    // the Python engine even when the sphere centre sits on a boundary.
    const cx = Math.round(centre[0] * 1e9) / 1e9;
    const cz = Math.round(centre[2] * 1e9) / 1e9;
    const point = [cx, this.height(cx, cz), cz];
    return {distance: hitT, point, normal: n, centre};
  }
}

function baseTerrain() { return new Heightfield([{kind: 'base', y: 0}]); }
function slopeTerrain() {
  return new Heightfield([
    {kind: 'base', y: 0},
    {kind: 'slope', x0: 12, x1: 42, y0: 0, y1: 6.4},
    {kind: 'slope', x0: 42, x1: 72, y0: 6.4, y1: 0},
  ]);
}
function bumpTerrain() {
  return new Heightfield([
    {kind: 'base', y: 0},
    {kind: 'bump', cx: 12, cz: 0, sx: 6, sz: 6, amp: 0.38, base_y: 0},
    {kind: 'bump', cx: 24, cz: 1.6, sx: 4.5, sz: 4.5, amp: 0.3, base_y: 0},
    {kind: 'bump', cx: 35, cz: -1.2, sx: 5, sz: 5, amp: 0.34, base_y: 0},
  ]);
}
function edgeTerrain() {
  return new Heightfield([
    {kind: 'base', y: 0},
    {kind: 'slope', x0: 4, x1: 9, y0: 0, y1: 1},
    {kind: 'platform', x0: 9, x1: 26, z0: -4, z1: 3, top_y: 1},
  ]);
}
const TERRAINS = {flat: baseTerrain, slope: slopeTerrain,
                  bump: bumpTerrain, edge: edgeTerrain};

window.Heightfield = Heightfield;
window.TERRAINS = TERRAINS;
