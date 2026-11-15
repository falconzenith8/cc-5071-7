// 3D vector / quaternion helpers - line-for-line port of engine/math3d.py.
// Plain arrays of numbers, no dependencies, deterministic across browsers.
'use strict';

const EPS = 1e-12;

function add(a, b) { return [a[0] + b[0], a[1] + b[1], a[2] + b[2]]; }
function sub(a, b) { return [a[0] - b[0], a[1] - b[1], a[2] - b[2]]; }
function scale(a, s) { return [a[0] * s, a[1] * s, a[2] * s]; }
function dot(a, b) { return a[0] * b[0] + a[1] * b[1] + a[2] * b[2]; }
function cross(a, b) {
  return [a[1] * b[2] - a[2] * b[1],
          a[2] * b[0] - a[0] * b[2],
          a[0] * b[1] - a[1] * b[0]];
}
function length(a) { return Math.sqrt(dot(a, a)); }
function norm(a) {
  const n = length(a);
  return n < EPS ? [0, 0, 0] : [a[0] / n, a[1] / n, a[2] / n];
}
function lerp(a, b, t) {
  return [a[0] + (b[0] - a[0]) * t,
          a[1] + (b[1] - a[1]) * t,
          a[2] + (b[2] - a[2]) * t];
}
function clamp(x, lo, hi) { return x < lo ? lo : (x > hi ? hi : x); }

function qIdentity() { return [1, 0, 0, 0]; }
function qMul(p, q) {
  const [pw, px, py, pz] = p, [qw, qx, qy, qz] = q;
  return [pw * qw - px * qx - py * qy - pz * qz,
          pw * qx + px * qw + py * qz - pz * qy,
          pw * qy - px * qz + py * qw + pz * qx,
          pw * qz + px * qy - py * qx + pz * qw];
}
function qFromAxisAngle(axis, angle) {
  axis = norm(axis);
  const h = angle * 0.5, s = Math.sin(h);
  return [Math.cos(h), axis[0] * s, axis[1] * s, axis[2] * s];
}
function qNormalize(q) {
  const n = Math.sqrt(q[0] ** 2 + q[1] ** 2 + q[2] ** 2 + q[3] ** 2);
  return n < EPS ? qIdentity() : [q[0] / n, q[1] / n, q[2] / n, q[3] / n];
}
function qRotate(q, v) {
  const u = [q[1], q[2], q[3]];
  const t = scale(cross(u, v), 2);
  return add(add(v, scale(t, q[0])), cross(u, t));
}
function qIntegrate(q, omegaWorld, dt) {
  const sp = length(omegaWorld);
  if (sp < EPS) return q;
  const spin = qFromAxisAngle(omegaWorld, sp * dt);
  return qNormalize(qMul(spin, q));
}
function qToEuler(q) {
  const [w, x, y, z] = q;
  const roll = Math.atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y));
  const sinp = 2 * (w * y - z * x);
  const pitch = Math.asin(clamp(sinp, -1, 1));
  const yaw = Math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z));
  return [roll, pitch, yaw];
}
function rotMatrix(q) {
  const [w, x, y, z] = q;
  return [[1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
          [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
          [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)]];
}
function mVec(m, v) {
  return [m[0][0] * v[0] + m[0][1] * v[1] + m[0][2] * v[2],
          m[1][0] * v[0] + m[1][1] * v[1] + m[1][2] * v[2],
          m[2][0] * v[0] + m[2][1] * v[1] + m[2][2] * v[2]];
}
function mTranspose(m) {
  return [0, 1, 2].map(r => [0, 1, 2].map(c => m[c][r]));
}
function worldToBody(q, v) { return mVec(mTranspose(rotMatrix(q)), v); }
function bodyToWorld(q, v) { return mVec(rotMatrix(q), v); }
const deg = a => a * 180 / Math.PI;
const rad = a => a * Math.PI / 180;

window.M3 = {add, sub, scale, dot, cross, length, norm, lerp, clamp,
             qIdentity, qMul, qNormalize, qIntegrate, qToEuler,
             rotMatrix, worldToBody, bodyToWorld, deg, rad};
