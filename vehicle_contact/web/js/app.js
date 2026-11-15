// Contact exercise window: drivable vehicle, live suspension tuning and a
// diagnostics area showing wheel-ground contact and chassis motion.
// Physics runs locally through the JS port; /api/verify cross-checks it
// against the reference Python engine using identical replay inputs.
'use strict';

const {Simulation, FIXED_DT, DURATIONS, PROFILES, runReplay, sampleAt} = window.Sim;

const state = {
  scenario: 'slope',
  sim: null,
  mode: 'idle',           // idle | drive | replay | recording
  keys: new Set(),
  liveControl: {throttle: 0, brake: 0, steer: 0},
  replay: null,
  replayIndex: 0,
  accumulator: 0,
  lastFrame: performance.now(),
  recorded: [],
  verify: null,
};

const canvas = document.getElementById('viewport');
const renderer = new Renderer(canvas);
const $ = id => document.getElementById(id);

function buildSim() {
  state.sim = new Simulation(state.scenario, FIXED_DT);
  state.replayIndex = 0;
  state.accumulator = 0;
  state.recorded = [];
  refreshSuspensionInputs();
}

// ------------------------------------------------------------- input
const KEYMAP = {
  ArrowUp: 'throttle', KeyW: 'throttle',
  ArrowDown: 'brake', KeyS: 'brake',
  ArrowLeft: 'steerL', KeyA: 'steerL',
  ArrowRight: 'steerR', KeyD: 'steerR',
};
window.addEventListener('keydown', e => {
  if (KEYMAP[e.code]) { state.keys.add(KEYMAP[e.code]); e.preventDefault(); }
  if (e.code === 'Space') { state.keys.add('brake'); e.preventDefault(); }
});
window.addEventListener('keyup', e => {
  if (KEYMAP[e.code]) state.keys.delete(KEYMAP[e.code]);
  if (e.code === 'Space') state.keys.delete('brake');
});

function keyboardControl() {
  const k = state.keys;
  let steer = 0;
  if (k.has('steerL')) steer += 1;
  if (k.has('steerR')) steer -= 1;
  return {
    throttle: k.has('throttle') ? 1 : 0,
    brake: k.has('brake') ? 1 : 0,
    steer,
  };
}

// -------------------------------------------------------- suspension UI
function refreshSuspensionInputs() {
  const p = state.sim.vehicle.suspensionParams();
  $('tau').value = p.tau;
  $('travel').value = p.max_travel;
  $('stiffness').value = p.spring_k;
  $('mu').value = p.tire_mu;
  $('query').value = p.query_kind;
  $('damping').textContent = p.damping_ratio.toFixed(2);
}

function applySuspension() {
  state.sim.vehicle.setSuspensionParams({
    tau: parseFloat($('tau').value),
    max_travel: parseFloat($('travel').value),
    spring_k: parseFloat($('stiffness').value),
    tire_mu: parseFloat($('mu').value),
    query_kind: $('query').value,
  });
  $('damping').textContent =
    state.sim.vehicle.suspensionParams().damping_ratio.toFixed(2);
}
['tau', 'travel', 'stiffness', 'mu', 'query'].forEach(id =>
  $(id).addEventListener('input', applySuspension));

// ------------------------------------------------------------- buttons
$('scenario').addEventListener('change', e => {
  state.scenario = e.target.value;
  state.mode = 'idle';
  buildSim();
});
$('btn-drive').onclick = () => { state.mode = 'drive'; buildSim(); };
$('btn-reset').onclick = () => { state.mode = 'idle'; buildSim(); };
let recording = false;
$('btn-record').onclick = () => {
  if (!recording) {
    recording = true;
    state.mode = 'recording';
    buildSim();
    $('btn-record').textContent = 'stop & export replay';
  } else {
    recording = false;
    exportReplay();
    $('btn-record').textContent = 'record inputs';
  }
};
$('btn-replay').onclick = async () => {
  const r = await fetch('/api/replay?name=' + state.scenario);
  state.replay = await r.json();
  state.mode = 'replay';
  state.sim = runReplay(state.replay);
  state.replayIndex = 0;
};
$('btn-verify').onclick = verifyEngine;

function exportReplay() {
  const controls = compactControls(state.recorded);
  const v = state.sim.vehicle;
  const w0 = v.wheels[0];
  const replay = {
    format: 'vehicle-contact-replay/1',
    scenario: state.scenario,
    dt: FIXED_DT,
    duration: state.sim.time,
    vehicle_params: {
      mass: v.mass, query_kind: v.queryKind, tire_mu: v.tireMu,
      suspension: {rest_length: w0.restLength, max_travel: w0.maxTravel,
                   spring_k: w0.springK, tau: w0.tau},
    },
    controls,
    meta: {kind: 'manual-recording'},
  };
  const blob = new Blob([JSON.stringify(replay, null, 1)],
                        {type: 'application/json'});
  const a = document.createElement('a');
  a.href = URL.createObjectURL(blob);
  a.download = state.scenario + '-manual.json';
  a.click();
  URL.revokeObjectURL(a.href);
}

// ------------------------------------------------- Python cross-check
async function verifyEngine() {
  const controls = state.recorded.length
    ? compactControls(state.recorded)
    : replayControlsOfScenario(state.scenario);
  const steps = Math.round(DURATIONS[state.scenario] / FIXED_DT);
  // Re-run locally from the same piecewise controls to gather a trajectory.
  const local = new Simulation(state.scenario, FIXED_DT);
  for (let i = 0; i < steps; i++)
    local.update(sampleAt(controls, i * FIXED_DT), false);
  $('verify-status').textContent = 'checking…';
  try {
    const resp = await fetch('/api/verify', {
      method: 'POST',
      headers: {'Content-Type': 'application/json'},
      body: JSON.stringify({scenario: state.scenario, steps, controls,
                            trajectory: local.trajectory}),
    });
    const report = await resp.json();
    state.verify = report;
    renderVerify(report);
  } catch (err) {
    $('verify-status').innerHTML =
      '<span class="bad">server unreachable — start `python3 -m cli.vc serve`</span>';
  }
}

function compactControls(list) {
  const out = [];
  let last = null;
  for (const c of list) {
    const triple = [c.throttle, c.brake, c.steer];
    if (!last || triple.some((v, i) => v !== last[i])) {
      out.push(c);
      last = triple;
    }
  }
  return out;
}
function replayControlsOfScenario(name) {
  const sim = new Simulation(name, FIXED_DT);
  const n = Math.round(DURATIONS[name] / FIXED_DT);
  const list = [];
  for (let i = 0; i < n; i++) {
    list.push({t: i * FIXED_DT, ...PROFILES[name](i * FIXED_DT)});
  }
  return compactControls(list);
}

function renderVerify(r) {
  const el = $('verify-status');
  if (r.error) {
    el.innerHTML = '<span class="bad">error: ' + r.error + '</span>';
    return;
  }
  const cls = r.ok ? 'good' : 'bad';
  el.innerHTML = `<span class="${cls}">${r.ok ? 'MATCH' : 'DIVERGENCE'}</span>
    steps ${r.compared_steps}, max position error
    ${r.max_position_error_m.toExponential(2)} m, contact mismatches
    ${r.max_contact_count_error}`;
}

// ------------------------------------------------------------- loop
function frame(now) {
  const frameDt = Math.min(0.05, (now - state.lastFrame) / 1000);
  state.lastFrame = now;
  if (!state.sim) buildSim();

  let snap = state.sim.snapshots[state.sim.snapshots.length - 1];
  if (state.mode === 'drive' || state.mode === 'recording') {
    state.accumulator += frameDt;
    while (state.accumulator >= FIXED_DT) {
      const c = keyboardControl();
      const tBefore = state.sim.time;
      snap = state.sim.update(c);
      if (state.mode === 'recording')
        state.recorded.push({t: tBefore, ...c});
      state.accumulator -= FIXED_DT;
    }
  } else if (state.mode === 'replay' && state.replay) {
    // Advance in real time through the precomputed replay trajectory.
    state.accumulator += frameDt;
    const idx = Math.min(state.sim.trajectory.length - 1,
      Math.floor(state.accumulator / FIXED_DT));
    state.replayIndex = idx;
    snap = state.sim.snapshots[idx];
    if (idx >= state.sim.trajectory.length - 1) state.accumulator = 0;
  }
  render(snap);
  renderDiagnostics(snap);
  requestAnimationFrame(frame);
}

function render(snap) {
  renderer.resize();
  renderer.clear();
  const v = state.sim.vehicle;
  const target = v.position;
  const b = boundsFor(state.scenario);
  renderer.drawTerrain(v.terrain, target, b);
  renderer.drawBox(v.position, v.quat,
    [v.halfLen, v.halfH, v.halfW], target, '#4f86c6');

  // Wheels + suspension struts + contact markers.
  for (const w of v.wheels) {
    const anchor = v.wheelAnchorWorld(w);
    const center = w.grounded
      ? window.M3.add(anchor, window.M3.scale(v.suspensionDir(),
          w.contactDistance))
      : window.M3.add(anchor, window.M3.scale(v.suspensionDir(),
          w.restLength + w.maxTravel));
    renderer.line(anchor, center, target, '#9fb3c8', 2);
    renderer.drawWheel(center, w.radius, w.steerAngle, w.spinTotal,
                       target, w.grounded);
    if (w.grounded) {
      const cp = w.contactPoint, n = w.contactNormal;
      renderer.line(cp, window.M3.add(cp, window.M3.scale(n, 0.6)),
                    target, '#ff5d5d', 2);
      renderer.line(window.M3.add(cp, [-0.18, 0, 0]),
                    window.M3.add(cp, [0.18, 0, 0]), target, '#ffffff', 2);
    }
  }
  // Velocity vector.
  renderer.line(v.position,
    window.M3.add(v.position, window.M3.scale(v.velocity, 0.4)),
    target, '#7CFC9A', 2);
}

function boundsFor(scenario) {
  return {slope: [-4, 80, -8, 8], bump: [-4, 48, -10, 10],
          edge: [-4, 30, -8, 8], flat: [-8, 30, -8, 8]}[scenario];
}

function renderDiagnostics(snap) {
  if (!snap) return;
  $('mode').textContent = state.mode;
  $('pos').textContent = fmtVec(snap.position, 3);
  $('vel').textContent = fmtVec(snap.velocity, 3);
  $('speed').textContent = snap.speed.toFixed(2);
  $('euler').textContent = snap.orientation_euler
    .map(a => a.toFixed(1) + '°').join(' ');
  $('contacts').textContent = snap.contact_count + ' / 4';
  $('contacts').className = snap.contact_count === 4 ? 'good'
    : snap.contact_count === 0 ? 'bad' : 'warn';
  $('body-contact').textContent = snap.body_contact ? 'YES' : 'no';
  $('body-contact').className = snap.body_contact ? 'bad' : 'good';

  const rows = snap.wheels.map(w => `
    <tr class="${w.grounded ? 'ok' : 'air'}">
      <td>${w.name}</td>
      <td>${w.grounded ? '<b class="good">ground</b>' : '<b class="warn">air</b>'}</td>
      <td>${(w.travel_ratio * 100).toFixed(0)}%</td>
      <td>${w.compression.toFixed(3)}</td>
      <td>${w.load.toFixed(0)}</td>
      <td>${w.slip.toFixed(3)}</td>
      <td>${w.omega.toFixed(1)}</td>
      <td>${w.spin_ground.toFixed(2)}</td>
      <td>${w.spin_air.toFixed(2)}</td>
      <td>${w.query}</td>
    </tr>`).join('');
  $('wheel-rows').innerHTML = rows;

  const warnings = snap.warnings.length
    ? snap.warnings.map(x => '<li>' + x + '</li>').join('')
    : '<li class="good">no contact anomalies</li>';
  $('warnings').innerHTML = warnings;
}

function fmtVec(v, d) { return '(' + v.map(x => x.toFixed(d)).join(', ') + ')'; }

buildSim();
requestAnimationFrame(frame);
