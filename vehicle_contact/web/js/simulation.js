// Fixed-step simulation, scenario input profiles and replay playback.
// Port of engine/simulation.py + engine/scenarios.py.
'use strict';

const FIXED_DT = 1 / 120;

const SCENARIO_DEFAULTS = {
  flat: {position: [0, 0.85, 0]},
  slope: {position: [0, 0.85, 0]},
  bump: {position: [0, 0.85, 0]},
  edge: {position: [0, 0.85, 0]},
};
const DURATIONS = {slope: 22, bump: 24, edge: 20};

function smoothRamp(t, t0, t1, target) {
  if (t <= t0) return 0;
  if (t >= t1) return target;
  const u = (t - t0) / (t1 - t0);
  return target * 0.5 * (1 - Math.cos(Math.PI * u));
}

function slopeProfile(t) {
  let throttle = smoothRamp(t, 0, 1.5, 0.55);
  if (t > 12) throttle *= Math.max(0, 1 - (t - 12) / 2);
  return {throttle: Math.round(throttle * 1e4) / 1e4, brake: 0, steer: 0};
}
function bumpProfile(t) {
  const throttle = smoothRamp(t, 0, 1.5, 0.32);
  let steer = 0;
  if (t > 8 && t < 20) steer = 0.22 * Math.sin(0.7 * (t - 8));
  return {throttle: Math.round(throttle * 1e4) / 1e4, brake: 0,
          steer: Math.round(steer * 1e4) / 1e4};
}
function edgeProfile(t) {
  let throttle = smoothRamp(t, 0, 1, 0.35);
  let steer = 0;
  if (t > 9) throttle = smoothRamp(t, 9, 10, 0.22);
  if (t > 11 && t < 14.5) steer = 0.8;
  if (t > 16) throttle = 0;
  return {throttle: Math.round(throttle * 1e4) / 1e4,
          brake: t > 17 ? 0.3 : 0,
          steer: Math.round(steer * 1e4) / 1e4};
}
const PROFILES = {slope: slopeProfile, bump: bumpProfile, edge: edgeProfile};

class Simulation {
  constructor(scenario = 'flat', dt = FIXED_DT, params = null) {
    const terrain = window.TERRAINS[scenario]();
    const vp = {...SCENARIO_DEFAULTS[scenario], ...(params || {})};
    this.vehicle = new window.Vehicle(terrain, vp);
    this.scenario = scenario;
    this.dt = dt;
    this.time = 0;
    this.stepIndex = 0;
    this.recordedControls = [];
    this.snapshots = [];
    this.trajectory = [];
    this.settle();
  }

  settle(settleTime = 1.5) {
    const idle = {throttle: 0, brake: 0, steer: 0};
    const n = Math.round(settleTime / this.dt);
    for (let i = 0; i < n; i++) this.vehicle.step(this.dt, idle);
    this.time = 0;
    this.stepIndex = 0;
  }

  update(control, record = true) {
    if (record) this.recordedControls.push({t: this.time, ...control});
    const snap = this.vehicle.step(this.dt, control);
    this.time += this.dt;
    this.stepIndex++;
    this.snapshots.push(snap);
    this.trajectory.push({
      t: this.time,
      position: [...snap.position],
      velocity: [...snap.velocity],
      speed: snap.speed,
      contact_count: snap.contact_count,
      body_contact: snap.body_contact,
      travel: snap.wheels.map(w => w.compression),
      spin_ground: snap.wheels.map(w => w.spin_ground),
      spin_air: snap.wheels.map(w => w.spin_air),
    });
    return snap;
  }

  runSeconds(duration, controlFn) {
    const n = Math.round(duration / this.dt);
    let snap = null;
    for (let i = 0; i < n; i++) snap = this.update(controlFn(this.time));
    return snap;
  }
}

function sampleAt(controls, t) {
  let active = {throttle: 0, brake: 0, steer: 0};
  for (const c of controls) {
    if (c.t <= t + 1e-12)
      active = {throttle: c.throttle ?? 0, brake: c.brake ?? 0,
                steer: c.steer ?? 0};
    else break;
  }
  return active;
}

function runReplay(replay, overrides = null) {
  const params = {...(replay.vehicle_params || {})};
  if (overrides) {
    params.suspension = {...(params.suspension || {}), ...overrides};
  }
  const sim = new Simulation(replay.scenario, replay.dt, params);
  const n = Math.round(replay.duration / sim.dt);
  for (let i = 0; i < n; i++)
    sim.update(sampleAt(replay.controls, i * sim.dt), false);
  return sim;
}

window.Sim = {Simulation, FIXED_DT, DURATIONS, PROFILES, runReplay, sampleAt};
