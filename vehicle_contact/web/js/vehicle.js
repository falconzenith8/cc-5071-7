// Rigid-body wheeled vehicle: chassis integration around the wheel-ground
// module.  Contact support, suspension and tyre forces live in
// web/js/wheel_ground.js; this file keeps the same public shape as before so
// the simulation/replay layer and the Python parity check are unaffected.
'use strict';

const M = window.M3;
const GRAVITY = [0, -9.81, 0];
const {Wheel, WheelGroundModule, SupportQuery, TireParameters,
       wheelGroundState} = window.WG;

class Vehicle {
  constructor(terrain, params = {}) {
    this.terrain = terrain;
    this.mass = params.mass ?? 900;
    this.halfLen = params.half_len ?? 1.7;
    this.halfW = params.half_w ?? 0.85;
    this.halfH = params.half_h ?? 0.45;
    this.inertiaBody = this.boxInertia();
    this.position = [...(params.position ?? [0, 0.85, 0])];
    this.velocity = [0, 0, 0];
    this.quat = M.qIdentity();
    this.angularVelocity = [0, 0, 0];

    this.tireMu = params.tire_mu ?? 1.1;
    this.engineForce = params.engine_force ?? 3200;
    this.brakeForce = params.brake_force ?? 6000;
    this.maxSteer = M.rad(params.max_steer_deg ?? 28);
    this.rollingResistance = params.rolling_resistance ?? 26;
    this.aeroDrag = params.aero_drag ?? 1.6;

    const wheelR = params.wheel_radius ?? 0.32;
    const fx = this.halfLen * 0.82, fz = this.halfW * 0.9;
    this.wheels = [
      new Wheel('FL', [fx, -0.15, fz], wheelR, true, true),
      new Wheel('FR', [fx, -0.15, -fz], wheelR, true, true),
      new Wheel('RL', [-fx, -0.15, fz], wheelR, true, false),
      new Wheel('RR', [-fx, -0.15, -fz], wheelR, true, false),
    ];
    const susp = params.suspension || {};
    for (const w of this.wheels) {
      w.restLength = susp.rest_length ?? w.restLength;
      w.maxTravel = susp.max_travel ?? w.maxTravel;
      w.springK = susp.spring_k ?? w.springK;
      w.tau = susp.tau ?? w.tau;
    }

    this.contact = new WheelGroundModule(
      terrain, this.wheels, params.query_kind ?? 'sphere',
      new TireParameters(this.tireMu, this.engineForce, this.brakeForce,
                         this.rollingResistance),
      this.maxSteer);

    this.lastContactCount = 0;
    this.warnings = [];
    this.bodyContact = false;
  }

  // Support-query kind is owned by the wheel-ground module.
  get queryKind() { return this.contact.query.kind; }
  set queryKind(kind) { this.contact.query = new SupportQuery(this.terrain, kind); }
  get tire() { return this.contact.tire; }
  set tire(value) { this.contact.tire = value; }

  boxInertia() {
    const m = this.mass, hx = this.halfLen, hy = this.halfH, hz = this.halfW;
    return [m / 3 * (hy * hy + hz * hz),
            m / 3 * (hx * hx + hz * hz),
            m / 3 * (hx * hx + hy * hy)];
  }

  suspensionParams() {
    const w0 = this.wheels[0];
    const [c, zeta] = w0.effectiveDamping(this.mass / this.wheels.length);
    return {query_kind: this.queryKind, rest_length: w0.restLength,
            max_travel: w0.maxTravel, spring_k: w0.springK, tau: w0.tau,
            damping_ratio: zeta, damping_c: c, tire_mu: this.tireMu,
            wheel_radius: w0.radius};
  }

  setSuspensionParams(cfg) {
    for (const w of this.wheels) {
      if (cfg.rest_length !== undefined) w.restLength = +cfg.rest_length;
      if (cfg.max_travel !== undefined) w.maxTravel = +cfg.max_travel;
      if (cfg.spring_k !== undefined) w.springK = +cfg.spring_k;
      if (cfg.tau !== undefined) w.tau = +cfg.tau;
    }
    if (cfg.query_kind !== undefined) this.queryKind = cfg.query_kind;
    if (cfg.tire_mu !== undefined) {
      this.tireMu = +cfg.tire_mu;
      this.contact.tire.mu = this.tireMu;
    }
  }

  wheelAnchorWorld(w) { return this.contact.wheelAnchorWorld(this, w); }
  suspensionDir() { return this.contact.suspensionDirection(this); }

  // Public wheel-ground interface consumed by the diagnostics/UI.
  wheelGround() { return wheelGroundState(this); }

  step(dt, control) {
    this.warnings = [];
    let forceTotal = [0, 0, 0];
    let torqueTotal = [0, 0, 0];
    for (let i = 0; i < 3; i++) forceTotal[i] += GRAVITY[i] * this.mass;
    const speed = M.length(this.velocity);
    if (speed > 1e-6)
      forceTotal = M.add(forceTotal, M.scale(this.velocity,
                                            -this.aeroDrag * speed));

    const {steps, warnings} = this.contact.perWheelStep(this, control, dt);
    this.warnings.push(...warnings);

    this.lastContactCount = 0;
    for (const s of steps) {
      if (!s.grounded) continue;
      this.lastContactCount++;
      forceTotal = M.add(M.add(forceTotal, s.fSuspension), s.fTire);
      torqueTotal = M.add(M.add(torqueTotal,
        M.cross(s.contactOffset, s.fSuspension)),
        M.cross(s.contactOffset, s.fTire));
    }
    if (this.lastContactCount === 0) this.warnings.push('all wheels airborne');

    const accel = M.scale(forceTotal, 1 / this.mass);
    for (let i = 0; i < 3; i++) {
      this.velocity[i] += accel[i] * dt;
      this.position[i] += this.velocity[i] * dt;
    }
    const tauBody = M.worldToBody(this.quat, torqueTotal);
    const angAccel = [tauBody[0] / this.inertiaBody[0],
                      tauBody[1] / this.inertiaBody[1],
                      tauBody[2] / this.inertiaBody[2]];
    let omegaBody = M.worldToBody(this.quat, this.angularVelocity);
    omegaBody = M.add(omegaBody, M.scale(angAccel, dt));
    this.angularVelocity = M.bodyToWorld(this.quat, omegaBody);
    this.quat = M.qIntegrate(this.quat, this.angularVelocity, dt);

    this.resolveBodyPenetration();
    this.contact.integrateGroundSpin(this, dt);
    return this.snapshot();
  }

  resolveBodyPenetration() {
    const q = this.quat;
    this.bodyContact = false;
    for (const sx of [-1, 1]) for (const sz of [-1, 1]) {
      const local = [sx * this.halfLen, -this.halfH, sz * this.halfW];
      const corner = M.add(this.position, M.bodyToWorld(q, local));
      const ground = this.terrain.height(corner[0], corner[2]);
      if (corner[1] < ground) {
        this.position[1] += ground - corner[1];
        if (this.velocity[1] < 0) this.velocity[1] = 0;
        this.bodyContact = true;
      }
    }
    if (this.bodyContact) this.warnings.push('chassis hull contacted terrain');
  }

  snapshot() {
    const [roll, pitch, yaw] = M.qToEuler(this.quat);
    return {
      position: [...this.position],
      velocity: [...this.velocity],
      speed: M.length(this.velocity),
      orientation_euler: [M.deg(roll), M.deg(pitch), M.deg(yaw)],
      angular_velocity: [...this.angularVelocity],
      contact_count: this.lastContactCount,
      body_contact: this.bodyContact,
      warnings: [...this.warnings],
      anomalies: window.WG.classifyWarnings(this.warnings),
      wheels: this.wheels.map(w => ({
        name: w.name, grounded: w.grounded, query: w.queryKind,
        center: w.center,
        compression: w.compression, max_travel: w.maxTravel,
        travel_ratio: w.compression / Math.max(w.maxTravel, 1e-9),
        contact_point: w.contactPoint, contact_normal: w.contactNormal,
        contact_distance: w.contactDistance, load: w.load, slip: w.slip,
        steer_deg: M.deg(w.steerAngle), omega: w.omega,
        spin_ground: w.spinGround, spin_air: w.spinAir,
        spin_total: w.spinTotal,
        f_suspension: w.fSuspension, f_tire: w.fTire,
      })),
    };
  }
}

window.Vehicle = Vehicle;
window.Wheel = Wheel;
