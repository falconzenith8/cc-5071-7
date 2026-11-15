// Rigid-body wheeled vehicle driven by explicit contact queries.
// JavaScript port of engine/vehicle.py; keep the arithmetic in the same
// order so replay trajectories match the Python engine bit-for-bit.
'use strict';

const M = window.M3;
const GRAVITY = [0, -9.81, 0];

class Wheel {
  constructor(name, anchorBody, radius, powered, steered) {
    this.name = name;
    this.anchorBody = anchorBody;
    this.radius = radius;
    this.powered = powered;
    this.steered = steered;
    this.restLength = 0.35;
    this.maxTravel = 0.22;
    this.springK = 32000;
    this.tau = 0.12;
    this.compression = 0;
    this.compressionVel = 0;
    this.grounded = false;
    this.contactPoint = null;
    this.contactNormal = null;
    this.contactDistance = null;
    this.queryKind = null;
    this.spinGround = 0;
    this.spinAir = 0;
    this.omega = 0;
    this.steerAngle = 0;
    this.fSuspension = [0, 0, 0];
    this.fTire = [0, 0, 0];
    this.slip = 0;
    this.load = 0;
  }
  get spinTotal() { return this.spinGround + this.spinAir; }

  effectiveDamping(sprungMass) {
    const omegaN = Math.sqrt(this.springK / Math.max(sprungMass, 1e-6));
    let zeta = 1 / Math.max(this.tau * omegaN, 0.05);
    zeta = M.clamp(zeta, 0.15, 3);
    const c = 2 * zeta * Math.sqrt(this.springK * Math.max(sprungMass, 1e-6));
    return [c, zeta];
  }
}

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

    this.queryKind = params.query_kind ?? 'sphere';
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
    this.lastContactCount = 0;
    this.warnings = [];
    this.bodyContact = false;
  }

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
    if (cfg.tire_mu !== undefined) this.tireMu = +cfg.tire_mu;
  }

  wheelAnchorWorld(w) {
    return M.add(this.position, M.bodyToWorld(this.quat, w.anchorBody));
  }
  suspensionDir() {
    return M.norm(M.bodyToWorld(this.quat, [0, -1, 0]));
  }
  cast(origin, direction, length, radius) {
    return this.queryKind === 'ray'
      ? this.terrain.raycast(origin, direction, length)
      : this.terrain.sphereCast(origin, direction, length, radius);
  }

  step(dt, control) {
    this.warnings = [];
    let forceTotal = [0, 0, 0];
    let torqueTotal = [0, 0, 0];
    for (let i = 0; i < 3; i++) forceTotal[i] += GRAVITY[i] * this.mass;
    const speed = M.length(this.velocity);
    if (speed > 1e-6)
      forceTotal = M.add(forceTotal, M.scale(this.velocity, -this.aeroDrag * speed));

    this.lastContactCount = 0;
    const suspDir = this.suspensionDir();
    for (const w of this.wheels) {
      this.updateWheelContact(w, suspDir, dt);
      if (w.grounded) {
        this.lastContactCount++;
        const [fw, tw] = this.wheelForces(w, control, dt);
        w.fSuspension = fw; w.fTire = tw;
        forceTotal = M.add(M.add(forceTotal, fw), tw);
        const r = M.sub(w.contactPoint, this.position);
        torqueTotal = M.add(M.add(torqueTotal, M.cross(r, fw)), M.cross(r, tw));
      } else {
        w.fSuspension = [0, 0, 0]; w.fTire = [0, 0, 0]; w.load = 0;
        this.airSpin(w, control, dt);
      }
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
    this.integrateGroundSpin(dt);
    return this.snapshot();
  }

  updateWheelContact(w, suspDir, dt) {
    const anchor = this.wheelAnchorWorld(w);
    const length = w.restLength + w.maxTravel;
    const hit = this.cast(anchor, suspDir, length, w.radius);
    const prevGrounded = w.grounded;
    w.grounded = false;
    w.contactPoint = null; w.contactNormal = null; w.contactDistance = null;
    w.queryKind = this.queryKind;
    if (hit === null) {
      const rate = -w.compression / Math.max(w.tau, 1e-3);
      w.compressionVel = rate;
      w.compression = Math.max(0, w.compression + rate * dt);
      if (prevGrounded) this.warnings.push(w.name + ' lost support (edge/air)');
      return;
    }
    // Quantise the root so bisection rounding differences across runtimes
    // cannot feed the damper/tyre branches.
    const d = Math.round(hit.distance * 1e9) / 1e9;
    const compression = M.clamp(w.restLength - d, 0, w.maxTravel);
    if (d <= w.restLength + w.maxTravel) {
      w.grounded = true;
      w.contactPoint = hit.point;
      w.contactNormal = hit.normal;
      w.contactDistance = d;
      w.compressionVel = (compression - w.compression) / dt;
      w.compression = compression;
      if (!prevGrounded) this.warnings.push(w.name + ' gained support');
      if (compression >= w.maxTravel - 1e-4)
        this.warnings.push(w.name + ' suspension bottomed');
    }
  }

  wheelInertia(w) {
    const cornerMass = this.mass / this.wheels.length * 0.12 + 8;
    return 0.5 * cornerMass * w.radius * w.radius;
  }

  wheelForces(w, control, dt) {
    const n = w.contactNormal;
    const sprung = this.mass / this.wheels.length;
    const [c] = w.effectiveDamping(sprung);
    const fSpring = w.springK * w.compression;
    const fDamper = c * w.compressionVel;
    const fSuspMag = Math.max(fSpring + fDamper, 0);
    const fSusp = M.scale(n, fSuspMag);

    const r = M.sub(w.contactPoint, this.position);
    const vContact = M.add(this.velocity, M.cross(this.angularVelocity, r));

    const targetSteer = w.steered ? (control.steer ?? 0) * this.maxSteer : 0;
    w.steerAngle += (targetSteer - w.steerAngle) * Math.min(1, dt / 0.12);
    const fwd = M.norm(M.bodyToWorld(this.quat,
      [Math.cos(w.steerAngle), 0, Math.sin(w.steerAngle)]));
    const lat = M.norm(M.cross(n, fwd));
    const load = Math.max(M.dot(fSusp, n), 0);
    w.load = load;
    const vLong = M.dot(vContact, fwd);

    const inertiaW = this.wheelInertia(w);
    let tDrive = 0;
    if (w.powered) {
      const nPowered = this.wheels.filter(x => x.powered).length;
      tDrive = (control.throttle ?? 0) * this.engineForce * w.radius / nPowered;
    }
    let tBrake = (control.brake ?? 0) * this.brakeForce * w.radius;
    if (w.omega > 0) tBrake = -tBrake;
    else if (w.omega < 0) tBrake = Math.abs(tBrake);

    const rollingSpeed = w.omega * w.radius;
    const denom = Math.max(Math.abs(vLong), Math.abs(rollingSpeed), 1);
    w.slip = (rollingSpeed - vLong) / denom;

    const fSlip = this.tireMu * load * M.clamp(w.slip / 0.15, -1, 1);
    const idle = (control.throttle ?? 0) === 0 && tBrake === 0;
    let fRoll;
    if (idle && Math.abs(vLong) < 0.6)
      fRoll = -M.clamp(vLong * 800, -this.rollingResistance * 8,
                       this.rollingResistance * 8);
    else
      fRoll = -this.rollingResistance * (vLong >= 0 ? 1 : -1);
    const fLong = M.clamp(fSlip + fRoll, -this.tireMu * load, this.tireMu * load);

    const vLat = M.dot(vContact, lat);
    const fLatTarget = -vLat * this.mass * 6;
    const budget = Math.max((this.tireMu * load) ** 2 - fLong ** 2, 0);
    const fLat = M.clamp(fLatTarget, -Math.sqrt(budget), Math.sqrt(budget));
    const fTire = M.add(M.scale(fwd, fLong), M.scale(lat, fLat));

    const tReaction = -fLong * w.radius;
    w.omega += ((tDrive + tBrake + tReaction) / inertiaW) * dt;
    // Free-roll relaxation only while the patch carries load; a zero-load
    // wheel keeps its spin rather than snapping onto the contact velocity.
    if (tDrive === 0 && tBrake === 0 && load > 1) {
      const roll = vLong / w.radius;
      w.omega += (roll - w.omega) * Math.min(1, dt / 0.25);
    }
    return [fSusp, fTire];
  }

  airSpin(w, control, dt) {
    const inertiaW = this.wheelInertia(w);
    let tDrive = 0;
    if (w.powered) {
      const nPowered = this.wheels.filter(x => x.powered).length;
      tDrive = (control.throttle ?? 0) * this.engineForce * w.radius * 0.4 / nPowered;
    }
    let tBrake = (control.brake ?? 0) * this.brakeForce * w.radius;
    if (w.omega > 0) tBrake = -tBrake;
    else if (w.omega < 0) tBrake = Math.abs(tBrake);
    w.omega += ((tDrive + tBrake) / inertiaW) * dt;
    w.spinAir += w.omega * dt;
  }

  integrateGroundSpin(dt) {
    for (const w of this.wheels) {
      if (!w.grounded) continue;
      const r = M.sub(w.contactPoint, this.position);
      const vContact = M.add(this.velocity, M.cross(this.angularVelocity, r));
      const fwd = M.norm(M.bodyToWorld(this.quat,
        [Math.cos(w.steerAngle), 0, Math.sin(w.steerAngle)]));
      const vLong = M.dot(vContact, fwd);
      const rollOmega = vLong / w.radius;
      w.omega += (rollOmega - w.omega) * Math.min(1, dt / 0.10);
      w.spinGround += w.omega * dt;
    }
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
      wheels: this.wheels.map(w => ({
        name: w.name, grounded: w.grounded, query: w.queryKind,
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
