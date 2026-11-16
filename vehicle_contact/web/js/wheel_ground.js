// Wheel-ground contact boundary (JavaScript mirror of engine/wheel_ground).
//
// Same boundaries as the Python module: frames (contact velocity/force
// coordinates), support (explicit query + finite-reach rule), suspension,
// tire, and per-wheel orchestration.  Arithmetic order matches the Python
// reference so replay trajectories stay identical.
'use strict';

const M = window.M3;

// ---------------------------------------------------------------- frames
function relativeVelocity(bodyVelocity, bodyAngularVelocity, contactOffset) {
  return M.add(bodyVelocity, M.cross(bodyAngularVelocity, contactOffset));
}

class ContactFrame {
  constructor(normal, forward) {
    this.normal = normal;
    this.forward = forward;
    this.lateral = M.norm(M.cross(normal, forward));
  }
  decompose(worldVector) {
    return [M.dot(worldVector, this.normal),
            M.dot(worldVector, this.forward),
            M.dot(worldVector, this.lateral)];
  }
  force(forwardComponent, lateralComponent) {
    return M.add(M.scale(this.forward, forwardComponent),
                 M.scale(this.lateral, lateralComponent));
  }
}

// --------------------------------------------------------------- support
class SupportQuery {
  constructor(terrain, kind = 'sphere') {
    if (kind !== 'ray' && kind !== 'sphere')
      throw new Error('unknown support query kind: ' + kind);
    this.terrain = terrain;
    this.kind = kind;
  }
  probe(origin, direction, maxDistance, radius) {
    const hit = this.kind === 'ray'
      ? this.terrain.raycast(origin, direction, maxDistance)
      : this.terrain.sphereCast(origin, direction, maxDistance, radius);
    if (hit === null) return null;
    return {distance: hit.distance, point: hit.point, normal: hit.normal,
            centre: hit.centre ?? null};
  }
  static quantizeDistance(distance) {
    return Math.round(distance * 1e9) / 1e9;
  }
  static withinReach(distance, restLength, maxTravel) {
    return distance <= restLength + maxTravel;
  }
}

// ----------------------------------------------------------- suspension
class SuspensionParameters {
  constructor(restLength = 0.35, maxTravel = 0.22, springK = 32000,
              tau = 0.12) {
    this.restLength = restLength;
    this.maxTravel = maxTravel;
    this.springK = springK;
    this.tau = tau;
  }
  supportCompression(distance) {
    return M.clamp(this.restLength - distance, 0, this.maxTravel);
  }
  effectiveDamping(sprungMass) {
    const omegaN = Math.sqrt(this.springK / Math.max(sprungMass, 1e-6));
    let zeta = 1 / Math.max(this.tau * omegaN, 0.05);
    zeta = M.clamp(zeta, 0.15, 3);
    const c = 2 * zeta * Math.sqrt(this.springK * Math.max(sprungMass, 1e-6));
    return [c, zeta];
  }
}

function evaluateSuspension(params, compression, compressionVel, normal,
                            sprungMass) {
  const [c, zeta] = params.effectiveDamping(sprungMass);
  const springForce = params.springK * compression;
  const damperForce = c * compressionVel;
  const magnitude = Math.max(springForce + damperForce, 0);
  return {c, zeta, springForce, damperForce, magnitude,
          force: M.scale(normal, magnitude)};
}

function relaxAirborne(params, compression, dt) {
  const rate = -compression / Math.max(params.tau, 1e-3);
  return [Math.max(0, compression + rate * dt), rate];
}

// ---------------------------------------------------------------- tire
class TireParameters {
  constructor(mu = 1.1, engineForce = 3200, brakeForce = 6000,
              rollingResistance = 26) {
    this.mu = mu;
    this.engineForce = engineForce;
    this.brakeForce = brakeForce;
    this.rollingResistance = rollingResistance;
  }
}

function updateSteer(steerAngle, targetSteer, dt) {
  return steerAngle + (targetSteer - steerAngle) * Math.min(1, dt / 0.12);
}

function brakeTorque(params, control, radius, omega) {
  let tBrake = (control.brake ?? 0) * params.brakeForce * radius;
  if (omega > 0) tBrake = -tBrake;
  else if (omega < 0) tBrake = Math.abs(tBrake);
  return tBrake;
}

function driveTorque(params, control, radius, powered, poweredCount,
                     scale = 1) {
  if (!powered) return 0;
  return (control.throttle ?? 0) * params.engineForce * radius * scale
    / poweredCount;
}

function evaluateTire(params, control, load, vLong, vLat, forward, lateral,
                      omega, radius, inertia, bodyMass, powered,
                      poweredCount, dt) {
  const tDrive = driveTorque(params, control, radius, powered, poweredCount);
  const tBrake = brakeTorque(params, control, radius, omega);

  const rollingSpeed = omega * radius;
  const denom = Math.max(Math.abs(vLong), Math.abs(rollingSpeed), 1);
  const slip = (rollingSpeed - vLong) / denom;

  const fSlip = params.mu * load * M.clamp(slip / 0.15, -1, 1);
  const idle = (control.throttle ?? 0) === 0 && tBrake === 0;
  let fRoll;
  if (idle && Math.abs(vLong) < 0.6)
    fRoll = -M.clamp(vLong * 800, -params.rollingResistance * 8,
                     params.rollingResistance * 8);
  else
    fRoll = -params.rollingResistance * (vLong >= 0 ? 1 : -1);
  const fLong = M.clamp(fSlip + fRoll, -params.mu * load, params.mu * load);

  const fLatTarget = -vLat * bodyMass * 6;
  const budget = Math.max((params.mu * load) ** 2 - fLong ** 2, 0);
  const fLat = M.clamp(fLatTarget, -Math.sqrt(budget), Math.sqrt(budget));
  const fTire = M.add(M.scale(forward, fLong), M.scale(lateral, fLat));

  const tReaction = -fLong * radius;
  omega = omega + ((tDrive + tBrake + tReaction) / inertia) * dt;
  if (tDrive === 0 && tBrake === 0 && load > 1) {
    const roll = vLong / radius;
    omega += (roll - omega) * Math.min(1, dt / 0.25);
  }
  return {slip, slipForce: fSlip, rollingForce: fRoll, longitudinal: fLong,
          lateral: fLat, force: fTire, driveTorque: tDrive,
          brakeTorque: tBrake, reactionTorque: tReaction, omega};
}

function freeSpinOmega(params, control, radius, inertia, powered,
                       poweredCount, omega, dt) {
  const tDrive = driveTorque(params, control, radius, powered, poweredCount,
                             0.4);
  const tBrake = brakeTorque(params, control, radius, omega);
  return omega + ((tDrive + tBrake) / inertia) * dt;
}

function relaxToRolling(omega, vLong, radius, dt) {
  const rollOmega = vLong / radius;
  return omega + (rollOmega - omega) * Math.min(1, dt / 0.10);
}

// ---------------------------------------------------------------- wheel
class Wheel {
  constructor(name, anchorBody, radius, powered, steered) {
    this.name = name;
    this.anchorBody = anchorBody;
    this.radius = radius;
    this.powered = powered;
    this.steered = steered;
    this.suspension = new SuspensionParameters();
    this.compressionValue = 0;
    this.compressionVel = 0;
    this.grounded = false;
    this.contactPoint = null;
    this.contactNormal = null;
    this.contactDistance = null;
    this.queryKind = null;
    this.center = null;
    this.spinGround = 0;
    this.spinAir = 0;
    this.omega = 0;
    this.steerAngle = 0;
    this.steerDeg = 0;
    this.fSuspension = [0, 0, 0];
    this.fTire = [0, 0, 0];
    this.slip = 0;
    this.load = 0;
  }
  get restLength() { return this.suspension.restLength; }
  set restLength(v) { this.suspension.restLength = v; }
  get maxTravel() { return this.suspension.maxTravel; }
  set maxTravel(v) { this.suspension.maxTravel = v; }
  get springK() { return this.suspension.springK; }
  set springK(v) { this.suspension.springK = v; }
  get tau() { return this.suspension.tau; }
  set tau(v) { this.suspension.tau = v; }
  get compression() { return this.compressionValue; }
  set compression(v) { this.compressionValue = v; }
  get spinTotal() { return this.spinGround + this.spinAir; }
  effectiveDamping(sprungMass) {
    return this.suspension.effectiveDamping(sprungMass);
  }
}

class WheelGroundModule {
  constructor(terrain, wheels, queryKind = 'sphere', tire = null,
              steerLimit = 0, query = null) {
    this.terrain = terrain;
    this.wheels = wheels;
    this.query = query || new SupportQuery(terrain, queryKind);
    this.tire = tire || new TireParameters();
    this.steerLimit = steerLimit;
  }

  wheelAnchorWorld(body, wheel) {
    return M.add(body.position, M.bodyToWorld(body.quat, wheel.anchorBody));
  }
  suspensionDirection(body) {
    return M.norm(M.bodyToWorld(body.quat, [0, -1, 0]));
  }

  perWheelStep(body, control, dt) {
    const steps = [];
    const warnings = [];
    const suspDir = this.suspensionDirection(body);
    const poweredCount = this.wheels.filter(w => w.powered).length;

    for (const w of this.wheels) {
      w.queryKind = this.query.kind;
      const prevGrounded = w.grounded;
      w.grounded = false;
      w.contactPoint = null;
      w.contactNormal = null;
      w.contactDistance = null;

      const anchor = this.wheelAnchorWorld(body, w);
      const probe = this.query.probe(anchor, suspDir,
                                     w.restLength + w.maxTravel, w.radius);

      if (probe === null) {
        this.unsupported(w, dt, prevGrounded, warnings, true);
        w.center = M.add(anchor,
          M.scale(suspDir, w.restLength + w.maxTravel));
        this.airSpin(w, body, control, dt, poweredCount);
        steps.push({wheel: w, grounded: false, fSuspension: [0, 0, 0],
                    fTire: [0, 0, 0], contactOffset: null});
        continue;
      }

      const distance = SupportQuery.quantizeDistance(probe.distance);
      if (!SupportQuery.withinReach(distance, w.restLength, w.maxTravel)) {
        // A query hit is a candidate only: beyond the finite reach it is not
        // support, carries no force and is never retained.
        this.unsupported(w, dt, prevGrounded, warnings, false);
        w.center = M.add(anchor,
          M.scale(suspDir, w.restLength + w.maxTravel));
        this.airSpin(w, body, control, dt, poweredCount);
        steps.push({wheel: w, grounded: false, fSuspension: [0, 0, 0],
                    fTire: [0, 0, 0], contactOffset: null});
        continue;
      }

      const compression = w.suspension.supportCompression(distance);
      w.grounded = true;
      w.contactPoint = probe.point;
      w.contactNormal = probe.normal;
      w.contactDistance = distance;
      w.center = M.add(anchor, M.scale(suspDir, distance));
      w.compressionVel = (compression - w.compression) / dt;
      w.compression = compression;
      if (!prevGrounded) warnings.push(w.name + ' gained support');
      if (compression >= w.maxTravel - 1e-4)
        warnings.push(w.name + ' suspension bottomed');

      const res = this.groundForces(body, w, control, dt, poweredCount);
      w.fSuspension = res.fSuspension;
      w.fTire = res.fTire;
      w.steerDeg = M.deg(w.steerAngle);
      steps.push({wheel: w, grounded: true, fSuspension: res.fSuspension,
                  fTire: res.fTire, contactOffset: res.contactOffset});
    }
    return {steps, warnings};
  }

  unsupported(w, dt, prevGrounded, warnings, relax) {
    if (relax) {
      const [compression, rate] = relaxAirborne(w.suspension, w.compression,
                                                dt);
      w.compressionVel = rate;
      w.compression = compression;
    }
    if (prevGrounded) warnings.push(w.name + ' lost support (edge/air)');
    w.fSuspension = [0, 0, 0];
    w.fTire = [0, 0, 0];
    w.load = 0;
  }

  groundForces(body, w, control, dt, poweredCount) {
    const n = w.contactNormal;
    const sprung = body.mass / this.wheels.length;
    const fSusp = evaluateSuspension(w.suspension, w.compression,
                                     w.compressionVel, n, sprung).force;

    const offset = M.sub(w.contactPoint, body.position);
    const vContact = relativeVelocity(body.velocity, body.angularVelocity,
                                      offset);

    const targetSteer = w.steered ? (control.steer ?? 0) * this.steerLimit : 0;
    w.steerAngle = updateSteer(w.steerAngle, targetSteer, dt);
    const fwd = M.norm(M.bodyToWorld(body.quat,
      [Math.cos(w.steerAngle), 0, Math.sin(w.steerAngle)]));
    const frame = new ContactFrame(n, fwd);

    const load = Math.max(M.dot(fSusp, n), 0);
    w.load = load;
    const [, vLong, vLat] = frame.decompose(vContact);

    const tres = evaluateTire(
      this.tire, control, load, vLong, vLat, frame.forward, frame.lateral,
      w.omega, w.radius, this.wheelInertia(w, body), body.mass, w.powered,
      poweredCount, dt);
    w.slip = tres.slip;
    w.omega = tres.omega;
    return {fSuspension: fSusp, fTire: tres.force, contactOffset: offset};
  }

  wheelInertia(w, body) {
    const cornerMass = body.mass / this.wheels.length * 0.12 + 8;
    return 0.5 * cornerMass * w.radius * w.radius;
  }

  airSpin(w, body, control, dt, poweredCount) {
    w.omega = freeSpinOmega(this.tire, control, w.radius,
                            this.wheelInertia(w, body), w.powered,
                            poweredCount, w.omega, dt);
    w.spinAir += w.omega * dt;
  }

  integrateGroundSpin(body, dt) {
    for (const w of this.wheels) {
      if (!w.grounded) continue;
      const offset = M.sub(w.contactPoint, body.position);
      const vContact = relativeVelocity(body.velocity, body.angularVelocity,
                                        offset);
      const fwd = M.norm(M.bodyToWorld(body.quat,
        [Math.cos(w.steerAngle), 0, Math.sin(w.steerAngle)]));
      const vLong = M.dot(vContact, fwd);
      w.omega = relaxToRolling(w.omega, vLong, w.radius, dt);
      w.spinGround += w.omega * dt;
    }
  }
}

// ------------------------------------------------------------ interface
const WHEEL_GROUND_PREFIXES = ['gained support', 'lost support',
                               'suspension bottomed', 'all wheels airborne'];
const CHASSIS_PREFIXES = ['chassis hull contacted terrain'];

function classifyWarnings(warnings) {
  const groups = {wheel_ground: [], chassis_motion: []};
  for (const text of warnings) {
    if (CHASSIS_PREFIXES.some(p => text.startsWith(p)))
      groups.chassis_motion.push(text);
    else
      groups.wheel_ground.push(text);
  }
  return groups;
}

function wheelGroundState(vehicle) {
  const wheels = vehicle.wheels.map(w => ({
    name: w.name, grounded: w.grounded, query: w.queryKind,
    center: w.center, contact_point: w.contactPoint,
    contact_normal: w.contactNormal, contact_distance: w.contactDistance,
    compression: w.compression, compression_vel: w.compressionVel,
    max_travel: w.maxTravel,
    travel_ratio: w.compression / Math.max(w.maxTravel, 1e-9),
    load: w.load, slip: w.slip, omega: w.omega, steer_deg: w.steerDeg,
    spin_ground: w.spinGround, spin_air: w.spinAir,
    f_suspension: w.fSuspension, f_tire: w.fTire,
  }));
  return {
    contact_count: vehicle.lastContactCount,
    body_contact: vehicle.bodyContact,
    wheel_ground: wheels,
    anomalies: classifyWarnings(vehicle.warnings),
  };
}

window.WG = {relativeVelocity, ContactFrame, SupportQuery,
             SuspensionParameters, evaluateSuspension, relaxAirborne,
             TireParameters, evaluateTire, freeSpinOmega, relaxToRolling,
             updateSteer, Wheel, WheelGroundModule, classifyWarnings,
             wheelGroundState};
