/**
 * The DHT22 (AM2302) or DHT11 humidity and temperature module (kind "dht", its `variant` parameter 22 or 11). The
 * sensor itself lives in dht.ts as a target on the board pin its DATA line shares a net with; the engine owns one per
 * component (ctx.boardPart) and attaches it to that pin. This model is the module's electrical side: the 10 kOhm
 * pull-up the 3-pin modules carry from DATA to VCC, a behavioural supply load, power gating (an unpowered sensor
 * neither answers nor holds the line, and needs its first second of power again), and the ambient humidity and
 * temperature its parameters set, which the next measurement reads.
 */
import type { AcDeviceContext, DeviceModel } from "../device-registry.js";
import { stampAcAdmittance } from "../ac-system.js";
import { stampResistor } from "../elements.js";
import type { DhtSensor } from "../dht.js";

const PULL_UP_OHMS = 10_000;
// About 0.5 mA averaged over its measuring and standby (AM2302: 1-1.5 mA measuring, 40-50 uA standby).
const SUPPLY_OHMS = 10_000;

type Topology = Pick<AcDeviceContext, "pinNode" | "isOpenPin">;

// When each sensor last came up powered, on the simulation's clock.
const poweredSince = new WeakMap<DhtSensor, number>();

function pin(ctx: Topology, compId: string, pinId: string): number | null {
  return ctx.isOpenPin(compId, pinId) ? null : ctx.pinNode(compId, pinId);
}

/** The module's ambient humidity (%RH) and temperature (degC), from its parameters. */
export function dhtAmbient(params: Record<string, unknown>): { humidity: number; celsius: number } {
  const num = (value: unknown, fallback: number) => {
    const n = Number(value);
    return Number.isFinite(n) ? n : fallback;
  };
  return { humidity: num(params.humidity, 50), celsius: num(params.temperature, 25) };
}

function stampModule(ctx: Topology, compId: string, stamp: (a: number, b: number, ohms: number) => void, powered: boolean): void {
  const vcc = pin(ctx, compId, "vcc");
  const data = pin(ctx, compId, "data");
  // The pull-up is a resistor on the module: it joins DATA to VCC whether or not the sensor is supplied.
  if (vcc !== null && data !== null) stamp(data, vcc, PULL_UP_OHMS);
  const gnd = pin(ctx, compId, "gnd");
  if (powered && vcc !== null && vcc >= 0 && gnd !== null) stamp(vcc, gnd, SUPPLY_OHMS);
}

export const dhtModel: DeviceModel = {
  kinds: ["dht"],
  stamp: (ctx, comp, xGuess) => {
    stampModule(ctx, comp.id, (a, b, ohms) => stampResistor(ctx.mna, a, b, ohms), ctx.icPowerInfo(comp, xGuess).powered);
  },
  updateDigital: (ctx, comp, x) => {
    const sensor = ctx.boardPart(comp.id) as DhtSensor | undefined;
    if (!sensor) return;
    const powered = ctx.icPowerInfo(comp, x).powered;
    const now = ctx.simTime();
    if (powered && !sensor.powered) {
      sensor.reset();
      poweredSince.set(sensor, now);
    }
    sensor.powered = powered;
    // Measured from the step that found it powered, so it is late by at most one step; a rewound clock starts over.
    const since = Math.min(now, poweredSince.get(sensor) ?? now);
    poweredSince.set(sensor, since);
    sensor.poweredFor = powered ? now - since : 0;
    const { humidity, celsius } = dhtAmbient(comp.params);
    sensor.setAmbient(humidity, celsius);
  },
  updateCurrent: (ctx, comp, x) => {
    const vcc = pin(ctx, comp.id, "vcc");
    const gnd = pin(ctx, comp.id, "gnd");
    const powered = ctx.icPowerInfo(comp, x).powered;
    ctx.setElementCurrent(comp.id, powered && vcc !== null && gnd !== null ? (ctx.vAt(x, vcc) - ctx.vAt(x, gnd)) / SUPPLY_OHMS : 0);
  },
  acStamp: (ctx, comp, ac) => {
    stampModule(ctx, comp.id, (a, b, ohms) => stampAcAdmittance(ac, a, b, 1 / ohms, 0), ctx.icPowerInfoAtOp(comp).powered);
  },
};
