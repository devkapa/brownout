/**
 * The BMP280 pressure and temperature module (kind "bmp280"). The sensor itself lives in bmp280.ts as a target on a
 * board's TWI bus; the engine owns one per component (ctx.boardPart) and routes the board's I2C traffic to it when
 * its SDA and SCL share nets with the board's. This model is the module's electrical side: a behavioural supply load,
 * power gating (a module without supply neither answers the bus nor keeps its registers), and the ambient
 * temperature and pressure its parameters set, which the next conversion measures.
 */
import type { AcDeviceContext, DeviceModel } from "../device-registry.js";
import { stampAcAdmittance } from "../ac-system.js";
import { stampResistor } from "../elements.js";
import type { Bmp280 } from "../bmp280.js";

// The module's regulator and the sensor in normal mode draw a few hundred microamps; asleep the sensor alone takes
// about 0.1 uA. One load stands for both: the teaching point is the reading.
const MODULE_OHMS = 15_000;

type Topology = Pick<AcDeviceContext, "pinNode" | "isOpenPin" | "boardPart">;

function suppliedPins(ctx: Topology, compId: string): [number, number] | null {
  if (ctx.isOpenPin(compId, "vcc") || ctx.isOpenPin(compId, "gnd")) return null;
  const vcc = ctx.pinNode(compId, "vcc");
  return vcc >= 0 ? [vcc, ctx.pinNode(compId, "gnd")] : null;
}

/** The module's ambient temperature (degC) and pressure (Pa), from its parameters, clamped to the sensor's range. */
export function bmp280Ambient(params: Record<string, unknown>): { celsius: number; pascals: number } {
  const num = (value: unknown, fallback: number) => {
    const n = Number(value);
    return Number.isFinite(n) ? n : fallback;
  };
  // Operating range (datasheet section 1): -40 to 85 degC, 300 to 1100 hPa.
  const celsius = Math.min(85, Math.max(-40, num(params.temperature, 25)));
  const hpa = Math.min(1100, Math.max(300, num(params.pressure, 1013.25)));
  return { celsius, pascals: hpa * 100 };
}

export const bmp280Model: DeviceModel = {
  kinds: ["bmp280"],
  stamp: (ctx, comp, xGuess) => {
    if (!ctx.icPowerInfo(comp, xGuess).powered) return;
    const pins = suppliedPins(ctx, comp.id);
    if (pins) stampResistor(ctx.mna, pins[0], pins[1], MODULE_OHMS);
  },
  updateDigital: (ctx, comp, x) => {
    const sensor = ctx.boardPart(comp.id) as Bmp280 | undefined;
    if (!sensor) return;
    const powered = ctx.icPowerInfo(comp, x).powered;
    // Power returning is a power-on reset: registers to their defaults, no conversion yet.
    if (powered && !sensor.powered) sensor.reset();
    sensor.powered = powered;
    const { celsius, pascals } = bmp280Ambient(comp.params);
    sensor.setAmbient(celsius, pascals);
  },
  updateCurrent: (ctx, comp, x) => {
    const pins = ctx.icPowerInfo(comp, x).powered ? suppliedPins(ctx, comp.id) : null;
    ctx.setElementCurrent(comp.id, pins ? (ctx.vAt(x, pins[0]) - ctx.vAt(x, pins[1])) / MODULE_OHMS : 0);
  },
  acStamp: (ctx, comp, ac) => {
    if (!ctx.icPowerInfoAtOp(comp).powered) return;
    const pins = suppliedPins(ctx, comp.id);
    if (pins) stampAcAdmittance(ac, pins[0], pins[1], 1 / MODULE_OHMS, 0);
  },
};
