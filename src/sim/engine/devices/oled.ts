/**
 * The SSD1306 OLED module (kind "ssd1306"). The controller itself lives in i2c-devices.ts as a target on a board's
 * TWI bus; the engine owns one per component (ctx.boardPart) and routes the board's I2C traffic to it when its SDA
 * and SCL share nets with the board's. This model is the module's electrical side and its display report: a
 * behavioural supply load, power gating (a module without supply neither answers the bus nor keeps GDDRAM), and
 * the panel image each committed step.
 */
import type { AcDeviceContext, DeviceModel } from "../device-registry.js";
import { stampAcAdmittance } from "../ac-system.js";
import { stampResistor } from "../elements.js";
import type { Ssd1306 } from "../i2c-devices.js";

// About 20 mA at 5 V with the panel lit (the module's charge pump and a typical screenful), 10 uA asleep. The real
// draw grows with the pixels lit; the teaching point here is the image.
const LIT_OHMS = 250;
const ASLEEP_OHMS = 500_000;

// The members both the transient and the AC contexts carry.
type Topology = Pick<AcDeviceContext, "pinNode" | "isOpenPin" | "boardPart">;

function loadOhms(ctx: Topology, compId: string): number {
  const panel = ctx.boardPart(compId) as Ssd1306 | undefined;
  return panel?.lit ? LIT_OHMS : ASLEEP_OHMS;
}

function suppliedPins(ctx: Topology, compId: string): [number, number] | null {
  if (ctx.isOpenPin(compId, "vcc") || ctx.isOpenPin(compId, "gnd")) return null;
  const vcc = ctx.pinNode(compId, "vcc");
  return vcc >= 0 ? [vcc, ctx.pinNode(compId, "gnd")] : null;
}

export const ssd1306Model: DeviceModel = {
  kinds: ["ssd1306"],
  stamp: (ctx, comp, xGuess) => {
    if (!ctx.icPowerInfo(comp, xGuess).powered) return;
    const pins = suppliedPins(ctx, comp.id);
    if (pins) stampResistor(ctx.mna, pins[0], pins[1], loadOhms(ctx, comp.id));
  },
  updateDigital: (ctx, comp, x) => {
    const panel = ctx.boardPart(comp.id) as Ssd1306 | undefined;
    if (!panel) return;
    const powered = ctx.icPowerInfo(comp, x).powered;
    // Power returning is a power-on reset: registers to their defaults, GDDRAM cleared.
    if (powered && !panel.powered) panel.reset();
    panel.powered = powered;
    const image = panel.image();
    ctx.setDisplayInfo(comp.id, { kind: "ssd1306", ...image, on: powered && image.on });
  },
  updateCurrent: (ctx, comp, x) => {
    const pins = ctx.icPowerInfo(comp, x).powered ? suppliedPins(ctx, comp.id) : null;
    ctx.setElementCurrent(comp.id, pins ? (ctx.vAt(x, pins[0]) - ctx.vAt(x, pins[1])) / loadOhms(ctx, comp.id) : 0);
  },
  acStamp: (ctx, comp, ac) => {
    if (!ctx.icPowerInfoAtOp(comp).powered) return;
    const pins = suppliedPins(ctx, comp.id);
    if (pins) stampAcAdmittance(ac, pins[0], pins[1], 1 / loadOhms(ctx, comp.id), 0);
  },
};
