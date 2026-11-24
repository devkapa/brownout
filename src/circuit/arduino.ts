import type { CircuitComponent, ComponentKind, Pin, Position, Rotation } from "./types.js";

/** Both Arduino boards share the ATmega328P core, params, USB-power model, and pin name space. */
export function isArduinoBoardKind(kind: ComponentKind | string): boolean {
  return kind === "arduino_uno" || kind === "arduino_nano";
}

/**
 * Board kinds hosted by a `MicrocontrollerCore` in the engine (`state.arduinos`).
 * Grows as new MCU boards are added (e.g. `raspberry_pi_pico`); today it is
 * exactly the Arduino kinds. Prefer this over `isArduinoBoardKind` in
 * core-agnostic engine loops so a new board is picked up automatically.
 */
export function isMcuBoardKind(kind: ComponentKind | string): boolean {
  return isArduinoBoardKind(kind) || kind === "raspberry_pi_pico";
}

// Footprint of the de:volt board art (public/parts/arduino-uno-board.svg),
// drawn at real size: the host draws 19 px per 0.1" (7.48 px/mm), and the Uno
// R3 PCB is 68.58 x 53.34 mm. The PCB body maps to (0,0)-(WIDTH,HEIGHT); the
// USB-B connector + barrel jack overhang the left edge (negative x, covered by
// ARDUINO_UNO_BOUNDS.x).
//
// The board art and the pin offsets below live in ONE coordinate space. A
// single origin + scale maps SVG units to component-local px so every wired
// terminal sits exactly on its drawn header hole. Component-local (0,0) = the
// PCB's top-left corner = SVG (142.85, 36.4). The scale is set by the PCB
// outline (807.74 SVG units wide); the art's header pitch comes out at about
// 2.6 mm, a property of the board art. canvas/parts/arduino-uno.ts imports
// these to place the board image, LEDs, reset dome and USB plug.
export const ARDUINO_UNO_SVG_ORIGIN_X = 142.85;
export const ARDUINO_UNO_SVG_ORIGIN_Y = 36.4;
const UNO_PCB_SVG_WIDTH = 807.74;
const PX_PER_MM = 19 / 2.54;
export const ARDUINO_UNO_SVG_SCALE = round4((68.58 * PX_PER_MM) / UNO_PCB_SVG_WIDTH);
/**
 * The scale this board was drawn at before it became real size (about half).
 * Hosts multiply hand-placed art offsets by ARDUINO_UNO_ART_GROWTH so they
 * scale with the board.
 */
export const ARDUINO_UNO_LEGACY_SVG_SCALE = 0.3256;
export const ARDUINO_UNO_ART_GROWTH = ARDUINO_UNO_SVG_SCALE / ARDUINO_UNO_LEGACY_SVG_SCALE;
const grow = (n: number) => round2(n * ARDUINO_UNO_ART_GROWTH);

export const ARDUINO_UNO_WIDTH = Math.round(UNO_PCB_SVG_WIDTH * ARDUINO_UNO_SVG_SCALE);
export const ARDUINO_UNO_HEIGHT = Math.round((204 / ARDUINO_UNO_LEGACY_SVG_SCALE) * ARDUINO_UNO_SVG_SCALE);
const UNO_OVERHANG_X = grow(-80);
export const ARDUINO_UNO_BOUNDS = {
  x: UNO_OVERHANG_X,
  y: 0,
  w: ARDUINO_UNO_WIDTH - UNO_OVERHANG_X,
  h: ARDUINO_UNO_HEIGHT,
} as const;

// Clickable USB region: the USB-B socket on the left edge plus the sliding cable
// plug (drawn in canvas/parts/arduino-uno.ts). Pressing it toggles usb_power,
// which eases the plug in/out.
export const ARDUINO_USB_TOGGLE_BOUNDS = {
  x: grow(-76),
  y: grow(32),
  w: grow(116),
  h: grow(49),
} as const;

/** Component-local hit region for the tactile reset button (matches renderer art). */
export const ARDUINO_RESET_BUTTON_BOUNDS = {
  x: grow(11.5),
  y: grow(1.5),
  w: grow(24.5),
  h: grow(24.5),
} as const;

function round4(n: number): number {
  return Math.round(n * 10000) / 10000;
}

/** Map an SVG-space X to component-local px. */
export function arduinoUnoSvgX(x: number): number {
  return (x - ARDUINO_UNO_SVG_ORIGIN_X) * ARDUINO_UNO_SVG_SCALE;
}
/** Map an SVG-space Y to component-local px. */
export function arduinoUnoSvgY(y: number): number {
  return (y - ARDUINO_UNO_SVG_ORIGIN_Y) * ARDUINO_UNO_SVG_SCALE;
}

// Header holes, laid out the way the Uno R3 drawing places them: every header
// on a 0.1" (19 px) grid, with the 160 mil step between D7 and D8 that keeps
// shields from plugging in the wrong way round, VIN under D7, and A0 two
// pitches right of VIN. Two holes anchor the grid where the board art has
// them: D7 for the top row, VIN for the bottom row. The art's own holes were
// traced about 2.7 % too far apart; hosts redraw them from these offsets.
export const ARDUINO_UNO_HEADER_PITCH = 19;
/** D7 to D8: 160 mil. */
export const ARDUINO_UNO_D7_D8_STEP = 1.6 * ARDUINO_UNO_HEADER_PITCH;
const AU_TOP_HOLE_Y = 55.21; // digital header (top edge), SVG units
const AU_BOTTOM_HOLE_Y = 637.87; // power + analog headers (bottom edge), SVG units
const AU_D7_HOLE_X = 681.79;
const AU_VIN_HOLE_X = 683.52;
function round2(n: number): number {
  return Math.round(n * 100) / 100;
}
const TOP_Y = round2(arduinoUnoSvgY(AU_TOP_HOLE_Y));
const BOTTOM_Y = round2(arduinoUnoSvgY(AU_BOTTOM_HOLE_Y));
const D7_X = round2(arduinoUnoSvgX(AU_D7_HOLE_X));
const D8_X = D7_X - ARDUINO_UNO_D7_D8_STEP;
const VIN_X = round2(arduinoUnoSvgX(AU_VIN_HOLE_X));
const P = ARDUINO_UNO_HEADER_PITCH;
const auTop = (x: number): Position => ({ x, y: TOP_Y });
const auBot = (x: number): Position => ({ x, y: BOTTOM_Y });

/**
 * Every header hole the board has, modelled pin or not, in component-local
 * px: left to right along each row. Hosts redraw the art's holes from this.
 */
export const ARDUINO_UNO_HEADER_HOLES: {
  top: readonly number[];
  bottom: readonly number[];
  topY: number;
  bottomY: number;
} = {
  // SCL, SDA, AREF, GND, D13..D8 | D7..D0
  top: [
    ...Array.from({ length: 10 }, (_, i) => D8_X - (9 - i) * P),
    ...Array.from({ length: 8 }, (_, i) => D7_X + i * P),
  ],
  // NC, IOREF, RST, 3V3, 5V, GND, GND, VIN | A0..A5
  bottom: [
    ...Array.from({ length: 8 }, (_, i) => VIN_X - (7 - i) * P),
    ...Array.from({ length: 6 }, (_, i) => VIN_X + (2 + i) * P),
  ],
  topY: TOP_Y,
  bottomY: BOTTOM_Y,
};

export const ARDUINO_UNO_PINS: Pin[] = [
  // Power header (bottom-left). The leftmost hole is an unlabeled NC pad.
  { id: "reset", label: "RST", offset: auBot(VIN_X - 5 * P) },
  { id: "io_ref", label: "IOREF", offset: auBot(VIN_X - 6 * P) },
  { id: "3v3", label: "3V3", offset: auBot(VIN_X - 4 * P) },
  { id: "5v", label: "5V", offset: auBot(VIN_X - 3 * P) },
  { id: "gnd", label: "GND", offset: auBot(VIN_X - 2 * P) },
  { id: "gnd2", label: "GND", offset: auBot(VIN_X - P) },
  { id: "vin", label: "VIN", offset: auBot(VIN_X) },

  // Analog header (bottom-right).
  { id: "a0", label: "A0", offset: auBot(VIN_X + 2 * P) },
  { id: "a1", label: "A1", offset: auBot(VIN_X + 3 * P) },
  { id: "a2", label: "A2", offset: auBot(VIN_X + 4 * P) },
  { id: "a3", label: "A3", offset: auBot(VIN_X + 5 * P) },
  { id: "a4", label: "A4", offset: auBot(VIN_X + 6 * P) },
  { id: "a5", label: "A5", offset: auBot(VIN_X + 7 * P) },

  // Digital header (top). The SCL, SDA and top-GND holes are decorative
  // (no engine node) — the sim exposes AREF + D0..D13, as the prior model did.
  { id: "aref", label: "AREF", offset: auTop(D8_X - 7 * P) },
  { id: "d13", label: "13", offset: auTop(D8_X - 5 * P) },
  { id: "d12", label: "12", offset: auTop(D8_X - 4 * P) },
  { id: "d11", label: "11", offset: auTop(D8_X - 3 * P) },
  { id: "d10", label: "10", offset: auTop(D8_X - 2 * P) },
  { id: "d9", label: "9", offset: auTop(D8_X - P) },
  { id: "d8", label: "8", offset: auTop(D8_X) },
  { id: "d7", label: "7", offset: auTop(D7_X) },
  { id: "d6", label: "6", offset: auTop(D7_X + P) },
  { id: "d5", label: "5", offset: auTop(D7_X + 2 * P) },
  { id: "d4", label: "4", offset: auTop(D7_X + 3 * P) },
  { id: "d3", label: "3", offset: auTop(D7_X + 4 * P) },
  { id: "d2", label: "2", offset: auTop(D7_X + 5 * P) },
  { id: "d1", label: "1", offset: auTop(D7_X + 6 * P) },
  { id: "d0", label: "0", offset: auTop(D7_X + 7 * P) },
];

export const ARDUINO_UNO_PIN_IDS = new Set(ARDUINO_UNO_PINS.map((pin) => pin.id));

export function cloneArduinoUnoPins(): Pin[] {
  return ARDUINO_UNO_PINS.map((pin) => ({
    ...pin,
    offset: { ...pin.offset },
  }));
}

export function mapLegacyArduinoPinId(pinId: string): string | null {
  switch (pinId) {
    case "vcc":
    case "avcc":
      return "5v";
    case "gnd":
      return "gnd";
    case "gnd2":
    case "gnd_top":
      return "gnd2";
    case "xtal1":
    case "xtal2":
      return null;
    default:
      return ARDUINO_UNO_PIN_IDS.has(pinId) ? pinId : null;
  }
}

export function normalizeArduinoUnoPins(pins: Pin[] = []): Pin[] {
  const byId = new Map<string, Pin>();
  for (const pin of pins) {
    const id = mapLegacyArduinoPinId(pin.id);
    if (!id || byId.has(id)) continue;
    byId.set(id, pin);
  }
  return ARDUINO_UNO_PINS.map((canonical) => {
    const existing = byId.get(canonical.id);
    return {
      ...canonical,
      net_id: existing?.net_id ?? canonical.net_id,
      offset: { ...canonical.offset },
    };
  });
}

/**
 * Internal DIP order for the Nano. The bottom edge is left-to-right; the top
 * edge is right-to-left so the generic DIP placer produces the visible
 * left-to-right silkscreen order D12 ... TX1.
 */
export const ARDUINO_NANO_PINS: Pin[] = [
  { id: "d13",    label: "D13", offset: { x: 0, y: 0 }, function: "io" },
  { id: "3v3",    label: "3V3", offset: { x: 0, y: 0 }, function: "vcc" },
  { id: "aref",   label: "REF", offset: { x: 0, y: 0 }, function: "input" },
  { id: "a0",     label: "A0",  offset: { x: 0, y: 0 }, function: "analog_in" },
  { id: "a1",     label: "A1",  offset: { x: 0, y: 0 }, function: "analog_in" },
  { id: "a2",     label: "A2",  offset: { x: 0, y: 0 }, function: "analog_in" },
  { id: "a3",     label: "A3",  offset: { x: 0, y: 0 }, function: "analog_in" },
  { id: "a4",     label: "A4",  offset: { x: 0, y: 0 }, function: "analog_in" },
  { id: "a5",     label: "A5",  offset: { x: 0, y: 0 }, function: "analog_in" },
  { id: "a6",     label: "A6",  offset: { x: 0, y: 0 }, function: "analog_in" },
  { id: "a7",     label: "A7",  offset: { x: 0, y: 0 }, function: "analog_in" },
  { id: "5v",     label: "5V",  offset: { x: 0, y: 0 }, function: "vcc" },
  { id: "reset2", label: "RST", offset: { x: 0, y: 0 }, function: "reset" },
  { id: "gnd2",   label: "GND", offset: { x: 0, y: 0 }, function: "gnd" },
  { id: "vin",    label: "VIN", offset: { x: 0, y: 0 }, function: "supply_pos" },
  { id: "d1",     label: "TX1", offset: { x: 0, y: 0 }, function: "io" },
  { id: "d0",     label: "RX0", offset: { x: 0, y: 0 }, function: "io" },
  { id: "reset",  label: "RST", offset: { x: 0, y: 0 }, function: "reset" },
  { id: "gnd",    label: "GND", offset: { x: 0, y: 0 }, function: "gnd" },
  { id: "d2",     label: "D2",  offset: { x: 0, y: 0 }, function: "io" },
  { id: "d3",     label: "D3",  offset: { x: 0, y: 0 }, function: "io" },
  { id: "d4",     label: "D4",  offset: { x: 0, y: 0 }, function: "io" },
  { id: "d5",     label: "D5",  offset: { x: 0, y: 0 }, function: "io" },
  { id: "d6",     label: "D6",  offset: { x: 0, y: 0 }, function: "io" },
  { id: "d7",     label: "D7",  offset: { x: 0, y: 0 }, function: "io" },
  { id: "d8",     label: "D8",  offset: { x: 0, y: 0 }, function: "io" },
  { id: "d9",     label: "D9",  offset: { x: 0, y: 0 }, function: "io" },
  { id: "d10",    label: "D10", offset: { x: 0, y: 0 }, function: "io" },
  { id: "d11",    label: "D11", offset: { x: 0, y: 0 }, function: "io" },
  { id: "d12",    label: "D12", offset: { x: 0, y: 0 }, function: "io" },
];

export const ARDUINO_NANO_PIN_IDS = new Set(ARDUINO_NANO_PINS.map((pin) => pin.id));

export function normalizeArduinoNanoPins(pins: Pin[] = []): Pin[] {
  const byId = new Map<string, Pin>();
  for (const pin of pins) {
    if (!ARDUINO_NANO_PIN_IDS.has(pin.id) || byId.has(pin.id)) continue;
    byId.set(pin.id, pin);
  }
  return ARDUINO_NANO_PINS.map((canonical) => {
    const existing = byId.get(canonical.id);
    return {
      ...canonical,
      net_id: existing?.net_id ?? canonical.net_id,
      offset: { ...canonical.offset },
    };
  });
}

export function arduinoUnoParams(params: CircuitComponent["params"]): CircuitComponent["params"] {
  return {
    vcc: 5,
    hex: "",
    hexName: "",
    notes: "",
    usb_power: 1,
    ...params,
  };
}

/** Shared param defaults for Uno and Nano (identical core/MCU). */
export const arduinoBoardParams = arduinoUnoParams;

export function arduinoUnoAabb(at: Position): { x: number; y: number; w: number; h: number } {
  return {
    x: at.x + ARDUINO_UNO_BOUNDS.x,
    y: at.y + ARDUINO_UNO_BOUNDS.y,
    w: ARDUINO_UNO_BOUNDS.w,
    h: ARDUINO_UNO_BOUNDS.h,
  };
}

export function arduinoUnoRotatedAabb(
  comp: Pick<CircuitComponent, "position" | "rotation">,
): { x: number; y: number; w: number; h: number } {
  const corners = [
    { x: ARDUINO_UNO_BOUNDS.x, y: ARDUINO_UNO_BOUNDS.y },
    { x: ARDUINO_UNO_BOUNDS.x + ARDUINO_UNO_BOUNDS.w, y: ARDUINO_UNO_BOUNDS.y },
    { x: ARDUINO_UNO_BOUNDS.x, y: ARDUINO_UNO_BOUNDS.y + ARDUINO_UNO_BOUNDS.h },
    { x: ARDUINO_UNO_BOUNDS.x + ARDUINO_UNO_BOUNDS.w, y: ARDUINO_UNO_BOUNDS.y + ARDUINO_UNO_BOUNDS.h },
  ].map((corner) => rotateLocal(corner, comp.position, comp.rotation));
  const xs = corners.map((p) => p.x);
  const ys = corners.map((p) => p.y);
  const minX = Math.min(...xs);
  const minY = Math.min(...ys);
  return {
    x: minX,
    y: minY,
    w: Math.max(...xs) - minX,
    h: Math.max(...ys) - minY,
  };
}

function rotateLocal(local: Position, origin: Position, rotation: Rotation): Position {
  const rad = (rotation * Math.PI) / 180;
  const cos = Math.cos(rad);
  const sin = Math.sin(rad);
  return {
    x: origin.x + local.x * cos - local.y * sin,
    y: origin.y + local.x * sin + local.y * cos,
  };
}
