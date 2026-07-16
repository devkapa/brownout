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

// Footprint of the de:volt board art (public/parts/arduino-uno-board.svg). The
// PCB body maps to (0,0)-(WIDTH,HEIGHT); the USB-B connector + barrel jack
// overhang the left edge (negative x, covered by ARDUINO_UNO_BOUNDS.x).
export const ARDUINO_UNO_WIDTH = 263;
export const ARDUINO_UNO_HEIGHT = 204;

export const ARDUINO_UNO_BOUNDS = {
  x: -80,
  y: 0,
  w: ARDUINO_UNO_WIDTH + 80,
  h: ARDUINO_UNO_HEIGHT,
} as const;

// Clickable USB region: the USB-B socket on the left edge plus the sliding cable
// plug (drawn in canvas/parts/arduino-uno.ts). Pressing it toggles usb_power,
// which eases the plug in/out.
export const ARDUINO_USB_TOGGLE_BOUNDS = {
  x: -76,
  y: 32,
  w: 116,
  h: 49,
} as const;

/** Component-local hit region for the tactile reset button (matches renderer art). */
export const ARDUINO_RESET_BUTTON_BOUNDS = {
  x: 11.5,
  y: 1.5,
  w: 24.5,
  h: 24.5,
} as const;

// ── SVG shared-space mapping ──────────────────────────────────────────────
// The board art and the pin offsets below live in ONE coordinate space. A
// single origin + scale maps SVG units to component-local px so every wired
// terminal sits exactly on its drawn header hole. Component-local (0,0) = the
// PCB's top-left corner = SVG (142.85, 36.4); scale keeps the header pitch at
// ~10 px, matching the prior model. canvas/parts/arduino-uno.ts imports these
// to place the board image, LEDs, reset dome and USB plug.
export const ARDUINO_UNO_SVG_ORIGIN_X = 142.85;
export const ARDUINO_UNO_SVG_ORIGIN_Y = 36.4;
export const ARDUINO_UNO_SVG_SCALE = 0.3256;

/** Map an SVG-space X to component-local px. */
export function arduinoUnoSvgX(x: number): number {
  return (x - ARDUINO_UNO_SVG_ORIGIN_X) * ARDUINO_UNO_SVG_SCALE;
}
/** Map an SVG-space Y to component-local px. */
export function arduinoUnoSvgY(y: number): number {
  return (y - ARDUINO_UNO_SVG_ORIGIN_Y) * ARDUINO_UNO_SVG_SCALE;
}

// Verified header-hole centres in SVG space (derived from a silkscreen overlay).
// Each row shares a y; x is per pin.
const AU_TOP_HOLE_Y = 55.21; // digital header (top edge)
const AU_BOTTOM_HOLE_Y = 637.87; // power + analog headers (bottom edge)
function round2(n: number): number {
  return Math.round(n * 100) / 100;
}
const auTop = (x: number): Position => ({
  x: round2(arduinoUnoSvgX(x)),
  y: round2(arduinoUnoSvgY(AU_TOP_HOLE_Y)),
});
const auBot = (x: number): Position => ({
  x: round2(arduinoUnoSvgX(x)),
  y: round2(arduinoUnoSvgY(AU_BOTTOM_HOLE_Y)),
});

export const ARDUINO_UNO_PINS: Pin[] = [
  // Power header (bottom-left). The leftmost SVG hole is an unlabeled NC pad.
  { id: "reset", label: "RST", offset: auBot(529.66) },
  { id: "io_ref", label: "IOREF", offset: auBot(498.89) },
  { id: "3v3", label: "3V3", offset: auBot(560.43) },
  { id: "5v", label: "5V", offset: auBot(591.2) },
  { id: "gnd", label: "GND", offset: auBot(621.97) },
  { id: "gnd2", label: "GND", offset: auBot(652.74) },
  { id: "vin", label: "VIN", offset: auBot(683.51) },

  // Analog header (bottom-right).
  { id: "a0", label: "A0", offset: auBot(744) },
  { id: "a1", label: "A1", offset: auBot(774.87) },
  { id: "a2", label: "A2", offset: auBot(805.74) },
  { id: "a3", label: "A3", offset: auBot(836.61) },
  { id: "a4", label: "A4", offset: auBot(867.48) },
  { id: "a5", label: "A5", offset: auBot(898.36) },

  // Digital header (top). The SVG's SCL, SDA and top-GND holes are decorative
  // (no engine node) — the sim exposes AREF + D0..D13, as the prior model did.
  { id: "aref", label: "AREF", offset: auTop(418.1) },
  { id: "d13", label: "13", offset: auTop(479.52) },
  { id: "d12", label: "12", offset: auTop(510.24) },
  { id: "d11", label: "11", offset: auTop(540.95) },
  { id: "d10", label: "10", offset: auTop(571.66) },
  { id: "d9", label: "9", offset: auTop(602.37) },
  { id: "d8", label: "8", offset: auTop(633.09) },
  { id: "d7", label: "7", offset: auTop(681.79) },
  { id: "d6", label: "6", offset: auTop(712.5) },
  { id: "d5", label: "5", offset: auTop(743.22) },
  { id: "d4", label: "4", offset: auTop(773.93) },
  { id: "d3", label: "3", offset: auTop(804.64) },
  { id: "d2", label: "2", offset: auTop(835.36) },
  { id: "d1", label: "1", offset: auTop(866.07) },
  { id: "d0", label: "0", offset: auTop(896.79) },
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
