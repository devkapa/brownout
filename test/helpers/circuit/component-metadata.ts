// Test-support copy extracted from devolt (B2 corpus migration). App-domain
// module that the engine corpus exercises but the published package does not
// ship; only the import specifiers were rewritten to brownout paths.
import type { ComponentKind, CircuitComponent } from "../../../src/circuit/types.js";

/*
 * Placement surfaces — the load-bearing product rule
 *
 * de:volt is a breadboard simulator. The workspace canvas is not a freeform
 * schematic surface. Exactly two placement surfaces exist, and every
 * ComponentKind belongs to exactly one of them (`placementSurfaceFor`):
 *
 *   "workspace"  — freestanding on the canvas: breadboards themselves plus
 *                  the WORKSPACE_ONLY_KINDS below (power supplies / bench
 *                  signal sources and the Arduino Uno). These are the ONLY
 *                  kinds that may ever exist off a breadboard.
 *   "breadboard" — everything else (LEDs, resistors, switches, ICs, …).
 *                  These kinds cannot be placed on the bare canvas: drops
 *                  outside a breadboard are rejected (`computeDropCandidate`
 *                  returns null and the drag cursor shows not-allowed), so a
 *                  breadboard-only part is never legitimately freestanding.
 *
 * When adding a part, do NOT design a freestanding render or placement path
 * for a breadboard-only kind — the only visual variant it needs is the
 * breadboard-mounted one. (The renderer keeps a generic off-board fallback
 * solely for pre-breadboard-era saves; never add per-kind freestanding art
 * on top of it.)
 */

export const WORKSPACE_ONLY_KINDS = [
  "battery_pack",
  "bench_psu",
  "pulse_gen",
  "clock_gen",
  "signal_gen",
  // Legacy aliases for old-save back-compat
  "voltage_source",
  "pulse_source",
  "clock",
  "arduino_uno",
  // Free-standing, wired-to parts (not breadboard-mounted). Motors and displays
  // are physically too large / multi-terminal to plug into a breadboard, and
  // buzzers/speakers read as discrete transducers you wire to. They render at
  // the component origin (like bench_psu) with terminal pins drawn off-body.
  "dc_motor",
  "servo",
  "stepper",
  "max7219",
  "hd44780",
  "buzzer",
  "speaker",
  // NOTE: raspberry_pi_pico moved to DIP_PIN_COUNTS (wide DIP-40, plugs into
  // the breadboard straddling the centre gap like the Arduino Nano).
  // BBC micro:bit — free-standing board; on-board display/buttons run in the
  // embedded MicroPython sim, the five edge pads wire to the breadboard.
  "microbit",
  // S18b — HC-SR04 ultrasonic distance sensor: wired by leads like the servo/
  // dc_motor, not breadboard-mounted (real modules ship with jumper wires).
  "hcsr04",
] as const satisfies readonly ComponentKind[];

export const PALETTE_HIDDEN_KINDS = [] as const satisfies readonly ComponentKind[];

export const LEADED_BREADBOARD_KINDS = [
  "resistor",
  "diode",
  "zener_diode",
  "schottky_diode",
  "tvs_diode",
  "led",
  "capacitor",
  "inductor",
  "fuse",
  "ptc_fuse",
  "ldr",
  "thermistor",
  // Ferrite bead is a 2-pin leaded axial part, same breadboard mounting as resistor.
  "ferrite_bead",
  // NOTE: buzzer, speaker, dc_motor, servo, stepper were reclassified to
  // WORKSPACE_ONLY_KINDS (free-standing wired-to parts), so they are no longer
  // leaded breadboard parts.
] as const satisfies readonly ComponentKind[];

export const SWITCH_LIKE_KINDS = [
  "switch",
  "push_button",
  "spdt_switch",
  "push_dpdt",
  "dip_switch",
] as const satisfies readonly ComponentKind[];

// Components that produce a periodic / programmed output ON THEIR OWN -- no human
// in the loop. Used by the Learn toggle_rate "autonomous" anti-gaming gate to
// distinguish a genuine oscillator/MCU blink from a hand-flicked switch.
// Kind-based (not topology-based): de:volt has no discrete RC/inverter astable
// parts, so every legitimate self-driver a learner can build is one of these
// IC / signal-source / microcontroller kinds. MCUs ARE included -- a firmware
// Blink (digitalWrite) is autonomous. buzzer is excluded: an active buzzer
// self-oscillates acoustically but does not toggle its electrical drive net.
export const AUTONOMOUS_DRIVER_KINDS = [
  "clock_gen",
  "pulse_gen",
  "signal_gen",
  "ne555",
  "cd4060",
  "arduino_uno",
  "arduino_nano",
  "raspberry_pi_pico",
] as const satisfies readonly ComponentKind[];

export const GAP_BRIDGE_KINDS = [
  "switch",
  "push_button",
  "spdt_switch",
  // 6-pin DPDT latching push switch: a DIP-6-style module straddling the centre
  // gap (3 pins per breadboard half). Custom pin semantics, so it has its own
  // breadboardPhysicalPins case rather than going through the generic DIP path.
  "push_dpdt",
  "seg7_cc",
  "seg7_ca",
  "dip_switch",
  // Resistor array straddles the centre gap of a breadboard in the same way as
  // a DIP IC — top row a1..a4 on one side, bottom row b1..b4 on the other.
  "resistor_array",
  // W6.2 — ULN2003 (DIP-16) and ULN2803 (DIP-18) straddle the breadboard gap
  // like all DIP ICs. GAP_BRIDGE is required in addition to DIP_PIN_COUNTS so
  // the placement surface check routes them through the DIP breadboard path.
  "uln2003",
  "uln2803",
  // W6.3 — L293D (DIP-16) and TB6612 (modelled as DIP-16 breadboard module)
  // straddle the breadboard gap identically to other DIP ICs.
  "l293d",
  "tb6612",
  // DC-DC buck module: sits ON the centre gap overlapping the holes, with its
  // four header pads on the holes (Nano/DIP-style), not a single-row leaded part.
  "dcdc_converter",
  // SRD relay cube: sits over the gap, body overlapping the holes, 5 header
  // pads on the holes (Nano/DIP-style), not a body-above-a-single-row part.
  "relay",
] as const satisfies readonly ComponentKind[];

export const DIP_PIN_COUNTS: Partial<Record<ComponentKind, number>> = {
  // Resistor array: fixed DIP-8 (4 isolated resistors, 2×4 pins).
  resistor_array: 8,
  ne555: 8,
  "74ls00": 14,
  "74ls04": 14,
  "74ls08": 14,
  "74ls32": 14,
  "74ls86": 14,
  "74ls161": 16,
  "74ls173": 16,
  "74ls189": 16,
  "74ls157": 16,
  "74ls245": 20,
  "74ls283": 16,
  "74hc595": 16,
  "74hc165": 16,
  "74hc14": 14,
  "74hc138": 16,
  "74ls47": 16,
  "74hc74": 14,
  "cd4017": 16,
  "cd4511": 16,
  "cd4060": 16,
  "28c16": 24,
  "28c256": 28,
  arduino_nano: 30,
  // Raspberry Pi Pico: 40-pin castellated board on a wide-DIP footprint —
  // plugs into the breadboard straddling the centre gap like the Nano.
  raspberry_pi_pico: 40,
  // W4.1 — analog op-amp ICs (all DIP-8)
  lm358: 8,
  mcp6002: 8,
  lm386: 8,
  // W4.2 — LM393 dual comparator is DIP-8; TL431 is a 3-pin leaded part (not DIP)
  lm393: 8,
  // W6.2 — Darlington sink arrays: ULN2003 is DIP-16, ULN2803 is DIP-18.
  // These are the standard packages for the ULN2003A and ULN2803A ICs.
  uln2003: 16,
  uln2803: 18,
  // W6.3 — L293D is DIP-16 (standard PDIP-16 package).
  // TB6612 SSOP-24 is modelled as a 16-pin breadboard module footprint
  // (the real IC has 24 pads but breadboard breakout boards expose a DIP-16
  // compatible subset — 2×8 pin headers — which is what we model).
  l293d: 16,
  tb6612: 16,
  // DC-DC buck module: 4 header pads in a 2×2 grid bridging the gap. The pin
  // count drives DIP classification + placement; the actual 2-column-spread
  // offsets come from breadboardDcdcConverterPins and the body width from the
  // breadboardDipSize special case (a packed DIP-4 would be too narrow).
  dcdc_converter: 4,
  // SRD relay cube: 5 header pads bridging the gap (3 contacts top, 2 coil
  // bottom). Custom offsets from breadboardRelayPins; width from the
  // breadboardDipSize special case.
  relay: 5,
};

const workspaceOnly = new Set<ComponentKind>(WORKSPACE_ONLY_KINDS);
const paletteHidden = new Set<ComponentKind>(PALETTE_HIDDEN_KINDS as unknown as ComponentKind[]);
const leadedBreadboard = new Set<ComponentKind>(LEADED_BREADBOARD_KINDS);
const switchLike = new Set<ComponentKind>(SWITCH_LIKE_KINDS);
const autonomousDriver = new Set<ComponentKind>(AUTONOMOUS_DRIVER_KINDS);
const gapBridge = new Set<ComponentKind>(GAP_BRIDGE_KINDS);

export type PlacementSurface = "workspace" | "breadboard";

/**
 * The single classifier for where a kind is allowed to exist. See the
 * placement-surfaces note at the top of this file: "workspace" kinds are the
 * only ones that may be freestanding on the canvas; everything else is
 * breadboard-only.
 */
export function placementSurfaceFor(kind: ComponentKind): PlacementSurface {
  return kind === "breadboard" || workspaceOnly.has(kind) ? "workspace" : "breadboard";
}

/** True for kinds that can only exist mounted on a breadboard. */
export function isBreadboardOnlyKind(kind: ComponentKind): boolean {
  return placementSurfaceFor(kind) === "breadboard";
}

export function isWorkspaceOnlyKind(kind: ComponentKind): boolean {
  return workspaceOnly.has(kind);
}

export function isPaletteHiddenKind(kind: ComponentKind): boolean {
  return paletteHidden.has(kind);
}

export function isLeadedBreadboardKind(kind: ComponentKind): boolean {
  return leadedBreadboard.has(kind);
}

export function isSwitchLikeKind(kind: ComponentKind): boolean {
  return switchLike.has(kind);
}

export function isAutonomousDriverKind(kind: ComponentKind): boolean {
  return autonomousDriver.has(kind);
}

export function isGapBridgeKind(kind: ComponentKind): boolean {
  return gapBridge.has(kind);
}

export function dipPinCount(kind: ComponentKind): number | null {
  return DIP_PIN_COUNTS[kind] ?? null;
}

export function isDipIcKind(kind: ComponentKind): boolean {
  return dipPinCount(kind) != null;
}

export function isPhysicalBreadboardKind(kind: ComponentKind): boolean {
  return isLeadedBreadboardKind(kind) ||
    isDipIcKind(kind) ||
    isGapBridgeKind(kind) ||
    kind === "bicolor_led" ||
    kind === "rgb_led" ||
    kind === "bjt_npn" ||
    kind === "bjt_pnp" ||
    kind === "nmos" ||
    kind === "pmos" ||
    kind === "potentiometer" ||
    kind === "trimmer" ||
    // W4.2 — TL431 is a 3-pin leaded TO-92 part (same breadboard surface as BJT)
    kind === "tl431" ||
    // W5.1 — linear_reg (TO-220 3-pin) and lm317 (TO-220 adjustable) are
    // 3-pin leaded parts that mount in a row of 3 breadboard holes, same as BJT.
    kind === "linear_reg" ||
    kind === "lm317";
  // NOTE: dcdc_converter and relay are now gap-bridging DIP-style modules (in
  // DIP_PIN_COUNTS + GAP_BRIDGE_KINDS), so isDipIcKind/isGapBridgeKind already
  // make them physical breadboard parts — no explicit clause needed here.
  // NOTE: max7219 and hd44780 were reclassified to WORKSPACE_ONLY_KINDS
  // (free-standing display modules), so they are no longer physical breadboard
  // parts. dip_switch is included via GAP_BRIDGE_KINDS above.
}

export function componentBoardId(comp: CircuitComponent): string | undefined {
  const board = comp.params.board;
  return typeof board === "string" ? board : undefined;
}

// ── DIP switch param-dependent size ──────────────────────────────────────────

// Inlined to avoid a circular dep: breadboard.ts imports this module, so we
// cannot import back from breadboard.ts.
const _DIP_TIE_PITCH = 19;         // px per 0.1" hole pitch
const _DIP_BODY_OVERHANG = 8;      // body margin beyond the first/last pin column
const _DIP_PIN_SPAN_Y = 55;        // main-top-row-4 to main-bot-row-0 (225 − 170)

/** Default positions used for catalog pin_layout and fallback sizes. */
export const DIP_SWITCH_DEFAULT_POSITIONS = 4;
export const DIP_SWITCH_MIN_POSITIONS = 2;
export const DIP_SWITCH_MAX_POSITIONS = 8;

/**
 * Width and height of a DIP switch body for N switch positions.
 *
 * Each position occupies one DIP column (one TIE_PITCH).  The body overhangs
 * the outermost pin column on each side by DIP_BODY_OVERHANG px.  Height is
 * the standard DIP pin-row span, same as a 2-pin SPST that straddles the gap.
 */
export function dipSwitchSize(positions: number): { w: number; h: number } {
  const n = Math.max(DIP_SWITCH_MIN_POSITIONS, Math.min(DIP_SWITCH_MAX_POSITIONS, Math.round(positions)));
  return {
    w: (n - 1) * _DIP_TIE_PITCH + _DIP_BODY_OVERHANG * 2,
    h: _DIP_PIN_SPAN_Y,
  };
}
