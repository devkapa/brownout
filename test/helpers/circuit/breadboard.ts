// Test-support copy extracted from devolt (B2 corpus migration). App-domain
// module that the engine corpus exercises but the published package does not
// ship; only the import specifiers were rewritten to brownout paths.
/**
 * Breadboard geometry model.
 *
 * Coordinate system: world-space pixels, origin at the breadboard component's
 * top-left corner. The numbers intentionally mirror the legacy pygame board
 * config for the 830-point board so parts line up with the real texture:
 * 19 px per 0.1", main grid at (21,94), power rails at (65,19).
 *
 * Layout (top → bottom):
 *   top rail row 0 (Vcc)
 *   top rail row 1 (GND)
 *   main top rows 0–4 (a–e)
 *   CENTRE_GAP
 *   main bottom rows 0–4 (f–j)
 *   bottom rail row 0 (Vcc)
 *   bottom rail row 1 (GND)
 */
import type { Circuit, CircuitComponent, ComponentKind, Pin, Position } from "../../../src/circuit/types.js";
import {
  dipPinCount,
  isDipIcKind,
  isLeadedBreadboardKind,
  isWorkspaceOnlyKind,
  dipSwitchSize,
  DIP_SWITCH_DEFAULT_POSITIONS,
  DIP_SWITCH_MIN_POSITIONS,
  DIP_SWITCH_MAX_POSITIONS,
} from "./component-metadata.js";
import { normalizeArduinoNanoPins, normalizeArduinoUnoPins } from "../../../src/circuit/arduino.js";
export { dipPinCount, isDipIcKind, dipSwitchSize, DIP_SWITCH_DEFAULT_POSITIONS, DIP_SWITCH_MIN_POSITIONS, DIP_SWITCH_MAX_POSITIONS } from "./component-metadata.js";

export const TIE_PITCH = 19;  // px between tie-point centres (= 0.1" real)
export const COLS = 63;
export const RAIL_COLS = 50;  // ten repeated 5-hole groups
export const ROWS = 5;        // main rows per side
export const RAIL_ROWS = 2;   // power-rail rows per side (Vcc + GND)

// Legacy texture dimensions for bb-830.png.
export const BOARD_WIDTH = 1219;
export const BOARD_HEIGHT = 394;
export const BOARD_400_WIDTH = 650;

// Layout constants — exported so the renderer can draw holes at exact positions.
export const MARGIN_X = 21;
export const MARGIN_Y = 19;
export const RAIL_X0 = 65;
export const RAIL_REPEAT_GAP = 113;
export const RAIL_GROUP_COLS = 5;
export const RAIL_TOP_Y0 = 28;
export const RAIL_BOT_Y0 = 347;
export const MAIN_TOP_Y0 = 94;
export const MAIN_BOT_Y0 = 225;
export const CENTRE_GAP_TOP = MAIN_TOP_Y0 + (ROWS - 1) * TIE_PITCH + TIE_PITCH / 2;
export const CENTRE_GAP_BOT = MAIN_BOT_Y0 - TIE_PITCH / 2;
export const DIP_PIN_SPAN_Y = MAIN_BOT_Y0 - (MAIN_TOP_Y0 + (ROWS - 1) * TIE_PITCH);
export const SEG7_PIN_SPAN_Y = DIP_PIN_SPAN_Y + TIE_PITCH * 4;
/**
 * Wide-DIP pin-row span (0.6" between rows instead of the canonical 0.3").
 * Used by the Arduino Nano, which has its long-edge pin rows 0.6" apart so the
 * body covers most of the breadboard's width. Pins land in main row 2 of each
 * side instead of the canonical-DIP "row 4 of top + row 0 of bottom" placement.
 */
export const WIDE_DIP_PIN_SPAN_Y = DIP_PIN_SPAN_Y + TIE_PITCH * 4;
export const DIP_BODY_OVERHANG = 8;

function dipPinSpanY(kind: ComponentKind): number {
  // MCU boards (Nano, Pico) have their long-edge pin rows 0.6" apart so the
  // body covers most of the breadboard's width.
  return kind === "arduino_nano" || kind === "raspberry_pi_pico"
    ? WIDE_DIP_PIN_SPAN_Y
    : DIP_PIN_SPAN_Y;
}

const BOARD_EDGE_PAD = 12;

// ── Rail-less boards ──────────────────────────────────────────────────────────
// A rail-less board is the terminal-strip core of a full board with the two
// power-rail banks removed, so several can be stacked vertically and share the
// power rails of a neighbouring full board (legacy "RL Breadboard"). Selected by
// `params.rails === 0`; every other board defaults to `rails === 1`, which keeps
// all existing saves — those omit the key or store `rails: 1` — on the full board.
//
// With the rails gone the main grid slides UP so the first row sits near the top
// edge. The gap BETWEEN the two main banks (and therefore DIP_PIN_SPAN_Y) is
// unchanged, so DIP ICs still straddle the centre channel exactly as on a full
// board — only the absolute Y of every hole shifts by RAILLESS_Y_SHIFT.
export const RAILLESS_MAIN_TOP_Y0 = 28;
// Pixels the grid rises when the rails are removed. Exported so a rails toggle
// can shift the board's Y by the same amount and keep parts on their holes.
export const RAILLESS_Y_SHIFT = MAIN_TOP_Y0 - RAILLESS_MAIN_TOP_Y0; // 66
// Body height keeps the two hole banks vertically centred: a RAILLESS_MAIN_TOP_Y0
// margin above the first row and an equal margin below the last row. The holes
// span (MAIN_BOT_Y0 - MAIN_TOP_Y0) + (ROWS - 1) * TIE_PITCH from first to last
// row, independent of the shift.
export const RAILLESS_BOARD_HEIGHT =
  RAILLESS_MAIN_TOP_Y0 * 2 + (MAIN_BOT_Y0 - MAIN_TOP_Y0) + (ROWS - 1) * TIE_PITCH; // 263

/** False only for the rail-less variant (`params.rails === 0`). */
export function boardHasRails(board: CircuitComponent): boolean {
  return Number(board.params.rails ?? 1) !== 0;
}

/** Rendered body height — rail-less boards drop both power-rail banks. */
export function boardHeight(board: CircuitComponent): number {
  return boardHasRails(board) ? BOARD_HEIGHT : RAILLESS_BOARD_HEIGHT;
}

/** Pixels the main grid (and centre channel) shift UP when a board has no rails. */
export function mainYShift(board: CircuitComponent): number {
  return boardHasRails(board) ? 0 : RAILLESS_Y_SHIFT;
}

export type BoardSide = "top" | "bottom";

export interface TiePoint {
  col: number;      // 0 … COLS-1
  row: number;      // 0 … ROWS-1 for main; 0 … RAIL_ROWS-1 for rail
  side: BoardSide;
  isRail: boolean;
}

export interface BreadboardGeometry {
  /** Component ID of the breadboard in the circuit. */
  id: string;
  position: Position;
}

// ── Canonical ID helpers ──────────────────────────────────────────────────────

/** Unique string key for a main-grid tie-point. */
export function tiePointId(col: number, row: number, side: BoardSide): string {
  return `tp-${side}-r${row}-c${col}`;
}

/** Unique string key for a rail tie-point. */
export function railTiePointId(col: number, railIndex: number, side: BoardSide): string {
  return `rail-${side}-${railIndex}-c${col}`;
}

export function tiePointFromId(id: string): TiePoint | null {
  const main = /^tp-(top|bottom)-r(\d+)-c(\d+)$/.exec(id);
  if (main) {
    return {
      side: main[1] as BoardSide,
      row: Number(main[2]),
      col: Number(main[3]),
      isRail: false,
    };
  }
  const rail = /^rail-(top|bottom)-(\d+)-c(\d+)$/.exec(id);
  if (rail) {
    return {
      side: rail[1] as BoardSide,
      row: Number(rail[2]),
      col: Number(rail[3]),
      isRail: true,
    };
  }
  return null;
}

export function tiePointIdFor(tp: TiePoint): string {
  return tp.isRail ? railTiePointId(tp.col, tp.row, tp.side) : tiePointId(tp.col, tp.row, tp.side);
}

/**
 * Net ID shared by all tie-points in one rail row.
 * railIndex 0 = Vcc (+), railIndex 1 = GND (−).
 */
export function railNetId(side: BoardSide, railIndex: number): string {
  return `rail-${side}-${railIndex}`;
}

/**
 * Net ID shared by all 5 tie-points in one column strip (one side).
 * Column strips on the same side that are not bridged by a wire or IC are
 * independent nets.
 */
export function stripNetId(col: number, side: BoardSide): string {
  return `strip-${side}-c${col}`;
}

export function railXForCol(col: number): number {
  const rep = Math.floor(col / RAIL_GROUP_COLS);
  const inRep = col % RAIL_GROUP_COLS;
  return RAIL_X0 + rep * RAIL_REPEAT_GAP + inRep * TIE_PITCH;
}

export function boardMainCols(board: CircuitComponent): number {
  return Number(board.params.points ?? 830) === 400 ? 30 : COLS;
}

export function boardRailCols(board: CircuitComponent): number {
  // Rail-less boards have no power-rail holes. Returning 0 makes every rail
  // tie-point fall off the board everywhere railCols is consulted (hit-testing,
  // placement, AI snapshot), so rails simply cannot exist on these boards.
  if (!boardHasRails(board)) return 0;
  return Number(board.params.points ?? 830) === 400 ? 25 : RAIL_COLS;
}

export function boardWidth(board: CircuitComponent): number {
  return Number(board.params.points ?? 830) === 400 ? BOARD_400_WIDTH : BOARD_WIDTH;
}

/**
 * Per-board left margin of the main hole grid.
 * 830-point boards keep the legacy `MARGIN_X = 21`. The 400-point board is
 * narrower (650 px) but uses the same `mainCols=30` grid pitch — so the grid
 * needs to be centered horizontally instead of left-aligned at MARGIN_X.
 * Choose the value that gives a visually-symmetric padding around the grid.
 */
export function boardMainStartX(board: CircuitComponent): number {
  if (Number(board.params.points ?? 830) === 400) {
    const mainCols = boardMainCols(board);
    const w = boardWidth(board);
    return Math.round((w - (mainCols - 1) * TIE_PITCH) / 2);
  }
  return MARGIN_X;
}

export function mainYFor(side: BoardSide, row: number, board?: CircuitComponent): number {
  const shift = board ? mainYShift(board) : 0;
  return (side === "top" ? MAIN_TOP_Y0 : MAIN_BOT_Y0) - shift + row * TIE_PITCH;
}

// ── World-space coordinate helpers ───────────────────────────────────────────

/** World position of a main-grid tie-point centre. */
export function tiePointToWorld(tp: TiePoint, boardPos: Position, board?: CircuitComponent): Position {
  const mainStart = board ? boardMainStartX(board) : MARGIN_X;
  const cx = boardPos.x + (tp.isRail ? railXForCol(tp.col) : mainStart + tp.col * TIE_PITCH);
  let cy: number;
  if (tp.isRail) {
    cy = tp.side === "top"
      ? boardPos.y + RAIL_TOP_Y0 + tp.row * TIE_PITCH
      : boardPos.y + RAIL_BOT_Y0 + tp.row * TIE_PITCH;
  } else {
    cy = boardPos.y + mainYFor(tp.side, tp.row, board);
  }
  return { x: cx, y: cy };
}

/**
 * Snap a world coordinate to the nearest tie-point.
 * Returns null if the point is outside the board bounds.
 */
export function worldToTiePoint(
  wx: number,
  wy: number,
  boardPos: Position,
  board?: CircuitComponent,
): TiePoint | null {
  const lx = wx - boardPos.x;
  const ly = wy - boardPos.y;

  const w = board ? boardWidth(board) : BOARD_WIDTH;
  const railCols = board ? boardRailCols(board) : RAIL_COLS;
  const mainCols = board ? boardMainCols(board) : COLS;
  const mainStart = board ? boardMainStartX(board) : MARGIN_X;
  const hasRails = board ? boardHasRails(board) : true;
  const yShift = board ? mainYShift(board) : 0;
  const mainTop = MAIN_TOP_Y0 - yShift;
  const mainBot = MAIN_BOT_Y0 - yShift;
  const h = board ? boardHeight(board) : BOARD_HEIGHT;

  if (lx < -BOARD_EDGE_PAD || lx > w + BOARD_EDGE_PAD) return null;
  if (ly < -BOARD_EDGE_PAD || ly > h + BOARD_EDGE_PAD) return null;

  // Rail bands only exist on full boards. On a rail-less board the top rail's old
  // Y range overlaps the (shifted-up) main rows, so it must be skipped entirely —
  // otherwise a click on a top row would resolve to a non-existent rail and miss.
  if (hasRails && ly >= RAIL_TOP_Y0 - TIE_PITCH / 2 && ly <= RAIL_TOP_Y0 + (RAIL_ROWS - 1) * TIE_PITCH + TIE_PITCH / 2) {
    const col = nearestRailCol(lx, railCols);
    if (col == null) return null;
    const row = Math.round((ly - RAIL_TOP_Y0) / TIE_PITCH);
    if (row < 0 || row >= RAIL_ROWS) return null;
    return { col, row, side: "top", isRail: true };
  }
  const mainCol = Math.round((lx - mainStart) / TIE_PITCH);
  if (mainCol < 0 || mainCol >= mainCols) return null;

  if (ly >= mainTop - TIE_PITCH / 2 && ly <= mainTop + (ROWS - 1) * TIE_PITCH + TIE_PITCH / 2) {
    const row = Math.round((ly - mainTop) / TIE_PITCH);
    if (row < 0 || row >= ROWS) return null;
    return { col: mainCol, row, side: "top", isRail: false };
  }
  if (ly >= mainBot - TIE_PITCH / 2 && ly <= mainBot + (ROWS - 1) * TIE_PITCH + TIE_PITCH / 2) {
    const row = Math.round((ly - mainBot) / TIE_PITCH);
    if (row < 0 || row >= ROWS) return null;
    return { col: mainCol, row, side: "bottom", isRail: false };
  }
  if (hasRails && ly >= RAIL_BOT_Y0 - TIE_PITCH / 2 && ly <= RAIL_BOT_Y0 + (RAIL_ROWS - 1) * TIE_PITCH + TIE_PITCH / 2) {
    const col = nearestRailCol(lx, railCols);
    if (col == null) return null;
    const row = Math.round((ly - RAIL_BOT_Y0) / TIE_PITCH);
    if (row < 0 || row >= RAIL_ROWS) return null;
    return { col, row, side: "bottom", isRail: true };
  }

  return null;
}

function nearestRailCol(lx: number, railCols: number = RAIL_COLS): number | null {
  let bestCol = -1;
  let bestDx = Infinity;
  for (let col = 0; col < railCols; col++) {
    const dx = Math.abs(lx - railXForCol(col));
    if (dx < bestDx) {
      bestCol = col;
      bestDx = dx;
    }
  }
  return bestDx <= TIE_PITCH / 2 ? bestCol : null;
}

// ── Net aliasing rules ────────────────────────────────────────────────────────

/**
 * Returns all rail tie-point IDs grouped by their shared net.
 * Used by rebuildTiePointNets to initialise the rail assignments.
 */
export function railNetGroups(): Map<string, string[]> {
  const groups = new Map<string, string[]>();
  for (const side of ["top", "bottom"] as BoardSide[]) {
    for (let railIndex = 0; railIndex < RAIL_ROWS; railIndex++) {
      const netId = railNetId(side, railIndex);
      const ids: string[] = [];
      for (let col = 0; col < RAIL_COLS; col++) {
        ids.push(railTiePointId(col, railIndex, side));
      }
      groups.set(netId, ids);
    }
  }
  return groups;
}

export function tiePointNetId(tp: TiePoint): string {
  return tp.isRail ? railNetId(tp.side, tp.row) : stripNetId(tp.col, tp.side);
}

export function tiePointIdToNetId(id: string): string | null {
  const tp = tiePointFromId(id);
  return tp ? tiePointNetId(tp) : null;
}

export function breadboardDipSize(kind: ComponentKind): { w: number; h: number } | null {
  if (kind === "dcdc_converter") {
    // 2×2 corner module: width spans the IN→OUT column gap plus overhang each
    // side (a packed DIP-4 would be only ~35 px — too narrow for the module art).
    return { w: DIP_BODY_OVERHANG * 2 + DCDC_COL_SPAN * TIE_PITCH, h: dipPinSpanY(kind) };
  }
  if (kind === "relay") {
    // Square-ish SRD cube spanning the 3-wide pin footprint.
    return { w: DIP_BODY_OVERHANG * 2 + RELAY_COL_SPAN * TIE_PITCH, h: dipPinSpanY(kind) };
  }
  const pins = dipPinCount(kind);
  if (!pins) return null;
  return { w: (pins / 2 - 1) * TIE_PITCH + DIP_BODY_OVERHANG * 2, h: dipPinSpanY(kind) };
}

export function breadboardDipPins(kind: ComponentKind, pins: Pin[]): Pin[] {
  const pinCount = dipPinCount(kind);
  if (!pinCount || pins.length !== pinCount) return pins.map((p) => ({ ...p, offset: { ...p.offset } }));

  const perSide = pinCount / 2;
  const spanY = dipPinSpanY(kind);
  return pins.map((pin, index) => {
    const onBottom = index < perSide;
    const x = DIP_BODY_OVERHANG + (onBottom ? index * TIE_PITCH : (pinCount - 1 - index) * TIE_PITCH);
    return {
      ...pin,
      offset: {
        x,
        y: onBottom ? spanY : 0,
      },
    };
  });
}

/**
 * Synthesize 2×N pin offsets for a DIP switch with `positions` positions.
 *
 * Top row: a1..aN at (0..N-1) * TIE_PITCH, y=0
 * Bottom row: b1..bN at (0..N-1) * TIE_PITCH, y=DIP_PIN_SPAN_Y
 *
 * Existing pins are used as templates for function/label when available;
 * missing pins are synthesised from scratch.  This is the canonical synthesis
 * used at both placement time and when positions changes in the Inspector.
 */
export function breadboardDipSwitchPins(positions: number, existingPins: Pin[] = []): Pin[] {
  const n = Math.max(DIP_SWITCH_MIN_POSITIONS, Math.min(DIP_SWITCH_MAX_POSITIONS, Math.round(positions)));
  const byId = new Map(existingPins.map((pin) => [pin.id, pin]));
  const result: Pin[] = [];
  for (let i = 1; i <= n; i++) {
    const aId = `a${i}`;
    const bId = `b${i}`;
    const base = byId.get(aId);
    result.push({ ...(base ?? { id: aId, label: aId, function: "passive" as const }), id: aId, offset: { x: (i - 1) * TIE_PITCH, y: 0 } });
    const bBase = byId.get(bId);
    result.push({ ...(bBase ?? { id: bId, label: bId, function: "passive" as const }), id: bId, offset: { x: (i - 1) * TIE_PITCH, y: DIP_PIN_SPAN_Y } });
  }
  return result;
}

/**
 * Synthesize fixed 2×4 pin offsets for a resistor array (DIP-8 body).
 *
 * Top row: a1..a4 at DIP_BODY_OVERHANG + (0..3) * TIE_PITCH, y=0
 * Bottom row: b1..b4 at DIP_BODY_OVERHANG + (0..3) * TIE_PITCH, y=DIP_PIN_SPAN_Y
 *
 * Unlike a JEDEC DIP IC, a resistor array's pins are columnar pairs (a1/b1
 * share column 0, a2/b2 share column 1, …), so the generic breadboardDipPins
 * JEDEC ordering does not apply.  The x offsets DO follow the JEDEC DIP frame
 * (pins inset DIP_BODY_OVERHANG from the body edge): the part renders through
 * drawDipIcGlyph, whose body spans 0..breadboardDipSize().w, so a 0-based pin
 * field would sit 8 px left of the drawn package.
 */
export function breadboardResistorArrayPins(existingPins: Pin[] = []): Pin[] {
  const byId = new Map(existingPins.map((pin) => [pin.id, pin]));
  const result: Pin[] = [];
  for (let i = 1; i <= 4; i++) {
    const aId = `a${i}`;
    const bId = `b${i}`;
    const x = DIP_BODY_OVERHANG + (i - 1) * TIE_PITCH;
    const aBase = byId.get(aId);
    result.push({ ...(aBase ?? { id: aId, label: aId, function: "passive" as const }), id: aId, offset: { x, y: 0 } });
    const bBase = byId.get(bId);
    result.push({ ...(bBase ?? { id: bId, label: bId, function: "passive" as const }), id: bId, offset: { x, y: DIP_PIN_SPAN_Y } });
  }
  return result;
}

/** Tie-pitches between the DC-DC buck module's IN (left) and OUT (right) columns. */
export const DCDC_COL_SPAN = 4;

/**
 * Synthesize the DC-DC buck module's 2×2 corner pin layout, bridging the gap.
 *
 * Left column (IN): in_pos top (y=0), in_neg bottom (y=DIP_PIN_SPAN_Y).
 * Right column (OUT): out_pos top, out_neg bottom, DCDC_COL_SPAN tie-pitches
 * to the right. Offsets follow the JEDEC DIP frame (inset DIP_BODY_OVERHANG from
 * the body edge) so the module renders through the same body box as
 * breadboardDipSize("dcdc_converter"), with the four header pads sitting on the
 * breadboard holes under the overlapping PCB (Nano/DIP-style placement).
 */
export function breadboardDcdcConverterPins(existingPins: Pin[] = []): Pin[] {
  const byId = new Map(existingPins.map((pin) => [pin.id, pin]));
  const leftX = DIP_BODY_OVERHANG;
  const rightX = DIP_BODY_OVERHANG + DCDC_COL_SPAN * TIE_PITCH;
  const spec: Array<[string, number, number, string]> = [
    ["in_pos", leftX, 0, "IN+"],
    ["in_neg", leftX, DIP_PIN_SPAN_Y, "IN−"],
    ["out_pos", rightX, 0, "OUT+"],
    ["out_neg", rightX, DIP_PIN_SPAN_Y, "OUT−"],
  ];
  return spec.map(([id, x, y, label], i) => {
    const base = byId.get(id) ?? existingPins[i];
    return { ...(base ?? { id, label, function: "passive" as const }), id, offset: { x, y } };
  });
}

/** Tie-pitches spanned by the relay cube's pin columns (3-wide footprint). */
export const RELAY_COL_SPAN = 2;

/**
 * Synthesize the SRD relay cube's 5-pin layout, bridging the gap. The cube body
 * overlaps the holes with header pads on the pins (Nano/DIP-style placement).
 *
 * Top row (y=0): com (col 0), no (col 1), nc (col 2) — the SPDT contacts.
 * Bottom row (y=DIP_PIN_SPAN_Y): coil_a (col 0), coil_b (col 2) — the coil.
 */
export function breadboardRelayPins(existingPins: Pin[] = []): Pin[] {
  const byId = new Map(existingPins.map((pin) => [pin.id, pin]));
  const col = (n: number) => DIP_BODY_OVERHANG + n * TIE_PITCH;
  const spec: Array<[string, number, number, string]> = [
    ["com", col(0), 0, "COM"],
    ["no", col(1), 0, "NO"],
    ["nc", col(2), 0, "NC"],
    ["coil_a", col(0), DIP_PIN_SPAN_Y, "C1"],
    ["coil_b", col(2), DIP_PIN_SPAN_Y, "C2"],
  ];
  return spec.map(([id, x, y, label], i) => {
    const base = byId.get(id) ?? existingPins[i];
    return { ...(base ?? { id, label, function: "passive" as const }), id, offset: { x, y } };
  });
}

export function breadboardComponentPins(kind: ComponentKind, pins: Pin[] = []): Pin[] {
  if (kind === "arduino_uno") return normalizeArduinoUnoPins(pins);
  if (kind === "arduino_nano") return breadboardDipPins(kind, normalizeArduinoNanoPins(pins));
  if (kind === "dip_switch") {
    // Pin count encodes positions (2N pins); fall back to the default 4 positions.
    const n = pins.length >= 4 ? pins.length / 2 : DIP_SWITCH_DEFAULT_POSITIONS;
    return breadboardDipSwitchPins(n, pins);
  }
  if (kind === "resistor_array") {
    // Fixed 2×4 columnar layout — not JEDEC CCW, so bypass the generic DIP path.
    return breadboardResistorArrayPins(pins);
  }
  if (kind === "dcdc_converter") {
    // 2×2 corner layout (IN column left, OUT column right) bridging the gap —
    // not JEDEC packed, so bypass the generic DIP path.
    return breadboardDcdcConverterPins(pins);
  }
  if (kind === "relay") {
    // 5-pin cube layout: COM/NO/NC on the top row, coil pins on the bottom row.
    return breadboardRelayPins(pins);
  }
  if (isDipIcKind(kind)) return breadboardDipPins(kind, pins);
  if (kind === "microbit") {
    // The micro:bit's edge pads (five bare, or eleven docked in breakout mode —
    // see circuit/microbit-breakout.ts) are NOT on a 0.1" (TIE_PITCH) grid —
    // their positions are fixed by the SVG board art / dock painter. Preserve
    // the exact offsets already on the passed-in pins so each wire node stays
    // centred on its drawn hole; the generic tie-snap below would pull them off.
    return pins.map((p) => ({ ...p, offset: { ...p.offset } }));
  }
  const workspace = workspaceOnlyPins(kind, pins);
  if (workspace) return workspace;
  const physical = breadboardPhysicalPins(kind, pins);
  if (physical) return physical;
  if (kind === "breadboard" || isLeadedBreadboardKind(kind) || pins.length === 0) {
    return pins.map((p) => ({ ...p, offset: { ...p.offset } }));
  }

  const anchor = pins[0].offset;
  return pins.map((pin, index) => {
    if (index === 0) return { ...pin, offset: { ...pin.offset } };
    const dx = Math.round((pin.offset.x - anchor.x) / TIE_PITCH) * TIE_PITCH;
    const dy = Math.round((pin.offset.y - anchor.y) / TIE_PITCH) * TIE_PITCH;
    return {
      ...pin,
      offset: {
        x: anchor.x + dx,
        y: anchor.y + dy,
      },
    };
  });
}

function workspaceOnlyPins(kind: ComponentKind, pins: Pin[]): Pin[] | null {
  const byId = new Map(pins.map((pin) => [pin.id, pin]));
  const make = (ids: string[], offsets: Position[], labels: string[]) => ids.map((id, index) => {
    const pin = byId.get(id) ?? pins[index] ?? { id, offset: { x: 0, y: 0 } };
    return { ...pin, id, label: labels[index], offset: { ...offsets[index] } };
  });

  switch (kind) {
    case "battery_pack":
      return make(["pos", "neg"], [{ x: 100, y: 18 }, { x: 100, y: 32 }], ["+", "−"]);
    case "bench_psu":
      return make(["pos", "neg"], [{ x: 130, y: 55 }, { x: 130, y: 75 }], ["+", "−"]);
    // Reclassified free-standing parts. Terminal offsets land on the body edge
    // where the painter draws solder tabs / a pin header (see the matching
    // painter under canvas/parts/ and componentSizeForCanvas for the footprint).
    case "dc_motor": // 72×46 can, terminals on the LEFT brush end
      return make(["m1", "m2"], [{ x: 0, y: 16 }, { x: 0, y: 30 }], ["M+", "M−"]);
    case "servo": // 120×96, 3-wire cable exits the RIGHT edge
      return make(
        ["sig", "vplus", "gnd"],
        [{ x: 120, y: 30 }, { x: 120, y: 48 }, { x: 120, y: 66 }],
        ["SIG", "V+", "GND"],
      );
    case "stepper": // 124×76, 4 coil wires exit the RIGHT edge
      return make(
        ["a1", "a2", "b1", "b2"],
        [{ x: 124, y: 16 }, { x: 124, y: 31 }, { x: 124, y: 46 }, { x: 124, y: 61 }],
        ["A1", "A2", "B1", "B2"],
      );
    case "max7219": // 132×170, 5-pin header centred on the BOTTOM edge (pitch 19)
      return make(
        ["din", "clk", "cs", "vcc", "gnd"],
        [{ x: 28, y: 170 }, { x: 47, y: 170 }, { x: 66, y: 170 }, { x: 85, y: 170 }, { x: 104, y: 170 }],
        ["DIN", "CLK", "CS", "VCC", "GND"],
      );
    case "hd44780": // 340×140, 16-pin header centred along the TOP edge (pitch 16)
      return make(
        ["vss", "vdd", "v0", "rs", "rw", "e", "d0", "d1", "d2", "d3", "d4", "d5", "d6", "d7", "a", "k"],
        [50, 66, 82, 98, 114, 130, 146, 162, 178, 194, 210, 226, 242, 258, 274, 290].map((x) => ({ x, y: 0 })),
        ["VSS", "VDD", "V0", "RS", "RW", "E", "D0", "D1", "D2", "D3", "D4", "D5", "D6", "D7", "A", "K"],
      );
    case "buzzer": // 48×46, two pins out the BOTTOM
      return make(["p1", "p2"], [{ x: 17, y: 46 }, { x: 33, y: 46 }], ["+", "−"]);
    case "speaker": // 56×64, two terminal tabs at the BOTTOM
      return make(["p1", "p2"], [{ x: 18, y: 64 }, { x: 38, y: 64 }], ["+", "−"]);
    case "hcsr04": // 100×64, 4-pin header (VCC/TRIG/ECHO/GND) centred on the BOTTOM edge (pitch 19)
      return make(
        ["vcc", "trig", "echo", "gnd"],
        [{ x: 22, y: 64 }, { x: 41, y: 64 }, { x: 60, y: 64 }, { x: 79, y: 64 }],
        ["VCC", "TRIG", "ECHO", "GND"],
      );
    default:
      return null;
  }
}

function breadboardPhysicalPins(kind: ComponentKind, pins: Pin[]): Pin[] | null {
  if (pins.length === 0) return null;
  const byId = new Map(pins.map((pin) => [pin.id, pin]));
  const make = (ids: string[], offsets: Position[]) => ids.map((id, index) => {
    const pin = byId.get(id) ?? pins[index] ?? { id, offset: { x: 0, y: 0 } };
    return { ...pin, id, offset: { ...offsets[index] } };
  });
  const p = TIE_PITCH;

  switch (kind) {
    case "switch":
      // 2-pin SPST: anchored across the centre channel so the body sits
      // squarely on the board. Renderer draws the 4-leg tact body visually.
      return make(["a", "b"], [{ x: 0, y: 0 }, { x: p * 2, y: DIP_PIN_SPAN_Y }]);
    case "push_button":
      // Real 4-pin tactile button. The two legs on each side of the body are
      // internally tied — a≡a2 on the left, b≡b2 on the right —
      // and the momentary contact bridges the two sides when pressed (see the
      // push_button stamp in sim-engine). The four holes sit at the corners of
      // the SAME bounding box the old 2-pin layout spanned (cols 0..2 across
      // the centre channel), so the painted body does not move; only two extra
      // corner leads appear. a,b stay first so positional readers (engine
      // current report, operating-point power) still see the two terminals.
      return make(
        ["a", "b", "a2", "b2"],
        [
          { x: 0, y: 0 },                  // a  — top-left,     terminal A
          { x: p * 2, y: 0 },              // b  — top-right,    terminal B
          { x: 0, y: DIP_PIN_SPAN_Y },     // a2 — bottom-left,  tied to a
          { x: p * 2, y: DIP_PIN_SPAN_Y }, // b2 — bottom-right, tied to b
        ],
      );
    case "spdt_switch":
      return make(["com", "b", "c"], [{ x: p, y: DIP_PIN_SPAN_Y }, { x: 0, y: 0 }, { x: p * 2, y: 0 }]);
    case "push_dpdt":
      // 6-pin DPDT latching push switch — a DIP-6 footprint straddling the
      // centre channel. Pole 1 is the top row, pole 2 the bottom row; the
      // centre column holds the two commons, the left/right columns the two
      // throw positions. position 0 latches commons↔left, position 1 ↔right.
      return make(
        ["l1", "c1", "r1", "l2", "c2", "r2"],
        [
          { x: 0,     y: 0 },              // l1 — top-left,     pole-1 left throw
          { x: p,     y: 0 },              // c1 — top-centre,   pole-1 common
          { x: p * 2, y: 0 },              // r1 — top-right,    pole-1 right throw
          { x: 0,     y: DIP_PIN_SPAN_Y }, // l2 — bottom-left,  pole-2 left throw
          { x: p,     y: DIP_PIN_SPAN_Y }, // c2 — bottom-centre,pole-2 common
          { x: p * 2, y: DIP_PIN_SPAN_Y }, // r2 — bottom-right, pole-2 right throw
        ],
      );
    case "bicolor_led":
      return make(["a1", "k", "a2"], [{ x: 0, y: 0 }, { x: p, y: 0 }, { x: p * 2, y: 0 }]);
    case "rgb_led":
      return make(["r_a", "g_a", "b_a", "com_k"], [{ x: 0, y: 0 }, { x: p, y: 0 }, { x: p * 2, y: 0 }, { x: p * 3, y: 0 }]);
    case "bjt_npn":
    case "bjt_pnp":
      return make(["c", "b", "e"], [{ x: 0, y: 0 }, { x: p, y: 0 }, { x: p * 2, y: 0 }]);
    // W4.2 — TL431: 3-pin TO-92, same 3-hole-in-a-row breadboard footprint as BJT.
    // Catalog pin ids: ref (left), anode (centre), cathode (right).
    case "tl431":
      return make(["ref", "anode", "cathode"], [{ x: 0, y: 0 }, { x: p, y: 0 }, { x: p * 2, y: 0 }]);
    // W5.1 — linear_reg (TO-220): 3-pin leaded, 3-hole-in-a-row.
    // Catalog pin ids: in (left), gnd (centre), out (right).
    case "linear_reg":
      return make(["in", "gnd", "out"], [{ x: 0, y: 0 }, { x: p, y: 0 }, { x: p * 2, y: 0 }]);
    // W5.1 — lm317 (TO-220 adjustable): 3-pin leaded, 3-hole-in-a-row.
    // Catalog pin ids: in (left), adj (centre), out (right).
    case "lm317":
      return make(["in", "adj", "out"], [{ x: 0, y: 0 }, { x: p, y: 0 }, { x: p * 2, y: 0 }]);
    // W5.2 — dcdc_converter: 4-pin in-line breadboard module.
    // Two input pins on the left (in_pos, in_neg) and two output pins on the right
    // (out_pos, out_neg), in a single 4-hole row.  This is the simplest footprint
    // for a module that has no canonical DIP body — 4 holes in a line is the lowest-
    // friction placement for a small power-module breakout board.
    case "dcdc_converter":
      return make(
        ["in_pos", "in_neg", "out_pos", "out_neg"],
        [{ x: 0, y: 0 }, { x: p, y: 0 }, { x: p * 2, y: 0 }, { x: p * 3, y: 0 }],
      );
    // W6.1 — relay: 5-pin leaded breadboard part.
    // Pins in a single row: coil_a, coil_b, com, no, nc.
    // This matches the physical layout of through-hole SPDT relay modules.
    case "relay":
      return make(
        ["coil_a", "coil_b", "com", "no", "nc"],
        [{ x: 0, y: 0 }, { x: p, y: 0 }, { x: p * 2, y: 0 }, { x: p * 3, y: 0 }, { x: p * 4, y: 0 }],
      );
    // W7.1 — dc_motor: 2-pin leaded breadboard part.
    // Pins m1 (+) and m2 (−) in a single row, one hole pitch apart.
    // Matches the through-hole motor connector footprint (screw-terminal or solder pads).
    case "dc_motor":
      return make(
        ["m1", "m2"],
        [{ x: 0, y: 0 }, { x: p, y: 0 }],
      );
    // W7.2 — servo: 3-pin leaded breadboard part.
    // Standard hobby servo 3-wire connector: sig (signal/control), vplus (+V), gnd.
    // Left-to-right in a single row, one hole pitch apart — same footprint as BJT/TL431.
    case "servo":
      return make(
        ["sig", "vplus", "gnd"],
        [{ x: 0, y: 0 }, { x: p, y: 0 }, { x: p * 2, y: 0 }],
      );
    // W7.2 — stepper: 4-pin leaded breadboard part.
    // Bipolar 4-wire stepper connector: a1, a2 (coil A), b1, b2 (coil B) in a single row.
    // 4 holes × one pitch — same footprint width as dcdc_converter.
    case "stepper":
      return make(
        ["a1", "a2", "b1", "b2"],
        [{ x: 0, y: 0 }, { x: p, y: 0 }, { x: p * 2, y: 0 }, { x: p * 3, y: 0 }],
      );
    // W8.1 — max7219: 5-pin breadboard module (din, clk, cs, vcc, gnd) in a single row.
    // Mirrors the dcdc_converter footprint extended by one pitch for the 5th pin.
    case "max7219":
      return make(
        ["din", "clk", "cs", "vcc", "gnd"],
        [{ x: 0, y: 0 }, { x: p, y: 0 }, { x: p * 2, y: 0 }, { x: p * 3, y: 0 }, { x: p * 4, y: 0 }],
      );
    // W8.2 — hd44780: 16-pin breadboard module in a single row.
    // 16 pins × 10 px pitch (tighter than standard 19 px to fit the wide LCD body).
    case "hd44780":
      return make(
        ["vss", "vdd", "v0", "rs", "rw", "e", "d0", "d1", "d2", "d3", "d4", "d5", "d6", "d7", "a", "k"],
        [
          { x: 0, y: 0 }, { x: p, y: 0 }, { x: p * 2, y: 0 }, { x: p * 3, y: 0 },
          { x: p * 4, y: 0 }, { x: p * 5, y: 0 }, { x: p * 6, y: 0 }, { x: p * 7, y: 0 },
          { x: p * 8, y: 0 }, { x: p * 9, y: 0 }, { x: p * 10, y: 0 }, { x: p * 11, y: 0 },
          { x: p * 12, y: 0 }, { x: p * 13, y: 0 }, { x: p * 14, y: 0 }, { x: p * 15, y: 0 },
        ],
      );
    case "nmos":
    case "pmos":
      return make(["d", "g", "s"], [{ x: 0, y: 0 }, { x: p, y: 0 }, { x: p * 2, y: 0 }]);
    case "potentiometer":
    case "trimmer":
      return make(["cw", "wiper", "ccw"], [{ x: 0, y: 0 }, { x: p, y: 0 }, { x: p * 2, y: 0 }]);
    case "seg7_cc":
    case "seg7_ca":
      return make(
        ["g", "f", "com", "a", "b", "e", "d", "com2", "c", "dp"],
        [
          { x: 0, y: 0 }, { x: p, y: 0 }, { x: p * 2, y: 0 }, { x: p * 3, y: 0 }, { x: p * 4, y: 0 },
          { x: 0, y: SEG7_PIN_SPAN_Y }, { x: p, y: SEG7_PIN_SPAN_Y }, { x: p * 2, y: SEG7_PIN_SPAN_Y }, { x: p * 3, y: SEG7_PIN_SPAN_Y }, { x: p * 4, y: SEG7_PIN_SPAN_Y },
        ],
      );
    case "dip_switch": {
      // Variable-width DIP switch: N positions, 2N pins (a1..aN on top row,
      // b1..bN on bottom row).  The top-row pins sit at y=0, bottom at y=DIP_PIN_SPAN_Y.
      // We read positions from the existing pin count rather than from params to
      // stay compatible with the static `make` helper used here.  Callers that
      // need params-driven synthesis use breadboardDipSwitchPins() directly.
      const n = Math.max(
        DIP_SWITCH_MIN_POSITIONS,
        Math.min(DIP_SWITCH_MAX_POSITIONS, pins.length / 2 || DIP_SWITCH_DEFAULT_POSITIONS),
      );
      return breadboardDipSwitchPins(n, pins);
    }
    default:
      return null;
  }
}

export function componentPinWorld(comp: CircuitComponent, pin: Pin): Position {
  const rad = (comp.rotation * Math.PI) / 180;
  const cos = Math.cos(rad);
  const sin = Math.sin(rad);
  return {
    x: comp.position.x + pin.offset.x * cos - pin.offset.y * sin,
    y: comp.position.y + pin.offset.x * sin + pin.offset.y * cos,
  };
}

export function componentTiePointIds(comp: CircuitComponent, board: CircuitComponent): string[] {
  const ids: string[] = [];
  for (const pin of comp.pins) {
    const tp = pinTiePointOnBoard(comp, pin, board);
    if (!tp) return [];
    ids.push(tiePointIdFor(tp));
  }
  return ids;
}

export function occupiedTiePointIds(
  circuit: Circuit,
  board: CircuitComponent,
  excluding = new Set<string>(),
): Set<string> {
  const occupied = new Set<string>();
  for (const comp of circuit.components) {
    if (comp.kind === "breadboard" || isWorkspaceOnlyKind(comp.kind) || excluding.has(comp.id)) continue;
    for (const pin of comp.pins) {
      const tp = pinTiePointOnBoard(comp, pin, board);
      if (tp) occupied.add(tiePointIdFor(tp));
    }
  }
  for (const wire of circuit.wires) {
    if (wire.from_component === board.id && isTiePointOnBoard(wire.from_pin, board)) {
      occupied.add(wire.from_pin);
    }
    if (wire.to_component === board.id && isTiePointOnBoard(wire.to_pin, board)) {
      occupied.add(wire.to_pin);
    }
  }
  return occupied;
}

export function canPlaceComponentOnBreadboard(
  circuit: Circuit,
  comp: CircuitComponent,
  board: CircuitComponent,
  excluding = new Set<string>(),
): boolean {
  if (comp.kind === "breadboard") return true;
  const ids = componentTiePointIds(comp, board);
  if (ids.length !== comp.pins.length) return false;
  if (new Set(ids).size !== ids.length) return false;
  const occupied = occupiedTiePointIds(circuit, board, excluding);
  return ids.every((id) => !occupied.has(id));
}

export function canPlaceComponentOnBreadboards(
  circuit: Circuit,
  comp: CircuitComponent,
  boards: CircuitComponent[],
  excluding = new Set<string>(),
): boolean {
  if (comp.kind === "breadboard") return true;
  const placements: Array<{ board: CircuitComponent; id: string }> = [];
  for (const pin of comp.pins) {
    const placement = pinBreadboardPlacement(comp, pin, boards);
    if (!placement) return false;
    placements.push(placement);
  }
  const seen = new Set<string>();
  const occupiedByBoard = new Map<string, Set<string>>();
  for (const placement of placements) {
    const key = `${placement.board.id}\x00${placement.id}`;
    if (seen.has(key)) return false;
    seen.add(key);

    let occupied = occupiedByBoard.get(placement.board.id);
    if (!occupied) {
      occupied = occupiedTiePointIds(circuit, placement.board, excluding);
      occupiedByBoard.set(placement.board.id, occupied);
    }
    if (occupied.has(placement.id)) return false;
  }
  return true;
}

export function pinTiePoint(comp: CircuitComponent, pin: Pin, board: CircuitComponent): TiePoint | null {
  const world = componentPinWorld(comp, pin);
  return worldToTiePoint(world.x, world.y, board.position, board);
}

function pinTiePointOnBoard(comp: CircuitComponent, pin: Pin, board: CircuitComponent): TiePoint | null {
  const tp = pinTiePoint(comp, pin, board);
  return tp && isTiePointOnBoard(tiePointIdFor(tp), board) ? tp : null;
}

function pinBreadboardPlacement(
  comp: CircuitComponent,
  pin: Pin,
  boards: CircuitComponent[],
): { board: CircuitComponent; id: string } | null {
  for (const board of boards) {
    const tp = pinTiePointOnBoard(comp, pin, board);
    if (tp) return { board, id: tiePointIdFor(tp) };
  }
  return null;
}

function isTiePointOnBoard(id: string, board: CircuitComponent): boolean {
  const tp = tiePointFromId(id);
  if (!tp) return false;
  return tp.isRail ? tp.col < boardRailCols(board) : tp.col < boardMainCols(board);
}

type PinRef = { component: string; pin: string };

class UnionFind {
  private parent = new Map<string, string>();

  find(x: string): string {
    const parent = this.parent.get(x) ?? x;
    if (parent === x) {
      this.parent.set(x, x);
      return x;
    }
    const root = this.find(parent);
    this.parent.set(x, root);
    return root;
  }

  union(a: string, b: string): void {
    const ra = this.find(a);
    const rb = this.find(b);
    if (ra !== rb) this.parent.set(rb, ra);
  }
}

function breadboardAliasWires(circuit: Circuit, boards: CircuitComponent[]): Circuit["wires"] {
  const boardById = new Map(boards.map((board) => [board.id, board]));
  const boardIds = new Set(boardById.keys());
  const uf = new UnionFind();

  const boardNetNode = (boardId: string, pin: string): string | null => {
    const netId = tiePointIdToNetId(pin);
    return netId ? `bb:${boardId}\x00${netId}` : null;
  };
  const pinNode = (component: string, pin: string): string => `pin:${component}\x00${pin}`;

  for (const wire of circuit.wires) {
    const from = boardIds.has(wire.from_component)
      ? boardNetNode(wire.from_component, wire.from_pin)
      : pinNode(wire.from_component, wire.from_pin);
    const to = boardIds.has(wire.to_component)
      ? boardNetNode(wire.to_component, wire.to_pin)
      : pinNode(wire.to_component, wire.to_pin);
    if (from && to) uf.union(from, to);
  }

  const pinNodes: Array<{ key: string; component: string; pin: string }> = [];
  for (const comp of circuit.components) {
    if (comp.kind === "breadboard" || isWorkspaceOnlyKind(comp.kind)) continue;
    for (const board of boards) {
      for (const pin of comp.pins) {
        const tp = pinTiePoint(comp, pin, board);
        if (!tp) continue;
        const key = pinNode(comp.id, pin.id);
        uf.union(key, `bb:${board.id}\x00${tiePointNetId(tp)}`);
        pinNodes.push({ key, component: comp.id, pin: pin.id });
      }
    }
  }
  for (const wire of circuit.wires) {
    if (!boardIds.has(wire.from_component) && !boardIds.has(wire.to_component)) continue;
    if (!boardIds.has(wire.from_component)) {
      pinNodes.push({
        key: pinNode(wire.from_component, wire.from_pin),
        component: wire.from_component,
        pin: wire.from_pin,
      });
    }
    if (!boardIds.has(wire.to_component)) {
      pinNodes.push({
        key: pinNode(wire.to_component, wire.to_pin),
        component: wire.to_component,
        pin: wire.to_pin,
      });
    }
  }

  const groups = new Map<string, Array<{ component: string; pin: string }>>();
  const seenPins = new Set<string>();
  for (const pin of pinNodes) {
    const stableKey = `${pin.component}\x00${pin.pin}`;
    if (seenPins.has(stableKey)) continue;
    seenPins.add(stableKey);
    const root = uf.find(pin.key);
    groups.set(root, [...(groups.get(root) ?? []), { component: pin.component, pin: pin.pin }]);
  }

  const wires: Circuit["wires"] = [];
  for (const [netId, pins] of groups) {
    if (pins.length < 2) continue;
    const [first, ...rest] = pins;
    for (const pin of rest) {
      wires.push({
        id: `bb-${netId.replace(/\W+/g, "-")}-${first.component}-${first.pin}-${pin.component}-${pin.pin}`,
        from_component: first.component,
        from_pin: first.pin,
        to_component: pin.component,
        to_pin: pin.pin,
        resistance: 0,
      });
    }
  }
  return wires;
}

/**
 * Build the breadboard connectivity union-find for a circuit and resolve each
 * group's live MNA net id. This mirrors the alias-wire generator so that net
 * resolution from a bare tie-point matches what the engine sees.
 *
 * Crucially it counts component pins reachable from a board through BOTH paths:
 * pins physically sitting on a hole (pinTiePoint) AND off-board pins wired into a
 * board (e.g. a battery pack jumpered to a power rail). Without the latter, a rail
 * fed only by an off-board supply has no resolvable net.
 */
function buildBoardConnectivity(
  circuit: Circuit,
  nets: ReadonlyArray<{ id: string; pins: ReadonlyArray<readonly [string, string]> }>,
  boards: CircuitComponent[],
): { uf: UnionFind; rootToNet: Map<string, string> } {
  const boardIds = new Set(boards.map((b) => b.id));
  const uf = new UnionFind();
  const boardNetNode = (boardId: string, pin: string): string | null => {
    const netId = tiePointIdToNetId(pin);
    return netId ? `bb:${boardId}\x00${netId}` : null;
  };
  const pinNode = (component: string, pin: string): string => `pin:${component}\x00${pin}`;

  const pinRefs: Array<{ key: string; component: string; pin: string }> = [];
  const addPinRef = (component: string, pin: string) =>
    pinRefs.push({ key: pinNode(component, pin), component, pin });

  for (const wire of circuit.wires) {
    const from = boardIds.has(wire.from_component)
      ? boardNetNode(wire.from_component, wire.from_pin)
      : pinNode(wire.from_component, wire.from_pin);
    const to = boardIds.has(wire.to_component)
      ? boardNetNode(wire.to_component, wire.to_pin)
      : pinNode(wire.to_component, wire.to_pin);
    if (from && to) uf.union(from, to);
    // Off-board endpoint(s) of a wire that touches a board must be resolvable to a
    // net (a supply wired straight into a rail, a part jumpered to a strip, ...).
    if (boardIds.has(wire.from_component) || boardIds.has(wire.to_component)) {
      if (!boardIds.has(wire.from_component)) addPinRef(wire.from_component, wire.from_pin);
      if (!boardIds.has(wire.to_component)) addPinRef(wire.to_component, wire.to_pin);
    }
  }

  for (const comp of circuit.components) {
    if (comp.kind === "breadboard" || isWorkspaceOnlyKind(comp.kind)) continue;
    for (const b of boards) {
      for (const pin of comp.pins) {
        const tp = pinTiePoint(comp, pin, b);
        if (!tp) continue;
        uf.union(pinNode(comp.id, pin.id), `bb:${b.id}\x00${tiePointNetId(tp)}`);
        addPinRef(comp.id, pin.id);
      }
    }
  }

  const rootToNet = new Map<string, string>();
  for (const ref of pinRefs) {
    const root = uf.find(ref.key);
    if (rootToNet.has(root)) continue;
    const net = nets.find((n) => n.pins.some(([c, p]) => c === ref.component && p === ref.pin));
    if (net) rootToNet.set(root, net.id);
  }
  return { uf, rootToNet };
}

/**
 * Resolve the live MNA net id (e.g. "n0", "gnd") for any breadboard tie-point —
 * including empty holes on a strip that no component pin physically occupies, and
 * rails fed only by an off-board supply. Makes a multimeter probe behave like a
 * real one: touching any hole on an electrically-common strip reads its node.
 */
export function netIdForTiePoint(
  circuit: Circuit,
  nets: ReadonlyArray<{ id: string; pins: ReadonlyArray<readonly [string, string]> }>,
  board: CircuitComponent,
  tp: TiePoint,
): string | null {
  const boards = circuit.components.filter((c) => c.kind === "breadboard");
  const { uf, rootToNet } = buildBoardConnectivity(circuit, nets, boards);
  return rootToNet.get(uf.find(`bb:${board.id}\x00${tiePointNetId(tp)}`)) ?? null;
}

/** Precomputed index for the node-highlight view, built once per topology. */
export interface BreadboardNodeIndex {
  /**
   * Per live net id: the contiguous hole runs that belong to it — each run is a
   * single strip column or rail segment, as a list of hole world positions. The
   * renderer draws one translucent rectangle per run, so a node reads as seamless
   * bands rather than scattered dots. Includes empty holes on the run.
   */
  groupsByNet: Map<string, Position[][]>;
  /**
   * Resolve a wire endpoint that lands on a breadboard hole to its live net id.
   * Key is `${boardId}\x00${tiePointId}`. Lets the renderer highlight wires whose
   * endpoints are breadboard holes (jumpers, supply→rail wires) — those can't be
   * resolved through the live `nets[]`, which only knows component pins.
   */
  netByTiePoint: Map<string, string>;
}

/**
 * Build the node-highlight index for every breadboard tie-point — including empty
 * holes — by reusing the same union-find connectivity as `netIdForTiePoint`, but
 * resolving the WHOLE board in a single pass so callers can cache it and avoid the
 * per-tie-point rebuild cost. Returns empty maps when there are no boards.
 */
export function buildBreadboardNodeIndex(
  circuit: Circuit,
  nets: ReadonlyArray<{ id: string; pins: ReadonlyArray<readonly [string, string]> }>,
): BreadboardNodeIndex {
  const groupsByNet = new Map<string, Position[][]>();
  const netByTiePoint = new Map<string, string>();
  const boards = circuit.components.filter((c) => c.kind === "breadboard");
  if (boards.length === 0) return { groupsByNet, netByTiePoint };

  const { uf, rootToNet } = buildBoardConnectivity(circuit, nets, boards);

  // For each board, each strip/rail group that resolves to a live net becomes one
  // contiguous run (its own rectangle) and seeds netByTiePoint for wire lookup.
  const groups = [...stripNetGroups(), ...railNetGroups()];
  for (const board of boards) {
    for (const [localNetId, tpIds] of groups) {
      const root = uf.find(`bb:${board.id}\x00${localNetId}`);
      const liveNet = rootToNet.get(root);
      if (!liveNet) continue;
      const pts: Position[] = [];
      for (const tpId of tpIds) {
        const tp = tiePointFromId(tpId);
        if (!tp) continue;
        pts.push(tiePointToWorld(tp, board.position, board));
        netByTiePoint.set(`${board.id}\x00${tpId}`, liveNet);
      }
      if (pts.length === 0) continue;
      let runs = groupsByNet.get(liveNet);
      if (!runs) {
        runs = [];
        groupsByNet.set(liveNet, runs);
      }
      runs.push(pts);
    }
  }
  return { groupsByNet, netByTiePoint };
}

/**
 * Returns all main-strip tie-point IDs grouped by their shared net.
 * Each column+side is an independent net until bridged by a component or wire.
 */
export function stripNetGroups(): Map<string, string[]> {
  const groups = new Map<string, string[]>();
  for (const side of ["top", "bottom"] as BoardSide[]) {
    for (let col = 0; col < COLS; col++) {
      const netId = stripNetId(col, side);
      const ids: string[] = [];
      for (let row = 0; row < ROWS; row++) {
        ids.push(tiePointId(col, row, side));
      }
      groups.set(netId, ids);
    }
  }
  return groups;
}

/**
 * Whether a tie-point's rail is on the power (Vcc) row.
 * Rail index 0 = Vcc, rail index 1 = GND by convention.
 */
export function isVccRail(tp: TiePoint): boolean {
  return tp.isRail && tp.row === 0;
}

export function isGndRail(tp: TiePoint): boolean {
  return tp.isRail && tp.row === 1;
}

// ── Round-trip conversion ─────────────────────────────────────────────────────

/**
 * Convert a breadboard circuit into the electrical circuit consumed by
 * the simulator. The visual breadboard component has no MNA model, so its
 * tie-point aliases are materialized as wires and the board itself is omitted.
 */
export function breadboardToSimCircuit(circuit: Circuit): Circuit {
  const boards = circuit.components.filter((c) => c.kind === "breadboard");
  // Read-only showcase fixtures can render a breadboard behind explicitly
  // wired components without adding solderless-board electrical aliases.
  const electricalBoards = boards.filter((board) => !board.params.visualOnly);
  const aliasWires = breadboardAliasWires(circuit, electricalBoards);
  const boardIds = new Set(boards.map((b) => b.id));
  // The engine's Circuit no longer models the app's legacy editorMode field, so
  // there is nothing left to strip here — the spread carries only engine data.
  const { ...rest } = circuit;
  return {
    ...rest,
    components: circuit.components.filter((c) => c.kind !== "breadboard"),
    wires: [
      ...circuit.wires.filter((w) => !boardIds.has(w.from_component) && !boardIds.has(w.to_component)),
      ...aliasWires,
    ],
  };
}

export function defaultBreadboardComponent(): CircuitComponent {
  return {
    id: "bb_main",
    kind: "breadboard",
    catalogUid: "breadboard-830",
    position: { x: 80, y: 80 },
    rotation: 0,
    pins: [],
    params: { railTopVcc: 5, railBotVcc: 5 },
  };
}

export function ensureBreadboardCircuit(circuit: Circuit): Circuit {
  const { ...rest } = circuit;
  return {
    ...rest,
    components: circuit.components,
  };
}
