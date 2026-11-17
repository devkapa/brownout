/**
 * Supplemental classic cases for the W1e LED and display kinds (rgb_led,
 * bicolor_led, seg7_cc, seg7_ca).
 *
 * WHY THESE EXIST. The classic suites that build these kinds (led-current.test.ts,
 * seg7-load.test.ts) read their result as per-channel currents through
 * `getElementChannelI()` and check it against bands (7-11 mA, under 1 uA). The
 * cases below check the junction VOLTAGE against the diode law the parts are
 * defined by, node voltages only (`nets`/`getNetV` and `step()`, the same
 * convention spst-switch.test.ts uses), so each runs on classic and on the core
 * alike and pins the forward voltage, not just a current band. They were first
 * written to give rgb_led, bicolor_led and seg7_ca a portable gate-1 case while
 * the gate-1 adapter still stubbed `getElementChannelI()` as `{}`; since the
 * adapter delegates it, the channel cases in those suites are portable too.
 *
 * WHERE THE NUMBERS COME FROM. Not from either engine: each expectation solves
 * the load line of one junction against the diode law the parts are defined
 * by (spec chapter 6.1; core/README.md, "Kernel semantics: LEDs and displays",
 * point 2): `Is = iRated * exp(-Vf / (n * Vt))`, `n = 2`, `Vt = kT/q` at 25 C,
 * `I(v) = Is * (exp(v / (n * Vt)) - 1)`, in series with the resistor, and
 * bisects for the junction voltage. Vf is the part's forward voltage at
 * iRated: 1.8 V red, 2.0 V green and amber segments, 3.0 V blue, the
 * datasheet-typical numbers the catalog carries. The junction sits below Vf
 * because the drive current (9 mA on a 330 ohm resistor from 5 V) is under the
 * 20 mA rating.
 *
 * Everything is settled inside one load: five 100 us steps, no reload.
 */
import { describe, expect, it } from "vitest";
import { SimEngine, type SimCircuit } from "../../../src/sim/engine/sim-engine.js";

const K_B_OVER_Q = 8.617333262145e-5;
const AMBIENT_C = 25;
const IDEALITY = 2;
/** 0.1 mV on a junction voltage: 0.3 uA through 330 ohm, far inside either engine's Newton tolerance. */
const VOLTS_TOLERANCE = 1e-4;

/** The junction voltage where the diode's current equals the series resistor's, found by bisection on the load line. */
function loadLineJunctionVolts(supply: number, resistance: number, vf: number, iRated: number): number {
  const vtn = IDEALITY * K_B_OVER_Q * (AMBIENT_C + 273.15);
  const saturation = iRated * Math.exp(-vf / vtn);
  let low = 0;
  let high = supply;
  for (let i = 0; i < 200; i++) {
    const mid = (low + high) / 2;
    const diodeAmps = saturation * (Math.exp(mid / vtn) - 1);
    const resistorAmps = (supply - mid) / resistance;
    if (diodeAmps < resistorAmps) low = mid;
    else high = mid;
  }
  return (low + high) / 2;
}

function pin(id: string): { id: string } {
  return { id };
}

function resistor(id: string, resistance: number): SimCircuit["components"][number] {
  return { id, kind: "resistor", pins: [pin("a"), pin("b")], params: { resistance } };
}

function supply(id: string, voltage: number): SimCircuit["components"][number] {
  return { id, kind: "voltage_source", pins: [pin("pos"), pin("neg")], params: { voltage } };
}

function wire(
  fromComponent: string,
  fromPin: string,
  toComponent: string,
  toPin: string,
): SimCircuit["wires"][number] {
  return { from_component: fromComponent, from_pin: fromPin, to_component: toComponent, to_pin: toPin };
}

function netVoltage(engine: SimEngine, componentId: string, pinId: string): number {
  const net = engine.nets.find((n) => n.pins.some(([cid, pid]) => cid === componentId && pid === pinId));
  if (!net) throw new Error(`No net found for ${componentId}.${pinId}`);
  return engine.getNetV()[net.id] ?? 0;
}

function run(circuit: SimCircuit): SimEngine {
  const engine = new SimEngine();
  engine.load(circuit);
  for (let i = 0; i < 5; i++) engine.step(1e-4);
  return engine;
}

describe("rgb_led — each channel is its own junction against the common cathode", () => {
  it("drops its own forward voltage on every channel: blue above green above red", () => {
    const vf = { r: 1.8, g: 2.0, b: 3.0 };
    const iRated = 0.02;
    const circuit: SimCircuit = {
      components: [
        supply("vcc", 5),
        resistor("r_r", 330),
        resistor("r_g", 330),
        resistor("r_b", 330),
        {
          id: "led1",
          kind: "rgb_led",
          pins: [pin("r_a"), pin("g_a"), pin("b_a"), pin("com_k")],
          params: { vf_r: vf.r, vf_g: vf.g, vf_b: vf.b, iRated },
        },
      ],
      wires: [
        wire("vcc", "pos", "r_r", "a"),
        wire("vcc", "pos", "r_g", "a"),
        wire("vcc", "pos", "r_b", "a"),
        wire("r_r", "b", "led1", "r_a"),
        wire("r_g", "b", "led1", "g_a"),
        wire("r_b", "b", "led1", "b_a"),
        wire("led1", "com_k", "vcc", "neg"),
      ],
      environment: { temperatureC: AMBIENT_C },
    };
    const engine = run(circuit);
    const red = netVoltage(engine, "led1", "r_a");
    const green = netVoltage(engine, "led1", "g_a");
    const blue = netVoltage(engine, "led1", "b_a");

    expect(Math.abs(red - loadLineJunctionVolts(5, 330, vf.r, iRated))).toBeLessThan(VOLTS_TOLERANCE);
    expect(Math.abs(green - loadLineJunctionVolts(5, 330, vf.g, iRated))).toBeLessThan(VOLTS_TOLERANCE);
    expect(Math.abs(blue - loadLineJunctionVolts(5, 330, vf.b, iRated))).toBeLessThan(VOLTS_TOLERANCE);
    expect(blue).toBeGreaterThan(green);
    expect(green).toBeGreaterThan(red);
  });
});

describe("bicolor_led — two anodes against one shared cathode", () => {
  it("drops each anode's own forward voltage through its own resistor", () => {
    const iRated = 0.02;
    const circuit: SimCircuit = {
      components: [
        supply("vcc", 5),
        resistor("r1", 330),
        resistor("r2", 1_000),
        {
          id: "led1",
          kind: "bicolor_led",
          pins: [pin("a1"), pin("k"), pin("a2")],
          params: { vf1: 1.8, vf2: 2.0, iRated },
        },
      ],
      wires: [
        wire("vcc", "pos", "r1", "a"),
        wire("vcc", "pos", "r2", "a"),
        wire("r1", "b", "led1", "a1"),
        wire("r2", "b", "led1", "a2"),
        wire("led1", "k", "vcc", "neg"),
      ],
      environment: { temperatureC: AMBIENT_C },
    };
    const engine = run(circuit);

    expect(Math.abs(netVoltage(engine, "led1", "a1") - loadLineJunctionVolts(5, 330, 1.8, iRated))).toBeLessThan(
      VOLTS_TOLERANCE,
    );
    expect(Math.abs(netVoltage(engine, "led1", "a2") - loadLineJunctionVolts(5, 1_000, 2.0, iRated))).toBeLessThan(
      VOLTS_TOLERANCE,
    );
  });
});

// ---- 7-segment displays ----------------------------------------------------

type SegKind = "seg7_cc" | "seg7_ca";

/** Package pin order of the catalog part; every pin is addressed by id, so the order is only for realism. */
const SEG7_PIN_IDS = ["g", "f", "com", "a", "b", "e", "d", "com2", "c", "dp"];

/**
 * One segment (`a`) wired through 330 ohm to the far supply rail and the
 * common lead to the near rail; the other seven segments stay unwired.
 * `polarity` "correct" puts the common lead where the kind's diodes want it
 * (ground for a common cathode, 5 V for a common anode); "reversed" swaps it.
 */
function segmentCircuit(kind: SegKind, polarity: "correct" | "reversed", iRated: number): SimCircuit {
  const commonAtGround = (kind === "seg7_cc") === (polarity === "correct");
  return {
    components: [
      supply("vcc", 5),
      resistor("r_a", 330),
      { id: "disp", kind, pins: SEG7_PIN_IDS.map(pin), params: { vf: 2.0, iRated } },
    ],
    wires: commonAtGround
      ? [wire("vcc", "pos", "r_a", "a"), wire("r_a", "b", "disp", "a"), wire("vcc", "neg", "disp", "com")]
      : [wire("vcc", "pos", "disp", "com"), wire("disp", "a", "r_a", "a"), wire("r_a", "b", "vcc", "neg")],
    environment: { temperatureC: AMBIENT_C },
  };
}

describe("7-segment displays — a segment is one junction against the common lead", () => {
  it.each(["seg7_cc", "seg7_ca"] as const)("%s: a correctly wired segment drops its forward voltage", (kind) => {
    const iRated = 0.02;
    const engine = run(segmentCircuit(kind, "correct", iRated));
    const segment = netVoltage(engine, "disp", "a");
    // Common cathode: the segment node is the anode, so it sits at the junction voltage above ground.
    // Common anode: the segment node is the cathode, so it sits the junction voltage below the 5 V common.
    const junction = kind === "seg7_cc" ? segment : 5 - segment;

    expect(Math.abs(junction - loadLineJunctionVolts(5, 330, 2.0, iRated))).toBeLessThan(VOLTS_TOLERANCE);
  });

  it.each(["seg7_cc", "seg7_ca"] as const)("%s: a segment wired against its polarity does not conduct", (kind) => {
    const engine = run(segmentCircuit(kind, "reversed", 0.02));
    const segment = netVoltage(engine, "disp", "a");
    // Reverse-biased, the diode carries only leakage; the 330 ohm resistor then holds the node at its far rail.
    const farRail = kind === "seg7_cc" ? 0 : 5;

    expect(Math.abs(segment - farRail)).toBeLessThan(1e-6);
  });
});
