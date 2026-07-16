/**
 * Tests for buildOperatingPoint and operatingPointToCsv.
 */

import { describe, expect, it } from "vitest";
import {
  buildOperatingPoint,
  operatingPointCurrentMeaning,
  operatingPointPowerMeaning,
  operatingPointPowerUnavailableLabel,
  operatingPointToCsv,
} from "../../src/analysis/operating-point.js";
import type { OperatingPointInput } from "../../src/analysis/operating-point.js";
import type { Net } from "../../src/sim/engine/graph.js";
import type { PhysicsTelemetrySnapshot } from "../../src/sim/messages.js";

// Minimal SimFailure-like object — only need componentId in tests.
type MinFailure = { componentId?: string; kind: string; message: string };

function net(id: string, pins: Array<[string, string]>): Net {
  return { id, pins: pins.map(([c, p]) => [c, p]) };
}

function defaultInput(overrides: Partial<OperatingPointInput> = {}): OperatingPointInput {
  return {
    nets: [
      net("gnd", [["r1", "a"]]),
      net("n0", [["r1", "b"], ["led1", "a"]]),
      net("n1", [["led1", "b"]]),
    ],
    netV: { gnd: 0, n0: 3.3, n1: 5.0 },
    elementI: { r1: 0.015, led1: 0.015 },
    elementChannelI: {},
    digitalState: {},
    failures: {},
    converged: true,
    simTime: 0.001,
    components: [
      { id: "r1", kind: "resistor", label: "R1" },
      { id: "led1", kind: "led", label: "LED1" },
    ],
    netLabels: {},
    ...overrides,
  };
}

const PROVENANCE = {
  source: "worker-derived" as const,
  quality: "model-derived" as const,
  method: "test",
};

function physicsTelemetry(
  components: PhysicsTelemetrySnapshot["components"],
): PhysicsTelemetrySnapshot {
  return {
    schemaVersion: 1,
    sampledAtSimTimeS: 0.001,
    electricalSample: "last-committed-solution",
    components,
  };
}

describe("buildOperatingPoint", () => {
  // ── Net sorting ────────────────────────────────────────────────────────────

  it("puts gnd first in net list", () => {
    const op = buildOperatingPoint(defaultInput());
    expect(op.nets[0]!.id).toBe("gnd");
  });

  it("sorts numeric nets in natural order after gnd", () => {
    const input = defaultInput({
      nets: [
        net("n2", [["c", "a"]]),
        net("n0", [["a", "a"]]),
        net("gnd", [["b", "a"]]),
        net("n1", [["d", "a"]]),
      ],
      netV: { gnd: 0, n0: 1, n1: 2, n2: 3 },
    });
    const op = buildOperatingPoint(input);
    const ids = op.nets.map((n) => n.id);
    expect(ids).toEqual(["gnd", "n0", "n1", "n2"]);
  });

  it("applies netLabels to net rows", () => {
    const op = buildOperatingPoint(defaultInput({ netLabels: { n0: "VCC" } }));
    const n0 = op.nets.find((n) => n.id === "n0")!;
    expect(n0.label).toBe("VCC");
  });

  it("falls back to the net id when no label is present", () => {
    const op = buildOperatingPoint(defaultInput({ netLabels: {} }));
    const n0 = op.nets.find((n) => n.id === "n0")!;
    expect(n0.label).toBe("n0");
  });

  // ── Voltages ───────────────────────────────────────────────────────────────

  it("records net voltages from netV", () => {
    const op = buildOperatingPoint(defaultInput());
    const n0 = op.nets.find((n) => n.id === "n0")!;
    expect(n0.voltage).toBe(3.3);
  });

  it("keeps a missing net voltage unavailable instead of fabricating 0 V", () => {
    const op = buildOperatingPoint(defaultInput({ netV: { gnd: 0, n1: 5 } }));
    const n0 = op.nets.find((n) => n.id === "n0")!;
    expect(n0.voltage).toBeUndefined();
    expect(operatingPointToCsv(op)).toContain("n0,n0,");
  });

  // ── Component currents ────────────────────────────────────────────────────

  it("records component branch currents", () => {
    const op = buildOperatingPoint(defaultInput());
    const r1 = op.components.find((c) => c.id === "r1")!;
    expect(r1.current).toBe(0.015);
  });

  // ── Power computation ─────────────────────────────────────────────────────

  it("computes power for a two-pin resistive kind when both pin nets are resolvable", () => {
    // r1 connects gnd (0 V) and n0 (3.3 V), current 0.015 A
    // Expected power: |3.3 - 0| * |0.015| = 0.0495 W
    const op = buildOperatingPoint(defaultInput());
    const r1 = op.components.find((c) => c.id === "r1")!;
    expect(r1.power).toBeCloseTo(0.0495, 8);
  });

  it("computes power for led kind", () => {
    // led1 connects n0 (3.3 V) and n1 (5.0 V), current 0.015 A
    // Power = |5.0 - 3.3| * 0.015 = 0.0255 W
    const op = buildOperatingPoint(defaultInput());
    const led1 = op.components.find((c) => c.id === "led1")!;
    expect(led1.power).toBeCloseTo(0.0255, 8);
  });

  it("prefers exact identity-matched structured current and power", () => {
    const op = buildOperatingPoint(defaultInput({
      components: [{ id: "c1", kind: "capacitor", label: "C1", catalogUid: "cap-film" }],
      elementI: { c1: 99 },
      physicsTelemetry: physicsTelemetry({
        c1: {
          componentKind: "capacitor",
          catalogUid: "cap-film",
          current: {
            valueA: 0.002,
            reference: "pin0-to-pin1",
            direction: "positive-pin0-to-pin1",
            aggregation: "single-path",
            provenance: PROVENANCE,
          },
          power: {
            valueW: 0.01,
            signConvention: "positive-absorbed",
            terminals: ["a", "b"],
            provenance: PROVENANCE,
          },
        },
      }),
    }));

    const c1 = op.components[0]!;
    expect(c1.current).toBe(0.002);
    expect(c1.power).toBe(0.01);
    expect(c1.currentTelemetry).toEqual({
      valueA: 0.002,
      reference: "pin0-to-pin1",
      direction: "positive-pin0-to-pin1",
      aggregation: "single-path",
      provenance: PROVENANCE,
    });
    expect(c1.powerTelemetry).toEqual({
      valueW: 0.01,
      signConvention: "positive-absorbed",
      terminals: ["a", "b"],
      provenance: PROVENANCE,
    });
    expect(operatingPointCurrentMeaning(c1)).toContain("positive from pin 0 to pin 1");
    expect(operatingPointPowerMeaning(c1)).toContain("Positive absorbed, negative delivered");
    expect(c1.powerUnavailableReason).toBeUndefined();
  });

  it("rejects structured telemetry from a stale catalog identity", () => {
    const op = buildOperatingPoint(defaultInput({
      components: [{ id: "r1", kind: "resistor", catalogUid: "resistor-1k" }],
      elementI: { r1: 99 },
      physicsTelemetry: physicsTelemetry({
        r1: {
          componentKind: "resistor",
          catalogUid: "resistor-10k",
          current: {
            valueA: 99,
            reference: "pin0-to-pin1",
            provenance: PROVENANCE,
          },
          power: {
            valueW: 99,
            signConvention: "positive-absorbed",
            terminals: ["a", "b"],
            provenance: PROVENANCE,
          },
        },
      }),
    }));

    expect(op.components[0]?.current).toBeUndefined();
    expect(op.components[0]?.power).toBeUndefined();
    expect(op.components[0]?.powerUnavailableReason).toBe("unavailable");
  });

  it("marks omitted active-device aggregate power as not exposed", () => {
    const op = buildOperatingPoint(defaultInput({
      components: [{ id: "q1", kind: "bjt_npn", catalogUid: "2n3904" }],
      physicsTelemetry: physicsTelemetry({
        q1: {
          componentKind: "bjt_npn",
          catalogUid: "2n3904",
          current: {
            valueA: 0.02,
            reference: "collector",
            aggregation: "single-path",
            provenance: PROVENANCE,
          },
        },
      }),
    }));

    expect(op.components[0]?.current).toBe(0.02);
    expect(op.components[0]?.power).toBeUndefined();
    expect(op.components[0]?.powerUnavailableReason).toBe("not-exposed");
    expect(operatingPointPowerUnavailableLabel(op.components[0]?.powerUnavailableReason))
      .toBe("Not exposed");
  });

  it("omits power for a potentiometer even in a 2-net (rheostat) wiring", () => {
    // pot1 wired as a rheostat (wiper tied to an end) touches only 2 nets, but
    // its elementI is the whole-track current, not the conducting-path current,
    // so power is not honestly computable and must be omitted regardless of net
    // count.
    const input = defaultInput({
      nets: [
        net("gnd", [["pot1", "ccw"]]),
        net("n0", [["pot1", "cw"], ["pot1", "wiper"]]),
      ],
      netV: { gnd: 0, n0: 3.3 },
      elementI: { pot1: 0.01 },
      components: [{ id: "pot1", kind: "potentiometer", label: "POT1" }],
    });
    const op = buildOperatingPoint(input);
    const pot1 = op.components.find((c) => c.id === "pot1")!;
    expect(pot1.current).toBe(0.01);
    expect(pot1.power).toBeUndefined();
  });

  it("omits power when a component appears in more than two nets (unresolvable)", () => {
    // cap1 appears in three nets — power must be omitted.
    const input = defaultInput({
      nets: [
        net("gnd", [["r1", "a"], ["cap1", "a"]]),
        net("n0", [["r1", "b"], ["cap1", "b"]]),
        net("n1", [["led1", "a"], ["cap1", "c"]]),
        net("n2", [["led1", "b"]]),
      ],
      components: [
        { id: "r1", kind: "resistor", label: "R1" },
        { id: "led1", kind: "led", label: "LED1" },
        { id: "cap1", kind: "capacitor", label: "C1" },
      ],
      elementI: { r1: 0.01, led1: 0.01, cap1: 0.001 },
      netV: { gnd: 0, n0: 5, n1: 3, n2: 0 },
    });
    const op = buildOperatingPoint(input);
    const cap1 = op.components.find((c) => c.id === "cap1")!;
    // cap1 is not in TWO_PIN_RESISTIVE_KINDS and appears in 3 nets → no power.
    expect(cap1.power).toBeUndefined();
  });

  it("omits power for non-resistive kinds (capacitor)", () => {
    const input = defaultInput({
      nets: [
        net("gnd", [["r1", "a"], ["cap1", "a"]]),
        net("n0", [["r1", "b"], ["cap1", "b"]]),
      ],
      components: [
        { id: "r1", kind: "resistor" },
        { id: "cap1", kind: "capacitor" },
      ],
      elementI: { r1: 0.01, cap1: 0.001 },
      netV: { gnd: 0, n0: 5 },
    });
    const op = buildOperatingPoint(input);
    const cap1 = op.components.find((c) => c.id === "cap1")!;
    expect(cap1.power).toBeUndefined();
  });

  it("omits power when current is undefined (no elementI entry)", () => {
    const input = defaultInput({
      nets: [
        net("gnd", [["r1", "a"]]),
        net("n0", [["r1", "b"]]),
      ],
      components: [{ id: "r1", kind: "resistor" }],
      elementI: {}, // no entry for r1
      netV: { gnd: 0, n0: 5 },
    });
    const op = buildOperatingPoint(input);
    const r1 = op.components.find((c) => c.id === "r1")!;
    expect(r1.current).toBeUndefined();
    expect(r1.power).toBeUndefined();
  });

  // ── Non-converged warning ─────────────────────────────────────────────────

  it("adds a non-converged warning when converged = false", () => {
    const op = buildOperatingPoint(defaultInput({ converged: false }));
    expect(op.warnings.length).toBeGreaterThan(0);
    expect(op.warnings[0]).toMatch(/not converged/i);
  });

  it("has no warnings when converged = true", () => {
    const op = buildOperatingPoint(defaultInput({ converged: true }));
    expect(op.warnings).toHaveLength(0);
  });

  // ── Digital states and failures ───────────────────────────────────────────

  it("carries digital state through to component rows", () => {
    const op = buildOperatingPoint(
      defaultInput({ digitalState: { clk1: 1 }, components: [{ id: "clk1", kind: "clock" }] }),
    );
    const clk1 = op.components.find((c) => c.id === "clk1")!;
    expect(clk1.digitalState).toBe(1);
  });

  it("marks failed components with failed = true", () => {
    const op = buildOperatingPoint(
      defaultInput({
        failures: { "f1": { componentId: "r1", kind: "open", message: "open-circuit" } as MinFailure as never },
      }),
    );
    const r1 = op.components.find((c) => c.id === "r1")!;
    expect(r1.failed).toBe(true);
  });

  it("does not mark failed when there are no failures", () => {
    const op = buildOperatingPoint(defaultInput());
    for (const comp of op.components) {
      expect(comp.failed).toBeUndefined();
    }
  });

  it("does not mark a reversible output-sag warning as component failure", () => {
    const op = buildOperatingPoint(
      defaultInput({
        failures: {
          sag: {
            componentId: "r1",
            kind: "output_sag",
            message: "loaded output below guaranteed HIGH",
          } as MinFailure as never,
        },
      }),
    );
    expect(op.components.find((component) => component.id === "r1")?.failed).toBeUndefined();
  });

  // ── simTime and converged in output ───────────────────────────────────────

  it("passes simTime and converged through to the output", () => {
    const op = buildOperatingPoint(defaultInput({ simTime: 0.123, converged: false }));
    expect(op.simTime).toBe(0.123);
    expect(op.converged).toBe(false);
  });
});

// ── operatingPointToCsv ───────────────────────────────────────────────────────

describe("operatingPointToCsv", () => {
  it("produces a two-section CSV with net and component tables", () => {
    const op = buildOperatingPoint(defaultInput());
    const csv = operatingPointToCsv(op);
    expect(csv).toContain("# Nets");
    expect(csv).toContain("# Components");
    expect(csv).toContain("net_id,label,voltage_V");
    expect(csv).toContain("component_id,kind,label,current_value_A,power_value_W");
    expect(csv).toContain("current_reference,current_positive_direction,current_aggregation");
    expect(csv).toContain("power_sign_convention,power_terminals");
  });

  it("exports structured sign, aggregation, terminal, and provenance fields", () => {
    const op = buildOperatingPoint(defaultInput({
      components: [{ id: "src", kind: "signal_gen", catalogUid: "sig-1" }],
      physicsTelemetry: physicsTelemetry({
        src: {
          componentKind: "signal_gen",
          catalogUid: "sig-1",
          current: {
            valueA: 0.02,
            reference: "source-output",
            direction: "positive-delivered",
            aggregation: "single-path",
            provenance: PROVENANCE,
          },
          power: {
            valueW: -0.1,
            signConvention: "positive-absorbed",
            terminals: ["out", "gnd"],
            provenance: PROVENANCE,
          },
        },
      }),
    }));

    const csv = operatingPointToCsv(op);
    expect(csv).toContain("source-output,positive-delivered,single-path,worker-derived,model-derived,test");
    expect(csv).toContain("positive-absorbed,out:gnd,worker-derived,model-derived,test");
  });

  it("labels legacy scalar signs as unqualified instead of borrowing a convention", () => {
    const csv = operatingPointToCsv(buildOperatingPoint(defaultInput()));
    expect(csv).toContain("unqualified-legacy");
    expect(csv).toContain("magnitude-only");
    expect(operatingPointCurrentMeaning(buildOperatingPoint(defaultInput()).components[0]!))
      .toMatch(/sign and aggregation are not qualified/i);
  });

  it("includes a warnings section when warnings are present", () => {
    const op = buildOperatingPoint(defaultInput({ converged: false }));
    const csv = operatingPointToCsv(op);
    expect(csv).toContain("# Warnings");
  });

  it("omits the warnings section when there are no warnings", () => {
    const op = buildOperatingPoint(defaultInput({ converged: true }));
    const csv = operatingPointToCsv(op);
    expect(csv).not.toContain("# Warnings");
  });

  it("uses CRLF line endings", () => {
    const op = buildOperatingPoint(defaultInput());
    const csv = operatingPointToCsv(op);
    expect(csv.endsWith("\r\n")).toBe(true);
    expect(csv.includes("\r\n")).toBe(true);
  });
});
