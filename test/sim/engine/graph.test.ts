import { describe, expect, it } from "vitest";
import { buildNets } from "../../../src/sim/engine/graph.js";
import { SimEngine, type SimCircuit } from "../../../src/sim/engine/sim-engine.js";

function netFor(
  nets: ReturnType<typeof buildNets>,
  componentId: string,
  pinId: string,
) {
  return nets.find((net) =>
    net.pins.some(([cid, pid]) => cid === componentId && pid === pinId),
  );
}

describe("buildNets — voltage reference without invented connectivity", () => {
  it("uses one source return as the reference without shorting a second source return", () => {
    const circuit = {
      components: [
        { id: "v1", kind: "voltage_source", pins: [{ id: "pos" }, { id: "neg" }] },
        { id: "v2", kind: "voltage_source", pins: [{ id: "pos" }, { id: "neg" }] },
      ],
      wires: [],
    };

    const nets = buildNets(circuit);
    const v1Neg = netFor(nets, "v1", "neg");
    const v2Neg = netFor(nets, "v2", "neg");

    expect(v1Neg?.id).toBe("gnd");
    expect(v2Neg).toBeDefined();
    expect(v2Neg?.id).not.toBe("gnd");
    expect(v2Neg?.id).not.toBe(v1Neg?.id);
  });

  it("supports sources in series instead of silently paralleling their negative terminals", () => {
    const circuit: SimCircuit = {
      components: [
        { id: "v1", kind: "voltage_source", pins: [{ id: "pos" }, { id: "neg" }], params: { voltage: 5 } },
        { id: "v2", kind: "voltage_source", pins: [{ id: "pos" }, { id: "neg" }], params: { voltage: 3 } },
        { id: "load", kind: "resistor", pins: [{ id: "a" }, { id: "b" }], params: { resistance: 800 } },
      ],
      wires: [
        { from_component: "v1", from_pin: "pos", to_component: "v2", to_pin: "neg" },
        { from_component: "v2", from_pin: "pos", to_component: "load", to_pin: "a" },
        { from_component: "load", from_pin: "b", to_component: "v1", to_pin: "neg" },
      ],
    };

    const engine = new SimEngine();
    engine.load(circuit);
    for (let i = 0; i < 5; i++) engine.step(1e-4);

    const v2PosNet = engine.nets.find((net) =>
      net.pins.some(([cid, pid]) => cid === "v2" && pid === "pos"),
    );
    const output = v2PosNet ? engine.getNetV()[v2PosNet.id] : undefined;

    // 5 V + 3 V in series across 800 ohm gives 8 V and 10 mA.
    expect(output).toBeCloseTo(8, 6);
    expect(Math.abs(engine.getElementI().v1 ?? 0)).toBeCloseTo(0.01, 6);
    expect(engine.lastConverged).toBe(true);
  });

  it("keeps an MCU ground separate from the source return until it is wired", () => {
    const circuit = {
      components: [
        { id: "supply", kind: "voltage_source", pins: [{ id: "pos" }, { id: "neg" }] },
        { id: "uno", kind: "arduino_uno", pins: [{ id: "5v" }, { id: "gnd" }, { id: "gnd2" }] },
      ],
      wires: [],
    };

    const nets = buildNets(circuit);
    const supplyReturn = netFor(nets, "supply", "neg");
    const unoGround = netFor(nets, "uno", "gnd");

    expect(supplyReturn?.id).toBe("gnd");
    expect(unoGround?.id).not.toBe("gnd");
    expect(unoGround?.pins).toEqual(expect.arrayContaining([
      ["uno", "gnd"],
      ["uno", "gnd2"],
    ]));
  });
});

describe("buildNets — physical same-package unions", () => {
  it("unions all four L293D ground pins", () => {
    const circuit = {
      components: [{
        id: "driver",
        kind: "l293d",
        pins: [
          { id: "gnd1" }, { id: "gnd2" }, { id: "gnd3" }, { id: "gnd4" },
          { id: "vcc1" }, { id: "vcc2" },
        ],
      }],
      wires: [],
    };

    const nets = buildNets(circuit);
    const ground = netFor(nets, "driver", "gnd1");
    expect(ground?.pins).toEqual(expect.arrayContaining([
      ["driver", "gnd1"], ["driver", "gnd2"],
      ["driver", "gnd3"], ["driver", "gnd4"],
    ]));
  });

  it("unions every TB6612 breakout ground pin", () => {
    const circuit = {
      components: [{
        id: "driver",
        kind: "tb6612",
        pins: [{ id: "gnd" }, { id: "gnd2" }, { id: "gnd3" }, { id: "vm" }],
      }],
      wires: [],
    };

    const nets = buildNets(circuit);
    const ground = netFor(nets, "driver", "gnd");
    expect(ground?.pins).toEqual(expect.arrayContaining([
      ["driver", "gnd"], ["driver", "gnd2"], ["driver", "gnd3"],
    ]));
  });

  it("unions the non-isolated buck module's input and output returns", () => {
    const circuit = {
      components: [{
        id: "buck",
        kind: "dcdc_converter",
        catalogUid: "dcdc-buck-5v",
        pins: [
          { id: "in_pos" }, { id: "in_neg" },
          { id: "out_pos" }, { id: "out_neg" },
        ],
      }],
      wires: [],
    };

    const nets = buildNets(circuit);
    const commonReturn = netFor(nets, "buck", "in_neg");
    expect(netFor(nets, "buck", "out_neg")?.id).toBe(commonReturn?.id);
    expect(commonReturn?.pins).toEqual(expect.arrayContaining([
      ["buck", "in_neg"], ["buck", "out_neg"],
    ]));
  });

  it.each(["seg7_cc", "seg7_ca"])("unions both %s common leads", (kind) => {
    const circuit = {
      components: [{
        id: "display",
        kind,
        pins: [
          { id: "com" }, { id: "com2" },
          { id: "a" }, { id: "b" }, { id: "c" }, { id: "d" },
          { id: "e" }, { id: "f" }, { id: "g" }, { id: "dp" },
        ],
      }],
      wires: [],
    };

    const nets = buildNets(circuit);
    const common = netFor(nets, "display", "com");
    expect(netFor(nets, "display", "com2")?.id).toBe(common?.id);
    expect(common?.pins).toEqual(expect.arrayContaining([
      ["display", "com"], ["display", "com2"],
    ]));
  });
});
