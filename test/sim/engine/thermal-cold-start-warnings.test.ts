/**
 * A package's thermal warnings after load().
 *
 * A thermal state that is new (no carried entry with the same catalog uid and
 * profile) starts cold. Every later step publishes the profile's own
 * assumptions plus that step's derating warnings, so the cold start must
 * publish the same union; before, it carried only the derating warnings and
 * the profile's assumptions appeared once a 1 ps seed solve had converged.
 */
import { describe, expect, it } from "vitest";
import { SimEngine, type SimCircuit } from "../../../src/sim/engine/sim-engine.js";
import { thermalProfilesByCatalogUid } from "../../../src/sim/thermal-physics.js";

function source(id: string, voltage: number): SimCircuit["components"][number] {
  return { id, kind: "voltage_source", pins: [{ id: "pos" }, { id: "neg" }], params: { voltage } };
}

function wire(
  from_component: string,
  from_pin: string,
  to_component: string,
  to_pin: string,
): SimCircuit["wires"][number] {
  return { from_component, from_pin, to_component, to_pin };
}

function resistorBoard(extraSource: boolean): SimCircuit {
  const components: SimCircuit["components"] = [
    source("supply", 5),
    {
      id: "load",
      kind: "resistor",
      catalogUid: "resistor",
      pins: [{ id: "a" }, { id: "b" }],
      params: { resistance: 100 },
    },
  ];
  const wires = [wire("supply", "pos", "load", "a"), wire("supply", "neg", "load", "b")];
  if (extraSource) {
    // A second, contradictory source across the same two nets makes the matrix
    // singular, so load()'s 1 ps seed solve is rejected and the cold-start state
    // is all the engine publishes.
    components.push(source("fight", 3));
    wires.push(wire("fight", "pos", "load", "a"), wire("fight", "neg", "load", "b"));
  }
  return { components, wires, environment: { temperatureC: 25, lux: 100 } };
}

describe("thermal cold start", () => {
  const profileWarnings = thermalProfilesByCatalogUid()["resistor"]!.warnings;

  it("publishes the profile's warnings when the seed solve is rejected", () => {
    const engine = new SimEngine();
    engine.load(resistorBoard(true));
    // Guard: the seed really was rejected, otherwise this is the ordinary path.
    expect(engine.lastConverged).toBe(false);

    const state = engine.getThermalState("load");
    expect(profileWarnings.length).toBeGreaterThan(0);
    expect(state?.warnings).toEqual(expect.arrayContaining([...profileWarnings]));
  });

  it("publishes the same profile warnings once the seed converges", () => {
    const engine = new SimEngine();
    engine.load(resistorBoard(false));
    expect(engine.lastConverged).toBe(true);
    expect(engine.getThermalState("load")?.warnings).toEqual(expect.arrayContaining([...profileWarnings]));
  });
});
