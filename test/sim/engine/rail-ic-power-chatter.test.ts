/**
 * A chip that cannot power up on its own current-limited rail froze the
 * simulation with nothing said. A 555 whose TRIG, THR, RESET and DIS sit on
 * the rail of a bench supply in current limit: the limit charges the rail
 * capacitor up to the chip's power-on level, and there, powered, THR at VCC
 * puts OUT low and DIS on, which loads the rail past the limit and pulls it
 * back under that level; unpowered, nothing loads it and the rail rises past
 * it again. No state holds at any step size, so the hosts retried down to
 * their 10 ns floor and stopped the clock (the cold fuzz's 2575 tpl 2 on the
 * core, its rail freed by a switch, stalled the same way at 1.81 V). The
 * engine still stops there, since no physical state lets it go on, but it now
 * says why: a warning naming the chip and the rail.
 */
import { describe, expect, it } from "vitest";
import { HeadlessRunner } from "../../../src/host/headless.js";
import type { SimCircuit } from "../../../src/sim/engine/sim-engine.js";

type Component = SimCircuit["components"][number];

const part = (
  id: string,
  kind: string,
  catalogUid: string | undefined,
  pins: string[],
  params: Record<string, number | string> = {},
): Component => ({ id, kind, ...(catalogUid ? { catalogUid } : {}), pins: pins.map((pinId) => ({ id: pinId })), params });

const wire = (fromComponent: string, fromPin: string, toComponent: string, toPin: string) => ({
  from_component: fromComponent,
  from_pin: fromPin,
  to_component: toComponent,
  to_pin: toPin,
});

const TIMER_PINS = ["1", "2", "3", "4", "5", "6", "7", "8"];
const SUPPLY_V = 9;
const LIMIT_A = 0.1;
const RAIL_C = 10e-9;

/**
 * A {9 V, 100 mA} bench supply with 10 nF on its rail and a 555 on it.
 * `starving`: TRIG, THR, RESET and DIS all on the rail, the chip's own pins
 * loading its supply. Otherwise a 1 kohm / 10 kohm / 100 nF astable, whose
 * supply passes the same power-on level on the way up.
 */
function timerOnLimitedRail(starving: boolean): SimCircuit {
  const components: Component[] = [
    part("gnd", "ground", undefined, ["gnd"]),
    part("psu", "bench_psu", "supply-var", ["pos", "neg"], { voltage: SUPPLY_V, iLimit: LIMIT_A }),
    part("crail", "capacitor", undefined, ["a", "b"], { capacitance: RAIL_C, esr: 0 }),
    part("timer", "ne555", "ne555", TIMER_PINS),
  ];
  const wires = [
    wire("psu", "neg", "gnd", "gnd"),
    wire("crail", "a", "psu", "pos"),
    wire("crail", "b", "gnd", "gnd"),
    wire("timer", "8", "psu", "pos"),
    wire("timer", "1", "gnd", "gnd"),
    wire("timer", "4", "psu", "pos"),
  ];
  if (starving) {
    for (const pin of ["2", "6", "7"]) wires.push(wire("timer", pin, "psu", "pos"));
  } else {
    components.push(
      part("r1", "resistor", "resistor", ["a", "b"], { resistance: 1000 }),
      part("r2", "resistor", "resistor", ["a", "b"], { resistance: 10000 }),
      part("ct", "capacitor", undefined, ["a", "b"], { capacitance: 100e-9, esr: 0 }),
    );
    wires.push(
      wire("r1", "a", "psu", "pos"),
      wire("r1", "b", "timer", "7"),
      wire("r2", "a", "timer", "7"),
      wire("r2", "b", "timer", "6"),
      wire("timer", "6", "timer", "2"),
      wire("ct", "a", "timer", "2"),
      wire("ct", "b", "gnd", "gnd"),
    );
  }
  return { components, wires };
}

describe("a rail IC that cannot power up on its own current-limited rail", () => {
  // The catalog 555's power-on level: 90% of its 4.5 V minimum supply.
  const powerOn = 0.9 * 4.5;

  it("stops at its power-on level and says which chip on which rail", () => {
    const runner = new HeadlessRunner();
    runner.load(timerOnLimitedRail(true));
    const rail = runner.netIdFor("psu", "pos")!;
    const result = runner.run({ durationS: 1e-3 });

    // By hand: the limit charges 10 nF at 100 mA / 10 nF = 1e7 V/s, so the
    // rail reaches 4.05 V 0.405 us after load.
    expect(result.hitMinStep).toBe(true);
    expect(runner.simTime).toBeGreaterThan(0.39e-6);
    expect(runner.simTime).toBeLessThan(0.43e-6);
    expect(Math.abs(runner.snapshot().netV[rail]! - powerOn)).toBeLessThan(0.1);

    const warnings = runner.engine.takeNewWarnings();
    expect(warnings).toHaveLength(1);
    const [warning] = warnings;
    expect(warning!.code).toBe("rail_ic_power_chatter");
    expect(warning!.componentId).toBe("timer");
    expect(warning!.message).toContain("timer");
    expect(warning!.message).toContain("psu");
    expect(warning!.message).toContain("its own pins load");
    expect(warning!.value).toBeCloseTo(powerOn, 9);

    // It stays stopped, and the warning is not announced again.
    const again = runner.run({ durationS: 1e-3 });
    expect(again.hitMinStep).toBe(true);
    expect(runner.simTime).toBeLessThan(0.43e-6);
    expect(runner.engine.takeNewWarnings()).toEqual([]);
    expect(Object.keys(runner.engine.getWarnings())).toHaveLength(1);
  });

  it("leaves a 555 that powers up through the same level alone", () => {
    const runner = new HeadlessRunner();
    runner.load(timerOnLimitedRail(false));
    const result = runner.run({ durationS: 2e-3 });
    expect(result.hitMinStep).toBe(false);
    expect(runner.simTime).toBeCloseTo(2e-3, 12);
    expect(runner.engine.takeNewWarnings()).toEqual([]);
  });
});
