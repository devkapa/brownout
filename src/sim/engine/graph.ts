/**
 * Client-side deterministic net builder.
 *
 * Converts a circuit's wire list into a list of nets using union-find.
 * Runs entirely in the worker, with no network round-trips.
 */

export type PinKey = [string, string]; // [componentId, pinId]

export interface Net {
  id: string;
  pins: PinKey[];
}

interface MinCircuit {
  components: Array<{
    id: string;
    kind: string;
    /** Exact model identity when available; omitted by legacy/minimal callers. */
    catalogUid?: string;
    pins: Array<{ id: string }>;
  }>;
  wires: Array<{
    from_component: string;
    from_pin: string;
    to_component: string;
    to_pin: string;
  }>;
}

function pk(cid: string, pid: string): string {
  return `${cid}\x00${pid}`;
}

function parsePk(s: string): PinKey {
  const i = s.indexOf("\x00");
  return [s.slice(0, i), s.slice(i + 1)];
}

function unionize(groups: Set<string>[]): Set<string>[] {
  const remaining = [...groups];
  const out: Set<string>[] = [];
  while (remaining.length > 0) {
    let cur = remaining.shift()!;
    let grew = true;
    while (grew) {
      grew = false;
      const next: Set<string>[] = [];
      for (const r of remaining) {
        let overlap = false;
        for (const e of r) {
          if (cur.has(e)) {
            overlap = true;
            break;
          }
        }
        if (overlap) {
          for (const e of r) cur.add(e);
          grew = true;
        } else {
          next.push(r);
        }
      }
      remaining.length = 0;
      remaining.push(...next);
    }
    out.push(cur);
  }
  return out;
}

export function buildNets(circuit: MinCircuit): Net[] {
  const groups: Set<string>[] = circuit.wires.map(
    (w) => new Set([pk(w.from_component, w.from_pin), pk(w.to_component, w.to_pin)]),
  );

  // MNA needs exactly one voltage reference, but that reference is only a
  // coordinate choice: it must never create electrical connectivity. Prefer
  // the first independent source return so the usual single-supply circuit
  // still reads 0 V at its negative terminal. A board ground is a fallback for
  // source-less/USB-powered circuits. Other source returns and board grounds
  // remain separate unless the user physically wires them together.
  const SOURCE_KINDS = new Set([
    "battery_pack",
    "bench_psu",
    "voltage_source",
    "signal_gen",
    "pulse_source",
    "pulse_gen",
    "clock",
    "clock_gen",
  ]);
  const sourceReferences: string[] = [];
  const boardReferences: string[] = [];

  const unionInternalPins = (componentId: string, pinIds: string[]): void => {
    if (pinIds.length > 1) {
      groups.push(new Set(pinIds.map((id) => pk(componentId, id))));
    }
  };

  for (const c of circuit.components) {
    if (SOURCE_KINDS.has(c.kind)) {
      const returnPin = c.pins.find((p) => p.id === "neg")
        ?? c.pins.find((p) => p.id === "gnd");
      if (returnPin) sourceReferences.push(pk(c.id, returnPin.id));
    }
    if (c.kind === "arduino_uno" || c.kind === "arduino_nano") {
      const arduinoGrounds = c.pins
        .map((p) => p.id)
        .filter((id) => id === "gnd" || id === "gnd2" || id === "gnd_top");
      boardReferences.push(...arduinoGrounds.map((id) => pk(c.id, id)));
      unionInternalPins(c.id, arduinoGrounds);
    }
    if (c.kind === "raspberry_pi_pico") {
      // All Pico GND/AGND pads share copper on the same physical board.
      const picoGrounds = c.pins
        .map((p) => p.id)
        .filter((id) => /^gnd\d*$/.test(id) || id === "agnd");
      boardReferences.push(...picoGrounds.map((id) => pk(c.id, id)));
      unionInternalPins(c.id, picoGrounds);
    }
    if (c.kind === "microbit") {
      if (c.pins.some((p) => p.id === "gnd")) boardReferences.push(pk(c.id, "gnd"));
    }
    if (c.kind === "l293d") {
      // Pins 4, 5, 12 and 13 are one exposed ground/heat-sink structure in
      // the package. Wiring any one therefore grounds all four electrically.
      unionInternalPins(
        c.id,
        c.pins.map((p) => p.id).filter((id) => /^gnd[1-4]$/.test(id)),
      );
    }
    if (c.kind === "tb6612") {
      // The breakout exposes several labels for the module's common ground.
      unionInternalPins(
        c.id,
        c.pins.map((p) => p.id).filter((id) => /^gnd\d*$/.test(id)),
      );
    }
    if (
      c.kind === "dcdc_converter"
      && (c.catalogUid === undefined || c.catalogUid === "dcdc-buck-5v")
    ) {
      // The catalog buck module is non-isolated: IN− and OUT− are the same
      // copper return. Legacy dcdc_converter instances resolve to this model too.
      unionInternalPins(
        c.id,
        c.pins.map((p) => p.id).filter((id) => id === "in_neg" || id === "out_neg"),
      );
    }
    if (c.kind === "seg7_cc" || c.kind === "seg7_ca") {
      // The two COM leads are duplicate package pins bonded to one common
      // anode/cathode node; wiring either must electrically reach both.
      unionInternalPins(
        c.id,
        c.pins.map((p) => p.id).filter((id) => id === "com" || id === "com2"),
      );
    }
    if (c.kind === "arduino_nano") {
      // Nano's RESET pin appears on both edges (`reset` top row, `reset2` bottom row); short them together
      // so a wire to either label resets the MCU and pin labels match the silkscreen on both sides.
      const resetPins = c.pins.map((p) => p.id).filter((id) => id === "reset" || id === "reset2");
      if (resetPins.length > 1) {
        groups.push(new Set(resetPins.map((id) => pk(c.id, id))));
      }
    }
    if (c.kind === "push_button") {
      // 4-pin tactile button: each side's two legs are one piece of metal, so
      // a≡a2 (left side) and b≡b2 (right side) are permanently the SAME net — not
      // two nets bridged by a resistor. Shorting them here (rather than via an
      // engine stamp) means the breadboard/visual net model and the solver agree,
      // and a leg alone in its column still reads as connected to its partner.
      const pbIds = new Set(c.pins.map((p) => p.id));
      if (pbIds.has("a") && pbIds.has("a2")) groups.push(new Set([pk(c.id, "a"), pk(c.id, "a2")]));
      if (pbIds.has("b") && pbIds.has("b2")) groups.push(new Set([pk(c.id, "b"), pk(c.id, "b2")]));
    }
  }

  const referencePin = sourceReferences[0] ?? boardReferences[0];
  // Make a standalone reference pin participate in unionize without joining
  // it to any other candidate. This is a numerical reference, not a wire.
  if (referencePin) groups.push(new Set([referencePin]));

  const merged = unionize(groups);

  let gndIdx = -1;
  if (referencePin) {
    for (let i = 0; i < merged.length; i++) {
      if (merged[i].has(referencePin)) {
        gndIdx = i;
        break;
      }
    }
  }

  let next = 0;
  const nets: Net[] = [];
  for (let i = 0; i < merged.length; i++) {
    const id = i === gndIdx ? "gnd" : `n${next++}`;
    nets.push({ id, pins: [...merged[i]].map(parsePk) });
  }

  // Floating pins each become their own single-pin net.
  const wired = new Set<string>();
  for (const n of nets) for (const p of n.pins) wired.add(pk(p[0], p[1]));
  for (const c of circuit.components) {
    for (const p of c.pins) {
      const key = pk(c.id, p.id);
      if (!wired.has(key)) {
        nets.push({ id: `n${next++}`, pins: [[c.id, p.id]] });
        wired.add(key);
      }
    }
  }

  return nets;
}
