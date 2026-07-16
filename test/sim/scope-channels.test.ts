import { describe, expect, it } from "vitest";
import {
  buildScopeChannels,
  reconcileScopeSelection,
  sameScopeTopology,
} from "../../src/sim/scope-channels.js";
import type { Net } from "../../src/sim/engine/sim-engine.js";

function net(id: string, pins: Array<[string, string]>): Net {
  return { id, pins };
}

describe("scope channel helpers", () => {
  it("keeps net channels in natural order and excludes ground", () => {
    expect(buildScopeChannels([
      net("n10", [["r1", "a"]]),
      net("gnd", [["g1", "g"]]),
      net("n2", [["r2", "a"]]),
      net("n0", [["r3", "a"]]),
    ]).map((channel) => channel.id)).toEqual(["n0", "n2", "n10"]);
  });

  it("detects topology changes even when net ids are reused", () => {
    const before = buildScopeChannels([
      net("n0", [["r1", "a"], ["led1", "anode"]]),
    ]);
    const after = buildScopeChannels([
      net("n0", [["r1", "b"], ["led1", "cathode"]]),
    ]);

    expect(sameScopeTopology(before, after)).toBe(false);
  });

  it("remaps selected channels by signature when ids shift", () => {
    const before = buildScopeChannels([
      net("n0", [["r1", "a"], ["led1", "anode"]]),
      net("n1", [["r1", "b"], ["g1", "g"]]),
    ]);
    const after = buildScopeChannels([
      net("n3", [["r1", "a"], ["led1", "anode"]]),
      net("n4", [["r1", "b"], ["g1", "g"]]),
    ]);
    const signatures = new Map(before.map((channel) => [channel.id, channel.signature]));

    expect([...reconcileScopeSelection(["n0"], signatures, after, { autoFollow: false })])
      .toEqual(["n3"]);
  });

  it("auto-follows default channels after the selection disappears", () => {
    const channels = buildScopeChannels([
      net("n0", [["a", "1"]]),
      net("n1", [["b", "1"]]),
      net("n2", [["c", "1"]]),
    ]);

    expect([...reconcileScopeSelection(["missing"], new Map(), channels, { autoFollow: false })])
      .toEqual(["n0", "n1", "n2"]);
  });
});
