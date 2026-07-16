import type { Net } from "./engine/sim-engine.js";

export const DEFAULT_SCOPE_CHANNELS = 4;
// Non-entitled fallback: the channel cap used when no entitlement limit is
// available (e.g. during SSR or before the entitlements store initialises).
export const MAX_SCOPE_CHANNELS = 8;
// The hard upper bound for Pro subscribers. Kept here so scope-channel logic
// stays co-located rather than being duplicated across UI and tests.
export const PRO_MAX_SCOPE_CHANNELS = 16;

export interface ScopeChannel {
  id: string;
  label: string;
  signature: string;
  pinCount: number;
}

export function netSignature(net: Pick<Net, "pins">): string {
  return net.pins
    .map(([component, pin]) => `${component}.${pin}`)
    .sort()
    .join("|");
}

export function buildScopeChannels(
  nets: Net[],
  netLabels: Record<string, string> = {},
): ScopeChannel[] {
  return nets
    .filter((net) => net.id !== "gnd")
    .map((net) => ({
      id: net.id,
      label: netLabels[net.id] || net.id,
      signature: netSignature(net),
      pinCount: net.pins.length,
    }))
    .sort((a, b) => naturalNetCompare(a.id, b.id));
}

export function sameScopeTopology(a: ScopeChannel[], b: ScopeChannel[]): boolean {
  if (a.length !== b.length) return false;
  return a.every((channel, index) => {
    const other = b[index];
    return channel.id === other?.id && channel.signature === other.signature;
  });
}

export function reconcileScopeSelection(
  previousSelection: Iterable<string>,
  previousSignatures: ReadonlyMap<string, string>,
  channels: ScopeChannel[],
  options: { autoFollow: boolean; maxChannels?: number; defaultChannels?: number },
): Set<string> {
  const maxChannels = options.maxChannels ?? MAX_SCOPE_CHANNELS;
  const defaultChannels = options.defaultChannels ?? DEFAULT_SCOPE_CHANNELS;
  const byId = new Map(channels.map((channel) => [channel.id, channel]));
  const bySignature = new Map(channels.map((channel) => [channel.signature, channel]));
  const next = new Set<string>();

  for (const id of previousSelection) {
    const direct = byId.get(id);
    if (direct) {
      next.add(direct.id);
    } else {
      const signature = previousSignatures.get(id);
      const remapped = signature ? bySignature.get(signature) : undefined;
      if (remapped) next.add(remapped.id);
    }
    if (next.size >= maxChannels) return next;
  }

  if (options.autoFollow || next.size === 0) {
    for (const channel of channels) {
      if (next.size >= Math.min(defaultChannels, maxChannels)) break;
      next.add(channel.id);
    }
  }

  return next;
}

function naturalNetCompare(a: string, b: string): number {
  const an = /^n(\d+)$/.exec(a);
  const bn = /^n(\d+)$/.exec(b);
  if (an && bn) return Number(an[1]) - Number(bn[1]);
  return a.localeCompare(b);
}
