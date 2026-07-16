export interface AlignedTraceData {
  t: number[];
  values: number[][];
}

/**
 * Build a plot-only min/max envelope while leaving acquisition memory intact.
 * Each bucket contributes every channel's extrema; the aligned union preserves
 * narrow pulses and cross-channel peaks. Finite/non-finite boundaries are
 * retained explicitly so reload/pause gaps cannot be drawn across.
 */
export function decimateAlignedTrace(
  t: readonly number[],
  values: readonly (readonly number[])[],
  maxPoints: number,
): AlignedTraceData {
  const length = Math.min(t.length, ...values.map((channel) => channel.length));
  const budget = Math.max(2, Math.floor(maxPoints));
  if (length <= budget) {
    return {
      t: t.slice(0, length),
      values: values.map((channel) => channel.slice(0, length)),
    };
  }

  const channelCount = Math.max(1, values.length);
  const bucketCount = Math.max(1, Math.floor(budget / (2 * channelCount)));
  const bucketWidth = Math.max(1, Math.ceil(length / bucketCount));
  const retained = new Set<number>([0, length - 1]);

  // Preserve both sides of every finite/gap transition. A long NaN run needs
  // only its boundaries to remain an honest disconnected segment.
  for (const channel of values) {
    let previousFinite = Number.isFinite(channel[0]);
    for (let index = 1; index < length; index++) {
      const finite = Number.isFinite(channel[index]);
      if (finite !== previousFinite) {
        retained.add(index - 1);
        retained.add(index);
      }
      previousFinite = finite;
    }
  }

  for (let start = 0; start < length; start += bucketWidth) {
    const end = Math.min(length, start + bucketWidth);
    for (const channel of values) {
      let minIndex = -1;
      let maxIndex = -1;
      let minValue = Number.POSITIVE_INFINITY;
      let maxValue = Number.NEGATIVE_INFINITY;
      for (let index = start; index < end; index++) {
        const value = channel[index];
        if (!Number.isFinite(value)) continue;
        if (value < minValue) {
          minValue = value;
          minIndex = index;
        }
        if (value > maxValue) {
          maxValue = value;
          maxIndex = index;
        }
      }
      if (minIndex >= 0) retained.add(minIndex);
      if (maxIndex >= 0) retained.add(maxIndex);
    }
  }

  const indices = [...retained].sort((a, b) => a - b);
  return {
    t: indices.map((index) => t[index]!),
    values: values.map((channel) => indices.map((index) => channel[index]!)),
  };
}
