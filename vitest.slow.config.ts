import { defineConfig } from "vitest/config";

/**
 * The slow regressions (`*.slow.test.ts`): minutes each, so the gate runs them
 * (`pnpm run test:slow`) and the default `pnpm test` does not. Fast forms of
 * the same checks run in the default suite or in cargo.
 */
export default defineConfig({
  test: {
    include: ["test/**/*.slow.test.ts"],
    environment: "node",
    globalSetup: ["test/setup/build-core.ts"],
    setupFiles: ["test/setup/register-mcu-modules.ts"],
    testTimeout: 900_000,
  },
});
