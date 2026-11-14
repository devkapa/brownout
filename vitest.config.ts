import { defineConfig } from "vitest/config";

export default defineConfig({
  test: {
    include: ["src/**/*.test.ts", "test/**/*.test.ts"],
    // Minutes-long regressions run in the gate (`pnpm run test:slow`,
    // vitest.slow.config.ts), not on every `pnpm test`.
    exclude: ["**/node_modules/**", "**/*.slow.test.ts"],
    environment: "node",
    // Builds the Rust core to wasm once, before any test file, so
    // src/core/brownout-core.wasm exists and is fresh against the current
    // source. globalSetup (unlike setupFiles) runs once for the whole run,
    // in the main process, not once per worker.
    globalSetup: ["test/setup/build-core.ts"],
    // The migrated corpus predates the avr8js injection seam; the setup file
    // registers the module once per worker so MCU suites run as authored.
    setupFiles: ["test/setup/register-mcu-modules.ts"],
  },
});
