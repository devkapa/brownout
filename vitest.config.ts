import { defineConfig } from "vitest/config";

export default defineConfig({
  test: {
    include: ["src/**/*.test.ts", "test/**/*.test.ts"],
    environment: "node",
    // The migrated corpus predates the avr8js injection seam; the setup file
    // registers the module once per worker so MCU suites run as authored.
    setupFiles: ["test/setup/register-mcu-modules.ts"],
  },
});
