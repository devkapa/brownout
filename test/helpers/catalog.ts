import fs from "node:fs";
import type { PartCatalog } from "../../src/circuit/types.js";

/**
 * Full devolt part catalog, copied to test/fixtures during the B2 corpus
 * migration. The corpus was authored against `import rawCatalog from
 * "@devolt/catalog"`; suites feed it into diagnostics and read expected
 * electrical specs from it, and several of those expectations (breadboards,
 * stripped metadata fields) exist only in the full catalog — the package's
 * bundled default-parts subset is deliberately smaller. Loaded via fs rather
 * than a JSON import so the helper stays agnostic to import-attribute
 * handling across tsc NodeNext and the vitest transform.
 */
const rawCatalog = JSON.parse(
  fs.readFileSync(new URL("../fixtures/catalog/parts.json", import.meta.url), "utf8"),
) as PartCatalog;

export default rawCatalog;
