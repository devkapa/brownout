import fs from "node:fs";
import type { ComponentKind, ComponentParams, ElectricalSpecs, Pin } from "../../src/circuit/types.js";

/**
 * Full devolt part catalog, copied to test/fixtures during the B2 corpus
 * migration. The corpus was authored against `import rawCatalog from
 * "@devolt/catalog"`; suites feed it into diagnostics and read expected
 * electrical specs from it, and several of those expectations (breadboards,
 * stripped metadata fields) exist only in the full catalog — the package's
 * bundled default-parts subset is deliberately smaller. Loaded via fs rather
 * than a JSON import so the helper stays agnostic to import-attribute
 * handling across tsc NodeNext and the vitest transform.
 *
 * App-domain module that the engine corpus exercises but the published package
 * does not ship. The catalog SHAPE lives here for the same reason the fixture
 * data does: the engine's PartDefinition is model identity only (uid, kind,
 * default_params, pin_layout, electrical_specs), while this fixture is a real
 * de:volt product catalog that additionally carries palette copy, BOM/purchasing
 * metadata and Inspector hints. Suites asserting on those fields are asserting
 * about the FIXTURE, not about engine API — declaring them on the shipped
 * PartDefinition would publish de:volt's product schema as though the solver
 * depended on it.
 */

/** Vendor reference used in a BOM line. */
export interface CatalogBomVendorRef {
  name: string;
  url: string;
  affiliate?: boolean;
}

/** Purchasing and assembly metadata for one catalog part. */
export interface CatalogBomMetadata {
  bomFamily: string;
  genericName: string;
  package: string;
  mounting: "through-hole" | "breadboard-leaded" | "module" | "panel" | "wire";
  breadboardFriendly: boolean;
  purchasable?: boolean;
  polarityNotes?: string;
  orientationNotes?: string;
  ratings?: string;
  tolerance?: string;
  commonSubstitutions?: string[];
  supportParts?: string[];
  beginnerDescription: string;
  sku?: string;
  mpn?: string;
  vendors?: CatalogBomVendorRef[];
}

/**
 * One entry of the de:volt product catalog: engine model identity plus the
 * presentation/commerce metadata the app layers on top. Structurally assignable
 * to the engine's PartDefinition, so a fixture part can be injected through
 * setPartLibrary() exactly as a host would inject its own catalog.
 */
export interface CatalogPartDefinition {
  // Engine model identity — mirrors the shipped PartDefinition.
  uid: string;
  kind: ComponentKind;
  default_params: ComponentParams;
  pin_layout: Pin[];
  spice_model?: string | null;
  electrical_specs?: ElectricalSpecs | null;

  // App-domain: presentation, teaching copy, commerce.
  name: string;
  description: string;
  category: string;
  icon?: string | null;
  width: number;
  height: number;
  paramUnits?: Record<string, string> | null;
  paramTypes?: Record<string, "number" | "boolean" | "enum" | "percent" | "integer"> | null;
  paramLabels?: Record<string, string> | null;
  paramHelp?: Record<string, string> | null;
  paramAdvanced?: string[] | null;
  paramEnums?: Record<string, string[]> | null;
  applications?: string | null;
  bom?: CatalogBomMetadata | null;
  availability?: "free" | "pro";
  paletteHidden?: boolean;
  keywords?: string[];
  alsoIn?: string[];
}

/** The fixture's own top-level shape (mirrors the engine's PartCatalog). */
export interface CatalogFixture {
  version: number;
  parts: CatalogPartDefinition[];
}

const rawCatalog = JSON.parse(
  fs.readFileSync(new URL("../fixtures/catalog/parts.json", import.meta.url), "utf8"),
) as CatalogFixture;

export default rawCatalog;
