/**
 * SPICE interop entry: the netlist subset parser, the .model card parameter
 * mappers, and the directive runner (.op/.tran/.dc/.ac) that executes a
 * parsed netlist on SimEngine.
 *
 * Everything the parser and runner return is re-exported so a consumer can
 * type a full round trip. The mapping from netlist devices onto engine kinds
 * happens inside run.ts and is intentionally not a public extension point:
 * new SPICE device support lands through the device registry plus a parser
 * update, not by consumers monkey-patching the element mapping.
 */

export * from "./sim/engine/spice/netlist.js";
export * from "./sim/engine/spice/models.js";
export * from "./sim/engine/spice/run.js";
