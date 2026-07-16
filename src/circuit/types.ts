/**
 * Circuit domain types used by the editor and live browser simulator.
 */

export type Rotation = 0 | 90 | 180 | 270;

export interface Position {
  x: number;
  y: number;
}

export const COMPONENT_KINDS = [
  "breadboard",
  // Legacy alias — engine still stamps these as voltage sources
  "voltage_source",
  "battery_pack",
  "bench_psu",
  "pulse_gen",
  "clock_gen",
  "signal_gen",
  // Legacy aliases kept for old-save back-compat
  "pulse_source",
  "clock",
  "resistor",
  "capacitor",
  "inductor",
  "led",
  "diode",
  "zener_diode",
  "schottky_diode",
  "switch",
  "ne555",
  "ic",
  "bjt_npn",
  "bjt_pnp",
  "nmos",
  "pmos",
  "bicolor_led",
  "rgb_led",
  "push_button",
  "spdt_switch",
  "push_dpdt",
  "potentiometer",
  "fuse",
  "trimmer",
  "ldr",
  "thermistor",
  "74ls00",
  "74ls04",
  "74ls08",
  "74ls32",
  "74ls86",
  "74ls161",
  "74ls173",
  "74ls189",
  "74ls157",
  "74ls245",
  "74ls283",
  "74hc595",
  "74hc165",
  "74hc14",
  "74hc138",
  "74ls47",
  "74hc74",
  "cd4017",
  "cd4511",
  "cd4060",
  "28c16",
  "28c256",
  "seg7_cc",
  "seg7_ca",
  "arduino_uno",
  "arduino_nano",
  "raspberry_pi_pico",
  "microbit",
  "dip_switch",
  "tvs_diode",
  "ptc_fuse",
  "ferrite_bead",
  "resistor_array",
  // W4.1 — analog op-amp ICs
  "lm358",
  "mcp6002",
  "lm386",
  // W4.2 — open-collector comparator + programmable shunt reference
  "lm393",
  "tl431",
  // W5.1 — linear voltage regulators (3-pin leaded, breadboard-mounted)
  "linear_reg",
  "lm317",
  // W5.2 — DC-DC converter module (4-pin breadboard module)
  "dcdc_converter",
  // W6.1 — electromechanical relay (SPDT, 5-pin leaded breadboard part)
  "relay",
  // W6.2 — Darlington sink arrays (open-collector, DIP-16 / DIP-18)
  "uln2003",
  "uln2803",
  // W6.2 — audio output loads (2-pin leaded breadboard parts)
  "buzzer",
  "speaker",
  // W6.3 — dual H-bridge motor driver ICs (DIP-16 / DIP-24-ish breadboard module)
  "l293d",
  "tb6612",
  // W7.1 — DC motor: series R+L winding with back-EMF and first-order speed dynamics.
  // 2-pin leaded breadboard part (m1 = + terminal, m2 = − terminal).
  "dc_motor",
  // W7.2 — Hobby servo: PWM-controlled position actuator, 3-pin leaded (sig, vplus, gnd).
  // Reads pulse HIGH-time on sig to compute angle; power pins draw idle current.
  "servo",
  // W7.2 — Bipolar stepper motor: dual-coil 4-wire, 4-pin leaded (a1, a2, b1, b2).
  // Two RL Norton companions (one per coil); step detection from committed coil sign.
  // NOTE: unipolar/ULN2003 5-/6-wire stepper is deferred to a future wave.
  "stepper",
  // W8.1 — MAX7219 8x8 LED matrix driver: SPI-like 3-wire serial (DIN/CLK/CS).
  // 5-pin breadboard module (din, clk, cs, vcc, gnd). Display state decoded from
  // sub-step MCU pin events (event path) or from sampled level comparisons (fallback).
  "max7219",
  // W8.2 — HD44780 16x2 character LCD: parallel interface (RS/RW/E + D0-D7).
  // 16-pin breadboard module. 4-bit or 8-bit mode; display state decoded from
  // sub-step MCU pin events (event path) or sampled levels (fallback).
  // v1 limits: 16x2 visible window, no RW read-back, no custom CGRAM glyphs.
  "hd44780",
  // S18b — HC-SR04 ultrasonic distance sensor: 4-pin leaded (vcc, trig, echo,
  // gnd), wide-supply (3.0-5.5 V). TRIG armed on rise, ECHO scheduled on fall;
  // see sim-engine.ts Hcsr04EngineState for the full state machine.
  "hcsr04",
  // Wave A6 — engine model families (devices/model-families.ts). Registered
  // device kinds first; canvas/catalog surfaces follow in a later wave.
  // Transformer / coupled inductors: pins a1/b1 (winding 1), a2/b2
  // (winding 2), a-pins dotted; params l1, l2, k, dcr1, dcr2.
  "coupled_inductor",
  // Depletion-mode JFETs: pins d/g/s; params vto (negative), idss, lambda.
  "njfet",
  "pjfet",
  // Latching thyristors: scr pins a/k/g, triac pins mt1/mt2/g; params vgt,
  // igt, ih, vtm, ron.
  "scr",
  "triac",
  // Optocoupler with NPN phototransistor output: pins led_a/led_k/c/e;
  // params ctr, vf, iRated, betaF.
  "opto_npn",
  // Quartz crystal: pins a/b; params fSeries (Hz), q, c0 (pF), rs (ohm).
  "crystal",
  // Transmission gate / analog switch: pins a/b/ctrl; params ron, roff.
  "analog_switch",
  // Wave A7 — independent current source (SPICE I-element interop): pins
  // pos/neg; params current (A, flowing pos -> neg through the source, the
  // SPICE convention), optional rParallel (ohm, terminal shunt).
  "current_source",
] as const;

export type ComponentKind = typeof COMPONENT_KINDS[number];

/** EEPROM component kinds whose params.contents is JSON-revision-only (never live-replicated). */
export const EEPROM_KINDS = ["28c16", "28c256"] as const;
export function isEepromKind(kind: string): boolean {
  return kind === "28c16" || kind === "28c256";
}

/**
 * Electrical role of a pin, used by the AI assistant to reason about wiring
 * without inventing function from pin labels alone. Optional everywhere so
 * legacy catalog entries and instantiated component pins stay valid.
 */
export type PinFunction =
  | "vcc"
  | "gnd"
  | "supply_pos"
  | "supply_neg"
  | "input"
  | "output"
  | "io"
  | "open_collector"
  | "open_drain"
  | "tri_state"
  | "analog_in"
  | "analog_out"
  | "clock"
  | "reset"
  | "enable"
  | "load"
  | "passive"
  | "anode"
  | "cathode"
  | "base"
  | "collector"
  | "emitter"
  | "gate"
  | "drain"
  | "source"
  | "wiper";

export interface Pin {
  id: string;
  label?: string;
  offset: Position;
  net_id?: string | null;
  function?: PinFunction | null;
}

export type ComponentParams = Record<string, number | string>;

export interface CircuitComponent {
  id: string;
  kind: ComponentKind;
  /**
   * Stable identity of the exact catalog entry used to create this instance.
   * `kind` selects simulator topology; `catalogUid` selects the physical/model
   * variant (for example 7805 vs AMS1117, or CR2032 vs 9 V battery).
   * Optional only for legacy/imported circuits; normal placement and migration
   * populate it and JSON/collaboration round-trips preserve it verbatim.
   */
  catalogUid?: string;
  position: Position;
  rotation: Rotation;
  pins: Pin[];
  params: ComponentParams;
  label?: string;
  /** When true the component's custom label stays drawn on the canvas (and so in
   *  exported/shared images), instead of only appearing on hover/selection. */
  labelPersistent?: boolean;
  /** When true the component cannot be moved, rotated, or deleted via the editor. Param edits remain permitted. */
  locked?: boolean;
}

export interface Wire {
  id: string;
  from_component: string;
  from_pin: string;
  to_component: string;
  to_pin: string;
  resistance: number;
  color?: string;
  label?: string;
  /** Intermediate world-space points the wire bends through. Empty/undefined = straight from-to line. */
  waypoints?: Position[];
  /** True when waypoints were generated by the router and may be regenerated. */
  autoRouted?: boolean;
  /**
   * When true the wire is a "jumper": electrically identical to any other wire
   * (it still connects from_pin → to_pin and is fed to the net builder the same
   * way), but BOTH endpoints render a draggable plug handle so either end can be
   * picked up and re-targeted to another hole/pin — the same feel as the Learn
   * multimeter probe, except a jumper has two movable ends. Drawn as a direct
   * point-to-point lead, so jumpers carry no waypoints. Optional: pre-jumper
   * saves simply lack the field (treated as a normal wire).
   */
  jumper?: boolean;
}

export interface Net {
  id: string;
  pins: [string, string][];
}

/**
 * A free-floating text label placed on the canvas for annotation (notes,
 * values, callouts). Not a component: it has no pins and no electrical meaning,
 * so the engine/BOM/AI ignore it entirely. Drawn by the shared renderer so it
 * appears in exported and shared images. Position is world-space (top-left
 * anchor of the text).
 */
export interface Annotation {
  id: string;
  text: string;
  position: Position;
  /** Font size in world px. Default ANNOTATION_DEFAULT_FONT_SIZE when omitted. */
  fontSize?: number;
  /** CSS colour. Default the paper/foreground token when omitted. */
  color?: string;
}

export const ANNOTATION_DEFAULT_FONT_SIZE = 13;

/**
 * Circuit-level environmental stimulus that applies to all sensor parts at once.
 * Moving stimulus here (instead of per-part params) makes it visible, editable,
 * and saved in one place — so a user can change ambient light for the whole
 * circuit without hunting for every LDR.
 */
export interface CircuitEnvironment {
  /** Ambient temperature in degrees Celsius. Read by temperature-sensitive parts (thermistor). */
  temperatureC: number;
  /** Ambient illuminance in lux. Read by light-sensitive parts (LDR). */
  lux: number;
}

/**
 * Default environment matches the historical per-part defaults (lux=100, tempC=25)
 * so circuits with no explicit environment behave identically to before migration.
 */
export const DEFAULT_ENVIRONMENT: CircuitEnvironment = { temperatureC: 25, lux: 100 };

export interface Circuit {
  id: string;
  name: string;
  components: CircuitComponent[];
  wires: Wire[];
  nets: Net[];
  schema_version: number;
  /** Circuit-level environmental stimulus. Optional because pre-v2 saves lack it. */
  environment?: CircuitEnvironment;
  /** Free-floating text annotations. Optional because pre-annotation saves lack it. */
  annotations?: Annotation[];
  netLabel?: Record<string, string>;
  /** Legacy import-only field. New saves omit it. */
  editorMode?: "open" | "breadboard";
}

/* ─────────────── Catalog ─────────────── */

/**
 * Structured operating-envelope source for the AI assistant and the
 * diagnostics/realism layers (Wave C: diagnostics engine; Waves D/E:
 * failure-mode and derating models). Every field is optional — passives carry
 * only what is relevant (e.g. tolerance), LEDs carry forward voltage, ICs
 * carry supply range and per-pin drive. The model uses this to pick parts,
 * choose values, and avoid driving outputs past their ratings.
 */
export interface ElectricalSpecs {
  /** Forward voltage drop for diodes/LEDs (volts). */
  vf?: number;
  /** Maximum forward current for LEDs/diodes (amps). */
  if_max?: number;
  /** Supply voltage range for ICs (volts). */
  vcc_range?: { min: number; nominal: number; max: number };
  /** Typical whole-package quiescent supply current (amps). */
  quiescent_current_a?: number;
  /** Maximum sourcing/sinking current per output pin (amps). */
  io_max?: number;
  /** Logic family — e.g. "LS-TTL", "HC-CMOS", "CMOS 4000". */
  logic_family?: string;
  /** Input voltage thresholds (volts) for digital inputs. v_t_plus/v_t_minus are
   *  Schmitt-trigger hysteresis thresholds defined at vcc_range.nominal; the engine
   *  scales them with the actual supply and holds state in the dead band. */
  v_input?: { v_il_max?: number; v_ih_min?: number; v_t_plus?: number; v_t_minus?: number };
  /** Maximum collector/drain current for transistors (amps). */
  i_c_max?: number;
  /** DC current gain (β) for BJTs at the nominal operating point. */
  beta_typical?: number;
  /** Catalog/package power envelope (watts), before any ambient thermal derating. */
  p_max?: number;
  /**
   * Package/body thermal reference consumed for exact supported catalog
   * identities. The engine integrates a one-pole temperature and ambient-
   * adjusted power allowance; electrical feedback and shutdown behavior remain
   * device-specific and must be disclosed by that model.
   */
  thermal?: {
    /** Junction/body-to-ambient thermal resistance (°C/W). */
    r_theta_ja_c_per_w: number;
    /** One-pole compact-model time constant (seconds). */
    time_constant_s: number;
    /** Ambient range over which the catalog profile is intended to be used. */
    ambient_range_c: { min: number; max: number };
    /** Continuous junction, film, or body-temperature ceiling (°C). */
    continuous_temperature_max_c: number;
    /** Absolute junction, film, or body-temperature ceiling (°C). */
    absolute_temperature_max_c: number;
    /** Package, PCB, or derivation condition behind r_theta_ja_c_per_w. */
    r_theta_condition: string;
    /** Optional reversible protection thresholds for devices that claim it. */
    shutdown?: {
      trip_c: number;
      restart_c: number;
      trip_is_assumed?: boolean;
      restart_is_assumed?: boolean;
    };
    /** Human-readable limits or compact-model assumptions. */
    assumptions: string[];
  };
  /** Maximum reverse/blocking voltage (volts) — rectifier and Schottky diodes. */
  vr_max?: number;
  /**
   * Breakdown knee current (amps) for zener/TVS diodes — the current flowing
   * at the rated breakdown voltage. Used to derive Is_z for the breakdown stamp.
   * Catalog default: 5 mA.
   */
  iz_knee?: number;
  /** Free-text spec the model can read when nothing structured fits. */
  notes?: string;
  /**
   * Fractional tolerance on the part's principal value (resistance or capacitance).
   * 0.05 = ±5%, 0.1 = ±10%, 0.2 = ±20%. Used by the Monte Carlo runner to
   * perturb params.resistance / params.capacitance per run.
   */
  value_tol?: number;
  /**
   * Fractional spread on the forward voltage (Vf).
   * 0.05 = ±5%. Used by the Monte Carlo runner to perturb params.vf
   * (and params.vf1/vf2 for bicolor, params.vf_r/vf_g/vf_b for RGB LEDs).
   */
  vf_tol?: number;
  /**
   * Fractional spread on logic input thresholds (v_il_max, v_ih_min).
   * METADATA ONLY in v1: the engine reads thresholds from the catalog module-level
   * catalogByKind map — there is no per-instance override mechanism — so the
   * Monte Carlo runner does NOT vary thresholds yet. Populated for documentation
   * and future runner support.
   */
  vth_tol?: number;
  /**
   * Catalog-declared propagation delay (nanoseconds) for logic parts.
   * The engine prefers this over the kind-prefix fallback map
   * (propagationDelayForKind in sim-engine.ts).
   */
  prop_delay_ns?: number;
}

/**
 * Vendor reference used in a BOM line. Intentionally left empty in catalog data for
 * v1 (generic BOM only); a later pass will populate sku/mpn/vendors.
 */
export interface BomVendorRef {
  name: string;
  url: string;
  affiliate?: boolean;
}

/**
 * Purchasing and assembly metadata for one catalog part. Used by a later extractor
 * to generate a shopping list (BOM) from any circuit. Fields are designed for a
 * human reader, not a machine-readable BOM format — the extractor formats the output.
 */
export interface BomMetadata {
  /**
   * Coarse purchasing family used to deduplicate and group interchangeable parts
   * in a BOM. Keep it coarse: a red LED and a green LED both belong to "led-5mm"
   * so the extractor can merge them into a single line item when colour doesn't matter.
   */
  bomFamily: string;

  /** Human-readable generic part name for a BOM line (e.g. "1/4 W carbon-film resistor"). */
  genericName: string;

  /** Physical package description (e.g. "axial", "radial", "TO-92", "DIP-8", "module"). */
  package: string;

  /** How the part is mounted / used. */
  mounting: "through-hole" | "breadboard-leaded" | "module" | "panel" | "wire";

  /** True when the part fits a standard 0.1"-pitch solderless breadboard directly. */
  breadboardFriendly: boolean;

  /**
   * Set false only for lab/bench instruments that are NOT bought per project
   * (bench PSU, pulse generator, clock generator). The extractor uses this flag
   * to exclude bench gear from the shopping list. Omit (treated as true) for
   * everything else, including the battery pack.
   */
  purchasable?: boolean;

  /** Polarity guidance for polarized parts (LEDs, electrolytic caps, diodes, ICs, etc.). */
  polarityNotes?: string;

  /** Pin-1 / orientation guidance (DIP notch side, transistor flat face, etc.). */
  orientationNotes?: string;

  /** Concise human-readable rating summary useful for purchasing (e.g. ">=1/4 W", ">=16 V"). */
  ratings?: string;

  /** Component tolerance (e.g. "+-5%", "+-10%"). */
  tolerance?: string;

  /** Acceptable substitute descriptions a buyer can use if the primary part is unavailable. */
  commonSubstitutions?: string[];

  /**
   * Stable slugs naming support-part FAMILIES this part commonly needs when used
   * on a breadboard. The extractor maps these slugs to reminder line items.
   * Allowed values: "series-resistor", "decoupling-capacitor", "dip-socket",
   * "usb-cable", "jumper-wires".
   */
  supportParts?: string[];

  /** One plain-English sentence a beginner can read to understand what this part is. */
  beginnerDescription: string;

  // Purchasing fields — left unset in catalog data for generic-BOM-only v1.
  // A later enrichment pass will populate these from distributor APIs.
  sku?: string;
  mpn?: string;
  vendors?: BomVendorRef[];
}

export interface PartDefinition {
  uid: string;
  name: string;
  description: string;
  kind: ComponentKind;
  category: string;
  icon?: string | null;
  width: number;
  height: number;
  default_params: ComponentParams;
  pin_layout: Pin[];
  spice_model?: string | null;
  paramUnits?: Record<string, string> | null;
  paramTypes?: Record<string, "number" | "boolean" | "enum" | "percent" | "integer"> | null;
  /** Plain-English display label per param key (Inspector). Falls back to the raw key. */
  paramLabels?: Record<string, string> | null;
  /** One-line beginner help per param key (Inspector, small muted text). */
  paramHelp?: Record<string, string> | null;
  /** Param keys tucked into the collapsed "Advanced" section (Inspector). */
  paramAdvanced?: string[] | null;
  paramEnums?: Record<string, string[]> | null;
  /** Short "use this when..." note shown to the AI assistant. */
  applications?: string | null;
  /** Structured electrical specs exposed to the AI assistant. */
  electrical_specs?: ElectricalSpecs | null;
  /** Purchasing and assembly metadata for BOM generation. */
  bom?: BomMetadata | null;
  /**
   * Part tier gating for a later rollout stage. Absent means free.
   * The field exists now so the rollout stage can gate parts without a schema change.
   */
  availability?: "free" | "pro";
  /**
   * Hide this variant from the component palette while keeping it in the catalog
   * for BOM resolution and the AI bundle. Used for the rail-less breadboard
   * variants, which users reach via the Inspector "Power rails" toggle rather
   * than as separate palette tiles.
   */
  paletteHidden?: boolean;
  /**
   * Lowercase search aliases for the palette's fuzzy filter (e.g. a teacher typing
   * "light sensor" should surface the photoresistor). Never rendered — search-only.
   * Electrical taxonomy (category) stays authoritative for BOM/exports; this field
   * exists purely to bridge functional vocabulary to that taxonomy.
   */
  keywords?: string[];
  /**
   * Raw category ids this part is ADDITIONALLY listed under in the palette (on top
   * of its primary `category`), for parts that are functionally at home in more
   * than one group (e.g. the potentiometer is electrically `passive` but a teacher
   * browsing "input" should still find it). Palette-display-only: does not change
   * `category`, and must never affect BOM, exports, or engine behavior.
   */
  alsoIn?: string[];
}

export interface PartCatalog {
  version: number;
  parts: PartDefinition[];
}

/* ─────────────── Simulation ─────────────── */

export type AnalysisKind = "op" | "transient" | "ac";

export interface OperatingPointAnalysis {
  kind: "op";
}

export interface TransientAnalysis {
  kind: "transient";
  stop: number;
  step: number;
  start?: number;
  chunk_size?: number;
}

export interface ACAnalysis {
  kind: "ac";
  start_frequency: number;
  stop_frequency: number;
  points_per_decade: number;
  variation: "dec" | "oct" | "lin";
}

export type Analysis =
  | OperatingPointAnalysis
  | TransientAnalysis
  | ACAnalysis;

export interface SimulationRequest {
  circuit: Circuit;
  analysis: Analysis;
  probes: string[];
}

export interface SimWarning {
  code: string;
  message: string;
  component_id?: string | null;
}

export interface OperatingPointResult {
  kind: "op";
  node_voltages: Record<string, number>;
  warnings: SimWarning[];
}

export interface TransientResult {
  kind: "transient";
  time: number[];
  node_voltages: Record<string, number[]>;
  warnings: SimWarning[];
}

export type SimulationResult = OperatingPointResult | TransientResult;
