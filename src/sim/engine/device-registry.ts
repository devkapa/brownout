/**
 * Device model registry — the extension seam that lets a component kind be
 * implemented outside sim-engine.ts (Wave A4).
 *
 * WHY THIS EXISTS
 * The engine historically implemented every component as a case inside four
 * giant per-pass switches (_stampAll, _updateState, _updateElementI,
 * _updateFailureStates). Adding a component meant forking the engine. This
 * registry inverts that: a device module registers a DeviceModel per kind,
 * and each engine pass consults the registry per component before falling
 * back to its remaining switch cases. Cohorts migrate incrementally; the
 * switches shrink to nothing without ever changing numeric behavior.
 *
 * DESIGN DECISIONS (the contract every device module relies on)
 *
 * 1. Dispatch preserves iteration order bit-exactly. The engine keeps
 *    iterating its compiled behavior buckets (_staticStampComponents,
 *    _dynamicStampComponents, _stateUpdateComponents,
 *    _digitalUpdateComponents, _failureUpdateComponents, and the full
 *    component list for element currents) in their existing order and looks
 *    up the handler per component. Handlers are never grouped or re-ordered
 *    by kind: float accumulation order into the MNA matrix is semantics, so
 *    the registry must be invisible to the solver's arithmetic.
 *
 * 2. Migration is MOVE-ONLY. A migrated handler body is the engine's old
 *    switch-case body verbatim, with identifier access adapted through the
 *    DeviceContext facade (ctx.pinNode for this._pinNode, ctx.state.caps for
 *    this.state.caps, and so on). Every DeviceContext member is therefore a
 *    thin alias of the exact engine expression the old case used — no
 *    caching, no reformulation — so a handler computes the same bits the
 *    case did.
 *
 * 3. Registration is explicit and deterministic. devices/index.ts registers
 *    every model in a fixed written-out order and sim-engine.ts imports that
 *    single module. Device modules themselves have no registration side
 *    effects, so nothing depends on incidental import graph order. Duplicate
 *    kind registration throws: silently replacing a physics model is never
 *    acceptable, and load order must not decide which model wins.
 *
 * 4. staticStamp requires staticSignature. Kinds that participate in the
 *    captured static MNA base (the FULL_STATIC_STAMP_KINDS optimization) are
 *    re-stamped only when their signature changes; a static model without a
 *    signature hook would silently serve a stale matrix after a direct
 *    params mutation (tests and analysis tools mutate params in place).
 *    Registration rejects that combination up front.
 *
 * 5. Buckets and engine machinery stay in the engine. The compiled metadata,
 *    static-base capture/refresh, DC operating-point ladder (Wave A3),
 *    trap-mode gating (Wave A2), and the per-pass loops all remain in
 *    sim-engine.ts. Bucket membership for registered kinds is derived as
 *    (engine const set) OR (registry hook present), which is a no-op for
 *    migrated kinds — they remain listed in the engine sets — and lets a
 *    third-party kind join a pass without editing the engine.
 *
 * 6. DeviceContext is the future public plugin API of the standalone
 *    engine. It deliberately does not expose the SimEngine instance. `mna`
 *    is typed as MnaStampSurface (size/add/addB only) so the stamp-only
 *    contract is structural, not documentation: the clear/captureBase/solve
 *    lifecycle is engine-owned, and the shared element stamp helpers in
 *    elements.ts accept the same narrowed surface.
 *
 * 7. Import layering: device modules may import the registry types, the
 *    pure stamp helpers in elements.ts, and pure exported helper FUNCTIONS
 *    and consts from sim-engine.ts (function declarations are hoisted, so
 *    the sim-engine -> devices/index -> device module -> sim-engine cycle is
 *    well-defined). They must never read sim-engine module state at module
 *    evaluation time — only from inside handlers, which run long after both
 *    modules are initialized. The module-scope failure mode is bundler-
 *    dependent and can be SILENT: reading an in-cycle const at module scope
 *    throws a TDZ ReferenceError under native ESM, but esbuild's const->var
 *    lowering makes the same read evaluate to undefined, registering a
 *    model with NaN physics constants. Keep every such read inside handlers.
 */

import type { AcStampSurface } from "./ac-system.js";
import type { MnaStampSurface } from "./linear-system.js";
import type { MicrocontrollerCore } from "./mcu.js";
import type { ElectricalSpecs, PartDefinition } from "../../circuit/types.js";
import type {
  BatteryOperatingPoint,
  BatteryPhysicsProfile,
} from "../battery-physics.js";
import type { DisplayInfo } from "../messages.js";
import type {
  BatteryRuntimeState,
  EepromState,
  Hcsr04EngineState,
  Hd44780State,
  IcPowerInfo,
  MosfetGateState,
  MotorEngineState,
  NE555EngineState,
  PtcState,
  RelayEngineState,
  ServoEngineState,
  SimCircuit,
  SimFailure,
  SimFailureKind,
  SimWarning,
  SimWarningCode,
  StepperEngineState,
  ThermalRuntimeState,
} from "./sim-engine.js";

/** One loaded component instance, exactly as the engine passes see it. */
export type DeviceComponent = SimCircuit["components"][number];

/**
 * Element-state maps a device may read and commit. Each getter returns the
 * engine's LIVE map for the currently loaded topology (load() replaces the
 * maps, so consumers must always go through the context, never retain one).
 * The set grows as cohorts migrate; only maps a migrated cohort actually
 * touches are exposed, to keep the public surface auditable.
 */
export interface DeviceStateMaps {
  /** compId -> internal ideal-C voltage (pin0 - pin1). */
  readonly caps: Map<string, number>;
  /** compId -> total terminal current (pin0 -> pin1). */
  readonly capCurrents: Map<string, number>;
  /** compId -> last accepted series-branch current (trap history, Wave A2). */
  readonly capsI: Map<string, number>;
  /** compId -> winding current (pin0 -> pin1). */
  readonly inds: Map<string, number>;
  /** compId -> last accepted winding terminal voltage (trap history). */
  readonly indsV: Map<string, number>;
  /** compId -> PTC resettable-fuse trip state machine. */
  readonly ptcs: Map<string, PtcState>;
  /** compId -> committed body temperature (degrees C). */
  readonly thermalTemps: Map<string, number>;
  /** compId -> runtime primary-cell chemistry/SoC state. */
  readonly batteries: Map<string, BatteryRuntimeState>;
  /** compId -> named committed state slots (IC timing, CC/CV regime folds). */
  readonly icState: Map<string, Record<string, number>>;
  /** compId -> committed 555 SR flip-flop output state. */
  readonly ne555s: Map<string, NE555EngineState>;
  /** compId -> EEPROM contents plus write-cycle/SDP protocol state. */
  readonly eeproms: Map<string, EepromState>;
  /** compId -> HD44780 DDRAM plus bus-decode protocol state. */
  readonly lcds: Map<string, Hd44780State>;
  /** compId -> MOSFET gate-capacitor BE history voltages (vgs/vgd). */
  readonly mosfetGates: Map<string, MosfetGateState>;
  /** compId -> relay coil current plus committed contact state. */
  readonly relays: Map<string, RelayEngineState>;
  /** compId -> dc_motor winding current plus committed rotor speed. */
  readonly motors: Map<string, MotorEngineState>;
  /** compId -> servo PWM decode state plus committed shaft angle. */
  readonly servos: Map<string, ServoEngineState>;
  /** compId -> stepper coil currents plus phase/position count. */
  readonly steppers: Map<string, StepperEngineState>;
  /** compId -> HC-SR04 TRIG/ECHO state machine and schedule. */
  readonly hcsr04: Map<string, Hcsr04EngineState>;
  /**
   * compId -> running MCU core. Read-only for devices in BOTH senses: the
   * map (boot/reset lifecycle is engine-owned) and the cores themselves —
   * device handlers may only call observer methods (getStepPinEvents,
   * pinDriveState), exactly what the old servo/hcsr04 blocks read.
   */
  readonly arduinos: ReadonlyMap<string, MicrocontrollerCore>;
  /**
   * compId -> exact catalog package thermal state. Read-only for devices
   * (LED stamps read the committed junction temperature); writes belong to
   * the engine's package-thermal pass, which owns snapshot/rollback for it.
   */
  readonly thermalDevices: ReadonlyMap<string, ThermalRuntimeState>;
}

/**
 * The facade device handlers use to reach engine internals. Every member is
 * a direct alias of the engine expression the pre-registry switch cases used
 * (decision 2 above): same reads, same floats, same side effects.
 *
 * TRUST BOUNDARY: mutating members that take a component/state id
 * (setElementCurrent, setDigitalState, setBjtPrevJunction,
 * recordAccumulatedStress, the live maps in `state`, ...) accept ARBITRARY
 * ids — the engine validates which PASS a sink is writable in, not which
 * component a write addresses, because the old switch bodies had the same
 * unscoped access and per-write validation would tax every hot-path commit.
 * A model must only write under its own component's ids; writing another
 * component's entries can latch failures on it or corrupt its committed
 * state. When this becomes the standalone engine's public plugin API, the
 * dispatch loop (which knows the component under dispatch) should enforce
 * this per-sink before untrusted models are ever loaded.
 */
export interface DeviceContext {
  // ── Topology and solution access ─────────────────────────────────────────
  /** MNA row for a pin's net; -1 for ground or an unmapped pin. */
  pinNode(compId: string, pinId: string): number;
  /** True when the pin's net reaches no other component via an explicit wire. */
  isOpenPin(compId: string, pinId: string): boolean;
  /**
   * Extra-variable (branch) row allocated for a key this model returned from
   * branchRows(), or one of the engine's own historical keys.
   */
  vsrcRow(key: string): number | undefined;
  /**
   * MNA row of an internal solver node this model declared via
   * internalNodes(); -1 when the name was never allocated (mirroring
   * pinNode's unmapped fallback). See DeviceModel.internalNodes for the
   * full contract these rows obey.
   */
  internalNode(compId: string, name: string): number;
  /** Voltage at a matrix row in a solution/guess vector (0 for ground). */
  vAt(x: Float64Array, row: number): number;
  /**
   * The linear system for the solve in flight, as its stamp-only surface:
   * add/addB (directly or through the elements.ts helpers). The narrowed
   * type is the guard — clear/capture/solve are not reachable from device
   * code because that lifecycle belongs to the engine (decision 6).
   */
  readonly mna: MnaStampSurface;

  // ── Time ─────────────────────────────────────────────────────────────────
  /** Simulation time (s) at the START of the step being solved/committed. */
  simTime(): number;

  // ── Catalog and parameter access ─────────────────────────────────────────
  /** Authored param if present, else the exact-identity catalog default. */
  modelParam(
    comp: DeviceComponent,
    key: string,
    fallback: number | string,
  ): number | string;
  /** Electrical specs of the resolved catalog part, if any. */
  electricalSpecs(comp: DeviceComponent): ElectricalSpecs | undefined;
  /** Full resolved catalog part, if any. */
  catalogPart(comp: DeviceComponent): PartDefinition | undefined;

  // ── Environment ──────────────────────────────────────────────────────────
  /** Ambient illuminance (lux) with the engine's legacy fallback chain. */
  envLux(comp: DeviceComponent): number;
  /** Ambient temperature (deg C) with the engine's legacy fallback chain. */
  envTempC(comp: DeviceComponent): number;
  /** Circuit ambient temperature (deg C), default 25. */
  ambientTempC(): number;
  /** Junction thermal voltage kT/q at circuit ambient. */
  junctionVt(): number;
  /** Temperature-adjusted junction forward voltage from a 25 C catalog value. */
  junctionVf(vfAt25C: number): number;

  // ── Integration and DC-analysis gates (Waves A2/A3) ──────────────────────
  /** Selected integration method. Gates trap HISTORY upkeep, not stamping. */
  integrationMethod(): "be" | "trap";
  /**
   * True when THIS solve stamps trapezoidal companions (trap mode and not
   * re-anchoring across a discontinuity). Stamp/commit handlers must use
   * this, never derive it from integrationMethod() alone.
   */
  useTrapThisSolve(): boolean;
  /** True inside a true-DC operating-point solve (caps open, inductors R). */
  dcSolveMode(): boolean;
  /** Source-stepping homotopy scale for INDEPENDENT source magnitudes. */
  independentSourceMagnitude(value: number): number;
  /** Waveform sample instant for the solve in flight (OP holds pin it). */
  waveformSampleTime(h: number): number;

  // ── Independent-source machinery (SOURCES cohort) ────────────────────────
  /**
   * Stamp an ideal independent voltage source on a component's first two
   * pins. Routes through the engine's static ideal-source incidence
   * (STATIC_IDEAL_SOURCE_KINDS): when the topology pattern already lives in
   * the captured static base, only the RHS is written per solve. That
   * machinery is engine-owned, so migrated source stamps must call this
   * instead of stamping the pattern themselves.
   */
  stampIndependentVoltageSource(
    comp: DeviceComponent,
    branch: number,
    voltage: number,
  ): void;
  /**
   * Chemistry-aware battery operating point (OCV, internal resistance) for a
   * battery_pack. Wraps the engine helper because it reads the per-load
   * battery model cache and serves the public telemetry readout too.
   */
  batteryOperatingPoint(comp: DeviceComponent): {
    state: BatteryRuntimeState;
    profile: BatteryPhysicsProfile;
    nominalVoltageScale: number;
    referenceInternalResistanceOhm: number | undefined;
    point: BatteryOperatingPoint;
  };
  /**
   * Monotonic per-solve current-limit active set (stamp side). The backing
   * sets are engine-owned solve state shared across every regulated-source
   * device model (bench_psu/linear_reg/lm317/dcdc_converter); they are
   * cleared once per Newton solve, never by devices.
   */
  useCurrentLimitEntryClamp(
    key: string,
    branchCurrentGuess: number,
    currentLimit: number,
  ): boolean;
  useCurrentLimitComplianceClamp(
    key: string,
    outputMagnitudeGuess: number,
    complianceMagnitude: number,
  ): boolean;
  /** Read-only views of the same active sets for post-solve regime commits. */
  currentLimitEntryClampActive(key: string): boolean;
  currentLimitComplianceClampActive(key: string): boolean;

  // ── Junction limiting and readouts (SEMICONDUCTORS cohort) ───────────────
  /**
   * Per-load pnjlim precomputation for a BJT: temperature-scaled Is, the
   * per-junction n*Vt, and SPICE critical voltages. The cache itself is
   * rebuilt by the engine on every load()/coldLoad() (params change only at
   * the load boundary); undefined falls back to the handler's inline
   * recompute, exactly as the old switch case did.
   */
  bjtJunctionCache(compId: string): {
    IsT: number;
    VtNF: number;
    VtNR: number;
    vCritBE: number;
    vCritBC: number;
  } | undefined;
  /**
   * The junction voltages pnjlim accepted on the PREVIOUS Newton iteration
   * (engine _vPrevBJT). The engine clears the map at the start of every
   * solve so iteration 1 sees no entry and the limiter is a no-op there.
   */
  bjtPrevJunction(compId: string): { vBE: number; vBC: number } | undefined;
  setBjtPrevJunction(compId: string, prev: { vBE: number; vBC: number }): void;
  /**
   * Record that pnjlim clamped a junction in this stamp pass. The Newton
   * loop's acceptance guard (Wave A3) consults the underlying flag through
   * limitedThisIteration, so a damped iterate is never mistaken for a
   * converged one; the engine resets the flag before each _stampAll.
   */
  markJunctionLimitedThisIteration(): void;
  /**
   * Pre-commit MOSFET gate history for the element-current pass: Cgd
   * displacement current must difference against the gate voltages the
   * stamp used, which commitState has already overwritten by the time
   * element currents are reconstructed. Valid only inside an updateCurrent
   * handler (the engine scopes the snapshot to that pass).
   */
  previousMosfetGate(compId: string): MosfetGateState | undefined;
  /**
   * Per-channel diode currents for multi-junction packages, exactly as the
   * public getElementChannelI() telemetry readout computes them (pure over
   * the last committed solution and latched failures). Inside the failure
   * pass the readout is computed at most ONCE and shared across handlers —
   * the engine's old pass-local cache — so consult it only from
   * updateFailures hooks; elsewhere every call recomputes fresh values.
   */
  elementChannelCurrents(): Record<string, number[]>;

  // ── IC supply/output machinery (AMPS-REGULATORS cohort) ─────────────────
  /**
   * Package supply readout (rails, powered gate, logic thresholds, output
   * resistance) evaluated over a solution/guess vector. Engine-owned because
   * the unmigrated digital IC switch cases consume the same helper; devices
   * must pass the vector their pass is iterating on (xGuess when stamping,
   * the accepted x when committing), exactly as the old cases did.
   */
  icPowerInfo(comp: DeviceComponent, x: Float64Array): IcPowerInfo;
  /**
   * Stamp one finite-strength digital/open-collector output stage referenced
   * to the package rails (falls back to a local Norton target only for
   * supply-less behavioural parts). Engine-owned for the same shared-consumer
   * reason as icPowerInfo.
   */
  stampDigitalOutput(
    comp: DeviceComponent,
    pinId: string,
    high: boolean,
    power: IcPowerInfo,
  ): void;
  /**
   * Monotonic per-solve REG/DROPOUT/OFF headroom selection for regulated
   * sources. The backing per-solve sets are engine solve state cleared once
   * per Newton solve alongside the current-limit active sets above; devices
   * never mutate them directly.
   */
  regulatorHeadroomRegime(
    key: string,
    acceptedRegime: number,
    headroomGuess: number,
    regulatedVoltage: number,
    dropoutVoltage: number,
  ): 0 | 1 | 3;

  // ── Digital IC machinery (DIGITAL-ICS cohort) ────────────────────────────
  /**
   * Sampled logic level of a pin against the package thresholds. Open pins
   * resolve through the engine's disclosed seeded floating-input model.
   * Engine-owned: the unmigrated MCU/driver kinds and the generic
   * output-sag scan consume the same helper (and its seeded-noise clock).
   */
  logicHigh(
    comp: DeviceComponent,
    pinId: string,
    x: Float64Array,
    power: IcPowerInfo,
  ): boolean;
  /**
   * logicHigh, except pins listed in IC_OPEN_HIGH_PINS resolve
   * deterministically HIGH when unconnected (on-chip pull-ups).
   */
  logicHighH(
    comp: DeviceComponent,
    pinId: string,
    x: Float64Array,
    power: IcPowerInfo,
  ): boolean;
  /**
   * Schmitt-trigger pin read with per-pin hysteresis state persisted in
   * icState. commit=true (persist a level change) is only valid over a
   * converged post-solve x; stamps must pass false so intermediate Newton
   * iterates never latch hysteresis state.
   */
  schmittLogicHigh(
    comp: DeviceComponent,
    pinId: string,
    x: Float64Array,
    power: IcPowerInfo,
    commit: boolean,
  ): boolean;
  /** Stamp the engine's disclosed weak seeded pulls on open digital inputs. */
  stampFloatingDigitalInputs(
    comp: DeviceComponent,
    x: Float64Array,
    power: IcPowerInfo,
  ): void;
  /**
   * Cached delay-free combinational truth-table evaluation in ELECTRICAL
   * convention (true = HIGH). commit flows into the Schmitt pin reads (see
   * schmittLogicHigh); the signature cache is engine solve state.
   */
  combinationalOutputs(
    comp: DeviceComponent,
    xGuess: Float64Array,
    power: IcPowerInfo,
    commit?: boolean,
  ): Record<string, boolean>;
  /**
   * Currently VISIBLE (propagation-delayed) level for an output pin. The
   * delayed-output event scheduler itself — pending/due icState keys and
   * the due-time step limiter they feed — is engine-owned.
   */
  delayedOutputLevel(
    comp: DeviceComponent,
    pinId: string,
    immediate: boolean,
  ): boolean;
  /**
   * Advance the propagation-delay scheduler for one component and return
   * the visible output levels. Post-solve only (the digital pass); the
   * engine invalidates its next-due-time cache when schedules change.
   */
  updateDelayedOutputs(
    comp: DeviceComponent,
    immediate: Record<string, boolean>,
    h: number,
    power: IcPowerInfo,
  ): Record<string, boolean>;
  /**
   * Thevenin output referenced to explicit low/high rail rows via
   * complementary conductances (rail-current conserving). Engine-owned
   * because the unmigrated MCU board kinds stamp the same stage.
   */
  stampRailReferencedOutput(
    node: number,
    lowRail: number,
    highRail: number,
    fraction: number,
    outputResistance: number,
  ): void;
  /** 4-bit address from pins a0..a3 (shared with the output-sag scan). */
  readAddr4(
    comp: DeviceComponent,
    x: Float64Array,
    power?: IcPowerInfo,
  ): number;
  /**
   * Ordered sub-step MCU pin events for a display's protocol pins, with
   * the engine's lazily built shared Arduino-pin-driver map behind it
   * (first display in the pass builds it, later ones reuse it — exactly
   * the old `_dispDrivers ??=` behavior). Empty events = sampled fallback.
   */
  mergedDisplayEvents(
    comp: DeviceComponent,
    pinNames: string[],
  ): {
    arduinoCompId: string | null;
    events: Array<{ displayPin: string; level: 0 | 1 | null; cycle: number }>;
  };
  /**
   * Publish one key of the public digitalState map for the step being
   * committed. Valid only inside an updateDigital handler (the engine
   * scopes the sink to that pass); last write wins, matching `ds[key] =`.
   */
  setDigitalState(key: string, value: number): void;
  /** Publish a display decode result. Same pass scoping as setDigitalState. */
  setDisplayInfo(compId: string, info: DisplayInfo): void;

  // ── MCU edge timing and bridges (ELECTROMECH-AUDIO cohort) ───────────────
  /** Loaded component by id (engine _componentById), for driver lookups. */
  componentById(compId: string): DeviceComponent | undefined;
  /**
   * MCU board component + pin driving an MNA node, from the engine's lazy
   * includePico=true pin-driver map. The map is built at most ONCE per
   * state-commit pass by whichever handler (servo PWM decode, hcsr04 TRIG
   * edges) consults it first and reset at pass entry — exactly the shared
   * `mcuEventDrivers ??=` local the old _updateState blocks kept. It is a
   * SEPARATE cache from mergedDisplayEvents' uno/nano-only map.
   */
  mcuEventDriverForNode(node: number): { compId: string; pin: string } | undefined;
  /** Whether an MCU board is powered enough to run (engine _mcuPowered). */
  mcuPowered(comp: DeviceComponent, x: Float64Array): boolean;
  /**
   * simTime at which a board's core (re)booted — the origin that converts
   * a PinEvent's cumulative cycle count into absolute seconds.
   */
  mcuBootSimTime(compId: string): number | undefined;
  /**
   * micro:bit ECHO-bridge target for an hcsr04, if its ECHO pin shares a
   * net with a micro:bit edge pin. Engine-owned topology scan (this.nets
   * plus the MICROBIT_EDGE_PIN_TO_SIM table stay in the engine).
   */
  hcsr04MicrobitBridge(
    comp: DeviceComponent,
  ): { boardComponentId: string; pinId: string } | null;

  // ── Element state ────────────────────────────────────────────────────────
  readonly state: DeviceStateMaps;

  // ── Failures and readouts ────────────────────────────────────────────────
  /** True when the given latched failure exists for the component (pin). */
  hasFailure(compId: string, kind: SimFailureKind, pinId?: string): boolean;
  /**
   * Accumulate normalized stress toward a latched failure, exactly as the
   * engine's damage integrator does: positive stressRate integrates up,
   * otherwise recoveryRate bleeds down; crossing threshold latches
   * makeFailure() and clears the accumulator.
   */
  recordAccumulatedStress(
    kind: SimFailureKind,
    componentId: string,
    pinId: string,
    stressRate: number,
    recoveryRate: number,
    h: number,
    threshold: number,
    makeFailure: () => SimFailure,
  ): void;
  /** True when the given latched engine warning exists for the component (pin). */
  hasWarning(compId: string, code: SimWarningCode, pinId?: string): boolean;
  /**
   * recordAccumulatedStress's twin for advisories: the same dwell integrator,
   * latching makeWarning() once when threshold is crossed. A latched warning
   * never alters a stamp — it exists to be surfaced by the host (via
   * SimEngine.takeNewWarnings()) once per condition, not once per step.
   */
  recordAccumulatedWarning(
    code: SimWarningCode,
    componentId: string,
    pinId: string,
    stressRate: number,
    recoveryRate: number,
    h: number,
    threshold: number,
    makeWarning: () => SimWarning,
  ): void;
  /** Last committed element current (A, pin0 -> pin1) for a component. */
  elementCurrent(compId: string): number | undefined;
  /**
   * Publish this component's element current for the step being committed.
   * Valid only inside an updateCurrent handler (the engine scopes the sink
   * to that pass); last write wins, matching the old switch's `out[id] =`.
   */
  setElementCurrent(compId: string, amps: number): void;
}

/**
 * The catalog-only slice of DeviceContext that branchRows receives.
 * _buildMatrix consults the hook BEFORE the load rebuilds its topology
 * metadata (the matrix does not exist yet), so the full context's topology
 * accessors would silently serve the PREVIOUS load's rows there. Narrowing
 * the parameter makes the hook's purity contract (pure over comp/catalog)
 * structural instead of documentation-only.
 */
export type DeviceCatalogContext = Pick<
  DeviceContext,
  "modelParam" | "electricalSpecs" | "catalogPart"
>;

/**
 * The read-only facade acStamp handlers use to linearize about a committed
 * DC operating point (Wave A5 small-signal AC analysis). Deliberately much
 * smaller than DeviceContext: an AC stamp is a pure function of the held OP
 * solution, the committed regime/trip state, and catalog parameters — it
 * never advances device state, never sees Newton iterates, and never touches
 * the transient MNA. Every member is a thin alias of the exact engine
 * expression the corresponding transient stamp reads (mirroring registry
 * decision 2), so a small-signal stamp linearizes exactly the model its
 * large-signal stamp solved at that point.
 */
export interface AcDeviceContext {
  // ── Topology ─────────────────────────────────────────────────────────────
  /** MNA row for a pin's net; -1 for ground or an unmapped pin. */
  pinNode(compId: string, pinId: string): number;
  /** True when the pin's net reaches no other component via an explicit wire. */
  isOpenPin(compId: string, pinId: string): boolean;
  /** Extra-variable (branch) row for a branchRows() key. */
  vsrcRow(key: string): number | undefined;
  /** MNA row of an internalNodes() row (same alias as DeviceContext's);
   *  opVoltage() accepts it like any node row. */
  internalNode(compId: string, name: string): number;

  // ── Held operating-point solution ────────────────────────────────────────
  /** OP node voltage at a matrix row (0 for ground / branch rows). */
  opVoltage(row: number): number;
  /** OP voltage at a component pin — opVoltage(pinNode(compId, pinId)). */
  opPinVoltage(compId: string, pinId: string): number;

  // ── Catalog and parameter access (same aliases as DeviceContext) ────────
  modelParam(
    comp: DeviceComponent,
    key: string,
    fallback: number | string,
  ): number | string;
  electricalSpecs(comp: DeviceComponent): ElectricalSpecs | undefined;
  catalogPart(comp: DeviceComponent): PartDefinition | undefined;

  // ── Environment (same fallback chains as DeviceContext) ─────────────────
  envLux(comp: DeviceComponent): number;
  envTempC(comp: DeviceComponent): number;
  ambientTempC(): number;
  junctionVt(): number;
  junctionVf(vfAt25C: number): number;

  // ── Committed device state at the OP (read-only views) ──────────────────
  /**
   * The committed state maps small-signal stamps consult: trip/regime and
   * temperature state the OP commit left behind. Read-only on purpose — the
   * AC analysis must never mutate the engine it is linearizing.
   */
  readonly state: {
    readonly ptcs: ReadonlyMap<string, PtcState>;
    readonly thermalTemps: ReadonlyMap<string, number>;
    readonly icState: ReadonlyMap<string, Record<string, number>>;
    readonly ne555s: ReadonlyMap<string, NE555EngineState>;
    readonly eeproms: ReadonlyMap<string, EepromState>;
    readonly relays: ReadonlyMap<string, RelayEngineState>;
    readonly servos: ReadonlyMap<string, ServoEngineState>;
    readonly hcsr04: ReadonlyMap<string, Hcsr04EngineState>;
    readonly thermalDevices: ReadonlyMap<string, ThermalRuntimeState>;
  };
  /** True when the given latched failure exists for the component (pin). */
  hasFailure(compId: string, kind: SimFailureKind, pinId?: string): boolean;
  /** Chemistry-aware battery OP (OCV, internal resistance) — same helper as
   *  DeviceContext.batteryOperatingPoint, evaluated at the committed state. */
  batteryOperatingPoint(comp: DeviceComponent): {
    state: BatteryRuntimeState;
    profile: BatteryPhysicsProfile;
    nominalVoltageScale: number;
    referenceInternalResistanceOhm: number | undefined;
    point: BatteryOperatingPoint;
  };
  /** Package supply readout (rails, powered gate, output resistance)
   *  evaluated over the held OP solution vector. */
  icPowerInfoAtOp(comp: DeviceComponent): IcPowerInfo;

  // ── Committed logic reads over the held OP ───────────────────────────────
  // Digital-IC acStamps re-derive exactly the output-stage gating their
  // transient stamps computed from the final OP iterate (RCO enables, /OE
  // tristates, address decodes). These are the same _logicHigh family reads,
  // evaluated over the held OP solution — pure by construction (the floating
  // model is a hash of the frozen simTime bucket, and no Schmitt state is
  // committed on these paths).
  /** _logicHigh over the held OP solution (thresholds + floating model). */
  logicHighAtOp(comp: DeviceComponent, pinId: string, power: IcPowerInfo): boolean;
  /** _logicHighH over the held OP solution (IC_OPEN_HIGH_PINS aware). */
  logicHighHAtOp(comp: DeviceComponent, pinId: string, power: IcPowerInfo): boolean;
  /** _readAddr4 over the held OP solution (74LS189 address decode). */
  readAddr4AtOp(comp: DeviceComponent, power: IcPowerInfo): number;
  /** Engine simulated time — frozen for the whole analysis (the OP never
   *  advances it); protocol stamps (EEPROM write polling) read it. */
  simTime(): number;

  // ── AC drive designation ─────────────────────────────────────────────────
  /**
   * Unit AC drive magnitude for a source component: 1 for the analysis'
   * designated input, 0 for every other independent source. Sources multiply
   * this into their branch-row RHS (or Norton injection), which is exactly
   * the SPICE convention: every independent source is AC-zeroed except the
   * one under study, and reported magnitudes are gains relative to 1 V.
   */
  acInputMagnitude(compId: string): number;
}

/**
 * A registered device implementation for one or more component kinds.
 * Kinds sharing one model correspond exactly to switch cases that shared a
 * body in the engine (e.g. potentiometer/trimmer); a model is looked up per
 * component, so handlers may still branch on comp.kind internally where the
 * original shared body did.
 *
 * Hook-to-pass mapping (a hook is only consulted where stated):
 * - internalNodes: _buildMatrix, once per load, in component order, BEFORE
 *   branch-row allocation. Must be pure over comp (same names for the same
 *   component every load). See the hook's own comment for the row contract.
 * - branchRows: _buildMatrix, once per load, in component order. Return the
 *   vsrcIdx keys to allocate, in order; must be pure over comp/catalog (the
 *   matrix does not exist yet), which the DeviceCatalogContext parameter
 *   type enforces.
 * - staticStamp/staticSignature: membership in the captured static base and
 *   its staleness signature (see registry decision 4).
 * - stamp: every visit by _stampAll — per Newton iteration for dynamic
 *   kinds, base capture (and DC system extraction) for staticStamp kinds.
 * - commitState: _updateState, after a solve is accepted. The engine's
 *   DC-operating-point physical-time hold is applied BEFORE dispatch.
 * - updateDigital: _updateDigitalState, after commitState. The engine skips
 *   the whole pass while digital state is held (homotopy rung solves), so
 *   handlers never see overlay voltages.
 * - updateCurrent: _updateElementI, after updateDigital. Runs only for
 *   components with at least 2 pins (engine preamble rule).
 * - updateFailures: _updateFailureStates (skipped during OP physical-time
 *   hold). The engine's generic supply-range/output-sag scan still runs
 *   after this hook for every component with declared vcc_range specs.
 * - acStamp: runSmallSignalAc (ac-analysis.ts) only, once per component per
 *   frequency point, in component order — never consulted by any transient
 *   pass, so its presence cannot change transient bucket membership or
 *   float order.
 */
export interface DeviceModel {
  /** Component kinds this model implements. */
  kinds: readonly string[];
  /** Participates in the captured static MNA base (never re-stamped per solve). */
  staticStamp?: boolean;
  /**
   * Names of internal SOLVER nodes to allocate in _buildMatrix, in
   * allocation order (Wave A6). An internal node is an extra MNA node row a
   * multi-branch model needs between its own branches (a crystal's motional
   * midpoint, an optocoupler's phototransistor base) without inventing a
   * net for it. Contract:
   *
   * - Rows are allocated AFTER every net node and BEFORE every branch row,
   *   keyed `${comp.id}:${name}`, and reached via ctx.internalNode(). For
   *   circuits without internal-node kinds the allocation is empty, so all
   *   existing row indices — and therefore every solve — are bit-identical.
   * - They are inside [0, nodeCount): they receive the NODE_RSHUNT_G shunt
   *   (transient static base, gmin homotopy overlay, and the AC system's
   *   seed loop all iterate node rows), are readable through ctx.vAt /
   *   opVoltage, and participate in the dense/sparse solve like any node.
   * - They are NOT nets: netV/getNetV(), probes, and tick payloads are
   *   built by iterating the net index only, so an internal node can never
   *   surface in a public voltage readout.
   * - They carry no committed state of their own. The engine warm-starts
   *   them from the previous solution vector when sizes match (net rows
   *   warm-start from netV, which cannot carry non-nets); any history a
   *   model needs must live in the element-state maps as usual.
   * - Must be pure over comp: same names for the same component, every
   *   load, or the row keying goes stale.
   */
  internalNodes?(comp: DeviceComponent): readonly string[];
  /** Branch-row keys to allocate in _buildMatrix, in allocation order. */
  branchRows?(comp: DeviceComponent, ctx: DeviceCatalogContext): readonly string[];
  /**
   * Push the values (beyond component id/kind, which the engine always
   * includes) that must be equal for the captured static base to remain
   * valid onto `out`, in a fixed order. Out-parameter on purpose: the
   * signature scan runs once per step over every static component, and the
   * old switch pushed into the engine's accumulator with zero per-component
   * allocations — returning a fresh array here measurably regressed that
   * hot path (Wave A4 perf review).
   */
  staticSignature?(ctx: DeviceContext, comp: DeviceComponent, out: string[]): void;
  /** Stamp the component into ctx.mna, linearised about xGuess. */
  stamp?(
    ctx: DeviceContext,
    comp: DeviceComponent,
    xGuess: Float64Array,
    h: number,
  ): void;
  /** Commit accepted-step element/device state from the solution x. */
  commitState?(
    ctx: DeviceContext,
    comp: DeviceComponent,
    x: Float64Array,
    h: number,
  ): void;
  /**
   * Commit digital/protocol device state and publish digitalState/display
   * entries via ctx.setDigitalState / ctx.setDisplayInfo for the accepted
   * step. Presence adds the kind to the digital-update bucket.
   */
  updateDigital?(
    ctx: DeviceContext,
    comp: DeviceComponent,
    x: Float64Array,
    h: number,
  ): void;
  /**
   * Publish the component's element current via ctx.setElementCurrent.
   * `h` is the accepted step interval — the engine's element-current pass
   * has always carried it (waveform sources measure against the same
   * sample instant they stamped); handlers that don't need it omit it.
   */
  updateCurrent?(
    ctx: DeviceContext,
    comp: DeviceComponent,
    x: Float64Array,
    h: number,
  ): void;
  /** Integrate damage/trip state and latch failures for the accepted step. */
  updateFailures?(
    ctx: DeviceContext,
    comp: DeviceComponent,
    x: Float64Array,
    h: number,
  ): void;
  /**
   * Stamp the component's small-signal (linearized) contribution about the
   * held DC operating point into the AC system at angular frequency `omega`
   * (rad/s, always > 0). The stamp must be the derivative of the transient
   * stamp at the committed OP: conductances/Jacobian entries keep their
   * values, energy-storage elements become complex admittances, independent
   * sources are AC-zeroed except the designated input
   * (ctx.acInputMagnitude), and committed regimes/latches select the same
   * piecewise-linear region the transient stamp would use — never re-decide
   * a regime from AC quantities. Kinds without this hook contribute no
   * small-signal admittance (documented high-Z default); the one generic
   * exception is the combinational-IC cohort, whose committed `delay:` slot
   * output stages are covered once by the driver's Thevenin default (see
   * ac-analysis.ts). Every kind whose transient stamp writes conductances
   * from any OTHER state encoding must carry its own hook — the acStamp
   * coverage census pins that partition.
   */
  acStamp?(
    ctx: AcDeviceContext,
    comp: DeviceComponent,
    ac: AcStampSurface,
    omega: number,
  ): void;
}

const modelsByKind = new Map<string, DeviceModel>();
const registrationOrder: string[] = [];

/**
 * Register a device model for every kind it declares. Throws on duplicate
 * kinds and on staticStamp-without-staticSignature (decisions 3 and 4).
 */
export function registerDeviceModel(model: DeviceModel): void {
  if (model.kinds.length === 0) {
    throw new Error("DeviceModel must declare at least one kind");
  }
  if (model.staticStamp === true && !model.staticSignature) {
    throw new Error(
      `DeviceModel for "${model.kinds[0]}" declares staticStamp without a ` +
        "staticSignature; the captured static base could go stale undetected",
    );
  }
  for (const kind of model.kinds) {
    if (modelsByKind.has(kind)) {
      throw new Error(`Device kind "${kind}" is already registered`);
    }
  }
  for (const kind of model.kinds) {
    modelsByKind.set(kind, model);
    registrationOrder.push(kind);
  }
}

/** The model registered for a kind, or undefined for engine-switch kinds. */
export function getDeviceModel(kind: string): DeviceModel | undefined {
  return modelsByKind.get(kind);
}

/** Registered kinds in registration order (deterministic by decision 3). */
export function listRegisteredKinds(): string[] {
  return [...registrationOrder];
}
