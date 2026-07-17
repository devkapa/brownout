# Contributing to brownout

Thanks for looking. Brownout is a simulation engine, so the bar here is a bit
different from most TypeScript projects: the tests do not just check that the
code runs, they pin what the physics *answers*. Most of this page is about
that.

`pnpm install`, then:

```bash
pnpm run typecheck
pnpm run build
pnpm test
pnpm run check:package
```

`README.md` (Development) covers the rest of the scripts, the source layout,
and why the examples are typechecked separately.

## The two test lanes

The corpus runs twice per change, once per linear-solver backend:

```bash
pnpm test               # dense backend (the default)
pnpm run test:sparse    # SIMCORE_LINEAR_BACKEND=sparse — the same corpus, forced sparse
```

Both must be green. The sparse lane is not redundant: backend selection is by
size, and sparse only engages automatically above 64 unknowns, so the default
run never touches sparse on the small fixtures that make up most of the
corpus. Forcing it is how the two backends are held to the same answers.

If you have `ngspice` installed, `pnpm test` also runs the cross-validation
suite against it; without a binary that suite skips itself. CI installs ngspice
and asserts the suite actually ran, so "it passed locally" and "it passed in
CI" are not the same statement — CI checks strictly more.

## The bit-identity contract

**A change that is supposed to be a refactor must not move a single bit of the
solved trajectory.** Performance work, restructuring, extraction, cleanup: all
of it must leave the deterministic baselines byte-for-byte identical. Those
baselines exist precisely so that "I only made it faster" is a claim that can
be checked instead of believed.

Two consequences worth knowing before a test failure surprises you:

- The **dense** backend is the bit-exact reference. Where the sparse lane
  legitimately differs — pivot order changes the last couple of ULPs — the
  assertion is backend-aware: dense stays exact, sparse asserts within solver
  roundoff. Do not "fix" a dense bit-exact assertion by loosening it to match
  sparse.
- Adding a device, an integration method, or an analysis is allowed to change
  *nothing* about existing circuits' answers unless the change is opt-in.
  Trapezoidal integration shipped this way: it is behind
  `setIntegrationMethod`, and backward Euler was proven bit-identical before
  and after.

## No numeric assertion changes without evidence

If a test asserts a number and your change makes that number wrong, **the
default assumption is that your change is wrong.**

Editing the expected value, widening a tolerance, or relaxing an envelope is a
last resort that has to be argued, in the diff, with a mechanism. A PR that
moves a number is reviewed as a physics claim, not a test fix. What an
acceptable argument looks like:

- **What the number is now, what it was, and why the new one is more correct** —
  an analytic derivation, a datasheet figure, a reference-simulator comparison.
  "The old value was measured from the old code" is not evidence of anything
  except the old code.
- **Which mechanism moved it**, named specifically: a pivot-order rounding
  difference, an integration-order change, a model constant with a documented
  vintage. If you cannot name the mechanism, you have found a bug, not a new
  baseline.
- **Why the envelope still describes reality.** Envelopes in
  `docs/physics-reference-benchmarks.md` and the ngspice harness are disclosed
  validity contracts, not tuning targets. Never widen one to make a failing
  case pass. A breach is either a mapping bug (fix it) or a legitimate model
  difference (document it in the case comment, with the mechanism).

The same rule covers deleting an inconvenient test. If a test is genuinely
wrong, say so in the PR and explain how it was ever green.

## License provenance: never port GPL code

**Do not copy, port, translate, or "adapt" code from Falstad's `circuitjs`, or
from any other GPL-licensed simulator, into this repository.** Not the device
models, not the stamps, not the numerical tricks. Brownout is MIT, and GPL code
laundered through a rewrite is still GPL code — it is invisible in a diff and
it silently contaminates every downstream consumer who chose this package
*because* it is MIT.

This is not a general suspicion of prior art. Published math is fine and is
what the engine is built from: MNA, Newton-Raphson with `pnjlim`,
Gilbert-Peierls, AMD ordering, and homotopy continuation are decades-old
literature, and citing a paper or a textbook in a comment is encouraged. The
line is between an algorithm from the literature and source code from a
copyleft project.

If you are unsure whether a reference you read counts, say where it came from
in the PR. That question is cheap to answer up front and expensive to answer
after a merge.

Third-party material that *is* vendored is recorded in `NOTICE`, and its
notices must survive into the published tarball. If you add any, update
`NOTICE` in the same PR.

## Sign your commits off

Brownout uses the [Developer Certificate of Origin](https://developercertificate.org/)
— one line, no CLA:

```bash
git commit -s -m "your message"
```

`-s` appends `Signed-off-by: Your Name <your@email>`, which is you asserting
you have the right to contribute that code under MIT. See the license
provenance rule above for what that actually means.

## Changesets

If your change is user-visible — new behavior, a fix, a breaking change, a
notable perf characteristic — add a changeset:

```bash
pnpm changeset
```

Pick the bump, write a line aimed at someone reading the CHANGELOG to decide
whether to upgrade, and commit the generated `.changeset/*.md` alongside your
code. While the package is 0.x, a breaking change is a *minor* bump.

Internal-only changes (tests, refactors, docs, CI) do not need one. Releases
are the owner's action: see [`docs/releasing.md`](docs/releasing.md).

## Style

- Comments explain **why**, not what. The interesting comments in this codebase
  are the ones recording a constraint, a convention, or a boundary someone
  would otherwise re-litigate.
- Document boundaries in the module header that owns them. Those headers are
  the authority the docs are compiled from.
- No emojis in code or docs.
