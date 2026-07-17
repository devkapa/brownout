# Releasing brownout

The owner's runbook. Steady state is one action — **merge the "Version
Packages" PR** — and everything else here exists to get to that steady state
once, or to explain what the automation is doing when it misbehaves.

Every external claim on this page was checked against the vendor's own docs on
2026-07-17. The ones that could not be checked are collected under
[What is unverified](#what-is-unverified) rather than papered over, because a
runbook that guesses is worse than one that admits a gap.

## The pieces

| Piece | Where | Does what |
| --- | --- | --- |
| `.github/workflows/ci.yml` | on PRs + pushes to main | typecheck, build, both test lanes, the ngspice cross-validation, `check:package`, examples, on Node 20.18.3 (the `engines` floor, pinned exactly) and 22 |
| `.github/workflows/release.yml` | on pushes to main | re-runs the same gate on Node 22, then opens/refreshes the version PR, or publishes once that PR merges |
| `.changeset/` | repo root | pending release notes + the version bump they imply |
| `pnpm run release` | `package.json` | `changeset publish` — the command the workflow runs |
| npm trusted publisher | npmjs.com, configured by hand | authorizes `release.yml` to publish without a token |

There is no `NPM_TOKEN` secret in this repository, and there should never be
one. The release job authenticates with a short-lived OIDC token minted by
GitHub for that specific workflow run.

### What actually stops a bad release

Be precise about this, because the honest answer is shorter than it looks:

| Layer | Enforced? | Notes |
| --- | --- | --- |
| The gate inside `release.yml` | **yes** | typecheck, build, both test lanes, a proven ngspice crossval, `check:package` — all run *before* `changesets/action`. A red tree cannot reach the publish step. This is the one that actually holds. |
| `ci.yml` on a PR | advisory | it reports, it does not block — see branch protection below |
| Branch protection on `main` | **no — not configured** | `main` has no protection rule and no ruleset. A red PR merges; a direct push to main is allowed |
| An `environment:` gate on the release job | no — deliberately absent | see "Why the workflow looks the way it does" |

**`main` is unprotected today.** Verified, not assumed:
`gh api repos/devkapa/brownout/branches/main/protection` returns 404 "Branch
not protected", and `gh api repos/devkapa/brownout/rulesets` returns `[]`. So
nothing at the *merge* boundary is enforced — the enforcement is the gate
inside `release.yml`, which is why that gate exists rather than trusting
`ci.yml`'s green tick.

If you want the merge boundary enforced too — recommended, and it is the
missing half of the "approve workflows" step below — this is an owner action in
the repo settings, not something the workflows can do for themselves:

```bash
# Requires admin on the repo. Review before running: this changes repo settings.
gh api -X PUT repos/devkapa/brownout/branches/main/protection \
  --input - <<'JSON'
{
  "required_status_checks": { "strict": true, "contexts": ["build (22)", "build (20.18.3)"] },
  "enforce_admins": false,
  "required_pull_request_reviews": null,
  "restrictions": null
}
JSON
```

Check the context names against a real run first (`gh run view --json jobs`);
a required check whose name never appears will block every merge forever.

## Read this before the first publish

**npm cannot bootstrap trusted publishing.** A trusted publisher is configured
in a package's settings on npmjs.com, and a package has no settings page until
it exists — so the first publish of `brownout` cannot use OIDC. This is not a
misconfiguration to debug; it is [npm/cli#8544][8544], open at the time of
writing, whose text reads: "it's not possible to publish the initial version of
a package using OIDC, it needs to be published manually or using a token."
(PyPI solved this with pending publishers; npm has not.)

**There is no changeset for the initial release, and that is deliberate.** A
changeset describes a *bump*, so adding one for `0.1.0` would make
`changeset version` produce `0.1.1` (or `0.2.0`) and the version that has never
shipped would be skipped. `.changeset/` is therefore empty until the first
change *after* `0.1.0` lands. Note what is and is not a changesets limitation:
`changeset publish` publishes any package whose local version is not on the
registry, so it can and does publish `0.1.0` — Route A below leans on exactly
that. What changesets cannot do is bootstrap the npmjs.com side, and that is
the real gate on the first publish.

**Order matters, and getting it wrong makes a red release run.** The moment
`release.yml` lands on main with no pending changesets, the action runs
`pnpm run release`, `changeset publish` sees that the local `0.1.0` is not on
the registry, and it tries to publish. If npmjs.com does not yet trust this
workflow, that run fails. So do the npmjs.com side **before** merging the PR
that adds these workflows.

## First publish

Two routes out of the chicken-and-egg. Both need one temporary npm token; they
differ only in whether `0.1.0` itself gets a provenance attestation.

**Route A (recommended): placeholder first, so the first real release is
signed.** Publish a throwaway `0.0.0` by hand to bring the package into
existence, wire up trusted publishing, then let CI publish `0.1.0` with
provenance. Costs one junk version, which is deprecated at the end.

**Route B (simpler): publish `0.1.0` by hand.** One less step, but `0.1.0`
carries no provenance and never can — attestations are generated at publish
time. CI takes over from `0.2.0`.

Route A is written out below. For Route B, skip step 3's version edit, publish
the real `0.1.0` in step 4, and skip step 8.

### 1. Rehearse the package

Do not skip this. The clean-install rehearsal is the only thing that proves the
exports map, the type conditions, and the lazy `rp2040js` import survive
packaging. It is specified in
[`docs/packaging-verification.md`](packaging-verification.md) — follow that
page, do not improvise a substitute.

It is a gate, not a formality: the 2026-07-17 rehearsal came back with a
publish blocker (F1 — `engines.node: ">=20"` admits Node versions the package
cannot load on). That one is now fixed (the floor reads `>=20.18.3`, and
`ci.yml` pins a matrix leg to it), but **check that page's Findings section is
clear before publishing anything.** A wrong `engines` range is cheap to fix now
and permanent once a version is on the registry — npm versions are immutable,
so the fix would cost a whole release.

Then, from a fresh clone (not your working tree — `dist/` and `node_modules/`
in a dirty tree hide exactly the mistakes this catches):

```bash
git clone https://github.com/devkapa/brownout.git /tmp/brownout-release
cd /tmp/brownout-release
pnpm install --frozen-lockfile
pnpm run typecheck
pnpm run build
pnpm test
pnpm run test:sparse
pnpm run check:package
pnpm run examples
npm pack --dry-run     # read the file list: dist + docs + LICENSE + NOTICE + README
```

### 2. Take the npm account side

Log in as the account that will own `brownout` and confirm the name is still
free:

```bash
npm login
npm whoami
npm view brownout      # expect E404 — anything else means the name went
```

Enable 2FA on the account if it is not already on. It does not block anything
below; `npm publish` prompts for the OTP.

### 3. Set the placeholder version

In `/tmp/brownout-release` only — **never commit this**:

```bash
npm pkg set version=0.0.0
```

### 4. Publish by hand

```bash
npm publish --access public --dry-run   # last look at what goes up
npm publish --access public
```

`--access public` is required on a first publish. No `--provenance` here: npm
generates attestations only from a supported cloud CI provider, so passing it
from a laptop errors out. That is the whole reason Route A exists.

`prepack` rebuilds `dist/` as part of this, so the tarball has real code in it
even though `dist/` is gitignored.

### 5. Configure the trusted publisher

On npmjs.com, go to the package's settings and find the "Trusted Publisher"
section (`https://www.npmjs.com/package/brownout/access`), choose **GitHub
Actions**, and fill in:

| Field | Value |
| --- | --- |
| Organization or user | `devkapa` |
| Repository | `brownout` |
| Workflow filename | `release.yml` |
| Environment | leave empty |

Every field is case-sensitive and must match exactly. Leave Environment empty
unless you also add an `environment:` key to the release job — the two have to
agree, and a mismatch reads as an authorization failure, not a config error.

### 6. Delete the temporary token

Revoke the token or session from step 2 (`npmjs.com` → Access Tokens). From
here on the workflow's OIDC token is the only thing that can publish, which is
the point of the exercise.

### 7. Merge the workflows PR

`release.yml` runs on the push to main, runs the full gate (typecheck, build,
both test lanes, the ngspice crossval, `check:package`), finds no changesets,
runs `pnpm run release`, and publishes `0.1.0` — this time from CI, with
provenance. If the gate is red, nothing publishes; fix forward and push again.

Confirm it landed:

```bash
npm view brownout version           # 0.1.0
npm view brownout dist-tags
```

Then open `https://www.npmjs.com/package/brownout` and check for the
provenance badge linking back to the `release.yml` run. No badge means the
publish succeeded but the attestation did not. npm attests by default under
trusted publishing, so the first suspect is that the publish never took the
OIDC path — check the job log for "using npm trusted publishing", then that the
job had `id-token: write`, then that the repo is public (npm does not attest
from private repos). `NPM_CONFIG_PROVENANCE` is the last thing to suspect, not
the first: it is redundant with npm's default here.

### 8. Deprecate the placeholder

```bash
npm deprecate brownout@0.0.0 "Placeholder to enable npm trusted publishing. Use 0.1.0 or later."
```

## Every release after the first

This is the steady state, and the only part that should ever feel routine.

**1. A change lands with a changeset.** Whoever writes a user-visible change
runs `pnpm changeset` on their branch, picks patch/minor/major, writes a line
of prose aimed at a consumer reading the CHANGELOG, and commits the generated
`.changeset/*.md` with the code. While the package is 0.x, "major" means the
0.x → 0.y bump: a breaking change to the device registry is a minor bump today,
not a major one. That is the versioning posture 0.x buys, and the README says
so out loud.

**2. `release.yml` opens a "Version Packages" PR.** It consumes the changeset
files, bumps `version` in `package.json`, and writes `CHANGELOG.md`. It
refreshes that PR as more changesets land.

**CI does not start on that PR by itself — you have to click "Approve workflows
to run".** This is not a misconfiguration. The PR is authored by
`GITHUB_TOKEN`, and per GitHub's docs, "when a workflow using `GITHUB_TOKEN`
creates or updates a pull request, the resulting `pull_request` event creates
workflow runs in an **approval-required** state"; they start only when "a user
with write access… selecting **Approve workflows to run**". So on the one PR
that constitutes the release, the checks sit idle until a human starts them.
Approve them, wait for green, then merge. (The alternative is to author the PR
with a PAT or GitHub App token, which trades a human step for a long-lived
secret — a bad trade for this repo, whose whole point is having no publish
secret.)

**3. The owner merges it.** That is the release. The next `release.yml` run
finds no changesets, re-runs the full gate, publishes the new version with
provenance, then pushes the git tag and cuts the GitHub release.

Nothing else is required, and in particular nothing local: do not
`npm publish` by hand once trusted publishing is live, because a local publish
produces an unattested version and breaks the chain the badge advertises.

### If a release run fails

| Symptom | Likely cause |
| --- | --- |
| `E404` on the `PUT` to the registry | First suspect: the trusted publisher fields do not match (org/repo/workflow filename are case-sensitive), or the workflow was renamed. Note there is also an open npm bug, [npm/cli#8976][8976], where OIDC publishes of *scoped* packages via `changesets/action` 404 with everything configured correctly. `brownout` is unscoped, so it should not be in scope for that one — but if the fields all check out, read the issue before assuming the config is wrong. |
| `ENEEDAUTH` / asks for a token | The OIDC exchange never happened: check `id-token: write` on the job, and that the runner npm is >= 11.5.1 (`npm -v` in the job). |
| Published, but no provenance badge | Do **not** start with `NPM_CONFIG_PROVENANCE` — under trusted publishing npm attests by default ("npm automatically generates and publishes provenance attestations… This happens by default—you don't need to add the `--provenance` flag"), so a missing badge usually means the publish did not go through the trusted-publishing path at all, or the repo/package is not public. Check in that order: was the publish actually OIDC (look for the "using npm trusted publishing" line in the job log), is the repo public, is the package public. The env var is belt-and-braces, not the switch. |
| Action opened a PR when you expected a publish | There were still pending `.changeset/*.md` files on main. |
| Action did nothing | No changesets and the version already exists on the registry — that is a no-op, and green is correct. |

## Why the workflow looks the way it does

Non-obvious constraints, each verified rather than assumed:

- **The publish gate lives inside `release.yml`, duplicating `ci.yml`.**
  `ci.yml` and `release.yml` both fire on `push: main` *independently*: neither
  waits for the other, and `prepack` only runs `build`. Without the gate,
  `release.yml` would publish anything that compiles — a commit failing every
  test would reach the registry, and an npm version is immutable once there.
  `needs:` cannot cross workflows. `workflow_run` *could* chain this after
  `ci.yml` (checking out `event.workflow_run.head_sha` to pin the tested
  commit) and would save the duplicate minutes — a fair alternative, deliberately
  not taken: it moves the gate one indirection away from the publish it guards,
  and its extra failure modes (wrong conclusion check, wrong ref) fail *open*,
  meaning they publish. Inline fails closed. The gate runs on Node 22 only (the
  version that publishes); `ci.yml` still owns the 20.18.3 floor leg. Yes, this
  burns CI minutes twice on every push to main — that is the cheap side of the
  trade, and it is the price of `main` being unprotected.
- **Node 22 and a hand-installed npm in `release.yml`.** npm's trusted
  publishing docs require npm >= 11.5.1 and Node >= 22.14.0. Node 22 ships
  npm 10.x, so the workflow runs `npm install -g npm@11` before publishing.
  The package still supports Node >= 20.18.3 for consumers; `ci.yml` proves
  that on the floor itself.
- **`npm@11`, not `npm@latest`.** `latest` is npm 12 today, and it is a live
  hazard in a job that holds a publish grant. npm 12's engines are
  `^22.22.2 || ^24.15.0 || >=26.0.0` — the `node-version: 22` above clears that
  floor by a single minor, on luck, and an npm 13 that raises the floor to Node
  24 would break the publish step with no warning. npm 12 also already broke the
  exact surface changesets uses: it wraps `npm info --json` output in an array,
  and this pipeline survives that only because the lockfile pins a
  `@changesets/cli` carrying the unwrap patch. That is a coincidence, not a
  design. npm 11's engines (`^20.17.0 || >=22.9.0`) leave real margin. Pinning
  the major still takes patches.
- **Actions pinned to commit SHAs, not tags.** The release job holds
  `id-token: write` and `contents: write`. A tag is mutable — whoever owns the
  action repo can repoint it at new code — so a tag pin extends trust forever to
  an upstream this repo does not control. The SHAs are the commits the tags
  pointed at when this was written; the trailing `# v7` comments are for humans.
  Note when re-resolving these: `pnpm/action-setup` and `changesets/action`
  publish *annotated* tags, so `git/ref/tags/<tag>` returns a tag-object SHA,
  **not** a commit SHA. Actions needs the commit. Dereference it
  (`gh api repos/<repo>/git/tags/<tag-object-sha> --jq .object.sha`) or you will
  pin something that cannot resolve.
- **`runs-on: ubuntu-24.04`, not `ubuntu-latest`.** `-latest` is a moving label:
  GitHub migrates it to a new LTS gradually over 1-2 months, and ubuntu-26.04 is
  already in public preview. Since the apt ngspice version rides on the image,
  `-latest` would silently change the binary the cross-validation runs against —
  which is precisely the drift the ngspice caveat below warns about. Pinning
  makes that caveat actionable: the version only moves when someone moves it.
- **pnpm pinned to 10.x.** pnpm's docs state that since v11 `pnpm publish` is
  implemented natively and no longer delegates to the npm CLI. On 10.x the npm
  installed above is what performs the OIDC exchange — a well-trodden path.
  On 11.x, pnpm itself becomes the trusted-publishing client, and pnpm 11.0.8
  shipped an OIDC-publish regression ([pnpm#11513][11513], since closed).
  Bumping pnpm here means re-verifying a publish end to end.
- **`NPM_CONFIG_PROVENANCE: true` in the workflow, not `publishConfig`.**
  `changeset publish` shells out to `pnpm publish` with only
  `--access`/`--tag`/`--no-git-checks`; it never passes `--provenance`. Setting
  it in `package.json` `publishConfig` would work in CI but would also fire on
  a local publish — including the first-publish steps above — where npm has no
  CI to attest to and errors out.
- **`changesets/action` pinned to `v1.9.0`'s SHA exactly.** There is no moving
  `v1` tag on that repo, and `v2.0.0-next.*` renames the inputs (`publish` →
  `publish-script`). v1.9.0 is the version whose source handles OIDC: it
  creates an `.npmrc` only when `NPM_TOKEN` is set, and otherwise logs "No
  NPM_TOKEN found, but OIDC is available - using npm trusted publishing".
- **No `environment:` on the release job, and Environment left empty on
  npmjs.com.** These two must agree, so they are one decision. An environment
  with required reviewers would be a genuine human checkpoint before an
  irreversible publish, and npm's trusted publisher config supports binding to
  one ("Environment name (optional): If using GitHub environments for deployment
  protection"). It is deliberately not wired up yet for one reason: the gate
  inside `release.yml` already blocks a red publish, and an environment adds a
  second place the config can silently disagree with npmjs.com — a mismatch
  reads as an authorization failure, not a config error, and the first publish
  has never been run. **If you want it, it is a two-sided change: create the
  environment with required reviewers in repo settings, add `environment:` to
  the release job, and set the matching Environment name on npmjs.com. Do all
  three or none.**
- **`cancel-in-progress: false` on the release concurrency group.** npm
  publishes are not idempotent; a cancelled run mid-publish is worse than a
  queued one.
- **The `repository` field is left as the `github:devkapa/brownout`
  shorthand.** npm's own normalizer expands it to
  `{"type":"git","url":"git+https://github.com/devkapa/brownout.git"}` before
  the manifest reaches the registry, which is the form provenance wants. This
  was checked by running npm's bundled `@npmcli/package-json` `prepare()`
  against this manifest, not assumed.

## What is unverified

Stated plainly so nobody mistakes reasoning for evidence:

- **The publish itself has never been executed.** Everything about the
  registry's behavior here is from npm's docs and from reading the tools'
  source, not from a completed publish. The first run is the experiment.
- ~~**`NPM_CONFIG_PROVENANCE` reaching npm through pnpm 10's delegation.**~~
  **Now verified** by reading the shipped tools' source: pnpm 10's `runNpm`
  calls `runScriptSync` with `{...createEnv(opts), ...opts.env}`, `createEnv`
  spreads `process.env`, and `getEnvWithTokens` returns `{}` when no
  `:tokenHelper` is configured, so nothing can clobber the variable on the way
  through. It reaches npm. (Which matters less than it looks — npm attests by
  default under trusted publishing regardless; see the badge row in the failure
  table.)
- **npm's trusted-publisher UI** is described from npm's docs; the actual
  settings page was not opened (it needs an authenticated session). Field
  labels may read slightly differently.
- **npm's docs do not themselves state the first-publish limitation.** The
  claim rests on [npm/cli#8544][8544] being open, plus corroborating
  third-party write-ups. If npm ships pending-publisher support, Route A's
  placeholder becomes unnecessary — re-check the issue before following this
  page.
- **The Node 20.18.3 matrix leg runs only the quick-start against `dist/`
  locally, not the suite** (this machine is on Node 25). The floor itself is
  measured — 20.9.0 SyntaxErrors, 20.10.0 and 20.18.2 warn, 20.18.3 is clean;
  see `docs/packaging-verification.md`. But "the whole test corpus passes on
  20.18.3" is a claim `engines` makes and CI is what tests it.
- **CI's ngspice is not the ngspice the envelopes were measured against.**
  Ubuntu 24.04 universe carries ngspice 42; the cross-validation numbers in the
  README were measured against Homebrew's ngspice 46. If a case breaches an
  envelope only in CI, that is a real signal about a version difference —
  investigate it, do not weaken the envelope. Both workflows now pin
  `ubuntu-24.04` rather than `ubuntu-latest` specifically so this stays a known
  difference instead of a moving one: on `-latest` the image (and with it the
  apt ngspice) would migrate on GitHub's schedule, changing what the envelopes
  are tested against with no signal at all.

[8544]: https://github.com/npm/cli/issues/8544
[8976]: https://github.com/npm/cli/issues/8976
[11513]: https://github.com/pnpm/pnpm/issues/11513
