# Contributing to the research website

The repository is public with the owner's explicit approval. Work in a feature
branch or a fork and submit a pull request. The owner reviews accepted changes
before updating the existing website. No contributor is granted website
publishing credentials by this repository.

## Validate a change

```text
node tests/run-tests.cjs
node tests/viewer-harness.cjs
node scripts/build.mjs
node tests/validate-build.cjs
```

The workflow in `.github/workflows/validate.yml` runs these checks on pushes and
pull requests. It uploads the static build as an artifact; it does not deploy.
Actual branch-protection and required-review settings are separate from this
document and the Code Owners file.

Keep the interface and scientific labels in English. Explain the plant equation,
controller, assumptions, units, and reproducible seed when changing a simulation.
Preserve recorded physical cost values instead of plotting a monotone envelope.
Native input effort has model-specific reference units, not common physical
energy units across all robot types.

Edit `src/` and `docs/`. Generated `dist/`, full `research/` recordings, local
`test-results/`, and credentials are not source inputs. Preserve the exact Site
identity in `.openai/hosting.json`. Do not add another Site for routine updates.

The requested collaborator is `sheepcy-cxy`. A repository invitation, accepted
membership, and direct write permission must be checked separately. Public
fork/pull-request contribution does not require direct repository write access.
