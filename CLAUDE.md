# Claude Agent Workflow

## Git workflow

- Never proactively run `git commit` or `git push`. After making changes,
  point out that this is a good point to commit — state briefly why —
  and propose a concise commit message. The user runs the commit
  themselves.
- Commit message suggestions must be in English.
- When work is risky, exploratory, or unrelated to the current main
  line, suggest creating a new branch — state briefly why — and propose
  a concise branch name. Only run `git branch` after the user agrees.
- Branch names must be in English.

## Documentation principle

Single source of truth: every fact is maintained in exactly one place.
Don't restate project facts (mission, architecture, status) here — this
file is operating conventions only.

## Where to find things

See `README.md` — it has the project overview and directory structure.
Don't duplicate that list here.

## Working mode

- Human: system design, architecture, technical decisions.
- AI: implementation, algorithms, documentation — always against an
  agreed spec in `docs/spec/`, not ad hoc.

## Language

Respond in Chinese or English only.
