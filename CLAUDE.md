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

Cross-references ("详见 X" / "see Y") exist to prevent two places from
drifting out of sync — use them only where that risk is real: durable
facts substantial enough to actually get duplicated and go stale.
Don't add one by reflex for every fact that lives elsewhere, and never
for minor or transient details (a one-off note, a passing rationale,
something nobody will need to find again) — just state it once, in
whichever place a reader would naturally look, and leave it at that.
When in doubt, prefer no pointer over one more pointer.

## Where to find things

See `README.md` — it has the project overview and directory structure.
Don't duplicate that list here.

## Code comments on variables/constants

Put the comment on the same line as the variable, after the value —
not wrapped across multiple lines. If a comment genuinely needs more
than a short trailing note, put it on the line(s) above the variable
instead of wrapping the trailing comment — but prefer trimming the
comment down to fit on one line over doing this.

```python
_APPROACH_TOLERANCE_M = 0.15  # 精细对准的水平容差
```

## Working mode

- Human: system design, architecture, technical decisions.
- AI: implementation, algorithms, documentation — always against an
  agreed spec in `docs/spec/`, not ad hoc.

## Language

Respond in Chinese or English only.
