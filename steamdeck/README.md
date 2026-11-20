# Steam Deck

```text
steamdeck/
  real/
    v4   # the deploy Go module: realtime-core, olp, camera, pendant commands
    v5   # the native teach pendant (Qt), built and installed by `pendant`
```

[`real/v5`](real/v5/README.md) is the Deck's teach pendant. It runs OLP's own
program logic, so the pendant and the desktop cannot disagree; build and
install it with `go run . pendant build|install` from `real/v4`
([`real/v4/README.md`](real/v4/README.md), Tier 4, separate owner approval).

[`real/v4`](real/v4/README.md) is the deployment CLI for every cell. Its old
native pendant (`real/v4/steamdeck`), the browser replica (`virtual/`) and the
earlier `real/v1`-`v3` demos were removed before the open-source release; a
Deck that still has the v4 pendant installed keeps it as its rollback, and
`git show 5654b33d:<path>` reads the removed source.
