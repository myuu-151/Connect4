# Credits

## Board

### "Connect 4" by Solvern

> This work is based on ["Connect 4"](https://sketchfab.com/3d-models/connect-4-90bc2c11ed694147be30413c3ac05de7)
> by [Solvern](https://sketchfab.com/Solvern) licensed under
> [CC-BY-4.0](http://creativecommons.org/licenses/by/4.0/)

Licence: **CC-BY-4.0** — attribution required, commercial use allowed, no NoDerivatives clause.

Parts: `MainFrame` (5,306 tris), `Chip_Red` (160), `Stack_Red` (1,120), `ReleaseTray` (68).
A full board is roughly 12k triangles including 42 chips.

Note on textures: `YellowChips` ships at 2048x2048 while `RedChips` is 256x256, for the same
object. That is 64x the texture area for a disc seen at a couple of centimetres, and 16 MB as
RGBA8 against Flipper's 1 MB texture cache. Downscale yellow to match red before cooking.

## Environments

### House — "Modern apartment interior" by Katydid

> This work is based on ["Modern apartment interior"](https://sketchfab.com/3d-models/modern-apartment-interior-400c9069181a4342a7142433dfa3466e)
> by [Katydid](https://sketchfab.com/Katydid.) licensed under
> [CC-BY-4.0](http://creativecommons.org/licenses/by/4.0/)

Licence: **CC-BY-4.0** — attribution required, commercial use allowed, and no NoDerivatives
clause, so the decimated and recooked GameCube version is permitted. The credit line above is
the one the licence asks to be reproduced verbatim wherever the work is shared.

### Subway — by yulaflibiskuvi

Author profile: https://sketchfab.com/yulaflibiskuvi

**Licence unverified.** The download in `external/metrosubway-station-interior/` contains no
`license.txt` — only `source/station.blend` and `textures/`. Before this ships anywhere public,
the model page needs checking for:

- [ ] Exact model title and URL
- [ ] Licence type, and whether attribution is required
- [ ] Whether it carries **NoDerivatives** — decimating and retexturing a scan for GX is
      squarely a derivative work, so ND would rule the model out entirely
- [ ] Whether it carries **NonCommercial**, which constrains how the ISO can be distributed

Until that is confirmed, treat the subway as unusable in any public build.

## Engine

Built with [Octave-libogc](https://github.com/myuu-151/Octave-libogc), a GameCube/Wii fork of
[Octave](https://github.com/mholtkamp/octave) by mholtkamp.

Sky domes from [OctaveSimpleSkies](https://github.com/myuu-151/OctaveSimpleSkies).

---

## Where the credit has to appear

CC-BY requires attribution to accompany **the work itself**, not only its source repository. So
the apartment credit needs to be visible in the game — a credits screen, or at minimum a line
on the title screen — as well as here and in any release notes. The FNAF port's release-notes
format is the precedent for the repository side.
