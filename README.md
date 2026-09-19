# Connect4

Connect Four for the GameCube, built with [Octave-libogc](https://github.com/myuu-151/Octave-libogc).

## Layout

| Path | |
|---|---|
| `Connect4/Source/Board.*` | Rules and AI. No engine dependency, so it compiles and self-tests on PC |
| `Connect4/Source/Connect4Game.*` | Turn flow, input, and the hooks presentation attaches to |
| `Connect4/Source/Connect4Scene.*` | The board in the world: cell positions, the drop, the rerack |
| `Connect4/Assets` | Meshes, materials, textures, scenes |
| `Connect4/Scripts` | Lua — currently the sky |
| `external/` | Source downloads for the interiors, before conversion |
| `docs/` | Notes |

## Credits

See [CREDITS.md](CREDITS.md). The apartment interior is CC-BY-4.0 and its attribution must
appear in the game itself, not only here.
