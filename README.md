# Connect4

Connect Four for the GameCube, built with [Octave-libogc](https://github.com/myuu-151/Octave-libogc).

A small, finishable game in a deliberately expensive room. The rules cost almost nothing, which
is the point: with the gameplay essentially free, the whole frame budget goes into the scene —
a photogrammetry interior, a procedural sky, and physics used purely for feel.

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

## Loading screen

The screen itself belongs to the engine. What is on it comes from here, in two different ways,
because the two are needed at different times.

The **logo** is named in `Connect4/Config.ini`:

    LoadingScreenLogo=T_Connect4Logo

It has to be config rather than a call from game code: the engine puts the loading screen up
during its own startup asset load, which finishes before any of this project's code runs. The
texture is `Assets/UI/T_Connect4Logo.png`, padded to 512x256 -- GX addresses textures in powers
of two, so the original 636x201 cannot be uploaded at all. Only the height is padded, so the logo
still spans the full width and keeps its proportions.

The **message** is passed from code, in `Connect4Scene.cpp`:

    renderer->DrawLoadingFrame(progress, "Setting up the board...");

The engine says "Loading..." during its own startup load. By the time the board is being built
this project's code is running and can say what it is actually doing, so it overrides the text
per call. Nothing project-specific lives in the engine: a game that sets neither gets a plain
screen with the default message.

## Building

Open `Connect4/Connect4.octp` in the Octave editor, or build the GameCube target directly:

```sh
cd Connect4
make -f Makefile_GCN          # -> Build/GCN/Connect4.dol
```

The packager produces the ISO:

```sh
Octave.exe -headless -project <path>/Connect4.octp -build GameCube
```

## Design notes

**The board is authoritative; physics is decoration.** A disc's column is decided by the rules
before anything is simulated, so a bouncing disc can never change where it lands. That frees
Bullet to be tuned purely for feel — the fall and the clatter, and the *rerack*, where the
release is pulled and all 42 discs tumble out.

**The rules self-test on boot.** `Board.cpp` carries its own tests (all four win directions,
stacking, full-column rejection, undo, and the AI both taking and blocking a win) and the game
refuses to start if any fail. It costs well under a millisecond and the rules are the one part
that must never be subtly wrong.

**Difficulty blunders rather than searching shallowly.** A shallow search still never misses an
immediate win, which reads as unbeatable-but-stupid; an occasional random legal move feels like
a beatable opponent. Easy blunders 40% of the time, Normal 10%, Hard never.

**There is no point spending CPU on a deeper search.** Depth 8 already beats Easy 19–1, and
Connect Four is solved — push far enough and the AI simply never loses, which is worse to play
against. The Gekko is better spent on the room.

## Credits

See [CREDITS.md](CREDITS.md). The apartment interior is CC-BY-4.0 and its attribution must
appear in the game itself, not only here.
