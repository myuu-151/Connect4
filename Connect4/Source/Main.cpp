#include <stdint.h>

#undef min
#undef max

#include "Engine.h"
#include "Log.h"
#include "Renderer.h"

#include "Connect4Game.h"

// Octave's packager generates these; embedded builds fill them.
#if __has_include("../Generated/EmbeddedAssets.h")
#include "../Generated/EmbeddedAssets.h"
#include "../Generated/EmbeddedScripts.h"
#define CONNECT4_HAS_GENERATED 1
#else
#define CONNECT4_HAS_GENERATED 0
#endif

// SD diagnostic log (Octave System_Dolphin.cpp; writes /octiso.log when the local logger is
// enabled). The FNAF port leaned on this heavily for anything that only misbehaves on hardware.
void OctLog(const char* format, ...);

static Connect4Game* sGame = nullptr;

void OctPreInitialize(EngineConfig& config)
{
    GetEngineState()->mStandalone = true;

    // The project name comes from Config.ini, as in any packaged Octave game.

    if (config.mWindowWidth == 0)
        config.mWindowWidth = 640;

    if (config.mWindowHeight == 0)
        config.mWindowHeight = 480;

#if CONNECT4_HAS_GENERATED
    config.mEmbeddedAssetCount = gNumEmbeddedAssets;
    config.mEmbeddedAssets = gEmbeddedAssets;
    config.mEmbeddedScriptCount = gNumEmbeddedScripts;
    config.mEmbeddedScripts = gEmbeddedScripts;
    config.mEmbeddedConfig = gEmbeddedConfig_Data;
    config.mEmbeddedConfigSize = gEmbeddedConfig_Size;
#endif
}

void OctPostInitialize()
{
    // No log lines drawn over the game. The engine's console widget is created whenever
    // CONSOLE_ENABLED is set and is only hidden for Windows, Linux and Android release builds,
    // so on the GameCube it sits in the top-left corner printing everything that goes through
    // LogX. Hiding the widget stops it being drawn and nothing else.
    if (Renderer::Get() != nullptr)
    {
        Renderer::Get()->EnableConsole(false);
    }

    // Full resolution. This is a board game: a handful of discs and a static camera, so it is
    // nowhere near pixel-bound and there is nothing to buy by rendering smaller. Half
    // resolution made sense for a level streaming past at speed; here it would only soften the
    // board for free.

    OctLog("Connect4: engine initialized, screen %dx%d",
           GetEngineState()->mWindowWidth, GetEngineState()->mWindowHeight);

    sGame = new Connect4Game();

    if (!sGame->Initialize())
    {
        OctLog("Connect4: game initialization FAILED");
        LogError("Connect4: initialization failed");
    }
    else
    {
        OctLog("Connect4: game initialized");
    }
}

void OctPreUpdate()
{

}

void OctPostUpdate()
{
    if (sGame != nullptr)
    {
        sGame->Update(GetEngineState()->mGameDeltaTime);
    }
}

void OctPreShutdown()
{
    delete sGame;
    sGame = nullptr;
}

void OctPostShutdown()
{

}
