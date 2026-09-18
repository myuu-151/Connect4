#pragma once

#include <stdint.h>

#include "Board.h"
#include "Connect4Scene.h"

// Connect Four for the GameCube.
//
// Structure: Board.h holds the rules and the AI and knows nothing about the engine; this class
// is the glue -- turn order, input, timing, and the hooks the presentation layer hangs off.
//
// The board is authoritative and physics is decoration. A disc's column is decided by the rules
// before anything is simulated, so a bouncing disc can never change where it lands. Bullet gets
// two jobs, both purely visual:
//
//   the fall    -- a disc dropped into a column, clattering off the rim and settling
//   the rerack  -- the release pulled at the end, all 42 discs tumbling out of the bottom
//
// Neither can affect the outcome, which means both can be tuned entirely for feel.

class Connect4Game
{
public:

    enum class State : uint8_t
    {
        Playing,        // waiting for a move
        Dropping,       // a disc is falling; input is ignored until it settles
        GameOver,       // result shown, winning line highlighted
        Reracking,      // the release is open and the discs are falling out
    };

    enum class Mode : uint8_t
    {
        TwoPlayer,
        VersusAI,
    };

    Connect4Game();
    ~Connect4Game();

    bool Initialize();
    void Update(float deltaTime);

    State GetState() const { return mState; }
    const C4::Board& GetBoard() const { return mBoard; }
    C4::Cell GetTurn() const { return mTurn; }
    int GetCursorColumn() const { return mCursorCol; }

    void SetMode(Mode mode) { mMode = mode; }
    void SetAIDifficulty(const C4::AI::Difficulty& diff) { mDifficulty = &diff; }

    void NewGame();
    void StartRerack();

private:

    void UpdatePlaying(float deltaTime);
    void UpdateDropping(float deltaTime);
    void UpdateGameOver(float deltaTime);
    void UpdateRerack(float deltaTime);

    bool TryDrop(int col);
    void MoveCursor(int delta);
    bool IsAITurn() const;

    // --- presentation hooks -------------------------------------------------
    // Deliberately empty for now. They are where the models, sounds and rigid bodies attach,
    // so the rules and turn flow can be finished and tested before any art exists.
    void OnCursorMoved(int col);
    void OnDiscDropped(const C4::Move& move);
    void OnDiscLanded(const C4::Move& move);
    void OnGameEnded(C4::Result result);
    void OnRerackStarted();

    Connect4Scene mScene;

    C4::Board mBoard;
    State mState = State::Playing;
    Mode mMode = Mode::VersusAI;
    C4::Cell mTurn = C4::Cell::Red;

    const C4::AI::Difficulty* mDifficulty = &C4::AI::kNormal;
    uint32_t mRandState = 0x1234567u;

    int mCursorCol = C4::kCols / 2;
    C4::Move mPendingMove;

    float mStateTime = 0.0f;
    float mDropTimer = 0.0f;
    float mAIThinkTimer = 0.0f;
    float mRepeatTimer = 0.0f;

    bool mLoggedFirstFrame = false;
};
