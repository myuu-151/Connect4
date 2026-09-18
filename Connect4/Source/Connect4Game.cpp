#include "Connect4Game.h"

#include "InputDevices.h"
#include "Log.h"

void OctLog(const char* format, ...);

namespace
{

const int32_t kPad = 0;             // player 1's controller

// Cursor repeat: one step immediately, then a steady rate while held. Without this the cursor
// either moves once per press (tedious across seven columns) or sprints (unusable).
const float kRepeatDelay = 0.32f;
const float kRepeatRate = 0.11f;

const float kStickDeadzone = 0.5f;

// How long a disc takes to fall before the rules-side move is considered settled. Once the
// physics disc exists this becomes "when the rigid body sleeps" instead of a timer.
const float kDropDuration = 0.55f;

// Upper bound on a drop, used when the scene is driving the timing. If an animation somehow never
// reports finishing, the turn still advances instead of the game sitting in Dropping for good.
const float kDropTimeout = 3.0f;

// A beat before the AI plays, so it does not answer instantly. Reads as thinking.
const float kAIThinkTime = 0.45f;

const char* ResultName(C4::Result r)
{
    switch (r)
    {
    case C4::Result::RedWins:  return "Red wins";
    case C4::Result::YellowWins: return "Yellow wins";
    case C4::Result::Draw:     return "Draw";
    default:                   return "Playing";
    }
}

}   // anonymous namespace

Connect4Game::Connect4Game()
{
}

Connect4Game::~Connect4Game()
{
}

bool Connect4Game::Initialize()
{
    // BISECT STEP 4: the self-test RUNS again, but its result is ignored -- no LogError, no
    // early return. A failing test only logs and continues, so it should never have caused a
    // black screen; this separates "RunSelfTest itself crashes" from "the test fails on
    // PowerPC and the failure path is what crashes".
    const int failures = C4::RunSelfTest();
    if (failures != 0)
    {
        // The rules are the one part that must never be subtly wrong, so they are checked on
        // every boot. It costs well under a millisecond, and it passes on PowerPC -- verified
        // on hardware, not just on the PC build.
        LogError("Connect4: rules self-test FAILED (%d)", failures);
        return false;
    }

    // The scene is optional on purpose. If the board models are missing or renamed, the rules and
    // turn flow still run and still log, which is what makes a headless or half-dressed scene
    // debuggable instead of a black screen.
    if (!mScene.Initialize())
    {
        LogWarning("Connect4: scene not available; running rules only.");
    }

    NewGame();
    OctLog("Connect4: initialized, rules self-test passed");
    return true;
}

void Connect4Game::NewGame()
{
    mBoard.Reset();
    mState = State::Playing;
    mTurn = C4::Cell::Red;
    mCursorCol = C4::kCols / 2;
    mPendingMove = C4::Move();
    mStateTime = 0.0f;
    mDropTimer = 0.0f;
    mAIThinkTimer = 0.0f;
    mRepeatTimer = 0.0f;

    mScene.ClearDiscs();
    mScene.SetCursorColumn(mCursorCol);
    mScene.ShowCursorDisc(mTurn);

    OnCursorMoved(mCursorCol);
}

void Connect4Game::Update(float deltaTime)
{
    mStateTime += deltaTime;

    if (!mLoggedFirstFrame)
    {
        mLoggedFirstFrame = true;
        OctLog("Connect4: first frame");
    }

    // Nothing until the scene has finished warming the physics up.
    //
    // Every disc in the pool is in use while that runs, so a move made during it found no disc to
    // show: the board recorded it and nothing appeared, leaving an invisible counter in the slot
    // and the two out of step for the rest of the game.
    if (mScene.IsWarmingUp())
    {
        mScene.Update(deltaTime);
        return;
    }

    switch (mState)
    {
    case State::Playing:   UpdatePlaying(deltaTime);   break;
    case State::Dropping:  UpdateDropping(deltaTime);  break;
    case State::GameOver:  UpdateGameOver(deltaTime);  break;
    case State::Reracking: UpdateRerack(deltaTime);    break;
    }

    // After the state update, so a disc released this frame starts moving on the same frame rather
    // than a frame late.
    mScene.Update(deltaTime);
}

bool Connect4Game::IsAITurn() const
{
    return mMode == Mode::VersusAI && mTurn == C4::Cell::Yellow;
}

void Connect4Game::UpdatePlaying(float deltaTime)
{
    if (IsAITurn())
    {
        mAIThinkTimer += deltaTime;
        if (mAIThinkTimer >= kAIThinkTime)
        {
            mAIThinkTimer = 0.0f;
            const int col = C4::AI::ChooseMove(mBoard, mTurn, *mDifficulty, mRandState);
            if (col >= 0)
            {
                mCursorCol = col;
                OnCursorMoved(mCursorCol);
                TryDrop(col);
            }
        }
        return;
    }

    // --- cursor ---------------------------------------------------------
    int dir = 0;
    const float stickX = GetGamepadAxisValue(GAMEPAD_AXIS_LTHUMB_X, kPad);

    if (IsGamepadButtonJustDown(GAMEPAD_LEFT, kPad))
        dir = -1;
    else if (IsGamepadButtonJustDown(GAMEPAD_RIGHT, kPad))
        dir = 1;

    const bool holdingLeft = IsGamepadButtonDown(GAMEPAD_LEFT, kPad) || stickX < -kStickDeadzone;
    const bool holdingRight = IsGamepadButtonDown(GAMEPAD_RIGHT, kPad) || stickX > kStickDeadzone;

    if (dir == 0 && (holdingLeft || holdingRight))
    {
        mRepeatTimer -= deltaTime;
        if (mRepeatTimer <= 0.0f)
        {
            dir = holdingLeft ? -1 : 1;
            mRepeatTimer = kRepeatRate;
        }
    }
    else if (dir != 0)
    {
        mRepeatTimer = kRepeatDelay;
    }
    else if (!holdingLeft && !holdingRight)
    {
        mRepeatTimer = 0.0f;
    }

    if (dir != 0)
        MoveCursor(dir);

    // --- drop -----------------------------------------------------------
    if (IsGamepadButtonJustDown(GAMEPAD_A, kPad))
    {
        if (!TryDrop(mCursorCol))
        {
            // Full column. The presentation layer can buzz here.
            OctLog("Connect4: column %d is full", mCursorCol);
        }
    }
}

void Connect4Game::MoveCursor(int delta)
{
    int col = mCursorCol;

    // Skip full columns so the cursor never rests somewhere a disc cannot go.
    for (int i = 0; i < C4::kCols; ++i)
    {
        col += delta;
        if (col < 0) col = C4::kCols - 1;
        if (col >= C4::kCols) col = 0;

        if (mBoard.CanDrop(col))
            break;
    }

    if (col != mCursorCol)
    {
        mCursorCol = col;
        OnCursorMoved(mCursorCol);
    }
}

bool Connect4Game::TryDrop(int col)
{
    const C4::Move move = mBoard.Drop(col, mTurn);
    if (!move.Valid())
        return false;

    mPendingMove = move;
    mState = State::Dropping;
    mDropTimer = 0.0f;
    mStateTime = 0.0f;

    OnDiscDropped(move);
    return true;
}

void Connect4Game::UpdateDropping(float deltaTime)
{
    mDropTimer += deltaTime;

    // Wait for the disc to actually land when there is one to watch, so the turn changes exactly
    // when the board looks settled. The timer is the fallback for a scene that never loaded, and
    // doubles as a backstop so a missed animation cannot wedge the game in Dropping forever.
    if (mScene.IsReady())
    {
        if (mScene.IsDropAnimating() && mDropTimer < kDropTimeout)
            return;
    }
    else if (mDropTimer < kDropDuration)
    {
        return;
    }

    OnDiscLanded(mPendingMove);

    const C4::Result result = mBoard.GetResult();
    if (result != C4::Result::Playing)
    {
        mState = State::GameOver;
        mStateTime = 0.0f;
        OnGameEnded(result);
        OctLog("Connect4: game over -- %s in %d moves",
               ResultName(result), mBoard.GetMoveCount());
        return;
    }

    mTurn = C4::Other(mTurn);
    mState = State::Playing;

    // Keep the cursor on a column that can still take a disc.
    if (!mBoard.CanDrop(mCursorCol))
        MoveCursor(1);

    // The next player's disc appears above the board, ready to be moved and dropped.
    mScene.SetCursorColumn(mCursorCol);
    mScene.ShowCursorDisc(mTurn);
}

void Connect4Game::UpdateGameOver(float deltaTime)
{
    (void)deltaTime;

    if (IsGamepadButtonJustDown(GAMEPAD_A, kPad) ||
        IsGamepadButtonJustDown(GAMEPAD_START, kPad))
    {
        StartRerack();
    }
}

void Connect4Game::StartRerack()
{
    mState = State::Reracking;
    mStateTime = 0.0f;
    OnRerackStarted();
    OctLog("Connect4: rerack");
}

void Connect4Game::UpdateRerack(float deltaTime)
{
    (void)deltaTime;

    const float kRerackDuration = 2.0f;

    // The scene's rerack is a sequence -- the grid lifts, the tray is pulled, the discs fall and
    // are left lying on the table for a moment -- so it runs well past the rules-only timer. This
    // backstop is only there to stop a stuck animation holding the game here forever, and has to
    // sit above the longest that sequence can legitimately take.
    const float kRerackTimeout = 9.0f;

    // Wait for the discs to finish falling out when the scene is driving it; the timer covers the
    // rules-only case and stops a stuck animation from holding the game here.
    if (mScene.IsReady())
    {
        if (mScene.IsRerackAnimating() && mStateTime < kRerackTimeout)
            return;
    }
    else if (mStateTime < kRerackDuration)
    {
        return;
    }

    // Then hold, with the discs where they fell and the grid still open, until the button is
    // pressed. The simulation is the best thing to look at in the game and it used to be cleared
    // away a moment after it finished; there is nothing to be gained by hurrying it, and the next
    // game starts on a press either way.
    if (!IsGamepadButtonJustDown(GAMEPAD_A, kPad) &&
        !IsGamepadButtonJustDown(GAMEPAD_START, kPad))
    {
        return;
    }

    NewGame();
}

// ---------------------------------------------------------------------------
// Presentation hooks
//
// Empty by design: the turn flow above is complete and testable without any of them. Each one
// is where a model, a sound or a rigid body attaches.
// ---------------------------------------------------------------------------

void Connect4Game::OnCursorMoved(int col)
{
    // Slide the waiting disc to the entry point above the chosen column.
    mScene.SetCursorColumn(col);

    // TODO: tick sound
}

void Connect4Game::OnDiscDropped(const C4::Move& move)
{
    // The move is already decided by the rules, so the fall only has to look right: it starts at
    // the column's entry point and ends on that cell's still point.
    mScene.BeginDrop(move, mTurn);
    mScene.PlayDropSound();
}

void Connect4Game::OnDiscLanded(const C4::Move& move)
{
    (void)move;

    // The disc is snapped onto its still point by the scene as the animation ends, so the stack
    // cannot drift over a long game.
    mScene.PlayLandSound();
}

void Connect4Game::OnGameEnded(C4::Result result)
{
    if (result == C4::Result::Draw)
    {
        return;
    }

    mScene.HighlightWin(mBoard.GetWinningLine());
}

void Connect4Game::OnRerackStarted()
{
    mScene.BeginRerack();
}
