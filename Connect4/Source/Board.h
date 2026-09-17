#pragma once

#include <stdint.h>

// Connect Four rules and AI.
//
// Deliberately free of any engine dependency: no Octave headers, no rendering, no input. The
// rules are the part worth getting exactly right, and keeping them standalone means they can be
// compiled and exercised on a PC without booting the game. Board.cpp has a self-test entry
// point (RunSelfTest) for that.
//
// The board is authoritative. Physics animates a disc falling into a slot the rules already
// chose, never the other way round -- a simulated disc must not be able to decide the outcome.

namespace C4
{

static const int kCols = 7;
static const int kRows = 6;
static const int kConnect = 4;

enum class Cell : uint8_t
{
    Empty = 0,
    Red   = 1,
    Blue  = 2,
};

inline Cell Other(Cell p) { return (p == Cell::Red) ? Cell::Blue : Cell::Red; }

enum class Result : uint8_t
{
    Playing,
    RedWins,
    BlueWins,
    Draw,
};

// A single dropped disc, as the presentation layer needs it.
struct Move
{
    int mCol = -1;
    int mRow = -1;          // 0 is the bottom row
    Cell mPlayer = Cell::Empty;
    bool Valid() const { return mCol >= 0; }
};

class Board
{
public:

    void Reset();

    Cell Get(int col, int row) const;

    // Lowest empty row in a column, or -1 if the column is full.
    int NextRow(int col) const;
    bool CanDrop(int col) const { return NextRow(col) >= 0; }

    // Places a disc and returns where it landed. The returned Move is invalid if the column was
    // full, so callers can reject the input without a separate check.
    Move Drop(int col, Cell player);

    // Takes back the last disc in a column. Used by the search, and by an undo button.
    void Undo(int col);

    Result GetResult() const { return mResult; }
    int GetMoveCount() const { return mMoveCount; }
    bool IsFull() const { return mMoveCount >= kCols * kRows; }

    // The four cells that won, valid only once GetResult() is a win. The presentation layer
    // uses these to highlight the winning line.
    const Move* GetWinningLine() const { return mWinLine; }

private:

    bool CheckWinFrom(int col, int row, Cell player);
    void RecomputeResult();

    Cell mCells[kCols][kRows] = {};
    int mHeight[kCols] = {};        // discs in each column
    int mMoveCount = 0;
    Result mResult = Result::Playing;
    Move mWinLine[kConnect] = {};
};

// Negamax with alpha-beta pruning.
//
// The search is cheap enough to run inline on the Gekko: the branching factor is at most 7 and
// a full board is 42 plies, so even a naive implementation searches deeply in a few
// milliseconds. Depth is exposed so difficulty can be a menu option rather than a rebuild.
class AI
{
public:

    struct Difficulty
    {
        int mDepth;
        int mBlunderPercent;    // chance of ignoring the best move, for the easier settings
        const char* mName;
    };

    static const Difficulty kEasy;
    static const Difficulty kNormal;
    static const Difficulty kHard;

    // Returns the column to play, or -1 if the board is full.
    static int ChooseMove(const Board& board, Cell player, const Difficulty& diff,
                          uint32_t& randState);

    // Exposed for tests: the raw search, no blunder chance.
    static int BestMove(const Board& board, Cell player, int depth, int& outScore);
};

// Compiles and runs without the engine. Returns the number of failures.
int RunSelfTest();

}   // namespace C4
