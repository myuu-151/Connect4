#include "Board.h"

namespace C4
{

// ---------------------------------------------------------------------------
// Board
// ---------------------------------------------------------------------------

void Board::Reset()
{
    for (int c = 0; c < kCols; ++c)
    {
        mHeight[c] = 0;
        for (int r = 0; r < kRows; ++r)
            mCells[c][r] = Cell::Empty;
    }

    mMoveCount = 0;
    mResult = Result::Playing;

    for (int i = 0; i < kConnect; ++i)
        mWinLine[i] = Move();
}

Cell Board::Get(int col, int row) const
{
    if (col < 0 || col >= kCols || row < 0 || row >= kRows)
        return Cell::Empty;

    return mCells[col][row];
}

int Board::NextRow(int col) const
{
    if (col < 0 || col >= kCols)
        return -1;

    return (mHeight[col] < kRows) ? mHeight[col] : -1;
}

Move Board::Drop(int col, Cell player)
{
    Move move;

    if (player == Cell::Empty || mResult != Result::Playing)
        return move;

    const int row = NextRow(col);
    if (row < 0)
        return move;

    mCells[col][row] = player;
    mHeight[col] += 1;
    mMoveCount += 1;

    move.mCol = col;
    move.mRow = row;
    move.mPlayer = player;

    // Only the cell just placed can complete a line, so the check starts there and walks
    // outward in four directions -- far cheaper than rescanning the whole board.
    if (CheckWinFrom(col, row, player))
    {
        mResult = (player == Cell::Red) ? Result::RedWins : Result::YellowWins;
    }
    else if (IsFull())
    {
        mResult = Result::Draw;
    }

    return move;
}

void Board::Undo(int col)
{
    if (col < 0 || col >= kCols || mHeight[col] <= 0)
        return;

    mHeight[col] -= 1;
    mCells[col][mHeight[col]] = Cell::Empty;
    mMoveCount -= 1;
    mResult = Result::Playing;

    for (int i = 0; i < kConnect; ++i)
        mWinLine[i] = Move();
}

bool Board::CheckWinFrom(int col, int row, Cell player)
{
    // horizontal, vertical, and the two diagonals
    static const int kDirs[4][2] = { {1, 0}, {0, 1}, {1, 1}, {1, -1} };

    for (int d = 0; d < 4; ++d)
    {
        const int dc = kDirs[d][0];
        const int dr = kDirs[d][1];

        int count = 1;

        // walk both ways from the placed disc
        for (int sign = -1; sign <= 1; sign += 2)
        {
            int c = col + dc * sign;
            int r = row + dr * sign;

            while (Get(c, r) == player)
            {
                count += 1;
                c += dc * sign;
                r += dr * sign;
            }
        }

        if (count >= kConnect)
        {
            // Walk back to the start of the run so the highlight covers the actual four.
            int c = col;
            int r = row;
            while (Get(c - dc, r - dr) == player)
            {
                c -= dc;
                r -= dr;
            }

            for (int i = 0; i < kConnect; ++i)
            {
                mWinLine[i].mCol = c + dc * i;
                mWinLine[i].mRow = r + dr * i;
                mWinLine[i].mPlayer = player;
            }

            return true;
        }
    }

    return false;
}

// ---------------------------------------------------------------------------
// AI
// ---------------------------------------------------------------------------

const AI::Difficulty AI::kEasy   = { 2, 40, "Easy" };
const AI::Difficulty AI::kNormal = { 5, 10, "Normal" };
const AI::Difficulty AI::kHard   = { 8,  0, "Hard" };

namespace
{

const int kWinScore = 1000000;

// Centre columns are worth more: a disc there belongs to more possible lines. Searching them
// first also prunes far harder, which is most of why the search stays cheap.
const int kColOrder[kCols] = { 3, 2, 4, 1, 5, 0, 6 };
const int kColValue[kCols] = { 1, 2, 3, 4, 3, 2, 1 };

int ScoreWindow(int mine, int theirs, int empty)
{
    if (mine > 0 && theirs > 0)
        return 0;                       // blocked, worth nothing to either side

    if (mine == 4)   return 10000;
    if (mine == 3 && empty == 1) return 60;
    if (mine == 2 && empty == 2) return 8;

    if (theirs == 4) return -10000;
    if (theirs == 3 && empty == 1) return -80;   // blocking is valued slightly above building
    if (theirs == 2 && empty == 2) return -8;

    return 0;
}

int Evaluate(const Board& board, Cell player)
{
    const Cell opp = Other(player);
    int score = 0;

    for (int c = 0; c < kCols; ++c)
    {
        for (int r = 0; r < kRows; ++r)
        {
            if (board.Get(c, r) == player)
                score += kColValue[c];
        }
    }

    static const int kDirs[4][2] = { {1, 0}, {0, 1}, {1, 1}, {1, -1} };

    for (int c = 0; c < kCols; ++c)
    {
        for (int r = 0; r < kRows; ++r)
        {
            for (int d = 0; d < 4; ++d)
            {
                const int dc = kDirs[d][0];
                const int dr = kDirs[d][1];

                const int endC = c + dc * (kConnect - 1);
                const int endR = r + dr * (kConnect - 1);
                if (endC < 0 || endC >= kCols || endR < 0 || endR >= kRows)
                    continue;

                int mine = 0, theirs = 0, empty = 0;
                for (int i = 0; i < kConnect; ++i)
                {
                    const Cell cell = board.Get(c + dc * i, r + dr * i);
                    if (cell == player)     mine += 1;
                    else if (cell == opp)   theirs += 1;
                    else                    empty += 1;
                }

                score += ScoreWindow(mine, theirs, empty);
            }
        }
    }

    return score;
}

int Negamax(Board& board, Cell player, int depth, int alpha, int beta)
{
    const Result res = board.GetResult();
    if (res != Result::Playing)
    {
        if (res == Result::Draw)
            return 0;

        const bool playerWon = (res == Result::RedWins && player == Cell::Red) ||
                               (res == Result::YellowWins && player == Cell::Yellow);

        // Prefer winning sooner and losing later: without the depth term the search is happy to
        // postpone a forced win indefinitely, which looks like the AI toying with you.
        return playerWon ? (kWinScore + depth) : -(kWinScore + depth);
    }

    if (depth <= 0)
        return Evaluate(board, player);

    int best = -kWinScore * 2;

    for (int i = 0; i < kCols; ++i)
    {
        const int col = kColOrder[i];
        if (!board.CanDrop(col))
            continue;

        board.Drop(col, player);
        const int score = -Negamax(board, Other(player), depth - 1, -beta, -alpha);
        board.Undo(col);

        if (score > best)
            best = score;

        if (best > alpha)
            alpha = best;

        if (alpha >= beta)
            break;                      // opponent would avoid this line
    }

    return best;
}

uint32_t NextRand(uint32_t& state)
{
    // xorshift32; deterministic, and identical on PC and GameCube, which matters when a bug
    // only shows up in one AI line.
    state ^= state << 13;
    state ^= state >> 17;
    state ^= state << 5;
    return state;
}

}   // anonymous namespace

int AI::BestMove(const Board& board, Cell player, int depth, int& outScore)
{
    Board work = board;

    int bestCol = -1;
    int bestScore = -kWinScore * 2;
    int alpha = -kWinScore * 2;
    const int beta = kWinScore * 2;

    for (int i = 0; i < kCols; ++i)
    {
        const int col = kColOrder[i];
        if (!work.CanDrop(col))
            continue;

        work.Drop(col, player);
        const int score = -Negamax(work, Other(player), depth - 1, -beta, -alpha);
        work.Undo(col);

        if (bestCol < 0 || score > bestScore)
        {
            bestScore = score;
            bestCol = col;
        }

        if (bestScore > alpha)
            alpha = bestScore;
    }

    outScore = bestScore;
    return bestCol;
}

int AI::ChooseMove(const Board& board, Cell player, const Difficulty& diff, uint32_t& randState)
{
    int score = 0;
    const int best = BestMove(board, player, diff.mDepth, score);
    if (best < 0)
        return -1;

    // The easier settings blunder rather than search shallowly on their own: a shallow search
    // still never misses an immediate win, which reads as unbeatable-but-stupid. An occasional
    // random legal move feels much more like a beatable opponent.
    if (diff.mBlunderPercent > 0)
    {
        const uint32_t roll = NextRand(randState) % 100u;
        if ((int)roll < diff.mBlunderPercent)
        {
            int legal[kCols];
            int count = 0;
            for (int c = 0; c < kCols; ++c)
            {
                if (board.CanDrop(c))
                    legal[count++] = c;
            }

            if (count > 0)
                return legal[NextRand(randState) % (uint32_t)count];
        }
    }

    return best;
}

// ---------------------------------------------------------------------------
// Self-test
// ---------------------------------------------------------------------------

namespace
{

int sFailures = 0;

void Check(bool condition, const char* what)
{
    if (!condition)
    {
        sFailures += 1;
        // No engine logging here on purpose: this has to build without Octave.
    }
    (void)what;
}

}   // anonymous namespace

int RunSelfTest()
{
    sFailures = 0;

    // stacking
    {
        Board b;
        b.Reset();
        Check(b.NextRow(0) == 0, "empty column starts at row 0");
        Move m = b.Drop(0, Cell::Red);
        Check(m.Valid() && m.mRow == 0, "first disc lands on the bottom row");
        m = b.Drop(0, Cell::Yellow);
        Check(m.Valid() && m.mRow == 1, "second disc stacks");

        for (int i = 0; i < kRows; ++i)
            b.Drop(0, Cell::Red);

        Check(!b.CanDrop(0), "column fills");
        Check(!b.Drop(0, Cell::Red).Valid(), "dropping into a full column is rejected");
    }

    // vertical, horizontal and both diagonals
    {
        Board b;
        b.Reset();
        for (int i = 0; i < 3; ++i) b.Drop(1, Cell::Red);
        Check(b.GetResult() == Result::Playing, "three in a column is not a win");
        b.Drop(1, Cell::Red);
        Check(b.GetResult() == Result::RedWins, "four in a column wins");
    }
    {
        Board b;
        b.Reset();
        for (int c = 0; c < 4; ++c) b.Drop(c, Cell::Yellow);
        Check(b.GetResult() == Result::YellowWins, "four in a row wins");
    }
    {
        Board b;
        b.Reset();
        // staircase up to the right
        b.Drop(0, Cell::Red);
        b.Drop(1, Cell::Yellow); b.Drop(1, Cell::Red);
        b.Drop(2, Cell::Yellow); b.Drop(2, Cell::Yellow); b.Drop(2, Cell::Red);
        b.Drop(3, Cell::Yellow); b.Drop(3, Cell::Yellow); b.Drop(3, Cell::Yellow);
        Check(b.GetResult() == Result::Playing, "diagonal not complete yet");
        b.Drop(3, Cell::Red);
        Check(b.GetResult() == Result::RedWins, "rising diagonal wins");
    }

    // undo restores state exactly
    {
        Board b;
        b.Reset();
        b.Drop(3, Cell::Red);
        b.Drop(3, Cell::Yellow);
        b.Undo(3);
        Check(b.Get(3, 1) == Cell::Empty, "undo clears the cell");
        Check(b.NextRow(3) == 1, "undo restores the height");
        Check(b.GetMoveCount() == 1, "undo restores the move count");
    }

    // the AI must take an immediate win, and block one
    {
        Board b;
        b.Reset();
        b.Drop(0, Cell::Red); b.Drop(1, Cell::Red); b.Drop(2, Cell::Red);
        int score = 0;
        Check(AI::BestMove(b, Cell::Red, 4, score) == 3, "AI completes four");
    }
    {
        Board b;
        b.Reset();
        b.Drop(0, Cell::Red); b.Drop(1, Cell::Red); b.Drop(2, Cell::Red);
        int score = 0;
        Check(AI::BestMove(b, Cell::Yellow, 4, score) == 3, "AI blocks four");
    }

    return sFailures;
}

}   // namespace C4
