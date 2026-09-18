#pragma once

#include <stdint.h>

#include "Board.h"

#include "glm/glm.hpp"

class Node;
class Node3D;
class StaticMesh3D;
class StaticMesh;
class Material;

// The presentation half of Connect Four: everything the rules layer deliberately knows nothing
// about. Board.h decides what happens, this decides what it looks like.
//
// The board's geometry is expressed as two sets of points, both derived from the frame model
// rather than typed in as world coordinates:
//
//   entry points  -- seven, one above each column, where a disc waits before it is dropped.
//                    Moving the cursor left and right steps between these.
//   still points  -- forty-two, one per cell, where a disc comes to rest.
//
// They are built in the frame mesh's own local space and then run through the frame node's world
// transform, so the board can be moved, rotated or rescaled in the editor and the points follow it
// without anything here changing.
class Connect4Layout
{
public:

    // Measures the frame mesh and fills in both sets of points. Returns false if the frame node
    // has no mesh to measure.
    bool Build(StaticMesh3D* frameNode);

    bool IsBuilt() const { return mBuilt; }

    const glm::vec3& GetEntryPoint(int col) const;
    const glm::vec3& GetStillPoint(int col, int row) const;

    // Spacing between adjacent cells, in world units. Used to size the discs' fall and to keep
    // effects proportional to however large the board has been scaled.
    float GetColSpacing() const { return mColSpacing; }
    float GetRowSpacing() const { return mRowSpacing; }

private:

    glm::vec3 mEntry[C4::kCols];
    glm::vec3 mStill[C4::kCols][C4::kRows];

    float mColSpacing = 1.0f;
    float mRowSpacing = 1.0f;
    bool mBuilt = false;
};

class Connect4Scene
{
public:

    Connect4Scene();
    ~Connect4Scene();

    // Finds the board nodes in the loaded scene and builds the disc pool. Returns false if the
    // scene is missing something it needs, in which case the game still runs -- it just has
    // nothing to show.
    bool Initialize();

    bool IsReady() const { return mReady; }

    void Update(float deltaTime);

    // --- cursor -------------------------------------------------------------
    void ShowCursorDisc(C4::Cell who);
    void HideCursorDisc();
    void SetCursorColumn(int col);

    // --- drops --------------------------------------------------------------
    void BeginDrop(const C4::Move& move, C4::Cell who);
    bool IsDropAnimating() const { return mDropActive; }

    // --- whole board --------------------------------------------------------
    void HighlightWin(const C4::Move* winLine);
    void BeginRerack();
    bool IsRerackAnimating() const { return mRerackActive; }
    void ClearDiscs();

    const Connect4Layout& GetLayout() const { return mLayout; }

private:

    StaticMesh3D* CreateDisc(const char* name);
    Material* GetDiscMaterial(C4::Cell who);

    Connect4Layout mLayout;

    Node3D* mBoardRoot = nullptr;        // parent for spawned discs
    StaticMesh3D* mFrameNode = nullptr;
    StaticMesh3D* mCursorDisc = nullptr;

    StaticMesh* mDiscMesh = nullptr;
    Material* mRedMaterial = nullptr;
    Material* mYellowMaterial = nullptr;

    // One disc per cell, spawned up front and reused. A board game has a known, small ceiling on
    // how many discs can exist, so there is no reason to allocate during play -- and on the
    // GameCube an allocation mid-drop is exactly the kind of thing that causes a visible hitch.
    struct Disc
    {
        StaticMesh3D* mNode = nullptr;
        glm::vec3 mFrom = glm::vec3(0.0f);
        glm::vec3 mTo = glm::vec3(0.0f);
        glm::vec3 mVelocity = glm::vec3(0.0f);   // rerack only
        float mSpin = 0.0f;                      // rerack only
        bool mInUse = false;
    };

    Disc mDiscs[C4::kCols * C4::kRows];
    uint32_t mNumDiscsUsed = 0;

    // The disc currently falling. Index into mDiscs, or -1.
    int32_t mDroppingDisc = -1;
    float mDropTime = 0.0f;
    float mDropDuration = 0.0f;
    bool mDropActive = false;

    bool mRerackActive = false;
    float mRerackTime = 0.0f;

    // Winning discs, so they can be pulsed while the result is up.
    int32_t mWinDiscs[4] = { -1, -1, -1, -1 };
    int32_t mNumWinDiscs = 0;
    float mWinPulse = 0.0f;

    int mCursorCol = C4::kCols / 2;
    float mCursorSlide = 0.0f;       // eased progress of the cursor's slide between columns
    glm::vec3 mCursorFrom = glm::vec3(0.0f);
    glm::vec3 mCursorTo = glm::vec3(0.0f);

    bool mReady = false;
};
