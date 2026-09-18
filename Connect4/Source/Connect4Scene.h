#pragma once

#include <stdint.h>

#include "Board.h"

#include "glm/glm.hpp"
#include "glm/gtc/quaternion.hpp"

class Node;
class Node3D;
class StaticMesh3D;
class StaticMesh;
class Material;
class Box3D;
class SoundWave;

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
    //
    // depthSample, when given, sets how deep in the slot the discs sit: whatever depth that node
    // has been placed at is the depth they are given. Nudging the chip in the editor until it looks
    // right in the slot is a far better way to set that than a number in a header, and it is the
    // one part of the layout the mesh itself cannot answer -- the hole tells you where a disc goes
    // across the board, not how far into it. Falls back to the middle of the panel.
    bool Build(StaticMesh3D* frameNode, Node3D* depthSample);

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

    // --- sound --------------------------------------------------------------
    void PlayDropSound();
    void PlayLandSound();
    void PlayRerackSound();

    // --- whole board --------------------------------------------------------
    void HighlightWin(const C4::Move* winLine);
    void BeginRerack();
    bool IsRerackAnimating() const { return mRerackPhase != RerackPhase::Idle; }
    void ClearDiscs();

    const Connect4Layout& GetLayout() const { return mLayout; }

private:

    // Emptying the board is the real Connect Four gesture, which is why the frame, the stand and
    // the release tray are separate models: the grid lifts out of its stand, the tray is pulled,
    // and the discs drop through onto the table.
    enum class RerackPhase : uint8_t
    {
        Idle,
        Lift,       // the grid rises out of the stand, discs still held in their slots
        PullTray,   // the release slides out from under them
        Fall,       // they drop onto the table and scatter
        Settle,     // resting, before the board is set up again
    };

    StaticMesh3D* CreateDisc(const char* name);
    Material* GetDiscMaterial(C4::Cell who);

    void UpdateRerack(float deltaTime);
    void ResetBoardParts();
    void BuildObstacles();
    void BuildPhysicsColliders();
    void StartDiscPhysics();
    void StopDiscPhysics();
    bool AreDiscsAsleep() const;

    Connect4Layout mLayout;

    StaticMesh3D* mFrameNode = nullptr;
    StaticMesh3D* mCursorDisc = nullptr;

    // Spawned discs copy the chip already in the scene: same parent, same local scale and
    // rotation. Parenting them anywhere else means inheriting that node's scale instead, and the
    // frame and the chip are modelled at wildly different sizes -- the frame mesh is hundreds of
    // units across and scaled down to fit, the chip is about one unit and scaled roughly 1. A disc
    // parented under the frame comes out around a hundred times too small to see.
    Node3D* mDiscParent = nullptr;
    glm::vec3 mDiscScale = glm::vec3(1.0f);
    glm::vec3 mDiscRotation = glm::vec3(0.0f);

    // Optional: a missing sound is silent rather than a failure, so the game still runs with the
    // audio not yet in place.
    SoundWave* mSlotBeginSound = nullptr;
    SoundWave* mSlotEndSound = nullptr;
    SoundWave* mRerackSound = nullptr;

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
        // Rerack: the speed the disc is let go with when the tray is pulled. Bullet takes over
        // from there, so nothing here tracks how it falls, what it hits, or how it comes to rest.
        glm::vec3 mVelocity = glm::vec3(0.0f);

        bool mInUse = false;
    };

    Disc mDiscs[C4::kCols * C4::kRows];
    uint32_t mNumDiscsUsed = 0;

    // The disc currently falling. Index into mDiscs, or -1.
    int32_t mDroppingDisc = -1;
    float mDropTime = 0.0f;
    float mDropDuration = 0.0f;
    bool mDropActive = false;

    // The other two pieces of the board, moved during a rerack and put back afterwards.
    Node3D* mStandNode = nullptr;
    Node3D* mTrayNode = nullptr;

    glm::vec3 mFrameHome = glm::vec3(0.0f);
    glm::vec3 mTrayHome = glm::vec3(0.0f);

    glm::vec3 mLiftAxis = glm::vec3(0.0f, 1.0f, 0.0f);   // the board's own up, in world
    glm::vec3 mTrayAxis = glm::vec3(1.0f, 0.0f, 0.0f);   // the way the tray travels, in world
    glm::vec3 mSpillAxis = glm::vec3(0.0f);              // the way released discs drift, in world
    glm::vec3 mSpreadAxis = glm::vec3(1.0f, 0.0f, 0.0f); // across the table, square to the spill

    float mLiftDistance = 0.0f;
    float mTrayDistance = 0.0f;

    // Where a disc comes to rest once it has fallen out: the top of the table, which is the
    // bottom of the stand.
    float mTableY = 0.0f;

    // In world units, for placing the static colliders and deciding when a disc has settled.
    float mDiscRadius = 0.0f;
    float mDiscHalfThickness = 0.0f;

    // In the mesh's own units, for the collision shape. Primitive3D applies the node's world scale
    // to whatever shape it is given, so a shape built from world measurements would come out
    // scaled a second time.
    float mDiscLocalRadius = 0.0f;
    float mDiscLocalHalfThickness = 0.0f;

    // The table and the stand's feet, as static bodies. Everything the discs land on.
    Box3D* mGroundCollider = nullptr;
    Box3D* mFootColliders[2] = { nullptr, nullptr };

    bool mDiscPhysicsRunning = false;
    bool mRerackSoundPlayed = false;

    // Which of the chip model's local axes runs through the flat of the disc. Needed to work out
    // which way it is facing, since the model is free to be built along any of them.
    int32_t mDiscFaceAxis = 2;

    // The stand's feet, as boxes on the table. Discs spread out to exactly where these sit, and
    // without them they slid straight through. Two, because the stand has a foot at each end;
    // measured from the model so they follow it if it is moved or rescaled.
    struct Obstacle
    {
        glm::vec3 mMin = glm::vec3(0.0f);
        glm::vec3 mMax = glm::vec3(0.0f);
    };

    Obstacle mObstacles[2];
    uint32_t mNumObstacles = 0;

    // The underside of the frame, unlifted. A disc is still inside the board until it is below
    // this plus however far the grid has risen.
    float mFrameBottomY = 0.0f;

    RerackPhase mRerackPhase = RerackPhase::Idle;
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
