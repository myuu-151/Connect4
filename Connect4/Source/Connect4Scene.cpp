#include "Connect4Scene.h"

#include "Engine.h"
#include "World.h"
#include "Log.h"
#include "AssetManager.h"
#include "Assets/StaticMesh.h"
#include "Assets/MaterialLite.h"
#include "Nodes/Node.h"
#include "Nodes/3D/Node3d.h"
#include "Nodes/3D/StaticMesh3d.h"

#include <math.h>

void OctLog(const char* format, ...);

namespace
{

// ---------------------------------------------------------------------------
// Board layout, as fractions of the frame model's own bounding box.
//
// Fractions rather than world coordinates so the board can be moved, rotated or rescaled in the
// editor without touching code. These are the numbers to nudge if the discs do not sit dead centre
// in their holes -- nothing else here needs to change.
//
// The span values are the distance between the OUTERMOST hole centres, not the full playfield, so
// kColSpanFrac = 0.84 means the centres of columns 0 and 6 sit 84% of the frame's width apart.
// ---------------------------------------------------------------------------
const float kColSpanFrac = 0.84f;      // column 0 centre -> column 6 centre, over frame width
const float kRowSpanFrac = 0.70f;      // row 0 centre -> row 5 centre, over frame height
const float kGridCenterYFrac = 0.50f;  // where the grid's midpoint sits, 0 = frame bottom, 1 = top

// How far above the top of the frame a waiting disc hovers, as a multiple of row spacing.
const float kEntryHeightRows = 0.9f;

// Fall timing. A disc accelerates like something dropped rather than sliding at a constant rate,
// so the time depends on how far it has to go: a disc into an empty column takes noticeably longer
// than one landing on a nearly full stack, which is what the eye expects.
const float kFallAccel = 26.0f;        // in row-spacings per second squared
const float kMinFallTime = 0.18f;
const float kMaxFallTime = 0.85f;

// A short bounce once it lands, scaled to the board so it stays proportional.
const float kBounceHeightRows = 0.14f;
const float kBounceTime = 0.16f;

// Cursor slide between columns. Short enough to feel responsive when stepping across the board,
// long enough not to teleport.
const float kCursorSlideTime = 0.09f;

const float kRerackDuration = 2.4f;
const float kRerackGravity = 18.0f;    // row-spacings per second squared

const glm::vec4 kRedTint = glm::vec4(1.00f, 1.00f, 1.00f, 1.0f);   // the mesh is already red
const glm::vec4 kYellowTint = glm::vec4(2.05f, 1.62f, 0.22f, 1.0f);

// Smoothstep, for the cursor slide.
float EaseInOut(float t)
{
    t = glm::clamp(t, 0.0f, 1.0f);
    return t * t * (3.0f - 2.0f * t);
}

// Local-space axis-aligned bounds of a mesh. The engine's Bounds is a sphere, which cannot give
// the separate width and height the grid needs, so measure the vertices directly. This runs once
// at startup on a few thousand vertices.
bool MeasureMeshBounds(StaticMesh* mesh, glm::vec3& outMin, glm::vec3& outMax)
{
    if (mesh == nullptr || mesh->GetNumVertices() == 0)
    {
        return false;
    }

    const uint32_t numVerts = mesh->GetNumVertices();
    outMin = glm::vec3(1e9f);
    outMax = glm::vec3(-1e9f);

    if (mesh->HasVertexColor())
    {
        const VertexColor* verts = mesh->GetColorVertices();
        for (uint32_t i = 0; i < numVerts; ++i)
        {
            outMin = glm::min(outMin, verts[i].mPosition);
            outMax = glm::max(outMax, verts[i].mPosition);
        }
    }
    else
    {
        const Vertex* verts = mesh->GetVertices();
        for (uint32_t i = 0; i < numVerts; ++i)
        {
            outMin = glm::min(outMin, verts[i].mPosition);
            outMax = glm::max(outMax, verts[i].mPosition);
        }
    }

    return true;
}

}   // anonymous namespace

// ---------------------------------------------------------------------------
// Connect4Layout
// ---------------------------------------------------------------------------

bool Connect4Layout::Build(StaticMesh3D* frameNode)
{
    mBuilt = false;

    if (frameNode == nullptr)
    {
        return false;
    }

    StaticMesh* mesh = frameNode->GetStaticMesh();

    glm::vec3 localMin(0.0f);
    glm::vec3 localMax(0.0f);

    if (!MeasureMeshBounds(mesh, localMin, localMax))
    {
        LogError("Connect4: frame node has no mesh to measure.");
        return false;
    }

    const glm::vec3 size = localMax - localMin;

    // Work in the mesh's own local space, then transform. Doing it this way means the frame's
    // scale and rotation are applied by the same matrix the renderer uses, so the points cannot
    // drift out of step with what is drawn.
    const float colSpanLocal = size.x * kColSpanFrac;
    const float rowSpanLocal = size.y * kRowSpanFrac;

    const float colStepLocal = colSpanLocal / float(C4::kCols - 1);
    const float rowStepLocal = rowSpanLocal / float(C4::kRows - 1);

    const float centerX = (localMin.x + localMax.x) * 0.5f;
    const float centerY = localMin.y + size.y * kGridCenterYFrac;
    const float centerZ = (localMin.z + localMax.z) * 0.5f;

    // Row 0 is the bottom of the board, matching Board.h, so the grid is built upwards from the
    // lowest cell.
    const float bottomY = centerY - rowSpanLocal * 0.5f;
    const float leftX = centerX - colSpanLocal * 0.5f;

    const float entryYLocal = localMax.y + rowStepLocal * kEntryHeightRows;

    const glm::mat4& toWorld = frameNode->GetTransform();

    for (int col = 0; col < C4::kCols; ++col)
    {
        const float x = leftX + colStepLocal * float(col);

        mEntry[col] = glm::vec3(toWorld * glm::vec4(x, entryYLocal, centerZ, 1.0f));

        for (int row = 0; row < C4::kRows; ++row)
        {
            const float y = bottomY + rowStepLocal * float(row);
            mStill[col][row] = glm::vec3(toWorld * glm::vec4(x, y, centerZ, 1.0f));
        }
    }

    // Report the spacing in world units, which is what the animation code wants: the local step
    // means nothing on its own once the board has been scaled.
    mColSpacing = glm::length(mStill[1][0] - mStill[0][0]);
    mRowSpacing = glm::length(mStill[0][1] - mStill[0][0]);

    mBuilt = true;

    OctLog("Connect4: layout built, col spacing %.3f, row spacing %.3f",
           mColSpacing, mRowSpacing);

    return true;
}

const glm::vec3& Connect4Layout::GetEntryPoint(int col) const
{
    col = glm::clamp(col, 0, C4::kCols - 1);
    return mEntry[col];
}

const glm::vec3& Connect4Layout::GetStillPoint(int col, int row) const
{
    col = glm::clamp(col, 0, C4::kCols - 1);
    row = glm::clamp(row, 0, C4::kRows - 1);
    return mStill[col][row];
}

// ---------------------------------------------------------------------------
// Connect4Scene
// ---------------------------------------------------------------------------

Connect4Scene::Connect4Scene()
{
}

Connect4Scene::~Connect4Scene()
{
}

bool Connect4Scene::Initialize()
{
    mReady = false;

    World* world = GetWorld(0);
    Node* root = world ? world->GetRootNode() : nullptr;

    if (root == nullptr)
    {
        LogError("Connect4: no world root; scene not set up.");
        return false;
    }

    // The frame is what the whole layout is measured from, so it is the one node that must exist.
    mFrameNode = root->FindChild<StaticMesh3D>("MainFrame", true);

    if (mFrameNode == nullptr)
    {
        LogError("Connect4: could not find a StaticMesh3D named MainFrame.");
        return false;
    }

    if (!mLayout.Build(mFrameNode))
    {
        return false;
    }

    // The chip already placed in the scene is the template: it supplies both the mesh and the red
    // material, so the disc pool matches whatever is in the editor without naming assets here.
    StaticMesh3D* templateChip = root->FindChild<StaticMesh3D>("Chip_Red", true);

    if (templateChip != nullptr)
    {
        mDiscMesh = templateChip->GetStaticMesh();
        mRedMaterial = templateChip->GetMaterial();
    }

    if (mDiscMesh == nullptr)
    {
        LogError("Connect4: could not find the Chip_Red mesh.");
        return false;
    }

    // Yellow is the red material tinted, rather than a second asset to keep in sync. MaterialLite
    // multiplies its colour over the texture, so the chip keeps its moulding and wear.
    if (mRedMaterial != nullptr)
    {
        MaterialLite* yellow = MaterialLite::New(mRedMaterial);

        if (yellow != nullptr)
        {
            yellow->SetColor(kYellowTint);
            mYellowMaterial = yellow;
        }
    }

    // Spawned discs go beside the chip that is already in the scene, and copy its local scale and
    // rotation. The two models are authored at very different sizes -- the frame is hundreds of
    // units across and scaled right down, the chip is about a unit and scaled near 1 -- so a disc
    // parented under the frame would inherit the frame's scale and come out far too small to see.
    if (templateChip != nullptr)
    {
        mDiscParent = templateChip->GetParent() ? templateChip->GetParent()->As<Node3D>() : nullptr;
        mDiscScale = templateChip->GetScale();
        mDiscRotation = templateChip->GetRotationEuler();
    }

    if (mDiscParent == nullptr)
    {
        mDiscParent = root->As<Node3D>();
    }

    if (mDiscParent == nullptr)
    {
        LogError("Connect4: no Node3D to parent discs to.");
        return false;
    }

    // Reuse the chip already in the scene as the waiting disc, so nothing is left sitting in the
    // middle of the board once the game starts driving it.
    if (templateChip != nullptr)
    {
        mCursorDisc = templateChip;
    }
    else
    {
        mCursorDisc = CreateDisc("CursorDisc");
    }

    for (uint32_t i = 0; i < C4::kCols * C4::kRows; ++i)
    {
        char name[32];
        snprintf(name, sizeof(name), "Disc%02u", i);

        mDiscs[i].mNode = CreateDisc(name);
        mDiscs[i].mInUse = false;

        if (mDiscs[i].mNode != nullptr)
        {
            mDiscs[i].mNode->SetVisible(false);
        }
    }

    if (mCursorDisc != nullptr)
    {
        mCursorDisc->SetVisible(false);
        mCursorFrom = mLayout.GetEntryPoint(mCursorCol);
        mCursorTo = mCursorFrom;
        mCursorSlide = 1.0f;
    }

    mReady = true;
    OctLog("Connect4: scene ready");
    return true;
}

StaticMesh3D* Connect4Scene::CreateDisc(const char* name)
{
    if (mDiscParent == nullptr || mDiscMesh == nullptr)
    {
        return nullptr;
    }

    StaticMesh3D* disc = mDiscParent->CreateChild<StaticMesh3D>();

    if (disc == nullptr)
    {
        return nullptr;
    }

    disc->SetName(name);
    disc->SetStaticMesh(mDiscMesh);

    // Match the chip in the scene rather than defaulting to an identity transform, so the disc is
    // the size and orientation the model was placed at.
    disc->SetScale(mDiscScale);
    disc->SetRotation(mDiscRotation);

    disc->SetVisible(false);

    return disc;
}

Material* Connect4Scene::GetDiscMaterial(C4::Cell who)
{
    if (who == C4::Cell::Yellow && mYellowMaterial != nullptr)
    {
        return mYellowMaterial;
    }

    return mRedMaterial;
}

void Connect4Scene::ShowCursorDisc(C4::Cell who)
{
    if (mCursorDisc == nullptr)
    {
        return;
    }

    mCursorDisc->SetMaterialOverride(GetDiscMaterial(who));
    mCursorDisc->SetVisible(true);

    // Appear directly over the current column rather than sliding in from wherever the last disc
    // was released.
    const glm::vec3 entry = mLayout.GetEntryPoint(mCursorCol);
    mCursorFrom = entry;
    mCursorTo = entry;
    mCursorSlide = 1.0f;
    mCursorDisc->SetWorldPosition(entry);
}

void Connect4Scene::HideCursorDisc()
{
    if (mCursorDisc != nullptr)
    {
        mCursorDisc->SetVisible(false);
    }
}

void Connect4Scene::SetCursorColumn(int col)
{
    col = glm::clamp(col, 0, C4::kCols - 1);

    if (col == mCursorCol && mCursorSlide >= 1.0f)
    {
        return;
    }

    // Slide from wherever the disc actually is, not from the column it was nominally on, so
    // stepping quickly across several columns stays continuous instead of snapping.
    mCursorFrom = (mCursorDisc != nullptr && mCursorSlide < 1.0f)
                ? mCursorDisc->GetWorldPosition()
                : mLayout.GetEntryPoint(mCursorCol);

    mCursorCol = col;
    mCursorTo = mLayout.GetEntryPoint(col);
    mCursorSlide = 0.0f;
}

void Connect4Scene::BeginDrop(const C4::Move& move, C4::Cell who)
{
    if (!mReady || !move.Valid())
    {
        return;
    }

    // Take the next disc from the pool. The pool is exactly board-sized, so running out means the
    // board is full, which the rules prevent.
    int32_t index = -1;

    for (uint32_t i = 0; i < C4::kCols * C4::kRows; ++i)
    {
        if (!mDiscs[i].mInUse && mDiscs[i].mNode != nullptr)
        {
            index = (int32_t)i;
            break;
        }
    }

    if (index < 0)
    {
        LogWarning("Connect4: no free disc to drop.");
        return;
    }

    Disc& disc = mDiscs[index];
    disc.mInUse = true;
    disc.mFrom = mLayout.GetEntryPoint(move.mCol);
    disc.mTo = mLayout.GetStillPoint(move.mCol, move.mRow);

    disc.mNode->SetMaterialOverride(GetDiscMaterial(who));
    disc.mNode->SetWorldPosition(disc.mFrom);
    disc.mNode->SetVisible(true);

    // Time the fall from the distance, so a disc into a deep column takes longer than one landing
    // on top of a stack.
    const float rowSpacing = glm::max(mLayout.GetRowSpacing(), 0.0001f);
    const float distanceRows = glm::length(disc.mTo - disc.mFrom) / rowSpacing;
    const float freeFallTime = sqrtf(2.0f * distanceRows / kFallAccel);

    mDropDuration = glm::clamp(freeFallTime, kMinFallTime, kMaxFallTime) + kBounceTime;
    mDropTime = 0.0f;
    mDroppingDisc = index;
    mDropActive = true;
    mNumDiscsUsed++;

    // The waiting disc has been released, so it stops being drawn until the next turn puts a new
    // one up.
    HideCursorDisc();
}

void Connect4Scene::Update(float deltaTime)
{
    if (!mReady)
    {
        return;
    }

    // --- cursor slide -------------------------------------------------------
    if (mCursorDisc != nullptr && mCursorSlide < 1.0f)
    {
        mCursorSlide += deltaTime / kCursorSlideTime;
        mCursorSlide = glm::min(mCursorSlide, 1.0f);

        const glm::vec3 pos = glm::mix(mCursorFrom, mCursorTo, EaseInOut(mCursorSlide));
        mCursorDisc->SetWorldPosition(pos);
    }

    // --- falling disc -------------------------------------------------------
    if (mDropActive && mDroppingDisc >= 0)
    {
        Disc& disc = mDiscs[mDroppingDisc];

        mDropTime += deltaTime;

        const float fallTime = glm::max(mDropDuration - kBounceTime, 0.0001f);

        if (mDropTime < fallTime)
        {
            // Accelerating fall: position goes with the square of time, which is what dropping
            // actually looks like. A linear interpolation here reads as the disc being lowered.
            const float t = mDropTime / fallTime;
            const float eased = t * t;
            disc.mNode->SetWorldPosition(glm::mix(disc.mFrom, disc.mTo, eased));
        }
        else if (mDropTime < mDropDuration)
        {
            // Land, then a single small hop. Discs in a real Connect Four rattle rather than stop
            // dead, and without this the disc arriving reads as a snap.
            const float t = (mDropTime - fallTime) / kBounceTime;
            const float hop = sinf(t * 3.14159265f) * kBounceHeightRows * mLayout.GetRowSpacing();

            glm::vec3 pos = disc.mTo;
            pos.y += hop;
            disc.mNode->SetWorldPosition(pos);
        }
        else
        {
            // Settle exactly on the still point. The animation is approximate; where the disc ends
            // up is not, so a long game does not accumulate drift in the stack.
            disc.mNode->SetWorldPosition(disc.mTo);
            mDropActive = false;
            mDroppingDisc = -1;
        }
    }

    // --- winning line -------------------------------------------------------
    if (mNumWinDiscs > 0)
    {
        mWinPulse += deltaTime;

        // Lift the four discs slightly in turn, so the line reads as a line rather than four
        // separate discs twitching.
        for (int32_t i = 0; i < mNumWinDiscs; ++i)
        {
            const int32_t index = mWinDiscs[i];

            if (index < 0 || mDiscs[index].mNode == nullptr)
            {
                continue;
            }

            const float phase = mWinPulse * 4.0f - float(i) * 0.5f;
            const float lift = (sinf(phase) * 0.5f + 0.5f) * 0.10f * mLayout.GetRowSpacing();

            glm::vec3 pos = mDiscs[index].mTo;
            pos.y += lift;
            mDiscs[index].mNode->SetWorldPosition(pos);
        }
    }

    // --- rerack -------------------------------------------------------------
    if (mRerackActive)
    {
        mRerackTime += deltaTime;

        const float gravity = kRerackGravity * mLayout.GetRowSpacing();

        for (uint32_t i = 0; i < C4::kCols * C4::kRows; ++i)
        {
            Disc& disc = mDiscs[i];

            if (!disc.mInUse || disc.mNode == nullptr)
            {
                continue;
            }

            disc.mVelocity.y -= gravity * deltaTime;

            glm::vec3 pos = disc.mNode->GetWorldPosition();
            pos += disc.mVelocity * deltaTime;
            disc.mNode->SetWorldPosition(pos);

            disc.mNode->SetRotation(disc.mNode->GetRotationEuler() +
                                    glm::vec3(0.0f, 0.0f, disc.mSpin * deltaTime));
        }

        if (mRerackTime >= kRerackDuration)
        {
            ClearDiscs();
            mRerackActive = false;
        }
    }
}

void Connect4Scene::HighlightWin(const C4::Move* winLine)
{
    mNumWinDiscs = 0;
    mWinPulse = 0.0f;

    if (winLine == nullptr)
    {
        return;
    }

    // Match each winning cell back to the disc sitting on it. Discs are identified by where they
    // came to rest, which is exact because every disc is snapped to its still point on landing.
    for (int32_t i = 0; i < 4; ++i)
    {
        const glm::vec3 target = mLayout.GetStillPoint(winLine[i].mCol, winLine[i].mRow);

        for (uint32_t d = 0; d < C4::kCols * C4::kRows; ++d)
        {
            if (!mDiscs[d].mInUse)
            {
                continue;
            }

            if (glm::distance(mDiscs[d].mTo, target) < 0.001f)
            {
                mWinDiscs[mNumWinDiscs++] = (int32_t)d;
                break;
            }
        }
    }
}

void Connect4Scene::BeginRerack()
{
    // Stop pulsing the winning line: from here every disc is falling out together.
    mNumWinDiscs = 0;

    mRerackActive = true;
    mRerackTime = 0.0f;

    HideCursorDisc();

    uint32_t seed = 0x9E3779B9u;

    for (uint32_t i = 0; i < C4::kCols * C4::kRows; ++i)
    {
        Disc& disc = mDiscs[i];

        if (!disc.mInUse)
        {
            continue;
        }

        // A little scatter so they do not all fall as one rigid block. Deterministic, because a
        // board game replaying identically is a feature and it keeps this reproducible on hardware.
        seed = seed * 1664525u + 1013904223u;
        const float rx = ((seed >> 16) & 0xFF) / 255.0f - 0.5f;
        seed = seed * 1664525u + 1013904223u;
        const float rz = ((seed >> 16) & 0xFF) / 255.0f - 0.5f;
        seed = seed * 1664525u + 1013904223u;
        const float rs = ((seed >> 16) & 0xFF) / 255.0f - 0.5f;

        const float spacing = mLayout.GetRowSpacing();

        disc.mVelocity = glm::vec3(rx * spacing * 1.2f,
                                   -0.2f * spacing,
                                   rz * spacing * 0.8f);
        disc.mSpin = rs * 540.0f;
    }
}

void Connect4Scene::ClearDiscs()
{
    for (uint32_t i = 0; i < C4::kCols * C4::kRows; ++i)
    {
        if (mDiscs[i].mNode != nullptr)
        {
            mDiscs[i].mNode->SetVisible(false);

            // Back to the orientation the chip was placed at, not to identity: the rerack spins
            // the discs, and resetting to zero would leave the next game's discs lying at whatever
            // angle identity happens to be for this model.
            mDiscs[i].mNode->SetRotation(mDiscRotation);
        }

        mDiscs[i].mInUse = false;
        mDiscs[i].mVelocity = glm::vec3(0.0f);
        mDiscs[i].mSpin = 0.0f;
    }

    mNumDiscsUsed = 0;
    mNumWinDiscs = 0;
    mDroppingDisc = -1;
    mDropActive = false;
}
