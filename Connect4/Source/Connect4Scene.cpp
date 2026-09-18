#include "Connect4Scene.h"

#include "Engine.h"
#include "World.h"
#include "Log.h"
#include "AssetManager.h"
#include "Renderer.h"
#include "Assets/StaticMesh.h"
#include "Assets/MaterialLite.h"
#include "Nodes/Node.h"
#include "Nodes/3D/Node3d.h"
#include "Nodes/3D/StaticMesh3d.h"
#include "Nodes/3D/Camera3d.h"

#include <math.h>

void OctLog(const char* format, ...);

namespace
{

// ---------------------------------------------------------------------------
// Board layout: the measured centre of every hole in the frame model.
//
// These were not guessed. The frame mesh was analysed directly: the holes are through-bores in a
// thin panel, so their walls are the surfaces whose normals lie in the plane of the board. Welding
// the mesh by position (it is flat shaded, so no two triangles share a vertex index) and running
// union-find over those walls separates each bore exactly, and their centroids are the numbers
// below. The grid came out regular to within about 2 units in a 109-unit pitch, and every one of
// the 42 cells was accounted for.
//
// They are stored as fractions of the mesh's bounding box rather than absolute coordinates, so a
// re-export of the same model at a different scale still lands correctly. Only remodelling the
// board would invalidate them, and the mismatch would be obvious.
//
// Column 0 is at the model's -X end, row 0 at its -Y end (the bottom), matching Board.h.
// ---------------------------------------------------------------------------
const float kColFrac[C4::kCols] =
{
    0.103791f, 0.239605f, 0.368578f, 0.497419f, 0.628205f, 0.758798f, 0.892981f
};

const float kRowFrac[C4::kRows] =
{
    0.093573f, 0.257181f, 0.421684f, 0.581253f, 0.742253f, 0.901033f
};

// Discs sit in the middle of the panel's thickness.
const float kPlaneFrac = 0.5f;

// Set this if pressing right moves the disc left on screen. The board is free to be rotated in the
// editor, and a half turn mirrors the model's X against the screen; nothing else has to change,
// since Connect Four is symmetric and only the cursor direction is affected.
const bool kMirrorColumns = false;

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

// --- rerack ----------------------------------------------------------------
// The grid lifts out of its stand, the release tray is pulled, and the discs drop onto the table.
const float kLiftTime = 0.70f;
const float kPullTime = 0.45f;
const float kSettleTime = 1.10f;       // how long they are left lying on the table

// How far the grid rises, as a fraction of the frame's own height: far enough that the bottom row
// clears the stand and the discs have somewhere to fall.
const float kLiftFrac = 0.60f;

// How far the tray is pulled, as a multiple of the board's thickness. The tray comes out towards
// the player rather than sideways, so the distance that matters is how deep the board is: enough
// to be clearly out from under the slots, not so far it looks detached.
const float kPullDepthMul = 2.2f;

const float kRerackGravity = 20.0f;    // row-spacings per second squared
const float kDiscRestitution = 0.32f;  // how much of the fall is given back as a bounce
const float kDiscFriction = 0.78f;     // horizontal speed kept per bounce
const float kDiscRestSpeed = 0.35f;    // below this, in row-spacings per second, a disc has stopped
const float kToppleTime = 0.22f;       // how long a landed disc takes to fall flat

const glm::vec4 kRedTint = glm::vec4(1.00f, 1.00f, 1.00f, 1.0f);   // the mesh is already red
const glm::vec4 kYellowTint = glm::vec4(2.05f, 1.62f, 0.22f, 1.0f);

// The orientation a disc ends up in once it has fallen over: flat on the table, face up, turned by
// some arbitrary amount so a heap of them does not look stamped from one mould.
//
// Built from whichever model axis runs through the flat of the disc, rather than assuming the chip
// was modelled facing any particular way.
glm::quat FlatRotation(int32_t faceAxis, float yawDegrees)
{
    glm::quat layDown;

    switch (faceAxis)
    {
    case 0:  layDown = glm::angleAxis(glm::radians(90.0f), glm::vec3(0, 0, 1)); break;
    case 1:  layDown = glm::quat(1.0f, 0.0f, 0.0f, 0.0f); break;
    default: layDown = glm::angleAxis(glm::radians(-90.0f), glm::vec3(1, 0, 0)); break;
    }

    return glm::angleAxis(glm::radians(yawDegrees), glm::vec3(0, 1, 0)) * layDown;
}

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

bool Connect4Layout::Build(StaticMesh3D* frameNode, Node3D* depthSample)
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
    const glm::mat4& toWorld = frameNode->GetTransform();

    // Depth in the slot. The holes say where a disc sits across the face of the board but nothing
    // about how far into it, so take that from the chip placed in the scene: whatever depth it has
    // been nudged to is the depth every disc gets. Its position is otherwise unused -- the game
    // drives it -- so the placement is free to mean this.
    float planeZ = localMin.z + size.z * kPlaneFrac;

    if (depthSample != nullptr)
    {
        const glm::vec3 sampleLocal =
            glm::vec3(glm::inverse(toWorld) * glm::vec4(depthSample->GetWorldPosition(), 1.0f));

        planeZ = sampleLocal.z;
    }

    // Row spacing is needed for the entry height, and is taken from the measured rows rather than
    // assumed, so it stays right even though the rows are not perfectly evenly spaced.
    const float rowStepLocal = (kRowFrac[C4::kRows - 1] - kRowFrac[0]) * size.y / float(C4::kRows - 1);
    const float entryYLocal = localMax.y + rowStepLocal * kEntryHeightRows;

    for (int col = 0; col < C4::kCols; ++col)
    {
        const int srcCol = kMirrorColumns ? (C4::kCols - 1 - col) : col;
        const float x = localMin.x + size.x * kColFrac[srcCol];

        mEntry[col] = glm::vec3(toWorld * glm::vec4(x, entryYLocal, planeZ, 1.0f));

        for (int row = 0; row < C4::kRows; ++row)
        {
            const float y = localMin.y + size.y * kRowFrac[row];
            mStill[col][row] = glm::vec3(toWorld * glm::vec4(x, y, planeZ, 1.0f));
        }
    }

    // Report the spacing in world units, which is what the animation code wants: the local step
    // means nothing on its own once the board has been scaled.
    mColSpacing = glm::length(mStill[1][0] - mStill[0][0]);
    mRowSpacing = glm::length(mStill[0][1] - mStill[0][0]);

    mBuilt = true;

    LogDebug("C4: layout col %.3f row %.3f", mColSpacing, mRowSpacing);

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

    // The chip already placed in the scene is the template: it supplies the mesh, the red material,
    // the scale and rotation to spawn discs at, and -- through where it has been placed -- how deep
    // in the slot they sit. So it has to be found before the layout is built.
    StaticMesh3D* templateChip = root->FindChild<StaticMesh3D>("Chip_Red", true);

    if (templateChip != nullptr)
    {
        mDiscMesh = templateChip->GetStaticMesh();
        mRedMaterial = templateChip->GetMaterial();

        // Captured here rather than further down because the rerack needs the scale to work out how
        // thick a disc is, and that is computed before the pool is built.
        mDiscScale = templateChip->GetScale();
        mDiscRotation = templateChip->GetRotationEuler();
    }

    if (mDiscMesh == nullptr)
    {
        LogError("Connect4: could not find the Chip_Red mesh.");
        return false;
    }

    if (!mLayout.Build(mFrameNode, templateChip))
    {
        return false;
    }

    // The stand and the release tray, which the rerack moves. Both are optional: without them the
    // discs still fall, there is just less of a gesture to it.
    mStandNode = root->FindChild<Node3D>("Stand", true);
    mTrayNode = root->FindChild<Node3D>("ReleaseTray", true);

    // World positions, not local ones. The lift and pull distances are measured in world units, and
    // these nodes sit under a heavily scaled parent -- the frame model is hundreds of units across
    // and scaled right down -- so treating those distances as local offsets moves the board by a
    // hundredth of what was asked for, which looks like it not moving at all.
    mFrameHome = mFrameNode->GetWorldPosition();

    if (mTrayNode != nullptr)
    {
        mTrayHome = mTrayNode->GetWorldPosition();
    }

    // Directions are taken from the models rather than assumed to be world axes, so the board can
    // be stood anywhere and rotated any way and the grid still rises along its own up, with the
    // tray sliding along its own length.
    {
        const glm::mat4& frameToWorld = mFrameNode->GetTransform();
        mLiftAxis = glm::normalize(glm::vec3(frameToWorld * glm::vec4(0, 1, 0, 0)));

        glm::vec3 frameMin(0.0f);
        glm::vec3 frameMax(0.0f);

        if (MeasureMeshBounds(mFrameNode->GetStaticMesh(), frameMin, frameMax))
        {
            // Convert the height to world units by measuring it through the transform, so the
            // frame's scale is accounted for without having to read it out separately.
            const glm::vec3 bottom = glm::vec3(frameToWorld * glm::vec4(frameMin.x, frameMin.y, frameMin.z, 1.0f));
            const glm::vec3 top = glm::vec3(frameToWorld * glm::vec4(frameMin.x, frameMax.y, frameMin.z, 1.0f));

            mLiftDistance = glm::length(top - bottom) * kLiftFrac;
        }
    }

    // The tray is pulled out towards the player, so it travels along the board's facing direction.
    // The distance is the board's own thickness: whatever the model is scaled to, that is the depth
    // the tray has to clear.
    if (mTrayNode != nullptr)
    {
        const glm::mat4& frameToWorld = mFrameNode->GetTransform();

        mTrayAxis = glm::normalize(glm::vec3(frameToWorld * glm::vec4(0, 0, 1, 0)));

        glm::vec3 frameMin(0.0f);
        glm::vec3 frameMax(0.0f);

        if (MeasureMeshBounds(mFrameNode->GetStaticMesh(), frameMin, frameMax))
        {
            const glm::vec3 front = glm::vec3(frameToWorld * glm::vec4(frameMin.x, frameMin.y, frameMin.z, 1.0f));
            const glm::vec3 back = glm::vec3(frameToWorld * glm::vec4(frameMin.x, frameMin.y, frameMax.z, 1.0f));

            mTrayDistance = glm::length(back - front) * kPullDepthMul;
        }
    }

    // The table top is the underside of the stand -- it is what the board is standing on, so it is
    // exactly the height a disc should come to rest at.
    //
    // The fallback has to be BELOW the board. Falling back to the bottom row meant a disc dropped a
    // few centimetres, landed back inside the frame it was supposed to be emptying out of, and
    // stopped there looking stuck, which is indistinguishable from the fall being broken. Start
    // from a height that is clearly under the board and let the stand refine it.
    const float bottomRowY = mLayout.GetStillPoint(0, 0).y;

    // Anchor on the frame's own bottom rather than on the bottom row. The row is only a few
    // centimetres above the base of the frame, so a fallback measured from it lands the discs
    // inside the board -- which is what "a bit below the board" turned out to mean in practice.
    // The frame's extent is measured the same way the hole positions are, and those are demonstrably
    // right, so it is the one distance here that can be relied on.
    float frameBottomY = bottomRowY;
    float frameHeightWorld = mLayout.GetRowSpacing() * float(C4::kRows);

    {
        const glm::mat4& frameToWorld = mFrameNode->GetTransform();

        glm::vec3 fMin(0.0f);
        glm::vec3 fMax(0.0f);

        if (MeasureMeshBounds(mFrameNode->GetStaticMesh(), fMin, fMax))
        {
            float lowest = 1e9f;
            float highest = -1e9f;

            for (int c = 0; c < 8; ++c)
            {
                const glm::vec3 corner(
                    (c & 1) ? fMax.x : fMin.x,
                    (c & 2) ? fMax.y : fMin.y,
                    (c & 4) ? fMax.z : fMin.z);

                const float y = glm::vec3(frameToWorld * glm::vec4(corner, 1.0f)).y;
                lowest = glm::min(lowest, y);
                highest = glm::max(highest, y);
            }

            frameBottomY = lowest;
            frameHeightWorld = highest - lowest;
        }
    }

    mTableY = frameBottomY - frameHeightWorld * 0.20f;
    mFrameBottomY = frameBottomY;

    if (mStandNode != nullptr)
    {
        StaticMesh3D* standMesh = mStandNode->As<StaticMesh3D>();

        glm::vec3 standMin(0.0f);
        glm::vec3 standMax(0.0f);

        if (standMesh != nullptr && MeasureMeshBounds(standMesh->GetStaticMesh(), standMin, standMax))
        {
            const glm::mat4& standToWorld = mStandNode->GetTransform();

            // Take the lowest corner of the box in world space; which local corner that is depends
            // on how the stand has been rotated, so test them all rather than guess.
            float lowest = 1e9f;

            for (int c = 0; c < 8; ++c)
            {
                const glm::vec3 corner(
                    (c & 1) ? standMax.x : standMin.x,
                    (c & 2) ? standMax.y : standMin.y,
                    (c & 4) ? standMax.z : standMin.z);

                lowest = glm::min(lowest, glm::vec3(standToWorld * glm::vec4(corner, 1.0f)).y);
            }

            // Only trust it if it is below the bottom of the FRAME, not merely below the bottom
            // row. The row sits just above the frame's base, so "below the bottom row" is a test
            // almost anything passes, including a measurement that still leaves the discs inside
            // the board.
            if (lowest < frameBottomY)
            {
                mTableY = lowest;
            }
            else
            {
                LogWarning("Connect4: stand measured above the frame base (%.3f vs %.3f); "
                           "using a fallback table height.", lowest, frameBottomY);
            }
        }
    }

    // A disc resting on the table sits half its thickness above it. The thickness is the smallest
    // of the chip's dimensions, whichever axis the model happens to use for it.
    //
    // Measured through the chip's world transform, not by multiplying the mesh by the node's own
    // scale. The node's scale is relative to its parent, and that parent is scaled to about a
    // hundredth here, so using it gave a disc a hundred times too thick -- which put the resting
    // height a good fraction of the board above the table. Discs never fell out: they were snapped
    // straight back up to that height as soon as they dropped to it.
    if (templateChip != nullptr)
    {
        glm::vec3 discMin(0.0f);
        glm::vec3 discMax(0.0f);

        if (MeasureMeshBounds(mDiscMesh, discMin, discMax))
        {
            const glm::mat4& chipToWorld = templateChip->GetTransform();

            glm::vec3 worldMin(1e9f);
            glm::vec3 worldMax(-1e9f);

            for (int c = 0; c < 8; ++c)
            {
                const glm::vec3 corner(
                    (c & 1) ? discMax.x : discMin.x,
                    (c & 2) ? discMax.y : discMin.y,
                    (c & 4) ? discMax.z : discMin.z);

                const glm::vec3 w = glm::vec3(chipToWorld * glm::vec4(corner, 1.0f));
                worldMin = glm::min(worldMin, w);
                worldMax = glm::max(worldMax, w);
            }

            const glm::vec3 discSize = worldMax - worldMin;
            mDiscRestOffset = glm::min(glm::min(discSize.x, discSize.y), discSize.z) * 0.5f;

            // The thin direction is the one through the flat of the disc. Measured on the model's
            // own axes rather than in world, since that is the axis a rotation has to be built
            // around to lay it down.
            const glm::vec3 localSize = discMax - discMin;

            if (localSize.x <= localSize.y && localSize.x <= localSize.z)
                mDiscFaceAxis = 0;
            else if (localSize.y <= localSize.x && localSize.y <= localSize.z)
                mDiscFaceAxis = 1;
            else
                mDiscFaceAxis = 2;
        }
    }

    LogDebug("C4: lift %.3f pull %.3f tableY %.3f frameBot %.3f rowY0 %.3f",
             mLiftDistance, mTrayDistance, mTableY, frameBottomY, bottomRowY);

    // Prefer a real yellow chip if one has been put in the scene, so its own texture is used
    // rather than an approximation of it.
    StaticMesh3D* yellowChip = root->FindChild<StaticMesh3D>("Chip_Yellow", true);

    if (yellowChip != nullptr)
    {
        mYellowMaterial = yellowChip->GetMaterial();

        // It is only there to be sampled; the pool draws the actual discs.
        yellowChip->SetVisible(false);
    }

    // Failing that, take the yellow chip's material straight from the assets. The chip only has to
    // have been imported, not placed in the scene -- the pool draws the discs, so an instance of it
    // sitting in the level serves no purpose beyond being somewhere to read the material from.
    if (mYellowMaterial == nullptr)
    {
        mYellowMaterial = LoadAsset<Material>("M_Chip_Yellow_YellowChips");
    }

    // Otherwise tint the red material. MaterialLite multiplies its colour over the texture, so the
    // chip keeps its moulding and wear instead of turning into a flat yellow disc.
    if (mYellowMaterial == nullptr && mRedMaterial != nullptr)
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

    Renderer* renderer = Renderer::Get();

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

        // Drive the loading screen as the board is built. Every few discs rather than every one,
        // since a frame costs far more than spawning a node and the point is to show progress, not
        // to render forty-two frames.
        //
        // The message overrides the engine's own "Loading...", which is what was on screen through
        // the startup asset load just before this. The logo above it is not set here: it is named
        // in Config.ini as LoadingScreenLogo, because the screen is already up before any of this
        // project's code has run.
        if (renderer != nullptr && (i % 6) == 0)
        {
            const float progress = float(i) / float(C4::kCols * C4::kRows);
            renderer->DrawLoadingFrame(progress, "Setting up the board...");
        }
    }

    if (renderer != nullptr)
    {
        renderer->DrawLoadingFrame(1.0f, "Setting up the board...");
        renderer->EnableLoadingScreen(false);
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

    // These are moved by script every frame. If the mesh brought collision in with it and anything
    // started simulating them, the two would fight and the result would look like a disc jamming.
    disc->EnablePhysics(false);
    disc->EnableCollision(false);

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

    UpdateRerack(deltaTime);
}

void Connect4Scene::UpdateRerack(float deltaTime)
{
    if (mRerackPhase == RerackPhase::Idle)
    {
        return;
    }

    mRerackTime += deltaTime;

    switch (mRerackPhase)
    {
    case RerackPhase::Lift:
    {
        // The grid rises out of its stand, carrying the discs with it -- they are still sitting in
        // their slots at this point, held up by the tray underneath.
        const float t = EaseInOut(mRerackTime / kLiftTime);
        const glm::vec3 offset = mLiftAxis * (mLiftDistance * t);

        mFrameNode->SetWorldPosition(mFrameHome + offset);

        for (uint32_t i = 0; i < C4::kCols * C4::kRows; ++i)
        {
            Disc& disc = mDiscs[i];

            if (disc.mInUse && disc.mNode != nullptr)
            {
                disc.mNode->SetWorldPosition(disc.mTo + offset);
            }
        }

        if (mRerackTime >= kLiftTime)
        {
            // Remember where each disc ended up, so the fall starts from where it is rather than
            // from where it originally landed.
            for (uint32_t i = 0; i < C4::kCols * C4::kRows; ++i)
            {
                if (mDiscs[i].mInUse)
                {
                    mDiscs[i].mTo += offset;
                }
            }

            mRerackPhase = RerackPhase::PullTray;
            mRerackTime = 0.0f;
        }

        break;
    }

    case RerackPhase::PullTray:
    {
        // The release slides out from under the discs. Nothing falls yet: the pull is the moment,
        // and it reads better if the discs wait for it to finish.
        if (mTrayNode != nullptr)
        {
            const float t = EaseInOut(mRerackTime / kPullTime);
            mTrayNode->SetWorldPosition(mTrayHome + mTrayAxis * (mTrayDistance * t));
        }

        if (mRerackTime >= kPullTime)
        {
            mRerackPhase = RerackPhase::Fall;
            mRerackTime = 0.0f;
        }

        break;
    }

    case RerackPhase::Fall:
    {
        const float spacing = mLayout.GetRowSpacing();
        const float gravity = kRerackGravity * spacing;
        const float restY = mTableY + mDiscRestOffset;
        const float restSpeed = kDiscRestSpeed * spacing;

        // The underside of the grid where it now stands, having been lifted. A disc is out of the
        // board once it is below this.
        const float exitY = mFrameBottomY + mLiftDistance;

        bool anyMoving = false;

        for (uint32_t i = 0; i < C4::kCols * C4::kRows; ++i)
        {
            Disc& disc = mDiscs[i];

            if (!disc.mInUse || disc.mNode == nullptr)
            {
                continue;
            }

            glm::vec3 pos = disc.mNode->GetWorldPosition();

            // Settled, and finished falling over: leave it exactly where it stopped rather than
            // letting it creep. A disc still toppling has to be allowed through -- it has already
            // stopped moving, so skipping it here left it set up to fall over and never advanced,
            // which is why they stayed standing on their edges.
            const bool settled = (pos.y <= restY + 0.0001f && glm::length(disc.mVelocity) < restSpeed);
            const bool toppling = (disc.mFlatT >= 0.0f && disc.mFlatT < 1.0f);

            if (settled && !toppling)
            {
                continue;
            }

            if (settled)
            {
                // Only the topple is left to run.
                disc.mFlatT = glm::min(disc.mFlatT + deltaTime / kToppleTime, 1.0f);
                disc.mNode->SetWorldRotation(
                    glm::slerp(disc.mRotFrom, disc.mRotTo, EaseInOut(disc.mFlatT)));

                anyMoving = true;
                continue;
            }

            anyMoving = true;

            // Clear of the board? Then it can start to spread. Until it is, only gravity acts on
            // it, so it drops down the column and out through the bottom.
            if (!disc.mCleared && pos.y <= exitY)
            {
                disc.mCleared = true;

                // mFrom is holding the spread this disc was given when it was released; it is not
                // needed as a start point once the fall is under way.
                disc.mVelocity.x = disc.mFrom.x;
                disc.mVelocity.z = disc.mFrom.z;
            }

            disc.mVelocity.y -= gravity * deltaTime;
            pos += disc.mVelocity * deltaTime;

            if (pos.y < restY)
            {
                pos.y = restY;

                // Bounce, giving back a fraction of the impact and scrubbing off horizontal speed
                // as it skids. Discs land flat and do not bounce much, so the numbers are low.
                disc.mVelocity.y = -disc.mVelocity.y * kDiscRestitution;
                disc.mVelocity.x *= kDiscFriction;
                disc.mVelocity.z *= kDiscFriction;
                disc.mSpin *= kDiscFriction;

                if (glm::abs(disc.mVelocity.y) < restSpeed)
                {
                    disc.mVelocity = glm::vec3(0.0f);
                    disc.mSpin = 0.0f;

                    // Down flat. A disc leaves the board standing on edge, the way it sat in its
                    // slot, and a disc on edge does not stay there.
                    if (disc.mFlatT < 0.0f)
                    {
                        disc.mRotFrom = disc.mNode->GetWorldRotationQuat();
                        disc.mRotTo = FlatRotation(mDiscFaceAxis, disc.mSpin + float(i) * 37.0f);
                        disc.mFlatT = 0.0f;
                    }
                }
            }

            disc.mNode->SetWorldPosition(pos);

            // Tumble only once it is out of the board. Spinning while still between the slats made
            // it look as though the frame were not there at all.
            if (disc.mCleared && disc.mFlatT < 0.0f)
            {
                disc.mNode->SetRotation(disc.mNode->GetRotationEuler() +
                                        glm::vec3(0.0f, 0.0f, disc.mSpin * deltaTime));
            }

            if (disc.mFlatT >= 0.0f && disc.mFlatT < 1.0f)
            {
                disc.mFlatT = glm::min(disc.mFlatT + deltaTime / kToppleTime, 1.0f);
                disc.mNode->SetWorldRotation(
                    glm::slerp(disc.mRotFrom, disc.mRotTo, EaseInOut(disc.mFlatT)));
            }
        }

        // Move on once they have stopped, rather than after a fixed time, so the board is never
        // reset out from under a disc still rolling. The time limit is only a backstop.
        if (!anyMoving || mRerackTime > 4.0f)
        {
            mRerackPhase = RerackPhase::Settle;
            mRerackTime = 0.0f;
        }

        break;
    }

    case RerackPhase::Settle:
    {
        // A beat with the discs lying on the table before the board goes back together.
        if (mRerackTime >= kSettleTime)
        {
            ResetBoardParts();
            ClearDiscs();
            mRerackPhase = RerackPhase::Idle;
        }

        break;
    }

    default:
        break;
    }
}

void Connect4Scene::ResetBoardParts()
{
    // Exactly the positions they started at, not an approximation of them: the layout was measured
    // through the frame's transform, so letting the frame come back a fraction low would put every
    // still point out by the same amount for the next game.
    if (mFrameNode != nullptr)
    {
        mFrameNode->SetWorldPosition(mFrameHome);
    }

    if (mTrayNode != nullptr)
    {
        mTrayNode->SetWorldPosition(mTrayHome);
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

    mRerackPhase = RerackPhase::Lift;
    mRerackTime = 0.0f;

    HideCursorDisc();

    // Point the pull at whoever is watching. The board's facing axis is a line, not a direction --
    // which of its two ends is the front depends on how the board was rotated in the editor -- so
    // resolve it against the camera rather than guessing, and the tray comes towards the player
    // however the board has been placed.
    if (mTrayNode != nullptr)
    {
        World* world = GetWorld(0);
        Camera3D* camera = world ? world->GetActiveCamera() : nullptr;

        if (camera != nullptr)
        {
            const glm::vec3 toCamera = camera->GetWorldPosition() - mTrayNode->GetWorldPosition();

            // Away from the camera: the tray goes back under the board rather than out towards
            // the player.
            if (glm::dot(mTrayAxis, toCamera) > 0.0f)
            {
                mTrayAxis = -mTrayAxis;
            }

            // The discs go the other way, towards the player, and level rather than tilted up at
            // the camera.
            mSpillAxis = -mTrayAxis;
            mSpillAxis.y = 0.0f;

            if (glm::length(mSpillAxis) > 0.0001f)
            {
                mSpillAxis = glm::normalize(mSpillAxis);
            }
            else
            {
                mSpillAxis = glm::vec3(0.0f);
            }

            // Across the table: square to the spill and level, so most of the scatter runs along
            // the width of the table, where there is room for it and nothing to clip against.
            mSpreadAxis = glm::cross(glm::vec3(0.0f, 1.0f, 0.0f), mSpillAxis);

            if (glm::length(mSpreadAxis) > 0.0001f)
            {
                mSpreadAxis = glm::normalize(mSpreadAxis);
            }
            else
            {
                mSpreadAxis = glm::vec3(1.0f, 0.0f, 0.0f);
            }
        }
    }

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

        // Straight down to begin with. A disc still inside the board can only go one way -- down
        // its own column and out of the bottom -- and letting it drift sideways immediately sent it
        // out through the plastic instead of through the opening the tray just uncovered.
        //
        // The spread is held here and applied the moment it clears the frame.
        disc.mVelocity = glm::vec3(0.0f);
        disc.mCleared = false;
        disc.mFlatT = -1.0f;

        // Spread mostly across the table rather than towards the player. Sending them at the
        // camera walked the front row into the near clip plane, which cuts geometry on a flat
        // plane square to the view and sliced the closest discs in half. There is also far more
        // table to land on sideways than there is in front of the board.
        //
        // Fast enough to travel: the fall out of the board lasts under a second, so a drift of a
        // fraction of a cell per second moves a disc less than its own radius.
        disc.mFrom = mSpillAxis * (spacing * 0.8f)
                   + mSpreadAxis * (rx * spacing * 4.5f)
                   + glm::vec3(0.0f, 0.0f, rz * spacing * 0.6f);

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
        mDiscs[i].mCleared = false;
        mDiscs[i].mFlatT = -1.0f;
    }

    mNumDiscsUsed = 0;
    mNumWinDiscs = 0;
    mDroppingDisc = -1;
    mDropActive = false;
}
