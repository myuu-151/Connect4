#include "Connect4Scene.h"

#include "Engine.h"
#include "World.h"
#include "Log.h"
#include "AssetManager.h"
#include "AudioManager.h"
#include "Assets/SoundWave.h"
#include "Renderer.h"
#include "Enums.h"
#include "Assets/StaticMesh.h"
#include "Assets/MaterialLite.h"
#include "Nodes/Node.h"
#include "Nodes/3D/Node3d.h"
#include "Nodes/3D/StaticMesh3d.h"
#include "Nodes/3D/Camera3d.h"
#include "Nodes/3D/Box3d.h"

#include "BulletCollision/CollisionShapes/btCylinderShape.h"
#include "BulletCollision/CollisionShapes/btConvexInternalShape.h"

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

// Gravity, bounce, friction and when a disc counts as stopped are Bullet's now, set per body in
// StartDiscPhysics rather than hand-integrated here.
//
// How much of the world's gravity the discs feel. Below 1 because the board is only centimetres
// across in world units: at full gravity a disc falls its own height in a few hundredths of a
// second, which is correct and unwatchable.
const float kDiscGravityScale = 0.45f;

// How long a disc has to go nowhere before it is taken out of the simulation.
//
// This was half a second, which took discs out while they were still settling -- a pile relaxes
// for a good while after it stops obviously moving, and cutting that short is most of why the
// result did not look simulated. It is a backstop against a disc that never settles, not a way of
// tidying up, so it should be long enough that a disc reaching it is genuinely stuck.
const float kDiscRetireTime = 2.0f;

// The longest any disc is simulated for. A backstop against one that never settles.
const float kDiscMaxLiveTime = 8.0f;

// What a disc weighs. Only meaningful against the other masses in the scene, and everything else
// here is static, so this is really just "light".
const float kDiscMass = 0.05f;

// The collision group the rerack runs in.
//
// Discs collide with each other, with the table and with the stand's feet, and with nothing else.
// Everything in the scene has collision by default -- the frame, the stand, the room -- and a disc
// sitting in its slot is inside the frame's own collision hull. Left on the default group the
// discs were confined by the board they were supposed to be falling out of: they could not move
// sideways at all, only shuffle where they stood while the solver pushed them out of the hull, and
// then drop once the grid lifted clear of them.
//
// That is what "they barely move, then drop" was, and no amount of pushing them harder could have
// helped, because they had nowhere to go.
const uint8_t kRerackColGroup = ColGroup3;

// How hard a disc left standing on its edge is leaned on, in radians per second squared. Enough to
// get it past its balance point in a fraction of a second; gravity does the rest, which is the
// whole point of doing this rather than rotating it by hand.
const float kToppleAccel = 4.0f;

// How many discs are simulated at once.
//
// Not a performance figure but a memory limit. Bullet allocates solver bodies and contact arrays
// for every body in an island, and a heap of discs settling together is one island; somewhere
// around thirty-nine of them the allocation fails, and it fails by writing through a null pointer
// rather than by complaining.
//
// So this is set just under where it breaks rather than comfortably below it. Most games do not
// fill the board anyway -- a win usually lands well before forty-two discs are down -- so in
// practice the whole board goes at once and the queue never comes into it. Only a draw, or very
// nearly one, releases in two waves.
//
// How many discs are simulated at once.
//
// A memory limit, not a performance one. The solver's contact constraint pool is sized by how many
// contact points exist at that instant, and a heap of discs asks for a single large contiguous
// block that this machine does not reliably have -- which is why a rerack of thirty-nine could
// fail after one of forty-two succeeded. It depends on how badly the pile tangles, not on how many
// discs are in it.
//
// The whole board, so a rerack is always one drop.
//
// This is only safe with memory to spare: the solver's contact pool wants a single large contiguous
// block when the pile is at its most tangled, and it is the allocation for that which fails. The
// warm-up reserves what it can at startup, but it cannot reserve more than exists.
//
// If a full board starts crashing again, this is the number to lower -- but the real fix is to stop
// large textures sitting in memory as RGBA8 when RGB5A3 holds them at half the size with their
// alpha intact.
const uint32_t kMaxLiveDiscs = C4::kCols * C4::kRows;

// How many frames the startup warm-up runs for.
// Long enough for the heap to actually form and its contacts to peak. Too few and the discs are
// still in the air, touching nothing, when it is switched off again.
const int32_t kWarmUpFrames = 45;

// How long a retired disc takes to fall flat.
const float kRetireToppleTime = 0.28f;

const glm::vec4 kRedTint = glm::vec4(1.00f, 1.00f, 1.00f, 1.0f);   // the mesh is already red
const glm::vec4 kYellowTint = glm::vec4(2.05f, 1.62f, 0.22f, 1.0f);

// Where a disc ends up once it has fallen over: face up, reached by the shortest tip from wherever
// it is. Taking the shortest arc is what makes it look like falling rather than being turned --
// it follows whichever way the disc is already leaning.
glm::quat ToppleRotation(const glm::quat& current, int32_t faceAxis)
{
    glm::vec3 localNormal(0.0f);
    localNormal[glm::clamp(faceAxis, 0, 2)] = 1.0f;

    glm::vec3 normal = current * localNormal;

    const float normalLen = glm::length(normal);
    normal = (normalLen > 0.0001f) ? (normal / normalLen) : glm::vec3(0.0f, 1.0f, 0.0f);

    const glm::vec3 up(0.0f, 1.0f, 0.0f);

    // Either face can end up on top, so tip towards whichever is nearer.
    const glm::vec3 target = (glm::dot(normal, up) < 0.0f) ? -up : up;

    glm::vec3 axis = glm::cross(normal, target);

    if (glm::length(axis) < 0.001f)
    {
        return current;   // already flat
    }

    const float angle = acosf(glm::clamp(glm::dot(normal, target), -1.0f, 1.0f));

    return glm::angleAxis(angle, glm::normalize(axis)) * current;
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

            mDiscHalfThickness = glm::min(glm::min(discSize.x, discSize.y), discSize.z) * 0.5f;
            mDiscRadius = glm::max(glm::max(discSize.x, discSize.y), discSize.z) * 0.5f;

            const glm::vec3 localSize = discMax - discMin;
            mDiscLocalHalfThickness = glm::min(glm::min(localSize.x, localSize.y), localSize.z) * 0.5f;
            mDiscLocalRadius = glm::max(glm::max(localSize.x, localSize.y), localSize.z) * 0.5f;

            // The thin direction is the one through the flat of the disc. Measured on the model's
            // own axes, since that is the axis the collision cylinder has to be built around.
            if (localSize.x <= localSize.y && localSize.x <= localSize.z)
                mDiscFaceAxis = 0;
            else if (localSize.y <= localSize.x && localSize.y <= localSize.z)
                mDiscFaceAxis = 1;
            else
                mDiscFaceAxis = 2;
        }
    }

    BuildObstacles();
    BuildPhysicsColliders();

    // Sounds are optional. A name that is not there leaves that one silent rather than failing the
    // whole scene, so the game still runs while the audio is being put together.
    mSlotBeginSound = LoadAsset<SoundWave>("slot_begin");
    mSlotEndSound = LoadAsset<SoundWave>("slot_end");

    // rerack2 rather than rerack: mono at 22 kHz instead of stereo at 44, which is a quarter of
    // the decoded PCM for a clatter nobody is listening to closely. The original ran the machine
    // out of memory -- what costs is the decoded size, not the file, and these are the last assets
    // loaded, so they are the ones that find nothing left.
    mRerackSound = LoadAsset<SoundWave>("rerack2");

    if (mRerackSound == nullptr)
    {
        mRerackSound = LoadAsset<SoundWave>("rerack");
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

    // Drop the whole set once, out of sight, so Bullet sizes its solver arrays now rather than
    // during a rerack when there is no memory left to size them with.
    {
        const glm::vec3 above = mLayout.GetStillPoint(C4::kCols / 2, C4::kRows - 1) +
                                glm::vec3(0.0f, mLayout.GetRowSpacing() * 2.0f, 0.0f);

        for (uint32_t i = 0; i < C4::kCols * C4::kRows; ++i)
        {
            Disc& disc = mDiscs[i];

            if (disc.mNode == nullptr)
            {
                continue;
            }

            // A plausible heap, not a manufactured worst case.
            //
            // An earlier version packed them deliberately overlapping to force the solver to
            // reserve for the worst tangle imaginable. It did exactly that, and asked for more
            // memory than the machine has -- it crashed during the warm-up itself, having made the
            // problem it was meant to solve twice as large.
            //
            // Three narrow columns: they collapse into each other on the way down and settle into
            // something like the pile a rerack makes, which is what needs to fit.
            const float across = mDiscRadius * 0.7f;
            const glm::vec3 offset(((i % 3) - 1) * across,
                                   mDiscRadius * 1.1f * float(i / 3),
                                   (((i / 3) % 3) - 1) * across);

            disc.mInUse = true;
            disc.mFrom = glm::vec3(0.0f);
            disc.mNode->SetWorldPosition(above + offset);
            disc.mNode->SetVisible(false);
        }

        StartDiscPhysics();

        for (uint32_t i = 0; i < C4::kCols * C4::kRows; ++i)
        {
            if (mDiscs[i].mAwaitingRelease)
            {
                ReleaseDisc(mDiscs[i], i);
            }
        }

        // Stay invisible through all of it.
        for (uint32_t i = 0; i < C4::kCols * C4::kRows; ++i)
        {
            if (mDiscs[i].mNode != nullptr)
            {
                mDiscs[i].mNode->SetVisible(false);
            }
        }

        mWarmUpFrames = kWarmUpFrames;
    }

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

    // Squarely in its slot, whatever the disc was doing last time it was used. Discs come back from
    // a rerack at every angle, and one reused without this would drop into the board turned.
    disc.mNode->SetRotation(mDiscRotation);

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

    // Outside the rerack's phases on purpose. The discs are left simulating once it has nominally
    // finished so the result can be watched for as long as anyone likes, which means stragglers
    // still have to be retired and laid down after the phases are over.
    // Before anything else: while this is running the discs are not the game's, they are a heap
    // being dropped to make Bullet allocate.
    if (mWarmUpFrames > 0)
    {
        WarmUpSolver();
        return;
    }

    UpdateDiscRelease();
    TipOverIfStanding(deltaTime);
    UpdateDiscRetirement(deltaTime);
    UpdateToppling(deltaTime);
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
        // Bullet has it from here. Everything up to this point was placed by hand -- the discs were
        // held in the grid while it lifted -- and the moment the tray is out of the way they become
        // rigid bodies and fall, land, bounce off the stand's feet and off each other.
        //
        // This is the one part of the game that is simulated. The board is decided by the rules
        // long before any of it runs, so nothing here can affect the outcome; it only has to look
        // right, which is exactly what physics is good at and what hand-written motion kept getting
        // wrong -- discs landed flat every time because they were told to, with no contacts to
        // decide otherwise.
        if (!mDiscPhysicsRunning)
        {
            StartDiscPhysics();
        }

        // The clatter plays on the first disc to reach the table, not when they are let go. They
        // are released at the top of the board and fall for the best part of a second, so playing
        // it on release put the sound well ahead of anything hitting anything.
        if (!mRerackSoundPlayed)
        {
            const float contactY = mTableY + mDiscRadius * 2.5f;

            for (uint32_t i = 0; i < C4::kCols * C4::kRows; ++i)
            {
                const Disc& disc = mDiscs[i];

                if (disc.mInUse &&
                    disc.mNode != nullptr &&
                    disc.mNode->GetWorldPosition().y <= contactY)
                {
                    PlayRerackSound();
                    mRerackSoundPlayed = true;
                    break;
                }
            }
        }

        // Give them a moment before checking: they start slow, and asking immediately would find
        // everything below the threshold and finish before anything had fallen.
        const bool longEnough = (mRerackTime > 0.5f);

        if ((longEnough && AreDiscsAsleep()) || mRerackTime > 3.5f)
        {
            // Deliberately without stopping the simulation. Anything still moving carries on --
            // the board now waits for a button before it is set up again, so the last discs to
            // settle should be allowed to finish rather than being frozen at the moment the rest
            // of them happen to be still. Discs are retired one at a time as they come to rest,
            // which is what keeps the cost down; there is nothing to be gained by switching off
            // the ones that are still doing something.
            mRerackPhase = RerackPhase::Settle;
            mRerackTime = 0.0f;
        }

        break;
    }

    case RerackPhase::Settle:
    {
        // The discs are left lying where they fell and the grid stays open. Clearing them here
        // meant the result of the simulation was on screen for about a second before being tidied
        // away; the game now decides when to put the board back, so it can be looked at for as
        // long as anyone wants.
        if (mRerackTime >= kSettleTime)
        {
            mRerackPhase = RerackPhase::Idle;
        }

        break;
    }

    default:
        break;
    }
}

void Connect4Scene::PlayDropSound()
{
    if (mSlotBeginSound != nullptr)
    {
        AudioManager::PlaySound2D(mSlotBeginSound);
    }
}

void Connect4Scene::PlayLandSound()
{
    if (mSlotEndSound != nullptr)
    {
        AudioManager::PlaySound2D(mSlotEndSound);
    }
}

void Connect4Scene::PlayRerackSound()
{
    if (mRerackSound != nullptr)
    {
        AudioManager::PlaySound2D(mRerackSound);
    }
}

void Connect4Scene::BuildPhysicsColliders()
{
    World* world = GetWorld(0);
    Node* root = world ? world->GetRootNode() : nullptr;

    if (root == nullptr || mDiscRadius <= 0.0f)
    {
        return;
    }

    Node3D* root3d = root->As<Node3D>();

    if (root3d == nullptr)
    {
        return;
    }

    auto makeStaticBox = [&](const char* name, glm::vec3 center, glm::vec3 extents) -> Box3D*
    {
        Box3D* box = root3d->CreateChild<Box3D>();

        if (box == nullptr)
        {
            return nullptr;
        }

        box->SetName(name);
        box->SetExtents(extents);
        box->SetWorldPosition(center);

        // Mass zero is what makes a body static in Bullet: it collides and never moves.
        box->SetMass(0.0f);
        box->SetFriction(0.7f);
        box->SetRestitution(0.1f);

        // Only the discs, so these invisible boxes are not in the way of anything else in the room.
        box->SetCollisionGroup(kRerackColGroup);
        box->SetCollisionMask(kRerackColGroup);

        box->EnableCollision(true);
        box->EnablePhysics(true);

        // And the same margin correction the discs get.
        //
        // A margin inflates a static box as surely as it fattens a disc, so the default 0.04 left
        // the table's surface sitting that far above where it looks like it is -- further above it
        // than a disc is wide. Discs came to rest hovering over the table and over the stand's
        // feet, on a surface that was not where it appeared to be.
        btCollisionShape* boxShape = box->GetCollisionShape();

        if (boxShape != nullptr && boxShape->isConvex())
        {
            btConvexInternalShape* convex = static_cast<btConvexInternalShape*>(boxShape);

            // What the box would be if nothing had been taken out of it for the margin.
            const btScalar oldMargin = convex->getMargin();
            const btVector3 trueHalfExtents = convex->getImplicitShapeDimensions() +
                                              btVector3(oldMargin, oldMargin, oldMargin);

            const btScalar newMargin = btMax(btScalar(trueHalfExtents[trueHalfExtents.minAxis()] * 0.1f),
                                             btScalar(0.00001f));

            convex->setMargin(newMargin);
            convex->setImplicitShapeDimensions(trueHalfExtents - btVector3(newMargin, newMargin, newMargin));
        }

        // Present for collision only; there is already a table and a stand to look at.
        box->SetVisible(false);

        return box;
    };

    // The table top. Sized to roughly the real table, not to something comfortably enormous: the
    // first version was twenty-four columns across, so there was invisible floor everywhere and a
    // disc could never reach an edge to fall off one. Discs going over the side is half the point
    // of tipping a board out.
    //
    // Measured as a multiple of the board, since the table belongs to the room's mesh and cannot
    // be picked out of it. These are the two numbers to nudge if discs stop short of the real edge
    // or hang in the air past it.
    const float boardWidth = mLayout.GetColSpacing() * float(C4::kCols - 1);
    const float tableWidth = boardWidth * 1.45f;
    const float tableDepth = boardWidth * 0.80f;
    const float tableThickness = mDiscRadius * 8.0f;

    const glm::vec3 boardCenter = mLayout.GetStillPoint(C4::kCols / 2, 0);

    mGroundCollider = makeStaticBox(
        "RerackGround",
        glm::vec3(boardCenter.x, mTableY - tableThickness * 0.5f, boardCenter.z),
        glm::vec3(tableWidth, tableThickness, tableDepth));

    // Well below, so a disc that goes over the edge lands somewhere instead of falling for ever.
    // Without it nothing ever comes to rest and the rerack only ends on its timeout.
    makeStaticBox(
        "RerackFloor",
        glm::vec3(boardCenter.x, mTableY - boardWidth * 1.2f, boardCenter.z),
        glm::vec3(boardWidth * 8.0f, tableThickness, boardWidth * 8.0f));

    // The feet, already measured for their boxes.
    for (uint32_t i = 0; i < mNumObstacles && i < 2; ++i)
    {
        const glm::vec3 center = (mObstacles[i].mMin + mObstacles[i].mMax) * 0.5f;
        const glm::vec3 extents = mObstacles[i].mMax - mObstacles[i].mMin;

        char name[24];
        snprintf(name, sizeof(name), "RerackFoot%u", i);

        mFootColliders[i] = makeStaticBox(name, center, extents);
    }

    LogDebug("C4: colliders ground %s feet %u",
             mGroundCollider ? "ok" : "FAILED", mNumObstacles);
}

// Hand the discs over to Bullet. Up to this point they have been placed by hand -- they were held
// in the grid while it lifted -- so their bodies are started from wherever they currently are.
// Hand the discs to Bullet, a few at a time.
//
// They are queued here rather than all released at once. Bullet sizes its solver arrays by the
// number of bodies in an island, and forty-two discs coming to rest in one heap is a single island
// large enough to exhaust the machine -- it ran out of memory inside the solver and wrote through
// the failed allocation. Keeping a bound on how many are live at any moment bounds that.
void Connect4Scene::StartDiscPhysics()
{
    World* world = GetWorld(0);

    for (uint32_t i = 0; i < C4::kCols * C4::kRows; ++i)
    {
        Disc& disc = mDiscs[i];

        if (disc.mInUse && disc.mNode != nullptr)
        {
            disc.mAwaitingRelease = true;
            disc.mSlowTime = 0.0f;
            disc.mLiveTime = 0.0f;
        }
    }

    // Slightly fewer solver iterations while this is running.
    //
    // This was four, which is where most of the trouble came from: a pile solved that loosely
    // sinks into itself, jitters, and will not stack, and every correction added on top of that was
    // treating a symptom of it. Eight is close enough to the default to behave properly and still
    // cheaper than ten at the heaviest moment in the game.
    if (world != nullptr && world->GetDynamicsWorld() != nullptr)
    {
        btContactSolverInfo& solverInfo = world->GetDynamicsWorld()->getSolverInfo();

        mSavedSolverIterations = solverInfo.m_numIterations;
        solverInfo.m_numIterations = 8;
    }

    mDiscPhysicsRunning = true;
}

// Let go of the next few discs whenever there is room for them.
//
// Lowest first, so the board empties from the bottom the way a real one does, and so the discs
// already on the table are the ones supporting whatever comes down next.
// Make Bullet reserve the solver memory a full rerack needs, while the game is still loading.
//
// A rerack used to crash: the solver allocates its arrays from the number of bodies in an island,
// a heap of discs settling together is one island, and somewhere around thirty-nine of them the
// allocation failed -- silently, by writing through a null pointer. The machine is at its memory
// ceiling by the time a game is running, and worse, the heap is fragmented by everything the scene
// loaded, so a single large contiguous request can fail with plenty of total memory free.
//
// The arrays grow and are never shrunk, so if they are grown once while memory is still clean the
// capacity is there for the rest of the session and a rerack never has to allocate at all.
//
// It has to be a real pile. Islands are built from contact manifolds, so bodies that are not
// touching produce no island and the solver reserves nothing -- warming up during the lift, while
// the discs are still sitting apart in their slots, would have allocated nothing at all. So every
// disc is genuinely dropped into a heap here, invisibly, and switched off again a few frames later.
void Connect4Scene::WarmUpSolver()
{
    if (mWarmUpFrames <= 0)
    {
        return;
    }

    mWarmUpFrames--;

    if (mWarmUpFrames > 0)
    {
        return;
    }

    // Done: put everything back as it was. The discs were never visible and the board has not
    // started, so nothing here is observable except the memory that is now reserved.
    for (uint32_t i = 0; i < C4::kCols * C4::kRows; ++i)
    {
        Disc& disc = mDiscs[i];

        if (disc.mNode != nullptr)
        {
            disc.mNode->EnablePhysics(false);
            disc.mNode->EnableCollision(false);
            disc.mNode->SetVisible(false);

            // Back to the orientation the chip was placed at. The warm-up flings them about, and
            // without this they return to the pool holding whatever angle they were thrown into --
            // so the first discs of the game appeared in their slots tilted.
            disc.mNode->SetRotation(mDiscRotation);
        }

        disc.mInUse = false;
        disc.mAwaitingRelease = false;
        disc.mSlowTime = 0.0f;
        disc.mLiveTime = 0.0f;
        disc.mFlatT = -1.0f;
    }

    mDiscPhysicsRunning = false;

    LogDebug("C4: solver warm-up done");
}

void Connect4Scene::UpdateDiscRelease()
{
    if (!mDiscPhysicsRunning)
    {
        return;
    }

    uint32_t live = 0;

    for (uint32_t i = 0; i < C4::kCols * C4::kRows; ++i)
    {
        if (mDiscs[i].mInUse && mDiscs[i].mNode != nullptr && mDiscs[i].mNode->IsPhysicsEnabled())
        {
            live++;
        }
    }

    // Lowest first, so the board comes apart from the bottom the way a real one does.
    //
    // The order used to be whatever order the discs were played in, which is scattered all over the
    // board -- discs let go from the middle of a column while the ones beneath them stayed put, so
    // it read as discs being picked out rather than a board emptying. Which disc is where is known
    // from its resting place, so release follows that instead.
    while (live < kMaxLiveDiscs)
    {
        int32_t lowest = -1;
        float lowestY = 0.0f;

        for (uint32_t i = 0; i < C4::kCols * C4::kRows; ++i)
        {
            const Disc& disc = mDiscs[i];

            if (!disc.mAwaitingRelease || !disc.mInUse || disc.mNode == nullptr)
            {
                continue;
            }

            if (lowest < 0 || disc.mTo.y < lowestY)
            {
                lowest = (int32_t)i;
                lowestY = disc.mTo.y;
            }
        }

        if (lowest < 0)
        {
            break;
        }

        ReleaseDisc(mDiscs[lowest], (uint32_t)lowest);
        live++;
    }
}

void Connect4Scene::ReleaseDisc(Disc& disc, uint32_t index)
{
    World* world = GetWorld(0);
    const float spacing = mLayout.GetRowSpacing();

    disc.mAwaitingRelease = false;
    uint32_t seed = 0x51ED2701u + index * 2654435761u;

    // A disc is a cylinder. Built along whichever of the model's axes runs through its flat,
    // and in the model's own units, since the node's scale is applied to the shape on top.
    btCollisionShape* shape = nullptr;
    const float r = mDiscLocalRadius;
    const float h = mDiscLocalHalfThickness;

    btVector3 halfExtents;
    btCylinderShape* cylinder = nullptr;

    switch (mDiscFaceAxis)
    {
    case 0:  halfExtents = btVector3(h, r, r); cylinder = new btCylinderShapeX(halfExtents); break;
    case 1:  halfExtents = btVector3(r, h, r); cylinder = new btCylinderShape(halfExtents);  break;
    default: halfExtents = btVector3(r, r, h); cylinder = new btCylinderShapeZ(halfExtents); break;
    }

    shape = cylinder;

    // Size the collision margin to the disc's world size.
    //
    // This is the single reason the rerack never looked like physics. A margin is a rounded skin
    // Bullet keeps around a convex shape, and it is the shape the solver actually collides with.
    // Bullet's default is 0.04 -- chosen for a world measured in metres -- and, as its own header
    // says, "collisionMargin is not scaled". The engine applies the node's world scale to the
    // shape every frame; the margin is left exactly where it was.
    //
    // The discs sit under a transform at 0.03, so the cylinder's real dimensions scale down to
    // thousandths of a unit while the margin stays put and ends up several times larger than the
    // disc it is wrapping. What the solver sees is not a disc at all: it is a rounded blob with no
    // flat face to lie on, no rim to roll along and no edge to tip over. Hence discs that will not
    // tumble, rest at angles nothing could hold them at, and sink into one another -- and hence
    // every attempt to fix those by adjusting gravity, friction, damping and solver iterations
    // failing, because none of them were the problem.
    //
    // The margin has to be a fraction of the disc's size as it is actually simulated, and the core
    // dimensions have to give that room back so the disc still ends up its true thickness. The
    // shape is built in the model's units and scaled afterwards, so the margin -- which is not
    // scaled -- is divided back out of the dimensions here.
    const glm::vec3 nodeScale = disc.mNode->GetWorldScale();
    const float uniformScale = glm::max(glm::min(glm::min(nodeScale.x, nodeScale.y), nodeScale.z), 0.0001f);

    const float smallestWorldHalfExtent = glm::min(h, r) * uniformScale;
    const float margin = glm::max(smallestWorldHalfExtent * 0.1f, 0.00001f);

    cylinder->setMargin(margin);
    cylinder->setImplicitShapeDimensions(halfExtents - btVector3(margin, margin, margin) / uniformScale);

    disc.mNode->SetCollisionShape(shape);

    disc.mNode->SetMass(kDiscMass);
    disc.mNode->SetFriction(0.5f);
    disc.mNode->SetRestitution(0.35f);       // light plastic clatters rather than thuds
    disc.mNode->SetLinearDamping(0.02f);

    // Rolling friction and angular damping stay low on purpose. They are what slows a disc
    // rolling away on its edge, which is the best thing the simulation does -- the spinning on
    // the spot is a different rotation entirely and is dealt with below, so there is no reason
    // to spend these on it and flatten the rolling in the process.
    disc.mNode->SetRollingFriction(0.012f);
    disc.mNode->SetAngularDamping(0.1f);

    // Each other, the table and the stand's feet. Nothing else -- above all not the frame they
    // are falling out of.
    disc.mNode->SetCollisionGroup(kRerackColGroup);
    disc.mNode->SetCollisionMask(kRerackColGroup);

    disc.mNode->EnableCollision(true);
    disc.mNode->EnablePhysics(true);

    // Start the body where the node already is, rather than wherever it was when the body was
    // last created.
    disc.mNode->FullSyncRigidBodyTransform();

    // Less gravity than the world's.
    //
    // The world's is correct for a world measured in metres, and the discs were obeying it
    // exactly -- which is the problem. The board is only centimetres across in world units, so
    // a disc falls its own height in a few hundredths of a second and the whole rerack is over
    // before the eye can follow it. It reads as something dense being dropped rather than a
    // plastic counter tipping out of a rack.
    //
    // Slowing gravity for these bodies alone keeps the arcs and the tumbling and just gives
    // them time to be seen. Nothing else in the scene is simulated, so there is nothing for
    // this to be inconsistent with.
    if (world != nullptr && world->GetDynamicsWorld() != nullptr && disc.mNode->GetRigidBody() != nullptr)
    {
        btRigidBody* body = disc.mNode->GetRigidBody();

        // Force the mass properties onto the body rather than trusting SetMass.
        //
        // Primitive3D::SetMass does nothing at all when the value has not changed, and after the
        // startup warm-up every disc already carries this mass -- so on a real rerack the call
        // above is a no-op and the body keeps whatever inertia it was last given. The warm-up also
        // means the rigid body already exists, so the release takes the swap-the-shape path rather
        // than building a body around a fresh cylinder.
        //
        // Neither is a problem on its own, but both make the disc's dynamics depend on what
        // happened to it earlier in the session rather than on what is being asked for now. Setting
        // it here, from the shape that is actually attached, does not care.
        btCollisionShape* attached = body->getCollisionShape();

        if (attached != nullptr && attached->getShapeType() != EMPTY_SHAPE_PROXYTYPE)
        {
            btVector3 inertia(0.0f, 0.0f, 0.0f);
            attached->calculateLocalInertia(kDiscMass, inertia);
            body->setMassProps(kDiscMass, inertia);
            body->updateInertiaTensor();
        }

        // Free to turn about every axis. Nothing sets this otherwise, but a body that cannot
        // rotate is indistinguishable from the fault being chased here, so it is stated.
        body->setAngularFactor(btVector3(1.0f, 1.0f, 1.0f));

        const btVector3 worldGravity = world->GetDynamicsWorld()->getGravity();
        body->setGravity(worldGravity * kDiscGravityScale);

        // Spinning friction, which is the one that stops a disc turning on the spot.
        //
        // Rolling friction resists a disc rolling along on its edge; nothing in it opposes a
        // rotation about the point of contact, so a disc that came to rest flat kept spinning
        // where it lay with only damping to slow it, which took a very long time. This is the
        // parameter for that case and it was simply never set.
        body->setSpinningFriction(0.08f);

        // Never let Bullet put these to sleep.
        //
        // A sleeping body stops being simulated and ignores anything done to it, and the threshold
        // for sleeping was above the speeds at which a pile actually settles -- so discs were
        // dropping below it while still resolving against each other and freezing exactly as they
        // were, half settled, in poses nothing would hold. It also swallowed the torque meant to
        // tip a standing disc over, leaving it to be laid flat by hand.
        //
        // There is already a mechanism for deciding a disc has finished: it is retired when it
        // stops going anywhere, which is measured over a couple of seconds rather than from an
        // instantaneous speed. Two systems deciding the same thing, on different evidence, is what
        // produced the odd poses -- so only one of them keeps the job.
        body->setActivationState(DISABLE_DEACTIVATION);

        // Sweep the disc along its path instead of testing where it lands.
        //
        // This is the difference between the rerack looking simulated and not. A disc is about
        // twenty millimetres thick and, half a second into a fall, covers ninety in a single
        // physics step -- four times its own thickness. Without a swept test it is on one side of
        // another disc in one step and the far side in the next, so the contact between them is
        // found late or missed altogether. Everything then looks wrong at once: discs pass through
        // each other, land at the wrong height, stop dead for no reason.
        //
        // No amount of adjusting gravity, friction or solver iterations can fix that, because the
        // contacts being solved are the wrong ones. Several rounds of tuning went into symptoms of
        // it before the arithmetic was checked.
        body->setCcdMotionThreshold(mDiscHalfThickness);
        body->setCcdSweptSphereRadius(mDiscHalfThickness * 0.8f);
    }

    // The spread it was given when the tray was pulled, and a turn to go with it.
    seed = seed * 1664525u + 1013904223u;
    const float ax = ((seed >> 16) & 0xFF) / 255.0f - 0.5f;
    seed = seed * 1664525u + 1013904223u;
    const float ay = ((seed >> 16) & 0xFF) / 255.0f - 0.5f;
    seed = seed * 1664525u + 1013904223u;
    const float az = ((seed >> 16) & 0xFF) / 255.0f - 0.5f;

    disc.mCheckPos = disc.mNode->GetWorldPosition();
    disc.mSlowTime = 0.0f;
    disc.mLiveTime = 0.0f;

    disc.mNode->SetLinearVelocity(disc.mFrom);
    disc.mNode->SetAngularVelocity(glm::vec3(ax, ay, az) * spacing * 18.0f);
}

// Take a disc out of the simulation, and lay it down if it was left standing.
//
// Physics off, collision left on. Turning both off made a retired disc a ghost: it stopped being
// simulated, which is the point, but it also stopped being something to land on, so every disc
// that came down afterwards fell straight through it and came to rest inside it. A disc that has
// settled is still there -- it just does not need moving any more.
void Connect4Scene::RetireDisc(Disc& disc)
{
    if (disc.mNode == nullptr)
    {
        return;
    }

    const glm::quat current = disc.mNode->GetWorldRotationQuat();

    glm::vec3 localNormal(0.0f);
    localNormal[glm::clamp(mDiscFaceAxis, 0, 2)] = 1.0f;

    const glm::vec3 normal = current * localNormal;
    const float uprightness = glm::abs(normal.y);

    disc.mNode->EnablePhysics(false);

    // Near enough flat already: leave it exactly where the simulation put it.
    //
    // This is now the last resort rather than the usual path. A disc that stops on its edge is
    // nudged over and allowed to fall properly; it only reaches here if it has been simulated for
    // as long as it is going to be and is still not down, in which case being laid flat is better
    // than being left standing.
    if (uprightness > 0.85f)
    {
        return;
    }

    // A disc lying on the table has nothing to lean on, so it cannot rest at an angle -- it falls
    // over, every time. One resting on top of others can be propped at any angle it likes, and
    // those arrangements are the best thing the simulation produces.
    //
    // So the question is not how tilted it is but whether there is anything under it. A disc on the
    // table sits within about a radius of it however it is leaning; anything higher is on top of
    // something else and is left alone.
    const glm::vec3 position = disc.mNode->GetWorldPosition();
    const bool restingOnTable = (position.y < mTableY + mDiscRadius * 1.25f);

    if (!restingOnTable && uprightness > 0.3f)
    {
        return;
    }

    disc.mRotFrom = current;
    disc.mRotTo = ToppleRotation(current, mDiscFaceAxis);

    disc.mPosFrom = position;
    disc.mPosTo = disc.mPosFrom;

    // How far it has to come down is how far over it has to go: a disc on its edge is a radius up,
    // flat it is half its thickness.
    disc.mPosTo.y -= (mDiscRadius - mDiscHalfThickness) * (1.0f - uprightness);

    disc.mFlatT = 0.0f;
}

// Retire each disc as it stops travelling, rather than waiting for all of them to be still at
// once.
//
// This does two jobs. A disc balanced on its edge will spin like a coin for as long as Bullet is
// asked to keep simulating it -- the contact is effectively a point, so there is almost nothing to
// slow it. Taking it out of the simulation ends that outright, and RetireDisc lays it down.
//
// And it is the cost: a retired disc is one fewer body in the solver, which both keeps the frame
// time down and makes room for the next disc waiting to be released.
// Push over any disc that has come to rest standing on its edge.
//
// A disc on its edge is balanced, and Bullet will hold it there indefinitely -- the contact is
// effectively a point, so nothing decides which way it should go. Real ones fall.
//
// This is a torque applied every frame, not a shove every so often. An earlier version nudged once
// each time the disc was checked, half a second apart, which was both too weak to get it past the
// balance point and visible as exactly that: a twitch, a pause, another twitch. Leaning on it
// steadily tips it over in a fraction of a second and looks like nothing at all -- what is seen is
// the disc falling, which is gravity's work once it is past the point of no return.
void Connect4Scene::TipOverIfStanding(float deltaTime)
{
    if (!mDiscPhysicsRunning)
    {
        return;
    }

    // Strict, so a disc still sliding or rolling is left to do it. Tipping one that is
    // still travelling is what made the landing read as drop, stop, flop.
    const float goingNowhere = mLayout.GetRowSpacing() * 0.25f;

    glm::vec3 localNormal(0.0f);
    localNormal[glm::clamp(mDiscFaceAxis, 0, 2)] = 1.0f;

    for (uint32_t i = 0; i < C4::kCols * C4::kRows; ++i)
    {
        Disc& disc = mDiscs[i];

        if (!disc.mInUse || disc.mNode == nullptr || !disc.mNode->IsPhysicsEnabled())
        {
            continue;
        }

        // Still travelling: leave it alone. A disc rolling away on its edge is supposed to be on
        // its edge, and tipping it over mid-roll would be the same mistake as animating it flat.
        if (glm::length(disc.mNode->GetLinearVelocity()) > goingNowhere)
        {
            continue;
        }

        const glm::vec3 normal = disc.mNode->GetWorldRotationQuat() * localNormal;
        const float uprightness = glm::abs(normal.y);

        // Only on the table. A disc propped against others at an angle is resting on something,
        // and that is a real arrangement worth keeping.
        const bool onTable = (disc.mNode->GetWorldPosition().y < mTableY + mDiscRadius * 1.25f);

        if (uprightness > 0.6f || !onTable)
        {
            continue;
        }

        // About the axis that carries its face towards vertical, so it goes over the way it is
        // already leaning rather than being turned to some chosen side.
        glm::vec3 axis = glm::cross(normal, glm::vec3(0.0f, 1.0f, 0.0f));

        if (glm::length(axis) > 0.001f)
        {
            // Wake it first.
            //
            // Bullet deactivates a body once it stops, and a sleeping body ignores everything --
            // so this torque was being thrown away every frame at exactly the moment it was
            // needed. The disc stood there untouched until retirement gave up on it and laid it
            // flat by hand, which is why it sat still and then snapped over.
            if (disc.mNode->GetRigidBody() != nullptr)
            {
                disc.mNode->GetRigidBody()->activate(true);
            }

            disc.mNode->AddAngularVelocity(glm::normalize(axis) * kToppleAccel * deltaTime);
        }
    }
}

void Connect4Scene::UpdateDiscRetirement(float deltaTime)
{
    if (!mDiscPhysicsRunning)
    {
        return;
    }

    // How far a disc has to travel between checks to count as still going somewhere.
    const float worthwhileTravel = mDiscRadius * 0.75f;

    for (uint32_t i = 0; i < C4::kCols * C4::kRows; ++i)
    {
        Disc& disc = mDiscs[i];

        if (!disc.mInUse || disc.mNode == nullptr || !disc.mNode->IsPhysicsEnabled())
        {
            continue;
        }

        disc.mSlowTime += deltaTime;
        disc.mLiveTime += deltaTime;

        // Anything still going after this long is not going to settle. A disc spinning on its edge
        // can keep itself alive indefinitely, and there is no arrangement of friction and damping
        // that reliably stops it -- so it is stopped by the clock.
        if (disc.mLiveTime > kDiscMaxLiveTime)
        {
            RetireDisc(disc);
            continue;
        }

        if (disc.mSlowTime < kDiscRetireTime)
        {
            continue;
        }

        // Measured as distance covered, not as reported speed.
        //
        // A velocity test kept being defeated by discs going nowhere in any meaningful sense but
        // not holding still either: one spinning on its edge wobbles, and the wobble alone cleared
        // the threshold every frame and reset the timer. Where it actually is, compared with where
        // it was a moment ago, does not care about that.
        const glm::vec3 now = disc.mNode->GetWorldPosition();

        if (glm::distance(now, disc.mCheckPos) >= worthwhileTravel)
        {
            disc.mCheckPos = now;
            disc.mSlowTime = 0.0f;
            continue;
        }

        // It has stopped going anywhere, and TipOverIfStanding has had every frame since it landed
        // to push it over. If it is still up at this point it is not going to come down on its own.
        RetireDisc(disc);
    }
}

// Advance any disc that is in the middle of falling over. Runs whatever the rerack is doing, since
// a disc retired late is still on its way down when the rest have finished.
void Connect4Scene::UpdateToppling(float deltaTime)
{
    for (uint32_t i = 0; i < C4::kCols * C4::kRows; ++i)
    {
        Disc& disc = mDiscs[i];

        if (!disc.mInUse || disc.mNode == nullptr || disc.mFlatT < 0.0f || disc.mFlatT >= 1.0f)
        {
            continue;
        }

        disc.mFlatT = glm::min(disc.mFlatT + deltaTime / kRetireToppleTime, 1.0f);

        // Accelerating, because something falling over starts slowly and arrives fast.
        const float fall = disc.mFlatT * disc.mFlatT;

        disc.mNode->SetWorldRotation(glm::slerp(disc.mRotFrom, disc.mRotTo, fall));
        disc.mNode->SetWorldPosition(glm::mix(disc.mPosFrom, disc.mPosTo, fall));
    }
}

void Connect4Scene::StopDiscPhysics()
{
    // Every disc, not only the ones still being simulated. A retired disc has its physics off
    // already but keeps its collision, so that later discs have something to land on; without
    // clearing it here those colliders would outlive the rerack and sit invisibly on the table
    // through the next game.
    for (uint32_t i = 0; i < C4::kCols * C4::kRows; ++i)
    {
        if (mDiscs[i].mNode != nullptr)
        {
            mDiscs[i].mNode->EnablePhysics(false);
            mDiscs[i].mNode->EnableCollision(false);
        }
    }

    // Back to whatever the rest of the game expects.
    World* world = GetWorld(0);

    if (mSavedSolverIterations > 0 && world != nullptr && world->GetDynamicsWorld() != nullptr)
    {
        world->GetDynamicsWorld()->getSolverInfo().m_numIterations = mSavedSolverIterations;
        mSavedSolverIterations = 0;
    }

    mDiscPhysicsRunning = false;
}

// Settled when nothing is moving any more. Asked of the bodies rather than timed, so the board is
// never put back together while a disc is still rolling.
bool Connect4Scene::AreDiscsAsleep() const
{
    const float threshold = mLayout.GetRowSpacing() * 0.25f;
    const float spinThreshold = 0.6f;

    for (uint32_t i = 0; i < C4::kCols * C4::kRows; ++i)
    {
        const Disc& disc = mDiscs[i];

        // Still waiting to be let go: the rerack is certainly not finished.
        if (disc.mAwaitingRelease)
        {
            return false;
        }

        if (!disc.mInUse || disc.mNode == nullptr || !disc.mNode->IsPhysicsEnabled())
        {
            continue;
        }

        if (glm::length(disc.mNode->GetLinearVelocity()) > threshold)
        {
            return false;
        }

        // Turning counts as moving, not just travelling.
        if (glm::length(disc.mNode->GetAngularVelocity()) > spinThreshold)
        {
            return false;
        }
    }

    return true;
}

void Connect4Scene::BuildObstacles()
{
    mNumObstacles = 0;

    StaticMesh3D* standMesh = (mStandNode != nullptr) ? mStandNode->As<StaticMesh3D>() : nullptr;

    if (standMesh == nullptr || standMesh->GetStaticMesh() == nullptr)
    {
        return;
    }

    StaticMesh* mesh = standMesh->GetStaticMesh();
    const uint32_t numVerts = mesh->GetNumVertices();

    if (numVerts == 0)
    {
        return;
    }

    const glm::mat4& toWorld = mStandNode->GetTransform();

    // Only the part of the stand that is down near the table matters. A disc skidding across the
    // table can hit a foot; it is never high enough to reach the uprights, and taking the whole
    // stand would put a wall across the middle of the table where the discs are supposed to land.
    glm::vec3 standMin(1e9f);
    glm::vec3 standMax(-1e9f);

    const bool hasColor = mesh->HasVertexColor();
    const VertexColor* colorVerts = hasColor ? mesh->GetColorVertices() : nullptr;
    const Vertex* plainVerts = hasColor ? nullptr : mesh->GetVertices();

    for (uint32_t v = 0; v < numVerts; ++v)
    {
        const glm::vec3 local = hasColor ? colorVerts[v].mPosition : plainVerts[v].mPosition;
        const glm::vec3 world = glm::vec3(toWorld * glm::vec4(local, 1.0f));

        standMin = glm::min(standMin, world);
        standMax = glm::max(standMax, world);
    }

    const float footTop = standMin.y + (standMax.y - standMin.y) * 0.22f;
    const float middleX = (standMin.x + standMax.x) * 0.5f;

    // One box per side. Splitting on the stand's own middle separates the two feet without having
    // to work out where either of them is.
    glm::vec3 lo[2] = { glm::vec3(1e9f), glm::vec3(1e9f) };
    glm::vec3 hi[2] = { glm::vec3(-1e9f), glm::vec3(-1e9f) };
    uint32_t counts[2] = { 0, 0 };

    for (uint32_t v = 0; v < numVerts; ++v)
    {
        const glm::vec3 local = hasColor ? colorVerts[v].mPosition : plainVerts[v].mPosition;
        const glm::vec3 world = glm::vec3(toWorld * glm::vec4(local, 1.0f));

        if (world.y > footTop)
        {
            continue;
        }

        const uint32_t side = (world.x < middleX) ? 0u : 1u;

        lo[side] = glm::min(lo[side], world);
        hi[side] = glm::max(hi[side], world);
        counts[side]++;
    }

    for (uint32_t side = 0; side < 2; ++side)
    {
        if (counts[side] < 8)
        {
            continue;
        }

        mObstacles[mNumObstacles].mMin = lo[side];
        mObstacles[mNumObstacles].mMax = hi[side];
        mNumObstacles++;
    }

    LogDebug("C4: %u stand obstacle(s)", mNumObstacles);
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
    mRerackSoundPlayed = false;


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
        // Only the sideways spread is decided here. The turn each disc picks up is set with its
        // body in StartDiscPhysics, along with everything else Bullet needs.

        const float spacing = mLayout.GetRowSpacing();

        // Straight down to begin with. A disc still inside the board can only go one way -- down
        // its own column and out of the bottom -- and letting it drift sideways immediately sent it
        // out through the plastic instead of through the opening the tray just uncovered.
        //
        // The spread is held here and applied the moment it clears the frame.
        disc.mVelocity = glm::vec3(0.0f);

        // Barely a push. Discs tipping out of a rack are not thrown anywhere -- they drop, and
        // they scatter because they catch each other and the tray on the way out. Launching them
        // was left over from the hand-written fall, which had no contacts and so had to fake the
        // spread; with Bullet doing the work it only made all forty-two set off in the same
        // direction at once, which is what "moving as a group" was.
        //
        // What remains is a nudge to break the symmetry, so they do not fall in perfect lockstep
        // out of a perfectly regular grid.
        disc.mSlowTime = 0.0f;
        disc.mFlatT = -1.0f;

        // Enough push to spread out. This was cut to almost nothing when every disc was being
        // given the same shove and the whole set set off together; the problem then was that the
        // push was shared, not that it existed. Most of this is per disc, so they go their own
        // ways rather than travelling as a block.
        // Scaled against the board, which is small -- a row spacing is a few centimetres, so what
        // looked like a generous multiplier was a few centimetres of drift across the whole fall,
        // less than a disc's width. These are the numbers that actually move a disc somewhere.
        disc.mFrom = mSpreadAxis * (rx * spacing * 11.0f)
                   + mSpillAxis * (spacing * 2.0f)
                   + glm::vec3(0.0f, 0.0f, rz * spacing * 6.0f);

    }
}

void Connect4Scene::ClearDiscs()
{
    // Not while the warm-up is running.
    //
    // The game calls NewGame as soon as the scene is initialised, and NewGame clears the discs --
    // which would switch off the heap that has just been dropped, before a single frame of it had
    // been simulated. The warm-up would then run its frames with nothing in the world and reserve
    // nothing, while reporting that it had finished.
    if (mWarmUpFrames > 0)
    {
        return;
    }

    // Nothing should still be simulated once the discs are back in the pool.
    StopDiscPhysics();

    // Putting the discs away and putting the board back together are the same moment, so the
    // frame and tray return here rather than at the end of the rerack.
    ResetBoardParts();

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
        mDiscs[i].mSlowTime = 0.0f;
        mDiscs[i].mFlatT = -1.0f;
        mDiscs[i].mAwaitingRelease = false;
    }

    mNumDiscsUsed = 0;
    mNumWinDiscs = 0;
    mDroppingDisc = -1;
    mDropActive = false;
}
