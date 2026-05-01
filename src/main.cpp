#include <raylib.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include <algorithm>

// ── Fast approximate rsqrt (for WASM where sqrtf is slow) ────────────────────
// Uses the classic Quake III fast inverse square root with one Newton iteration
static inline float fast_rsqrt(float x) {
    float x2 = x * 0.5f;
    float y = x;
    unsigned int i;
    memcpy(&i, &y, sizeof(i));
    i = 0x5f3759df - (i >> 1);  // magic number
    memcpy(&y, &i, sizeof(y));
    y = y * (1.5f - (x2 * y * y));  // one Newton iteration
    return y;
}

// ── Constants ────────────────────────────────────────────────────────────────
// Use #define for compile-time constants that affect array sizes
#define SCREEN_WIDTH  960
#define SCREEN_HEIGHT 540
#define SPHERE_COUNT  10000
#define SPHERE_RADIUS 6.0f
#define MIN_DIST      (SPHERE_RADIUS * 2.0f)
#define MIN_DIST_SQ   (MIN_DIST * MIN_DIST)

const int ScreenWidth  = SCREEN_WIDTH;
const int ScreenHeight = SCREEN_HEIGHT;
const int SphereCount  = SPHERE_COUNT;
const float SphereRadius = SPHERE_RADIUS;

static int numColumns = 20;
const float SphereSpacingX = 2.1f;
const float SphereSpacingY = 2.1f;
const float Gravity = 980.0f;
const float GroundY = (float)(SCREEN_HEIGHT - 50);
static float Restitution = 0.9f;
static float Friction = 0.0f;

// Spatial grid - pre-computed constants (CellSize = 6*4 = 24)
// GridCols = (960/24) + 3 = 43, GridRows = (540/24) + 3 = 25
#define GRID_COLS 43
#define GRID_ROWS 25
#define CELL_SIZE 24.0f

// ── Sleep thresholds ─────────────────────────────────────────────────────────
#define SLEEP_VELOCITY_THRESHOLD 200.0f     // velocity magnitude below this = candidate for sleep
#define SLEEP_TIME_THRESHOLD     0.3f       // seconds below threshold before going to sleep
//#define WAKE_VELOCITY_THRESHOLD  30.0f    // if a neighbor has velocity above this, wake up
#define WAKE_VELOCITY_THRESHOLD SLEEP_VELOCITY_THRESHOLD

// ── Ball structure ───────────────────────────────────────────────────────────
struct Ball {
    Vector2 Position;
    Vector2 Velocity;
    Color   Color;
    bool    IsAlive;
    bool    IsSleeping;
    float   sleepTimer;   // time spent with velocity below threshold
};

static Ball balls[SPHERE_COUNT];

// ── Spatial grid (pre-allocated arrays, no allocations) ──────────────────────
static int gridHeads[GRID_COLS * GRID_ROWS]; // -1 = empty
static int gridNext[SPHERE_COUNT];           // linked list per cell
static int gridCellIdx[SPHERE_COUNT];        // which cell each ball is in

// ── Pre-rendered circle textures ─────────────────────────────────────────────
static Texture2D circleTexture;
static Texture2D highlightTexture;

// ── Performance tracking ─────────────────────────────────────────────────────
static double physicsTime = 0;
static double renderTime  = 0;
static double frameTime   = 0;
static int sleepingCount = 0;

// ── Auto-restart timer ───────────────────────────────────────────────────────
const float RestartInterval = 8.0f;
static float restartTimer = 0;

// ── RNG (simple xorshift32 seeded with 42) ───────────────────────────────────
static unsigned int rngState = 42;

static unsigned int xorshift32() {
    unsigned int x = rngState;
    x ^= x << 13;
    x ^= x >> 17;
    x ^= x << 5;
    rngState = x;
    return x;
}

static float randFloat() {
    return (float)xorshift32() / (float)0xFFFFFFFFu;
}

static int randInt(int min, int max) {
    return min + (int)(randFloat() * (max - min + 1));
}

// ── Forward declarations ─────────────────────────────────────────────────────
static void InitTextures(void);
static void InitBalls(void);
static void RandomizeParams(void);
static void DrawSpheres(void);
static void BuildGrid(void);
static void UpdatePhysics(float dt);
static void ResolveCollision(int i, int j);

// ── Main ─────────────────────────────────────────────────────────────────────
int main(void)
{
    InitWindow(ScreenWidth, ScreenHeight, "Raylib C++ 2D Physics Simulation");
    SetTargetFPS(60);

    // Initialize grid heads to -1
    memset(gridHeads, -1, sizeof(gridHeads));

    InitTextures();
    InitBalls();

    while (!WindowShouldClose())
    {
        float dt = GetFrameTime();
        if (dt > 0.033f) dt = 0.033f;

        if (IsKeyPressed(KEY_R))
            InitBalls();

        // Auto-restart every RestartInterval seconds
        restartTimer += dt;
        if (restartTimer >= RestartInterval)
        {
            restartTimer = 0;
            InitBalls();
        }

        // Physics
        double physStart = GetTime();
        UpdatePhysics(dt);
        physicsTime = (GetTime() - physStart) * 1000.0;

        // Render
        BeginDrawing();
        ClearBackground(Color{ 15, 15, 25, 255 });

        DrawRectangle(0, (int)GroundY, ScreenWidth, 4, SKYBLUE);

        double renderStart = GetTime();
        DrawSpheres();
        renderTime = (GetTime() - renderStart) * 1000.0;

        // UI
        int ls = 25;
        int sy = 10;
        DrawFPS(10, sy);
        char buf[256];

        sprintf(buf, "Spheres: %d", SphereCount);
        DrawText(buf, 10, sy + ls, 20, LIGHTGRAY);

        sprintf(buf, "Physics: %.2fms  Draw: %.2fms  Frame: %.2fms",
                physicsTime, renderTime, frameTime);
        DrawText(buf, 10, sy + 2 * ls, 20, LIGHTGRAY);

        sprintf(buf, "Restitution: %.2f  Friction: %.2f  Cols: %d  Asleep: %d",
                Restitution, Friction, numColumns, sleepingCount);
        DrawText(buf, 10, sy + 3 * ls, 20, LIGHTGRAY);

        float timeLeft = RestartInterval - restartTimer;
        sprintf(buf, "Press R to reset  (auto: %.1fs)", timeLeft);
        DrawText(buf, 10, sy + 4 * ls, 20, LIGHTGRAY);

        EndDrawing();
        frameTime = (GetTime() - physStart) * 1000.0;
    }

    UnloadTexture(circleTexture);
    UnloadTexture(highlightTexture);
    CloseWindow();

    return 0;
}

// ── InitTextures ─────────────────────────────────────────────────────────────
static void InitTextures(void)
{
    int texSize = (int)(SphereRadius * 4);
    if (texSize < 4) texSize = 4;

    Image circleImg = GenImageColor(texSize, texSize, Color{ 0, 0, 0, 0 });
    Vector2 center = { texSize / 2.0f, texSize / 2.0f };

    for (int y = 0; y < texSize; y++)
    {
        for (int x = 0; x < texSize; x++)
        {
            float dx = (float)x - center.x;
            float dy = (float)y - center.y;
            if (dx * dx + dy * dy <= SphereRadius * SphereRadius)
                ImageDrawPixel(&circleImg, x, y, WHITE);
        }
    }
    circleTexture = LoadTextureFromImage(circleImg);
    UnloadImage(circleImg);

    Image highlightImg = GenImageColor(texSize, texSize, Color{ 0, 0, 0, 0 });
    Vector2 hc = { center.x - 2, center.y - 2 };
    float hr = SphereRadius * 0.3f;

    for (int y = 0; y < texSize; y++)
    {
        for (int x = 0; x < texSize; x++)
        {
            float dx = (float)x - hc.x;
            float dy = (float)y - hc.y;
            if (dx * dx + dy * dy <= hr * hr)
                ImageDrawPixel(&highlightImg, x, y, WHITE);
        }
    }
    highlightTexture = LoadTextureFromImage(highlightImg);
    UnloadImage(highlightImg);
}

// ── RandomizeParams ──────────────────────────────────────────────────────────
static void RandomizeParams(void)
{
    numColumns = randInt(10, 30);
    Restitution = 0.1f + randFloat() * 0.8f;
    Friction = randFloat() * 0.5f;
}

// ── InitBalls ────────────────────────────────────────────────────────────────
static void InitBalls(void)
{
    RandomizeParams();

    float spacingX = SphereRadius * SphereSpacingX;
    float startX = ScreenWidth / 2.0f - (numColumns - 1) * spacingX / 2.0f;
    float spacingY = SphereRadius * SphereSpacingY;

    for (int i = 0; i < SphereCount; i++)
    {
        int col = i % numColumns;
        int row = i / numColumns;

        float x = startX + col * spacingX + (randFloat() - 0.5f) * spacingX * 0.5f;
        float y = -SphereRadius * 2 - row * spacingY;

        balls[i].Position = Vector2{ x, y };
        balls[i].Velocity = Vector2{ 0, 0 };
        balls[i].Color = Color{
            (unsigned char)(randFloat() * 256),
            (unsigned char)(randFloat() * 256),
            (unsigned char)(randFloat() * 256),
            255
        };
        balls[i].IsAlive = true;
        balls[i].IsSleeping = false;
        balls[i].sleepTimer = 0.0f;
    }
}

// ── DrawSpheres ──────────────────────────────────────────────────────────────
static void DrawSpheres(void)
{
    for (int i = 0; i < SphereCount; i++)
    {
        if (!balls[i].IsAlive) continue;

        Vector2 pos = balls[i].Position;

        // Skip spheres that are entirely outside the screen
        if (pos.x + SphereRadius < 0 || pos.x - SphereRadius > ScreenWidth ||
            pos.y + SphereRadius < 0 || pos.y - SphereRadius > ScreenHeight)
            continue;

        DrawCircleV(pos, SphereRadius, balls[i].Color);
    }
}

// ── BuildGrid ────────────────────────────────────────────────────────────────
static void BuildGrid(void)
{
    memset(gridHeads, -1, sizeof(gridHeads));

    for (int i = 0; i < SphereCount; i++)
    {
        if (!balls[i].IsAlive) continue;

        int col = (int)(balls[i].Position.x / CELL_SIZE);
        int row = (int)(balls[i].Position.y / CELL_SIZE);

        col = std::max(0, std::min(col, GRID_COLS - 1));
        row = std::max(0, std::min(row, GRID_ROWS - 1));

        int cell = row * GRID_COLS + col;
        gridNext[i] = gridHeads[cell];
        gridHeads[cell] = i;
        gridCellIdx[i] = cell;
    }
}

// ── Helper: compute velocity squared magnitude ──────────────────────────────
static inline float velMagSq(const Ball& b) {
    return b.Velocity.x * b.Velocity.x + b.Velocity.y * b.Velocity.y;
}

// ── UpdatePhysics ────────────────────────────────────────────────────────────
static void UpdatePhysics(float dt)
{
    int substeps = 4;
    float subDt = dt / substeps;
    float sleepVelSq = SLEEP_VELOCITY_THRESHOLD * SLEEP_VELOCITY_THRESHOLD;
    float wakeVelSq  = WAKE_VELOCITY_THRESHOLD * WAKE_VELOCITY_THRESHOLD;

    for (int step = 0; step < substeps; step++)
    {
        // Phase 1: Apply forces (skip dead and sleeping balls)
        for (int i = 0; i < SphereCount; i++)
        {
            if (!balls[i].IsAlive || balls[i].IsSleeping) continue;

            balls[i].Velocity.y += Gravity * subDt;
            float damp = 1.0f - 0.5f * subDt;
            if (damp < 0.0f) damp = 0.0f;
            balls[i].Velocity.x *= damp;
            balls[i].Velocity.y *= damp;
            balls[i].Position.x += balls[i].Velocity.x * subDt;
            balls[i].Position.y += balls[i].Velocity.y * subDt;

            // Mark balls that fall far below the ground as dead
            if (balls[i].Position.y > GroundY + ScreenHeight * 0.5f)
                balls[i].IsAlive = false;
        }

        // Phase 2: Build spatial grid (only alive, non-sleeping balls)
        // Sleeping balls are static obstacles — they still go in the grid so
        // awake balls can collide with them, but they don't move.
        memset(gridHeads, -1, sizeof(gridHeads));
        for (int i = 0; i < SphereCount; i++)
        {
            if (!balls[i].IsAlive) continue;

            int col = (int)(balls[i].Position.x / CELL_SIZE);
            int row = (int)(balls[i].Position.y / CELL_SIZE);
            col = std::max(0, std::min(col, GRID_COLS - 1));
            row = std::max(0, std::min(row, GRID_ROWS - 1));

            int cell = row * GRID_COLS + col;
            gridNext[i] = gridHeads[cell];
            gridHeads[cell] = i;
            gridCellIdx[i] = cell;
        }

        // Phase 3: Wall/ground collision (only alive, non-sleeping balls)
        for (int i = 0; i < SphereCount; i++)
        {
            if (!balls[i].IsAlive || balls[i].IsSleeping) continue;

            if (balls[i].Position.y + SphereRadius > GroundY)
            {
                balls[i].Position.y = GroundY - SphereRadius;
                balls[i].Velocity.y = -balls[i].Velocity.y * Restitution;
                balls[i].Velocity.x *= (1.0f - Friction);
                if (fabsf(balls[i].Velocity.y) < 10.0f)
                    balls[i].Velocity.y = 0;
            }
        }

        // Phase 4: Sphere-sphere collision via spatial grid
        // Only awake balls check for collisions. If a collision happens with a
        // sleeping ball, the sleeping ball gets woken up.
        for (int i = 0; i < SphereCount; i++)
        {
            if (!balls[i].IsAlive || balls[i].IsSleeping) continue;

            int cell = gridCellIdx[i];
            int row = cell / GRID_COLS;
            int col = cell % GRID_COLS;

            // Check same cell (only higher indices to avoid double-checks)
            for (int j = gridHeads[cell]; j != -1; j = gridNext[j])
            {
                if (j <= i) continue;
                // Skip if both are sleeping (shouldn't happen since i is awake,
                // but j could be sleeping — that's fine, we want to wake it)
                ResolveCollision(i, j);
            }

            // Check 4 adjacent cells (right, bottom-right, bottom, bottom-left)
            int neighborCols[] = { col + 1, col + 1, col, col - 1 };
            int neighborRows[] = { row, row + 1, row + 1, row + 1 };

            for (int n = 0; n < 4; n++)
            {
                int nc = neighborCols[n];
                int nr = neighborRows[n];
                if (nc < 0 || nc >= GRID_COLS || nr < 0 || nr >= GRID_ROWS) continue;

                int neighborCell = nr * GRID_COLS + nc;
                for (int j = gridHeads[neighborCell]; j != -1; j = gridNext[j])
                {
                    if (j <= i) continue;
                    ResolveCollision(i, j);
                }
            }
        }
    }

    // ── Sleep management (after all substeps) ──────────────────────────────
    sleepingCount = 0;

    for (int i = 0; i < SphereCount; i++)
    {
        if (!balls[i].IsAlive) continue;

        float magSq = velMagSq(balls[i]);

        if (balls[i].IsSleeping)
        {
            // Wake up if velocity exceeds threshold (e.g. from a collision)
            if (magSq > wakeVelSq)
            {
                balls[i].IsSleeping = false;
                balls[i].sleepTimer = 0.0f;
            }
            else
            {
                sleepingCount++;
            }
        }
        else
        {
            // Track how long velocity has been below sleep threshold
            if (magSq < sleepVelSq)
            {
                balls[i].sleepTimer += dt;
                if (balls[i].sleepTimer >= SLEEP_TIME_THRESHOLD)
                {
                    // Go to sleep — zero out velocity to stop all jitter
                    balls[i].IsSleeping = true;
                    balls[i].Velocity.x = 0.0f;
                    balls[i].Velocity.y = 0.0f;
                    balls[i].sleepTimer = 0.0f;
                    sleepingCount++;
                }
            }
            else
            {
                // Reset timer if velocity spikes up
                balls[i].sleepTimer = 0.0f;
            }
        }
    }
}

// ── ResolveCollision ─────────────────────────────────────────────────────────
static void ResolveCollision(int i, int j)
{
    float dx = balls[j].Position.x - balls[i].Position.x;
    float dy = balls[j].Position.y - balls[i].Position.y;
    float distSq = dx * dx + dy * dy;

    if (distSq < MIN_DIST_SQ && distSq > 0.0001f)
    {
        // Use fast rsqrt: invDist = 1/sqrt(distSq), then dist = distSq * invDist
        float invDist = fast_rsqrt(distSq);
        float dist = distSq * invDist;
        float nx = dx * invDist;
        float ny = dy * invDist;
        float overlap = MIN_DIST - dist;

        // Separate
        float halfOverlap = overlap * 0.5f;
        balls[i].Position.x -= nx * halfOverlap;
        balls[i].Position.y -= ny * halfOverlap;
        balls[j].Position.x += nx * halfOverlap;
        balls[j].Position.y += ny * halfOverlap;

        // Elastic collision
        float relVelX = balls[j].Velocity.x - balls[i].Velocity.x;
        float relVelY = balls[j].Velocity.y - balls[i].Velocity.y;
        float velAlongNormal = relVelX * nx + relVelY * ny;

        if (velAlongNormal < 0)
        {
            float impulse = -(1.0f + Restitution) * velAlongNormal * 0.5f;
            float impX = nx * impulse;
            float impY = ny * impulse;

            // Only wake sleeping balls if the impulse is significant enough
            // to actually matter (avoids jitter wake-sleep cycles)
            float impulseMag = impulse * impulse;
            bool significantImpulse = impulseMag > 4.0f;

            if (significantImpulse)
            {
                if (balls[i].IsSleeping)
                {
                    balls[i].IsSleeping = false;
                    balls[i].sleepTimer = 0.0f;
                }
                if (balls[j].IsSleeping)
                {
                    balls[j].IsSleeping = false;
                    balls[j].sleepTimer = 0.0f;
                }
            }

            balls[i].Velocity.x -= impX;
            balls[i].Velocity.y -= impY;
            balls[j].Velocity.x += impX;
            balls[j].Velocity.y += impY;

            // Friction
            float tangentX = -ny;
            float tangentY =  nx;
            float velAlongTangent = relVelX * tangentX + relVelY * tangentY;
            float fricX = tangentX * (velAlongTangent * Friction * 0.5f);
            float fricY = tangentY * (velAlongTangent * Friction * 0.5f);
            balls[i].Velocity.x += fricX;
            balls[i].Velocity.y += fricY;
            balls[j].Velocity.x -= fricX;
            balls[j].Velocity.y -= fricY;
        }
    }
}
