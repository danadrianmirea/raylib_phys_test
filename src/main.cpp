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
#define SPHERE_COUNT  5000
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
#define SLEEP_VELOCITY_THRESHOLD 300.0f     // velocity magnitude below this = candidate for sleep
#define SLEEP_TIME_THRESHOLD     1.0f       // seconds below threshold before going to sleep
//#define WAKE_VELOCITY_THRESHOLD  30.0f    // if a neighbor has velocity above this, wake up
#define WAKE_VELOCITY_THRESHOLD SLEEP_VELOCITY_THRESHOLD*2

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
static int crowdedCells[GRID_COLS * GRID_ROWS]; // list of cells with 2+ balls
static int crowdedCount = 0;                     // number of crowded cells

// ── Pre-rendered circle textures ─────────────────────────────────────────────
static Texture2D circleTexture;
static Texture2D highlightTexture;

// ── Performance tracking ─────────────────────────────────────────────────────
static double physicsTime = 0;
static double renderTime  = 0;
static double frameTime   = 0;
static int sleepingCount = 0;

// ── Auto-restart timers ──────────────────────────────────────────────────────
const bool AutoRestart = false;
const float RestartInterval = 10.0f;
static float restartTimer = 0;

// ── Sleep-based restart ──────────────────────────────────────────────────────
const float RestartIntervalSleep = 2.0f;
static float restartSleepTimer = 0.0f;

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

void Restart()
{
    restartTimer = 0;
    restartSleepTimer = 0.0f;
    InitBalls();
}

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
        {
            Restart();
        }

        // If AutoRestart is true, Auto-restart every RestartInterval seconds
        restartTimer += dt;
        if (restartTimer >= RestartInterval && AutoRestart == true)
        {
            Restart();
        }

        // Physics
        double physStart = GetTime();
        UpdatePhysics(dt);
        physicsTime = (GetTime() - physStart) * 1000.0;

        // Sleep-based restart: when all balls are asleep, start a timer.
        // If any ball wakes up, reset the timer. After RestartIntervalSleep
        // seconds of all balls being asleep, restart.
        if (sleepingCount == SPHERE_COUNT)
        {
            restartSleepTimer += dt;
            if (restartSleepTimer >= RestartIntervalSleep)
            {
                Restart();
            }
        }
        else
        {
            restartSleepTimer = 0.0f;
        }

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
        if(AutoRestart)
        {
            sprintf(buf, "Press R to reset  (auto: %.1fs)", timeLeft);
        }
        else
        {
            sprintf(buf, "Press R to reset");
        }
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

    // ── Phase 1: Integration substeps ──────────────────────────────────────
    // Run multiple substeps for stability.
    // Grid rebuild and collision detection happen once after all substeps.
    // Track the highest Y position to know if any ball is on screen.
    float maxY = -1e9f;
    for (int step = 0; step < substeps; step++)
    {
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

            if (balls[i].Position.y > maxY)
                maxY = balls[i].Position.y;

            if (balls[i].Position.y > GroundY + ScreenHeight * 0.5f)
                balls[i].IsAlive = false;
        }
    }

    // ── Phase 2: Build spatial grid (once per frame) ───────────────────────
    memset(gridHeads, -1, sizeof(gridHeads));
    int cellCounts[GRID_COLS * GRID_ROWS];
    memset(cellCounts, 0, sizeof(cellCounts));
    for (int i = 0; i < SphereCount; i++)
    {
        if (!balls[i].IsAlive) continue;

        int col = (int)(balls[i].Position.x / CELL_SIZE);
        int row = (int)(balls[i].Position.y / CELL_SIZE);
        col = std::max(0, std::min(col, GRID_COLS - 1));
        row = std::max(0, std::min(row, GRID_ROWS - 1));

        int cell = row * GRID_COLS + col;
        cellCounts[cell]++;
        gridNext[i] = gridHeads[cell];
        gridHeads[cell] = i;
        gridCellIdx[i] = cell;
    }
    // Build crowded cells list
    crowdedCount = 0;
    for (int cell = 0; cell < GRID_COLS * GRID_ROWS; cell++)
    {
        if (cellCounts[cell] >= 2)
            crowdedCells[crowdedCount++] = cell;
    }

    // ── Phase 3: Wall/ground collision (once per frame) ────────────────────
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

    // ── Phase 4: Sphere-sphere collision (once per frame) ──────────────────
    // If no ball is on screen yet (all above y=0), skip entirely.
    // Balls above the screen are all falling at the same speed in a column
    // and can't collide with anything meaningful.
    if (crowdedCount > 0 && maxY >= 0)
    {
        static int visitedGen = 0;
        static int visited[SPHERE_COUNT];
        visitedGen++;

        for (int ci = 0; ci < crowdedCount; ci++)
        {
            int cell = crowdedCells[ci];
            int row = cell / GRID_COLS;
            int col = cell % GRID_COLS;

            // Same cell — collide every pair
            for (int i = gridHeads[cell]; i != -1; i = gridNext[i])
            {
                if (!balls[i].IsAlive || balls[i].IsSleeping) continue;
                visited[i] = visitedGen;

                for (int j = gridNext[i]; j != -1; j = gridNext[j])
                {
                    if (!balls[j].IsAlive || balls[j].IsSleeping) continue;
                    ResolveCollision(i, j);
                }
            }

            // 4 adjacent cells
            int nc[] = { col + 1, col + 1, col, col - 1 };
            int nr[] = { row, row + 1, row + 1, row + 1 };

            for (int n = 0; n < 4; n++)
            {
                if (nc[n] < 0 || nc[n] >= GRID_COLS || nr[n] < 0 || nr[n] >= GRID_ROWS)
                    continue;

                int neighborCell = nr[n] * GRID_COLS + nc[n];
                if (gridHeads[neighborCell] == -1)
                    continue;

                for (int i = gridHeads[cell]; i != -1; i = gridNext[i])
                {
                    if (!balls[i].IsAlive || balls[i].IsSleeping) continue;

                    for (int j = gridHeads[neighborCell]; j != -1; j = gridNext[j])
                    {
                        if (!balls[j].IsAlive || balls[j].IsSleeping) continue;
                        if (visited[j] == visitedGen) continue;
                        ResolveCollision(i, j);
                    }
                }
            }
        }
    }

    // ── Sleep management ───────────────────────────────────────────────────
    sleepingCount = 0;

    for (int i = 0; i < SphereCount; i++)
    {
        if (!balls[i].IsAlive) continue;

        float magSq = velMagSq(balls[i]);

        if (balls[i].IsSleeping)
        {
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
            if (magSq < sleepVelSq)
            {
                balls[i].sleepTimer += dt;
                if (balls[i].sleepTimer >= SLEEP_TIME_THRESHOLD)
                {
                    balls[i].IsSleeping = true;
                    balls[i].Velocity.x = 0.0f;
                    balls[i].Velocity.y = 0.0f;
                    balls[i].sleepTimer = 0.0f;
                    sleepingCount++;
                }
            }
            else
            {
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

    // Quick bounding-box check: if balls are more than MIN_DIST apart
    // in either axis, they can't possibly overlap. This is critical when
    // hundreds of balls share the same grid cell (e.g. falling column).
    float adx = dx < 0 ? -dx : dx;
    float ady = dy < 0 ? -dy : dy;
    if (adx >= MIN_DIST || ady >= MIN_DIST)
        return;

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
