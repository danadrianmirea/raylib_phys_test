# raylib_phys_test

A high-performance 2D physics simulation built with raylib and C++, featuring 5,000+ colliding spheres with spatial grid optimization, sleep management, and a pan/zoom camera.

## Features

- **5,000 Spheres**: Real-time collision detection and response for thousands of spheres
- **Spatial Grid Optimization**: Efficient broad-phase collision culling using a fixed-size grid (43×25 cells)
- **Sleep System**: Balls that settle below a velocity threshold are put to sleep, saving CPU cycles
- **Sleep-Based Auto-Restart**: When all balls are asleep, the simulation auto-restarts after a configurable delay
- **Pan & Zoom Camera**: WASD/Arrow keys or mouse drag to pan, mouse wheel to zoom (with zoom-to-cursor)
- **Substep Physics**: 4 substeps per frame for stable collision resolution
- **Fast Inverse Square Root**: Uses the classic Quake III `fast_rsqrt` approximation for performance on WASM
- **Pre-rendered Textures**: Circles are rendered via textured quads for efficient batching
- **Resizable Window**: Dynamic viewport resizing with camera offset adjustment
- **Desktop & Web Builds**: CMake for desktop, Emscripten for web

## Building

### Desktop Build (CMake)

Requires raylib source at `C:/raylib/raylib` (configurable in CMakeLists.txt).

```bash
mkdir build
cd build
cmake ..
cmake --build . --config Release
```

The executable will be created in the `build` directory.

### Web Build (Emscripten)

```bash
./build_web.sh
```

This will:
- Build the project using Emscripten
- Generate a web-compatible build
- Create a `web-build.zip` file ready for deployment

## Controls

| Key / Input | Action |
|---|---|
| WASD / Arrow Keys | Pan camera |
| Mouse Drag (Left Button) | Pan camera |
| Mouse Wheel | Zoom in/out (towards cursor) |
| R | Reset simulation |
| C | Reset camera to default position |

## Project Structure

- `src/main.cpp` — Single-file implementation (physics, rendering, input, UI)
- `data/` — Assets (font, sound effects)
- `lib/` — Runtime DLLs for local execution
- `CMakeLists.txt` — CMake build configuration
- `build_web.sh` — Web build script
- `custom_shell.html` — Custom HTML shell for web builds

## Technical Details

### Physics Pipeline

1. **Integration** (4 substeps): Apply gravity, velocity damping, and position update
2. **Spatial Grid Build**: Assign each ball to a grid cell; identify crowded cells (2+ balls)
3. **Wall Collision**: Ground collision with restitution and friction
4. **Sphere-Sphere Collision**: For each crowded cell, check all pairs within the cell and 4 adjacent cells
5. **Sleep Management**: Balls below velocity threshold for 1 second are marked as sleeping

### Collision Resolution

- Elastic collision with configurable restitution (0.1–0.9) and friction (0–0.5)
- Fast inverse square root for distance computation
- Axis-aligned bounding box pre-check to skip distant pairs within the same cell
- Impulse-based wake-up: only wakes sleeping balls if the collision impulse is significant

### Rendering

- Pre-rendered circle textures with bilinear filtering
- Camera-aware frustum culling using the spatial grid
- Screen-space HUD showing FPS, physics/render timing, and ball count

## License

This project is licensed under the terms specified in the `LICENSE.txt` file.