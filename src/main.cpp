#include <raylib.h>

const int windowWidth = 800;
const int windowHeight = 600;
const int radius = 100;
const Color circleColor = BLUE;

void Draw()
{
    BeginDrawing();
    ClearBackground(GRAY);
    DrawCircle(windowWidth/2, windowHeight/2, radius, circleColor);
    EndDrawing();    
}

int main()
{
    bool exit = false;
    InitWindow(windowWidth, windowHeight, "Raylib physics test");

    while ((!WindowShouldClose()))
    {
        Draw();
    }

    CloseWindow();

    return 0;
}