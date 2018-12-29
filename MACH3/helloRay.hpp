/**
  Raycasting hello world program for Gamebuino (ported from Pokitto), using
  raycastlib.

  author: Miloslav "drummyfish" Ciz
  license: CC0 1.0
*/

// #define SCREEN_WIDTH 160
// #define SCREEN_HEIGHT 128
#define SCREEN_WIDTH 80
#define SCREEN_HEIGHT 64

#define RCL_PIXEL_FUNCTION pixelFunc
/* ^ Before including raycastlib, this has to be set to the name of the
     function that will render pixels. It allows super performance. */

#define RCL_COMPUTE_WALL_TEXCOORDS 0
#define RCL_COMPUTE_FLOOR_TEXCOORDS 0
#define RCL_COMPUTE_FLOOR_DEPTH 0
#define RCL_COMPUTE_CEILING_DEPTH 0
/* ^ Turn off features we won't be using, so that the program runs faster.

  There are many other options that can be defined here, check the library
  for details. */

#include <Gamebuino-Meta.h>
#include "raycastlib.h"
// #include <utility/Graphics.h>
extern const uint8_t font5x7[];
extern const uint8_t font3x5[];
RCL_Camera camera;              // Defines a view that will be rendered.
RCL_RayConstraints constraints; // Says the details of casting individual rays.

/* Function that for given square coordinates returns height of the floor
   (in RCL_Units). */
RCL_Unit floorHeightAt(int16_t x, int16_t y)
{
  return x < 0 || x > 10 || y < 0 || y > 50 ?
    RCL_UNITS_PER_SQUARE * 2 : 0;
    /* ^ RCL_UNITS_PER_SQUARE is the length of one side of the game square.
         Since we'll place the camera at RCL_UNITS_PER_SQUARE height, let's
         make the walls twice as high. */

  /* You can either generate the level procedurally as above, or read it from
     an array and return it here. You can also animate the level here, there
     are no limits. Just keep in mind this function should be fast because
     it will get called a lot. */
}

/* Function that will be called by the library in order to draw individual
   pixels (similar to fragment shaders in OpenGL). */

void pixelFunc(RCL_PixelInfo *pixel)
{
  uint8_t color;

  /* The pixel variable holds all kind of info about the pixel to be rendered.
     Check the RCL_PixelInfo struct for details. */

  if (pixel->isWall)
    color = 4; //pixel->hit.direction + 2; // give walls different colors
  else
    color = pixel->isFloor ? 2 : 14; // also make ceiling and floor differ

  /* You can do all kinds of processing here and draw the pixel wherever you
     want, or even discard it. Just remember this function has to be fast and
     will usually be the bottleneck. */

  gb.display.setColor(color);
  gb.display.drawPixel(pixel->position.x,pixel->position.y);
}

void draw()
{
  /* This triggers the rendering, which will keep calling pixelFunc to render
     the camera view. */

  RCL_renderSimple(camera,floorHeightAt,0,0,constraints);
}

void setup()
{
  gb.begin();
  gb.display.init(SCREEN_WIDTH, SCREEN_HEIGHT, ColorMode::index);
  // gb.setFrameRate(30);
  // gb.display.setFont(font5x7);
  gb.display.setFont(font3x5);

  RCL_initCamera(&camera); /* To initialize all parameters so that none
                              remains undefined. */

  // Set the camera position to the middle of square [5;1].
  camera.position.x = 5 * RCL_UNITS_PER_SQUARE + RCL_UNITS_PER_SQUARE / 2;
  camera.position.y = 1 * RCL_UNITS_PER_SQUARE + RCL_UNITS_PER_SQUARE / 2;

  camera.height = RCL_UNITS_PER_SQUARE;

  // Set the camera resolution to Gamebuino display resolution.
  camera.resolution.x = SCREEN_WIDTH;
  camera.resolution.y = SCREEN_HEIGHT;
  camera.direction = +190;

  // This specifies the ray behavior.
  RCL_initRayConstraints(&constraints);
  constraints.maxHits = 2;   // Stop at first intersection with a wall.
  constraints.maxSteps = 42; // Trace maximum of 20 squares.
}

void loop()
{
  while(!gb.update());
  draw();
  // camera.direction -= 10; // Rotate the camera for some animation.
  camera.position.y += camera.position.y > 48*RCL_UNITS_PER_SQUARE ? 0 : 1 * RCL_UNITS_PER_SQUARE;

  // gb.tft.setCursor(0, 0);
  	// gb.tft.setColor(0, 1);
gb.display.setColor(0);
   gb.display.cursorX = 3;
    gb.display.cursorY = 3;
//         gb.tft.print(F("CPU/RAM: "));
// //         // this itself does actually cost a lot of performance!
//         gb.tft.print(gb.getCpuLoad());
//         gb.tft.print(F("/"));
//     gb.tft.print(gb.getFreeRam());
    gb.display.print(F("CPU/RAM: "));
//         // this itself does actually cost a lot of performance!
    gb.display.print(gb.getCpuLoad());
    gb.display.print(F("/"));
gb.display.print(gb.getFreeRam());
}
