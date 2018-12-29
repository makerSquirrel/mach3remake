/**
  General definitions common for Gamebuino raycasting demos.

  author: Miloslav "drummyfish" Ciz
  license: CC0 1.0
*/

#ifndef RAYCAST_DEMO_GENERAL_HPP
#define RAYCAST_DEMO_GENERAL_HPP

#ifndef SPEED_MULTIPLIER
  #define SPEED_MULTIPLIER 4
#endif

#ifndef TURN_SPEED_MULTIPLIER
  #define TURN_SPEED_MULTIPLIER 1
#endif

//#include "stdio.h" // for debugging raycastlibg

#define RCL_VERTICAL_FOV RCL_UNITS_PER_SQUARE /* Redefine camera vertical FOV:
                                         RCL_UNITS_PER_SQUARE would normally mean
                                         360 degrees, but it's not an actual
                                         angle, just linear approximation, so
                                         this is okay. */
#define RCL_PIXEL_FUNCTION pixelFunc
/* ^ This has to be defined to the name of the function that will render
     pixels. */

#include "raycastlib.h"
#include <Gamebuino-Meta.h>

#ifndef FPS
#define FPS 50
#endif

#define MS_PER_FRAME (1000 / FPS)

#ifndef PLAYER_SPEED
#define PLAYER_SPEED (4 * RCL_UNITS_PER_SQUARE)
#endif

#ifndef PLAYER_ROTATION_SPEED
#define PLAYER_ROTATION_SPEED (RCL_UNITS_PER_SQUARE / 2)
#endif

#ifndef PLAYER_JUMP_SPEED
#define PLAYER_JUMP_SPEED 500
#endif

#ifndef HEAD_BOB_HEIGHT
#define HEAD_BOB_HEIGHT 100
#endif

#ifndef HEAD_BOB_STEP
#define HEAD_BOB_STEP 10
#endif

#ifndef GRAVITY_ACCELERATION
#define GRAVITY_ACCELERATION ((3 * RCL_UNITS_PER_SQUARE) / 2)
#endif

#define SCREEN_WIDTH 80
#define SCREEN_HEIGHT 64

#define MIDDLE_ROW (SCREEN_HEIGHT / 2)
#define MIDDLE_COLUMN (SCREEN_WIDTH / 2)

#ifndef SUBSAMPLE
#define SUBSAMPLE 1
#endif

#define SUBSAMPLED_WIDTH (SCREEN_WIDTH / SUBSAMPLE)

#define TRANSPARENT_COLOR 0x8f ///< Transparent color for sprites and GUI.

#define HUE(c) (c * 16 + 8)    ///< Gives a middle color of given hue (0 to 15).

RCL_Unit zBuffer[SUBSAMPLED_WIDTH]; ///< 1D z-buffer for visibility determination.

RCL_RayConstraints defaultConstraints;

Gamebuino_Meta::Color palette[256];

inline void putPixel(int32_t x, int32_t y, uint8_t color)
{
  Gamebuino_Meta::Color c = palette[color];
  gb.display.drawPixel(x,y,c);
}

uint8_t encodeHSV(uint8_t hue, uint8_t saturation, uint8_t value)
{
  if (value > 15)
  {
    if (saturation > 84)
      return ((saturation / 85 - 1) << 7) | ((hue / 32) << 4)  | ((value - 16) / 15);
      // "normal" color, as 0bSHHHVVVV, VVVV != 0
    else
      return value / 16;
      // saturation near 0 => gray, as 0bVVVV0000, VVVV != 0
  }
  
  return 0;
  // value near 0 => black, as 0b00000000
}

void decodeHSV(uint8_t hsv, uint8_t *hue, uint8_t *saturation, uint8_t *value)
{
  uint8_t topHalf = hsv & 0b11110000;
  uint8_t bottomHalf = hsv & 0b00001111;

  if (topHalf != 0)
  {
    // "normal" color

    *value = bottomHalf != 15 ? (bottomHalf + 1) * 16 : 255;
    *saturation = (1 + ((hsv & 0b10000000) >> 7)) * 127;
    *hue = ((hsv & 0b01110000) >> 4) * 32;
  }
  else
  {
    // gray/white/black
    *hue = 0;
    *saturation = 0;
    *value = bottomHalf * 17;
  }
}

void convertHSVtoRGB(uint8_t hue, uint8_t saturation, uint8_t value,
  uint8_t *red, uint8_t *green, uint8_t *blue)
{
  #define M 16
  // ^ adds precision

  int32_t chroma = (value * saturation) / 256;

  int32_t h = (hue * M) / 42;

  int32_t a = (h % (2 * M)) - M;
  a = a < 0 ? -a : a; // abs

  int32_t x = (chroma * (M - a)) / M;

  if (h <= 1 * M)
    { *red = chroma;   *green = x;      *blue = 0; }
  else if (h <= 2 * M)
    { *red = x;        *green = chroma; *blue = 0; }
  else if (h <= 3 * M)
    { *red = 0;        *green = chroma; *blue = x; }
  else if (h <= 4 * M)
    { *red = 0;        *green = x;      *blue = chroma; }
  else if (h <= 5 * M)
    { *red = x;        *green = 0;      *blue = chroma; }
  else if (h <= 6 * M)
    { *red = chroma;   *green = 0;      *blue = x; }
  else
    { *red = 0;        *green = 0;      *blue = 0; }

  #undef M

  int32_t m = value - chroma;

  *red += m;
  *green += m;
  *blue += m;
}

/**
  Inits and loads a general 256 color palette.
*/
void initPalette()
{
  /* the palette is HSV-based because it makes brightness addition fast, which
     is important for fog/shadow diminishing */

  for (uint16_t i = 0; i < 256; ++i)
  {
    uint8_t h,s,v,r,g,b;

    decodeHSV(i,&h,&s,&v);
    convertHSVtoRGB(h,s,v,&r,&g,&b);
    palette[i] = gb.createColor(r,g,b);
  }
}

/**
  Adds given intensity to a color.

  @param color input color
  @param intensity intensity to add, can be negative, will be clamped
  @return new color
*/
inline uint8_t addIntensity(uint8_t color, int16_t intensity)
{
  int16_t newValue = (color & 0b00001111) + intensity; // value as in HSV

  if (newValue <= 0)
    return 0; // black

  if (newValue >= 16)
    newValue = 15;

  return (color & 0b11110000) | newValue;
}

/**
  Samples an image by normalized coordinates - each coordinate is in range
  0 to RCL_UNITS_PER_SQUARE (from raycastlib).
*/ 
inline uint8_t sampleImage(const unsigned char *image, RCL_Unit x, RCL_Unit y)
{
  x = RCL_wrap(x,RCL_UNITS_PER_SQUARE);
  y = RCL_wrap(y,RCL_UNITS_PER_SQUARE);

  int32_t index =
   (x / (RCL_UNITS_PER_SQUARE / TEXTURE_W)) * TEXTURE_H +
   (y / (RCL_UNITS_PER_SQUARE / TEXTURE_W));

  return image[2 + index];
}

/**
  Draws a scaled sprite on screen in an optimized way. The sprite has to be
  square in resolution for that.
*/
void inline drawSpriteSquare(const unsigned char *sprite, int16_t x, int16_t y, RCL_Unit depth, int16_t size, int16_t intensity)
{
  if (size < 0 || size > 200 || // let's not mess up with the incoming array
      sprite[0] != sprite[1])   // only draw square sprites
    return;

  int16_t samplingIndices[size];

  // optimization: precompute the indices

  for (RCL_Unit i = 0; i < size; ++i)
    samplingIndices[i] = (i * sprite[0]) / size;

  x -= size / 2;
  y -= size / 2;

  uint8_t c;

  int16_t jTo = size - max(0,y + size - 88);
  int16_t iTo = size - max(0,x + size - 110);

  for (RCL_Unit i = max(-1 * x,0); i < iTo; ++i)
  {
    int16_t xPos = x + i;

    if (zBuffer[xPos / SUBSAMPLE] <= depth)
      continue;

    int16_t columnLocation = 2 + samplingIndices[i] * sprite[0];

    for (RCL_Unit j = max(-1 * y,0); j < jTo; ++j)
    {
      c = sprite[columnLocation + samplingIndices[j]];
     
      if (c != TRANSPARENT_COLOR)
        putPixel(xPos,y + j,addIntensity(c,intensity));
    }
  }
}

/// Faster than drawSprite.
void drawImage(const unsigned char *image, int16_t x, int16_t y)
{
  for (int16_t i = 0; i < image[0]; ++i)
  {
    int16_t xPos = x + i;
    int16_t column = 2 + i * image[1];

    for (int16_t j = 0; j < image[1]; ++j)
    {
      char c = image[column + j];

      if (c != TRANSPARENT_COLOR)
        putPixel(xPos,y + j,image[column + j]);
    }
  }
}

/**
  General player class stuffed with everything the demos need. You'll probably
  want to write your own.
*/
class Player
{
public:
  RCL_Camera mCamera;
  RCL_Unit mVericalSpeed;
  RCL_Unit mVericalSpeedPrev; /* In order to detect whether player is standing
                                 on ground (for jumping) we need the derivative
                                 of vertical speed (not just the vertical
                                 speed) => we need two values. */
  bool mRunning;
  RCL_Unit mHeadBob;
  bool mHeadBobUp;

  Player()
  {
    RCL_initCamera(&mCamera);

    mCamera.position.x = 0;
    mCamera.position.y = 0;
    mCamera.direction = 0;
    mCamera.height = RCL_UNITS_PER_SQUARE * 3;
    mCamera.resolution.x = SCREEN_WIDTH / SUBSAMPLE;
    mCamera.resolution.y = SCREEN_HEIGHT;
    mCamera.shear = 0;
    mVericalSpeed = 0;
    mVericalSpeedPrev = 0;
    mRunning = false;
    mHeadBob = 0;
    mHeadBobUp = true;
  }

  void setPosition(RCL_Unit x, RCL_Unit y)
  {
    mCamera.position.x = x;
    mCamera.position.y = y;
  }

  void setPosition(RCL_Unit x, RCL_Unit y, RCL_Unit z, RCL_Unit direction)
  {
    mCamera.position.x = x;
    mCamera.position.y = y;
    mCamera.height = z;
    mCamera.direction = direction;
  }

  void setPositionSquare(int16_t squareX, int16_t squareY)
  {
    setPosition(
      squareX * RCL_UNITS_PER_SQUARE + RCL_UNITS_PER_SQUARE / 2,
      squareY * RCL_UNITS_PER_SQUARE + RCL_UNITS_PER_SQUARE / 2);
  }

  void update(int16_t moveDirection, bool strafe, int16_t turnDirection, bool jump,
    int16_t shearDirection, RCL_ArrayFunction floorHeightFunction,
    RCL_ArrayFunction ceilingHeightFunction, bool computeHeight, uint32_t dt)
  {
    RCL_Vector2D moveOffset;

    moveOffset.x = 0;
    moveOffset.y = 0;

    if (moveDirection != 0)
    {
      int16_t horizontalStep = (dt * PLAYER_SPEED * (mRunning ? 2 : 1)) / 1000 *
        (moveDirection > 0 ? 1 : -1);

      moveOffset = RCL_angleToDirection(mCamera.direction + (strafe ? RCL_UNITS_PER_SQUARE / 4 : 0));

      moveOffset.x = (moveOffset.x * horizontalStep) / RCL_UNITS_PER_SQUARE;
      moveOffset.y = (moveOffset.y * horizontalStep) / RCL_UNITS_PER_SQUARE;

      mHeadBob += mHeadBobUp ? HEAD_BOB_STEP : -HEAD_BOB_STEP;

      if (mHeadBob > HEAD_BOB_HEIGHT)
        mHeadBobUp = false;
      else if (mHeadBob < -HEAD_BOB_HEIGHT)
        mHeadBobUp = true; 
    }
    else
      mHeadBob /= 2;
 
    if (turnDirection != 0)
    {
      int16_t rotationStep = (dt * PLAYER_ROTATION_SPEED) / 512 * TURN_SPEED_MULTIPLIER;
      mCamera.direction = RCL_wrap(mCamera.direction + turnDirection * rotationStep,RCL_UNITS_PER_SQUARE);
    }

    RCL_Unit prevHeight = mCamera.height;

    RCL_moveCameraWithCollision(&mCamera,moveOffset,mVericalSpeed,
        floorHeightFunction, ceilingHeightFunction, computeHeight ? 1 : 0, 0);

    RCL_Unit heightDiff = mCamera.height - prevHeight;

    if (heightDiff == 0)
      mVericalSpeed = 0; // hit floor/ceiling

    if (jump && mVericalSpeed == 0 && mVericalSpeedPrev == 0)
      mVericalSpeed = PLAYER_JUMP_SPEED; // jump

    if (shearDirection != 0)
      mCamera.shear = RCL_clamp(mCamera.shear + shearDirection * 10,
        -1 * mCamera.resolution.y, mCamera.resolution.y);
    else
      mCamera.shear /= 2;

    mVericalSpeedPrev = mVericalSpeed;

    if (computeHeight)
      mVericalSpeed -= (dt * GRAVITY_ACCELERATION) / 1000; // gravity
  }
};

/**
  Sprite class, again just bare minimum to fit the needs. Prefer writing your
  own.
 */
class Sprite
{
public:
  const unsigned char *mImage;
  RCL_Vector2D mPosition;
  RCL_Unit mHeight;
  RCL_Unit mPixelSize;

  Sprite(const unsigned char *image, int16_t squareX, int16_t squareY, RCL_Unit z,
    RCL_Unit pixelSize):
    mImage(image),
    mPixelSize(pixelSize)
  {
    mPosition.x = squareX * RCL_UNITS_PER_SQUARE + RCL_UNITS_PER_SQUARE / 2;
    mPosition.y = squareY * RCL_UNITS_PER_SQUARE + RCL_UNITS_PER_SQUARE / 2;
    mHeight = z * RCL_UNITS_PER_SQUARE + RCL_UNITS_PER_SQUARE / 2;
  }

  Sprite():
    mImage(0), mHeight(0), mPixelSize(1)
  {
    mPosition.x = 0;
    mPosition.y = 0;
  }
};

void initGeneral()
{
  gb.begin();
  gb.setFrameRate(FPS);

  RCL_initRayConstraints(&defaultConstraints);

  initPalette(); 

  for (uint8_t i = 0; i < SUBSAMPLED_WIDTH; ++i)
    zBuffer[i] = 0;  
}

/**
  Computes an average color of given texture.
*/
unsigned char computeAverageColor(const unsigned char *texture, int16_t excludeColor=-1)
{
  uint8_t h,s,v;
  uint32_t sumH = 0;
  uint32_t sumS = 0;
  uint32_t sumV = 0;
  uint32_t pixels = texture[0] * texture[1];
  uint32_t count = 0;

  for (uint16_t i = 0; i < pixels; ++i)
  {
    uint8_t color = texture[2 + i];

    if (color == excludeColor)
      continue;

    decodeHSV(texture[2 + i],&h,&s,&v);

    sumH += h;
    sumS += s;
    sumV += v;
    count++;
  }

  return encodeHSV(sumH / count,sumS / count, sumV / count);
}

#endif
