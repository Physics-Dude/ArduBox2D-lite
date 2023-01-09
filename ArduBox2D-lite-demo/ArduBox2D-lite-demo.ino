/*
  This is a proof of concept program for the Arduboy and other small micros that simulates the physics of
  multiple rigid bodies in 2D space. Complete with complex collisions! It is a modification of Box2D-lite
  that uses Pharap's FixedPointsArduino library in place of all the floating point operations.

  More info: https://github.com/Physics-Dude/ArduBox2D-lite

  Further reading:
    Required: Pharap's FixedPointsArduino: https://github.com/Pharap/FixedPointsArduino/
    Required: mike-matera's ArduinoSTL: https://github.com/mike-matera/ArduinoSTL
    Required: Arduboy2 (only using it for drawing) https://github.com/MLXXXp/Arduboy2

    Not required, but uses physics code from: https://github.com/erincatto/box2d-lite
    More info on Arduboy https://community.arduboy.com/t/documentation/7836
*/

#include "Arduino.h"
#include <Arduboy2.h>

#include <FixedPoints.h>
#include <FixedPointsCommon.h>

#include <ArduinoSTL.h>
#include <vector>
#include <map>

#include "World.h"
#include "Body.h"

Arduboy2 arduboy;

const SQ7x8 FLT_MAX = 127.99; //temp

namespace {
Body bodies[5];
SQ7x8 timeStep = 1.0 / 60.0;
int iterations = 2;
Vec2 gravity(0.0, -10.0);
int numBodies = 0;
int width = 64;
int height = 60;
World world(gravity, iterations);
}

//From box2d-lite but adapted for arduboy.
static void DrawBody(Body* body) {
  Mat22 R(body->rotation);
  Vec2 x = body->position;
  Vec2 h = 0.5 * body->width;

  Vec2 v1 = x + R * Vec2(-h.x, -h.y);
  Vec2 v2 = x + R * Vec2( h.x, -h.y);
  Vec2 v3 = x + R * Vec2( h.x,  h.y);
  Vec2 v4 = x + R * Vec2(-h.x,  h.y);

  //Should probably round these
  arduboy.drawLine(v1.x.getInteger(), height - v1.y.getInteger(), v2.x.getInteger(), height - v2.y.getInteger(), WHITE);
  arduboy.drawLine(v2.x.getInteger(), height - v2.y.getInteger(), v3.x.getInteger(), height - v3.y.getInteger(), WHITE);
  arduboy.drawLine(v3.x.getInteger(), height - v3.y.getInteger(), v4.x.getInteger(), height - v4.y.getInteger(), WHITE);
  arduboy.drawLine(v4.x.getInteger(), height - v4.y.getInteger(), v1.x.getInteger(), height - v1.y.getInteger(), WHITE);
}

// A vertical stack with spinning toy
static void Demo4(Body* b, bool oneMore = false) {
  if (oneMore) {
    b->Set(Vec2(7.0, 5.0), 1.0);
    b->position.Set(22, 45);
    world.Add(b);
    ++b; ++numBodies;
  }

  //the floor
  b->Set(Vec2(100.0, 20.0), FLT_MAX);
  b->friction = 0.2;
  b->position.Set(0.0, -0.5 * b->width.y);
  b->rotation = 0.0;
  world.Add(b);
  ++b; ++numBodies;

  // add a spinning body of size 10x5
  b->Set(Vec2(10.0, 5.0), 1.0);
  b->friction = 0.2;
  b->angularVelocity = 10.0;
  b->position.Set(33, 45);
  world.Add(b);
  ++b; ++numBodies;

  // add a couple more bodies in a stack
  for (int i = 0; i < 2; ++i)
  {
    b->Set(Vec2(5.0, 10.0), 1.0);
    b->friction = 0.2;
    SQ7x8 x = 40;
    //b->angularVelocity = -2.0;
    b->position.Set(x - i, 10 + 11 * i);
    world.Add(b);
    ++b; ++numBodies;
  }
}

void setup() {
  arduboy.begin();
  world.Clear();
  numBodies = 0;
  Demo4(bodies);
}

bool isRunning = true;
void loop() {
  if (!arduboy.nextFrame())
    return;
  arduboy.pollButtons();
  arduboy.clear();

  if (isRunning) world.Step(timeStep);

  for (int i = 0; i < numBodies; ++i)
    DrawBody(bodies + i);

  // Create a new body
  if (arduboy.justPressed(RIGHT_BUTTON | LEFT_BUTTON | UP_BUTTON | DOWN_BUTTON )) {
    world.Clear();
    numBodies = 0;
    Demo4(bodies, true);
  }

  //Reset the simulation.
  if (arduboy.justPressed(B_BUTTON)) {
    world.Clear();
    numBodies = 0;
    Demo4(bodies);
  }
  
  //pause
  if (arduboy.justPressed(A_BUTTON)) {
    isRunning = !isRunning;
  }
  
  arduboy.display();
}
