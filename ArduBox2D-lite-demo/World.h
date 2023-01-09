/*
* Copyright (c) 2006-2009 Erin Catto http://www.gphysics.com
*
* Permission to use, copy, modify, distribute and sell this software
* and its documentation for any purpose is hereby granted without fee,
* provided that the above copyright notice appear in all copies.
* Erin Catto makes no representations about the suitability 
* of this software for any purpose.  
* It is provided "as is" without express or implied warranty.
* 
* 
  - Crudely modified to to use SQ7x8 fixed point numbers instead of floating points for small micros
  - Required: Pharap's  FixedPointsArduino library https://github.com/Pharap/FixedPointsArduino/
  -           mike-matera's ArduinoSTL library https://github.com/mike-matera/ArduinoSTL
    
*/

#ifndef WORLD_H
#define WORLD_H

#include <vector>
#include <map>
#include "MathUtils.h"
#include "Arbiter.h"

struct Body;

struct World
{
  World(Vec2 gravity, int iterations) : gravity(gravity), iterations(iterations) {}

  void Add(Body* body);
  void Clear();

  void Step(SQ7x8 dt);

  void BroadPhase();

  std::vector<Body*> bodies;
  std::map<ArbiterKey, Arbiter> arbiters;
  Vec2 gravity;
  int iterations;
  static bool accumulateImpulses;
  static bool warmStarting;
  static bool positionCorrection;
};

#endif
