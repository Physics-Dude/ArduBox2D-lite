/*
* Copyright (c) 2006-2007 Erin Catto http://www.gphysics.com
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

#include "Body.h"

#include <FixedPoints.h>
#include <FixedPointsCommon.h>
#include <ArduinoSTL.h>
const SQ7x8 FLT_MAX = 127.99;

Body::Body()
{
  position.Set(0.0, 0.0);
  rotation = 0.0;
  velocity.Set(0.0, 0.0);
  angularVelocity = 0.0;
  force.Set(0.0, 0.0);
  torque = 0.0;
  friction = 0.2;

  width.Set(1.0, 1.0);
  mass = FLT_MAX;
  invMass = 0.0;
  I = FLT_MAX;
  invI = 0.0;
}

void Body::Set(const Vec2& w, SQ7x8 m)
{
  position.Set(0.0, 0.0);
  rotation = 0.0;
  velocity.Set(0.0, 0.0);
  angularVelocity = 0.0;
  force.Set(0.0, 0.0);
  torque = 0.0;
  friction = 0.2;

  width = w;
  mass = m;

  if (mass < FLT_MAX)
  {
    invMass = 1.0 / mass;
    I = mass * (width.x * width.x + width.y * width.y) / 12.0;
    invI = 1.0 / I;
  }
  else
  {
    invMass = 0.0;
    I = FLT_MAX;
    invI = 0.0;
  }
}
