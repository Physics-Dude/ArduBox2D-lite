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

#ifndef BODY_H
#define BODY_H

#include "MathUtils.h"

#include <FixedPoints.h>
#include <FixedPointsCommon.h>
#include <ArduinoSTL.h>
struct Body
{
  Body();
  void Set(const Vec2& w, SQ7x8 m);

  void AddForce(const Vec2& f)
  {
    force += f;
  }

  Vec2 position;
  SQ7x8 rotation;

  Vec2 velocity;
  SQ7x8 angularVelocity;

  Vec2 force;
  SQ7x8 torque;

  Vec2 width;

  SQ7x8 friction;
  SQ7x8 mass, invMass;
  SQ7x8 I, invI;
};

#endif
