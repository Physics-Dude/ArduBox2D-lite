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

#ifndef ARBITER_H
#define ARBITER_H

#include "MathUtils.h"

#include <FixedPoints.h>
#include <FixedPointsCommon.h>
#include <ArduinoSTL.h>
struct Body;

union FeaturePair
{
  struct Edges
  {
    char inEdge1;
    char outEdge1;
    char inEdge2;
    char outEdge2;
  } e;
  int value;
};

struct Contact
{
  Contact() : Pn(0.0), Pt(0.0), Pnb(0.0) {}

  Vec2 position;
  Vec2 normal;
  Vec2 r1, r2;
  SQ7x8 separation;
  SQ7x8 Pn; // accumulated normal impulse
  SQ7x8 Pt; // accumulated tangent impulse
  SQ7x8 Pnb;  // accumulated normal impulse for position bias
  SQ7x8 massNormal, massTangent;
  SQ7x8 bias;
  FeaturePair feature;
};

struct ArbiterKey
{
  ArbiterKey(Body* b1, Body* b2)
  {
    if (b1 < b2)
    {
      body1 = b1; body2 = b2;
    }
    else
    {
      body1 = b2; body2 = b1;
    }
  }

  Body* body1;
  Body* body2;
};

struct Arbiter
{
  enum {MAX_POINTS = 2};

  Arbiter(Body* b1, Body* b2);

  void Update(Contact* contacts, int numContacts);

  void PreStep(SQ7x8 inv_dt);
  void ApplyImpulse();

  Contact contacts[MAX_POINTS];
  int numContacts;

  Body* body1;
  Body* body2;

  // Combined friction
  SQ7x8 friction;
};

// This is used by std::set
inline bool operator < (const ArbiterKey& a1, const ArbiterKey& a2)
{
  if (a1.body1 < a2.body1)
    return true;

  if (a1.body1 == a2.body1 && a1.body2 < a2.body2)
    return true;

  return false;
}

int Collide(Contact* contacts, Body* body1, Body* body2);

#endif
