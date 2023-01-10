/*
  Copyright (c) 2006-2009 Erin Catto http://www.gphysics.com

  Permission to use, copy, modify, distribute and sell this software
  and its documentation for any purpose is hereby granted without fee,
  provided that the above copyright notice appear in all copies.
  Erin Catto makes no representations about the suitability
  of this software for any purpose.
  It is provided "as is" without express or implied warranty.

  - Crudely modified to to use SQ7x8 fixed point numbers instead of floating points for small micros
  - Required: Pharap's  FixedPointsArduino library https://github.com/Pharap/FixedPointsArduino/
  -           mike-matera's ArduinoSTL library https://github.com/mike-matera/ArduinoSTL

*/

#include "Arbiter.h"
#include "Body.h"
#include "World.h"

#include <FixedPoints.h>
#include <FixedPointsCommon.h>
#include <ArduinoSTL.h>

Arbiter::Arbiter(Body* b1, Body* b2)
{
  if (b1 < b2)
  {
    body1 = b1;
    body2 = b2;
  }
  else
  {
    body1 = b2;
    body2 = b1;
  }

  numContacts = Collide(contacts, body1, body2);

  //friction = sqrtf(body1->friction * body2->friction); //old
  friction = sqrt(static_cast<float>(body1->friction * body2->friction));
}

void Arbiter::Update(Contact* newContacts, int numNewContacts)
{
  Contact mergedContacts[2];

  for (int i = 0; i < numNewContacts; ++i)
  {
    Contact* cNew = newContacts + i;
    int k = -1;
    for (int j = 0; j < numContacts; ++j)
    {
      Contact* cOld = contacts + j;
      if (cNew->feature.value == cOld->feature.value)
      {
        k = j;
        break;
      }
    }

    if (k > -1)
    {
      Contact* c = mergedContacts + i;
      Contact* cOld = contacts + k;
      *c = *cNew;
      if (World::warmStarting)
      {
        c->Pn = cOld->Pn;
        c->Pt = cOld->Pt;
        c->Pnb = cOld->Pnb;
      }
      else
      {
        c->Pn = 0.0;
        c->Pt = 0.0;
        c->Pnb = 0.0;
      }
    }
    else
    {
      mergedContacts[i] = newContacts[i];
    }
  }

  for (int i = 0; i < numNewContacts; ++i)
    contacts[i] = mergedContacts[i];

  numContacts = numNewContacts;
}


void Arbiter::PreStep(SQ7x8 inv_dt)
{
  const SQ7x8 k_allowedPenetration = 0.01;
  SQ7x8 k_biasFactor = World::positionCorrection ? 0.2 : 0.0;

  for (int i = 0; i < numContacts; ++i)
  {
    Contact* c = contacts + i;

    Vec2 r1 = c->position - body1->position;
    Vec2 r2 = c->position - body2->position;

    // Precompute normal mass, tangent mass, and bias.
    SQ7x8 rn1 = Dot(r1, c->normal);
    SQ7x8 rn2 = Dot(r2, c->normal);
    SQ7x8 kNormal = body1->invMass + body2->invMass;
    kNormal += body1->invI * (Dot(r1, r1) - rn1 * rn1) + body2->invI * (Dot(r2, r2) - rn2 * rn2);
    c->massNormal = 1.0 / kNormal;

    Vec2 tangent = Cross(c->normal, 1.0);
    SQ7x8 rt1 = Dot(r1, tangent);
    SQ7x8 rt2 = Dot(r2, tangent);
    SQ7x8 kTangent = body1->invMass + body2->invMass;
    kTangent += body1->invI * (Dot(r1, r1) - rt1 * rt1) + body2->invI * (Dot(r2, r2) - rt2 * rt2);
    c->massTangent = 1.0 /  kTangent;

    c->bias = -k_biasFactor * inv_dt * Min(0.0, c->separation + k_allowedPenetration);

    if (World::accumulateImpulses)
    {
      // Apply normal + friction impulse
      Vec2 P = c->Pn * c->normal + c->Pt * tangent;

      body1->velocity -= body1->invMass * P;
      body1->angularVelocity -= body1->invI * Cross(r1, P);

      body2->velocity += body2->invMass * P;
      body2->angularVelocity += body2->invI * Cross(r2, P);
    }
  }
}

void Arbiter::ApplyImpulse()
{
  Body* b1 = body1;
  Body* b2 = body2;

  for (int i = 0; i < numContacts; ++i)
  {
    Contact* c = contacts + i;
    c->r1 = c->position - b1->position;
    c->r2 = c->position - b2->position;

    // Relative velocity at contact
    Vec2 dv = b2->velocity + Cross(b2->angularVelocity, c->r2) - b1->velocity - Cross(b1->angularVelocity, c->r1);

    // Compute normal impulse
    SQ7x8 vn = Dot(dv, c->normal);

    SQ7x8 dPn = c->massNormal * (-vn + c->bias);

    if (World::accumulateImpulses)
    {
      // Clamp the accumulated impulse
      SQ7x8 Pn0 = c->Pn;
      c->Pn = Max(Pn0 + dPn, 0.0);
      dPn = c->Pn - Pn0;
    }
    else
    {
      dPn = Max(dPn, 0.0);
    }

    // Apply contact impulse
    Vec2 Pn = dPn * c->normal;

    b1->velocity -= b1->invMass * Pn;
    b1->angularVelocity -= b1->invI * Cross(c->r1, Pn);

    b2->velocity += b2->invMass * Pn;
    b2->angularVelocity += b2->invI * Cross(c->r2, Pn);

    // Relative velocity at contact
    dv = b2->velocity + Cross(b2->angularVelocity, c->r2) - b1->velocity - Cross(b1->angularVelocity, c->r1);

    Vec2 tangent = Cross(c->normal, 1.0);
    SQ7x8 vt = Dot(dv, tangent);
    SQ7x8 dPt = c->massTangent * (-vt);

    if (World::accumulateImpulses)
    {
      // Compute friction impulse
      SQ7x8 maxPt = friction * c->Pn;

      // Clamp friction
      SQ7x8 oldTangentImpulse = c->Pt;
      c->Pt = Clamp(oldTangentImpulse + dPt, -maxPt, maxPt);
      dPt = c->Pt - oldTangentImpulse;
    }
    else
    {
      SQ7x8 maxPt = friction * dPn;
      dPt = Clamp(dPt, -maxPt, maxPt);
    }

    // Apply contact impulse
    Vec2 Pt = dPt * tangent;

    b1->velocity -= b1->invMass * Pt;
    b1->angularVelocity -= b1->invI * Cross(c->r1, Pt);

    b2->velocity += b2->invMass * Pt;
    b2->angularVelocity += b2->invI * Cross(c->r2, Pt);
  }
}
