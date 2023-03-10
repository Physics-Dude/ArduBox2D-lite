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

#include "World.h"
#include "Body.h"

using std::vector;
//using std::map; 
using std::pair;

typedef std::map<ArbiterKey, Arbiter>::iterator ArbIter;
typedef pair<ArbiterKey, Arbiter> ArbPair;

bool World::accumulateImpulses = true;
bool World::warmStarting = true;
bool World::positionCorrection = true;

void World::Add(Body* body)
{
  bodies.push_back(body);
}


void World::Clear()
{
  bodies.clear();
  arbiters.clear();
}

void World::BroadPhase()
{
  // O(n^2) broad-phase
  for (int i = 0; i < (int)bodies.size(); ++i)
  {
    Body* bi = bodies[i];

    for (int j = i + 1; j < (int)bodies.size(); ++j)
    {
      Body* bj = bodies[j];

      if (bi->invMass == 0.0 && bj->invMass == 0.0)
        continue;

      Arbiter newArb(bi, bj);
      ArbiterKey key(bi, bj);

      if (newArb.numContacts > 0)
      {
        ArbIter iter = arbiters.find(key);
        if (iter == arbiters.end())
        {
          arbiters.insert(ArbPair(key, newArb));
        }
        else
        {
          iter->second.Update(newArb.contacts, newArb.numContacts);
        }
      }
      else
      {
        arbiters.erase(key);
      }
    }
  }
}

void World::Step(SQ7x8 dt)
{
  SQ7x8 inv_dt = dt > 0.0 ? 1.0 / dt : 0.0;

  // Determine overlapping bodies and update contact points.
  BroadPhase();

  // Integrate forces.
  for (int i = 0; i < (int)bodies.size(); ++i)
  {
    Body* b = bodies[i];

    if (b->invMass == 0.0)
      continue;

    b->velocity += dt * (gravity + b->invMass * b->force);
    b->angularVelocity += dt * b->invI * b->torque;
  }

  // Perform pre-steps.
  for (ArbIter arb = arbiters.begin(); arb != arbiters.end(); ++arb)
  {
    arb->second.PreStep(inv_dt);
  }

  // Perform iterations
  for (int i = 0; i < iterations; ++i)
  {
    for (ArbIter arb = arbiters.begin(); arb != arbiters.end(); ++arb)
    {
      arb->second.ApplyImpulse();
    }

  }

  // Integrate Velocities
  for (int i = 0; i < (int)bodies.size(); ++i)
  {
    Body* b = bodies[i];

    b->position += dt * b->velocity;
    b->rotation += dt * b->angularVelocity;

    b->force.Set(0.0, 0.0);
    b->torque = 0.0;
  }
}
