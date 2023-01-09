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

#include "Arbiter.h"
#include "Body.h"

#include <FixedPoints.h>
#include <FixedPointsCommon.h>
#include <ArduinoSTL.h>
// Box vertex and edge numbering:
//
//        ^ y
//        |
//        e1
//   v2 ------ v1
//    |        |
// e2 |        | e4  --> x
//    |        |
//   v3 ------ v4
//        e3

enum Axis
{
  FACE_A_X,
  FACE_A_Y,
  FACE_B_X,
  FACE_B_Y
};

enum EdgeNumbers
{
  NO_EDGE = 0,
  EDGE1,
  EDGE2,
  EDGE3,
  EDGE4
};

struct ClipVertex
{
  ClipVertex() { fp.value = 0; }
  Vec2 v;
  FeaturePair fp;
};

void Flip(FeaturePair& fp)
{
  Swap(fp.e.inEdge1, fp.e.inEdge2);
  Swap(fp.e.outEdge1, fp.e.outEdge2);
}

int ClipSegmentToLine(ClipVertex vOut[2], ClipVertex vIn[2],
            const Vec2& normal, SQ7x8 offset, char clipEdge)
{
  // Start with no output points
  int numOut = 0;

  // Calculate the distance of end points to the line
  SQ7x8 distance0 = Dot(normal, vIn[0].v) - offset;
  SQ7x8 distance1 = Dot(normal, vIn[1].v) - offset;

  // If the points are behind the plane
  if (distance0 <= 0.0) vOut[numOut++] = vIn[0];
  if (distance1 <= 0.0) vOut[numOut++] = vIn[1];

  // If the points are on different sides of the plane
  if (distance0 * distance1 < 0.0)
  {
    // Find intersection point of edge and plane
    SQ7x8 interp = distance0 / (distance0 - distance1);
    vOut[numOut].v = vIn[0].v + interp * (vIn[1].v - vIn[0].v);
    if (distance0 > 0.0)
    {
      vOut[numOut].fp = vIn[0].fp;
      vOut[numOut].fp.e.inEdge1 = clipEdge;
      vOut[numOut].fp.e.inEdge2 = NO_EDGE;
    }
    else
    {
      vOut[numOut].fp = vIn[1].fp;
      vOut[numOut].fp.e.outEdge1 = clipEdge;
      vOut[numOut].fp.e.outEdge2 = NO_EDGE;
    }
    ++numOut;
  }

  return numOut;
}

static void ComputeIncidentEdge(ClipVertex c[2], const Vec2& h, const Vec2& pos,
                const Mat22& Rot, const Vec2& normal)
{
  // The normal is from the reference box. Convert it
  // to the incident boxe's frame and flip sign.
  Mat22 RotT = Rot.Transpose();
  Vec2 n = -(RotT * normal);
  Vec2 nAbs = Abs(n);

  if (nAbs.x > nAbs.y)
  {
    if (Sign(n.x) > 0.0)
    {
      c[0].v.Set(h.x, -h.y);
      c[0].fp.e.inEdge2 = EDGE3;
      c[0].fp.e.outEdge2 = EDGE4;

      c[1].v.Set(h.x, h.y);
      c[1].fp.e.inEdge2 = EDGE4;
      c[1].fp.e.outEdge2 = EDGE1;
    }
    else
    {
      c[0].v.Set(-h.x, h.y);
      c[0].fp.e.inEdge2 = EDGE1;
      c[0].fp.e.outEdge2 = EDGE2;

      c[1].v.Set(-h.x, -h.y);
      c[1].fp.e.inEdge2 = EDGE2;
      c[1].fp.e.outEdge2 = EDGE3;
    }
  }
  else
  {
    if (Sign(n.y) > 0.0)
    {
      c[0].v.Set(h.x, h.y);
      c[0].fp.e.inEdge2 = EDGE4;
      c[0].fp.e.outEdge2 = EDGE1;

      c[1].v.Set(-h.x, h.y);
      c[1].fp.e.inEdge2 = EDGE1;
      c[1].fp.e.outEdge2 = EDGE2;
    }
    else
    {
      c[0].v.Set(-h.x, -h.y);
      c[0].fp.e.inEdge2 = EDGE2;
      c[0].fp.e.outEdge2 = EDGE3;

      c[1].v.Set(h.x, -h.y);
      c[1].fp.e.inEdge2 = EDGE3;
      c[1].fp.e.outEdge2 = EDGE4;
    }
  }

  c[0].v = pos + Rot * c[0].v;
  c[1].v = pos + Rot * c[1].v;
}

// The normal points from A to B
int Collide(Contact* contacts, Body* bodyA, Body* bodyB)
{
  // Setup
  Vec2 hA = 0.5 * bodyA->width;
  Vec2 hB = 0.5 * bodyB->width;

  Vec2 posA = bodyA->position;
  Vec2 posB = bodyB->position;

  Mat22 RotA(bodyA->rotation), RotB(bodyB->rotation);

  Mat22 RotAT = RotA.Transpose();
  Mat22 RotBT = RotB.Transpose();

  Vec2 dp = posB - posA;
  Vec2 dA = RotAT * dp;
  Vec2 dB = RotBT * dp;

  Mat22 C = RotAT * RotB;
  Mat22 absC = Abs(C);
  Mat22 absCT = absC.Transpose();

  // Box A faces
  Vec2 faceA = Abs(dA) - hA - absC * hB;
  if (faceA.x > 0.0 || faceA.y > 0.0)
    return 0;

  // Box B faces
  Vec2 faceB = Abs(dB) - absCT * hA - hB;
  if (faceB.x > 0.0 || faceB.y > 0.0)
    return 0;

  // Find best axis
  Axis axis;
  SQ7x8 separation;
  Vec2 normal;

  // Box A faces
  axis = FACE_A_X;
  separation = faceA.x;
  normal = dA.x > 0.0 ? RotA.col1 : -RotA.col1;

  const SQ7x8 relativeTol = 0.95;
  const SQ7x8 absoluteTol = 0.01;

  if (faceA.y > relativeTol * separation + absoluteTol * hA.y)
  {
    axis = FACE_A_Y;
    separation = faceA.y;
    normal = dA.y > 0.0 ? RotA.col2 : -RotA.col2;
  }

  // Box B faces
  if (faceB.x > relativeTol * separation + absoluteTol * hB.x)
  {
    axis = FACE_B_X;
    separation = faceB.x;
    normal = dB.x > 0.0 ? RotB.col1 : -RotB.col1;
  }

  if (faceB.y > relativeTol * separation + absoluteTol * hB.y)
  {
    axis = FACE_B_Y;
    separation = faceB.y;
    normal = dB.y > 0.0 ? RotB.col2 : -RotB.col2;
  }

  // Setup clipping plane data based on the separating axis
  Vec2 frontNormal, sideNormal;
  ClipVertex incidentEdge[2];
  SQ7x8 front, negSide, posSide;
  char negEdge, posEdge;

  // Compute the clipping lines and the line segment to be clipped.
  switch (axis)
  {
  case FACE_A_X:
    {
      frontNormal = normal;
      front = Dot(posA, frontNormal) + hA.x;
      sideNormal = RotA.col2;
      SQ7x8 side = Dot(posA, sideNormal);
      negSide = -side + hA.y;
      posSide =  side + hA.y;
      negEdge = EDGE3;
      posEdge = EDGE1;
      ComputeIncidentEdge(incidentEdge, hB, posB, RotB, frontNormal);
    }
    break;

  case FACE_A_Y:
    {
      frontNormal = normal;
      front = Dot(posA, frontNormal) + hA.y;
      sideNormal = RotA.col1;
      SQ7x8 side = Dot(posA, sideNormal);
      negSide = -side + hA.x;
      posSide =  side + hA.x;
      negEdge = EDGE2;
      posEdge = EDGE4;
      ComputeIncidentEdge(incidentEdge, hB, posB, RotB, frontNormal);
    }
    break;

  case FACE_B_X:
    {
      frontNormal = -normal;
      front = Dot(posB, frontNormal) + hB.x;
      sideNormal = RotB.col2;
      SQ7x8 side = Dot(posB, sideNormal);
      negSide = -side + hB.y;
      posSide =  side + hB.y;
      negEdge = EDGE3;
      posEdge = EDGE1;
      ComputeIncidentEdge(incidentEdge, hA, posA, RotA, frontNormal);
    }
    break;

  case FACE_B_Y:
    {
      frontNormal = -normal;
      front = Dot(posB, frontNormal) + hB.y;
      sideNormal = RotB.col1;
      SQ7x8 side = Dot(posB, sideNormal);
      negSide = -side + hB.x;
      posSide =  side + hB.x;
      negEdge = EDGE2;
      posEdge = EDGE4;
      ComputeIncidentEdge(incidentEdge, hA, posA, RotA, frontNormal);
    }
    break;
  }

  // clip other face with 5 box planes (1 face plane, 4 edge planes)

  ClipVertex clipPoints1[2];
  ClipVertex clipPoints2[2];
  int np;

  // Clip to box side 1
  np = ClipSegmentToLine(clipPoints1, incidentEdge, -sideNormal, negSide, negEdge);

  if (np < 2)
    return 0;

  // Clip to negative box side 1
  np = ClipSegmentToLine(clipPoints2, clipPoints1,  sideNormal, posSide, posEdge);

  if (np < 2)
    return 0;

  // Now clipPoints2 contains the clipping points.
  // Due to roundoff, it is possible that clipping removes all points.

  int numContacts = 0;
  for (int i = 0; i < 2; ++i)
  {
    SQ7x8 separation = Dot(frontNormal, clipPoints2[i].v) - front;

    if (separation <= 0)
    {
      contacts[numContacts].separation = separation;
      contacts[numContacts].normal = normal;
      // slide contact point onto reference face (easy to cull)
      contacts[numContacts].position = clipPoints2[i].v - separation * frontNormal;
      contacts[numContacts].feature = clipPoints2[i].fp;
      if (axis == FACE_B_X || axis == FACE_B_Y)
        Flip(contacts[numContacts].feature);
      ++numContacts;
    }
  }

  return numContacts;
}
