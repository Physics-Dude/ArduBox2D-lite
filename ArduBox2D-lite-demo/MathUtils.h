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

#ifndef MATHUTILS_H
#define MATHUTILS_H

#include <FixedPoints.h>
#include <FixedPointsCommon.h>
#include <ArduinoSTL.h>

#include "Trig.h"

//#include <math.h>
//#include <float.h>
#include <assert.h>
//#include <stdlib.h>

const SQ7x8 k_pi = 3.14159;

//PROGMEM const int16_t sinTable[] = {numbersnumbersnumbers};

struct Vec2
{
  Vec2() {}
  Vec2(SQ7x8 x, SQ7x8 y) : x(x), y(y) {}

  void Set(SQ7x8 x_, SQ7x8 y_) {
    x = x_;
    y = y_;
  }

  Vec2 operator -() {
    return Vec2(-x, -y);
  }

  void operator += (const Vec2& v)
  {
    x += v.x; y += v.y;
  }

  void operator -= (const Vec2& v)
  {
    x -= v.x; y -= v.y;
  }

  void operator *= (SQ7x8 a)
  {
    x *= a; y *= a;
  }

  SQ7x8 Length() const
  {
    return sqrt(static_cast<float>(x * x + y * y));
    //return sqrtf(static_cast<float>(x * x + y * y));
  }

  SQ7x8 x, y;
};

struct Mat22 {
  Mat22() {}
  Mat22(SQ7x8 angle)
  {
    SQ7x8 c = cosFixed(static_cast<uint8_t>(radiansToBrads(angle))),
          s = sinFixed(static_cast<uint8_t>(radiansToBrads(angle)));
    //SQ7x8 c = cos(static_cast<float>(angle)), s = sin(static_cast<float>(angle));
    //SQ7x8 c = fixedCos(angle), s = fixedSin(angle);

    col1.x = c; col2.x = -s;
    col1.y = s; col2.y = c;
  }

  Mat22(const Vec2& col1, const Vec2& col2) : col1(col1), col2(col2) {}

  Mat22 Transpose() const
  {
    return Mat22(Vec2(col1.x, col2.x), Vec2(col1.y, col2.y));
  }

  Mat22 Invert() const
  {
    SQ7x8 a = col1.x, b = col2.x, c = col1.y, d = col2.y;
    Mat22 B;
    SQ7x8 det = a * d - b * c;
    assert(det != 0.0);
    det = 1.0 / det;
    B.col1.x =  det * d;  B.col2.x = -det * b;
    B.col1.y = -det * c;  B.col2.y =  det * a;
    return B;
  }

  Vec2 col1, col2;
};

inline SQ7x8 Dot(const Vec2& a, const Vec2& b)
{
  return a.x * b.x + a.y * b.y;
}

inline SQ7x8 Cross(const Vec2& a, const Vec2& b)
{
  return a.x * b.y - a.y * b.x;
}

inline Vec2 Cross(const Vec2& a, SQ7x8 s)
{
  return Vec2(s * a.y, -s * a.x);
}

inline Vec2 Cross(SQ7x8 s, const Vec2& a)
{
  return Vec2(-s * a.y, s * a.x);
}

inline Vec2 operator * (const Mat22& A, const Vec2& v)
{
  return Vec2(A.col1.x * v.x + A.col2.x * v.y, A.col1.y * v.x + A.col2.y * v.y);
}

inline Vec2 operator + (const Vec2& a, const Vec2& b)
{
  return Vec2(a.x + b.x, a.y + b.y);
}

inline Vec2 operator - (const Vec2& a, const Vec2& b)
{
  return Vec2(a.x - b.x, a.y - b.y);
}

inline Vec2 operator * (SQ7x8 s, const Vec2& v)
{
  return Vec2(s * v.x, s * v.y);
}

inline Mat22 operator + (const Mat22& A, const Mat22& B)
{
  return Mat22(A.col1 + B.col1, A.col2 + B.col2);
}

inline Mat22 operator * (const Mat22& A, const Mat22& B)
{
  return Mat22(A * B.col1, A * B.col2);
}

inline SQ7x8 Abs(SQ7x8 a)
{
  return a > 0.0 ? a : -a;
}

inline Vec2 Abs(const Vec2& a)
{
  //return Vec2(fabsf(a.x), fabsf(a.y));//old
  return Vec2(abs(a.x), abs(a.y));
}

inline Mat22 Abs(const Mat22& A)
{
  return Mat22(Abs(A.col1), Abs(A.col2));
}

inline SQ7x8 Sign(SQ7x8 x)
{
  return x < 0.0 ? -1.0 : 1.0;
}

inline SQ7x8 Min(SQ7x8 a, SQ7x8 b)
{
  return a < b ? a : b;
}

inline SQ7x8 Max(SQ7x8 a, SQ7x8 b)
{
  return a > b ? a : b;
}
inline SQ7x8 Clamp(SQ7x8 a, SQ7x8 low, SQ7x8 high)
{
  return Max(low, Min(a, high));
}

template<typename T> inline void Swap(T& a, T& b)
{
  T tmp = a;
  a = b;
  b = tmp;
}

#endif
