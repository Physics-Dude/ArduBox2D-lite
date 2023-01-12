#pragma once

//
//  Copyright (C) 2023 Pharap (@Pharap)
//
//  Licensed under the Apache License, Version 2.0 (the "License");
//  you may not use this file except in compliance with the License.
//  You may obtain a copy of the License at
//
//       http://www.apache.org/licenses/LICENSE-2.0
//
//  Unless required by applicable law or agreed to in writing, software
//  distributed under the License is distributed on an "AS IS" BASIS,
//  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
//  See the License for the specific language governing permissions and
//  limitations under the License.
//

using AngleResult = SFixed<7, 8>;

constexpr AngleResult fixedAngleTable[64] PROGMEM
{
  0.0, 0.0245, 0.0491, 0.0736, 0.098, 0.1224, 0.1467, 0.171, 0.1951, 0.2191, 0.243, 0.2667, 0.2903, 0.3137, 0.3369, 0.3599, 0.3827, 0.4052, 0.4276, 0.4496, 0.4714, 0.4929, 0.5141, 0.535, 0.5556, 0.5758, 0.5957, 0.6152, 0.6344, 0.6532, 0.6716, 0.6895, 0.7071, 0.7242, 0.741, 0.7572, 0.773, 0.7883, 0.8032, 0.8176, 0.8315, 0.8449, 0.8577, 0.8701, 0.8819, 0.8932, 0.904, 0.9142, 0.9239, 0.933, 0.9415, 0.9495, 0.9569, 0.9638, 0.97, 0.9757, 0.9808, 0.9853, 0.9892, 0.9925, 0.9952, 0.9973, 0.9988, 0.9997
};

inline AngleResult fixedAngleLookup(uint8_t index)
{
  if(index == 64)
    return 1;

  const uint16_t value = pgm_read_word(&fixedAngleTable[(index & (0x3F))]);

  return AngleResult::fromInternal(value);
}

inline AngleResult sinFixed(uint8_t brads)
{
  const uint8_t quarter = ((brads & 0xC0) >> 6);
  const uint8_t index = ((brads & 0x3F) >> 0);
  
  switch(quarter)
  {
    case 0: return fixedAngleLookup(index);
    case 1: return fixedAngleLookup(64 - index);
    case 2: return -fixedAngleLookup(index);
    case 3: return -fixedAngleLookup(64 - index);
    default: return 0;
  }
}

inline AngleResult cosFixed(uint8_t brads)
{
  const uint8_t quarter = ((brads & 0xC0) >> 6);
  const uint8_t index = ((brads & 0x3F) >> 0);

  switch(quarter)
  {
    case 0: return fixedAngleLookup(64 - index);
    case 1: return -fixedAngleLookup(index);
    case 2: return -fixedAngleLookup(64 - index);
    case 3: return fixedAngleLookup(index);
    default: return 0;
  }
}


template<unsigned Integer, unsigned Fraction>
inline constexpr SFixed<Integer, Fraction> radiansToBrads(SFixed<Integer, Fraction> value)
{
  return (value * SFixed<Integer, Fraction>(128.0 / 3.14159265));
}
template<unsigned Integer, unsigned Fraction>
inline constexpr UFixed<Integer, Fraction> radiansToBrads(UFixed<Integer, Fraction> value)
{
  return (value * UFixed<Integer, Fraction>(128.0 / 3.14159265));
}
