// Copyright (c) 2007, 2008 libmv authors.
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to
// deal in the Software without restriction, including without limitation the
// rights to use, copy, modify, merge, publish, distribute, sublicense, and/or
// sell copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
// FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS
// IN THE SOFTWARE.

#ifndef LIBMV_CORRESPONDENCE_FEATURE_H_
#define LIBMV_CORRESPONDENCE_FEATURE_H_

#include "libmv/numeric/numeric.h"

namespace libmv {

/**
 * Abstract base class for features.
 */
class Feature {
 public:
  virtual ~Feature();
};

class PointFeature : public Feature {
 public:
  virtual ~PointFeature();

  PointFeature(float xx=0.0f, float yy=0.0f) {
    coords[0] = xx;
    coords[1] = yy;
    scale = 0.0;
    orientation = 0.0;
  }

  float x() const { return coords(0); }
  float y() const { return coords(1); }

  Vec2f coords;       // (x, y), i.e. (column, row).
  float scale;        // In pixels.
  float orientation;  // In radians.
};

class LineFeature : public Feature {
 public:
  virtual ~LineFeature();
  virtual const Vec2f &Point1() = 0;
  virtual const Vec2f &Point2() = 0;
};

}  // namespace libmv

#endif  // LIBMV_CORRESPONDENCE_FEATURE_H_
