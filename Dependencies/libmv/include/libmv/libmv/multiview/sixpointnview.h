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
//
// Projective reconstruction from six points of N views. For the case N = 3,
// the number of measurements exactly matches the degrees of freedom for the
// trifocal tensor (18); this method is one way to calculate the trifocal
// tensor.
//
// TODO(keir): Add a reference to Frederik's paper here.

#ifndef _LIBMV_MULTIVIEW_SIXPOINTNVIEW_H_
#define _LIBMV_MULTIVIEW_SIXPOINTNVIEW_H_

#include "libmv/base/vector.h"
#include "libmv/numeric/numeric.h"

namespace libmv {

struct SixPointReconstruction {
  vector<Mat34> P;
  Mat46 X;
};

/**
 * Form a projective reconstruction from 6 points visible in N views. This
 * routine is highly sensitive to noise, and should not be used on real data
 * without a robust wrapper such as RANSAC.
 *
 * The points are passed as a 2x6N matrix, with the projections of the six
 * world points shown in camera N in columns 6N to 6N+6, inclusive. There may be
 * multiple solutions, which are returned in reconstructions.
 */
void SixPointNView(const Mat2X &points,
                   vector<SixPointReconstruction> *reconstructions);

}  // namespace libmv

#endif  // _LIBMV_MULTIVIEW_SIXPOINTNVIEW_H_
