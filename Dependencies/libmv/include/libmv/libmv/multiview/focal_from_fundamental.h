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
// Calculate the focal length from a fundamental matrix.

#ifndef LIBMV_MULTIVIEW_FOCAL_FROM_FUNDAMENTAL_H_
#define LIBMV_MULTIVIEW_FOCAL_FROM_FUNDAMENTAL_H_

#include "libmv/numeric/numeric.h"

namespace libmv {

void EpipolesFromFundamental(const Mat3 &F, Vec3 *e1, Vec3 *e2);
void RotationToEliminateY(const Vec3 &x, Mat3 *T);
void FundamentalAlignEpipolesToXAxis(const Mat3 &F, Mat3 *Fp);

void FundamentalShiftPrincipalPoints(const Mat3 &F,
                                     const Vec2 &p1,
                                     const Vec2 &p1_new,
                                     const Vec2 &p2,
                                     const Vec2 &p2_new,
                                     Mat3 *F_new);

void FocalFromFundamental(const Mat3 &F,
                          const Vec2 &principal_point1,
                          const Vec2 &principal_point2,
                          double *f1,
                          double *f2);
                          
void FocalFromFundamentalExhaustive(const Mat3 &F,
                                    const Vec2 &principal_point,
                                    const Mat2X &x1,
                                    const Mat2X &x2,
                                    double min_focal,
                                    double max_focal,
                                    int n_samples,
                                    double *focal);
                                                    
} // namespace libmv

#endif  // LIBMV_MULTIVIEW_FOCAL_FROM_FUNDAMENTAL_H_
