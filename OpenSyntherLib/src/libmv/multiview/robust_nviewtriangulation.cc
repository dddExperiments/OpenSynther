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

#include "libmv/multiview/nviewtriangulation_kernel.h"
#include "libmv/multiview/robust_nviewtriangulation.h"
#include "libmv/multiview/robust_estimation.h"
#include "libmv/numeric/numeric.h"

namespace libmv {

	double NViewTriangulateRobust(
		const Mat2X &xs,
		const vector<Mat34> &Ps,
		double max_error,
		Vec4 *X,
		vector<int> *inliers,
		double outliers_probability) {
			// The threshold is on the sum of the squared errors in the two images.
			// Actually, Sampson's approximation of this error.
			double threshold = 2 * Square(max_error);
			double best_score = HUGE_VAL;
			typedef nviewtriangulation::kernel::Kernel Kernel;
			Kernel kernel(xs, Ps);
			*X = Estimate(kernel, MLEScorer<Kernel>(threshold), inliers, 
				&best_score, outliers_probability);

			return best_score;
	}

	double NviewTriangulateConsensus(const Mat2X &xs,
		const vector<Mat34> &Ps,
		Vec4 *X,
		vector<int> *inliers) {
						
			Mat2X xs_ = ExtractColumns(xs, *inliers);

			int nviews = inliers->size();
			vector<Mat34> Ps_(nviews); 
			for (int i=0; i<nviews; ++i)
				Ps_[i] = Ps[(*inliers)[i]];

			NViewTriangulate(xs_, Ps_, X);

			double err = 0;
			for (int i=0; i<nviews; ++i) {				
				double errTmp = RootMeanSquareError(xs_.col(i), *X, Ps_[i]);
				if (errTmp > err)
					err = errTmp;
			}
			return err;
	}

	double NViewTriangulateAlgebraicRobust(
		const Mat2X &xs,
		const vector<Mat34> &Ps,
		double max_error,
		Vec4 *X,
		vector<int> *inliers,
		double outliers_probability) {
			// The threshold is on the sum of the squared errors in the two images.
			// Actually, Sampson's approximation of this error.
			double threshold = 2 * Square(max_error);
			double best_score = HUGE_VAL;
			typedef nviewtriangulation::kernel::AlgebraicKernel Kernel;
			Kernel kernel(xs, Ps);
			*X = Estimate(kernel, MLEScorer<Kernel>(threshold), inliers, 
				&best_score, outliers_probability);
			return best_score;
	}

	double NviewTriangulateAlgebraicConsensus(const Mat2X &xs,
		const vector<Mat34> &Ps,
		Vec4 *X,
		vector<int> *inliers) {

			Mat2X xs_ = ExtractColumns(xs, *inliers);

			int nviews = inliers->size();
			vector<Mat34> Ps_(nviews); 
			for (int i=0; i<nviews; ++i)
				Ps_[i] = Ps[(*inliers)[i]];

			NViewTriangulateAlgebraic(xs_, Ps_, X);

			double err = 0;
			for (int i=0; i<nviews; ++i) {				
				double errTmp = RootMeanSquareError(xs_.col(i), *X, Ps_[i]);
				if (errTmp > err)
					err = errTmp;
			}
			return err;
	}

}  // namespace libmv
