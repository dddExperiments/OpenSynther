// Copyright (c) 2009 libmv authors.
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

#ifndef LIBMV_MULTIVIEW_NVIEWTRIANGULATION_KERNEL_H
#define LIBMV_MULTIVIEW_NVIEWTRIANGULATION_KERNEL_H

#include "libmv/base/vector.h"
#include "libmv/logging/logging.h"
#include "libmv/multiview/nviewtriangulation.h"
#include "libmv/multiview/projection.h"
#include "libmv/numeric/numeric.h"

namespace libmv {
namespace nviewtriangulation {
namespace kernel {

class Kernel {
public:
	typedef Vec4 Model;
	enum { MINIMUM_SAMPLES = 2 };

	Kernel(const Mat2X &xs, const vector<Mat34> &Ps) : xs_(xs), Ps_(Ps) {
		CHECK(xs.cols() == Ps.size());
	}
	void Fit(const vector<int> &samples, vector<Model> *models) const {
		Mat2X xs = ExtractColumns(xs_, samples);

		vector<Mat34> Ps(samples.size()); //HA: I don't know if there is a similar function as ExtractColumns for vector
		for (int i=0; i<samples.size(); ++i)
			Ps[i] = Ps_[samples[i]];

		Vec4 X_homogenous;
		NViewTriangulate(xs, Ps, &X_homogenous);
		models->push_back(X_homogenous);
	}
	double Error(int sample, const Model &model) const {
		return RootMeanSquareError(xs_.col(sample), model, Ps_[sample]);
	}
	int NumSamples() const {
		return xs_.cols();
	}
private:
	const Mat2X &xs_;
	const vector<Mat34> &Ps_;
};


class AlgebraicKernel {
public:
	typedef Vec4 Model;
	enum { MINIMUM_SAMPLES = 2 };

	AlgebraicKernel(const Mat2X &xs, const vector<Mat34> &Ps) : xs_(xs), Ps_(Ps) {
		CHECK(xs.cols() == Ps.size());
	}
	void Fit(const vector<int> &samples, vector<Model> *models) const {
		Mat2X xs = ExtractColumns(xs_, samples);

		vector<Mat34> Ps(samples.size()); //HA: I don't know if there is a similar function as ExtractColumns for vector
		for (int i=0; i<samples.size(); ++i)
			Ps[i] = Ps_[samples[i]];

		Vec4 X_homogenous;
		NViewTriangulateAlgebraic(xs, Ps, &X_homogenous);
		models->push_back(X_homogenous);
	}
	double Error(int sample, const Model &model) const {
		return RootMeanSquareError(xs_.col(sample), model, Ps_[sample]);
	}
	int NumSamples() const {
		return xs_.cols();
	}
private:
	const Mat2X &xs_;
	const vector<Mat34> &Ps_;
};


}  // namespace kernel
}  // namespace nviewtriangulation
}  // namespace libmv

#endif  // LIBMV_MULTIVIEW_NVIEWTRIANGULATION_KERNEL_H
