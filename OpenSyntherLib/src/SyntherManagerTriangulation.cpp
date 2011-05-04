/*
	Copyright (c) 2011 ASTRE Henri (http://www.visual-experiments.com)

	Permission is hereby granted, free of charge, to any person obtaining a copy
	of this software and associated documentation files (the "Software"), to deal
	in the Software without restriction, including without limitation the rights
	to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
	copies of the Software, and to permit persons to whom the Software is
	furnished to do so, subject to the following conditions:

	The above copyright notice and this permission notice shall be included in
	all copies or substantial portions of the Software.

	THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
	IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
	FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
	AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
	LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
	OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
	THE SOFTWARE.
*/

#include "SyntherManager.h"

using namespace Synther;

#include "libmv/multiview/fundamental.h"
#include "libmv/multiview/projection.h"
#include "libmv/multiview/twoviewtriangulation.h"
#include "libmv/multiview/triangulation.h"
#include "libmv/multiview/nviewtriangulation.h"
#include "libmv/multiview/robust_nviewtriangulation.h"

#include <omp.h>

//Unused right now
libmv::Vec2 Manager::undistort(double f, double k1, double k2, libmv::Vec2 p)
{
	libmv::Vec2 out = p;
	double r2 = (p.x()*p.x() + p.y()*p.y()) / (f * f);
	double factor = 1.0 + k1 * r2 + k2 * r2 * r2;

	out.x() *= factor;
	out.y() *= factor;

	return out;
}

void Manager::triangulateTracks()
{
	triangulateTracksOfLength2(); //noisy because thresholding based on angle doesn't seems to works well...
	triangulateTracksOfLengthN();
}

void Manager::triangulateTracksOfLength2()
{
	libmv::Mat3 Rx, Rxt;
	Rx << 1.0,  0.0,  0.0,
		0.0, -1.0,  0.0, 
		0.0,  0.0, -1.0;
	Rxt = Rx.transpose();

	#pragma omp parallel for
	for (int i=0; i<mTracks.size(); ++i)
	{
		TrackInfo& info = mTracks[i];
		if (info.viewpoints.size() == 2)
		{
			unsigned int indexPictureA  = info.viewpoints[0].first;
			unsigned int indexFeatureA  = info.viewpoints[0].second;
			const PictureInfo& pictureA = mPictures[indexPictureA];
			const Feature& featureA     = pictureA.features[indexFeatureA];

			unsigned int indexPictureB  = info.viewpoints[1].first;
			unsigned int indexFeatureB  = info.viewpoints[1].second;
			const PictureInfo& pictureB = mPictures[indexPictureB];
			const Feature& featureB     = pictureB.features[indexFeatureB];

			if (pictureA.photosynth_isValid && pictureB.photosynth_isValid)
			{
				libmv::Mat3 K1, K2, R1, R2;
				libmv::Vec3 t1, t2, c1, c2, X_euclidean;
				libmv::Mat34 P1, P2;

				double f1x = pictureA.photosynth_focal * std::max(pictureA.width, pictureA.height);
				double f1y = f1x;
				double f2x = pictureB.photosynth_focal * std::max(pictureB.width, pictureB.height);
				double f2y = f2x;

				K1 << -f1x,   0, 0,
					    0,  f1y, 0,
					    0,    0, 1;

				K2 << -f2x,   0, 0,
					    0,  f2y, 0,
					    0,    0, 1;

				R1 = libmv::Mat3::Map(pictureA.photosynth_rotation);
				R2 = libmv::Mat3::Map(pictureB.photosynth_rotation);
				c1 = libmv::Vec3::Map(pictureA.photosynth_translation);
				c2 = libmv::Vec3::Map(pictureB.photosynth_translation);

				R1.transposeInPlace();
				R2.transposeInPlace();

				R1 = Rxt * R1.transpose();
				R2 = Rxt * R2.transpose();

				t1 = -R1*c1;
				t2 = -R2*c2;
				
				libmv::P_From_KRt(K1, R1, t1, &P1);
				libmv::P_From_KRt(K2, R2, t2, &P2);

				libmv::Vec2 x1, x2;
				x1.x() = featureA.x - pictureA.width/2;
				x1.y() = featureA.y - pictureA.height/2;
				x2.x() = featureB.x - pictureB.width/2;
				x2.y() = featureB.y - pictureB.height/2;

				libmv::Vec3 v1, v2;
				v1 = R1*libmv::Vec3(0, 0, 1.0)*pictureA.photosynth_focal + R1*libmv::Vec3(1.0, 0, 0)*x1.x() + R1*libmv::Vec3(0, 1.0, 0)*x1.y();
				v2 = R2*libmv::Vec3(0, 0, 1.0)*pictureB.photosynth_focal + R2*libmv::Vec3(1.0, 0, 0)*x2.x() + R2*libmv::Vec3(0, 1.0, 0)*x2.y();
				
				double angle = acos((double)(v1.dot(v2))/(v1.norm()*v2.norm()));
				double angleDegree = abs(angle)*180.0/3.141592;
				if (angleDegree > 15  && angleDegree < 40)
				{
					libmv::TriangulateDLT(P1, x1, P2, x2, &X_euclidean);

					double err1 = libmv::RootMeanSquareError(x1, X_euclidean, K1, R1, t1);
					double err2 = libmv::RootMeanSquareError(x2, X_euclidean, K2, R2, t2);
					if (err1 < 2 && err2 < 2)
					{
						info.x = X_euclidean.x();
						info.y = X_euclidean.y();
						info.z = X_euclidean.z();
						info.isValid = true;
					}
				}
			}
		}
	}
}

void Manager::triangulateTracksOfLengthN()
{
	libmv::Mat3 Rx, Rxt;
	Rx << 1.0,  0.0,  0.0,
	0.0, -1.0,  0.0, 
	0.0,  0.0, -1.0;
	Rxt = Rx.transpose();

	#pragma omp parallel for
	for (int i=0; i<(int)mTracks.size(); ++i)
	{
		TrackInfo& trackInfo = mTracks[i];
		
		//Remove viewpoint if the corresponding camera was not registered by PhotoSynth (or if this camera is not in coord_system 0)
		std::vector<int> validIds;
		for (unsigned int j=0; j<trackInfo.viewpoints.size(); ++j)
		{
			unsigned int pictureIndex = trackInfo.viewpoints[j].first;
			if (mPictures[pictureIndex].photosynth_isValid)
				validIds.push_back(j);
		}

		unsigned int nbValidViews = (unsigned int) validIds.size();

		if (nbValidViews >  2)
		{			
			libmv::vector<libmv::Mat34> Ps;
			libmv::Mat2X xs(2, nbValidViews);

			for (unsigned int j=0; j<nbValidViews; ++j)
			{
				unsigned int pictureIndex = trackInfo.viewpoints[validIds[j]].first;
				unsigned int featureIndex = trackInfo.viewpoints[validIds[j]].second;
				const PictureInfo& pictureInfo = mPictures[pictureIndex];
				const Feature& feature = pictureInfo.features[featureIndex];

				libmv::Mat3 K, R;
				libmv::Vec3 t, c;
				libmv::Mat34 P;

				double focal = pictureInfo.photosynth_focal * std::max(pictureInfo.width, pictureInfo.height);

				K << -focal,     0, 0,
					      0, focal, 0,
					      0,     0, 1;

				R = libmv::Mat3::Map(pictureInfo.photosynth_rotation);
				c = libmv::Vec3::Map(pictureInfo.photosynth_translation);
				R.transposeInPlace(); //because of mapping

				R = Rxt * R.transpose();

				t = -R*c;

				libmv::P_From_KRt(K, R, t, &P);

				libmv::Vec2 x;
				x.x() = feature.x - pictureInfo.width/2;
				x.y() = feature.y - pictureInfo.height/2;

				//x = undistort(focal, pictureInfo.photosynth_distort1, pictureInfo.photosynth_distort2, x);
				Ps.push_back(P);
				xs.col(j) = x;
			}
			
			double maxReprojectionError = 10.0;
			libmv::Vec4 X_homogenous;
			//double err = libmv::NViewTriangulate(xs, Ps, &X_homogenous);
			
			libmv::vector<int> inliers;
			double err = libmv::NViewTriangulateRobust(xs, Ps, maxReprojectionError, &X_homogenous, &inliers);
			if (inliers.size() > 2)
				err = libmv::NviewTriangulateConsensus(xs, Ps, &X_homogenous, &inliers);
			else
				err = maxReprojectionError+1;
			libmv::Vec3 X_euclidean = libmv::HomogeneousToEuclidean(X_homogenous);

			//removing bad viewpoint from tracks according to Ransac information
			std::vector<ImageKey> viewpoints;
			for (int i=0; i<inliers.size(); ++i)
				viewpoints.push_back(trackInfo.viewpoints[validIds[inliers[i]]]);
			trackInfo.viewpoints = viewpoints;

			if (err <= maxReprojectionError)
			{
				trackInfo.isValid = true;
				trackInfo.x = X_euclidean.x();
				trackInfo.y = X_euclidean.y();
				trackInfo.z = X_euclidean.z();
			}
		}
	}
}