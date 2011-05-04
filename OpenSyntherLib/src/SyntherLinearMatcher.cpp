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

#include "SyntherLinearMatcher.h"
#include "SyntherManager.h"

#include <iostream>
#include <fstream>

#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <omp.h>

using namespace Synther;

LinearMatcher::LinearMatcher(const Manager* manager, LinearMatcherOption options)
: mManager(manager)
{
	mOptions = options;
}

LinearMatcher::~LinearMatcher()
{
	delete[] mDescriptorDatabase;
	flann_free_index(mDescriptorDatabaseIndex, NULL);
}

std::vector<Synther::MatchInfo> LinearMatcher::getMatchList()
{
	std::vector<Synther::MatchInfo> matches;

	unsigned int nbFeaturesUsedForLinearMatching = mOptions.nbFeaturesUsedPerPicture; //300
	unsigned int nbPictures    = (unsigned int) mManager->mPictures.size();
	unsigned int descLength    = Feature::descriptorLength;
	mDescriptorDatabase        = new float[nbPictures*nbFeaturesUsedForLinearMatching*descLength];

	//Fill descriptor database with 300 biggest (scale) features of each picture
	for (unsigned int i=0; i<nbPictures; ++i)
	{
		const PictureInfo& info = mManager->mPictures[i];
		std::vector<Feature> features = info.features;
		std::partial_sort(features.begin(), features.begin()+nbFeaturesUsedForLinearMatching, features.end(), FeatureSortFunction);

		for (unsigned int j=0; j<nbFeaturesUsedForLinearMatching; ++j)
		{
			float* dst = mDescriptorDatabase + i*nbFeaturesUsedForLinearMatching*descLength + j*descLength;			
			const float* src = &mManager->mPictures[i].descriptors[features[j].index];
			memcpy(dst, src, descLength*sizeof(float));
		}
	}

	mManager->clearScreen();
	std::cout << "[Building index]";
	clock_t startBuildingIndex = clock();

	//Build Flann index for fast ANN
	FLANNParameters params;
	params.algorithm        = FLANN_INDEX_KMEANS;
	params.log_level        = FLANN_LOG_INFO;
	params.memory_weight    = 0.001f;
	params.build_weight     = 0.01f;
	params.target_precision = -1;
	params.branching        = 32;
	params.centers_init     = FLANN_CENTERS_RANDOM;
	params.iterations       = 5;
	params.checks           = 172;
	params.cb_index         = (float)1.04522e+009;
	float speedup = 0;

	mDescriptorDatabaseIndex = flann_build_index(mDescriptorDatabase, nbPictures*nbFeaturesUsedForLinearMatching, descLength, &speedup, &params);

	mManager->clearScreen();
	std::cout<< "[Index built in " << floor(float(clock() - startBuildingIndex) / CLOCKS_PER_SEC)  << "s]" << std::endl;
	std::cout << "[Filling histogram]";
	clock_t startFillingHistogram = clock();

	//clear matching histogram
	mMatchingHistogram = std::vector<std::vector<MatchingHistogramInfo>>(nbPictures);
	for (unsigned int i=0; i<nbPictures; ++i)
	{
		mMatchingHistogram[i] = std::vector<MatchingHistogramInfo>(nbPictures);
		for (unsigned int j=0; j<nbPictures; ++j)
			mMatchingHistogram[i][j] = MatchingHistogramInfo(j, 0);
	}
	std::vector<std::vector<MatchingHistogramInfo>> histogram = mMatchingHistogram;
	
	//Fill histogram in parallel	
	#pragma omp parallel for
	for (int i=0; i<(int)nbPictures; ++i)
	{
		fillHistogramForLinearMatching(i, mMatchingHistogram[i]);
	}

	mManager->clearScreen();
	std::cout<< "[Histogram filled in " << floor(float(clock() - startFillingHistogram) / CLOCKS_PER_SEC)  << "s]" << std::endl;

	//Merge histogram	
 	for (unsigned int i=0; i<nbPictures; ++i)
	{
		for (unsigned int j=0; j<nbPictures; ++j)
		{
			unsigned int count = mMatchingHistogram[i][j].counter;
			histogram[i][j].counter += count;
			histogram[j][i].counter += count;
		}
	}

	mMatchingHistogram = histogram;

	exportHistogram("histogram.txt");

	/*
		Histogram  -> topRightHistogram

		0 a b c d    
		x 0 e f g	
		x x 0 h i  -> a, b, c, d, e, f, g, h, i, j
		x x x 0 j
		x x x x 0

		Then topRightHistogram is sorted according to counter value 	
	*/

	std::vector<std::pair<unsigned int, MatchingHistogramInfo*>> topRightHistogram;
	for (unsigned int i=0; i<nbPictures; ++i)
	{
		for (unsigned int j=i+1; j<nbPictures; ++j)
			topRightHistogram.push_back(std::make_pair(i, &mMatchingHistogram[i][j]));
	}
	unsigned int nbValueToBeSorted = std::min((unsigned int)mOptions.nbClosestPicturesUsed*nbPictures, (unsigned int)(nbPictures*(nbPictures-1)/2));
	std::partial_sort(topRightHistogram.begin(), topRightHistogram.begin()+nbValueToBeSorted, topRightHistogram.end(), MatchingHistogramSortFunction);

	for (unsigned int i=0; i<nbValueToBeSorted; ++i)
	{		
		unsigned int indexPictureA = topRightHistogram[i].first;
		unsigned int indexPictureB = topRightHistogram[i].second->pictureIndex;

		unsigned int indexA = std::min(indexPictureA, indexPictureB);
		unsigned int indexB = std::max(indexPictureA, indexPictureB);
		mMatchesMap[MatchIndex(indexA, indexB).index] = NULL;
	}
/*
	std::vector<std::vector<unsigned int>> pictureNeighbors(nbPictures);

	//Fill 8 closest neighbor list -> something is wrong: the neighbor list is not symetric!
	unsigned int nbClosestNeighbors = std::min((unsigned int)mOptions.nbClosestPicturesUsed, nbPictures); //8
	#pragma omp parallel for
	for (int i=0; i<(int)nbPictures; ++i)
	{
		std::partial_sort(mMatchingHistogram[i].begin(), mMatchingHistogram[i].begin()+nbClosestNeighbors, mMatchingHistogram[i].end(), MatchingHistogramInfoSortFunction);

		std::vector<unsigned int>& neighbors = pictureNeighbors[i];

		for (unsigned int j=0; j<nbClosestNeighbors; ++j)
		{
			unsigned int index = mMatchingHistogram[i][j].pictureIndex;
			if (index != i)
				neighbors.push_back(index);
		}
	}

	//Fill map with unique match
	mMatchesMap.clear();
	for (unsigned int i=0; i<nbPictures; ++i)
	{
		std::vector<unsigned int>& neighbors = pictureNeighbors[i];

		for (unsigned int j=0; j<neighbors.size(); ++j)
		{
			unsigned int indexA = std::min(i, neighbors[j]);
			unsigned int indexB = std::max(i, neighbors[j]);
			mMatchesMap[MatchIndex(indexA, indexB).index] = NULL;
		}
	}
*/
	//Fill match list using map
	for (std::map<__int64, MatchInfo*>::const_iterator it = mMatchesMap.begin(); it != mMatchesMap.end(); ++it)
	{		
		MatchIndex matchIndex = (*it).first;
		matches.push_back(MatchInfo(matchIndex.indexPictureA, matchIndex.indexPictureB));		
	}

	return matches;
}

void LinearMatcher::fillHistogramForLinearMatching(int pictureIndex, std::vector<MatchingHistogramInfo>& histogram)
{
	const PictureInfo& pictureInfo = mManager->mPictures[pictureIndex];
	unsigned int nbFeatures        = mOptions.nbFeaturesUsedPerPicture; //300

	FLANNParameters params;
	params.algorithm        = FLANN_INDEX_KMEANS;
	params.log_level        = FLANN_LOG_INFO;
	params.memory_weight    = 0.01f;
	params.build_weight     = 0.01f;
	params.target_precision = -1;
	params.branching        = 32;
	params.centers_init     = FLANN_CENTERS_RANDOM;
	params.iterations       = 5;
	params.checks           = 172;
	params.cb_index         = (float)1.04522e+009;

	//Filling histogram using Flann nearest neighbor search
	unsigned int nn = mOptions.nbNearestNeighborsInFeatureSpace; //6
	int* indices    = new int[nbFeatures*nn];
	float* dists    = new float[nbFeatures*nn];

	flann_find_nearest_neighbors_index(mDescriptorDatabaseIndex, mDescriptorDatabase + pictureIndex*nbFeatures*Feature::descriptorLength, nbFeatures, indices, dists, nn, &params);

	for (unsigned int j=0; j<nbFeatures; ++j)
	{
		for (unsigned int k=0; k<nn; ++k)
		{
			int index = (int) floor(indices[j*nn+k] / (float)nbFeatures);
			histogram[index].counter++;
		}
	}

	delete[] indices;
	delete[] dists;

	//empty the diagonal
	histogram[pictureIndex].counter = 0;
}

void LinearMatcher::exportHistogram(const std::string& filename)
{
	unsigned int nbPictures = (unsigned int) mManager->mPictures.size();
	
	//write ascii histogram
	std::ofstream output(filename.c_str());
	if (output.is_open())
	{
		for (unsigned int i=0; i<nbPictures; ++i)
		{
			for (unsigned int j=0; j<nbPictures; ++j)
				output << mMatchingHistogram[i][j].counter << ";";
			output << std::endl;
		}
	}
	output.close();

	//looking for max value to export normalized float png
	float maxValue = 1;
	for (unsigned int i=0; i<nbPictures; ++i)
	{
		for (unsigned int j=0; j<nbPictures; ++j)
		{
			if (mMatchingHistogram[i][j].counter > maxValue)
				maxValue = (float)mMatchingHistogram[i][j].counter;			
		}
	}

	//write normalized png histogram
	IplImage* histogramImg = cvCreateImage(cvSize(nbPictures, nbPictures), IPL_DEPTH_32F, 1);
	int step = histogramImg->widthStep/sizeof(float);
	float* histogramImgData = (float *)histogramImg->imageData;	
	for (unsigned int i=0; i<nbPictures; ++i)
	{
		for (unsigned int j=0; j<nbPictures; ++j)
		{
			histogramImgData[i*step+j] = (float) (mMatchingHistogram[i][j].counter / maxValue)*255;
		}
	}
	cvSaveImage("histogram.png", histogramImg);
	cvReleaseImage(&histogramImg);
}