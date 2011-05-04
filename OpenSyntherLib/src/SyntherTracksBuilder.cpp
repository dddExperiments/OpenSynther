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

#include "SyntherTracksBuilder.h"
#include "SyntherManager.h"

#include <algorithm>
#include <string>
#include <sstream>
#include <iostream>
#include <queue>

using namespace Synther;

bool CompareFirst(const FeatureMatch& m1, const FeatureMatch& m2)
{
	return m1.indexFeatureA < m2.indexFeatureA;
}

TracksBuilder::TracksBuilder(Manager* manager)
{
	mManager = manager;
}

TracksBuilder::~TracksBuilder()
{
	mMatchesMap.clear();
	mMatches.clear();
}

void TracksBuilder::fillSymetricMatchMap()
{
	const std::vector<MatchInfo>& matches = mManager->mMatches;
	mMatches.resize(matches.size());
	for (unsigned int i=0; i<matches.size(); ++i)
	{
		const MatchInfo& match = matches[i];
		unsigned int indexPictureA = match.indexPictureA;
		unsigned int indexPictureB = match.indexPictureB;

		MatchInfo symetricMatch = createSymetricMatchInfo(match);
		mMatches[i] = symetricMatch;
		mMatchesMap[MatchIndex(indexPictureB, indexPictureA).index] = &mMatches[i];
	}
	fillFastMatchAccessMap();
}

void TracksBuilder::fillFastMatchAccessMap()
{
	//Adding matches from mManager->mMatches in the map
	{
		const std::vector<MatchInfo>& matches = mManager->mMatches;
		for (unsigned int i=0; i<matches.size(); ++i)
		{
			const MatchInfo& match = matches[i];
			unsigned int indexPictureA = match.indexPictureA;
			unsigned int indexPictureB = match.indexPictureB;

			mMatchesMap[MatchIndex(indexPictureA, indexPictureB).index] = &match;
		}
	}

	//Adding matches from mMatches in the map
	{
		const std::vector<MatchInfo>& matches = mMatches;
		for (unsigned int i=0; i<matches.size(); ++i)
		{
			const MatchInfo& match = matches[i];
			unsigned int indexPictureA = match.indexPictureA;
			unsigned int indexPictureB = match.indexPictureB;

			mMatchesMap[MatchIndex(indexPictureA, indexPictureB).index] = &match;
		}
	}
}

std::vector<TrackInfo> TracksBuilder::getTracks()
{
	fillSymetricMatchMap();

	std::vector<TrackInfo> tracks;

	unsigned int nbPictures = (unsigned int) mManager->mPictures.size();

	//clear all marks for new images
	#pragma omp parallel for
	for (int i=0; i<(int)nbPictures; ++i)
	{
		mManager->createNeighborList(i);
		int numNeighbors = mManager->mPictures[i].getNumNeighbors();

		//if this image has no neighbors don't worry about its keys
		if (numNeighbors != 0)
		{
			int nbFeatures = mManager->mPictures[i].getNbFeature();
			mManager->mPictures[i].featuresFlags.resize(nbFeatures);
		}
	}

	//sort all match lists
	#pragma omp parallel for
	for (int i=0; i<(int)mMatches.size(); ++i)
	{
		std::vector<FeatureMatch>& list = mMatches[i].matches;
		std::sort(list.begin(), list.end(), CompareFirst);
	}

	#pragma omp parallel for
	for (int i=0; i<(int)mManager->mMatches.size(); ++i)
	{
		std::vector<FeatureMatch>& list = mManager->mMatches[i].matches;
		std::sort(list.begin(), list.end(), CompareFirst);
	}

	std::vector<int> touched(nbPictures);
	std::vector<bool> img_marked(nbPictures);

	for (unsigned int i=0; i<nbPictures; ++i)
		img_marked[i] = false;

	for (unsigned int i=0; i<nbPictures; ++i)
	{
		int nbFeatures  = mManager->mPictures[i].getNbFeature();
		int nbNeighbors = mManager->mPictures[i].getNumNeighbors();

		if (nbNeighbors == 0)
			continue;

		for (unsigned int j=0; j<(unsigned int)nbFeatures; ++j)
		{
			std::vector<ImageKey> features;
			std::queue<ImageKey> features_queue;

			//check if this feature was already visited
			if (mManager->mPictures[i].featuresFlags[j])
				continue;

			//reset flags
			unsigned int nbTouched = (unsigned int) touched.size();
			for (unsigned  int k=0; k<nbTouched; ++k)
				img_marked[touched[k]] = false;
			touched.clear();

			//do a breadth first search given this feature
			mManager->mPictures[i].featuresFlags[j] = true;

			features.push_back(ImageKey(i, j));
			features_queue.push(ImageKey(i, j));

			img_marked[i] = true;
			touched.push_back(i);

			while (!features_queue.empty())
			{
				ImageKey feature = features_queue.front();
				features_queue.pop();

				int img1 = feature.first;
				int f1 = feature.second;
				FeatureMatch dummy;
				dummy.indexFeatureA = f1;

				const std::vector<unsigned int>& neighbors = mManager->mPictures[img1].neighbors;
				for (unsigned int l=0; l<neighbors.size(); ++l)
				{
					unsigned int k = neighbors[l];

					if (img_marked[k])
						continue;

					unsigned int indexA = img1;
					unsigned int indexB = k;

					__int64 index = MatchIndex(indexA, indexB).index;
					assert(mMatchesMap.find(index) != mMatchesMap.end());

					const std::vector<FeatureMatch>& list = mMatchesMap[index]->matches;

					//do a binary search for the feature
					std::pair<std::vector<FeatureMatch>::const_iterator, std::vector<FeatureMatch>::const_iterator> p;

					p = equal_range(list.begin(), list.end(), dummy, CompareFirst);

					//not found
					if (p.first == p.second)
						continue;

					assert((p.first)->indexFeatureA == f1);

					unsigned int indexFeatureB = (p.first)->indexFeatureB;

					assert(indexFeatureB < mManager->mPictures[k].getNbFeature());

					//check if we already have visited this point
					if (mManager->mPictures[k].featuresFlags[indexFeatureB])
						continue;

					//mark and push the point
					mManager->mPictures[k].featuresFlags[indexFeatureB] = true;
					features.push_back(ImageKey(k, indexFeatureB));
					features_queue.push(ImageKey(k, indexFeatureB));

					img_marked[k] = true;
					touched.push_back(k);
				}
			} //while loop

			if (features.size() >= 2)
			{
				tracks.push_back(TrackInfo(features));
			} 
		} //loop over features
	} //loop over pictures

	return tracks;
}