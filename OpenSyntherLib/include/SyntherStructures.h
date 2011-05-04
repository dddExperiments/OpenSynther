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

#pragma once

#include <string>
#include <vector>

namespace Synther
{
	struct Feature
	{
		Feature();

		float x;
		float y;
		float scale;
		float orientation;
		int index;

		const static unsigned int descriptorLength;
	};

	inline bool FeatureSortFunction(const Feature& a, const Feature& b)
	{
		return a.scale > b.scale;
	}

	union MatchIndex
	{		
		MatchIndex();
		MatchIndex(__int64 index);
		MatchIndex(unsigned int indexPictureA, unsigned int indexPictureB);

		struct
		{			
			unsigned int indexPictureA : 32;
			unsigned int indexPictureB : 32;
		};
		__int64 index;
	};

	struct PictureInfo
	{
		PictureInfo(const std::string& filepath);

		std::string filepath; //absolute path to jpeg picture
		int width;            //picture with at which feature have been extracted (could be lower than actual picture width if downsizing is enabled)
		int height;           //same thing for height
		float scale;          //1.0 if picture downsizing is disabled (original picture width / downsized picture width) otherwise
		float focalInPixel;   //unused for now (until I start writing my own SFM)
		bool isValid;         //is not valid if jpeg loading failed or the picture has less than 300 features

		unsigned int getNbFeature() const;			
		std::vector<float> descriptors;
		std::vector<Feature> features;
		std::vector<unsigned int> featuresFlags; //For track building

		unsigned int getNumNeighbors() const;
		std::vector<unsigned int> neighbors;

		bool   photosynth_isValid;
		double photosynth_focal;
		double photosynth_distort1;
		double photosynth_distort2;
		double photosynth_rotation[9];
		double photosynth_translation[3];
	};

	struct FeatureMatch
	{
		FeatureMatch();
		FeatureMatch(unsigned int indexFeatureA, unsigned int indexFeatureB);

		unsigned int indexFeatureA;
		unsigned int indexFeatureB;
	};

	struct MatchInfo
	{
		MatchInfo();
		MatchInfo(unsigned int indexPictureA, unsigned int indexPictureB);

		unsigned int indexPictureA;
		unsigned int indexPictureB;
		std::vector<FeatureMatch> matches;
	};

	struct MatchingHistogramInfo
	{
		MatchingHistogramInfo();
		MatchingHistogramInfo(unsigned int pictureIndex, unsigned int counter);

		unsigned int pictureIndex;
		unsigned int counter;
	};

	inline bool MatchingHistogramInfoSortFunction(const MatchingHistogramInfo& a, const MatchingHistogramInfo& b)
	{
		return a.counter > b.counter;
	}

	inline bool MatchingHistogramSortFunction(const std::pair<unsigned int, MatchingHistogramInfo*>& a, const std::pair<unsigned int, MatchingHistogramInfo*>& b)
	{
		return a.second->counter > b.second->counter;
	}

	typedef std::pair<int,int> ImageKey; //picture index, feature index

	struct TrackInfo
	{		
		TrackInfo();
		TrackInfo(const std::vector<ImageKey>& viewpoints);

		std::vector<ImageKey> viewpoints;
		double x;
		double y;
		double z;
		bool isValid;
		unsigned char color[4]; //4 for padding
	};
}

