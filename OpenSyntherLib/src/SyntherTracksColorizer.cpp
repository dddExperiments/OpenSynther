/*
	Copyright (c) 2010 ASTRE Henri (http://www.visual-experiments.com)

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

#include "SyntherTracksColorizer.h"

#include <fstream>
#include <iostream>
#include <JpegUtils.h>
#include <omp.h>

using namespace Synther;

TracksColorizer::TracksColorizer(Manager* manager)
{
	mManager = manager;
	mPictures = std::vector<FeaturesMap>(mManager->mPictures.size());
}

TracksColorizer::~TracksColorizer()
{
	mPictures.clear();
}

void TracksColorizer::colorizeTracks()
{
	fillFeaturesMapPerPicture();
	findColorPerFeature();
	computeAverageColorPerTrack();
}

void TracksColorizer::fillFeaturesMapPerPicture()
{
	const std::vector<TrackInfo>& tracks = mManager->mTracks;
	for (unsigned int i=0; i<tracks.size(); ++i)
	{
		const TrackInfo& trackInfo = tracks[i];
		for (unsigned int j=0; j<trackInfo.viewpoints.size(); ++j)
		{
			ImageKey k = trackInfo.viewpoints[j];
			unsigned int pictureIndex = k.first;
			unsigned int featureIndex = k.second;
			const Feature& f = mManager->mPictures[pictureIndex].features[featureIndex];
			FeatureInfo finfo;
			finfo.x = f.x;
			finfo.y = f.y;
			mPictures[pictureIndex][featureIndex] = finfo;
		}
	}
}

void TracksColorizer::findColorPerFeature()
{
	int nbPictures = (int) mPictures.size(); 

	#pragma omp parallel for
	for (int i=0; i<nbPictures; ++i)
	{
		const PictureInfo& pictureInfo = mManager->mPictures[i];
		FeaturesMap& featuresMap = mPictures[i];

		Jpeg::Image img;
		Jpeg::load(pictureInfo.filepath, img);

		float scale = pictureInfo.scale;

		for (FeaturesMap::iterator it = featuresMap.begin(); it != featuresMap.end(); ++it)
		{
			FeatureInfo& finfo = (*it).second;
			img.getColor((int)(finfo.x*scale), (int)(finfo.y*scale), &finfo.color[0]);
		}
		delete[] img.buffer;
	}
}

void TracksColorizer::computeAverageColorPerTrack()
{
	std::vector<TrackInfo>& tracks = mManager->mTracks;
	int nbTracks = (int) tracks.size();
	
	#pragma omp parallel for
	for (int i=0; i<nbTracks; ++i)
	{
		TrackInfo& trackInfo = tracks[i];

		float r = 0.0f;
		float g = 0.0f;
		float b = 0.0f;
		for (unsigned int j=0; j<trackInfo.viewpoints.size(); ++j)
		{
			ImageKey& k = trackInfo.viewpoints[j];
			FeatureInfo& finfo = mPictures[k.first][k.second];
			r += finfo.color[0];
			g += finfo.color[1];
			b += finfo.color[2];
		}
		r /= trackInfo.viewpoints.size();
		g /= trackInfo.viewpoints.size();
		b /= trackInfo.viewpoints.size();

		trackInfo.color[0] = (unsigned char) r;
		trackInfo.color[1] = (unsigned char) g;
		trackInfo.color[2] = (unsigned char) b;
		trackInfo.color[3] = 255;
	}
}