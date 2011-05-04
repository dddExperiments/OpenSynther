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

#include "SyntherStructures.h"

using namespace Synther;

const unsigned int Feature::descriptorLength = 128; //Sift Feature -> descriptor of 128 float

Feature::Feature()
{
	x           = 0.0f;
	y           = 0.0f;
	scale       = 0.0f;
	orientation = 0.0f;
	index       = -1;
}

PictureInfo::PictureInfo(const std::string& filepath)
{
	this->filepath     = filepath;
	this->width        = 0;
	this->height       = 0;
	this->scale        = 1.0f;
	this->focalInPixel = 0;
	this->isValid      = false;
	this->photosynth_isValid = false;
}

unsigned int PictureInfo::getNbFeature() const
{
	return (unsigned int) features.size();
}

unsigned int PictureInfo::getNumNeighbors() const
{
	return (unsigned int) neighbors.size();
}

FeatureMatch::FeatureMatch()
{
	indexFeatureA = 0;
	indexFeatureB = 0;
}

FeatureMatch::FeatureMatch(unsigned int indexFeatureA, unsigned int indexFeatureB)
{
	this->indexFeatureA = indexFeatureA;
	this->indexFeatureB = indexFeatureB;
}

MatchInfo::MatchInfo()
{
	indexPictureA = 0;
	indexPictureB = 0;
}

MatchInfo::MatchInfo(unsigned int indexPictureA, unsigned int indexPictureB)
{
	this->indexPictureA = indexPictureA;
	this->indexPictureB = indexPictureB;
}

MatchIndex::MatchIndex()
{
	this->index = 0;
}

MatchIndex::MatchIndex(__int64 index)
{
	this->index = index;
}

MatchIndex::MatchIndex(unsigned int indexPictureA, unsigned int indexPictureB)
{
	this->indexPictureA = indexPictureA;
	this->indexPictureB = indexPictureB;
}

MatchingHistogramInfo::MatchingHistogramInfo()
{
	this->pictureIndex = 0;
	this->counter      = 0;
}

MatchingHistogramInfo::MatchingHistogramInfo(unsigned int pictureIndex, unsigned int counter)
{
	this->pictureIndex = pictureIndex;
	this->counter      = counter;
}

TrackInfo::TrackInfo()
{
	isValid  = false;
	color[0] = 0;
	color[1] = 0;
	color[2] = 0;
	color[3] = 255;
}

TrackInfo::TrackInfo(const std::vector<ImageKey>& viewpoints)
{
	this->viewpoints = viewpoints;
	this->isValid = false;
	this->x = 0;
	this->y = 0;
	this->z = 0;

	color[0] = 0;
	color[1] = 0;
	color[2] = 0;
	color[3] = 255;
}