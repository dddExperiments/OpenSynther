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

#include "SyntherSiftGPUExtractor.h"

#include <iostream>
#include <fstream>
#include <sstream>

#define GL_RGB8                           0x8051
#define GL_RGB                            0x1907
#define GL_RGBA							  0x1908
#define GL_LUMINANCE					  0x1909
#define GL_UNSIGNED_BYTE                  0x1401

using namespace Synther;

SiftGPUExtractor::SiftGPUExtractor(unsigned int allocWidth, unsigned int allocHeight)
{	
	mSift = new SiftGPU;
	mSift->SetVerbose(-2);
	if (mSift->CreateContextGL() == SiftGPU::SIFTGPU_FULL_SUPPORTED)
	{
		mIsInitialized = true;

		setNbFeatureMax(5000);
		setFirstOctave(0);
		mSift->AllocatePyramid(allocWidth, allocHeight);
	}
	else
	{
		mIsInitialized = false;
	}
}

SiftGPUExtractor::~SiftGPUExtractor()
{
	delete mSift;
}

void SiftGPUExtractor::setFirstOctave(int firstOctave)
{
	std::stringstream output;
	output << firstOctave;
	char buffer[256];
	strcpy(buffer, output.str().c_str());
	char* args[] = {"-fo", buffer};
	mSift->ParseParam(2, args);
}

void SiftGPUExtractor::setNbFeatureMax(int nbFeatures)
{
	if (nbFeatures == -1)
		nbFeatures = 999999999;

	std::stringstream output;
	output << nbFeatures;
	char buffer[256];
	strcpy(buffer, output.str().c_str());
	char* args[] = {"-tc2", buffer};
	mSift->ParseParam(2, args);
}

bool SiftGPUExtractor::isInitialized() const
{
	return mIsInitialized;
}

bool SiftGPUExtractor::extractFeatures(const Jpeg::Image& picture, std::vector<Feature>& features, std::vector<float>& descriptors)
{	
	if (mSift->RunSIFT(picture.width, picture.height, picture.buffer, GL_RGB, GL_UNSIGNED_BYTE))
	{
		int nbFeatures = mSift->GetFeatureNum();		
		descriptors.resize(nbFeatures*Feature::descriptorLength);
		features.resize(nbFeatures);
		
		SiftKeyPoints keys(nbFeatures);
		mSift->GetFeatureVector(&keys[0], &descriptors[0]);			
		for (unsigned int i=0; i<keys.size(); ++i)
		{
			const SiftKeypoint& skp = keys[i];
			Feature& f = features[i];

			f.x           = skp.x;
			f.y           = skp.y;
			f.scale       = skp.s;
			f.orientation = skp.o;			
			f.index       = i;

			//picture.getColor((unsigned int)skp.x, (unsigned int)skp.y, &f.color[0]);
		}

		return true;
	}

	return false;
}

bool SiftGPUExtractor::extractFeatures(const IplImage* picture, std::vector<Feature>& features, std::vector<float>& descriptors)
{
	assert(picture->nChannels == 1);
	if (mSift->RunSIFT(picture->width, picture->height, picture->imageData, GL_LUMINANCE, GL_UNSIGNED_BYTE))
	{
		int nbFeatures = mSift->GetFeatureNum();		
		descriptors.resize(nbFeatures*Feature::descriptorLength);
		features.resize(nbFeatures);

		SiftKeyPoints keys(nbFeatures);
		mSift->GetFeatureVector(&keys[0], &descriptors[0]);			
		for (unsigned int i=0; i<keys.size(); ++i)
		{
			const SiftKeypoint& skp = keys[i];
			Feature& f = features[i];

			f.x           = skp.x;
			f.y           = skp.y;
			f.scale       = skp.s;
			f.orientation = skp.o;			
			f.index       = i;
		}

		return true;
	}

	return false;
}


void SiftGPUExtractor::saveDescriptors(const std::string& filename, const SiftKeyDescriptors& descriptors)
{
	std::ofstream output(filename.c_str(), std::ios::binary);
	if (output.is_open())
	{
		unsigned int nbFeature = (unsigned int) descriptors.size()/128;
		output.write((char*)&nbFeature, sizeof(nbFeature));		
		output.write((char*)&descriptors[0], sizeof(float)*128*nbFeature);
	}
	output.close();
}

SiftKeyDescriptors SiftGPUExtractor::openDescriptors(const std::string& filename)
{
	unsigned int nbFeatures = 0;
	SiftKeyDescriptors descriptors;

	std::ifstream input(filename.c_str(), std::ios::binary);
	if (input.is_open())
	{
		input.read((char*)&nbFeatures, sizeof(nbFeatures));
		descriptors = SiftKeyDescriptors(Feature::descriptorLength*nbFeatures);
		input.read((char*)&descriptors[0], nbFeatures*Feature::descriptorLength*sizeof(float));
	}
	input.close();

	return descriptors;
}

void DescriptorConverter::toFloat(unsigned char* src, float* dst, unsigned int nbDescriptors)
{
	for (unsigned int i=0; i<nbDescriptors; ++i)
	{
		unsigned char* currentSrc = src + i*Feature::descriptorLength;
		float* currentDst = dst + i*Feature::descriptorLength;

		for (unsigned int j=0; j<Feature::descriptorLength; ++j)
		{			
			currentDst[j] = currentSrc[j] / 128.0f;
		}
	}
}

void DescriptorConverter::toUnsignedChar(float* src, unsigned char* dst, unsigned int nbDescriptors)
{
	for (unsigned int i=0; i<nbDescriptors; ++i)
	{
		float* currentSrc = src + i*Feature::descriptorLength;
		unsigned char* currentDst = dst + i*Feature::descriptorLength;

		for (unsigned int j=0; j<Feature::descriptorLength; ++j)
		{			
			currentDst[j] = (unsigned char) (currentSrc[j] * 240.0f);
		}
	}
}

bool DescriptorConverter::compareFloat(float* a, float* b, unsigned int nbDescriptors)
{
	bool identical = true;
	for (unsigned int i=0; i<nbDescriptors; ++i)
	{
		float* currentA = a + i*Feature::descriptorLength;
		float* currentB = b + i*Feature::descriptorLength;
		for (unsigned int j=0; j<Feature::descriptorLength; ++j)
		{
			if (abs(currentA[j]-currentB[j]) > 1e-6)
			{
				std::cout << currentA[j] << " != " << currentB[j] << std::endl;
				identical = false;
			}
		}
	}
	return identical;
}