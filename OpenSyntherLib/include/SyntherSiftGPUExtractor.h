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
#include <SiftGPU.h>
#include <JpegUtils.h>
#include <opencv/cv.h>

#include "SyntherStructures.h"

typedef std::vector<SiftGPU::SiftKeypoint> SiftKeyPoints;
typedef std::vector<float> SiftKeyDescriptors;

namespace Synther
{
	class SiftGPUExtractor
	{
		public:
			SiftGPUExtractor(unsigned int allocWidth = 2048, unsigned int allocHeight = 2048);
			~SiftGPUExtractor();

			bool extractFeatures(const IplImage* picture, std::vector<Feature>& features, std::vector<float>& descriptors);
			bool extractFeatures(const Jpeg::Image& picture, std::vector<Feature>& features, std::vector<float>& descriptors);

			void setFirstOctave(int firstOctave); // [-1 0 1 2 3 4 ...]
			void setNbFeatureMax(int nbFeatures);  // -1 to disable, 5000 by default

			bool isInitialized() const;

			static SiftKeyDescriptors openDescriptors(const std::string& filename);
			static void saveDescriptors(const std::string& filename, const SiftKeyDescriptors& descriptors);		

		protected:
			SiftGPU* mSift;	
			bool     mIsInitialized;
			int      mFirstOctave;
			int      mNbFeatureMax;
	};

	class DescriptorConverter
	{
		public:
			static void toFloat(unsigned char* src, float* dst, unsigned int nbDescriptors);
			static void toUnsignedChar(float* src, unsigned char* dst, unsigned int nbDescriptors);

			static bool compareFloat(float* a, float* b, unsigned int nbDescriptors);
			//static bool compareUnsignedChar(unsigned char* a, unsigned char* b, unsigned int nbDescriptors);
	};
}