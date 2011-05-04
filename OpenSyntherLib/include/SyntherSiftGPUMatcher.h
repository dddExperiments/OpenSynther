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

#include <string>
#include <SiftGPU.h>

#include "SyntherSiftGPUExtractor.h"

namespace Synther
{
	struct SiftGPUMatchResult
	{
		SiftGPUMatchResult();
		SiftGPUMatchResult(unsigned int nbMatch, int* matches);

		unsigned int nbMatch;
		int* matches;
	};

	class SiftGPUMatcher
	{
		public:
			SiftGPUMatcher();
			~SiftGPUMatcher();

			void setDescriptors(unsigned int index, const SiftKeyDescriptors& descriptors);
			SiftGPUMatchResult match();
			unsigned int match(const SiftKeyDescriptors& descriptorsA, const SiftKeyDescriptors& descriptorsB, int (*matchBuffer)[2]);

			void setThreshold(float threshold);

			bool isInitialized() const;

		protected:

			SiftMatchGPU* mMatcher;
			float         mMatchThreshold;		
			int           mMatchBuffer[4096][2];
			bool          mIsInitialized;
	};
}