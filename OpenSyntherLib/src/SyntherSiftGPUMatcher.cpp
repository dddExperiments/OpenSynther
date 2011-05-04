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

#include <SyntherSiftGPUMatcher.h>

using namespace Synther;

SiftGPUMatchResult::SiftGPUMatchResult()
{
	nbMatch = 0;
	matches = NULL;
}

SiftGPUMatchResult::SiftGPUMatchResult(unsigned int nbMatch, int* matches)
{
	this->nbMatch = nbMatch;
	this->matches = matches;
}

SiftGPUMatcher::SiftGPUMatcher()
{
	mMatcher = new SiftMatchGPU(4096);

	if (mMatcher->VerifyContextGL() == 0)
		mIsInitialized = false;
	else
		mIsInitialized = true;

	mMatchThreshold = 0.3f; //0.0 mean few match and 1.0 many match
}

SiftGPUMatcher::~SiftGPUMatcher()
{
	delete mMatcher;
}

void SiftGPUMatcher::setDescriptors(unsigned int index, const SiftKeyDescriptors& a)
{
	mMatcher->SetDescriptors(index, (unsigned int) a.size()/128, &a[0]);
}

SiftGPUMatchResult SiftGPUMatcher::match()
{
	int nbMatchs = mMatcher->GetSiftMatch(4096, mMatchBuffer, mMatchThreshold);

	return SiftGPUMatchResult(nbMatchs, &mMatchBuffer[0][0]);
}

unsigned int SiftGPUMatcher::match(const SiftKeyDescriptors& descriptorsA, const SiftKeyDescriptors& descriptorsB, int (*matchBuffer)[2])
{
	mMatcher->SetDescriptors(0, (int) descriptorsA.size()/128, &descriptorsA[0]);
	mMatcher->SetDescriptors(1, (int) descriptorsB.size()/128, &descriptorsB[0]);
	
	return (unsigned int) mMatcher->GetSiftMatch((int)descriptorsA.size()/128, matchBuffer, mMatchThreshold);
}

void SiftGPUMatcher::setThreshold(float threshold)
{
	mMatchThreshold = threshold;
}

bool SiftGPUMatcher::isInitialized() const
{
	return mIsInitialized;
}