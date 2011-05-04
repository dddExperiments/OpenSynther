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

#include <algorithm>
#include <string>
#include <sstream>
#include <iostream>
#include <boost/threadpool.hpp>
#include <boost/filesystem/operations.hpp>
#include <boost/filesystem/convenience.hpp>
#include <boost/algorithm/string.hpp> 
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <omp.h>

#include <JpegUtils.h>

#include "SyntherManager.h"
#include "SyntherLinearMatcher.h"
#include "SyntherTracksBuilder.h"
#include "SyntherTracksColorizer.h"
#include "SyntherLog.h"
#include "SyntherSiftGPUExtractor.h"
#include "SyntherSiftGPUMatcher.h"

#include <OgreMatrix3.h>
#include <OgreVector3.h>

namespace bf = boost::filesystem;

using namespace Synther;

ManagerOption::ManagerOption()
{
	//General
	verbose = false;
	camerasCCDFilepath = "cameras.xml";

	//Feature extraction		
	firstOctave         = 0;
	nbFeatureMax        = 5000;
	maxPictureDimension = 2500;

	//Feature matching
	matchingMode = MATCHINGMODE_UNSTRUCTURED_QUADRATIC;
	matchingThreshold = 0.8f;
	sequenceMatchingLength = 8;
}

LinearMatcherOption::LinearMatcherOption()
{
	nbFeaturesUsedPerPicture         = 300;
	nbNearestNeighborsInFeatureSpace = 6;
	nbClosestPicturesUsed            = 8;
}

Manager::Manager(ManagerOption& options, LinearMatcherOption& linearOptions)
: mCCDDatabaseManager(options.camerasCCDFilepath)
{
	mOptions       = options;
	mLinearOptions = linearOptions;
}

Manager::~Manager()
{

}

std::vector<std::string> Manager::getPictureList(const std::string& inputPath)
{
	std::vector<std::string> filenames;

	if (bf::is_directory(inputPath))
	{
		bf::path path = bf::path(inputPath);

		bf::directory_iterator itEnd;

		for (bf::directory_iterator it(path); it != itEnd; ++it)
		{
			if (!bf::is_directory(it->status()))
			{
				std::string filename = it->filename();
				std::string extension = bf::extension(*it).substr(1); //.JPG -> JPG
				boost::to_lower(extension);                           //jpg

				if (extension == "jpg" || extension == "png")
				{
					std::stringstream filepath;
					filepath << path << "/" << filename;

					filenames.push_back(filepath.str());
				}
			}
		}
	}

	return filenames;
}

float Manager::computeFocalLengthFromExif(Exif::Info& exifInfo, int width, int height)
{
	float focalInMM = exifInfo.focalLength;	

	//try to find CCD width in database
	float ccdWidthInMM = mCCDDatabaseManager.getCCDWidth(exifInfo.cameraMake, exifInfo.cameraModel);

	//if CCD width not found in database, try to find it in Exif info
	if (ccdWidthInMM == -1)
		ccdWidthInMM = exifInfo.CCDWidth;

	float largerSizeInPixel = (float) std::max(width, height);

	//compute focal in pixel if the CCD width was found
	if (ccdWidthInMM != 0 && ccdWidthInMM != -1)
		return largerSizeInPixel * (focalInMM/ccdWidthInMM);	
	else
		return 0;
}

void Manager::extractFeatures(const std::vector<std::string>& filenames)
{
	SiftGPUExtractor extractor;
	if (!extractor.isInitialized())
	{
		Synther::Log::write(Synther::Log::Error, "SiftGPU doesn't work on your computer");
		return;
	}
	else
	{
		extractor.setFirstOctave(mOptions.firstOctave);
		extractor.setNbFeatureMax(mOptions.nbFeatureMax);
	}

	//Loop through all pictures
	int previousPercent = 0;
	for (unsigned int i=0; i<filenames.size(); ++i)
	{
		PictureInfo info(filenames[i]);

		unsigned int width  = 0;
		unsigned int height = 0;

		//Check if jpeg is valid
		if (!Jpeg::getDimension(info.filepath, width, height))
		{			
			std::stringstream message;
			message << "Can't load Jpeg: " << info.filepath;
			Synther::Log::write(Synther::Log::Error, message.str());
			info.isValid = false;
			break;
		}

		//Load jpeg
		IplImage* srcImg = cvLoadImage(info.filepath.c_str());
		width  = srcImg->width;
		height = srcImg->height;

		//Resize jpeg if one size is larger than the max dimension allowed (keeping aspect ratio)
		unsigned int maxDim = mOptions.maxPictureDimension;
		if (width > maxDim || height > maxDim)
		{
			unsigned int newWidth;
			unsigned int newHeight;

			if (width > height)
			{
				newWidth  = maxDim;
				newHeight = (unsigned int) (maxDim*height/(float)width);
			}
			else
			{
				newHeight = maxDim;
				newWidth  = (unsigned int) (maxDim*width/(float)height);
			}
			IplImage* resizedImg = cvCreateImage(cvSize(newWidth, newHeight), srcImg->depth, srcImg->nChannels);
			cvResize(srcImg, resizedImg);
			cvReleaseImage(&srcImg);
			info.scale = width/(float)newWidth;

			srcImg = resizedImg;
			width  = srcImg->width;
			height = srcImg->height;
		}

		//Convert image from RGB to Gray
		if (srcImg->nChannels != 1)
		{
			IplImage* grayImg = cvCreateImage(cvSize(width, height), srcImg->depth, 1);
			cvCvtColor(srcImg, grayImg, CV_BGR2GRAY);
			cvReleaseImage(&srcImg);

			srcImg = grayImg;
		}
		info.width  = width;
		info.height = height;

		//Need to compute focal length using Exif data

		//Extract feature with SiftGPU
		bool featureExtracted = extractor.extractFeatures(srcImg, info.features, info.descriptors);
		info.isValid = featureExtracted && info.features.size() > 300;

		cvReleaseImage(&srcImg);
		
		int percent = (int)(((i+1)*100.0f) / (1.0f*filenames.size()));	
		if (percent != previousPercent)
		{
			previousPercent = percent;
			clearScreen();
			std::cout << "[Extracting features: " << percent << "%] - (" << i+1 << "/" << filenames.size()<<", #"<< info.features.size() <<" features)";
		}
		
		if (info.isValid)
			mPictures.push_back(info);
		else
		{
			std::stringstream message;
			message << info.filepath << " was removed because it can't be processed by SiftGPU or less than 300 features were found";
			Synther::Log::write(Synther::Log::Error, message.str());
		}
	}
	clearScreen();
}

void Manager::createUnstructuredLinearMatchList()
{		
	LinearMatcher matcher(this, mLinearOptions);
	mMatches = matcher.getMatchList();
}

void Manager::createUnstructuredQuadraticMatchList()
{
	unsigned int nbPictures = (unsigned int) mPictures.size();
	for (unsigned int i=0; i<nbPictures-1; ++i)
		for (unsigned int j=i+1; j<nbPictures; ++j)
			mMatches.push_back(MatchInfo(i, j));
}

void Manager::createSequenceLinearMatchList(bool isClosed)
{
	int nbPictures = (int) mPictures.size();

	if (isClosed) //closed loop (N*m) -> O(N) complexity (N: nbPicture and M: sequenceMatchingLength)
	{
		for (int i=0; i<nbPictures; ++i)
		{
			for (int j=1; j<=mOptions.sequenceMatchingLength; ++j)
			{
				int indexA = i;
				int indexB = i+j;

				if (indexB >= nbPictures)
				{
					mMatches.push_back(MatchInfo(indexB-nbPictures, indexA));
				}
				else
				{
					mMatches.push_back(MatchInfo(indexA, indexB));
				}
			}
		}
	}
	else //opened loop (N-m)*m + m*(m-1)/2 -> O(N) complexity (N: nbPicture and M: sequenceMatchingLength)
	{
		for (int i=0; i<nbPictures-1; ++i)
		{
			for (int j=1; j<=mOptions.sequenceMatchingLength; ++j)
			{
				int indexA = i;
				int indexB = i+j;

				if (indexB >= nbPictures)
					break;
				else
				{
					mMatches.push_back(MatchInfo(indexA, indexB));
				}
			}
		}
	}
}

void Manager::matchFeatures()
{
	SiftGPUMatcher matcher;
	if (!matcher.isInitialized())
	{
		Synther::Log::write(Synther::Log::Error, "SiftGPU::Matcher doesn't work on your computer");
		return;
	}
	else
	{
		matcher.setThreshold(mOptions.matchingThreshold);
	}

	if (mOptions.matchingMode == MATCHINGMODE_UNSTRUCTURED_LINEAR)
		createUnstructuredLinearMatchList();
	else if (mOptions.matchingMode == MATCHINGMODE_UNSTRUCTURED_QUADRATIC)
		createUnstructuredQuadraticMatchList();
	else if (mOptions.matchingMode == MATCHINGMODE_OPENED_SEQUENCE_LINEAR)
		createSequenceLinearMatchList(false);
	else //if (mOptions.matchingMode == MATCHINGMODE_CLOSED_SEQUENCE_LINEAR)
		createSequenceLinearMatchList(true);

	int previousPercent = 0;
	for (unsigned int i=0; i<mMatches.size(); ++i)
	{
		MatchInfo& info = mMatches[i];
		unsigned int indexPictureA = info.indexPictureA;
		unsigned int indexPictureB = info.indexPictureB;

		assert(indexPictureA < indexPictureB);

		int (*matchBuffer)[2] = new int[mPictures[indexPictureA].getNbFeature()][2];
		unsigned int nbMatch = matcher.match(mPictures[indexPictureA].descriptors, mPictures[indexPictureB].descriptors, matchBuffer);

		info.matches.resize(nbMatch);

		for (unsigned int j=0; j<nbMatch; ++j)
		{
			info.matches[j].indexFeatureA = (unsigned int) matchBuffer[j][0];
			info.matches[j].indexFeatureB = (unsigned int) matchBuffer[j][1];
		}

		int percent = (int)(((i+1)*100.0f) / (1.0f*mMatches.size()));
		if (percent != previousPercent)
		{
			previousPercent = percent;
			clearScreen();			
			std::cout << "[Matching features: " << percent << "%] - (" << i+1 << "/" << mMatches.size()<<", #"<< nbMatch <<" match)";
		}
		delete[] matchBuffer;
	}

/*
	for (unsigned int i=0; i<mMatches.size(); ++i)
	{		
		MatchInfo& info = mMatches[i];
		unsigned int indexPictureA = info.indexPictureA;
		unsigned int indexPictureB = info.indexPictureB;

		matcher.setDescriptors(0, mPictures[indexPictureA].descriptors);
		matcher.setDescriptors(1, mPictures[indexPictureB].descriptors);
		
		SiftGPUMatchResult matchResult = matcher.match();
		unsigned int nbMatch = matchResult.nbMatch;
		info.matches.resize(nbMatch);
		
		assert(nbMatch < 4096);

		for (unsigned int j=0; j<nbMatch; ++j)
		{
			info.matches[j].indexFeatureA = matchResult.matches[j*2+0];
			info.matches[j].indexFeatureB = matchResult.matches[j*2+1];
		}

		int percent = (int)(((i+1)*100.0f) / (1.0f*mMatches.size()));
		if (percent != previousPercent)
		{
			previousPercent = percent;
			clearScreen();			
			std::cout << "[Matching features: " << percent << "%] - (" << i+1 << "/" << mMatches.size()<<", #"<< nbMatch <<" match)";
		}
	}
*/
	clearScreen();
}

void Manager::pruneMatches()
{	
	{
		boost::threadpool::pool tp(boost::thread::hardware_concurrency());
		for (unsigned int i=0; i<mMatches.size(); ++i)
			tp.schedule(boost::bind(&Manager::pruneMatch, this, &mMatches[i]));
	}
}

void Manager::pruneMatch(MatchInfo* matchInfo)
{
	std::vector<FeatureMatch>& matches = matchInfo->matches;

	//Fill reverse map
	std::map<unsigned int, std::vector<unsigned int>> reverseMatches;
	for (unsigned int i=0; i<matches.size(); ++i)
	{
		unsigned int indexFeatureA = matches[i].indexFeatureA;
		unsigned int indexFeatureB = matches[i].indexFeatureB;
		if (reverseMatches.find(indexFeatureB) == reverseMatches.end())
			reverseMatches[indexFeatureB] = std::vector<unsigned int>();
		reverseMatches[indexFeatureB].push_back(indexFeatureA);
	}

	//Empty all matches
	matches.clear();

	//Fill only unique matches (pruning)
	for (std::map<unsigned int, std::vector<unsigned int>>::const_iterator it = reverseMatches.begin(); it != reverseMatches.end(); ++it)
	{
		if (it->second.size() == 1)
		{
			unsigned int indexFeatureA = it->second[0];
			unsigned int indexFeatureB = it->first;
			matches.push_back(FeatureMatch(indexFeatureA, indexFeatureB));
		}
	}
}

void Manager::estimateFundamentalMatrix()
{
	{
		boost::threadpool::pool tp(boost::thread::hardware_concurrency());
		for (unsigned int i=0; i<mMatches.size(); ++i)
		{
			if (mMatches[i].matches.size() > 12)
				tp.schedule(boost::bind(&Manager::estimateFundamentalMatrix, this, &mMatches[i]));
		}
	}
}

void Manager::estimateFundamentalMatrix(MatchInfo* info)
{
	unsigned int nbMatchs = (unsigned int) info->matches.size();

	CvMat* pointsMat1 = cvCreateMat(1, nbMatchs, CV_32FC2);
	CvMat* pointsMat2 = cvCreateMat(1, nbMatchs, CV_32FC2);
	CvMat* status     = cvCreateMat(1, nbMatchs, CV_8UC1);
	CvMat* fundMatrix = cvCreateMat(3, 3, CV_32FC1);

	const std::vector<Feature> featuresA = mPictures[info->indexPictureA].features;
	const std::vector<Feature> featuresB = mPictures[info->indexPictureB].features;

	//fill list of 2D point for Ransac fundamental matrix estimation
	for (unsigned int i=0; i<nbMatchs; ++i)
	{
		unsigned int indexFeatureA = info->matches[i].indexFeatureA;
		unsigned int indexFeatureB = info->matches[i].indexFeatureB;

		assert(indexFeatureA < featuresA.size());
		assert(indexFeatureB < featuresB.size());

		pointsMat1->data.fl[i*2+0] = featuresA[indexFeatureA].x;
		pointsMat1->data.fl[i*2+1] = featuresA[indexFeatureA].y;

		pointsMat2->data.fl[i*2+0] = featuresB[indexFeatureB].x;
		pointsMat2->data.fl[i*2+1] = featuresB[indexFeatureB].y;
	}

	//estimate fundamental matrix and gives status of matches (inliers/outliers)
	int fm_count = cvFindFundamentalMat(pointsMat1, pointsMat2, fundMatrix, CV_FM_RANSAC, 3, 0.99, status);

	//copy only inliers
	std::vector<FeatureMatch> matches;
	for (unsigned int i=0; i<nbMatchs; ++i)
	{		
		if (status->data.ptr[i] == 1) //0 for outliers and 1 for inliers
			matches.push_back(info->matches[i]);
	}

	//replace matches with only inliers
	info->matches = matches;
	//TODO: copy fundamental matrix for future use

	cvReleaseMat(&pointsMat1);
	cvReleaseMat(&pointsMat2);
	cvReleaseMat(&fundMatrix);
}

int Manager::buildTracks()
{
	fillFastMatchAccessMap();
	TracksBuilder tracksBuilder(this);
	mTracks = tracksBuilder.getTracks();
	trackSanityCheck();

	return (int) mTracks.size();
}

void Manager::colorizeTracks()
{
	TracksColorizer colorizer(this);
	colorizer.colorizeTracks();
}

void Manager::createNeighborList(unsigned int pictureIndex)
{
	std::vector<unsigned int> neighbors;
	for (unsigned int i=0; i<mPictures.size(); ++i)
	{
		if (i == pictureIndex)
			continue;

		unsigned int indexA = std::min(i, pictureIndex);
		unsigned int indexB = std::max(i, pictureIndex);

		__int64 index = MatchIndex(indexA, indexB).index;
		if (mMatchesMap.find(index) != mMatchesMap.end())
			neighbors.push_back(i);
	}
	mPictures[pictureIndex].neighbors = neighbors;
}

void Manager::fillFastMatchAccessMap()
{
	mMatchesMap.clear();
	for (unsigned int i=0; i<mMatches.size(); ++i)
	{
		unsigned int indexPictureA = mMatches[i].indexPictureA;
		unsigned int indexPictureB = mMatches[i].indexPictureB;
		assert(indexPictureA < indexPictureB);

		__int64 index = MatchIndex(indexPictureA, indexPictureB).index;
		mMatchesMap[index] = &mMatches[i];
	}
}

void Manager::clearScreen()
{
	std::cout << "\r                                                                          \r";
}

void Manager::matchSanityCheck()
{
	for (unsigned int i=0; i<mMatches.size(); ++i)
	{
		const MatchInfo& match = mMatches[i];
		unsigned int nbFeatureA = (unsigned int) mPictures[match.indexPictureA].features.size();
		unsigned int nbFeatureB = (unsigned int) mPictures[match.indexPictureB].features.size();

		for (unsigned int j=0; j<match.matches.size(); ++j)
		{
			unsigned int indexFeatureA  = match.matches[j].indexFeatureA;
			unsigned int indexFeatureB  = match.matches[j].indexFeatureB;
			if (indexFeatureA >= nbFeatureA)
			{
				std::cout << "Error: " << indexFeatureA << ">" <<nbFeatureA << std::endl;
			}
			if (indexFeatureB >= nbFeatureB)
			{
				std::cout << "Error: " << indexFeatureB << ">" <<nbFeatureB << std::endl;
			}
		}
	}
}

void Manager::trackSanityCheck()
{
	for (unsigned int i=0; i<mTracks.size(); ++i)
	{
		const TrackInfo& trackInfo = mTracks[i];

		//Fill reverse map
		std::map<unsigned int, std::vector<unsigned int>> reverseMap;
		for (unsigned int j=0; j<trackInfo.viewpoints.size(); ++j)
		{
			unsigned int pictureIndex = trackInfo.viewpoints[j].first;
			unsigned int featureIndex = trackInfo.viewpoints[j].second;

			if (reverseMap.find(pictureIndex) == reverseMap.end())
				reverseMap[pictureIndex] = std::vector<unsigned int>();
			reverseMap[pictureIndex].push_back(featureIndex);
		}

		//Fill only unique matches (pruning)
		for (std::map<unsigned int, std::vector<unsigned int>>::const_iterator it = reverseMap.begin(); it != reverseMap.end(); ++it)
		{
			if (it->second.size() != 1)
			{
				std::cout << "The track[" << i << "] contains " << it->second.size() << " features of picture[" << it->first << "]" << std::endl;
			}
		}
	}
}

void Manager::loadPhotoSynthCameraParameters(const std::string& filename)
{
	std::ifstream input(filename.c_str());
	if (input.is_open())
	{
		unsigned int nbCameras;
		input >> nbCameras;
		if (nbCameras == mPictures.size())
		{
			for (unsigned int i=0; i<nbCameras; ++i)
			{
				double f, k1, k2;
				double rot[9];
				double t[3];

				input >> f      >> k1     >> k2;
				input >> rot[0] >> rot[1] >> rot[2];
				input >> rot[3] >> rot[4] >> rot[5];
				input >> rot[6] >> rot[7] >> rot[8];
				input >> t[0]   >> t[1]   >> t[2];

				memcpy(&mPictures[i].photosynth_rotation[0], &rot[0], sizeof(double)*9);
				memcpy(&mPictures[i].photosynth_translation[0], &t[0], sizeof(double)*3);

				mPictures[i].photosynth_focal    = f;
				mPictures[i].photosynth_distort1 = k1;
				mPictures[i].photosynth_distort2 = k2;
				mPictures[i].photosynth_isValid  = true;
			}
		}
		else
		{
			std::cerr << "Can't load your PhotoSynth files..." << nbCameras << " pictures in your file and " << mPictures.size() << " in distort folder" << std::endl;
		}
	}
	input.close();
}

void Manager::loadPhotoSynthJson(PhotoSynth::Parser* parser)
{
	for (unsigned int i=0; i<parser->getNbCamera(0); ++i)
	{
		const PhotoSynth::Camera& cam = parser->getCamera(0, i);

		PictureInfo& info = mPictures[cam.index];
		info.photosynth_isValid = true;

		Ogre::Matrix3 rot;
		cam.orientation.ToRotationMatrix(rot);

		info.photosynth_rotation[0] = rot[0][0]; info.photosynth_rotation[1] = rot[0][1]; info.photosynth_rotation[2] = rot[0][2];
		info.photosynth_rotation[3] = rot[1][0]; info.photosynth_rotation[4] = rot[1][1]; info.photosynth_rotation[5] = rot[1][2];
		info.photosynth_rotation[6] = rot[2][0]; info.photosynth_rotation[7] = rot[2][1]; info.photosynth_rotation[8] = rot[2][2];

		info.photosynth_translation[0] = cam.position.x;
		info.photosynth_translation[1] = cam.position.y;
		info.photosynth_translation[2] = cam.position.z;

		info.photosynth_focal    = cam.focal;
		info.photosynth_distort1 = cam.distort1;
		info.photosynth_distort2 = cam.distort2;
	}
}