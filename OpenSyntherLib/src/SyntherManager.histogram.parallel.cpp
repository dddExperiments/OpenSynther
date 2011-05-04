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
#include "SyntherLog.h"
#include "SyntherSiftGPUExtractor.h"
#include "SyntherSiftGPUMatcher.h"

namespace bf = boost::filesystem;

using namespace Synther;

inline bool FeatureSortFunction(const Feature& a, const Feature& b)
{
	return a.scale > b.scale;
}

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
	matchingThreshold = 0.9f;
	sequenceMatchingLength = 8;
}

Manager::Manager(ManagerOption& options)
: mCCDDatabaseManager(options.camerasCCDFilepath)
{
	mOptions = options;
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

bool Manager::open(const std::string& inputPath)
{
	std::cout<< "[Extracting features...]";
	clock_t startFeatureExtraction = clock();
	extractFeatures(getPictureList(inputPath));
	clearScreen();
	std::cout<< "[Features extracted in " << floor(float(clock() - startFeatureExtraction) / CLOCKS_PER_SEC)  << "s]" << std::endl;

	if (mPictures.size() < 3)
	{
		Synther::Log::write(Synther::Log::Error, "You need at least 3 pictures");
		return false;
	}
	if (mOptions.verbose)
		exportVector();

	std::cout<< "[Matching features...]";
	clock_t startFeatureMatching = clock();
	matchFeatures();
	clearScreen();
	std::cout<< "[Features matched in " << floor(float(clock() - startFeatureMatching) / CLOCKS_PER_SEC)  << "s]" << std::endl;

	if (mOptions.verbose)
		exportMatrix("matrix.txt");

	std::cout<< "[Pruning matches...]";
	clock_t startMatchPruning = clock();
	pruneMatches();
	clearScreen();
	std::cout<< "[Matches pruned in " << floor(float(clock() - startMatchPruning) / CLOCKS_PER_SEC)  << "s]" << std::endl;

	if (mOptions.verbose)
		exportMatrix("matrix.pruned.txt");

	std::cout<< "[Estimating Fundamental matrices...]";
	clock_t startFundamentalMatrixEstimation = clock();
	estimateFundamentalMatrix();
	clearScreen();
	std::cout<< "[Fundamental matrices estimated in " << floor(float(clock() - startMatchPruning) / CLOCKS_PER_SEC)  << "s]" << std::endl;

	if (mOptions.verbose)
		exportMatrix("matrix.ransac.txt");

	std::cout << "[Building Tracks]";
	clock_t startTrackBuilding = clock();
	buildTracks();
	clearScreen();
	std::cout<< "[Tracks built in " << floor(float(clock() - startTrackBuilding) / CLOCKS_PER_SEC)  << "s]" << std::endl;

	return true;
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

		//Resize jpeg is one size is higher than the max dimension allowed (keeping aspect ratio)
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
	unsigned int nbFeatureUsedForLinearMatching = 300;
	unsigned int nbPicture     = (unsigned int) mPictures.size();
	unsigned int descLength    = Feature::descriptorLength;
	mDescriptorDatabase        = new float[nbPicture*nbFeatureUsedForLinearMatching*descLength];

	//Fill descriptor database with 300 biggest (scale) features of each picture
	for (unsigned int i=0; i<nbPicture; ++i)
	{
		PictureInfo& info = mPictures[i];
		std::vector<Feature> features = info.features;
		info.descriptors;
		std::partial_sort(features.begin(), features.begin()+nbFeatureUsedForLinearMatching, features.end(), FeatureSortFunction);

		for (unsigned int j=0; j<nbFeatureUsedForLinearMatching; ++j)
		{
			float* dst = mDescriptorDatabase + i*nbFeatureUsedForLinearMatching*descLength + j*descLength;			
			float* src = &mPictures[i].descriptors[features[j].index];
			memcpy(dst, src, descLength*sizeof(float));
		}
	}

	//Build Flann index for fast ANN
	FLANNParameters params;
	params.algorithm        = KMEANS;
	params.log_level        = LOG_INFO;
	params.memory_weight    = 0.001f;
	params.build_weight     = 0.01f;
	params.target_precision = -1;
	params.branching        = 32;
	params.centers_init     = CENTERS_RANDOM;
	params.iterations       = 5;
	params.checks           = 172;
	params.cb_index         = (float)1.04522e+009;
	float speedup = 0;

	mDescriptorDatabaseIndex = flann_build_index(mDescriptorDatabase, nbPicture*nbFeatureUsedForLinearMatching, descLength, &speedup, &params);
	
	//Allocating one histogram per core
	int nbCoreAvailable = boost::thread::hardware_concurrency();
	unsigned int histogramSize = nbPicture*nbPicture;

	std::vector<std::vector<unsigned int>> histograms(nbCoreAvailable);
	for (int i=0; i<nbCoreAvailable; ++i)
	{		
		histograms[i] = std::vector<unsigned int>(histogramSize);
		memset(&histograms[i][0], 0, sizeof(unsigned int)*histogramSize);
	}
	
	//Filling the histogram in parallel
	boost::thread_group threadGroup;
	int nbPicturePerCore = (int) floor(nbPicture / (float)nbCoreAvailable);
	int startOffset = 0;
	int remainingPicture = nbPicture - nbPicturePerCore*nbCoreAvailable;
	for (int i=0; i<nbCoreAvailable; ++i)
	{		
		int startIndex = startOffset;
		int endIndex = startIndex + nbPicturePerCore - 1;
		if (remainingPicture > 0)
		{
			remainingPicture--;
			endIndex++;
		}
		startOffset = endIndex + 1;
		if (startIndex <= endIndex)
		{
			//std::cout << "core[" << i << "] " << startIndex << " to " << endIndex << " -> " << (endIndex-startIndex+1) << std::endl;
			threadGroup.create_thread(boost::bind(&Manager::fillHistogramForLinearMatching, this, startIndex, endIndex, histograms[i]));
		}
	}
	threadGroup.join_all();

	//Merging the histogram
}

void Manager::fillHistogramForLinearMatching(int startPicIndex, int endPicIndex, std::vector<unsigned int>& histogram)
{
	int nbPicture = (int) mPictures.size();

	//fill histogram
	for (int i=startPicIndex; i<=endPicIndex; ++i)
	{
		//std::cout << "thread[" << boost::this_thread::get_id() << "] working on pic " << i << std::endl;
		
		PictureInfo& pictureInfo = mPictures[i];
		unsigned int nbFeature   = 300;

		//Flann index building
		FLANNParameters params;
		params.algorithm        = KMEANS;
		params.log_level        = LOG_INFO;
		params.memory_weight    = 0.01f;
		params.build_weight     = 0.01f;
		params.target_precision = -1;
		params.branching        = 32;
		params.centers_init     = CENTERS_RANDOM;
		params.iterations       = 5;
		params.checks           = 172;
		params.cb_index         = (float)1.04522e+009;

		//Flann nearest neighbor search
		unsigned int nn = 6;
		int* indices    = new int[nbFeature*nn];
		float* dists    = new float[nbFeature*nn];

		flann_find_nearest_neighbors_index(mDescriptorDatabaseIndex, mDescriptorDatabase + i*nbFeature*Feature::descriptorLength, nbFeature, indices, dists, nn, &params);

		for (unsigned int j=0; j<nbFeature; ++j)
		{
			for (unsigned int k=0; k<nn; ++k)
			{
				int pictureIndex = (int) floor(indices[j*nn+k] / (float)nbFeature);
				/*int pictureIndex = getImageIndex();
				mMatchingHistogram[i][pictureIndex].counter++;
				mMatchingHistogram[pictureIndex][i].counter++;*/
			}
		}

		delete[] indices;
		delete[] dists;
	}

	//empty the diagonal
	/*for (int i=0; i<nbPicture; ++i)
		mMatchingHistogram[i][i].counter = 0;*/

	//fill 8 closest neighbor list
	unsigned int nbClosestNeighbor = 8;
	for (int i=0; i<nbPicture; ++i)
	{
		/*std::partial_sort(mMatchingHistogram[i].begin(), mMatchingHistogram[i].begin()+nbClosestNeighbor, mMatchingHistogram[i].end(), MatchingHistogramInfoSortFunction);

		PictureInfo& pictureInfo = mPictures[i];
		for (unsigned int j=0; j<nbClosestNeighbor; ++j)
			pictureInfo.linearMatchingNeighbors.push_back(mMatchingHistogram[i][j].pictureIndex);*/
	}
}

void Manager::createUnstructuredQuadraticMatchList()
{
	unsigned int nbPicture = (unsigned int) mPictures.size();
	for (unsigned int i=0; i<nbPicture-1; ++i)
		for (unsigned int j=i+1; j<nbPicture; ++j)
			mMatches.push_back(MatchInfo(i, j));
}

void Manager::createSequenceLinearMatchList(bool isClosed)
{
	int nbPicture = (int) mPictures.size();

	if (isClosed) //closed loop (N*m) -> O(N) complexity (N: nbPicture and M: sequenceMatchingLength)
	{
		for (int i=0; i<nbPicture; ++i)
		{
			for (int j=1; j<=mOptions.sequenceMatchingLength; ++j)
			{
				int indexA = i;
				int indexB = i+j;

				if (indexB >= nbPicture)
				{
					mMatches.push_back(MatchInfo(indexB-nbPicture, indexA));
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
		for (int i=0; i<nbPicture-1; ++i)
		{
			for (int j=1; j<=mOptions.sequenceMatchingLength; ++j)
			{
				int indexA = i;
				int indexB = i+j;

				if (indexB >= nbPicture)
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
		matcher.setDescriptors(0, mPictures[indexPictureA].descriptors);
		matcher.setDescriptors(1, mPictures[indexPictureB].descriptors);
		
		SiftGPUMatchResult matchResult = matcher.match();
		unsigned int nbMatch = matchResult.nbMatch;
		info.matches.resize(nbMatch);

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
	unsigned int nbMatch = (unsigned int) info->matches.size();

	CvMat* pointsMat1 = cvCreateMat(1, nbMatch, CV_32FC2);
	CvMat* pointsMat2 = cvCreateMat(1, nbMatch, CV_32FC2);
	CvMat* status     = cvCreateMat(1, nbMatch, CV_8UC1);
	CvMat* fundMatrix = cvCreateMat(3, 3, CV_32FC1);

	const std::vector<Feature> featuresA = mPictures[info->indexPictureA].features;
	const std::vector<Feature> featuresB = mPictures[info->indexPictureB].features;

	//fill list of 2D point for Ransac fundamental matrix estimation
	for (unsigned int i=0; i<nbMatch; ++i)
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
	for (unsigned int i=0; i<nbMatch; ++i)
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

bool CompareFirst(const FeatureMatch& m1, const FeatureMatch& m2)
{
	return m1.indexFeatureA < m2.indexFeatureA;
}

void Manager::buildTracks()
{
	fillFastMatchAccessMap();

	unsigned int nbPicture = (unsigned int) mPictures.size();

	//clear all marks for new images
	#pragma omp parallel for
	for (unsigned int i=0; i<nbPicture; ++i)
	{
		createNeighborList(i);
		int numNeighbors = mPictures[i].getNumNeighbors();

		//if this image has no neighbors don't worry about its keys
		if (numNeighbors != 0)
		{
			int num_features = mPictures[i].getNbFeature();
			mPictures[i].featuresFlags.resize(num_features);
		}
	}

	
	//sort all match lists
	#pragma omp parallel for
	for (unsigned int i=0; i<mMatches.size(); ++i)
	{
		std::vector<FeatureMatch>& list = mMatches[i].matches;
		std::sort(list.begin(), list.end(), CompareFirst);
	}

	std::vector<int> touched(nbPicture);
	std::vector<bool> img_marked(nbPicture);

	for (unsigned int i=0; i<nbPicture; ++i)
		img_marked[i] = false;

	for (unsigned int i=0; i<nbPicture; ++i)
	{
		int num_features = mPictures[i].getNbFeature();
		int num_nbrs     = mPictures[i].getNumNeighbors();

		if (num_nbrs == 0)
			continue;

		for (unsigned int j=0; j<(unsigned int)num_features; ++j)
		{
			std::vector<ImageKey> features;
			std::queue<ImageKey> features_queue;

			//check if this feature was already visited
			if (mPictures[i].featuresFlags[j])
				continue;

			//reset flags
			unsigned int num_touched = (unsigned int) touched.size();
			for (unsigned  int k=0; k<num_touched; ++k)
				img_marked[touched[k]] = false;
			touched.clear();

			//do a breadth first search given this feature
			mPictures[i].featuresFlags[j] = true;

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

				const std::vector<unsigned int>& neighbors = mPictures[img1].neighbors; //getNeighbors(img1);
				for (unsigned int l=0; l<neighbors.size(); ++l)
				{
					unsigned int k = neighbors[l];

					if (img_marked[k])
						continue;

					unsigned int indexA = std::min((unsigned int)img1, k);
					unsigned int indexB = std::max((unsigned int)img1, k);

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

					assert(indexFeatureB < mPictures[k].getNbFeature());

					//check if we visited this point already
					if (mPictures[k].featuresFlags[indexFeatureB])
						continue;

					//mark and push the point
					mPictures[k].featuresFlags[indexFeatureB] = true;
					features.push_back(ImageKey(k, indexFeatureB));
					features_queue.push(ImageKey(k, indexFeatureB));

					img_marked[k] = true;
					touched.push_back(k);
				}
			} //while loop

			if (features.size() > 2)
			{
				mTracks.push_back(TrackInfo(features));
			} 
		} //loop over features
	} //loop over pictures
}
/*
void Manager::createNeighborList(unsigned int pictureIndex)
{
	std::map<unsigned int, unsigned int> neighborsMap;
	for (unsigned int i=0; i<mMatches.size(); ++i)
	{
		unsigned int indexPictureA = mMatches[i].indexPictureA;
		unsigned int indexPictureB = mMatches[i].indexPictureB;

		if (indexPictureA == pictureIndex)
			neighborsMap[indexPictureB] = 0;
		else if (indexPictureB == pictureIndex)
			neighborsMap[indexPictureA] = 0;
	}

	std::vector<unsigned int> neighbors; 
	for (std::map<unsigned int, unsigned int>::const_iterator it = neighborsMap.begin(); it != neighborsMap.end(); ++it)
		neighbors.push_back((*it).first);

	mPictures[pictureIndex].neighbors = neighbors;
}
*/
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

void Manager::exportVector()
{
	std::ofstream output("vector.txt");
	if (output.is_open())
	{
		for (unsigned int i=0; i<mPictures.size(); ++i)
			output << mPictures[i].getNbFeature() << ";" << mPictures[i].filepath << std::endl;
	}
	output.close();
}

void Manager::exportMatrix(const std::string& filename)
{
	unsigned int nbPicture = (unsigned int) mPictures.size();
	unsigned int* values = new unsigned int[nbPicture*nbPicture];
	memset(values, 0, sizeof(unsigned int)*nbPicture*nbPicture);

	//Filling matrix with matches values
	for (unsigned int i=0; i<mMatches.size(); ++i)
	{
		const MatchInfo info = mMatches[i];
		unsigned int nbMatch = (unsigned int) info.matches.size();
		values[info.indexPictureA*nbPicture+info.indexPictureB] = nbMatch;
	}

	//Writing matrix to file
	std::ofstream output(filename.c_str());
	if (output.is_open())
	{		
		for (unsigned int i=0; i<nbPicture; ++i)
		{
			for (unsigned int j=0; j<nbPicture; ++j)
			{
				output << values[i*nbPicture + j] << ";";
			}
			output << std::endl;
		}
	}
	output.close();

	delete[] values;
}

void Manager::clearScreen()
{
	std::cout << "\r                                                                          \r";
}