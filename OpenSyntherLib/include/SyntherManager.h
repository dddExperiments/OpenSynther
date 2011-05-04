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
#include <map>

#include <ExifReader.h>
#include "SyntherStructures.h"
#include "SyntherCCDDatabase.h"
#include "SyntherLinearMatcher.h"
#include <PhotoSynthParser.h>

#include <libmv/multiview/test_data_sets.h>

namespace Synther
{
	enum MatchingMode
	{
		MATCHINGMODE_UNSTRUCTURED_LINEAR,
		MATCHINGMODE_UNSTRUCTURED_QUADRATIC,
		MATCHINGMODE_OPENED_SEQUENCE_LINEAR,
		MATCHINGMODE_CLOSED_SEQUENCE_LINEAR
	};

	struct ManagerOption
	{
		ManagerOption();
		
		//General
		bool verbose;
		std::string camerasCCDFilepath;
		
		//Feature extraction
		int firstOctave;
		int nbFeatureMax;
		int maxPictureDimension;
		
		//Feature matching
		MatchingMode matchingMode;
		float matchingThreshold;
		int sequenceMatchingLength;
	};

	class Manager
	{
		public:			
			friend class LinearMatcher;
			friend class TracksBuilder;
			friend class TracksColorizer;

			Manager(ManagerOption& options, LinearMatcherOption& linearOptions);
			~Manager();

			std::vector<std::string> getPictureList(const std::string& inputPath);

			void extractFeatures(const std::vector<std::string>& filenames);
			void matchFeatures();
			void pruneMatches();
			void estimateFundamentalMatrix();
			int  buildTracks();
			void loadPhotoSynthCameraParameters(const std::string& filePath);
			void loadPhotoSynthJson(PhotoSynth::Parser* parser);			
			void triangulateTracks();
			void colorizeTracks();

			void debugTriangulation(const std::string& projectPath);

			//Export
			void exportVector(const std::string& filename);
			void exportMatrix(const std::string& filename);
			void exportMatchList(unsigned int i, unsigned int j, int (*matchBuffer)[2], int nbMatch);
			void exportMatchesForBundler(const std::string& filepath);
			void exportMatchesForGraphViz(const std::string& filepath, int threshold);			
			void exportDataForCudaRansac();
			void exportTracks(const std::string& filename);
			void exportTracksAsPointCloud(const std::string& filename);
			void exportTracksLengthHistogram(const std::string& filename);
			void exportBundlerOutput(const std::string& filename, bool undistorded = false);
			void exportVisibility(const std::string& filename, int threshold = 0);
			void exportDataForTracking(const std::string& projectPath);
			void saveAsciiDescriptors();

			static void clearScreen();

		protected:						

			//Feature matching
			void createUnstructuredLinearMatchList();
			void createUnstructuredQuadraticMatchList();
			void createSequenceLinearMatchList(bool isClosed = false);			
			void matchSanityCheck();

			//Matches pruning			
			void pruneMatch(MatchInfo* info);

			//Estimate Fundamental matrix			
			void estimateFundamentalMatrix(MatchInfo* info);
			
			//Focal Length
			float computeFocalLengthFromExif(Exif::Info& exifInfo, int width, int height);
			CCDDatabaseManager mCCDDatabaseManager;	

			//Track computation			
			void createNeighborList(unsigned int pictureIndex);
			void fillFastMatchAccessMap();
			void trackSanityCheck();
			libmv::Vec2 undistort(double f, double k1, double k2, libmv::Vec2 p);

			//Track triangulation
			void loadTracksForDebug(const std::string& filePath);
			void triangulateTracksOfLength2Bis();
			void triangulateTracksOfLength2();
			void triangulateTracksOfLengthN();

			ManagerOption mOptions;
			LinearMatcherOption mLinearOptions;

			std::vector<PictureInfo> mPictures;
			std::vector<MatchInfo>   mMatches;
			std::vector<TrackInfo>   mTracks;
			std::map<__int64, MatchInfo*> mMatchesMap;
	};
}