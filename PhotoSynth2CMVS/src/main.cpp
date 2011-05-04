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

#include <iostream>
#include <sstream>
#include <ctime>

#include <boost/filesystem/operations.hpp>
#include <boost/filesystem/convenience.hpp>

#include "SyntherManager.h"
#include "SyntherLinearMatcher.h"
#include <PhotoSynthParser.h>

void savePictureList(const std::vector<std::string>& pictureList, const std::string& projectPath);
bool isValidPath(const std::string& projectPath);
void initializeProject(const std::string& projectPath);

int main(int argc, char*argv[])
{	
	if (argc < 1)
	{
		std::cout << "Usage: " << argv[0] << " <inputPath>" <<std::endl;
		std::cout << "<inputPath> : PhotoSynth project folder" <<std::endl;

		return -1;
	}

	std::string projectPath = std::string(argv[1]);	

	if (!isValidPath(projectPath))
	{
		std::cout << "You need to use a valid folder created by PhotoSynthToolkit" << std::endl;
		std::cout << projectPath << "guid.txt doesn't exist" << std::endl;
		return -1;
	}

	initializeProject(projectPath);

	Synther::ManagerOption options;
	options.verbose             = true;
	options.matchingMode        = Synther::MATCHINGMODE_UNSTRUCTURED_QUADRATIC;
	options.maxPictureDimension = 2500;
	options.matchingThreshold   = 0.3f;
	
	Synther::LinearMatcherOption linearOptions;
	linearOptions.nbFeaturesUsedPerPicture         = 300;
	linearOptions.nbNearestNeighborsInFeatureSpace = 6;
	linearOptions.nbClosestPicturesUsed            = 8;
	
	clock_t start = clock();
	Synther::Manager manager(options, linearOptions);
	{
		std::stringstream path;
		path << projectPath << "distort";
		const std::vector<std::string>& pictureList = manager.getPictureList(path.str());
		savePictureList(pictureList, projectPath);

		std::cout<< "[Extracting features...]";
		clock_t startFeatureExtraction = clock();
		manager.extractFeatures(pictureList);
		manager.clearScreen();
		std::cout<< "[Features extracted in " << floor(float(clock() - startFeatureExtraction) / CLOCKS_PER_SEC)  << "s]" << std::endl;

		path.str("");
		path << projectPath << "bundler_tmp\\vector.txt";
		manager.exportVector(path.str());

		std::cout<< "[Matching features...]";
		clock_t startFeatureMatching = clock();
		manager.matchFeatures();
		manager.clearScreen();
		std::cout<< "[Features matched in " << floor(float(clock() - startFeatureMatching) / CLOCKS_PER_SEC)  << "s]" << std::endl;

		path.str("");
		path << projectPath << "bundler_tmp\\matrix.txt";
		manager.exportMatrix(path.str());

		std::cout<< "[Pruning matches...]";
		clock_t startMatchPruning = clock();
		manager.pruneMatches();
		manager.clearScreen();
		std::cout<< "[Matches pruned in " << floor(float(clock() - startMatchPruning) / CLOCKS_PER_SEC)  << "s]" << std::endl;

		path.str("");
		path << projectPath << "bundler_tmp\\matrix.pruned.txt";	
		manager.exportMatrix(path.str());

		std::cout<< "[Estimating Fundamental matrices...]";
		clock_t startFundamentalMatrixEstimation = clock();
		manager.estimateFundamentalMatrix();
		manager.clearScreen();
		std::cout<< "[Fundamental matrices estimated in " << floor(float(clock() - startMatchPruning) / CLOCKS_PER_SEC)  << "s]" << std::endl;
		
		path.str("");
		path << projectPath << "bundler_tmp\\matrix.ransac.txt";		
		manager.exportMatrix(path.str());

		std::cout << "[Building Tracks]";
		clock_t startTrackBuilding = clock();
		int nbTracks = manager.buildTracks();
		manager.clearScreen();
		std::cout<< "[" << nbTracks << " tracks built in " << floor(float(clock() - startTrackBuilding) / CLOCKS_PER_SEC)  << "s]" << std::endl;
				
		PhotoSynth::Parser parser;
		std::string guid = parser.getGuid(PhotoSynth::Parser::createFilePath(argv[1], PhotoSynth::Parser::guidFilename));
		parser.parseSoap(PhotoSynth::Parser::createFilePath(argv[1], PhotoSynth::Parser::soapFilename));
		parser.parseJson(PhotoSynth::Parser::createFilePath(argv[1], PhotoSynth::Parser::jsonFilename), guid);
		manager.loadPhotoSynthJson(&parser);

		std::cout<< "[Triangulating tracks...]";
		clock_t startTriangulatingTracks = clock();
		manager.triangulateTracks();
		manager.clearScreen();
		std::cout<< "[Tracks triangulated in " << floor(float(clock() - startTriangulatingTracks) / CLOCKS_PER_SEC)  << "s]" << std::endl;

		std::cout<< "[Colorizing tracks...]";
		clock_t startColorizingTracks = clock();
		manager.colorizeTracks();
		manager.clearScreen();
		std::cout<< "[Tracks colorized in " << floor(float(clock() - startColorizingTracks) / CLOCKS_PER_SEC)  << "s]" << std::endl;

		path.str("");
		path << projectPath << "bundler_tmp\\tracks.histogram.txt";	
		manager.exportTracksLengthHistogram(path.str());

		path.str("");
		path << projectPath << "bundler_tmp\\pointCloud.ply";	
		manager.exportTracksAsPointCloud(path.str());

		path.str("");
		path << projectPath << "bundler_tmp\\tracks.xml";	
		manager.exportTracks(path.str());

		path.str("");
		path << projectPath << "bundler_output\\bundle.out";	
		manager.exportBundlerOutput(path.str());

		path.str("");
		path << projectPath << "pmvs\\bundle.rd.out";
		manager.exportBundlerOutput(path.str(), true);

		path << projectPath << "pmvs\\vis.dat";
		manager.exportVisibility(path.str());

		manager.exportDataForTracking(projectPath);
	}

	std::cout << "Time elapsed: " << floor(float(clock() - start) / CLOCKS_PER_SEC) << "s" << std::endl;

	return 0;
}

void createDirectory(const std::string& path)
{
	if (!boost::filesystem::exists(path))
		boost::filesystem::create_directory(path);
}

void initializeProject(const std::string& projectPath)
{
	std::stringstream path;

	path << projectPath << "bundler_output";
	createDirectory(path.str());

	path.str("");
	path << projectPath << "bundler_tmp";
	createDirectory(path.str());

	path.str("");
	path << projectPath << "pmvs";
	createDirectory(path.str());
}

void savePictureList(const std::vector<std::string>& pictureList, const std::string& projectPath)
{	
	std::stringstream filepath;
	filepath << projectPath << "bundler_output\\list_absolute.txt";
	
	std::ofstream output(filepath.str().c_str());
	if (output.is_open())
	{
		for (unsigned int i=0; i<pictureList.size(); ++i)
			output << pictureList[i] << std::endl;
		output << std::endl;
	}
	output.close();
}

bool isValidPath(const std::string& projectPath)
{
	std::stringstream filepath;
	filepath << projectPath << "guid.txt";

	return boost::filesystem::exists(filepath.str());
}