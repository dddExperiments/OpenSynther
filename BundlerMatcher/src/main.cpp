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
#include <ctime>

#include "SyntherManager.h"
#include "SyntherLinearMatcher.h"

int main(int argc, char*argv[])
{	
	if (argc < 1)
	{
		std::cout << "Usage: " << argv[0] << " <inputPath>" <<std::endl;
		std::cout << "<inputPath> : folder containing your jpegs" <<std::endl;
		/*
		options:
		-matching_mode: 0
			unstructured linear 0
			unstructured quadratic 1
			opened sequence 2
			closed sequence 3
		-first_octave : 0
		-nb_feature_max : 5000
		-max_picture_dimension : 2500
		-matching_threshold : 0.9
		-sequence_matching_length : 8
		-verbose : 0
		-linear_matching:
			-nb_feature_used_per_picture : 300
			-nb_nearest_neighbor_in_feature_space : 6
			-nb_closest_picture_used : 8

		*/
		return -1;
	}

	bool linearMatchingEnabled = false;
	bool verbose               = false;

	Synther::ManagerOption options;
	options.verbose             = verbose;
	options.matchingMode        = linearMatchingEnabled ? Synther::MATCHINGMODE_UNSTRUCTURED_LINEAR : Synther::MATCHINGMODE_UNSTRUCTURED_QUADRATIC;
	options.maxPictureDimension = 2500;
	options.matchingMode        = Synther::MATCHINGMODE_UNSTRUCTURED_LINEAR;

	Synther::LinearMatcherOption linearOptions;
	linearOptions.nbFeaturesUsedPerPicture         = 300;
	linearOptions.nbNearestNeighborsInFeatureSpace = 6;
	linearOptions.nbClosestPicturesUsed            = 8;

	for (int i=1; i<argc; ++i)
	{
		std::string current(argv[i]);
		if (current == "-linear")
			linearMatchingEnabled = true;
		else if (current == "-verbose")
			verbose = true;
		else if (current == "matching_mode")
		{
			if (i+1<argc)
			{				
				int matchingMode = atoi(argv[i+1]);
				if (matchingMode >= 0)
				{
					options.matchingMode = (Synther::MatchingMode)matchingMode;
				}
				i++;
			}
		}
	}

	clock_t start = clock();
	Synther::Manager manager(options, linearOptions);
	{
		std::cout<< "[Extracting features...]";
		clock_t startFeatureExtraction = clock();
		manager.extractFeatures(manager.getPictureList(argv[1]));
		manager.clearScreen();
		std::cout<< "[Features extracted in " << floor(float(clock() - startFeatureExtraction) / CLOCKS_PER_SEC)  << "s]" << std::endl;

		manager.exportVector("vector.txt");

		std::cout<< "[Saving features...]";
		clock_t startSavingFeature = clock();
		manager.saveAsciiDescriptors();
		manager.clearScreen();
		std::cout<< "[Features saved in " << floor(float(clock() - startSavingFeature) / CLOCKS_PER_SEC)  << "s]" << std::endl;

		std::cout<< "[Matching features...]";
		clock_t startFeatureMatching = clock();
		manager.matchFeatures();
		manager.clearScreen();
		std::cout<< "[Features matched in " << floor(float(clock() - startFeatureMatching) / CLOCKS_PER_SEC)  << "s]" << std::endl;

		manager.exportMatrix("matrix.txt");

		
		std::cout<< "[Pruning matches...]";
		clock_t startMatchPruning = clock();
		manager.pruneMatches();
		manager.clearScreen();
		std::cout<< "[Matches pruned in " << floor(float(clock() - startMatchPruning) / CLOCKS_PER_SEC)  << "s]" << std::endl;

		manager.exportMatrix("matrix.pruned.txt");
		/*
		std::cout<< "[Estimating Fundamental matrices...]";
		clock_t startFundamentalMatrixEstimation = clock();
		manager.estimateFundamentalMatrix();
		manager.clearScreen();
		std::cout<< "[Fundamental matrices estimated in " << floor(float(clock() - startMatchPruning) / CLOCKS_PER_SEC)  << "s]" << std::endl;
		

		manager.exportMatrix("matrix.ransac.txt");
		*/
		std::cout << "[Building Tracks]";
		clock_t startTrackBuilding = clock();
		manager.buildTracks();
		manager.clearScreen();
		std::cout<< "[Tracks built in " << floor(float(clock() - startTrackBuilding) / CLOCKS_PER_SEC)  << "s]" << std::endl;
		
		manager.exportMatchesForBundler("matches.txt");
	}

	std::cout << "Time elapsed: " << floor(float(clock() - start) / CLOCKS_PER_SEC) << "s" << std::endl;
	system("pause");

	return 0;
}