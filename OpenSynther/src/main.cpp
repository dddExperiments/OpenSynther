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
	options.matchingMode        = Synther::MATCHINGMODE_UNSTRUCTURED_QUADRATIC;

	Synther::LinearMatcherOption linearOptions;
	linearOptions.nbFeaturesUsedPerPicture         = 300;
	linearOptions.nbNearestNeighborsInFeatureSpace = 6;
	linearOptions.nbClosestPicturesUsed            = 15;

	clock_t start = clock();
	Synther::Manager manager(options, linearOptions);
	{
		std::string projectPath = std::string(argv[1]);	
		manager.debugTriangulation(projectPath);
	}

	std::cout << "Time elapsed: " << floor(float(clock() - start) / CLOCKS_PER_SEC) << "s" << std::endl;
	system("pause");

	return 0;
}