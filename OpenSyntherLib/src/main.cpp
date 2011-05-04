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
	linearOptions.nbFeatureUsedPerPicture         = 300;
	linearOptions.nbNearestNeighborInFeatureSpace = 6;
	linearOptions.nbClosestPictureUsed            = 15;

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
	if (manager.open(argv[1]))
	{

	}
	std::cout << "Time elapsed: " << floor(float(clock() - start) / CLOCKS_PER_SEC) << "s" << std::endl;
	system("pause");

	return 0;
}