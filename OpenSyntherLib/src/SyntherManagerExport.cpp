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

#include "SyntherManager.h"

#include <fstream>
#include <sstream>
#include <iostream>

#include <OgreVector3.h>
#include <OgreMatrix3.h>

#include "libmv/multiview/test_data_sets.h"

using namespace Synther;

void Manager::exportVector(const std::string& filename)
{
	std::ofstream output(filename.c_str());
	if (output.is_open())
	{
		for (unsigned int i=0; i<mPictures.size(); ++i)
			output << mPictures[i].getNbFeature() << ";" << mPictures[i].filepath << std::endl;
	}
	output.close();
}

void Manager::exportMatrix(const std::string& filename)
{
	unsigned int nbPictures = (unsigned int) mPictures.size();
	unsigned int* values = new unsigned int[nbPictures*nbPictures];
	memset(values, 0, sizeof(unsigned int)*nbPictures*nbPictures);

	//Filling matrix with matches values
	for (unsigned int i=0; i<mMatches.size(); ++i)
	{
		const MatchInfo info = mMatches[i];
		unsigned int nbMatch = (unsigned int) info.matches.size();
		values[info.indexPictureA*nbPictures+info.indexPictureB] = nbMatch;
	}

	//Writing matrix to file
	std::ofstream output(filename.c_str());
	if (output.is_open())
	{		
		for (unsigned int i=0; i<nbPictures; ++i)
		{
			for (unsigned int j=0; j<nbPictures; ++j)
			{
				output << values[i*nbPictures + j] << ";";
			}
			output << std::endl;
		}
	}
	output.close();

	delete[] values;
}

void Manager::exportMatchList(unsigned int i, unsigned int j, int (*matchBuffer)[2], int nbMatch)
{	
	std::stringstream filename;
	filename << "match_" << i << "_" << j << ".txt";

	std::ofstream output(filename.str().c_str());
	output << nbMatch << std::endl;
	for (int k=0; k<nbMatch; ++k)
		output << matchBuffer[k][0] << " " << matchBuffer[k][1] << std::endl;
	output.close();
}

void Manager::exportMatchesForBundler(const std::string& filepath)
{
	std::ofstream output(filepath.c_str());
	if (output.is_open())
	{
		for (std::map<__int64, MatchInfo*>::const_iterator it = mMatchesMap.begin(); it != mMatchesMap.end(); ++it)
		{
			MatchIndex index((*it).first);
			const std::vector<FeatureMatch>& matches = (*it).second->matches;
			output << index.indexPictureA << " " << index.indexPictureB << std::endl;
			output << matches.size() << std::endl;
			for (unsigned int i=0; i<matches.size(); ++i)
				output << matches[i].indexFeatureA << " " << matches[i].indexFeatureB << std::endl;
		}
	}
	output.close();
}

/*
- intall GraphViz (http://www.graphviz.org/Download_windows.php)
- then run: neato -Tps matches.dot -o matches.ps
*/
void Manager::exportMatchesForGraphViz(const std::string& filepath, int threshold)
{	
	std::ofstream output(filepath.c_str());
	if (output.is_open())
	{
		output << "graph matches {" << std::endl;
		for (std::map<__int64, MatchInfo*>::const_iterator it = mMatchesMap.begin(); it != mMatchesMap.end(); ++it)
		{
			MatchIndex index((*it).first);
			const std::vector<FeatureMatch>& matches = (*it).second->matches;
			if ((int)matches.size() > threshold)
				output << "\t" << index.indexPictureA << " -- " << index.indexPictureB << ";" <<std::endl;
		}
		output << "}" << std::endl;
	}
	output.close();
}

void Manager::saveAsciiDescriptors()
{
	for (unsigned int i=0; i<mPictures.size(); ++i)
	{
		std::stringstream filename;
		filename << mPictures[i].filepath.substr(0,mPictures[i].filepath.size()-4) << ".key";

		unsigned int nbFeatures = mPictures[i].getNbFeature();
		const std::vector<Feature>& features = mPictures[i].features;

		float scale = mPictures[i].scale;
		std::ofstream output(filename.str().c_str());
		if (output.is_open())
		{
			output << nbFeatures << " 128" << std::endl;
			for (unsigned int i=0; i<nbFeatures; ++i)
			{
				//y x scale orientation
				output << features[i].y*scale << " " << features[i].x*scale << " " << features[i].scale << " " << features[i].orientation << std::endl;
				for (int j=0; j<128; ++j) 
				{
					output << "0 ";
					if ((j+1)%20 == 0) 
						output << std::endl;
				}
				output << std::endl;
			}
		}
		output.close();
	}
}

/*
	I should apply PictureInfo.scale to feature coordinate
*/
void Manager::exportDataForCudaRansac()
{
	std::ofstream outputList("CudaRansacList.txt");
	for (unsigned int i=0; i<mMatches.size(); ++i)
	{
		const MatchInfo& info = mMatches[i];
		const std::vector<FeatureMatch>& matches = info.matches;

		if (matches.size() > 12)
		{
			const std::vector<Feature>& featuresA = mPictures[info.indexPictureA].features;
			const std::vector<Feature>& featuresB = mPictures[info.indexPictureB].features;

			std::stringstream filename;
			filename << "cameras_" << info.indexPictureA << "_" << info.indexPictureB << ".txt";
			outputList <<  filename.str() << std::endl;

			std::ofstream output(filename.str().c_str());
			for (unsigned int k=0; k<matches.size(); ++k)
			{
				unsigned int indexFeatureA = matches[k].indexFeatureA;
				unsigned int indexFeatureB = matches[k].indexFeatureB;

				assert(indexFeatureA < featuresA.size());
				assert(indexFeatureB < featuresB.size());

				output << featuresA[indexFeatureA].x << " " << featuresA[indexFeatureA].y << " " << featuresB[indexFeatureB].x << " " << featuresB[indexFeatureB].y << std::endl;
			}
			output.close();
		}
	}
	outputList.close();
}

void Manager::exportTracks(const std::string& filename)
{
	std::ofstream output(filename.c_str());
	if (output.is_open())
	{
		output << "<tracks>" << std::endl;
		for (unsigned int i=0; i<mTracks.size(); ++i)
		{
			const std::vector<ImageKey>& viewpoints = mTracks[i].viewpoints;
			output << "	<track>" << std::endl;
			for (unsigned int j=0; j<viewpoints.size(); ++j)
			{
				const ImageKey& k = viewpoints[j];
				const PictureInfo& pictureInfo = mPictures[k.first];
				const Feature& f = pictureInfo.features[k.second];
				float scale = pictureInfo.scale;				

				std::stringstream paddedIndex; //contain %8d of i (42 -> 00000042)
				paddedIndex.width(8);
				paddedIndex.fill('0');
				paddedIndex << k.first;

				//output << "		<v x=\"" << f.x*scale << "\" y=\"" << f.y*scale << "\" s=\"" << f.scale << "\" o=\"" << f.orientation << "\" img=\"" << paddedIndex.str() << ".jpg\" />" << std::endl;
				output << "		<v x=\"" << f.x*scale << "\" y=\"" << f.y*scale << "\" s=\"" << f.scale << "\" o=\"" << f.orientation << "\" img=\"" << k.first << "\" />" << std::endl;
			}
			output << "	</track>" << std::endl;			
		}
		output << "</tracks>" << std::endl;
	}
	output.close();
}

void Manager::exportTracksAsPointCloud(const std::string& filepath)
{
	unsigned int nbPoints = 0;
	for (unsigned int i=0; i<mTracks.size(); ++i)
	{
		if (mTracks[i].isValid)
			nbPoints++;
	}

	std::ofstream output(filepath.c_str());
	if (output.is_open())
	{
		output << "ply" << std::endl;
		output << "format ascii 1.0" << std::endl;
		output << "element vertex " << (nbPoints + 2*mPictures.size()) << std::endl;
		output << "property float x" << std::endl;
		output << "property float y" << std::endl;
		output << "property float z" << std::endl;
		output << "property uchar red" << std::endl;
		output << "property uchar green" << std::endl;
		output << "property uchar blue" << std::endl;
		output << "element face 0" << std::endl;
		output << "property list uchar int vertex_indices" << std::endl;
		output << "end_header" << std::endl;

		for (unsigned int i=0; i<mPictures.size(); ++i)
		{
			const PictureInfo& info = mPictures[i];
			if (info.photosynth_isValid)
			{
				const double* pos = &(info.photosynth_translation)[0];

				if ((i % 2) == 0)
					output << pos[0] << " " << pos[1] << " " << pos[2] << " 0 255 0" << std::endl;
				else
					output << pos[0] << " " << pos[1] << " " << pos[2] << " 255 0 0" << std::endl;

				Ogre::Vector3 position;
				position.x = (float)pos[0]; position.y = (float)pos[1]; position.z = (float)pos[2];
				Ogre::Vector3 offset(0.0f, 0.0f, -0.05f);
				Ogre::Matrix3 ori;

				ori[0][0] = (float)info.photosynth_rotation[0]; ori[0][1] = (float)info.photosynth_rotation[1]; ori[0][2] = (float)info.photosynth_rotation[2];
				ori[1][0] = (float)info.photosynth_rotation[3]; ori[1][1] = (float)info.photosynth_rotation[4]; ori[1][2] = (float)info.photosynth_rotation[5];
				ori[2][0] = (float)info.photosynth_rotation[6]; ori[2][1] = (float)info.photosynth_rotation[7]; ori[2][2] = (float)info.photosynth_rotation[8];

				Ogre::Vector3 p = position + ori.Inverse() * offset;
				output << p.x << " " << p.y << " " << p.z << " 255 255 0" << std::endl;
			}
			else
			{
				output << "0 0 0 0 0 0" << std::endl;
				output << "0 0 0 0 0 0" << std::endl;
			}
		}

		for (unsigned int i=0; i<mTracks.size(); ++i)
		{
			if (mTracks[i].isValid)
			{
				const TrackInfo& t = mTracks[i];
				output << t.x << " " << t.y << " " << t.z << " " << (unsigned int) t.color[0] << " " << (unsigned int) t.color[1] << " " << (unsigned int) t.color[2] << std::endl;
			}
		}
	}
	output.close();
}

void Manager::exportBundlerOutput(const std::string& filename, bool undistorded)
{
	libmv::Mat3 Rx, Rxt;
	Rx << 1.0,  0.0,  0.0,
		0.0, -1.0,  0.0, 
		0.0,  0.0, -1.0;
	Rxt = Rx.transpose();

	std::ofstream output(filename.c_str());
     if (output.is_open())
     {
         unsigned int nbVertices = 0;
         unsigned int nbCameras = (unsigned int) mPictures.size();
         for (unsigned int i=0; i<mTracks.size(); ++i)
         {
             if (mTracks[i].isValid)
                 nbVertices++;
         }

         output << "# Bundle file v0.3" << std::endl;
         output << nbCameras << " " << nbVertices << std::endl;
         for (unsigned int i=0; i<nbCameras; ++i)
         {
             const PictureInfo& info = mPictures[i];
			 if (info.photosynth_isValid)
			 {
				 double focal = std::max(info.width, info.height)*info.photosynth_focal;
				 libmv::Mat3 R = libmv::Mat3::Map(info.photosynth_rotation);
				 libmv::Vec3 t = libmv::Vec3::Map(info.photosynth_translation);
				 R.transposeInPlace(); //because of mapping
				 R = Rxt * R.transpose();
				 t = -R * t;
				 output << focal*info.scale << " " << info.photosynth_distort1 << " " << info.photosynth_distort2 << std::endl;
				 output << R(0,0) << " " << R(0,1) << " " << R(0,2) << std::endl;
				 output << R(1,0) << " " << R(1,1) << " " << R(1,2) << std::endl;
				 output << R(2,0) << " " << R(2,1) << " " << R(2,2) << std::endl;
				 output << t.x() << " " << t.y() << " " << t.z() << std::endl;
			 }
			 else
			 {
				 output << "0 0 0"<<std::endl; //f k1 k2
				 output << "0 0 0"<<std::endl; //R[0]
				 output << "0 0 0"<<std::endl; //R[1]
				 output << "0 0 0"<<std::endl; //R[2]
				 output << "0 0 0"<<std::endl; //t
			 }
         }
         for (unsigned int j=0; j<mTracks.size(); ++j)
         {
             const TrackInfo& info = mTracks[j];
             if (info.isValid)
             {
                 unsigned int nbViewpoints = (unsigned int) info.viewpoints.size();

                 output << info.x << " " << info.y << " " << info.z << std::endl;				 
                 output << (unsigned int) info.color[0] << " " << (unsigned int) info.color[1] << " " << (unsigned int) info.color[2] << std::endl;
                 output << nbViewpoints << " ";
                 for (unsigned int k=0; k<nbViewpoints; ++k)
                 {
                     unsigned int pictureIndex = info.viewpoints[k].first;
                     unsigned int featureIndex = info.viewpoints[k].second;
					 const PictureInfo& pictureInfo = mPictures[pictureIndex];
                     const Feature& feature = pictureInfo.features[featureIndex];
					 float scale = pictureInfo.scale;

					 assert(pictureIndex < mPictures.size());
					 assert(featureIndex < mPictures[pictureIndex].features.size());

                     float featureX = feature.x;
					 float featureY = feature.y;
					 
					 if (undistorded && pictureInfo.photosynth_isValid)
					 {						
						double k1 = pictureInfo.photosynth_distort1;
						double k2 = pictureInfo.photosynth_distort2;
						double w = pictureInfo.width;
						double h = pictureInfo.height;
						double focal = std::max(w, h)*pictureInfo.photosynth_focal;

						double f2Inverse = 1.0 / (focal * focal);
						double xC = featureX - 0.5*w;
						double yC = featureY - 0.5*h;
						double r2 = (xC*xC + yC*yC) * f2Inverse;
						double factor = 1.0 + k1*r2 + k2*r2*r2;
						xC *= factor;
						yC *= factor;
						featureX = (float) (xC + 0.5*w);
						featureY = (float) (yC + 0.5*h);
					 }
					 output << pictureIndex << " " << featureIndex << " " << featureX*scale << " " << featureY*scale << " ";
                 }
                 output << std::endl;
             }
         }
     }
     output.close();
}

void Manager::exportTracksLengthHistogram(const std::string& filename)
{
	//look for max length
	unsigned int maxLength = 0;
	for (unsigned int i=0; i<mTracks.size(); ++i)
	{
		unsigned int currentLenght = (unsigned int) mTracks[i].viewpoints.size();
		if (currentLenght > maxLength)
			maxLength = currentLenght;
	}

	//allocating histogram
	std::vector<int> histogram(maxLength+1);

	//clearing histogram
	for (unsigned int i=0; i<histogram.size(); ++i)
		histogram[i] = 0;

	//Filling histogram
	for (unsigned int i=0; i<mTracks.size(); ++i)
	{
		unsigned int currentLenght = (unsigned int) mTracks[i].viewpoints.size();
		histogram[currentLenght]++;
	}

	//writing histogram to file
	std::ofstream output(filename.c_str());
	if (output.is_open())
	{
		for (unsigned int i=0; i<histogram.size(); ++i)
			output << i << " " << histogram[i] << std::endl;
	}
	output.close();
}

void Manager::exportVisibility(const std::string& filename, int threshold)
{
	unsigned int nbPictures = (unsigned int) mPictures.size();

	//allocation of visibility matrix
	std::vector<std::vector<int>> visibilityMap = std::vector<std::vector<int>>(nbPictures);
	for (unsigned int i=0; i<nbPictures; ++i)
	{
		visibilityMap[i] = std::vector<int>(nbPictures);
		for (unsigned int j=0; j<nbPictures; ++j)
		{
			visibilityMap[i][j] = 0;
		}
	}

	//filling visibility matrix
	for (unsigned int i=0; i<mTracks.size(); ++i)
	{
		const TrackInfo& trackInfo = mTracks[i];
		unsigned int nbViews = (unsigned int) trackInfo.viewpoints.size();
		for (unsigned int j=0; j<nbViews; ++j)
		{
			unsigned int indexPictureA = trackInfo.viewpoints[j].first;
			for (unsigned int k=j+1; k<nbViews; ++k)
			{
				unsigned int indexPictureB = trackInfo.viewpoints[k].first;
				if (j != k)
				{
					visibilityMap[indexPictureA][indexPictureB]++;
					visibilityMap[indexPictureB][indexPictureA]++;
				}
			}
		}
	}

	//writing visibility matrix to file
	std::ofstream output(filename.c_str());
	if (output.is_open())
	{
		output << "VISDATA" << std::endl;
		output << nbPictures << std::endl;
		for (unsigned int i=0; i<nbPictures; ++i)
		{
			std::vector<int> pictureIndexes;
			for (unsigned int j=0; j<nbPictures; ++j)
			{
				if (visibilityMap[i][j] > threshold)
					pictureIndexes.push_back(j);
			}

			output << i << " " << pictureIndexes.size();
			for (unsigned int j=0; j<(unsigned int)pictureIndexes.size(); ++j)
			{
				output << " " << pictureIndexes[j];
			}
			output << std::endl;
		}
	}
	output.close();
}

void Manager::exportDataForTracking(const std::string& projectPath)
{
	libmv::Mat3 Rx, Rxt;
	Rx << 1.0,  0.0,  0.0,
		0.0, -1.0,  0.0, 
		0.0,  0.0, -1.0;
	Rxt = Rx.transpose();

	std::stringstream filepath;
	filepath << projectPath << "bundler_output\\tracking.txt";

	//Write txt file
	{
		std::ofstream output(filepath.str().c_str());
		if (output.is_open())
		{
			output << "#BundlerTracking version 1" << std::endl;
			unsigned int nbPictures = (unsigned int) mPictures.size();
			output << nbPictures << std::endl;
			for (unsigned int i=0; i<nbPictures; ++i)
			{
				PictureInfo& info = mPictures[i];
				output << "#" << info.filepath << std::endl;
				if (info.photosynth_isValid)
				{
					double focal = std::max(info.width, info.height)*info.photosynth_focal;
					libmv::Mat3 R = libmv::Mat3::Map(info.photosynth_rotation);
					libmv::Vec3 t = libmv::Vec3::Map(info.photosynth_translation);
					R.transposeInPlace(); //because of mapping
					R = Rxt * R.transpose();
					t = -R * t;
					output << focal*info.scale << " " << info.photosynth_distort1 << " " << info.photosynth_distort2 << std::endl;
					output << t.x() << " " << t.y() << " " << t.z() << std::endl;
					output << R(0,0) << " " << R(0,1) << " " << R(0,2) << std::endl;
					output << R(1,0) << " " << R(1,1) << " " << R(1,2) << std::endl;
					output << R(2,0) << " " << R(2,1) << " " << R(2,2) << std::endl;					
				}
				else
				{
					output << "0 0 0"<<std::endl; //f k1 k2
					output << "0 0 0"<<std::endl; //t
					output << "0 0 0"<<std::endl; //R[0]
					output << "0 0 0"<<std::endl; //R[1]
					output << "0 0 0"<<std::endl; //R[2]
					
				}
			}
		}
		output.close();
	}

	filepath.str("");
	filepath << projectPath << "bundler_output\\tracking.bin";

	//Write bin file
	{
		std::ofstream output(filepath.str().c_str(), std::ios::binary);
		if (output.is_open())
		{
			unsigned int nb3DPoints = 0;
			for (unsigned int i=0; i<mTracks.size(); ++i)
			{
				if (mTracks[i].isValid)
					nb3DPoints++;
			}
			output.write((char*)&nb3DPoints, sizeof(nb3DPoints));
			for (unsigned int i=0; i<mTracks.size(); ++i)
			{
				const TrackInfo& trackInfo = mTracks[i];
				if (trackInfo.isValid)
				{
					float r = trackInfo.color[0]/255.0f;
					float g = trackInfo.color[1]/255.0f;
					float b = trackInfo.color[2]/255.0f;
					float x = (float) trackInfo.x;
					float y = (float) trackInfo.y;
					float z = (float) trackInfo.z;
					output.write((char*)&r, sizeof(r));
					output.write((char*)&g, sizeof(g));
					output.write((char*)&b, sizeof(b));
					output.write((char*)&x, sizeof(x));
					output.write((char*)&y, sizeof(y));
					output.write((char*)&z, sizeof(z));

					unsigned int viewPointLength = (unsigned int) trackInfo.viewpoints.size();
					output.write((char*)&viewPointLength, sizeof(viewPointLength));
					for (unsigned int j=0; j<viewPointLength; ++j)
					{
						unsigned int indexImg      = trackInfo.viewpoints[j].first;
						unsigned int indexFeature  = trackInfo.viewpoints[j].second;
						output.write((char*)&indexImg, sizeof(indexImg));

						const PictureInfo& pictureInfo = mPictures[indexImg];
						const Feature& feature = pictureInfo.features[indexFeature];
						float x           = feature.x*pictureInfo.scale;
						float y           = feature.y*pictureInfo.scale;
						float scale       = feature.scale;
						float orientation = feature.orientation;						
						const float* descriptor = &pictureInfo.descriptors[128*indexFeature];
						output.write((char*)&x, sizeof(x));
						output.write((char*)&y, sizeof(y));
						output.write((char*)&scale, sizeof(scale));
						output.write((char*)&orientation, sizeof(orientation));
						output.write((char*)descriptor, sizeof(float)*128);
					}
				}
			}
		}
		output.close();
	}
}