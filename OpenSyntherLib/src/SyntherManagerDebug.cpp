#include "SyntherManager.h"

#include <tinyxml.h>
#include <JpegUtils.h>

using namespace Synther;

void Manager::debugTriangulation(const std::string& projectPath)
{
	std::stringstream picturePath;
	picturePath << projectPath << "distort";
	std::vector<std::string> pictures = getPictureList(picturePath.str());

	mTracks.clear();
	mPictures.clear();

	//load pictures names
	for (unsigned int i=0; i<pictures.size(); ++i)
	{
		PictureInfo info(pictures[i]);
		unsigned int width, height;
		Jpeg::getDimension(info.filepath, width, height);

		/*unsigned int maxDim = mOptions.maxPictureDimension;
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
			width = newWidth;
			height = newHeight;
		}
		*/
		info.width  = width;
		info.height = height;

		mPictures.push_back(info);
	}

	//load info from photosynth
	PhotoSynth::Parser parser;
	std::string guid = parser.getGuid(PhotoSynth::Parser::createFilePath(projectPath, PhotoSynth::Parser::guidFilename));
	parser.parseSoap(PhotoSynth::Parser::createFilePath(projectPath, PhotoSynth::Parser::soapFilename));
	parser.parseJson(PhotoSynth::Parser::createFilePath(projectPath, PhotoSynth::Parser::jsonFilename), guid);
	loadPhotoSynthJson(&parser);

	//load tracks
	std::stringstream filePathTracks;
	filePathTracks << projectPath << "bundler_tmp\\tracks.xml";
	loadTracksForDebug(filePathTracks.str());

	//triangulate tracks
	std::stringstream pointcloudFilePath;
	pointcloudFilePath << projectPath << "bundler_tmp\\pointCloud.ply";
	triangulateTracksOfLength2();
	triangulateTracksOfLengthN();

	colorizeTracks();

	exportTracksAsPointCloud(pointcloudFilePath.str());
}

void Manager::loadTracksForDebug(const std::string& filePath)
{
	TiXmlDocument doc;
	if (doc.LoadFile(filePath))
	{
		TiXmlNode* root = doc.FirstChild("tracks");
		if (root)
		{
			for (TiXmlNode* node = root->FirstChild(); node; node = node->NextSibling())
			{
				//ignore comment
				if (node->Type() == TiXmlNode::ELEMENT)
				{		
					//add current track to mTracks
					if (std::string(node->ToElement()->Value()) == "track")
					{
						TrackInfo trackInfo;
						for (TiXmlNode* nodeV = node->FirstChild(); nodeV; nodeV = nodeV->NextSibling())
						{
							if (std::string(nodeV->ToElement()->Value()) == "v")
							{
								float x, y;
								int pictureIndex;
								nodeV->ToElement()->QueryFloatAttribute("x", &x);
								nodeV->ToElement()->QueryFloatAttribute("y", &y);
								nodeV->ToElement()->QueryIntAttribute("img", &pictureIndex);
								Synther::Feature f;
								f.x = x;
								f.y = y;
								mPictures[pictureIndex].features.push_back(f);
								int featureIndex = (int) mPictures[pictureIndex].features.size();
								trackInfo.viewpoints.push_back(std::make_pair(pictureIndex, featureIndex-1));
							}
						}
						mTracks.push_back(trackInfo);
					}
				}
			}
		}
	}
}