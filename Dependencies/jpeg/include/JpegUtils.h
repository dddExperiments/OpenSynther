/*
	Copyright (c) 2010 ASTRE Henri (http://www.visual-experiments.com)

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

namespace Jpeg
{
	struct Image
	{
		Image();

		unsigned char* buffer;
		unsigned int width;
		unsigned int height;
		
		//3: RGB, 1: luminance
		unsigned int nbComponents;

		size_t getBufferSize() const;
		void getColor(unsigned int x, unsigned int y, unsigned char* color) const;
	};
	
	enum ResizeMethod
	{
		ResizeMethod_NearestNeighbor,
		ResizeMethod_Bilinear
	};

	//load jpeg from file (this function allocate the buffer of img struct)
	bool load(const std::string& filename, Image& img);

	//write jpeg (quality 0: bad, 100: good)
	bool write(const std::string& filename, Image& img, unsigned int quality = 75);
	
	//save image as raw binary
	bool writeRaw(const std::string& filename, Image& img);
	
	//get dimension from header only
	bool getDimension(const std::string& filename, unsigned int& width, unsigned int& height);

	//downsize picture to maxSize if needed (using bilinear interpolation by default)
	void setMaxSize(Image& img, unsigned int maxSize, ResizeMethod method = Jpeg::ResizeMethod_Bilinear);

	//resize the picture to given dimension (using bilinear interpolation by default)
	void resize(Image& img, unsigned int width, unsigned int height, ResizeMethod method = Jpeg::ResizeMethod_Bilinear); // -> wrong !
}