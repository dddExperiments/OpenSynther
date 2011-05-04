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

#include "JpegUtils.h"

#include "jpeglib.h"

#include <fstream>
#include <iostream>
#include <cmath>
#define round(x) floor((x)+0.5) //It's a shame that it isn't included in cmath !

using namespace Jpeg;

Image::Image()
{
	buffer       = NULL;
	width        = 0;
	height       = 0;
	nbComponents = 0;
}

size_t Image::getBufferSize() const
{
	return width * height * nbComponents;
}

void Image::getColor(unsigned int x, unsigned int y, unsigned char* color) const
{
	if (nbComponents == 3)
	{
		unsigned char* tmp = buffer + y*width*3 + x*3;
		color[0] = tmp[0];
		color[1] = tmp[1];
		color[2] = tmp[2];
		color[3] = 255;	
	}
	else if (nbComponents == 1)
	{
		unsigned char* lum = buffer + y*width + x;
		color[0] = lum[0];
		color[1] = lum[0];
		color[2] = lum[0];
		color[3] = 255;
	}
	else
	{
		color[0] = 0;
		color[1] = 0;
		color[2] = 0;
		color[3] = 0;
	}
}

bool Jpeg::load(const std::string& filename, Image& img)
{
	FILE* fp = fopen(filename.c_str(), "rb");
	if (fp)
	{
		struct jpeg_decompress_struct cinfo;
		struct jpeg_error_mgr jerr;

		cinfo.err = jpeg_std_error(&jerr);
		jpeg_create_decompress(&cinfo);
		jpeg_stdio_src(&cinfo, fp);
		jpeg_read_header(&cinfo, true);
		jpeg_start_decompress(&cinfo);

		unsigned int width        = cinfo.output_width;
		unsigned int height       = cinfo.output_height;
		unsigned int nbComponents = cinfo.output_components;	

		img.width        = width;
		img.height       = height;
		img.nbComponents = nbComponents;
		img.buffer       = new unsigned char[img.getBufferSize()];
		
		size_t widthInBytes = width * nbComponents;
		JSAMPROW row = new JSAMPLE[widthInBytes];

		for (unsigned int y=0; y<height; y++) 
		{
			jpeg_read_scanlines(&cinfo, &row, 1);
			memcpy(img.buffer + y*widthInBytes, row, widthInBytes);
		}

		jpeg_finish_decompress(&cinfo);
		jpeg_destroy_decompress(&cinfo);

		fclose(fp);
		delete[] row;

		return true;
	}
	else
		return false;
}

bool Jpeg::write(const std::string& filename, Image& img, unsigned int quality)
{
	FILE* fp = fopen(filename.c_str(), "wb");
	if (fp)
	{
		struct jpeg_compress_struct cinfo;
		struct jpeg_error_mgr jerr;

		cinfo.err = jpeg_std_error(&jerr);
		jpeg_create_compress(&cinfo);
		jpeg_stdio_dest(&cinfo, fp);
		cinfo.image_width      = img.width;
		cinfo.image_height     = img.height;
		cinfo.input_components = img.nbComponents;
		cinfo.in_color_space   = (img.nbComponents == 3 ? JCS_RGB : JCS_GRAYSCALE);

		jpeg_set_defaults(&cinfo);
		jpeg_set_quality(&cinfo, (int)quality, true);
		jpeg_start_compress(&cinfo, true);

		size_t widthInBytes = img.width * img.nbComponents;
		JSAMPROW row = new JSAMPLE[widthInBytes];

		for (unsigned int y=0; y<img.height; y++) 
		{
			memcpy(row, img.buffer + y*widthInBytes, widthInBytes);
			jpeg_write_scanlines(&cinfo, &row, 1);
		}

		jpeg_finish_compress(&cinfo);
		jpeg_destroy_compress(&cinfo);

		fclose(fp);
		delete[] row;

		return true;
	}
	else
		return false;
}

bool Jpeg::writeRaw(const std::string& filename, Image& img)
{
	std::ofstream output(filename.c_str(), std::ios::binary);
	bool opened = output.is_open();
	if (opened)
		output.write((char*) img.buffer, img.getBufferSize());
	output.close();

	return opened;
}

bool Jpeg::getDimension(const std::string& filename, unsigned int& width, unsigned int& height)
{
	struct jpeg_decompress_struct cinfo;
	struct jpeg_error_mgr jerr;

	cinfo.err = jpeg_std_error(&jerr);
	jpeg_create_decompress(&cinfo);

	FILE* fp = fopen(filename.c_str(), "rb");
	if (fp)
	{
		jpeg_stdio_src(&cinfo, fp);
		jpeg_read_header(&cinfo, true);

		width  = cinfo.image_width;
		height = cinfo.image_height;

		jpeg_destroy_decompress(&cinfo);

		fclose(fp);
		return true;
	}
	else
	{
		width  = 0;
		height = 0;
		return false;
	}
}

void Jpeg::setMaxSize(Image& img, unsigned int maxSize, ResizeMethod method)
{
	if (img.width > maxSize || img.height > maxSize)
	{
		unsigned int width;
		unsigned int height;

		if (img.width > img.height)
		{
			width  = maxSize;
			height = (unsigned int) (maxSize*img.height/(float)img.width);
		}
		else
		{
			height = maxSize;
			width  = (unsigned int) (maxSize*img.width/(float)img.height);
		}
		resize(img, width, height, method);
	}	
}

void Jpeg::resize(Image& img, unsigned int width, unsigned int height, ResizeMethod method)
{
	if (img.width != width || img.height != height)
	{
		double resizeXFactor  = img.width  / (double)width;
		double resizeYFactor  = img.height / (double)height;
		unsigned char* buffer = new unsigned char[width*height*img.nbComponents];

		if (method == ResizeMethod_NearestNeighbor)
		{
			// Nearest neighbor:
			// -----------------

			//	+----> I
			//	|
			//	V
			//	   
			//	J

			for (unsigned int j=0; j<height; ++j)
			{
				for (unsigned int i=0; i<width; ++i)		
				{				
					unsigned char* dst = buffer + j*width*img.nbComponents + i*img.nbComponents;

					unsigned int srcI  = (unsigned int) round(i*resizeXFactor);
					unsigned int srcJ  = (unsigned int) round(j*resizeYFactor);
					unsigned char* src = img.buffer + srcJ*img.width*img.nbComponents + srcI*img.nbComponents;

					for (unsigned int k=0; k<img.nbComponents; ++k)
					{
						dst[k] = src[k];
					}
				}
			}
		}
		else
		{
			//	Bilinear interpolation: http://en.wikipedia.org/wiki/Bilinear_interpolation
			//  ---------------------------------------------------------------------------

			//	A---E-------B   +----> I
			//	|   |       |   |
			//	+---X-------+   V
			//	|   |       |   
			//	D---F-------C   J
			//                  
			// X(i,j)

			// A(floor(i),  floor(j))
			// B(ceil(i),   floor(j))
			// C(ceil(i),   ceil(j))
			// D(floor(i),  ceil(j))

			unsigned char* E = new unsigned char[img.nbComponents];
			unsigned char* F = new unsigned char[img.nbComponents];

			for (unsigned int j=0; j<height; ++j)
			{
				for (unsigned int i=0; i<width; ++i)		
				{				
					unsigned char* dst = buffer + j*width*img.nbComponents + i*img.nbComponents;

					double srcI = i*resizeXFactor;
					double srcJ = j*resizeYFactor;
					unsigned int srcIfloor = (unsigned int) floor(srcI);
					unsigned int srcIceil  = (unsigned int) ceil(srcI);
					unsigned int srcJfloor = (unsigned int) floor(srcJ);
					unsigned int srcJceil  = (unsigned int) ceil(srcJ);

					unsigned char* srcA = img.buffer + srcJfloor*img.width*img.nbComponents + srcIfloor*img.nbComponents;
					unsigned char* srcB = img.buffer + srcJfloor*img.width*img.nbComponents + srcIceil*img.nbComponents;
					unsigned char* srcC = img.buffer + srcJceil*img.width*img.nbComponents  + srcIceil*img.nbComponents;
					unsigned char* srcD = img.buffer + srcJceil*img.width*img.nbComponents  + srcIfloor*img.nbComponents;

					double alpha = (srcI-srcIfloor); // dist(A, E)
					double beta  = (srcJ-srcJfloor); // dist(E, X)

					for (unsigned int k=0; k<img.nbComponents; ++k)
					{
						E[k]   = (unsigned char) (srcA[k]*(1-alpha) + srcB[k]*alpha);
						F[k]   = (unsigned char) (srcD[k]*(1-alpha) + srcC[k]*alpha);
						dst[k] = (unsigned char) (E[k]*(1-beta)     + F[k]*beta);
					}
				}
			}

			delete[] E;
			delete[] F;
		}

		img.width  = width;
		img.height = height;
		delete[] img.buffer;
		img.buffer = buffer;
	}
}