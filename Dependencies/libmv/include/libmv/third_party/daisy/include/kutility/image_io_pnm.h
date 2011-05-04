//////////////////////////////////////////////////////////////////////////
// Software License Agreement (BSD License)                             //
//                                                                      //
// Copyright (c) 2009                                                   //
// Engin Tola                                                           //
// web   : http://cvlab.epfl.ch/~tola                                   //
// email : engin.tola@epfl.ch                                           //
//                                                                      //
// All rights reserved.                                                 //
//                                                                      //
// Redistribution and use in source and binary forms, with or without   //
// modification, are permitted provided that the following conditions   //
// are met:                                                             //
//                                                                      //
//  * Redistributions of source code must retain the above copyright    //
//    notice, this list of conditions and the following disclaimer.     //
//  * Redistributions in binary form must reproduce the above           //
//    copyright notice, this list of conditions and the following       //
//    disclaimer in the documentation and/or other materials provided   //
//    with the distribution.                                            //
//  * Neither the name of the EPFL nor the names of its                 //
//    contributors may be used to endorse or promote products derived   //
//    from this software without specific prior written permission.     //
//                                                                      //
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS  //
// "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT    //
// LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS    //
// FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE       //
// COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,  //
// INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, //
// BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;     //
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER     //
// CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT   //
// LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN    //
// ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE      //
// POSSIBILITY OF SUCH DAMAGE.                                          //
//                                                                      //
// See licence.txt file for more details                                //
//////////////////////////////////////////////////////////////////////////

#ifndef KUTILITY_IMAGE_IO_PNM_H
#define KUTILITY_IMAGE_IO_PNM_H

#include <fstream>
#include "string.h"
#include <cstdlib>
#include "limits.h"

#ifndef uchar
typedef unsigned char uchar;
#endif

namespace kutility
{
   void load_pbm(const char* name, uchar* &data, int &height, int &width);
   void load_pgm(const char* name, uchar* &data, int &height, int &width);
   void load_ppm(const char* name, uchar* &data, int &height, int &width);

   void save_pbm(const char* name, uchar *im, int height, int width);

   template<class T>
   void save_pgm(const char* name, T *im, int height, int width)
   {
      std::ofstream file(name, std::ios::out | std::ios::binary);

      file << "P5\n" << width << " " << height << "\n" << UCHAR_MAX << "\n";

      for( int k=0; k<width*height; k++ )
      {
         file <<(uchar)im[k];
      }
      file.close();
   }

   template<class T>
   void save_ppm(const char* name, T *im, int height, int width)
   {
      std::ofstream file(name, std::ios::out | std::ios::binary);

      file << "P6\n" << width << " " << height << "\n" << UCHAR_MAX << "\n";
      for( int k=0; k<3*width*height; k++ )
      {
         file <<(uchar)im[k];
      }
      file.close();
   }

   void get_size_ppm(const char* name, int &height, int &width);
}

#endif
