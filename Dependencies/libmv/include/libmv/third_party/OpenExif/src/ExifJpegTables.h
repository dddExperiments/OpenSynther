/*
 * Copyright (c) 2000-2009, Eastman Kodak Company
 * All rights reserved.
 * 
 * Redistribution and use in source and binary forms, with or without 
 * modification,are permitted provided that the following conditions are met:
 * 
 *     * Redistributions of source code must retain the above copyright notice, 
 *       this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the 
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Eastman Kodak Company nor the names of its 
 *       contributors may be used to endorse or promote products derived from 
 *       this software without specific prior written permission.
 * 
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE 
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE 
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR 
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF 
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS 
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN 
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) 
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE 
 * POSSIBILITY OF SUCH DAMAGE.
 * 
 *
 * Portions of the Original Code are
 * Copyright (c) 1988-1996 Sam Leffler
 * Copyright (c) 1991-1996 Silicon Graphics, Inc.
 *
 * Creation Date: 07/14/2001
 *
 * Original Author: 
 * George Sotak george.sotak@kodak.com 
 *
 * Contributor(s): 
 * FirstName LastName <email address>
 * FirstName LastName <email address>
 */ 

#ifndef _JPEG_TABLES_H_
#define _JPEG_TABLES_H_


#include "ExifTypeDefs.h"

ExifJpegQuantTable q_1 = {  0, 
	{2, 2, 3, 3, 3, 4, 3, 4, 
	2, 2, 4, 3, 2, 3, 4, 3,
	2, 2, 2, 2, 2, 5, 3, 4,
	2, 2, 2, 2, 5, 3, 5, 7,
	2, 3, 4, 2, 6, 6, 6, 8,
	3, 4, 2, 6, 7, 5, 9, 6,
	3, 3, 5, 8, 4, 4, 5, 8,
	4, 4, 3, 3, 4, 4, 9,11}};

unsigned char h_DC_1_bits[] = {0, 0, 2, 3, 1, 1, 1, 1, 0, 3, 0, 0, 0, 0, 0, 0, 0};
#define h_DC_1_bits_size 17
unsigned char h_DC_1_code[] = {0x01 ,0x02 ,0x00 ,0x03 ,0x04 ,0x05 ,0x06 ,0x07 ,0x08 ,0x09 ,0x0A ,0x0B};
#define h_DC_1_code_size 12
 
unsigned char h_AC_1_bits[] = {0, 0, 1, 3, 2, 4, 3, 6, 4, 4, 5, 1, 2, 0, 0, 0, 127};
#define h_AC_1_bits_size 17
unsigned char h_AC_1_code[] = {0x01, 0x00, 0x02, 0x11, 0x03, 0x21, 0x04, 0x12, 0x31, 0x41, 0x51, 0x61, 0x71, 0x05, 0x13, 0x22,
    0x81, 0x91, 0xA1, 0x32, 0xB1, 0xC1, 0xF0, 0x14, 0x42, 0x52, 0xD1, 0x06, 0x23, 0x62, 0x72, 0xE1, 
    0x33, 0xF1, 0x15, 0x82, 0x92, 0xB2, 0x24, 0x43, 0x53, 0x73, 0xA2, 0x34, 0x63, 0xC2, 0x16, 0x83, 
    0x25, 0xB3, 0xD2, 0x07, 0x35, 0x44, 0x54, 0x64, 0x74, 0x93, 0xA3, 0xE2, 0xC3, 0xF2, 0x17, 0x45, 
    0x55, 0x84, 0x94, 0xA4, 0x26, 0x65, 0x75, 0xB4, 0xD3, 0x08, 0x09, 0x0A, 0x18, 0x19, 0x1A, 0x27, 
    0x28, 0x29, 0x2A, 0x36, 0x37, 0x38, 0x39, 0x3A, 0x46, 0x47, 0x48, 0x49, 0x4A, 0x56, 0x57, 0x58, 
    0x59, 0x5A, 0x66, 0x67, 0x68, 0x69, 0x6A, 0x76, 0x77, 0x78, 0x79, 0x7A, 0x85, 0x86, 0x87, 0x88, 
    0x89, 0x8A, 0x95, 0x96, 0x97, 0x98, 0x99, 0x9A, 0xA5, 0xA6, 0xA7, 0xA8, 0xA9, 0xAA, 0xB5, 0xB6, 
    0xB7, 0xB8, 0xB9, 0xBA, 0xC4, 0xC5, 0xC6, 0xC7, 0xC8, 0xC9, 0xCA, 0xD4, 0xD5, 0xD6, 0xD7, 0xD8, 
    0xD9, 0xDA, 0xE3, 0xE4, 0xE5, 0xE6, 0xE7, 0xE8, 0xE9, 0xEA, 0xF3, 0xF4, 0xF5, 0xF6, 0xF7, 0xF8, 
    0xF9, 0xFA};
#define h_AC_1_code_size 162

#endif //_JPEG_TABLES_H_
