/***********************************************************************
 * Software License Agreement (BSD License)
 *
 * Copyright 2008-2009  Marius Muja (mariusm@cs.ubc.ca). All rights reserved.
 * Copyright 2008-2009  David G. Lowe (lowe@cs.ubc.ca). All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR ``AS IS'' AND ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
 * IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT
 * NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE NNIndexGOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
 * THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *************************************************************************/

#ifndef SAVING_H_
#define SAVING_H_

#include "flann/general.h"
#include "flann/algorithms/nn_index.h"
#include <cstdio>
#include <cstring>
#include <vector>

namespace flann
{

template <typename T>
struct Datatype {};
template<>
struct Datatype<char> { static flann_datatype_t type() { return FLANN_INT8; } };
template<>
struct Datatype<short> { static flann_datatype_t type() { return FLANN_INT16; } };
template<>
struct Datatype<int> { static flann_datatype_t type() { return FLANN_INT32; } };
template<>
struct Datatype<unsigned char> { static flann_datatype_t type() { return FLANN_UINT8; } };
template<>
struct Datatype<unsigned short> { static flann_datatype_t type() { return FLANN_UINT16; } };
template<>
struct Datatype<unsigned int> { static flann_datatype_t type() { return FLANN_UINT32; } };
template<>
struct Datatype<float> { static flann_datatype_t type() { return FLANN_FLOAT32; } };
template<>
struct Datatype<double> { static flann_datatype_t type() { return FLANN_FLOAT64; } };



FLANN_EXPORT extern const char FLANN_SIGNATURE[];
FLANN_EXPORT extern const char FLANN_VERSION[];

/**
 * Structure representing the index header.
 */
struct IndexHeader
{
    char signature[16];
    char version[16];
    flann_datatype_t data_type;
    flann_algorithm_t index_type;
    int rows;
    int cols;
};

/**
 * Saves index header to stream
 *
 * @param stream - Stream to save to
 * @param index - The index to save
 */
template<typename Distance>
void save_header(FILE* stream, const NNIndex<Distance>& index)
{
    IndexHeader header;
    memset(header.signature, 0, sizeof(header.signature));
    strcpy(header.signature, FLANN_SIGNATURE);
    memset(header.version, 0, sizeof(header.version));
    strcpy(header.version, FLANN_VERSION);
    header.data_type = Datatype<typename Distance::ElementType>::type();
    header.index_type = index.getType();
    header.rows = index.size();
    header.cols = index.veclen();

    std::fwrite(&header, sizeof(header),1,stream);
}


/**
 *
 * @param stream - Stream to load from
 * @return Index header
 */
FLANN_EXPORT IndexHeader load_header(FILE* stream);


template<typename T>
void save_value(FILE* stream, const T& value, int count = 1)
{
    fwrite(&value, sizeof(value),count, stream);
}

template<typename T>
void save_value(FILE* stream, const flann::Matrix<T>& value, int count = 1)
{
    fwrite(&value, sizeof(value),1, stream);
    fwrite(value.data, sizeof(T),value.rows*value.cols, stream);
}

template<typename T>
void save_value(FILE* stream, const std::vector<T>& value)
{
    size_t size = value.size();
    fwrite(&size, sizeof(size_t), 1, stream);
    fwrite(&value[0], sizeof(T), size, stream);
}

template<typename T>
void load_value(FILE* stream, T& value, int count = 1)
{
    int read_cnt = fread(&value, sizeof(value), count, stream);
    if (read_cnt != count) {
        throw FLANNException("Cannot read from file");
    }
}

template<typename T>
void load_value(FILE* stream, flann::Matrix<T>& value)
{
    int read_cnt = fread(&value, sizeof(value), 1, stream);
    if (read_cnt != 1) {
        throw FLANNException("Cannot read from file");
    }
    value.data = new T[value.rows*value.cols];
    read_cnt = fread(value.data, sizeof(T), value.rows*value.cols, stream);
    if (read_cnt != int(value.rows*value.cols)) {
        throw FLANNException("Cannot read from file");
    }
}


template<typename T>
void load_value(FILE* stream, std::vector<T>& value)
{
    size_t size;
    int read_cnt = fread(&size, sizeof(size_t), 1, stream);
    if (read_cnt!=1) {
        throw FLANNException("Cannot read from file");
    }
    value.resize(size);
    read_cnt = fread(&value[0], sizeof(T), size, stream);
    if (read_cnt!=int(size)) {
        throw FLANNException("Cannot read from file");
    }
}

}

#endif /* SAVING_H_ */
