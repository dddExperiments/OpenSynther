/***********************************************************************
 * Software License Agreement (BSD License)
 *
 * Copyright 2008-2009  Marius Muja (mariusm@cs.ubc.ca). All rights reserved.
 * Copyright 2008-2009  David G. Lowe (lowe@cs.ubc.ca). All rights reserved.
 *
 * THE BSD LICENSE
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
 * NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
 * THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *************************************************************************/

#ifndef COMPOSITETREE_H
#define COMPOSITETREE_H

#include "flann/general.h"
#include "flann/algorithms/nn_index.h"
#include "flann/algorithms/kdtree_index.h"
#include "flann/algorithms/kmeans_index.h"

namespace flann
{


struct CompositeIndexParams : public IndexParams
{
    CompositeIndexParams(int trees_ = 4, int branching_ = 32, int iterations_ = 11,
                         flann_centers_init_t centers_init_ = FLANN_CENTERS_RANDOM, float cb_index_ = 0.2 ) :
        IndexParams(FLANN_INDEX_COMPOSITE),
        trees(trees_),
        branching(branching_),
        iterations(iterations_),
        centers_init(centers_init_),
        cb_index(cb_index_)
    {
    }

    int trees;                 // number of randomized trees to use (for kdtree)
    int branching;             // branching factor (for kmeans tree)
    int iterations;            // max iterations to perform in one kmeans clustering (kmeans tree)
    flann_centers_init_t centers_init;          // algorithm used for picking the initial cluster centers for kmeans tree
    float cb_index;            // cluster boundary index. Used when searching the kmeans tree

    void fromParameters(const FLANNParameters& p)
    {
        assert(p.algorithm == algorithm);
        trees = p.trees;
        branching = p.branching;
        iterations = p.iterations;
        centers_init = p.centers_init;
        cb_index = p.cb_index;
    }

    void toParameters(FLANNParameters& p) const
    {
        p.algorithm = algorithm;
        p.trees = trees;
        p.branching = branching;
        p.iterations = iterations;
        p.centers_init = centers_init;
        p.cb_index = cb_index;
    }

    void print() const
    {
        logger.info("Index type: %d\n", (int)algorithm);
        logger.info("Trees: %d\n", trees);
        logger.info("Branching: %d\n", branching);
        logger.info("Iterations: %d\n", iterations);
        logger.info("Centres initialisation: %d\n", centers_init);
        logger.info("Cluster boundary weight: %g\n", cb_index);
    }
};



template <typename Distance>
class CompositeIndex : public NNIndex<Distance>
{
    typedef typename Distance::ElementType ElementType;
    typedef typename Distance::ResultType DistanceType;

    KMeansIndex<Distance>* kmeans;
    KDTreeIndex<Distance>* kdtree;

    const Matrix<ElementType> dataset;

    const CompositeIndexParams index_params;

    Distance distance;

public:

    CompositeIndex(const Matrix<ElementType>& inputData, const CompositeIndexParams& params = CompositeIndexParams(),
                   Distance d = Distance()) :
        dataset(inputData), index_params(params), distance(d)
    {
        KDTreeIndexParams kdtree_params(params.trees);
        KMeansIndexParams kmeans_params(params.branching, params.iterations, params.centers_init, params.cb_index);

        kdtree = new KDTreeIndex<Distance>(inputData, kdtree_params, d);
        kmeans = new KMeansIndex<Distance>(inputData, kmeans_params, d);

    }

    virtual ~CompositeIndex()
    {
        delete kdtree;
        delete kmeans;
    }


    flann_algorithm_t getType() const
    {
        return FLANN_INDEX_COMPOSITE;
    }


    size_t size() const
    {
        return dataset.rows;
    }

    size_t veclen() const
    {
        return dataset.cols;
    }


    int usedMemory() const
    {
        return kmeans->usedMemory() + kdtree->usedMemory();
    }

    void buildIndex()
    {
        logger.info("Building kmeans tree...\n");
        kmeans->buildIndex();
        logger.info("Building kdtree tree...\n");
        kdtree->buildIndex();
    }


    void saveIndex(FILE* stream)
    {
        kmeans->saveIndex(stream);
        kdtree->saveIndex(stream);
    }


    void loadIndex(FILE* stream)
    {
        kmeans->loadIndex(stream);
        kdtree->loadIndex(stream);
    }

    void findNeighbors(ResultSet<DistanceType>& result, const ElementType* vec, const SearchParams& searchParams)
    {
        kmeans->findNeighbors(result, vec, searchParams);
        kdtree->findNeighbors(result, vec, searchParams);
    }

    const IndexParams* getParameters() const
    {
        return &index_params;
    }


};

}

#endif //COMPOSITETREE_H
