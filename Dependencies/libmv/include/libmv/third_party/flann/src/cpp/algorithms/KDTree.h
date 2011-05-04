/*
Copyright 2008-2009  Marius Muja (mariusm@cs.ubc.ca). All rights reserved.
Copyright 2008-2009  David G. Lowe (lowe@cs.ubc.ca). All rights reserved.

THE BSD LICENSE

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions
are met:

1. Redistributions of source code must retain the above copyright
   notice, this list of conditions and the following disclaimer.
2. Redistributions in binary form must reproduce the above copyright
   notice, this list of conditions and the following disclaimer in the
   documentation and/or other materials provided with the distribution.

THIS SOFTWARE IS PROVIDED BY THE AUTHOR ``AS IS'' AND ANY EXPRESS OR
IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY DIRECT, INDIRECT,
INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT
NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/
#ifndef KDTREE_H
#define KDTREE_H

#include <algorithm>
#include <map>
#include <cassert>
#include "Heap.h"
#include "common.h"
#include "constants.h"
#include "Allocator.h"
#include "Dataset.h"
#include "ResultSet.h"
#include "Random.h"
#include "NNIndex.h"

using namespace std;


/**
 * Randomized kd-tree index
 *
 * Contains the k-d trees and other information for indexing a set of points
 * for nearest-neighbor matching.
 */
class KDTree : public NNIndex
{

	enum {
		/**
		 * To improve efficiency, only SAMPLE_MEAN random values are used to
		 * compute the mean and variance at each level when building a tree.
		 * A value of 100 seems to perform as well as using all values.
		 */
		SAMPLE_MEAN = 100,
		/**
		 * Top random dimensions to consider
		 *
		 * When creating random trees, the dimension on which to subdivide is
		 * selected at random from among the top RAND_DIM dimensions with the
		 * highest variance.  A value of 5 works well.
		 */
		RAND_DIM=5
	};


	/**
	 * Number of randomized trees that are used
	 */
	int numTrees;

	/**
	 *  Array of indices to vectors in the dataset.  When doing lookup,
	 *  this is used instead to mark checkID.
	 */
	int* vind;

	/**
	 * An unique ID for each lookup.
	 */
	int checkID;

	/**
	 * The dataset used by this index
	 */
	Dataset<float>& dataset;

    int size_;
    int veclen_;


    float* mean;
    float* var;


	/*--------------------- Internal Data Structures --------------------------*/

	/**
	 * A node of the binary k-d tree.
	 *
	 *  This is   All nodes that have vec[divfeat] < divval are placed in the
	 *   child1 subtree, else child2., A leaf node is indicated if both children are NULL.
	 */
	struct TreeSt {
		/**
		 * Index of the vector feature used for subdivision.
		 * If this is a leaf node (both children are NULL) then
		 * this holds vector index for this leaf.
		 */
		int divfeat;
		/**
		 * The value used for subdivision.
		 */
		float divval;
		/**
		 * The child nodes.
		 */
		TreeSt *child1, *child2;
	};
	typedef TreeSt* Tree;

    /**
     * Array of k-d trees used to find neighbors.
     */
    Tree* trees;
    typedef BranchStruct<Tree> BranchSt;
    typedef BranchSt* Branch;
    /**
     * Priority queue storing intermediate branches in the best-bin-first search
     */
    Heap<BranchSt>* heap;


	/**
	 * Pooled memory allocator.
	 *
	 * Using a pooled memory allocator is more efficient
	 * than allocating memory directly when there is a large
	 * number small of memory allocations.
	 */
	PooledAllocator pool;



public:

    flann_algorithm_t getType() const
    {
        return KDTREE;
    }

	/**
	 * KDTree constructor
	 *
	 * Params:
	 * 		inputData = dataset with the input features
	 * 		params = parameters passed to the kdtree algorithm
	 */
	KDTree(Dataset<float>& inputData, Params params) : dataset(inputData)
	{
        size_ = dataset.rows;
        veclen_ = dataset.cols;

		// get the parameters
		numTrees = (int)params["trees"];

		trees = new Tree[numTrees];
		heap = new Heap<BranchSt>(size_);
		checkID = -1000;

		// Create a permutable array of indices to the input vectors.
		vind = new int[size_];
		for (int i = 0; i < size_; i++) {
			vind[i] = i;
		}

        mean = new float[veclen_];
        var = new float[veclen_];
	}

	/**
	 * Standard destructor
	 */
	~KDTree()
	{
		delete[] vind;
        delete[] trees;
		delete heap;
        delete[] mean;
        delete[] var;
	}


	/**
	 * Builds the index
	 */
	void buildIndex()
	{
		/* Construct the randomized trees. */
		for (int i = 0; i < numTrees; i++) {
			/* Randomize the order of vectors to allow for unbiased sampling. */
			for (int j = size_; j > 0; --j) {
// 				int rand = cast(int) (drand48() * size);
				int rnd = rand_int(j);
				assert(rnd >=0 && rnd < size_);
				swap(vind[j-1], vind[rnd]);
			}
			trees[i] = NULL;
			divideTree(&trees[i], 0, size_ - 1);
		}
	}


    /**
    *  Returns size of index.
    */
    int size() const
    {
        return size_;
    }

    /**
    * Returns the length of an index feature.
    */
    int veclen() const
    {
        return veclen_;
    }


	/**
	 * Computes the inde memory usage
	 * Returns: memory used by the index
	 */
	int usedMemory() const
	{
		return  pool.usedMemory+pool.wastedMemory+dataset.rows*sizeof(int);   // pool memory and vind array memory
	}


    /**
     * Find set of nearest neighbors to vec. Their indices are stored inside
     * the result object.
     *
     * Params:
     *     result = the result object in which the indices of the nearest-neighbors are stored
     *     vec = the vector for which to search the nearest neighbors
     *     maxCheck = the maximum number of restarts (in a best-bin-first manner)
     */
    void findNeighbors(ResultSet& result, float* vec, Params searchParams)
    {
        int maxChecks;
        if (searchParams.find("checks") == searchParams.end()) {
            maxChecks = -1;
        }
        else {
            maxChecks = (int)searchParams["checks"];
        }

        if (maxChecks<0) {
            getExactNeighbors(result, vec);
        } else {
            getNeighbors(result, vec, maxChecks);
        }
    }


	void continueSearch(ResultSet& result, float* vec, int maxCheck)
	{
		BranchSt branch;

		int checkCount = 0;

		/* Keep searching other branches from heap until finished. */
		while ( heap->popMin(branch) && (checkCount < maxCheck || !result.full() )) {
			searchLevel(result, vec, branch.node,branch.mindistsq, checkCount, maxCheck);
		}

		assert(result.full());
	}


    Params estimateSearchParams(float precision, Dataset<float>* testset = NULL)
    {
        Params params;
        (void)precision;
        (void)testset;
        return params;
    }


private:

	/**
	 * Create a tree node that subdivides the list of vecs from vind[first]
	 * to vind[last].  The routine is called recursively on each sublist.
	 * Place a pointer to this new tree node in the location pTree.
	 *
	 * Params: pTree = the new node to create
	 * 			first = index of the first vector
	 * 			last = index of the last vector
	 */
	void divideTree(Tree* pTree, int first, int last)
	{
		Tree node;

		node = pool.allocate<TreeSt>(); // allocate memory
		*pTree = node;

		/* If only one exemplar remains, then make this a leaf node. */
		if (first == last) {
			node->child1 = node->child2 = NULL;    /* Mark as leaf node. */
			node->divfeat = vind[first];    /* Store index of this vec. */
		} else {
			chooseDivision(node, first, last);
			subdivide(node, first, last);
		}
	}




	/**
	 * Choose which feature to use in order to subdivide this set of vectors.
	 * Make a random choice among those with the highest variance, and use
	 * its variance as the threshold value.
	 */
	void chooseDivision(Tree node, int first, int last)
	{
        memset(mean,0,veclen_*sizeof(float));
        memset(var,0,veclen_*sizeof(float));

		/* Compute mean values.  Only the first SAMPLE_MEAN values need to be
			sampled to get a good estimate.
		*/
		int end = min(first + SAMPLE_MEAN, last);
		int count = end - first + 1;
		for (int j = first; j <= end; ++j) {
			float* v = dataset[vind[j]];
            for (int k=0; k<veclen_; ++k) {
                mean[k] += v[k];
            }
		}
        for (int k=0; k<veclen_; ++k) {
            mean[k] /= count;
        }

		/* Compute variances (no need to divide by count). */
		for (int j = first; j <= end; ++j) {
			float* v = dataset[vind[j]];
            for (int k=0; k<veclen_; ++k) {
                float dist = v[k] - mean[k];
                var[k] += dist * dist;
            }
		}
		/* Select one of the highest variance indices at random. */
		node->divfeat = selectDivision(var);
		node->divval = mean[node->divfeat];

	}


	/**
	 * Select the top RAND_DIM largest values from v and return the index of
	 * one of these selected at random.
	 */
	int selectDivision(float* v)
	{
		int num = 0;
		int topind[RAND_DIM];

		/* Create a list of the indices of the top RAND_DIM values. */
		for (int i = 0; i < veclen_; ++i) {
			if (num < RAND_DIM  ||  v[i] > v[topind[num-1]]) {
				/* Put this element at end of topind. */
				if (num < RAND_DIM) {
					topind[num++] = i;            /* Add to list. */
				}
				else {
					topind[num-1] = i;         /* Replace last element. */
				}
				/* Bubble end value down to right location by repeated swapping. */
				int j = num - 1;
				while (j > 0  &&  v[topind[j]] > v[topind[j-1]]) {
					swap(topind[j], topind[j-1]);
					--j;
				}
			}
		}
		/* Select a random integer in range [0,num-1], and return that index. */
// 		int rand = cast(int) (drand48() * num);
		int rnd = rand_int(num);
		assert(rnd >=0 && rnd < num);
		return topind[rnd];
	}


	/**
	 *  Subdivide the list of exemplars using the feature and division
	 *  value given in this node.  Call divideTree recursively on each list.
	*/
	void subdivide(Tree node, int first, int last)
	{
		/* Move vector indices for left subtree to front of list. */
		int i = first;
		int j = last;
		while (i <= j) {
			int ind = vind[i];
			float val = dataset[ind][node->divfeat];
			if (val < node->divval) {
				++i;
			} else {
				/* Move to end of list by swapping vind i and j. */
				swap(vind[i], vind[j]);
				--j;
			}
		}
		/* If either list is empty, it means we have hit the unlikely case
			in which all remaining features are identical. Split in the middle
            to maintain a balanced tree.
		*/
		if ( (i == first) || (i == last+1)) {
            i = (first+last+1)/2;
		}

		divideTree(& node->child1, first, i - 1);
		divideTree(& node->child2, i, last);
	}



	/**
	 * Performs an exact nearest neighbor search. The exact search performs a full
	 * traversal of the tree.
	 */
	void getExactNeighbors(ResultSet& result, float* vec)
	{
		checkID -= 1;  /* Set a different unique ID for each search. */

		if (numTrees > 1) {
            fprintf(stderr,"It doesn't make any sense to use more than one tree for exact search");
		}
		if (numTrees>0) {
			searchLevelExact(result, vec, trees[0], 0.0);
		}
		assert(result.full());
	}

	/**
	 * Performs the approximate nearest-neighbor search. The search is approximate
	 * because the tree traversal is abandoned after a given number of descends in
	 * the tree.
	 */
	void getNeighbors(ResultSet& result, float* vec, int maxCheck)
	{
		int i;
		BranchSt branch;

		int checkCount = 0;
		heap->clear();
		checkID -= 1;  /* Set a different unique ID for each search. */

		/* Search once through each tree down to root. */
		for (i = 0; i < numTrees; ++i) {
			searchLevel(result, vec, trees[i], 0.0, checkCount, maxCheck);
		}

		/* Keep searching other branches from heap until finished. */
		while ( heap->popMin(branch) && (checkCount < maxCheck || !result.full() )) {
			searchLevel(result, vec, branch.node,branch.mindistsq, checkCount, maxCheck);
		}

		assert(result.full());
	}


	/**
	 *  Search starting from a given node of the tree.  Based on any mismatches at
	 *  higher levels, all exemplars below this level must have a distance of
	 *  at least "mindistsq".
	*/
	void searchLevel(ResultSet& result, float* vec, Tree node, float mindistsq, int& checkCount, int maxCheck)
	{
		if (result.worstDist()<mindistsq) {
//			printf("Ignoring branch, too far\n");
			return;
		}

		float val, diff;
		Tree bestChild, otherChild;

		/* If this is a leaf node, then do check and return. */
		if (node->child1 == NULL  &&  node->child2 == NULL) {

			/* Do not check same node more than once when searching multiple trees.
				Once a vector is checked, we set its location in vind to the
				current checkID.
			*/
			if (vind[node->divfeat] == checkID || checkCount>=maxCheck) {
				if (result.full()) return;
			}
            checkCount++;
			vind[node->divfeat] = checkID;

			result.addPoint(dataset[node->divfeat],node->divfeat);
			return;
		}

		/* Which child branch should be taken first? */
		val = vec[node->divfeat];
		diff = val - node->divval;
		bestChild = (diff < 0) ? node->child1 : node->child2;
		otherChild = (diff < 0) ? node->child2 : node->child1;

		/* Create a branch record for the branch not taken.  Add distance
			of this feature boundary (we don't attempt to correct for any
			use of this feature in a parent node, which is unlikely to
			happen and would have only a small effect).  Don't bother
			adding more branches to heap after halfway point, as cost of
			adding exceeds their value.
		*/

		double new_distsq = flann_dist(&val, &val+1, &node->divval, mindistsq);
//		if (2 * checkCount < maxCheck  ||  !result.full()) {
		if (new_distsq < result.worstDist() ||  !result.full()) {
			heap->insert( BranchSt::make_branch(otherChild, new_distsq) );
		}

		/* Call recursively to search next level down. */
		searchLevel(result, vec, bestChild, mindistsq, checkCount, maxCheck);
	}

	/**
	 * Performs an exact search in the tree starting from a node.
	 */
	void searchLevelExact(ResultSet& result, float* vec, Tree node, float mindistsq)
	{
		if (mindistsq>result.worstDist()) {
			return;
		}

		float val, diff;
		Tree bestChild, otherChild;

		/* If this is a leaf node, then do check and return. */
		if (node->child1 == NULL  &&  node->child2 == NULL) {

			/* Do not check same node more than once when searching multiple trees.
				Once a vector is checked, we set its location in vind to the
				current checkID.
			*/
			if (vind[node->divfeat] == checkID)
				return;
			vind[node->divfeat] = checkID;

			result.addPoint(dataset[node->divfeat],node->divfeat);
			return;
		}

		/* Which child branch should be taken first? */
		val = vec[node->divfeat];
		diff = val - node->divval;
		bestChild = (diff < 0) ? node->child1 : node->child2;
		otherChild = (diff < 0) ? node->child2 : node->child1;


		/* Call recursively to search next level down. */
		searchLevelExact(result, vec, bestChild, mindistsq);
		double new_distsq = flann_dist(&val, &val+1, &node->divval, mindistsq);
		searchLevelExact(result, vec, otherChild, new_distsq);
	}

};   // class KDTree

register_index(KDTREE,KDTree)

#endif //KDTREE_H
