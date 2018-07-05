#pragma once

#include <algorithm>
#include <array>
#include <vector>


enum { DIM = 3 };

typedef unsigned char AttributeType;

typedef int DistanceType;

inline DistanceType SQ(DistanceType x) { return x * x; }

struct kdnode
{
    std::array<AttributeType, DIM> pos;
    std::array<DistanceType, DIM> data;
    std::array<unsigned long long, DIM> sq_data;
    int count;
    kdnode *left, *right;
};

typedef kdnode ObjectInfo;


/////////////////////////////////////////////////////////////////////////////////


typedef std::vector<ObjectInfo> ObjectInfos;

struct IsInfoLess
{
    IsInfoLess(int idx) : m_idx(idx) {}

    bool operator() (const ObjectInfo& x, const ObjectInfo& y) const
    {
        return x.pos[m_idx] < y.pos[m_idx];
    }
    bool operator() (const ObjectInfo* x, const ObjectInfo* y) const
    {
        return x->pos[m_idx] < y->pos[m_idx];
    }
private:
    int m_idx;
};

kdnode* insert(std::vector<ObjectInfo*>::iterator begin,
    std::vector<ObjectInfo*>::iterator end,
    kdnode* parent,
    int dir)
{
    if (begin == end)
        return 0;

    int diff = int(end - begin);
    if (1 == diff)
    {
        kdnode* node = *begin;

        node->left = 0;
        node->right = 0;

        return node;
    }

    int halfSize = diff / 2;

    auto middle = begin + halfSize;

    std::nth_element(begin, middle, end, IsInfoLess(dir));

    kdnode* node = *middle;

    const auto new_dir = (dir + 1) % DIM;
    node->left = insert(begin, middle, node, new_dir);
    node->right = insert(++middle, end, node, new_dir);

    return node;
}


struct SearchResult
{
    DistanceType dist_sq;
    SearchResult* next;
    int data;
};

bool operator < (const SearchResult& left, const SearchResult& right)
{
    return left.dist_sq < right.dist_sq;
}

bool operator < (const SearchResult& left, DistanceType right)
{
    return left.dist_sq < right;
}

bool operator < (DistanceType left, const SearchResult& right)
{
    return left < right.dist_sq;
}


class SearchResults
{
public:
    SearchResults() : m_full(false), m_dist_sq(0.), m_node(nullptr) {}
    bool isFull() const { return m_full; }
    double dist_sq() const { return m_dist_sq; }
    void insert(double dist_sq, kdnode* node)
    {
        if (!m_full || m_dist_sq > dist_sq)
        {
            m_dist_sq = dist_sq;
            m_node = node;
            m_full = true;
        }
    }
    kdnode* node() const { return m_node; }
private:
    bool m_full;
    double m_dist_sq;
    kdnode* m_node;
};


void kd_nearest_i(kdnode *node, const AttributeType *pos,
    SearchResults& result, DistanceType* sq_distances, DistanceType total_distance, int dir)
{
    kdnode *nearer_subtree, *farther_subtree;

    /* Decide whether to go left or right in the tree */
    const DistanceType dist = pos[dir] - node->pos[dir];
    if (dist <= 0) {
        nearer_subtree = node->left;
        farther_subtree = node->right;
    }
    else {
        nearer_subtree = node->right;
        farther_subtree = node->left;
    }

    const auto new_dir = (dir + 1) % DIM;

    if (nearer_subtree) {
        /* Recurse down into nearer subtree */
        kd_nearest_i(nearer_subtree, pos, result, sq_distances, total_distance, new_dir);
    }

    const auto sq_dist = SQ(dist);
    total_distance += sq_dist - sq_distances[dir];
    if (!result.isFull() || total_distance < result.dist_sq())
    {
        /* Check the distance of the point at the current node, compare it with our bests so far */
        //double dist_sq = sq_dist + SQ(node->pos[1 - dir] - pos[1 - dir]);
        DistanceType dist_sq = 0;
        for (int i = 0; i < DIM; ++i)
            dist_sq += SQ(node->pos[i] - pos[i]);

        result.insert(dist_sq, node);

        if (farther_subtree)
        {
            auto save_dist = sq_distances[dir];
            sq_distances[dir] = sq_dist;

            /* Recurse down into farther subtree */
            kd_nearest_i(farther_subtree, pos, result, sq_distances, total_distance, new_dir);

            sq_distances[dir] = save_dist;
        }
    }
}


void kd_nearest_i_nearer_subtree(kdnode *node, const AttributeType *pos,
    SearchResults& result, bool* flags,
    int dir)
{
    kdnode *nearer_subtree, *farther_subtree;
    int flagIdx;

    /* Decide whether to go left or right in the tree */
    const DistanceType dist = pos[dir] - node->pos[dir];
    if (dist <= 0) {
        nearer_subtree = node->left;
        farther_subtree = node->right;
        flagIdx = dir * 2;
    }
    else {
        nearer_subtree = node->right;
        farther_subtree = node->left;
        flagIdx = dir * 2 + 1;
    }

    const auto new_dir = (dir + 1) % DIM;

    if (nearer_subtree) {
        /* Recurse down into nearer subtree */
        kd_nearest_i_nearer_subtree(nearer_subtree, pos, result, flags, new_dir);
    }

    if (flags[flagIdx])
        return;

    const auto sq_dist = SQ(dist);
    if (!result.isFull() || sq_dist < result.dist_sq())
    {
        //if (node->pos != pos)
        {
            /* Check the distance of the point at the current node, compare it with our bests so far */
            //double dist_sq = sq_dist + SQ(node->pos[1 - dir] - pos[1 - dir]);
            DistanceType dist_sq = 0;
            for (int i = 0; i < DIM; ++i)
                dist_sq += SQ(node->pos[i] - pos[i]);

            result.insert(dist_sq, node);
        }

        if (farther_subtree)
        {
            DistanceType sq_distances[DIM]{ };
            //sq_distances[1 - dir] = 0;
            sq_distances[dir] = sq_dist;

            /* Recurse down into farther subtree */
            kd_nearest_i(farther_subtree, pos, result, sq_distances, sq_dist, new_dir);
        }
    }
    else
        flags[flagIdx] = true;
}
