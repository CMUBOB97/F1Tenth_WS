/**
 * @file kd_tree.h
 * @brief KD-Tree implementation
 *
 * @cite Adapted from: https://github.com/crvs/KDTree/blob/master/KDTree.cpp
 * @author Eric Jenny (ejenny@andrew.cmu.edu)
 */

#ifndef KD_TREE_H
#define KD_TREE_H

#include <vector>
#include <math.h>
#include "planner.h"

struct kd_node
{
    const double *pos;
    int id;
    int depth;
    kd_node *left, *right;
};

class KDTree
{

public:
    KDTree(int dim, double *pos)
    {
        this->dim = dim;
        root = new_node(pos, 0, 0);
    }

    ~KDTree()
    {
        delete_node(root);
    }

    void insert(double *pos, int id)
    {
        root = insert(root, pos, id, 0);
    }

    kd_node *nearest(const double *pos)
    {
        return nearest(root, pos, root, distance(pos, root->pos, dim));
    }

    std::vector<std::pair<int, double>> radius_search(const double *pos, double radius)
    {
        std::vector<std::pair<int, double>> ids;
        radius_search(root, pos, radius, ids);
        return ids;
    }

    int get_max_depth()
    {
        return max_depth;
    }

private:
    kd_node *root;
    int dim;
    int max_depth;

    kd_node *insert(kd_node *node, double *pos, int id, int depth)
    {
        if (node == NULL)
        {
            if (depth > max_depth)
            {
                max_depth = depth;
            }
            return new_node(pos, id, depth);
        }

        int cd = depth % dim;
        if (pos[cd] < node->pos[cd])
        {
            node->left = insert(node->left, pos, id, depth + 1);
        }
        else
        {
            node->right = insert(node->right, pos, id, depth + 1);
        }

        return node;
    }
    // kd_node *remove(kd_node *node, double *pos, int depth);
    kd_node *nearest(kd_node *node, const double *pos, kd_node *best, double best_dist)
    {
        // Check if we have hit an "empty leaf" (end of tree)
        if (node == NULL)
        {
            return NULL;
        }

        double dist = distance(pos, node->pos, dim);
        if (dist < best_dist)
        {
            best_dist = dist;
            best = node;
        }

        int cd = node->depth % dim;
        kd_node *nearer = NULL;
        kd_node *further = NULL;
        if (pos[cd] < node->pos[cd])
        {
            nearer = node->left;
            further = node->right;
        }
        else
        {
            nearer = node->right;
            further = node->left;
        }

        // Check the branch on the same side as the point first
        kd_node *temp = nearest(nearer, pos, best, best_dist);
        double temp_dist;
        if (temp != NULL)
        {
            temp_dist = distance(pos, temp->pos, dim);
            if (temp_dist < best_dist)
            {
                best_dist = temp_dist;
                best = temp;
            }
        }

        // Check the other side if the plane is closer than the best so far
        double plane_dist = fabs(pos[cd] - node->pos[cd]);
        if (best_dist > plane_dist)
        {
            temp = nearest(further, pos, best, best_dist);
            if (temp != NULL)
            {
                temp_dist = distance(pos, temp->pos, dim);
                if (temp_dist < best_dist)
                {
                    best_dist = temp_dist;
                    best = temp;
                }
            }
        }

        return best;
    }

    void radius_search(kd_node *node, const double *pos, double radius, std::vector<std::pair<int, double>> &ids)
    {
        if (node == NULL)
        {
            return;
        }

        double dist = distance(pos, node->pos, dim);
        if (dist < radius)
        {
            ids.push_back(std::make_pair(node->id, dist));
        }

        int cd = node->depth % dim;
        kd_node *nearer = NULL;
        kd_node *further = NULL;
        if (pos[cd] < node->pos[cd])
        {
            nearer = node->left;
            further = node->right;
        }
        else
        {
            nearer = node->right;
            further = node->left;
        }

        radius_search(nearer, pos, radius, ids);

        if (fabs(pos[cd] - node->pos[cd]) < radius)
        {
            radius_search(further, pos, radius, ids);
        }

        return;
    }

    kd_node *new_node(const double *pos, int id, int depth)
    {
        kd_node *node = new kd_node;
        node->pos = pos;
        node->id = id;
        node->depth = depth;
        node->left = NULL;
        node->right = NULL;
        return node;
    }

    void delete_node(kd_node *node)
    {
        if (node->left != NULL)
            delete_node(node->left);
        if (node->right != NULL)
            delete_node(node->right);
        delete node;
    }
};

#endif // KD_TREE_H