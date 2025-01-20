// https://learnopengl.com/Model-Loading/Model

#include <assimp/Importer.hpp>
#include <assimp/scene.h>
#include <assimp/postprocess.h>

#include <glm/glm.hpp>

#include <iostream>
#include <vector>
#include <fstream>
#include <string>
#include <functional>
#include <unordered_set>
#include <algorithm>

#include "bvh.h"


void BVH::build_bvh(int depth) {
    root = new BVHNode;

    root->min = glm::vec3(FLT_MAX);
    root->max = glm::vec3(-FLT_MAX);

    for (int i = 0; i < mesh.vertices.size(); i++) {
        glm::vec3 vertex = mesh.vertices[i];
        root->min = glm::min(root->min, vertex);
        root->max = glm::max(root->max, vertex);
    }

    root->faces = mesh.faces;

    grow_bvh(root, depth);
}


void BVH::grow_bvh(BVHNode *node, int depth) {
    if (depth <= 0 || node->faces.size() <= 1) {
        cout << "Child node min: " << node->min.x << " " << node->min.y << " " << node->min.z << endl;
        cout << "Child node max: " << node->max.x << " " << node->max.y << " " << node->max.z << endl;
        cout << endl;

        return;
    }

    glm::vec3 size = node->max - node->min;
    int axis = 0;
    if (size.y > size.x && size.y > size.z) {
        axis = 1;
    }
    if (size.z > size.y && size.z > size.x) {
        axis = 2;
    }

    std::vector<Face> faces_sorted = node->faces;
    std::sort(faces_sorted.begin(), faces_sorted.end(), [&](Face a, Face b) {
        glm::vec3 a_min = glm::vec3(FLT_MAX);
        glm::vec3 a_max = glm::vec3(-FLT_MAX);
        glm::vec3 b_min = glm::vec3(FLT_MAX);
        glm::vec3 b_max = glm::vec3(-FLT_MAX);

        for (int i = 0; i < 3; i++) {
            glm::vec3 a_vertex = mesh.vertices[a[i]];
            a_min = glm::min(a_min, a_vertex);
            a_max = glm::max(a_max, a_vertex);

            glm::vec3 b_vertex = mesh.vertices[b[i]];
            b_min = glm::min(b_min, b_vertex);
            b_max = glm::max(b_max, b_vertex);
        }

        return a_min[axis] < b_min[axis];
    });

    int split_i = faces_sorted.size() / 2;

    BVHNode *left = new BVHNode;
    left->min = glm::vec3(FLT_MAX);
    left->max = glm::vec3(-FLT_MAX);

    BVHNode *right = new BVHNode;
    right->min = glm::vec3(FLT_MAX);
    right->max = glm::vec3(-FLT_MAX);

    for (int i = 0; i < split_i; i++) {
        Face face = faces_sorted[i];
        for (int j = 0; j < 3; j++) {
            glm::vec3 vertex = mesh.vertices[face[j]];
            left->min = glm::min(left->min, vertex);
            left->max = glm::max(left->max, vertex);
        }
        left->faces.push_back(face);
    }

    for (int i = split_i; i < faces_sorted.size(); i++) {
        Face face = faces_sorted[i];
        for (int j = 0; j < 3; j++) {
            glm::vec3 vertex = mesh.vertices[face[j]];
            right->min = glm::min(right->min, vertex);
            right->max = glm::max(right->max, vertex);
        }
        right->faces.push_back(face);
    }

    cout << "Current depth is " << depth << endl;
    cout << "Parent node min: " << node->min.x << " " << node->min.y << " " << node->min.z << endl;
    cout << "Parent node max: " << node->max.x << " " << node->max.y << " " << node->max.z << endl;
    float split_coord = mesh.vertices[faces_sorted[split_i].v1][axis];
    cout << "Splitting at " << split_coord << " on axis " << axis << endl;
    cout << "Number of faces left: " << left->faces.size() << endl;
    cout << "Number of faces right: " << right->faces.size() << endl;
    cout << endl;

    if (left->faces.size() > 0) {
        node->left = left;
        grow_bvh(left, depth - 1);
    } else {
        delete left;
    }

    if (right->faces.size() > 0) {
        node->right = right;
        grow_bvh(right, depth - 1);
    } else {
        delete right;
    }
}


std::tuple<bool, int, float, float> // mask, leaf index, t_enter, t_exit
BVH::intersect_leaves(const glm::vec3& o, const glm::vec3& d) {
    return intersect(root, o, d);
}

std::tuple<bool, int, float, float> // mask, leaf index, t_enter, t_exit
BVH::intersect(const BVHNode *node, const glm::vec3& o, const glm::vec3& d) {
    if (node->inside(o)) {
        return {false, -1, 0, 0};
    }

    auto [mask, t1, t2] = ray_box_intersection(o, d, node->min, node->max);
    if (!mask) {
        return {false, -1, 0, 0};
    }

    if (node->is_leaf()) {
        return {true, -1, t1, t2};
    }

    auto [mask_l, leaf_l, t1_l, t2_l] = intersect(node->left, o, d);
    auto [mask_r, leaf_r, t1_r, t2_r] = intersect(node->right, o, d);

    if (!mask_l && !mask_r) {
        return {false, -1, 0, 0};
    }

    if (!mask_l) {
        return {mask_r, leaf_r, t1_r, t2_r};
    }

    if (!mask_r) {
        return {mask_l, leaf_l, t1_l, t2_l};
    }

    if (t1_l < t1_r) {
        return {mask_l, leaf_l, t1_l, t2_l};
    }

    return {mask_r, leaf_r, t1_r, t2_r};
}


void BVHNode::save_as_obj(const std::string& filename) {
    std::ofstream outFile(filename);

    if (!outFile.is_open()) {
        std::cerr << "Failed to open file: " << filename << std::endl;
        return;
    }

    int vertexOffset = 1; // Keeps track of vertex indices for face definitions

    // Lambda function to write a cube's vertices and faces
    auto writeCube = [&](const glm::vec3& min, const glm::vec3& max) {
        // Write vertices for the cube
        outFile << "v " << min.x << " " << min.y << " " << min.z << "\n"; // Bottom-left-front
        outFile << "v " << max.x << " " << min.y << " " << min.z << "\n"; // Bottom-right-front
        outFile << "v " << max.x << " " << max.y << " " << min.z << "\n"; // Top-right-front
        outFile << "v " << min.x << " " << max.y << " " << min.z << "\n"; // Top-left-front
        outFile << "v " << min.x << " " << min.y << " " << max.z << "\n"; // Bottom-left-back
        outFile << "v " << max.x << " " << min.y << " " << max.z << "\n"; // Bottom-right-back
        outFile << "v " << max.x << " " << max.y << " " << max.z << "\n"; // Top-right-back
        outFile << "v " << min.x << " " << max.y << " " << max.z << "\n"; // Top-left-back

        // Write faces for the cube (1-based indices)
        outFile << "f " << vertexOffset + 0 << " " << vertexOffset + 1 << " " << vertexOffset + 2 << " " << vertexOffset + 3 << "\n"; // Front
        outFile << "f " << vertexOffset + 4 << " " << vertexOffset + 5 << " " << vertexOffset + 6 << " " << vertexOffset + 7 << "\n"; // Back
        outFile << "f " << vertexOffset + 0 << " " << vertexOffset + 1 << " " << vertexOffset + 5 << " " << vertexOffset + 4 << "\n"; // Bottom
        outFile << "f " << vertexOffset + 3 << " " << vertexOffset + 2 << " " << vertexOffset + 6 << " " << vertexOffset + 7 << "\n"; // Top
        outFile << "f " << vertexOffset + 0 << " " << vertexOffset + 4 << " " << vertexOffset + 7 << " " << vertexOffset + 3 << "\n"; // Left
        outFile << "f " << vertexOffset + 1 << " " << vertexOffset + 5 << " " << vertexOffset + 6 << " " << vertexOffset + 2 << "\n"; // Right

        vertexOffset += 8; // Move to the next set of vertices
    };

    // Recursive function to traverse the BVH and write leaf nodes
    std::function<void(const BVHNode*)> traverseAndWrite = [&](const BVHNode* node) {
        if (!node) return;

        if (!node->left && !node->right) {
            // Leaf node: write its bounding box as a cube
            writeCube(node->min, node->max);
        }

        // Recursively traverse children
        traverseAndWrite(node->left);
        traverseAndWrite(node->right);
    };

    // Start traversal and writing
    traverseAndWrite(this);

    outFile.close();
    std::cout << "BVH saved as .obj file: " << filename << std::endl;
}


std::tuple<bool, float, float> // mask, t_enter, t_exit
ray_box_intersection(const glm::vec3 &o, const glm::vec3 &d, const glm::vec3 &min, const glm::vec3 &max) {
    glm::vec3 t1 = (min - o) / d;
    glm::vec3 t2 = (max - o) / d;

    glm::vec3 tmin = glm::min(t1, t2);
    glm::vec3 tmax = glm::max(t1, t2);

    float t_enter = glm::max(tmin.x, glm::max(tmin.y, tmin.z));
    float t_exit = glm::min(tmax.x, glm::min(tmax.y, tmax.z));

    if (t_exit < 0 || t_enter > t_exit) {
        return {false, 0, 0};
    }

    return {true, t_enter, t_exit};
}