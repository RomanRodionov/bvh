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

using std::cin, std::cout, std::endl;


struct Face {
    unsigned v1, v2, v3;
    Face(unsigned v1, unsigned v2, unsigned v3) : v1(v1), v2(v2), v3(v3) {}
    unsigned operator[](unsigned i) const {
        switch (i) {
            case 0: return v1;
            case 1: return v2;
            case 2: return v3;
            default: return 0;
        }
    }
};


struct Mesh {
    std::vector<glm::vec3> vertices;
    std::vector<Face> faces;
};


Mesh get_mesh(const char *scene_path) {
    Assimp::Importer importer;
    const aiScene *scene = importer.ReadFile(scene_path, aiProcess_Triangulate | aiProcess_JoinIdenticalVertices);
    if(!scene || scene->mFlags & AI_SCENE_FLAGS_INCOMPLETE || !scene->mRootNode) {
        cout << "Assimp error: " << importer.GetErrorString() << endl;
    }

    Mesh mesh;

    for (int mesh_i = 0; mesh_i < scene->mNumMeshes; mesh_i++) {
        aiMesh *ai_mesh = scene->mMeshes[mesh_i];

        for (int vertex_i = 0; vertex_i < ai_mesh->mNumVertices; vertex_i++) {
            aiVector3D vertex = ai_mesh->mVertices[vertex_i];
            mesh.vertices.push_back(glm::vec3(vertex.x, vertex.y, vertex.z));
        }

        for (int face_i = 0; face_i < ai_mesh->mNumFaces; face_i++) {
            aiFace face = ai_mesh->mFaces[face_i];
            for (int index_i = 0; index_i < face.mNumIndices; index_i++) {
                mesh.faces.push_back({face.mIndices[0], face.mIndices[1], face.mIndices[2]});
            }
        }
    }

    return mesh;
}


struct BVHNode {
    glm::vec3 min;
    glm::vec3 max;
    BVHNode *left = nullptr;
    BVHNode *right = nullptr;
    std::vector<Face> faces;

    int is_leaf() const {
        return left == nullptr && right == nullptr;
    }

    int depth() const {
        if (is_leaf()) {
            return 0;
        }
        return 1 + std::max(left->depth(), right->depth());
    }

    int n_faces() const {
        return faces.size();
    }

    ~BVHNode() {
        if (left != nullptr) {
            delete left;
        }
        if (right != nullptr) {
            delete right;
        }
    }

    int n_nodes() const {
        if (is_leaf()) {
            return 1;
        }
        return 1 + left->n_nodes() + right->n_nodes();
    }

    bool inside(glm::vec3 point) const {
        return point.x >= min.x && point.x <= max.x &&
               point.y >= min.y && point.y <= max.y &&
               point.z >= min.z && point.z <= max.z;
    }
};


BVHNode *get_root_BVH(const Mesh &mesh) {
    BVHNode *root = new BVHNode;

    root->min = glm::vec3(FLT_MAX);
    root->max = glm::vec3(-FLT_MAX);

    for (int i = 0; i < mesh.vertices.size(); i++) {
        glm::vec3 vertex = mesh.vertices[i];
        root->min = glm::min(root->min, vertex);
        root->max = glm::max(root->max, vertex);
    }

    root->faces = mesh.faces;

    return root;
}


void grow_bvh(BVHNode *node, const Mesh &mesh, int depth) {
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
        grow_bvh(left, mesh, depth - 1);
    } else {
        delete left;
    }

    if (right->faces.size() > 0) {
        node->right = right;
        grow_bvh(right, mesh, depth - 1);
    } else {
        delete right;
    }
}


void saveBVHAsOBJ(const std::string& filename, const BVHNode* root) {
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
    traverseAndWrite(root);

    outFile.close();
    std::cout << "BVH saved as .obj file: " << filename << std::endl;
}


struct Ray {
    glm::vec3 origin;
    glm::vec3 direction;
};


glm::vec3 ray_box_intersection(const Ray &ray, const glm::vec3 &min, const glm::vec3 &max) {
    glm::vec3 t1 = (min - ray.origin) / ray.direction;
    glm::vec3 t2 = (max - ray.origin) / ray.direction;

    glm::vec3 tmin = glm::min(t1, t2);
    glm::vec3 tmax = glm::max(t1, t2);

    float t_enter = glm::max(tmin.x, glm::max(tmin.y, tmin.z));
    float t_exit = glm::min(tmax.x, glm::min(tmax.y, tmax.z));

    if (t_exit < 0 || t_enter > t_exit) {
        return glm::vec3(FLT_MAX, FLT_MAX, 0);
    }

    return glm::vec3(t_enter, t_exit, 1);
}


glm::vec3 intersect(const Ray &ray, const BVHNode *node) {
    if (node->inside(ray.origin)) {
        // later
        return glm::vec3(FLT_MAX, FLT_MAX, 0);
    }

    glm::vec3 intersection = ray_box_intersection(ray, node->min, node->max);
    if (intersection.z == 0) {
        return glm::vec3(FLT_MAX, FLT_MAX, 0);
    }

    if (node->is_leaf()) {
        // Check intersection with faces
        return intersection;
    }

    glm::vec3 left = intersect(ray, node->left);
    glm::vec3 right = intersect(ray, node->right);

    if (!left.z && !right.z) {
        return glm::vec3(FLT_MAX, FLT_MAX, 0);
    }

    if (!left.z) {
        return right;
    }

    if (!right.z) {
        return left;
    }

    if (left.x < right.x) {
        return left;
    }

    return right;
}


int main() {
    Mesh mesh = get_mesh("cube.fbx");
    cout << "Number of vertices: " << mesh.vertices.size() << endl;
    cout << "Number of faces: " << mesh.faces.size() << endl;

    BVHNode *root = get_root_BVH(mesh);
    cout << "Root min: " << root->min.x << " " << root->min.y << " " << root->min.z << endl;
    cout << "Root max: " << root->max.x << " " << root->max.y << " " << root->max.z << endl;

    grow_bvh(root, mesh, 1);
    cout << "Depth: " << root->depth() << endl;
    cout << "Number of nodes: " << root->n_nodes() << endl;

    saveBVHAsOBJ("bvh.obj", root);

    Ray ray = {glm::vec3(10, 0, 0), glm::vec3(-1, 0, 0)};
    glm::vec3 intersection = intersect(ray, root);
    cout << "Intersection: " << intersection.x << " " << intersection.y << endl;

    return 0;
}