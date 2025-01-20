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

    Mesh() {}

    Mesh(const char *scene_path) {
        load_scene(scene_path);
    }

    void load_scene(const char *scene_path) {
        Assimp::Importer importer;
        const aiScene *scene = importer.ReadFile(scene_path, aiProcess_Triangulate | aiProcess_JoinIdenticalVertices);
        if(!scene || scene->mFlags & AI_SCENE_FLAGS_INCOMPLETE || !scene->mRootNode) {
            cout << "Assimp error: " << importer.GetErrorString() << endl;
            exit(1);
        }

        vertices.clear();
        faces.clear();

        for (int mesh_i = 0; mesh_i < scene->mNumMeshes; mesh_i++) {
            aiMesh *ai_mesh = scene->mMeshes[mesh_i];

            for (int vertex_i = 0; vertex_i < ai_mesh->mNumVertices; vertex_i++) {
                aiVector3D vertex = ai_mesh->mVertices[vertex_i];
                vertices.push_back(glm::vec3(vertex.x, vertex.y, vertex.z));
            }

            for (int face_i = 0; face_i < ai_mesh->mNumFaces; face_i++) {
                aiFace face = ai_mesh->mFaces[face_i];
                for (int index_i = 0; index_i < face.mNumIndices; index_i++) {
                    faces.push_back({face.mIndices[0], face.mIndices[1], face.mIndices[2]});
                }
            }
        }
    }
};


std::tuple<bool, float, float> // mask, t_enter, t_exit
ray_box_intersection(const glm::vec3 &o, const glm::vec3 &d, const glm::vec3 &min, const glm::vec3 &max);


struct BVHNode {
    glm::vec3 min, max;
    BVHNode *left, *right;
    std::vector<Face> faces;

    BVHNode() : left(nullptr), right(nullptr) {}

    int is_leaf() const {
        return left == nullptr && right == nullptr;
    }

    int depth() const {
        if (is_leaf()) {
            return 0;
        }
        return 1 + std::max(left->depth(), right->depth());
    }

    int n_nodes() const {
        int n = 1;
        if (left  != nullptr) n += left ->n_nodes();
        if (right != nullptr) n += right->n_nodes();
        return n;
    }

    bool inside(glm::vec3 point) const {
        return point.x >= min.x && point.x <= max.x &&
               point.y >= min.y && point.y <= max.y &&
               point.z >= min.z && point.z <= max.z;
    }

    ~BVHNode() {
        if (left != nullptr) {
            delete left;
        }
        if (right != nullptr) {
            delete right;
        }
    }

    // save leaves as boxes in .obj file
    void save_as_obj(const std::string& filename);
};


struct BVH {
    Mesh mesh;
    BVHNode *root;

    BVH() : root() {}

    void load_scene(const char *path) {
        mesh = Mesh(path);
    }

    void build_bvh(int depth); // inits root and grows bvh
    void grow_bvh(BVHNode *node, int depth); // recursive function to grow bvh

    void save_as_obj(const std::string& filename) {
        root->save_as_obj(filename);
    }
    
    std::tuple<bool, int, float, float> // mask, leaf index, t_enter, t_exit
    intersect_leaves(const glm::vec3& o, const glm::vec3& d); // calls intersect

    std::tuple<bool, int, float, float> // mask, leaf index, t_enter, t_exit
    intersect(const BVHNode *node, const glm::vec3& o, const glm::vec3& d); // recursive bvh traversal

    ~BVH() {
        if (root != nullptr) {
            delete root;
        }
    }
};
