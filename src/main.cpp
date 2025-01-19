#include "bvh.h"

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