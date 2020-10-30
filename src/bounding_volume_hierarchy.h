#pragma once
#include "ray_tracing.h"
#include "scene.h"

struct Node {
    bool isLeaf;
    AxisAlignedBox boundingbox;
    std::vector<int> indeces;
};

struct Tri {
    glm::vec3 a,b,c;
    Material material;
};

class BoundingVolumeHierarchy {

public:
    BoundingVolumeHierarchy(Scene* pScene);

    // Use this function to visualize your BVH. This can be useful for debugging.
    void debugDraw(int level);
    int numLevels() const;
    void buildTree(int currentLevel);
    void splitByBinning(int currentLevel, int index);
    void appendChildren(AxisAlignedBox& first, AxisAlignedBox& second, int currentLevel, int index);
    float eval(AxisAlignedBox& first, AxisAlignedBox& second, Node& nodeToSplit);
    bool isInsideAABB(AxisAlignedBox& aabb, Tri& t);
    AxisAlignedBox defineAABB(std::vector<int>& ind);
    AxisAlignedBox calculateAAB(Mesh& mesh);
    AxisAlignedBox calculateParentAAB(Scene* pScene);
    Node constructNode(Mesh mesh, int level);
    Node constructParent(Scene* pScene);

    // Return true if something is hit, returns false otherwise.
    // Only find hits if they are closer than t stored in the ray and the intersection
    // is on the correct side of the origin (the new t >= 0).
    bool intersect(Ray& ray, HitInfo& hitInfo) const;

private:
    std::vector<Tri> triangles;
    std::vector<Node> nodes;
    Scene* m_pScene;
};

