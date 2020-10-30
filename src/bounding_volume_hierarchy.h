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
};

class BoundingVolumeHierarchy {

public:
    BoundingVolumeHierarchy(Scene* pScene);

    // Use this function to visualize your BVH. This can be useful for debugging.
    void debugDraw(int level);
    int numLevels() const;
    void buildTree(int currentLevel, std::vector<Tri>& triangles);
    void splitByBinning(std::vector<Tri>& triangles, int currentLevel, int index);
    void appendChildren(AxisAlignedBox& first, AxisAlignedBox& second, std::vector<Tri>& arr, int currentLevel, int index);
    float eval(AxisAlignedBox& first, AxisAlignedBox& second, Node& nodeToSplit, std::vector<Tri>& arr);
    bool isInsideAABB(AxisAlignedBox& aabb, Tri& t);
    AxisAlignedBox defineAABB(std::vector<int>& ind, std::vector<Tri>& arr);
    AxisAlignedBox calculateAAB(Mesh& mesh);
    AxisAlignedBox calculateParentAAB(Scene* pScene);
    Node constructNode(Mesh mesh, int level);
    Node constructParent(Scene* pScene, std::vector<Tri>& triangles);

    // Return true if something is hit, returns false otherwise.
    // Only find hits if they are closer than t stored in the ray and the intersection
    // is on the correct side of the origin (the new t >= 0).
    bool intersect(Ray& ray, HitInfo& hitInfo) const;

private:
    std::vector<Node> nodes;
    Scene* m_pScene;
};

