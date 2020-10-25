#pragma once
#include "ray_tracing.h"
#include "scene.h"

struct Node {
    int index = 0;
    bool isLeaf;
    AxisAlignedBox boundingbox;
    int level;
    std::vector<int> children;
    std::vector<Triangle> triangles;
};

class BoundingVolumeHierarchy {

    std::vector<Node> nodes;
    //triangles??
    //tree

public:
    BoundingVolumeHierarchy(Scene* pScene);

    // Use this function to visualize your BVH. This can be useful for debugging.
    void debugDraw(int level);
    int numLevels() const;
    AxisAlignedBox calculateAAB(Mesh& mesh);
    Node constructNode(Mesh mesh, int level);

    // Return true if something is hit, returns false otherwise.
    // Only find hits if they are closer than t stored in the ray and the intersection
    // is on the correct side of the origin (the new t >= 0).
    bool intersect(Ray& ray, HitInfo& hitInfo) const;

private:
    Scene* m_pScene;
};
