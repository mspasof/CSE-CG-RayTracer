#include "bounding_volume_hierarchy.h"
#include "draw.h"

Node root;

BoundingVolumeHierarchy::BoundingVolumeHierarchy(Scene* pScene)
    : m_pScene(pScene)
{
    for (Mesh m : pScene->meshes) {
        Node root;
        root.boundingbox = calculateAAB(m);
        BoundingVolumeHierarchy::nodes.push_back(root);
    }
    // as an example of how to iterate over all meshes in the scene, look at the intersect method below
}

//Node constructNode(Mesh mesh, int level) {
//
//}
//
//Node constructTree(Mesh& mesh, int level) {
//    //return Node(mesh, level);
//}

AxisAlignedBox BoundingVolumeHierarchy::calculateAAB(Mesh& mesh) {
    AxisAlignedBox box;

    if (mesh.vertices.size() == 0) return box;

    float minx = mesh.vertices[0].p.x;
    float miny = mesh.vertices[0].p.y;
    float minz = mesh.vertices[0].p.z;
    float maxx = mesh.vertices[0].p.x;
    float maxy = mesh.vertices[0].p.y;
    float maxz = mesh.vertices[0].p.z;

    for (const auto& v : mesh.vertices) {
        if (v.p.x < minx) minx = v.p.x;
        if (v.p.y < miny) miny = v.p.y;
        if (v.p.z < minz) minz = v.p.z;
        if (v.p.x > maxx) maxx = v.p.x;
        if (v.p.y > maxy) maxy = v.p.y;
        if (v.p.z > maxz) maxz = v.p.z;
    }

    box.lower = glm::vec3{ minx, miny, minz };
    box.upper = glm::vec3{ maxx, maxy, maxz };
    return box;
}


// Use this function to visualize your BVH. This can be useful for debugging. Use the functions in
// draw.h to draw the various shapes. We have extended the AABB draw functions to support wireframe
// mode, arbitrary colors and transparency.
void BoundingVolumeHierarchy::debugDraw(int level)
{
    // Draw the AABB as a transparent green box.
    //AxisAlignedBox aabb{ glm::vec3(-0.05f), glm::vec3(0.05f, 1.05f, 1.05f) };
    //drawShape(aabb, DrawMode::Filled, glm::vec3(0.0f, 1.0f, 0.0f), 0.2f);

    // Draw the AABB as a (white) wireframe box.
    // AxisAlignedBox aabb { glm::vec3(-0.05f), glm::vec3(0.05f, 1.05f, 1.05f) };
    //drawAABB(aabb, DrawMode::Wireframe);
    //drawAABB(aabb, DrawMode::Filled, glm::vec3(0.05f, 1.0f, 0.05f), 0.1);
    for (Node node : BoundingVolumeHierarchy::nodes) {
        drawAABB(node.boundingbox, DrawMode::Filled, glm::vec3(0.0f, 1.0f, 0.0f), 0.1);
    }
}

int BoundingVolumeHierarchy::numLevels() const
{
    return 5;
}

// Return true if something is hit, returns false otherwise. Only find hits if they are closer than t stored
// in the ray and if the intersection is on the correct side of the origin (the new t >= 0). Replace the code
// by a bounding volume hierarchy acceleration structure as described in the assignment. You can change any
// file you like, including bounding_volume_hierarchy.h .
bool BoundingVolumeHierarchy::intersect(Ray& ray, HitInfo& hitInfo) const
{
    bool hit = false;
    // Intersect with all triangles of all meshes.
    for (const auto& mesh : m_pScene->meshes) {
        for (const auto& tri : mesh.triangles) {
            const auto v0 = mesh.vertices[tri[0]];
            const auto v1 = mesh.vertices[tri[1]];
            const auto v2 = mesh.vertices[tri[2]];
            if (intersectRayWithTriangle(v0.p, v1.p, v2.p, ray, hitInfo)) {
                hitInfo.material = mesh.material;
                hit = true;
            }
        }
    }
    // Intersect with spheres.
    for (const auto& sphere : m_pScene->spheres)
        hit |= intersectRayWithShape(sphere, ray, hitInfo);
    return hit;
}
