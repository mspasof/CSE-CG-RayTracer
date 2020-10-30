#include "bounding_volume_hierarchy.h"
#include "draw.h"
#include "math.h" 
#include <iostream>
#include <stack>

int maxLevel = 8;
const int bins = 8;

BoundingVolumeHierarchy::BoundingVolumeHierarchy(Scene* pScene)
    : m_pScene(pScene)
{
    for(const auto& mesh : pScene->meshes) {
        for (const auto& tri : mesh.triangles) {
            Tri triangleToPush;
            triangleToPush.a = mesh.vertices[tri[0]].p;
            triangleToPush.b = mesh.vertices[tri[1]].p;
            triangleToPush.c = mesh.vertices[tri[2]].p;
            triangleToPush.material = mesh.material;
            triangles.push_back(triangleToPush);    
        }
    }
    if(triangles.size() < 100) maxLevel = 0;
    nodes.push_back(constructParent(pScene));
    buildTree(1);
}

void BoundingVolumeHierarchy::buildTree(int currentLevel) {
    if(currentLevel == (maxLevel + 1)) return;
    for(int i = int((pow(2, (currentLevel-1)) - 1)); i < int((pow(2, currentLevel) - 1)); i++) {
        splitByBinning(currentLevel, i);
    }
   buildTree(++currentLevel);
}

void BoundingVolumeHierarchy::splitByBinning(int currentLevel, int index) { 
    Node nodeToSplit = BoundingVolumeHierarchy::nodes.at(index);
    float bestSplit = 1.0f;
    AxisAlignedBox first;
    AxisAlignedBox second;
    AxisAlignedBox aabb = nodeToSplit.boundingbox;
    for(int i = 1; i < bins; i++) {
        float frac = float(i)/bins;
        first.lower = aabb.lower;
        first.upper =  glm::vec3((frac * (aabb.upper.x - aabb.lower.x) + aabb.lower.x), aabb.upper.y, aabb.upper.z);

        second.lower = glm::vec3((frac * (aabb.upper.x - aabb.lower.x) + aabb.lower.x), aabb.lower.y, aabb.lower.z);
        second.upper = aabb.upper; 
        bestSplit = std::min(bestSplit, eval(first, second, nodeToSplit));
        first.lower = aabb.lower;
        first.upper =  glm::vec3(aabb.upper.x, (frac * (aabb.upper.y - aabb.lower.y) + aabb.lower.y), aabb.upper.z);

        second.lower = glm::vec3(aabb.lower.x, (frac * (aabb.upper.y - aabb.lower.y) + aabb.lower.y) , aabb.lower.z);
        second.upper = aabb.upper; 
        bestSplit = std::min(bestSplit, eval(first, second, nodeToSplit));

        first.lower = aabb.lower;
        first.upper =  glm::vec3(aabb.upper.x, aabb.upper.y, (frac * (aabb.upper.z - aabb.lower.z) + aabb.lower.z));

        second.lower = glm::vec3(aabb.lower.x, aabb.lower.y, (frac * (aabb.upper.z - aabb.lower.z) + aabb.lower.z));
        second.upper = aabb.upper; 
        bestSplit = std::min(bestSplit, eval(first, second, nodeToSplit));
    }    
    if(bestSplit < 1.0f) {
        for(int i = 1; i < bins; i++) {
            float frac = float(i)/bins;
            first.lower = aabb.lower;
            first.upper =  glm::vec3((frac * (aabb.upper.x - aabb.lower.x) + aabb.lower.x), aabb.upper.y, aabb.upper.z);

            second.lower = glm::vec3((frac * (aabb.upper.x - aabb.lower.x) + aabb.lower.x), aabb.lower.y, aabb.lower.z);
            second.upper = aabb.upper; 

            if(eval(first, second, nodeToSplit) == bestSplit) {
                appendChildren(first, second, currentLevel, index);   
                break;
            }

            first.lower = aabb.lower;
            first.upper =  glm::vec3(aabb.upper.x, (frac * (aabb.upper.y - aabb.lower.y) + aabb.lower.y), aabb.upper.z);

            second.lower = glm::vec3(aabb.lower.x, (frac * (aabb.upper.y - aabb.lower.y) + aabb.lower.y) , aabb.lower.z);
            second.upper = aabb.upper; 
        
            if(eval(first, second, nodeToSplit) == bestSplit) {
                appendChildren(first, second, currentLevel, index);   
                break;
            }

            first.lower = aabb.lower;
            first.upper =  glm::vec3(aabb.upper.x, aabb.upper.y, (frac * (aabb.upper.z - aabb.lower.z) + aabb.lower.z));

            second.lower = glm::vec3(aabb.lower.x, aabb.lower.y, (frac * (aabb.upper.z - aabb.lower.z) + aabb.lower.z));
            second.upper = aabb.upper;  

            if(eval(first, second, nodeToSplit) == bestSplit) {
                appendChildren(first, second, currentLevel, index);   
                break;
            }
        }
    } else {
        Node same = nodes.at(index);
        nodes.push_back(same);
        nodes.push_back(same);
        nodes.at(index).isLeaf = false;
        nodes.at(index).indeces.clear();
        nodes.at(index).indeces.push_back(2*index + 1);
        nodes.at(index).indeces.push_back(2*index + 2);
    }
}

void BoundingVolumeHierarchy::appendChildren(AxisAlignedBox& first, AxisAlignedBox& second, int currentLevel, int index) {
    Node nodeToSplit = BoundingVolumeHierarchy::nodes.at(index);
    Node leftChild = Node();
    leftChild.boundingbox = first;
    Node rightChild = Node();
    rightChild.boundingbox = second;
    std::vector<int> childToSplit;
    for(int i = 0; i < nodeToSplit.indeces.size(); i++) {
        childToSplit.push_back(nodeToSplit.indeces.at(i));
    }
    int k = 0;
    for(int i = 0; i < nodeToSplit.indeces.size(); i++) {
        if(isInsideAABB(first, triangles.at(nodeToSplit.indeces.at(i)))) {
            leftChild.indeces.push_back(nodeToSplit.indeces.at(i));
            childToSplit.erase(childToSplit.begin() + i - k);
            k++;
        }
        else if(isInsideAABB(second, triangles.at(nodeToSplit.indeces.at(i)))) {
            rightChild.indeces.push_back(nodeToSplit.indeces.at(i));
            childToSplit.erase(childToSplit.begin() + i - k);
            k++;
        }
    }
    for(int i = 0; i < childToSplit.size(); i++) {
        leftChild.indeces.push_back(childToSplit.at(i));
    }
    AxisAlignedBox lower = defineAABB(leftChild.indeces);
    AxisAlignedBox upper = defineAABB(rightChild.indeces);

    leftChild.boundingbox = lower;
    leftChild.isLeaf = true;
    rightChild.boundingbox = upper;
    rightChild.isLeaf = true;
    nodes.at(index).isLeaf = false;
    nodes.at(index).indeces.clear();
    nodes.at(index).indeces.push_back(2*index + 1);
    nodes.at(index).indeces.push_back(2*index + 2);
    nodes.push_back(leftChild);
    nodes.push_back(rightChild);
}

float BoundingVolumeHierarchy::eval(AxisAlignedBox& first, AxisAlignedBox& second, Node& nodeToSplit) {
    float totalArea = (first.upper.x - first.lower.x) * (first.upper.y - first.lower.y) * (first.upper.z - first.lower.z) + (second.upper.x - second.lower.x) * (second.upper.y - second.lower.y) * (second.upper.z - second.lower.z);
    Node leftChild = Node();
    leftChild.boundingbox = first;
    Node rightChild = Node();
    rightChild.boundingbox = second;
    std::vector<int> childToSplit;
    for(int i = 0; i < nodeToSplit.indeces.size(); i++) {
        childToSplit.push_back(nodeToSplit.indeces.at(i));
    }
    int k = 0;
    for(int i = 0; i < nodeToSplit.indeces.size(); i++) {
        if(isInsideAABB(first, triangles.at(nodeToSplit.indeces.at(i)))) {
            leftChild.indeces.push_back(nodeToSplit.indeces.at(i));
            childToSplit.erase(childToSplit.begin() + (i - k));
            k++;
        }
        else if(isInsideAABB(second, triangles.at(nodeToSplit.indeces.at(i)))) {
            rightChild.indeces.push_back(nodeToSplit.indeces.at(i));
            childToSplit.erase(childToSplit.begin() + (i - k));
            k++;
        }
    }
    for(int i = 0; i < childToSplit.size(); i++) {
        leftChild.indeces.push_back(childToSplit.at(i));
    }
    AxisAlignedBox lower = defineAABB(leftChild.indeces);
    AxisAlignedBox upper = defineAABB(rightChild.indeces);
    float sumArea = (lower.upper.x - lower.lower.x) * (lower.upper.y - lower.lower.y) * (lower.upper.z - lower.lower.z) + (upper.upper.x - upper.lower.x) * (upper.upper.y - upper.lower.y) * (upper.upper.z - upper.lower.z);
    return sumArea/totalArea;
}

AxisAlignedBox BoundingVolumeHierarchy::defineAABB(std::vector<int>& ind) {
    AxisAlignedBox res;
    if(ind.empty()) return res;

    float xMin = triangles.at(ind.at(0)).a.x;
    float xMax = triangles.at(ind.at(0)).a.x;
    float yMin = triangles.at(ind.at(0)).a.y;
    float yMax = triangles.at(ind.at(0)).a.y;
    float zMin = triangles.at(ind.at(0)).a.z;
    float zMax = triangles.at(ind.at(0)).a.z;

    for(int i = 0; i < ind.size(); i++) {
        Tri t = triangles.at(ind.at(i));
        if(t.a.x < xMin) xMin = t.a.x;
        if(t.a.x > xMax) xMax = t.a.x;
        if(t.a.y < yMin) yMin = t.a.y;
        if(t.a.y > yMax) yMax = t.a.y;
        if(t.a.z < zMin) zMin = t.a.z;
        if(t.a.z > zMax) zMax = t.a.z;

        if(t.b.x < xMin) xMin = t.b.x;
        if(t.b.x > xMax) xMax = t.b.x;
        if(t.b.y < yMin) yMin = t.b.y;
        if(t.b.y > yMax) yMax = t.b.y;
        if(t.b.z < zMin) zMin = t.b.z;
        if(t.b.z > zMax) zMax = t.b.z;

        if(t.c.x < xMin) xMin = t.c.x;
        if(t.c.x > xMax) xMax = t.c.x;
        if(t.c.y < yMin) yMin = t.c.y;
        if(t.c.y > yMax) yMax = t.c.y;
        if(t.c.z < zMin) zMin = t.c.z;
        if(t.c.z > zMax) zMax = t.c.z;
    }
    res.lower = glm::vec3(xMin, yMin, zMin);
    res.upper = glm::vec3(xMax, yMax, zMax);
    return res;
}

bool BoundingVolumeHierarchy::isInsideAABB(AxisAlignedBox& aabb, Tri& t) {
    bool cond1 = (t.a.x >= aabb.lower.x && t.a.y >= aabb.lower.y && t.a.z >= aabb.lower.z) && (t.a.x <= aabb.upper.x && t.a.y <= aabb.upper.y && t.a.z <= aabb.upper.z);
    bool cond2 = (t.b.x >= aabb.lower.x && t.b.y >= aabb.lower.y && t.b.z >= aabb.lower.z) && (t.b.x <= aabb.upper.x && t.b.y <= aabb.upper.y && t.b.z <= aabb.upper.z);
    bool cond3 = (t.c.x >= aabb.lower.x && t.c.y >= aabb.lower.y && t.c.z >= aabb.lower.z) && (t.c.x <= aabb.upper.x && t.c.y <= aabb.upper.y && t.c.z <= aabb.upper.z);
    return (cond1 && cond2 && cond3);
}

Node BoundingVolumeHierarchy::constructParent(Scene* pScene) {
    Node node;
    node.boundingbox = calculateParentAAB(pScene);
    node.isLeaf = true;
    for(int i = 0; i < triangles.size(); i++) node.indeces.push_back(i);
    return node;
}

AxisAlignedBox BoundingVolumeHierarchy::calculateParentAAB(Scene* pScene) {
    AxisAlignedBox aabb;
    if(pScene->meshes.empty()) return aabb;
    aabb = calculateAAB(pScene->meshes.at(0));
    float xMin = aabb.lower.x;
    float xMax = aabb.upper.x;
    float yMin = aabb.lower.y;
    float yMax = aabb.upper.y;
    float zMin = aabb.lower.z;
    float zMax = aabb.upper.z;

    AxisAlignedBox compare;
    for(auto& mesh : pScene->meshes) {
        compare = calculateAAB(mesh);
        if(compare.lower.x < xMin) xMin = compare.lower.x;
        if(compare.upper.x > xMax) xMax = compare.upper.x;
        if(compare.lower.y < yMin) yMin = compare.lower.y;
        if(compare.upper.y > yMax) yMax = compare.upper.y;
        if(compare.lower.z < zMin) zMin = compare.lower.z;
        if(compare.upper.z > zMax) zMax = compare.upper.z; 
    }
    aabb.lower = glm::vec3(xMin, yMin, zMin);
    aabb.upper = glm::vec3(xMax, yMax, zMax);
    return aabb;
}

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

    for(int i = int((pow(2, level) - 1)); i < int((pow(2, (level+1)) - 1)); i++) {
        drawAABB(BoundingVolumeHierarchy::nodes.at(i).boundingbox, DrawMode::Filled, glm::vec3(0.5f, 1.0f, 0.5f), 0.1f);
    }

    // for (Node node : BoundingVolumeHierarchy::nodes) {
    //     drawAABB(node.boundingbox, DrawMode::Filled, glm::vec3(0.0f, 1.0f, 0.0f), 0.1);
    // }
}

int BoundingVolumeHierarchy::numLevels() const
{
    //return int(log2(BoundingVolumeHierarchy::nodes.size()));
    return maxLevel + 1;
}

// Return true if something is hit, returns false otherwise. Only find hits if they are closer than t stored
// in the ray and if the intersection is on the correct side of the origin (the new t >= 0). Replace the code
// by a bounding volume hierarchy acceleration structure as described in the assignment. You can change any
// file you like, including bounding_volume_hierarchy.h .
bool BoundingVolumeHierarchy::intersect(Ray& ray, HitInfo& hitInfo) const
{

    bool hit = false;
    if(maxLevel != 0) {
        float origT;
        std::stack<Node> s;
        Node checkValue;
        bool intersect;
        Triangle tri;
        Tri triangleToCheck;
        if(!nodes.empty()) {
            s.push(nodes[0]);
            while(!s.empty()) {
                checkValue = s.top();
                s.pop();
                origT = ray.t;
                intersect = intersectRayWithShape(checkValue.boundingbox, ray);
                if(intersect) ray.t = origT;
                if(intersect && !checkValue.isLeaf) {
                    s.push(nodes.at(checkValue.indeces.at(0)));
                    s.push(nodes.at(checkValue.indeces.at(1)));
                } else if(intersect && checkValue.isLeaf) {
                    for(int i = 0; i < checkValue.indeces.size(); i++) {
                        triangleToCheck = triangles[checkValue.indeces[i]];
                        if(intersectRayWithTriangle(triangleToCheck.a, triangleToCheck.b, triangleToCheck.c, ray, hitInfo)) {
                            hitInfo.material = triangleToCheck.material;
                            hit = true;
                        }
                    }
                }
            }
        }
    } else {
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
    }

    // Intersect with spheres.
    for (const auto& sphere : m_pScene->spheres)
        hit |= intersectRayWithShape(sphere, ray, hitInfo);
    return hit;
}

