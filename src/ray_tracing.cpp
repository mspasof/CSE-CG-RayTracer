#include "ray_tracing.h"
#include "disable_all_warnings.h"
// Suppress warnings in third-party code.
DISABLE_WARNINGS_PUSH()
#include <glm/geometric.hpp>
#include <glm/gtx/component_wise.hpp>
#include <glm/vector_relational.hpp>
DISABLE_WARNINGS_POP()
#include <cmath>
#include <iostream>
#include <limits>

bool pointInTriangle(const glm::vec3& v0, const glm::vec3& v1, const glm::vec3& v2, const glm::vec3& n, const glm::vec3& p)
{
    float areaABP = glm::length(glm::cross(v1 - v0, p - v0)) / 2.0f;
    float areaACP = glm::length(glm::cross(v0 - v2, p - v2)) / 2.0f;
    float areaBCP = glm::length(glm::cross(v2 - v1, p - v1)) / 2.0f;
    float areaABC = glm::length(glm::cross(v1 - v0, v2 - v0)) / 2.0f;

    if (areaABP < 0.0f || areaACP < 0.0f || areaBCP < 0.0f || (areaABP + areaACP + areaBCP) / areaABC > 1.0001f)
        return false;
    else
        return true;
}

bool intersectRayWithPlane(const Plane& plane, Ray& ray)
{
    if (glm::dot(ray.direction, glm::normalize(plane.normal)) == 0)
        return false;

    float t = (plane.D - glm::dot(ray.origin, glm::normalize(plane.normal))) / glm::dot(ray.direction, glm::normalize(plane.normal));

    if (t > 0 && t < ray.t) {
        ray.t = t;
        return true;
    }
    else
        return false;
}

Plane trianglePlane(const glm::vec3& v0, const glm::vec3& v1, const glm::vec3& v2)
{
    Plane plane;
    glm::vec3 vecA = v1 - v0;
    glm::vec3 vecB = v2 - v0;

    plane.normal = glm::normalize(glm::cross(vecA, vecB));
    plane.D = glm::dot(plane.normal, v0);

    return plane;
}

/// Input: the three vertices of the triangle
/// Output: if intersects then modify the hit parameter ray.t and return true, otherwise return false
bool intersectRayWithTriangle(const glm::vec3& v0, const glm::vec3& v1, const glm::vec3& v2, Ray& ray, HitInfo& hitInfo)
{   
    Plane plane = trianglePlane(v0, v1, v2);
    float originalT = ray.t;
    bool hitPlane = intersectRayWithPlane(plane, ray);
    if (hitPlane) {
        glm::vec3 point = ray.origin + ray.t * ray.direction;
        bool inTriangle = pointInTriangle(v0, v1, v2, plane.normal, point);
        if (!inTriangle)
            ray.t = originalT;
        else {
            hitInfo.normal = plane.normal;
            return true;
        }  
    }
    return false;
}

/// Input: a sphere with the following attributes: sphere.radius, sphere.center
/// Output: if intersects then modify the hit parameter ray.t and return true, otherwise return false
bool intersectRayWithShape(const Sphere& sphere, Ray& ray, HitInfo& hitInfo)
{
    glm::vec3 newOrigin = ray.origin - sphere.center;

    float a = pow(ray.direction.x, 2) + pow(ray.direction.y, 2) + pow(ray.direction.z, 2);
    float b = 2 * (ray.direction.x * newOrigin.x + ray.direction.y * newOrigin.y + ray.direction.z * newOrigin.z);
    float c = pow(newOrigin.x, 2) + pow(newOrigin.y, 2) + pow(newOrigin.z, 2) - pow(sphere.radius, 2);

    float D = pow(b, 2) - 4 * a * c;

    if (D < 0)
        return false;

    if (D == 0) {
        float t = -b / (2 * a);
        if(t < ray.t && t > 0) {
            ray.t = t;
            hitInfo.material = sphere.material;
            hitInfo.normal = -sphere.center + (ray.origin + ray.t * ray.direction);
            return true;
        }
    }

    if (D > 0) {
        float t1 = (-b + sqrt(D)) / (2 * a);
        float t2 = (-b - sqrt(D)) / (2 * a);
        float t;
        if(t1 > 0 && t2 > 0) t = glm::min(t1, t2);
        else t = glm::max(t1, t2);
        if(t < ray.t && t > 0) {
            ray.t = t;
            hitInfo.material = sphere.material;
            hitInfo.normal = -sphere.center + (ray.origin + ray.t * ray.direction);
            return true;
        }
    }
    return false;
}

/// Input: an axis-aligned bounding box with the following parameters: minimum coordinates box.lower and maximum coordinates box.upper
/// Output: if intersects then modify the hit parameter ray.t and return true, otherwise return false
bool intersectRayWithShape(const AxisAlignedBox& box, Ray& ray)
{
    float txmin = (box.lower.x - ray.origin.x) / ray.direction.x;
    float tymin = (box.lower.y - ray.origin.y) / ray.direction.y;
    float tzmin = (box.lower.z - ray.origin.z) / ray.direction.z;
    float txmax = (box.upper.x - ray.origin.x) / ray.direction.x;
    float tymax = (box.upper.y - ray.origin.y) / ray.direction.y;
    float tzmax = (box.upper.z - ray.origin.z) / ray.direction.z;

    float tinx = glm::min(txmin, txmax);
    float toutx = glm::max(txmin, txmax);

    float tiny = glm::min(tymin, tymax);
    float touty = glm::max(tymin, tymax);

    float tinz = glm::min(tzmin, tzmax);
    float toutz = glm::max(tzmin, tzmax);

    float tin = glm::max(tinx, glm::max(tiny, tinz));
    float tout = glm::min(toutx, glm::min(touty, toutz));

    if (tin > tout || tout < 0)
        return false;
    else {
        if(tin < ray.t && tin > 0) {
            ray.t = tin;
            return true;
        }
    }
    return false;
}
