#include "bounding_volume_hierarchy.h"
#include "disable_all_warnings.h"
#include "draw.h"
#include "image.h"
#include "ray_tracing.h"
#include "screen.h"
#include "trackball.h"
#include "window.h"
// Disable compiler warnings in third-party code (which we cannot change).
DISABLE_WARNINGS_PUSH()
#include <glm/gtc/type_ptr.hpp>
#include <glm/vec2.hpp>
#include <glm/vec4.hpp>
#include <imgui.h>
DISABLE_WARNINGS_POP()
#include <chrono>
#include <cstdlib>
#include <fstream>
#include <iostream>
#include <optional>
#include <random>
#include <string>
#include <type_traits>
#ifdef USE_OPENMP
#include <omp.h>
#endif

// This is the main application. The code in here does not need to be modified.
constexpr glm::ivec2 windowResolution { 800, 800 };
const std::filesystem::path dataPath { DATA_DIR };
const std::filesystem::path outputPath { OUTPUT_DIR };
const float EPSILON = 0.00001f;

// Config
const int reflectionDepthLimit = 8;
int maxReflectionDepth = reflectionDepthLimit;
bool debugHardShadows = false;
bool debugNormals = false;
bool debugReflections = false;
bool debugTransmissions = false;

enum class ViewMode {
    Rasterization = 0,
    RayTracing = 1
};


static glm::vec3 getDiffuseLighting(const Scene& scene, Ray ray, HitInfo hitInfo);
static glm::vec3 getSpecularLighting(const Scene& scene, Ray ray, HitInfo hitInfo);
static glm::vec3 getCombReflRefr(const Scene& scene, const BoundingVolumeHierarchy& bvh, Ray ray, HitInfo hitInfo, int depth);static glm::vec3 ReflectDir(glm::vec3 vec, glm::vec3 norm);
static glm::vec3 RefractDir(glm::vec3& vec, glm::vec3& norm, float nextIOR, float currentIOR = 1.0f);

void clampLight(glm::vec3& light){
    if (light.x > 1.0f)
        light.x = 1.0f;
    if (light.y > 1.0f)
        light.y = 1.0f;
    if (light.z > 1.0f)
        light.z = 1.0f;
}


bool isVisibleByPointLight(const Scene& scene, const BoundingVolumeHierarchy& bvh, Ray ray, HitInfo& hitInfo, PointLight pointLight){
    Ray rayToPointLightSource;
    rayToPointLightSource.origin = ray.origin + ray.t*ray.direction;
    rayToPointLightSource.direction = -rayToPointLightSource.origin + pointLight.position;
    rayToPointLightSource.origin = rayToPointLightSource.origin + EPSILON * rayToPointLightSource.direction;
    rayToPointLightSource.t = 1;
    bool intersect = false;
    // You can uncomment to next line to draw a ray to each pointLight source regardless of intersections with other objects.
    // drawRay(rayToPointLightSource, glm::vec3(1));
    for (const auto& mesh : scene.meshes) {
        for (const auto& tri : mesh.triangles) {
            const auto v0 = mesh.vertices[tri[0]];
            const auto v1 = mesh.vertices[tri[1]];
            const auto v2 = mesh.vertices[tri[2]];
                    
            if (intersectRayWithTriangle(v0.p, v1.p, v2.p, rayToPointLightSource, hitInfo)) {
                // This can be used later if we work with transparent objects.
                hitInfo.material = mesh.material;
                intersect = true;
            }
        }
    }
    for(const auto& sphere : scene.spheres) {
        if(intersectRayWithShape(sphere, rayToPointLightSource, hitInfo)) {
            // This can be used later if we work with transparent objects.
            hitInfo.material = sphere.material;
            intersect = true;
        }   
    }

    // Drawing a red visual debug ray if it intersects any object apart from the pointLight.
    if(debugHardShadows){
        if(intersect == true) drawRay(rayToPointLightSource, glm::vec3(1.0f, 0.0f, 0.0f));
        else drawRay(rayToPointLightSource, glm::vec3(1.0f));
    }
    return intersect;
}

// NOTE(Mathijs): separate function to make recursion easier (could also be done with lambda + std::function).
static glm::vec3 getFinalColor(const Scene& scene, const BoundingVolumeHierarchy& bvh, Ray& refRay, int depth, bool changeRay = false)
{
    HitInfo hitInfo;
    Ray ray = refRay;
    if (bvh.intersect(ray, hitInfo)) {
        if(changeRay)
            refRay.t = ray.t;
        glm::vec3 color(0.0f);
        // Draw a white debug ray.
        if(depth == 0)
            drawRay(ray, glm::vec3(1.0f));
        
        for(const auto& pointLight : scene.pointLights) {
            (void)isVisibleByPointLight(scene, bvh, ray, hitInfo, pointLight);
        }

        if(depth<=maxReflectionDepth)
            color += getCombReflRefr(scene, bvh, ray, hitInfo, depth);

        if(debugNormals){
            Ray normal;
            normal.t = 1;
            normal.origin = ray.origin + ray.direction * ray.t;
            normal.direction = hitInfo.normal;
            drawRay(normal, glm::vec3{0.0f, 0.0f, 1.0f});
        }

        // Set the color of the pixel to white if the ray hits.

        color += getDiffuseLighting(scene, ray, hitInfo); // + getSpecularLighting(scene, ray, hitInfo);
        clampLight(color);
        return color;
    } else {
        // Draw a red debug ray if the ray missed.
        if(depth==0)
            drawRay(ray, glm::vec3(1.0f, 0.0f, 0.0f));
        // Set the color of the pixel to black if the ray misses.
        return glm::vec3(0.0f);
    }
}
static void setOpenGLMatrices(const Trackball& camera);
static void renderOpenGL(const Scene& scene, const Trackball& camera, int selectedLight);

// This is the main rendering function. You are free to change this function in any way (including the function signature).
static void renderRayTracing(const Scene& scene, const Trackball& camera, const BoundingVolumeHierarchy& bvh, Screen& screen)
{
#ifdef USE_OPENMP
#pragma omp parallel for
#endif
    for (int y = 0; y < windowResolution.y; y++) {
        for (int x = 0; x != windowResolution.x; x++) {
            // NOTE: (-1, -1) at the bottom left of the screen, (+1, +1) at the top right of the screen.
            const glm::vec2 normalizedPixelPos {
                float(x) / windowResolution.x * 2.0f - 1.0f,
                float(y) / windowResolution.y * 2.0f - 1.0f
            };
            Ray cameraRay = camera.generateRay(normalizedPixelPos);
            screen.setPixel(x, y, getFinalColor(scene, bvh, cameraRay, 0));
        }
    }
}

int main(int argc, char** argv)
{
    Trackball::printHelp();
    std::cout << "\n Press the [R] key on your keyboard to create a ray towards the mouse cursor" << std::endl
              << std::endl;

    Window window { "Final Project - Part 2", windowResolution, OpenGLVersion::GL2 };
    Screen screen { windowResolution };
    Trackball camera { &window, glm::radians(50.0f), 3.0f };
    camera.setCamera(glm::vec3(0.0f, 0.0f, 0.0f), glm::radians(glm::vec3(20.0f, 20.0f, 0.0f)), 3.0f);

    SceneType sceneType { SceneType::SingleTriangle };
    std::optional<Ray> optDebugRay;
    Scene scene = loadScene(sceneType, dataPath);
    BoundingVolumeHierarchy bvh { &scene };

    int bvhDebugLevel = 0;
    bool debugBVH { false };
    ViewMode viewMode { ViewMode::Rasterization };

    window.registerKeyCallback([&](int key, int /* scancode */, int action, int /* mods */) {
        if (action == GLFW_PRESS) {
            switch (key) {
            case GLFW_KEY_R: {
                // Shoot a ray. Produce a ray from camera to the far plane.
                const auto tmp = window.getNormalizedCursorPos();
                optDebugRay = camera.generateRay(tmp * 2.0f - 1.0f);
                viewMode = ViewMode::Rasterization;
            } break;
            case GLFW_KEY_ESCAPE: {
                window.close();
            } break;
            };
        }
    });

    int selectedLight { 0 };
    while (!window.shouldClose()) {
        window.updateInput();

        // === Setup the UI ===
        ImGui::Begin("Final Project - Part 2");
        {
            constexpr std::array items { "SingleTriangle", "Cube", "Cornell Box (with mirror)", "Cornell Box (spherical light and mirror)", "Cornell Box (Transparent)", "Monkey", "Dragon", /* "AABBs",*/ "Spheres", /*"Mixed",*/ "Custom" };
            if (ImGui::Combo("Scenes", reinterpret_cast<int*>(&sceneType), items.data(), int(items.size()))) {
                optDebugRay.reset();
                scene = loadScene(sceneType, dataPath);
                bvh = BoundingVolumeHierarchy(&scene);
                if (optDebugRay) {
                    HitInfo dummy {};
                    bvh.intersect(*optDebugRay, dummy);
                }
            }
        }
        {
            constexpr std::array items { "Rasterization", "Ray Traced" };
            ImGui::Combo("View mode", reinterpret_cast<int*>(&viewMode), items.data(), int(items.size()));
        }
        if (ImGui::Button("Render to file")) {
            {
                using clock = std::chrono::high_resolution_clock;
                const auto start = clock::now();
                renderRayTracing(scene, camera, bvh, screen);
                const auto end = clock::now();
                std::cout << "Time to render image: " << std::chrono::duration<float, std::milli>(end - start).count() << " milliseconds" << std::endl;
            }
            screen.writeBitmapToFile(outputPath / "render.bmp");
        }
        ImGui::Spacing();
        ImGui::Separator();

        // Add your visual debug switches in this section
        ImGui::Text("Visual Debuging");
        ImGui::Checkbox("Draw Direct Light", &debugHardShadows);
        ImGui::Checkbox("Draw Normals", &debugNormals);
        ImGui::Checkbox("Draw Reflections", &debugReflections);
        ImGui::Checkbox("Draw Transmissions", &debugTransmissions);
        
        ImGui::Spacing();
        ImGui::Separator();
        
        // If you have a variable you want to test, feel free to add a new GUI item to this section
        ImGui::Text("Custom Debug Variables");
        ImGui::SliderInt("Max Reflection Depth", &maxReflectionDepth, 0, reflectionDepthLimit);

        ImGui::Spacing();
        ImGui::Separator();
        ImGui::Text("Debugging");
        if (viewMode == ViewMode::Rasterization) {
            ImGui::Checkbox("Draw BVH", &debugBVH);
            if (debugBVH)
                ImGui::SliderInt("BVH Level", &bvhDebugLevel, 0, bvh.numLevels() - 1);
        }

        ImGui::Spacing();
        ImGui::Separator();
        ImGui::Text("Lights");
        if (!scene.pointLights.empty() || !scene.sphericalLight.empty()) {
            {
                std::vector<std::string> options;
                for (size_t i = 0; i < scene.pointLights.size(); i++) {
                    options.push_back("Point Light " + std::to_string(i + 1));
                }
                for (size_t i = 0; i < scene.sphericalLight.size(); i++) {
                    options.push_back("Spherical Light " + std::to_string(i + 1));
                }

                std::vector<const char*> optionsPointers;
                std::transform(std::begin(options), std::end(options), std::back_inserter(optionsPointers),
                    [](const auto& str) { return str.c_str(); });

                ImGui::Combo("Selected light", &selectedLight, optionsPointers.data(), static_cast<int>(optionsPointers.size()));
            }

            {
                const auto showLightOptions = [](auto& light) {
                    ImGui::DragFloat3("Light position", glm::value_ptr(light.position), 0.01f, -3.0f, 3.0f);
                    ImGui::ColorEdit3("Light color", glm::value_ptr(light.color));
                    if constexpr (std::is_same_v<std::decay_t<decltype(light)>, SphericalLight>) {
                        ImGui::DragFloat("Light radius", &light.radius, 0.01f, 0.01f, 0.5f);
                    }
                };
                if (selectedLight < static_cast<int>(scene.pointLights.size())) {
                    // Draw a big yellow sphere and then the small light sphere on top.
                    showLightOptions(scene.pointLights[selectedLight]);
                } else {
                    // Draw a big yellow sphere and then the smaller light sphere on top.
                    showLightOptions(scene.sphericalLight[selectedLight - scene.pointLights.size()]);
                }
            }
        }

        if (ImGui::Button("Add point light")) {
            scene.pointLights.push_back(PointLight { glm::vec3(0.0f), glm::vec3(1.0f) });
            selectedLight = int(scene.pointLights.size() - 1);
        }
        if (ImGui::Button("Add spherical light")) {
            scene.sphericalLight.push_back(SphericalLight { glm::vec3(0.0f), 0.1f, glm::vec3(1.0f) });
            selectedLight = int(scene.pointLights.size() + scene.sphericalLight.size() - 1);
        }
        if (ImGui::Button("Remove selected light")) {
            if (selectedLight < static_cast<int>(scene.pointLights.size())) {
                scene.pointLights.erase(std::begin(scene.pointLights) + selectedLight);
            } else {
                scene.sphericalLight.erase(std::begin(scene.sphericalLight) + (selectedLight - scene.pointLights.size()));
            }
            selectedLight = 0;
        }

        // Clear screen.
        glClearDepth(1.0f);
        glClearColor(0.0, 0.0, 0.0, 0.0);
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        // Draw either using OpenGL (rasterization) or the ray tracing function.
        switch (viewMode) {
        case ViewMode::Rasterization: {
            glPushAttrib(GL_ALL_ATTRIB_BITS);
            renderOpenGL(scene, camera, selectedLight);
            if (optDebugRay) {
                // Call getFinalColor for the debug ray. Ignore the result but tell the function that it should
                // draw the rays instead.
                enableDrawRay = true;
                (void)getFinalColor(scene, bvh, *optDebugRay, 0);
                enableDrawRay = false;
            }
            glPopAttrib();
        } break;
        case ViewMode::RayTracing: {
            screen.clear(glm::vec3(0.0f));
            renderRayTracing(scene, camera, bvh, screen);
            screen.setPixel(0, 0, glm::vec3(1.0f));
            screen.draw(); // Takes the image generated using ray tracing and outputs it to the screen using OpenGL.
        } break;
        default:
            break;
        };

        if (debugBVH) {
            glPushAttrib(GL_ALL_ATTRIB_BITS);
            setOpenGLMatrices(camera);
            glDisable(GL_LIGHTING);
            glEnable(GL_DEPTH_TEST);

            // Enable alpha blending. More info at:
            // https://learnopengl.com/Advanced-OpenGL/Blending
            glEnable(GL_BLEND);
            glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
            bvh.debugDraw(bvhDebugLevel);
            glPopAttrib();
        }

        ImGui::End();
        window.swapBuffers();
    }

    return 0; // execution never reaches this point
}

static void setOpenGLMatrices(const Trackball& camera)
{
    // Load view matrix.
    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();
    const glm::mat4 viewMatrix = camera.viewMatrix();
    glMultMatrixf(glm::value_ptr(viewMatrix));

    // Load projection matrix.
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    const glm::mat4 projectionMatrix = camera.projectionMatrix();
    glMultMatrixf(glm::value_ptr(projectionMatrix));
}

static void renderOpenGL(const Scene& scene, const Trackball& camera, int selectedLight)
{
    // Normals will be normalized in the graphics pipeline.
    glEnable(GL_NORMALIZE);
    // Activate rendering modes.
    glEnable(GL_DEPTH_TEST);
    // Draw front and back facing triangles filled.
    glPolygonMode(GL_FRONT, GL_FILL);
    glPolygonMode(GL_BACK, GL_FILL);
    // Interpolate vertex colors over the triangles.
    glShadeModel(GL_SMOOTH);
    setOpenGLMatrices(camera);

    glDisable(GL_LIGHTING);
    // Render point lights as very small dots
    for (const auto& light : scene.pointLights)
        drawSphere(light.position, 0.01f, light.color);
    for (const auto& light : scene.sphericalLight)
        drawSphere(light.position, light.radius, light.color);

    if (!scene.pointLights.empty() || !scene.sphericalLight.empty()) {
        if (selectedLight < static_cast<int>(scene.pointLights.size())) {
            // Draw a big yellow sphere and then the small light sphere on top.
            const auto& light = scene.pointLights[selectedLight];
            drawSphere(light.position, 0.05f, glm::vec3(1, 1, 0));
            glDisable(GL_DEPTH_TEST);
            drawSphere(light.position, 0.01f, light.color);
            glEnable(GL_DEPTH_TEST);
        } else {
            // Draw a big yellow sphere and then the smaller light sphere on top.
            const auto& light = scene.sphericalLight[selectedLight - scene.pointLights.size()];
            drawSphere(light.position, light.radius + 0.01f, glm::vec3(1, 1, 0));
            glDisable(GL_DEPTH_TEST);
            drawSphere(light.position, light.radius, light.color);
            glEnable(GL_DEPTH_TEST);
        }
    }

    // Activate the light in the legacy OpenGL mode.
    glEnable(GL_LIGHTING);

    int i = 0;
    const auto enableLight = [&](const auto& light) {
        glEnable(GL_LIGHT0 + i);
        const glm::vec4 position4 { light.position, 1 };
        glLightfv(GL_LIGHT0 + i, GL_POSITION, glm::value_ptr(position4));
        const glm::vec4 color4 { glm::clamp(light.color, 0.0f, 1.0f), 1.0f };
        const glm::vec4 zero4 { 0.0f, 0.0f, 0.0f, 1.0f };
        glLightfv(GL_LIGHT0 + i, GL_AMBIENT, glm::value_ptr(zero4));
        glLightfv(GL_LIGHT0 + i, GL_DIFFUSE, glm::value_ptr(color4));
        glLightfv(GL_LIGHT0 + i, GL_SPECULAR, glm::value_ptr(zero4));
        // NOTE: quadratic attenuation doesn't work like you think it would in legacy OpenGL.
        // The distance is not in world space but in NDC space!
        glLightf(GL_LIGHT0 + i, GL_CONSTANT_ATTENUATION, 1.0f);
        glLightf(GL_LIGHT0 + i, GL_LINEAR_ATTENUATION, 0.0f);
        glLightf(GL_LIGHT0 + i, GL_QUADRATIC_ATTENUATION, 0.0f);
        i++;
    };
    for (const auto& light : scene.pointLights)
        enableLight(light);
    for (const auto& light : scene.sphericalLight)
        enableLight(light);

    // Draw the scene and the ray (if any).
    drawScene(scene);

    // Draw a colored sphere at the location at which the trackball is looking/rotating around.
    glDisable(GL_LIGHTING);
    drawSphere(camera.lookAt(), 0.01f, glm::vec3(0.2f, 0.2f, 1.0f));
}

static glm::vec3 getDiffuseLighting(const Scene& scene, Ray ray, HitInfo hitInfo) {
    const glm::vec3 Kd = hitInfo.material.kd;

    glm::vec3 normal = hitInfo.normal;
    if (glm::dot(glm::normalize(hitInfo.normal), glm::normalize(ray.direction)) > 0)
        normal = -hitInfo.normal;
    
    const glm::vec3 pointPos = ray.origin + ray.t * ray.direction;

    glm::vec3 totalVector = glm::vec3(0.0f);

    for (PointLight light : scene.pointLights) {
        glm::vec3 lightPos = light.position;
        glm::vec3 pointToLight = lightPos - pointPos;

        float coefficient = glm::dot(glm::normalize(normal), glm::normalize(pointToLight));

        if (coefficient < 0.0f)
            coefficient = 0.0f;

        float distance2 = glm::dot(pointToLight, pointToLight);
        glm::vec3 lightcolor = light.color;

        coefficient = coefficient / distance2;
        glm::vec3 individualVector = Kd * lightcolor * coefficient;

        totalVector += individualVector;
    }


    const int N = 20; // number of points that will be distributed on the sphere
    for (SphericalLight sph_light : scene.sphericalLight) {
        std::vector<PointLight> placedPointLights;

        const double phi = (sqrt(5.0) + 1.0) / 2.0; // golden ratio
        const double g = (2.0 * glm::pi<float>()) * (2.0 - phi);  //golden angle

        for (int i = 1; i <= N; i++) {
            const double lat = asin(-1.0 + 2.0 * double(i) / (N + 1));
            const double lon = g * i;

            const double x = cos(lon) * cos(lat);
            const double y = sin(lon) * cos(lat);
            const double z = sin(lat);

            PointLight pointlight;
            pointlight.position = glm::vec3{ x, y, z } * sph_light.radius + sph_light.position;
            pointlight.color = sph_light.color;
            placedPointLights.push_back(pointlight);

            // above method for distributing points over a sphere adapted from:
            // https://bduvenhage.me/geometry/2019/07/31/generating-equidistant-vectors.html
        }

        for (PointLight light : placedPointLights) {
            glm::vec3 lightPos = light.position;
            glm::vec3 pointToLight = lightPos - pointPos;

            float coefficient = glm::dot(glm::normalize(normal), glm::normalize(pointToLight));

            if (coefficient < 0.0f)
                coefficient = 0.0f;

            float distance2 = glm::dot(pointToLight, pointToLight);
            glm::vec3 lightcolor = light.color;

            coefficient = coefficient / distance2 / N;
            glm::vec3 individualVector = Kd * lightcolor * coefficient;

            totalVector += individualVector;
        }
    }

    if (totalVector.x > 1.0f)
        totalVector.x = 1.0f;
    if (totalVector.y > 1.0f)
        totalVector.y = 1.0f;
    if (totalVector.z > 1.0f)
        totalVector.z = 1.0f;

    clampLight(totalVector);

    return totalVector;
}


static glm::vec3 getSpecularLighting(const Scene& scene, Ray ray, HitInfo hitInfo) {
    const glm::vec3 Ks = hitInfo.material.ks;
    const float s = hitInfo.material.shininess;

    glm::vec3 normal = hitInfo.normal;
    if (glm::dot(glm::normalize(hitInfo.normal), glm::normalize(ray.direction)) > 0)
        normal = -hitInfo.normal;

    const glm::vec3 pointPos = ray.origin + ray.t * ray.direction;

    glm::vec3 totalVector = glm::vec3(0.0f);

    for (PointLight light : scene.pointLights) {
        glm::vec3 lightPos = light.position;
        glm::vec3 lightToPoint = pointPos - lightPos;
        glm::vec3 reflection = glm::normalize(glm::reflect(glm::normalize(lightToPoint), glm::normalize(normal)));
        glm::vec3 view = ray.origin - pointPos;

        float coefficient = glm::pow((glm::dot(glm::normalize(reflection), glm::normalize(view))), s);

        if (coefficient < 0.0f)
            coefficient = 0.0f;

        glm::vec3 lightcolor = light.color;

        glm::vec3 individualVector = Ks * lightcolor * coefficient;

        totalVector += individualVector;
    }

    if (totalVector.x > 1.0f)
        totalVector.x = 1.0f;
    if (totalVector.y > 1.0f)
        totalVector.y = 1.0f;
    if (totalVector.z > 1.0f)
        totalVector.z = 1.0f;

    return totalVector;
}

static glm::vec3 getCombReflRefr(const Scene& scene, const BoundingVolumeHierarchy& bvh, Ray ray, HitInfo hitInfo, int depth){
    // Since we don't have extinction coefficients available, we can't use fully scientifically accurate formulas for computing reflections and transmissions (Fresnel).
    // Hence we will be using a combination of a base opacity value and a variation of the Schlick's approximation for the reflections/transmissions ratio and Snell's law for the actual refractions
    float cosi = std::max(-1.0f, std::min(1.0f, glm::dot(glm::normalize(ray.direction), hitInfo.normal)));
    glm::vec3 refN = hitInfo.normal;
    if(cosi < 0){
        cosi = -cosi;
    } else {
        cosi = 1 - std::abs(cosi);
        refN = -hitInfo.normal;
    }
    
    glm::vec3 refractDir = RefractDir(ray.direction, hitInfo.normal, hitInfo.material.ior);
    glm::vec3 reflectDir = glm::reflect(ray.direction, refN);
    glm::vec3 rayOrigin = ray.origin + ray.direction * ray.t;

    float ratioRef;
    if(refractDir==glm::vec3{0.0f}){
        ratioRef = 1.0; // Full internal reflection
    } else {
        ratioRef = hitInfo.material.transparency + (1.0f-hitInfo.material.transparency)*std::pow(1.0f - cosi, 5);
    }

    glm::vec3 final(0.0f);
    if(ratioRef>0 && hitInfo.material.ks!=glm::vec3(0.0f)){
        Ray refl;
        refl.direction = reflectDir;
        refl.origin = rayOrigin - ray.direction * EPSILON;
        final += ratioRef * hitInfo.material.ks * getFinalColor(scene, bvh, refl, depth+1, true);
        if(debugReflections)
            drawRay(refl, ratioRef * (refl.t==std::numeric_limits<float>::max()? glm::vec3(1.0f, 0.0f, 0.0f): glm::vec3(1.0f)));
    }
    if(ratioRef<1 && hitInfo.material.kt!=glm::vec3(0.0f)){
        Ray refr;
        refr.direction = refractDir;
        refr.origin = rayOrigin + ray.direction * EPSILON;
        final += (1 - ratioRef) * hitInfo.material.kt * getFinalColor(scene, bvh, refr, depth+1, true);
        if(debugTransmissions)
            drawRay(refr, (1.0f - ratioRef) * (refr.t==std::numeric_limits<float>::max()? glm::vec3(1.0f, 0.0f, 0.0f): glm::vec3(1.0f)));
    }
    return final;
}

static glm::vec3 RefractDir(glm::vec3& vec, glm::vec3& norm, float nextIOR, float currentIOR){
    float dot = std::max(-1.0f, std::min(1.0f, glm::dot(glm::normalize(vec), norm))); 
    glm::vec3 refN = norm; 
    if (dot < 0) { dot = -dot; } else { std::swap(currentIOR, nextIOR); refN= -norm; } 
    float ratioIOR = currentIOR / nextIOR; 
    float k = 1 - ratioIOR * ratioIOR * (1 - dot * dot);
    if(k<0)
        return glm::vec3 { 0.0f };
    return ratioIOR * vec + (ratioIOR * dot - sqrtf(k)) * refN; 
}
