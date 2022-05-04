//
// Created by LEI XU on 5/16/19.
//

#ifndef RAYTRACING_SAH_H
#define RAYTRACING_SAH_H

#include <atomic>
#include <vector>
#include <memory>
#include <ctime>
#include "Object.hpp"
#include "Ray.hpp"
#include "Bounds3.hpp"
#include "Intersection.hpp"
#include "Vector.hpp"

struct SAHBuildNode;
// SAHAccel Forward Declarations
struct SAHPrimitiveInfo;

// SAHAccel Declarations
inline int leafNodes, totalLeafNodes, totalPrimitives, interiorNodes;
class SAHAccel {

public:
    // SAHAccel Public Types
    enum class SplitMethod { NAIVE, SAH };

    // SAHAccel Public Methods
    SAHAccel(std::vector<Object*> p, int maxPrimsInNode = 1, SplitMethod splitMethod = SplitMethod::NAIVE);
    Bounds3 WorldBound() const;
    ~SAHAccel();

    Intersection Intersect(const Ray &ray) const;
    Intersection getIntersection(SAHBuildNode* node, const Ray& ray)const;
    bool IntersectP(const Ray &ray) const;
    SAHBuildNode* root;

    // SAHAccel Private Methods
    SAHBuildNode* recursiveBuild(std::vector<Object*>objects);

    // SAHAccel Private Data
    const int maxPrimsInNode;
    const SplitMethod splitMethod;
    std::vector<Object*> primitives;
};

struct SAHBuildNode {
    Bounds3 bounds;
    SAHBuildNode *left;
    SAHBuildNode *right;
    Object* object;

public:
    int splitAxis=0, firstPrimOffset=0, nPrimitives=0;
    // SAHBuildNode Public Methods
    SAHBuildNode(){
        bounds = Bounds3();
        left = nullptr;right = nullptr;
        object = nullptr;
    }
};




#endif //RAYTRACING_SAH_H
