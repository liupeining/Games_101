#include <algorithm>
#include <cassert>
#include "SAH.hpp"

//程序计时
SAHAccel::SAHAccel(std::vector<Object*> p, int maxPrimsInNode, SplitMethod splitMethod)
    : maxPrimsInNode(std::min(255, maxPrimsInNode)), splitMethod(splitMethod),
      primitives(std::move(p))
{
    time_t start, stop;
    time(&start);
    if (primitives.empty())
        return;

    root = recursiveBuild(primitives);

    time(&stop);
    double diff = difftime(stop, start);
    int hrs = (int)diff / 3600;
    int mins = ((int)diff / 60) - (hrs * 60);
    int secs = (int)diff - (hrs * 3600) - (mins * 60);

    printf(
        "\rSAH Generation complete: \nTime Taken: %i hrs, %i mins, %i secs\n\n",
        hrs, mins, secs);
}

SAHBuildNode* SAHAccel::recursiveBuild(std::vector<Object*> objects)
{
    SAHBuildNode* node = new SAHBuildNode();
    // Compute bounds of all primitives in SAH node
    Bounds3 bounds;
    for (int i = 0; i < objects.size(); ++i)
        bounds = Union(bounds, objects[i]->getBounds());
    //接下来该分的只有一个了，则将该节点设为叶子节点，不再包含子节点
    if (objects.size() == 1) {
        // Create leaf _SAHBuildNode_
        node->bounds = objects[0]->getBounds();
        node->object = objects[0];
        node->left = nullptr;
        node->right = nullptr;
        return node;
    }
    //接下来该分的只有两个，则将左右节点各一个三角形
    else if (objects.size() == 2) {
        node->left = recursiveBuild(std::vector{objects[0]});
        node->right = recursiveBuild(std::vector{objects[1]});
        node->bounds = Union(node->left->bounds, node->right->bounds);
        return node;
    }
    else {
        Bounds3 centroidBounds;
        //求整体包围盒：
        for (int i = 0; i < objects.size(); ++i)
            centroidBounds = Union(centroidBounds, objects[i]->getBounds().Centroid());
        //SAH相关参数：
        float Sn = centroidBounds.SurfaceArea(); //Sn
        int B = 10; //B usually < 32;
        int minCostCoor = 0, mincostIndex = 0; //最优分割方案，轴和B的值
        float minCost = std::numeric_limits<float>::infinity(); //最小花费
        //分别进行三个轴的划分
        for (int i = 0; i < 3; i++)     
        {   //对三角形按照所选择的轴的位置进行排序
            switch (i) {
            case 0:  //x轴最长，对x进行排序
                std::sort(objects.begin(), objects.end(), [](auto f1, auto f2) {return f1->getBounds().Centroid().x < f2->getBounds().Centroid().x;});
                break;
            case 1:  //y轴最长，对y进行排序
                std::sort(objects.begin(), objects.end(), [](auto f1, auto f2) {return f1->getBounds().Centroid().y < f2->getBounds().Centroid().y;});
                break;
            case 2:  //z轴最长，对z进行排序
                std::sort(objects.begin(), objects.end(), [](auto f1, auto f2) {return f1->getBounds().Centroid().z < f2->getBounds().Centroid().z;});
                break;
            }
            for(int j = 1; j < B; j++)
            {
                auto beginning = objects.begin();
                auto middling = objects.begin() + (objects.size() * j / B);
                auto ending = objects.end();
                auto leftshapes = std::vector<Object*>(beginning, middling);
                auto rightshapes = std::vector<Object*>(middling, ending);
                //求左右包围盒:
                Bounds3 leftBounds, rightBounds;
                for (int k = 0; k < leftshapes.size(); ++k)
                    leftBounds = Union(leftBounds, leftshapes[k]->getBounds().Centroid());
                for (int k = 0; k < rightshapes.size(); ++k)
                    rightBounds = Union(rightBounds, rightshapes[k]->getBounds().Centroid());
                float SA = leftBounds.SurfaceArea(); //SA
                float SB = rightBounds.SurfaceArea(); //SB
                float cost = 1 + (leftshapes.size() * SA + rightshapes.size() * SB) / Sn; //计算花费
                if(cost < minCost) //如果花费更小，记录当前坐标值
                {
                    minCost = cost;
                    mincostIndex = j;
                    minCostCoor = i;
                }
            }
        }
        switch (minCostCoor) {
            case 0:  //对x进行排序
                std::sort(objects.begin(), objects.end(), [](auto f1, auto f2) {return f1->getBounds().Centroid().x < f2->getBounds().Centroid().x;});
                break;
            case 1:  //对y进行排序
                std::sort(objects.begin(), objects.end(), [](auto f1, auto f2) {return f1->getBounds().Centroid().y < f2->getBounds().Centroid().y;});
                break;
            case 2:  //对z进行排序
                std::sort(objects.begin(), objects.end(), [](auto f1, auto f2) {return f1->getBounds().Centroid().z < f2->getBounds().Centroid().z;});
                break;
        }
        auto beginning = objects.begin();
        auto middling = objects.begin() + (objects.size() * mincostIndex / B);
        auto ending = objects.end();
        auto leftshapes = std::vector<Object*>(beginning, middling);
        auto rightshapes = std::vector<Object*>(middling, ending);
        assert(objects.size() == (leftshapes.size() + rightshapes.size()));
        node->left = recursiveBuild(leftshapes);
        node->right = recursiveBuild(rightshapes);
        node->bounds = Union(node->left->bounds, node->right->bounds);
    }
    return node;
}

Intersection SAHAccel::Intersect(const Ray& ray) const
{
    Intersection isect;
    if (!root)
        return isect;
    isect = SAHAccel::getIntersection(root, ray);
    return isect;
}

Intersection SAHAccel::getIntersection(SAHBuildNode* node, const Ray& ray) const
{
    Intersection inter;
    //invdir = 1 / D; bounds3.hpp中会用到。
    Vector3f invdir(1 / ray.direction.x, 1 / ray.direction.y, 1 / ray.direction.z);
    //判断射线的方向正负，如果负，为1；bounds3.hpp中会用到。
    std::array<int, 3> dirIsNeg;
    dirIsNeg[0] = ray.direction.x < 0;
    dirIsNeg[1] = ray.direction.y < 0;
    dirIsNeg[2] = ray.direction.z < 0;
    
    //没有交点
    if(!node -> bounds.IntersectP(ray, invdir, dirIsNeg))
        return inter;
    //有交点，且该点为叶子节点，去和三角形求交
    if(node -> left == nullptr && node -> right == nullptr)
        return node -> object -> getIntersection(ray);
    //该点为中间节点，继续判断，并返回最近的包围盒交点
    Intersection hit1 = getIntersection(node -> left,  ray);
    Intersection hit2 = getIntersection(node -> right, ray);
    return hit1.distance < hit2.distance ? hit1 : hit2;
}