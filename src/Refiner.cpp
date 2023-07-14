#include "Utils.h"
#include "ChunkGeneratorHell.h"
#include "ChunkGen.h"
#include "Refiner.h"
#include "PathFinder.h"

#include <vector>
#include <unordered_map>
#include <variant>
#include <optional>

// An efficient parametric algorithm for octree traversal
// http://citeseerx.ist.psu.edu/viewdoc/summary?doi=10.1.1.29.987
// https://stackoverflow.com/questions/10228690/ray-octree-intersection-algorithms

struct Ray {
    Vec3 origin;
    Vec3 dir; // between -1 and 1
};

std::pair<Ray, double> computeRay(Vec3 a, Vec3 b) {
    auto vec = Vec3{b.x - a.x, b.y - a.y, b.z - a.z};
    auto magnitude = std::sqrt(vec.x * vec.x + vec.y * vec.y + vec.z * vec.z);
    auto unitVec =  Vec3{vec.x / magnitude, vec.y / magnitude, vec.z / magnitude};
    return {
        {
            .origin = a,
            .dir = unitVec
        },
        magnitude
    };
}

double reflect(double x, double target) {
    return target - (x - target);
}

Ray reflectRay(Ray ray, const Vec3& center, uint8_t a) {
    if (a & 4) {
        ray.origin.x = reflect(ray.origin.x, center.x);
        ray.dir.x = -ray.dir.x;
    }
    if (a & 2) {
        ray.origin.y = reflect(ray.origin.y, center.y);
        ray.dir.y = -ray.dir.y;
    }
    if (a & 1) {
        ray.origin.z = reflect(ray.origin.z, center.z);
        ray.dir.z = -ray.dir.z;
    }
    return ray;
}

template<Size size>
auto nodeType0() {
    if constexpr (size == Size::X16) {
        return x16_t{};
    }
    if constexpr (size == Size::X8) {
        return x8_t{};
    }
    if constexpr (size == Size::X4) {
        return x4_t{};
    }
    if constexpr (size == Size::X2) {
        return x2_t{};
    }
}

template<Size size>
using NodeType = decltype(nodeType0<size>());

constexpr Size nextLowerSize(Size size) {
    return static_cast<Size>(static_cast<int>(size) - 1);
}

template<typename Node>
constexpr auto getDaughter(const Node& node, int i) {
    if constexpr (std::is_same_v<Node, x2_t>) {
        // im pretty confident the lower bits are closer to the origin
        return ((node >> i) & 1) != 0;
    } else {
        return &node[i];
    }
}

template<typename T>
constexpr bool isZero(const T& x) {
    if constexpr (sizeof(T) < sizeof(int64_t)) {
        return x == T{};
    } else {
        return isEmpty(x);
    }
}

template<typename Self, Size sz>
struct NodeBase {
    int x, y, z;

    int width() const {
        return ::width(sz);
    }

    Size size() const {
        return sz;
    }

    bool empty() const {
        return isZero(*static_cast<const Self*>(this)->node);
    }

    double minX() const {
        return static_cast<double>(this->x);
    }
    double minY() const {
        return static_cast<double>(this->y);
    }
    double minZ() const {
        return static_cast<double>(this->z);
    }
    double maxX() const {
        return static_cast<double>(this->x + width());
    }
    double maxY() const {
        return static_cast<double>(this->y + width());
    }
    double maxZ() const {
        return static_cast<double>(this->z + width());
    }
};

template<Size sz>
struct Node : NodeBase<Node<sz>, sz> {
    const NodeType<sz>* node;
};

template<>
struct Node<Size::X1> : NodeBase<Node<Size::X1>, Size::X1> {
    bool node;
};

template<>
struct Node<Size::X16> : NodeBase<Node<Size::X16>, Size::X16> {
    const x16_t* node;
    bool cachedEmpty;

    constexpr bool empty() const {
        return this->cachedEmpty;
    }
};


Vec3 center(const Node<Size::X16>& node) {
    constexpr int halfWidth = (::width(Size::X16) / 2);
    return {
        static_cast<double>(node.x + halfWidth),
        static_cast<double>(node.y + halfWidth),
        static_cast<double>(node.z + halfWidth)
    };
}

template<Size sz> requires (sz > Size::X1)
Node<nextLowerSize(sz)> daughter(const Node<sz>& node, int i) {
    // See figure 1
    constexpr auto halfWidth = ::width(nextLowerSize(sz));
    return {
        {
            node.x + (i & 4 ? halfWidth : 0), // 4,5,6,7
            node.y + (i & 2 ? halfWidth : 0), // 2,3,6,7
            node.z + (i & 1 ? halfWidth : 0)  // 1,3,5,7
        },
        getDaughter(*node.node, i)
    };
}

double max(double x, double y) {
    // clang actually generates different code with the unlikely attribute but there doesnt seem to be any difference in performance
    if (std::isnan(y)) [[unlikely]] return x;
    return x > y ? x : y;
}

double max(double x, double y, double z) {
    return max(max(x, y), z);
}

double min(double x, double y) {
    if (std::isnan(y)) [[unlikely]] return x;
    return x < y ? x : y;
}

double min(double x, double y, double z) {
    return min(min(x, y), z);
}



Plane exitPlane(double tx1, double ty1, double tz1) {
    if (tx1 < ty1) {
        if (tx1 < tz1) return Plane::YZ;  // YZ plane
    } else {
        if (ty1 < tz1) return Plane::XZ;  // XZ plane
    }
    return Plane::XY; // XY plane;
}

int first_node(double tx0, double ty0, double tz0, double txm, double tym, double tzm) {
    uint8_t answer = 0;   // initialize to 00000000
    // select the entry plane and set bits
    if (tx0 > ty0) {
        if (tx0 > tz0) { // PLANE YZ
            // see table 1
            if (tym < tx0) answer |= 2;    // set bit at position 1
            if (tzm < tx0) answer |= 1;    // set bit at position 0
            return (int) answer;
        }
    } else {
        if (ty0 > tz0) { // PLANE XZ
            if (txm < ty0) answer |= 4;    // set bit at position 2
            if (tzm < ty0) answer |= 1;    // set bit at position 0
            return (int) answer;
        }
    }
    // PLANE XY
    if (txm < tz0) answer |= 4;    // set bit at position 2
    if (tym < tz0) answer |= 2;    // set bit at position 1
    return (int) answer;
}

// clang makes this branchless
// https://gcc.godbolt.org/z/5Th7GYb4j
int new_node(double txm, int x, double tym, int y, double tzm, int z) {
    int out = z; // XY plane
    if (txm < tym) {
        out = txm < tzm ? x : out; // YZ plane
    } else {
        out = tym < tzm ? y : out; // XZ plane
    }
    return out;
}

enum class SubtreeResultType {
    HIT,
    FINISHED,
    MISS
};
struct SubtreeResult {
    SubtreeResultType type;
    double len{}; // only valid for hits
};

template<Size Size>
SubtreeResult proc_subtree(uint8_t a, const Vec3& origin, double targetLen, double tx0, double ty0, double tz0, double tx1, double ty1, double tz1, const Node<Size>& node) {
    using enum SubtreeResultType;
    // if this node is behind us
    if (tx1 < 0.0 || ty1 < 0.0 || tz1 < 0.0) {
        return {MISS};
    }

    if (tx0 >= targetLen || ty0 >= targetLen || tz0 >= targetLen) {
        return {FINISHED};
    }

    if constexpr (Size == Size::X1) { // leaf
        if (!node.node) {
            return SubtreeResult{MISS};
        }
        double len;
        if (tx0 > ty0 && tx0 > tz0) {
            len = tx0; // PLANE YZ
        } else if (ty0 > tz0) {
            len = ty0; // PLANE XZ
        } else {
            len = tz0; // PLANE XY
        }
        return SubtreeResult{HIT, len};
    } else {
        if (node.empty()) {
            // we know that all the leafs are empty
            return {MISS};
        }

        auto m = [](double origin, int nodeMin, int nodeMax, double t0, double t1) {
            constexpr double inf = std::numeric_limits<double>::infinity();
            if (t0 == inf | t1 == inf) {
                // 3.3
                const int center = (nodeMin + nodeMax) / 2;
                return origin < center ? inf : -inf;
            } else {
                return 0.5 * (t0 + t1);
            }

        };
        const double txm = m(origin.x, node.minX(), node.maxX(), tx0, tx1);
        const double tym = m(origin.y, node.minY(), node.maxY(), ty0, ty1);
        const double tzm = m(origin.z, node.minZ(), node.maxZ(), tz0, tz1);

        int currNode = first_node(tx0, ty0, tz0, txm, tym, tzm);
        do {
            switch (currNode) {
                case 0:
                    if (auto result = proc_subtree(a, origin, targetLen, tx0, ty0, tz0, txm, tym, tzm, daughter(node, a)); result.type != MISS) return result;
                    currNode = new_node(txm,4,tym,2,tzm,1);
                    break;
                case 1:
                    if (auto result = proc_subtree(a, origin, targetLen, tx0, ty0, tzm, txm, tym, tz1, daughter(node, 1 ^ a)); result.type != MISS) return result;
                    currNode = new_node(txm,5,tym,3,tz1,8);
                    break;
                case 2:
                    if (auto result = proc_subtree(a, origin, targetLen, tx0, tym, tz0, txm, ty1, tzm, daughter(node, 2 ^ a)); result.type != MISS) return result;
                    currNode = new_node(txm,6,ty1,8,tzm,3);
                    break;
                case 3:
                    if (auto result = proc_subtree(a, origin, targetLen, tx0, tym, tzm, txm, ty1, tz1, daughter(node, 3 ^ a)); result.type != MISS) return result;
                    currNode = new_node(txm,7,ty1,8,tz1,8);
                    break;
                case 4:
                    if (auto result = proc_subtree(a, origin, targetLen, txm, ty0, tz0, tx1, tym, tzm, daughter(node, 4 ^ a)); result.type != MISS) return result;
                    currNode = new_node(tx1,8,tym,6,tzm,5);
                    break;
                case 5:
                    if (auto result = proc_subtree(a, origin, targetLen, txm, ty0, tzm, tx1, tym, tz1, daughter(node, 5 ^ a)); result.type != MISS) return result;
                    currNode = new_node(tx1,8,tym,7,tz1,8);
                    break;
                case 6:
                    if (auto result = proc_subtree(a, origin, targetLen, txm, tym, tz0, tx1, ty1, tzm, daughter(node, 6 ^ a)); result.type != MISS) return result;
                    currNode = new_node(tx1,8,ty1,8,tzm,7);
                    break;
                case 7:
                    if (auto result = proc_subtree(a, origin, targetLen, txm, tym, tzm, tx1, ty1, tz1, daughter(node, 7 ^ a)); result.type != MISS) return result;
                    return {MISS};
            }
        } while (currNode < 8);
        return {MISS};
    }
}

// returns none if we didn't intersect at all
std::optional<RaytraceResult> raytrace16x(uint8_t a, Ray ray, double targetLen, const Node<Size::X16>& node) {
    // idk I copy/pasted this from stackoverflow
    // IEEE stability fix
    const double divx = 1 / ray.dir.x;
    const double divy = 1 / ray.dir.y;
    const double divz = 1 / ray.dir.z;

    const double tx0 = (node.minX() - ray.origin.x) * divx;
    const double tx1 = (node.maxX() - ray.origin.x) * divx;
    const double ty0 = (node.minY() - ray.origin.y) * divy;
    const double ty1 = (node.maxY() - ray.origin.y) * divy;
    const double tz0 = (node.minZ() - ray.origin.z) * divz;
    const double tz1 = (node.maxZ() - ray.origin.z) * divz;

    using enum SubtreeResultType;
    // condition 10
    const auto tmin = max(tx0, ty0, tz0);
    const auto tmax = min(tx1, ty1, tz1);
    if (tmin <= tmax) {
        if (auto result = proc_subtree(a, ray.origin, targetLen, tx0, ty0, tz0, tx1, ty1, tz1, node); result.type != MISS) {
            if (result.type == HIT) {
                const Ray unreflect = reflectRay(ray, center(node), a);

                if (result.len < 0.0) {
                    // if len is negative, then the origin was inside an occupied x1, therefore, the hit
                    // position should be the origin.
                    return Hit{unreflect.origin};
                }

                return Hit{
                    unreflect.origin.x + unreflect.dir.x * result.len,
                    unreflect.origin.y + unreflect.dir.y * result.len,
                    unreflect.origin.z + unreflect.dir.z * result.len
                };
            } else {
                return Finished{};
            }
        } else {
            const Plane exit = exitPlane(tx1, ty1, tz1);
            return Miss{exit};
        }
    } else {
        // didn't intersect the node, this means we did something wrong
        if (tmin != tmax) {
            assert(!"this shit broke");
        }
        // we hit the corner precisely
        return {};
    }
}

Node<Size::X16> x16Node(const Chunk& chunk, const BlockPos& pos) {
    return {
        {
            .x = pos.x & ~15,
            .y = pos.y & ~15,
            .z = pos.z & ~15
        },
        &chunk.getX16(pos.y),
        chunk.isEmpty<Size::X16>(0, pos.y, 0)
    };
}

// returns true if there is line of sight
RaytraceResult raytrace(Context& ctx, const Vec3& from, const Vec3& to, bool airIfFakeChunk) {
    const auto [ray, targetLen] = computeRay(from, to);
    // the algorithm only works in positive directions so we need to reflect around the target point
    uint8_t a = 0;
    if (ray.dir.x < 0.0) {
        a |= 4;
    }
    if (ray.dir.y < 0.0) {
        a |= 2;
    }
    if (ray.dir.z < 0.0) {
        a |= 1;
    }
    const BlockPos realOriginBlock = vecToBlockPos(from);
    auto firstNode = x16Node(getOrGenChunk(ctx, ctx.executors[0], realOriginBlock.toChunkPos(), airIfFakeChunk), realOriginBlock);

    Node<Size::X16> currentNode = firstNode;
    while (true) {
        Ray reflectedRay = reflectRay(ray, center(currentNode), a);
        auto result = raytrace16x(a, reflectedRay, targetLen, currentNode);
        if (!result) {
            std::cerr << "raytrace whiffed" << std::endl;
            exit(696969);
        }
        if (!std::holds_alternative<Miss>(result.value())) {
            return result.value();
        }

        const Plane plane = std::get<Miss>(result.value()).exitPlane;
        BlockPos neighborPos = {currentNode.x, currentNode.y, currentNode.z};
        switch (plane) {
            case Plane::XY:
                neighborPos.z += (a & 1) ? -16 : 16;
                break;
            case Plane::XZ:
                neighborPos.y += (a & 2) ? -16 : 16;
                break;
            case Plane::YZ:
                neighborPos.x += (a & 4) ? -16 : 16;
                break;
        }
        currentNode = x16Node(getOrGenChunk(ctx, ctx.executors[0], neighborPos.toChunkPos(), airIfFakeChunk), neighborPos);
    }
}

size_t lastVisibleNode(Context& ctx, const std::vector<BlockPos>& path, size_t currentNode) {
    if (currentNode == path.size() - 1) return currentNode;
    const BlockPos fromBlock = path[currentNode];
    const Vec3 from = blockPosToVec(fromBlock);
    size_t lastVisible = currentNode + 1; // can assume that the next node is always visible from the previous
    for (auto i = lastVisible + 1; i < path.size(); i++) {
        const auto& currentBlock = path[i];
        if (fromBlock == currentBlock) continue; // apparently the pathfinder can produce 2 consecutive equal points and that breaks the raytracer
        const auto result = raytrace(ctx, from, blockPosToVec(currentBlock), false);
        if (std::holds_alternative<Hit>(result)) {
            return lastVisible;
        }
        lastVisible = i;
    }
    return lastVisible;
}

std::vector<BlockPos> refine(Context& ctx, const std::vector<BlockPos>& path) {
    ChunkGenExec exec;
    std::vector<BlockPos> out;
    for (size_t i = 0; i < path.size(); i = lastVisibleNode(ctx, path, i)) {
        out.push_back(path[i]);
        if (i == path.size() - 1) break;
    }
    return out;
}
