#include "Utils.h"
#include "ChunkGeneratorHell.h"
#include "ChunkGen.h"

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

template<Size size>
using NodeType = decltype([] {
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
    //if constexpr (size == Size::X1) {
    //    return std::declval<bool>();
    //}
}());

constexpr Size nextLowerSize(Size size) {
    return static_cast<Size>(static_cast<int>(size) - 1);
}

template<typename Node>
constexpr decltype(auto) getDaughter(const Node& node, int i) {
    if constexpr (std::is_same_v<Node, x2_t>) {
        // im pretty confident the lower bits are closer to the origin
        return (node >> i) != 0;
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

    constexpr int width() const {
        return ::width(sz);
    }

    constexpr enum Size size() const {
        return sz;
    }

    constexpr bool empty() const {
        return isZero(static_cast<const Self*>(this)->node);
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

    Node<nextLowerSize(sz)> daughter(int i) const {
        // I hope this is actually how the chunk is actually organized
        // See figure 1
        constexpr auto halfWidth = ::width(nextLowerSize(sz));
        return {
            {
                this->x + (i & 4 ? halfWidth : 0), // 4,5,6,7
                this->y + (i & 2 ? halfWidth : 0), // 2,3,6,7
                this->z + (i & 1 ? halfWidth : 0)  // 1,3,5,7
            },
            getDaughter(*this->node, i)
        };
    }
};

template<>
struct Node<Size::X1> : NodeBase<Node<Size::X1>, Size::X1> {
    bool node;
};

constexpr double chunkXMin(ChunkPos pos) {
    return static_cast<double>(pos.x);
}
constexpr double chunkXMax(ChunkPos pos) {
    return static_cast<double>(pos.x + 16);
}
constexpr double chunkZMin(ChunkPos pos) {
    return static_cast<double>(pos.z);
}
constexpr double chunkZMax(ChunkPos pos) {
    return static_cast<double>(pos.z + 16);
}

double max(double x, double y, double z) {
    return std::max(std::max(x, y), z);
}

double min(double x, double y, double z) {
    return std::min(std::min(x, y), z);
}

Vec3 r(const Ray& ray, double t) {
    return {
        ray.origin.x + t * ray.dir.x,
        ray.origin.y + t * ray.dir.y,
        ray.origin.z + t * ray.dir.z
    };
}

enum class Plane {
    YZ,
    XZ,
    XY
};

// we intersected with a solid block
struct Hit {};
// line of sight confirmed
struct Finished {};
// we got to the other side of an x16 without hitting anything
struct Miss {
    Plane exitPlane;
};
using RaytraceResult = std::variant<Hit, Finished, Miss>;

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

int new_node(double txm, int x, double tym, int y, double tzm, int z) {
    if (txm < tym) {
        if (txm < tzm) return x;  // YZ plane
    } else {
        if (tym < tzm) return y;  // XZ plane
    }
    return z; // XY plane;
}

enum class SubtreeResult {
    HIT,
    FINISHED,
    MISS
};

template<Size Size>
SubtreeResult proc_subtree(uint8_t a, double targetLen, double tx0, double ty0, double tz0, double tx1, double ty1, double tz1, const Node<Size>& node) {
    using enum SubtreeResult;
    // if this node is behind us
    if (tx1 < 0.0 || ty1 < 0.0 || tz1 < 0.0) {
        return MISS;
    }

    // not sure if this is correct
    if (tx0 >= targetLen || ty0 >= targetLen || tz0 >= targetLen) {
        return FINISHED;
    }

    if constexpr (Size == Size::X1) { // leaf
        return node.node ? HIT : MISS;
    } else {
        // TODO: should be possible to find all of the empty children at once so we can skip some children in the case it's not full or empty
        if (node.empty()) {
            // we know that all the leafs are empty
            return MISS;
        }

        const double txm = 0.5 * (tx0 + tx1);
        const double tym = 0.5 * (ty0 + ty1);
        const double tzm = 0.5 * (tz0 + tz1);

        // shouldn't be necessary to go in order

        int currNode = first_node(tx0, ty0, tz0, txm, tym, tzm);
        do {
            switch (currNode) {
                case 0:
                    if (auto result = proc_subtree(a, targetLen, tx0, ty0, tz0, txm, tym, tzm, node.daughter(a)); result != MISS) return result;
                    currNode = new_node(txm,4,tym,2,tzm,1);
                    break;
                case 1:
                    if (auto result = proc_subtree(a, targetLen, tx0, ty0, tzm, txm, tym, tz1, node.daughter(1 ^ a)); result != MISS) return result;
                    currNode = new_node(txm,5,tym,3,tz1,8);
                    break;
                case 2:
                    if (auto result = proc_subtree(a, targetLen, tx0, tym, tz0, txm, ty1, tzm, node.daughter(2 ^ a)); result != MISS) return result;
                    currNode = new_node(txm,6,ty1,8,tzm,3);
                    break;
                case 3:
                    if (auto result = proc_subtree(a, targetLen, tx0, tym, tzm, txm, ty1, tz1, node.daughter(3 ^ a)); result != MISS) return result;
                    currNode = new_node(txm,7,ty1,8,tz1,8);
                    break;
                case 4:
                    if (auto result = proc_subtree(a, targetLen, txm, ty0, tz0, tx1, tym, tzm, node.daughter(4 ^ a)); result != MISS) return result;
                    currNode = new_node(tx1,8,tym,6,tzm,5);
                    break;
                case 5:
                    if (auto result = proc_subtree(a, targetLen, txm, ty0, tzm, tx1, tym, tz1, node.daughter(5 ^ a)); result != MISS) return result;
                    currNode = new_node(tx1,8,tym,7,tz1,8);
                    break;
                case 6:
                    if (auto result = proc_subtree(a, targetLen, txm, tym, tz0, tx1, ty1, tzm, node.daughter(6 ^ a)); result != MISS) return result;
                    currNode = new_node(tx1,8,ty1,8,tzm,7);
                    break;
                case 7:
                    if (auto result = proc_subtree(a, targetLen, txm, tym, tzm, tx1, ty1, tz1, node.daughter(7 ^ a)); result != MISS) return result;
                    currNode = 8;
                    break;
            }
        } while (currNode < 8);
        return MISS;
    }
}

// returns none if we didn't intersect at all
std::optional<RaytraceResult> raytrace16x(uint8_t a, Ray ray, double targetLen, const Node<Size::X16>& node) {
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

    using enum SubtreeResult;
    // condition 10
    auto tmin = max(tx0, ty0, tz0);
    auto tmax = min(tx1, ty1, tz1);
    if (tmin <= tmax) {
        if (auto result = proc_subtree(a, targetLen, tx0, ty0, tz0, tx1, ty1, tz1, node); result != MISS) {
            if (result == HIT) {
                return Hit{}; // we could return the point where the hit happened but it's not useful
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

template<typename T>
T reflect(T x, T target) {
    return target - (x - target);
}

template<typename Vec>
Vec reflect(uint8_t a, const Vec& vec, const Vec& target) {
    Vec out = vec;
    if (a & 4) {
        out.x = reflect(out.x, target.x);
    }
    if (a & 2) {
        out.y = reflect(out.y, target.y);
    }
    if (a & 1) {
        out.z = reflect(out.z, target.z);
    }
    return out;
}

// 3rd time I've copy/pasted this
Chunk& getOrGenChunk(const BlockPos& pos, const ChunkGeneratorHell& gen, ChunkGenExec& exec, cache_t & cache) {
    const ChunkPos chunkPos = pos.toChunkPos();
    auto it = cache.find(chunkPos);
    if (it != cache.end()) {
        return *it->second;
    } else {
        std::unique_ptr ptr = std::make_unique<Chunk>();
        auto& chunk = *ptr;
        gen.generateChunk(chunkPos.x, chunkPos.z, *ptr, exec);
        cache.emplace(chunkPos, std::move(ptr));
        return chunk;
    }
}

Plane exitPlaneFromRealRay(const Ray& ray, const Node<Size::X16>& node) {
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

    return exitPlane(tx1, ty1, tz1);
}

// returns true if there is line of sight
bool raytrace(const Vec3& from, const Vec3& to, const ChunkGeneratorHell& gen, ChunkGenExec& exec, cache_t& cache) {
    auto [ray, targetLen] = computeRay(from, to);
    // the algorithm only works in positive directions so we need to reflect around the target point
    uint8_t a = 0;
    if (ray.dir.x < 0.0) {
        ray.origin.x = reflect(ray.origin.x, to.x);
        ray.dir.x = -ray.dir.x;
        a |= 4;
    }
    if (ray.dir.y < 0.0) {
        ray.origin.y = reflect(ray.origin.y, to.y);
        ray.dir.y = -ray.dir.y;
        a |= 2;
    }
    if (ray.dir.z < 0.0) {
        ray.origin.z = reflect(ray.origin.z, to.z);
        ray.dir.z = -ray.dir.z;
        a |= 1;
    }
    const BlockPos realOriginBlock = vecToBlockPos(from);
    auto first16x = getOrGenChunk(realOriginBlock, gen, exec, cache).getX16(realOriginBlock.y);
    const BlockPos reflectedOriginBlock = vecToBlockPos(reflect(a, from, to));
    const auto firstNode = Node<Size::X16>{
        {
            reflectedOriginBlock.x & ~15,
            reflectedOriginBlock.y & ~15,
            reflectedOriginBlock.z & ~15
        },
        &first16x
    };

    Node<Size::X16> currentNode = firstNode;
    while (true) {
        auto result = raytrace16x(a, ray, targetLen, currentNode);
        if (!result) {
            std::cerr << "raytrace whiffed" << std::endl;
        }
        if (std::holds_alternative<Hit>(result.value())) {
            return false;
        }
        if (std::holds_alternative<Finished>(result.value())) {
            return true;
        }
        const Plane plane = std::get<Miss>(result.value()).exitPlane;
        BlockPos neighborPos = {currentNode.x, currentNode.y, currentNode.z};
        switch (plane) {
            case Plane::XY:
                neighborPos.z += 16;
                break;
            case Plane::XZ:
                neighborPos.y += 16;
                break;
            case Plane::YZ:
                neighborPos.x += 16;
                break;
        }
        // TODO: this reflection is wrong?
        const BlockPos realNeighborPos = vecToBlockPos(reflect(a, blockPosToVec(neighborPos), to));
        const x16_t& data = getOrGenChunk(realNeighborPos, gen, exec, cache).getX16(realNeighborPos.y);
        currentNode = Node<Size::X16>{
            neighborPos.x & ~15,
            neighborPos.y & ~15,
            neighborPos.z & ~15,
            &data
        };
    }
}

size_t lastVisibleNode(const std::vector<BlockPos>& path, size_t currentNode, const ChunkGeneratorHell& gen, ChunkGenExec& exec, cache_t& cache) {
    if (currentNode == path.size() - 1) return currentNode;
    const Vec3 from = blockPosToVec(path[currentNode]);
    size_t lastVisible = currentNode + 1; // can assume that the next node is always visible from the previous
    for (auto i = lastVisible; i < path.size(); i++) {
        const auto& currentBlock = path[i];
        if (!raytrace(from, blockPosToVec(currentBlock), gen, exec, cache)) {
            return lastVisible;
        }
        lastVisible = i;
    }
    return lastVisible;
}

std::vector<BlockPos> refine(const std::vector<BlockPos>& path, const ChunkGeneratorHell& gen, cache_t& cache) {
    ChunkGenExec exec;
    std::vector<BlockPos> out;
    for (size_t i = 0; i < path.size() - 1; i = lastVisibleNode(path, i, gen, exec, cache)) {
        out.push_back(path[i]);
    }
    return out;
}