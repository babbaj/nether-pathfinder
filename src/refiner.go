package main

// ok so basically main.go is a whole lot of plumbing
// to hook up all of the refiner passes in the right order, while being as concurrent as possible

// this is what actually refines a path

var DIRS = []BlockPos{
	{1, 0, 0},
	{-1, 0, 0},
	{0, 1, 0},
	{0, -1, 0},
	{0, 0, 1},
	{0, 0, -1},
}

// basically just the existing findPath0 EXCEPT it can ONLY to X1
// TODO also, perhaps, maybe, something that makes it stay away from walls?
func refine(start BlockPos, goal BlockPos, safeChunks *SafeChunkMap) Segment {
	nodes := make(map[NodePos]*PathNode)
	getNodeAtPosition := func(pos NodePos) *PathNode {
		if node, ok := nodes[pos]; ok {
			return node
		}
		node := &PathNode{
			pos:                 pos,
			estimatedCostToGoal: heuristic(pos, goal),
		}
		if pos.pos != start {
			node.cost = 1000000000
		}
		node.combinedCost = node.cost + node.estimatedCostToGoal
		nodes[pos] = node
		return node
	}
	var openSet BinaryHeapOpenSet
	openSet.Insert(getNodeAtPosition(NodePos{start}))
	chunks := &ThreadLocalChunkLRUCache{chunks: safeChunks}
	for !openSet.IsEmpty() {
		curr := openSet.RemoveLowest()
		if curr.pos.pos == goal {
			return walkBackwards(curr)
		}
		for _, dir := range DIRS {
			neighborPos := curr.pos.pos.plus(dir)
			if chunks.getChunk(neighborPos.toChunkPos()).Empty(neighborPos) {
				edgeCost := 1000
				if !fullyClearArea(neighborPos, chunks) {
					edgeCost += 500 // stay away from walls idk
				}
				tentativeCost := curr.cost + edgeCost
				neighborNode := getNodeAtPosition(NodePos{neighborPos})
				if neighborNode.cost > tentativeCost {
					neighborNode.cost = tentativeCost
					neighborNode.combinedCost = neighborNode.cost + neighborNode.estimatedCostToGoal
					neighborNode.previous = curr
					if neighborNode.IsOpen() {
						openSet.Update(neighborNode)
					} else {
						openSet.Insert(neighborNode)
					}
				}
			}
		}
	}
	panic("somehow no path?")
}

func fullyClearArea(pos BlockPos, chunks *ThreadLocalChunkLRUCache) bool {
	for _, dir := range DIRS {
		p := pos.plus(dir)
		if !chunks.getChunk(p.toChunkPos()).EmptyAt4X(p) {
			return false
		}
	}
	return true
}

func walkBackwards(node *PathNode) Segment {
	ret := make([]BlockPos, 0)
	for node != nil {
		ret = append(ret, node.pos.pos)
		node = node.previous
	}
	for i, j := 0, len(ret)-1; i < j; i, j = i+1, j-1 { // reverse a slice, pasted from stackoverflow lol
		ret[i], ret[j] = ret[j], ret[i]
	}
	return ret
}

type NodePos struct {
	pos BlockPos
	// i guess you could reuse the existing NodePos and just have size be X1 lol
}

func heuristic(pos NodePos, goal BlockPos) int {
	return 999 * (abs(pos.pos.x-goal.x) + abs(pos.pos.z-goal.z))
}

type PathNode struct {
	pos                 NodePos
	estimatedCostToGoal int // floating point is SO CRINGE
	cost                int
	combinedCost        int
	previous            *PathNode
	heapPosition        int
}

func (node *PathNode) IsOpen() bool {
	return node.heapPosition > 0
}

type BinaryHeapOpenSet interface {
	RemoveLowest() *PathNode
	Insert(val *PathNode)
	Update(val *PathNode)
	IsEmpty() bool
}

const CACHE_ENTRIES = 4

type LRUEntry struct {
	pos   *ChunkPos // zero init would incorrectly cache (0,0) lol
	chunk Chunk
}

type ThreadLocalChunkLRUCache struct {
	chunks *SafeChunkMap
	cache  [CACHE_ENTRIES]LRUEntry
}

func (cache *ThreadLocalChunkLRUCache) getChunk(pos ChunkPos) Chunk {
	found := CACHE_ENTRIES - 1
	var ret *Chunk // i really just mean "optional" but in go this is like the only way to do an optional lol
	for i := 0; i < CACHE_ENTRIES; i++ {
		if cache.cache[i].pos != nil && *cache.cache[i].pos == pos {
			found = i
			ret = &cache.cache[i].chunk
			break
		}
	}
	for i := found; i > 0; i-- {
		cache.cache[i] = cache.cache[i-1]
	}
	if ret == nil {
		chunk := cache.chunks.getOrGenerate(pos)
		ret = &chunk
	}
	cache.cache[0].pos = &pos
	cache.cache[0].chunk = *ret
	return *ret
}
