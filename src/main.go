package main

type BlockPos struct {
	x int
	y int
	z int
}

type ChunkPos struct {
	x int
	z int
}

type Chunk interface {
	Empty(pos BlockPos) bool // 1x

	EmptyAt4X(pos BlockPos) bool // 4x
}

func generate(pos ChunkPos) Chunk {
	panic("already implemented")
}

type Segment []BlockPos

type SegmentFromInitialPathfinder struct {
	// not a dense path. it skips blocks, because it goes from NodePos to NodePos, some of which are 2x and some of which are 16x
	Path Segment

	// return this instead of discarding it, because the refiner will use it uwu
	GeneratedChunks map[ChunkPos]Chunk // IMPORTANT: the initial pathfinder is the bottleneck. don't make it get into mutex-contention with the refiner on the loaded chunk map! so, only merge them once it's DONE
}

func findPath0(start BlockPos, goal BlockPos) SegmentFromInitialPathfinder {
	panic("already implemented")
}

func makeSegments(start BlockPos, goal BlockPos, output chan SegmentFromInitialPathfinder) { // this is the existing main top-level function findPath, that calls findPath0 in a loop
	defer close(output)
	for {
		path := findPath0(start, goal)
		output <- path
		endOfThisSegment := path.Path[len(path.Path)-1]
		if endOfThisSegment == goal {
			break
		}
		start = endOfThisSegment // start from the end of the previous one
	}
}

func handleInitialSegments(input chan SegmentFromInitialPathfinder, output chan Segment, chunks *SafeChunkMap) { // doesn't change the places where one segment ends and the next begins
	defer close(output)
	for segment := range input { // automatically exits loop (and therefore does the defer close) once the input is closed
		chunks.importFromInitialPathfinder(segment)
		output <- segment.Path
	}
}

func segmentRefiner(lengths int, offset int, input chan Segment, output chan Segment, chunks *SafeChunkMap) { // WILL change the places where one segment ends and the next begins

	distanceUntilNextSlice := offset // first segment has length "offset"
	var prevPos *BlockPos
	var sliceStartsAt *BlockPos

	outputInOrder := make(chan CompletableFuture, 16) // this channel needs a nonzero buffer so that we can use more than one core at a time, because callAsync will write to this alongside starting the goroutine to refine the slice
	go func() {
		defer close(output)
		for completableFuture := range outputInOrder {
			output <- completableFuture.getBlockingly()
		}
	}()
	// ^ the point of doing this weird channel-of-futures (each of which is a channel) is so that calculating the segments is multithreaded in order
	// i.e. the first slice is being A*'d at the same time as the second slice... but we still need the output of the first slice to stay before the second one, even if the second one completes first

	doSlice := func() {
		// slice from sliceStartsAt to prevPos inclusive
		// we **throw away** the entire path between the two
		// note that these could have come from different segments, or the same segment, it doesn't matter
		start := *sliceStartsAt
		end := *prevPos
		outputInOrder <- callAsync(func() Segment {
			return refine(start, end, chunks)
		})
		sliceStartsAt = prevPos
		distanceUntilNextSlice += lengths // all segments after the first have length "lengths"
	}
	for segment := range input {
		for _, pos := range segment {
			if prevPos == nil {
				prevPos = &pos // very first position! this will equal the overall "start"
				sliceStartsAt = &pos
			}
			// end of one segment matches beginning of next, but it's okay if pos==*prevPos because distanceManhattan will be 0, so it will pass by silently
			distanceUntilNextSlice -= distanceManhattan(*prevPos, pos) // because when it comes from the initial pathfinder, the X system means they won't always be right next to each other
			prevPos = &pos
			if distanceUntilNextSlice <= 0 {
				doSlice()
			}
		}
	}
	if *prevPos != *sliceStartsAt {
		doSlice() // don't forget the last block in the path
	}
}

func mainFunctionToBeCalledThroughJNI(start BlockPos, goal BlockPos) []BlockPos {
	initialPathSegments := make(chan SegmentFromInitialPathfinder) // LinkedBlockingQueue<SegmentFromInitialPathfinder>
	go makeSegments(start, goal, initialPathSegments)
	chunkMap := makeSafeChunkMap()
	preRefinedSegments := make(chan Segment)
	go handleInitialSegments(initialPathSegments, preRefinedSegments, chunkMap)

	refineHelper := func(lengths int, offset int, input chan Segment) chan Segment {
		out := make(chan Segment)
		go segmentRefiner(lengths, offset, input, out, chunkMap)
		return out
	}

	refined := preRefinedSegments
	refined = refineHelper(128, 64, refined)
	refined = refineHelper(128, 128, refined)
	refined = refineHelper(128, 64, refined)
	refined = refineHelper(128, 128, refined) // CPUs have a lot of cores, might as well use them

	prev := start
	ret := []BlockPos{start}
	for segment := range refined {
		for _, pos := range segment {
			if pos != prev {
				prev = pos
				ret = append(ret, pos)
			}
		}
	}
	return ret
}

func distanceManhattan(a BlockPos, b BlockPos) int {
	return abs(a.x-b.x) + abs(a.y-b.y) + abs(a.z-b.z)
}

func abs(i int) int {
	if i < 0 {
		return -i
	} else {
		return i
	}
}
