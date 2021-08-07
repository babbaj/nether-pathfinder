package main

import "sync"

type SafeChunkMap struct {
	data map[ChunkPos]Chunk
	lock sync.RWMutex
}

func makeSafeChunkMap() *SafeChunkMap {
	return &SafeChunkMap{data: make(map[ChunkPos]Chunk)}
}

func (chunks *SafeChunkMap) importFromInitialPathfinder(segment SegmentFromInitialPathfinder) {
	// yay batch of free chunks
	chunks.lock.Lock()
	defer chunks.lock.Unlock()
	for pos, chunk := range segment.GeneratedChunks {
		chunks.data[pos] = chunk
	}
}

func (chunks *SafeChunkMap) getIfExists(pos ChunkPos) (chunk Chunk, ok bool) {
	chunks.lock.RLock()
	defer chunks.lock.RUnlock()
	chunk, ok = chunks.data[pos]
	return
}

func (chunks *SafeChunkMap) getOrGenerate(pos ChunkPos) Chunk {
	if chunk, ok := chunks.getIfExists(pos); ok {
		return chunk
	}
	chunks.lock.Lock()
	defer chunks.lock.Unlock()
	if chunk, ok := chunks.data[pos]; ok { // this could have happened between giving up RUnlock and grabbing WLock (have to do it that way otherwise it would block forever)
		return chunk
	}
	chunk := generate(pos)
	chunks.data[pos] = chunk
	return chunk
}

type CompletableFuture struct {
	data chan Segment // abuse a channel's blocking nature to act like a future lol
}

func callAsync(supplier func() Segment) CompletableFuture {
	ret := CompletableFuture{data: make(chan Segment)}
	go func() {
		defer close(ret.data)
		ret.data <- supplier()
	}()
	return ret
}

func (c CompletableFuture) getBlockingly() Segment {
	return <-c.data
}

func (pos BlockPos) toChunkPos() ChunkPos {
	return ChunkPos{pos.x >> 4, pos.z >> 4}
}

func (pos BlockPos) plus(pos2 BlockPos) BlockPos {
	return BlockPos{pos.x + pos2.x, pos.y + pos2.y, pos.z + pos2.z}
}
