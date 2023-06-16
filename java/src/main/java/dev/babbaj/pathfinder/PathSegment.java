package dev.babbaj.pathfinder;

public class PathSegment {
    public final boolean finished;
    public final long[] packed;


    public PathSegment(boolean finished, long[] packed) {
        this.finished = finished;
        this.packed = packed;
    }
}