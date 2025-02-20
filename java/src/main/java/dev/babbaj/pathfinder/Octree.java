package dev.babbaj.pathfinder;

import sun.misc.Unsafe;

import java.lang.reflect.Field;

@SuppressWarnings("sunapi")
public class Octree {
    private static final Unsafe UNSAFE;

    static {
        try {
            Field f = Unsafe.class.getDeclaredField("theUnsafe");
            f.setAccessible(true);
            UNSAFE = (Unsafe) f.get(null);
        } catch (ReflectiveOperationException ex) {
            throw new RuntimeException(ex);
        }
    }

    private static final long X2_INDEX_PTR = NetherPathfinder.getX2Index();

    public static final int SIZEOF_X2 = 1;
    public static final int SIZEOF_X4 = SIZEOF_X2 * 8;
    public static final int SIZEOF_X8 = SIZEOF_X4 * 8;
    public static final int SIZEOF_X16 = SIZEOF_X8 * 8;
    public static final int SIZEOF_CHUNK = SIZEOF_X16 * 24;

    public static int x16Index(int y) {
        return y >> 4;
    }

    public static int x8Index(int x, int y, int z) {
        return ((x & 8) >> 1) | ((y & 8) >> 2) | ((z & 8) >> 3);
    }

    public static int x4Index(int x, int y, int z) {
        return ((x & 4)) | ((y & 4) >> 1) | ((z & 4) >> 2);
    }

    public static int x2Index(int x, int y, int z) {
        return ((x & 2) << 1) | ((y & 2)) | ((z & 2) >> 1);
    }

    public static int bitIndex(int x, int y, int z) {
        return ((x & 1) << 2) | ((y & 1) << 1) | ((z & 1));
    }

    private static long getX2Ptr(long chunk, int x, int y, int z) {
        //final int offset = (x16Index(y) * SIZEOF_X16) + (x8Index(x, y, z) * SIZEOF_X8) + (x4Index(x, y, z) * SIZEOF_X4) + (x2Index(x, y, z) * SIZEOF_X2);

        // std::array<std::array<std::array<uint16_t, 192>, 8>, 8>
        // auto x2Idx = X2_INDEX[x/2][z/2][y/2];
        // std::array<uint16_t, 192> = 384 bytes
        // std::array<std::array<uint16_t, 192>, 8> = 3072 bytes
        final int indexOffset = (3072 * (x/2)) + (384 * (z/2)) + (2 * (y/2));
        // uint16_t to signed int
        final int offset = UNSAFE.getShort(X2_INDEX_PTR + indexOffset);
        return chunk + offset;
    }

    // must be chunk relative coords
    public static void setBlock(long pointer, int dimension, int x, int y, int z, boolean solid) {
        y -= NetherPathfinder.DIMENSION_MIN_Y[dimension];
        final long x2Ptr = getX2Ptr(pointer, x, y, z);
        final int bit = bitIndex(x, y, z);
        byte x2 = UNSAFE.getByte(x2Ptr);

        if (solid) {
            x2 |= (1 << bit);
        } else {
            x2 &= ~(1 << bit);
        }
        UNSAFE.putByte(x2Ptr, x2);
    }

    public static void initBlock(long pointer, int dimension, int x, int y, int z, boolean solid) {
        y -= NetherPathfinder.DIMENSION_MIN_Y[dimension];
        final long x2Ptr = getX2Ptr(pointer, x, y, z);
        final int bit = bitIndex(x, y, z);
        byte x2 = UNSAFE.getByte(x2Ptr);

        // this is more likely to be branchless than setBlock
        int b = solid ? 1 : 0;
        x2 |= (b << bit);

        UNSAFE.putByte(x2Ptr, x2);
    }

    public static boolean getBlock(long pointer, int dimension, int x, int y, int z) {
        y -= NetherPathfinder.DIMENSION_MIN_Y[dimension];
        final long x2Ptr = getX2Ptr(pointer, x, y, z);
        final int bit = bitIndex(x, y, z);
        final byte x2 = UNSAFE.getByte(x2Ptr);
        return ((x2 >> bit) & 1) != 0;
    }
}
