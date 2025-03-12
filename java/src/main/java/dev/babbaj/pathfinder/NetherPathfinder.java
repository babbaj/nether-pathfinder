package dev.babbaj.pathfinder;

import org.tukaani.xz.XZInputStream;

import java.io.ByteArrayOutputStream;
import java.io.IOException;
import java.io.InputStream;
import java.io.UncheckedIOException;
import java.nio.file.*;
import java.util.Objects;
import java.util.zip.ZipEntry;
import java.util.zip.ZipInputStream;

public class NetherPathfinder {

    // How the raytracer will treat chunks that aren't actually observed
    public static int CACHE_MISS_GENERATE = 0;
    public static int CACHE_MISS_AIR = 1;
    public static int CACHE_MISS_SOLID = 2;

    public static int DIMENSION_OVERWORLD = 0;
    public static int DIMENSION_NETHER = 1;
    public static int DIMENSION_END = 2;

    // pass true to use the custom chunk allocator that will reduce memory usage and maybe be faster. false to just use new/delete
    // this is only supported on systems with 4k pages
    public static native long newContext(long seed, String baritoneCacheDirCanBeNull, int dimension, int maxHeight, boolean allocator);
    public static native void freeContext(long pointer);

    /*
    from BlockStateContainer
    private static int getIndex(int x, int y, int z)
    {
        return y << 8 | z << 4 | x;
    }

    chunkX and chunkZ are chunk coords, not block coords.
    This is currently not thread safe
    */
    public static native void insertChunkData(long context, int chunkX, int chunkZ, boolean[] data);

    public static native long getOrCreateChunk(long context, int x, int z);

    public static native long getChunkPointer(long context, int x, int z);

    // returns true if the chunk existed and the change was made
    public static native boolean setChunkState(long context, int x, int z, boolean fromJava);

    public static native boolean hasChunkFromJava(long context, int x, int z);

    public static native void cullFarChunks(long context, int chunkX, int chunkZ, int maxDistanceBlocks);

    public static native PathSegment pathFind(long context, int x1, int y1, int z1, int x2, int y2, int z2, boolean atLeastX4, boolean refine, int failTimeoutInMillis, boolean useAirIfChunkNotLoaded, double fakeChunkCost);

    private static native void raytrace0(long context, int fakeChunkMode, int inputs, double[] start, double[] end, boolean[] hitsOut, double[] hitPosOutCanBeNull);

    public static void raytrace(long context, int fakeChunkMode, int inputs, double[] start, double[] end, boolean[] hitsOut, double[] hitPosOutCanBeNull) {
        if (start.length < (inputs * 3) || end.length < (inputs * 3) || hitsOut.length < inputs || (hitPosOutCanBeNull != null && hitPosOutCanBeNull.length < (inputs * 3))) {
            throw new IllegalArgumentException("Bad array lengths idiot");
        }
        raytrace0(context, fakeChunkMode, inputs, start, end, hitsOut, hitPosOutCanBeNull);
    }
    private static native int isVisibleMulti0(long context, int fakeChunkMode, int inputs, double[] start, double[] end, boolean anyIfTrueElseAll);

    public static int isVisibleMulti(long context, int fakeChunkMode, int inputs, double[] start, double[] end, boolean anyIfTrueElseAll) {
        if (start.length < (inputs * 3) || end.length < (inputs * 3)) {
            throw new IllegalArgumentException("Bad array lengths idiot");
        }
        return isVisibleMulti0(context, fakeChunkMode, inputs, start, end, anyIfTrueElseAll);
    }

    public static native boolean isVisible(long context, int fakeChunkMode, double x1, double y1, double z1, double x2, double y2, double z2);

    public static native boolean cancel(long context);

    static native long getX2Index();

    // TODO: convenient function for computing a full path

    private static final boolean IS_LOADED;

    public static boolean isThisSystemSupported() {
        return IS_LOADED;
    }

    private static String getNativeLibName() {
        final int bits = Integer.parseInt(System.getProperty("sun.arch.data.model"));
        if (bits != 64) {
            throw new UnsupportedOperationException("Unsupported architecture (64-bit required)");
        }

        final String osName = System.getProperty("os.name").toLowerCase();
        final String osArch = System.getProperty("os.arch").toLowerCase();

        final String arch;
        if (osArch.contains("arm") || osArch.contains("aarch64")) {
            arch = "aarch64";
        } else if (osArch.equals("x86_64") || osArch.equals("amd64")) {
            arch = "x86_64";
        } else {
            throw new UnsupportedOperationException("Unsupported architecture: " + osArch);
        }

        if (osName.contains("linux")) {
            return "libnether_pathfinder-" + arch + ".so";
        } else if (osName.contains("windows")) {
            return "nether_pathfinder-" + arch + ".dll";
        } else if (osName.contains("mac")) {
            return "libnether_pathfinder-" + arch + ".dylib";
        } else {
            throw new UnsupportedOperationException("Unsupported operating system: " + osName);
        }
    }

    private static byte[] getNativeLib(final String libName) throws IOException {
        try (
            final InputStream nativesRaw = NetherPathfinder.class.getClassLoader().getResourceAsStream("natives.zip.xz");
            final XZInputStream nativesZx = new XZInputStream(nativesRaw);
            final ZipInputStream nativesZip = new ZipInputStream(nativesZx)
        ) {
            ZipEntry entry;
            while ((entry = nativesZip.getNextEntry()) != null) {
                if (!entry.getName().equals(libName)) {
                    continue;
                }

                final ByteArrayOutputStream byteStream = new ByteArrayOutputStream();
                final byte[] buffer = new byte[4096];

                int read;
                while ((read = nativesZip.read(buffer)) != -1) {
                    byteStream.write(buffer, 0, read);
                }

                return byteStream.toByteArray();
            }
        }
        throw new NullPointerException("Failed to find pathfinder library: " + libName);
    }

    private static void tryLoadLibrary() throws IOException {
        final String libName = getNativeLibName();
        final byte[] libBytes = getNativeLib(libName);

        final String[] split = libName.split("\\.");
        final Path tempFile = Files.createTempFile(split[0], "." + split[1]);
        System.out.println("[nether-pathfinder] Created temp file at " + tempFile.toAbsolutePath());

        try {
            Files.write(tempFile, libBytes);
            System.load(tempFile.toAbsolutePath().toString());
        } finally {
            try {
                Files.delete(tempFile);
            } catch (IOException ignored) {
                System.err.println("[nether-pathfinder] Failed to delete temp file");
//                System.out.println("trolled");
//                ex.printStackTrace();
            }
            if (!tempFile.toFile().delete()) {
                tempFile.toFile().deleteOnExit();
            }
        }
    }

    static {
        boolean loaded = false;
        try {
            tryLoadLibrary();
            System.out.println("[nether-pathfinder] Loaded shared library");
            loaded = true;
        } catch (Throwable e) {
            System.err.println("[nether-pathfinder] Failed to load shared library");
            e.printStackTrace();
        }
        IS_LOADED = loaded;
    }
}
