package dev.babbaj.pathfinder;

import java.io.IOException;
import java.io.InputStream;
import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.StandardCopyOption;
import java.util.Objects;

public class NetherPathfinder {

    public static native long newContext(long seed);
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

    public static native PathSegment pathFind(long context, int x1, int y1, int z1, int x2, int y2, int z2, boolean atLeastX4, int failTimeoutInMillis);

    public static native long[] refinePath(long context, long[] blocks);


    private static native void raytrace0(long context, boolean assumeFakeChunksAreAir, int inputs, long[] start, long[] end, boolean[] hitsOut, double[] hitPosOutCanBeNull);

    public static void raytrace(long context, boolean assumeFakeChunksAreAir, int inputs, long[] start, long[] end, boolean[] hitsOut, double[] hitPosOutCanBeNull) {
        if (start.length != inputs || end.length != inputs || hitsOut.length != inputs || (hitPosOutCanBeNull != null && hitPosOutCanBeNull.length != inputs)) {
            throw new IllegalArgumentException("Bad array lengths idiot");
        }
        raytrace0(context, assumeFakeChunksAreAir, inputs, start, end, hitsOut, hitPosOutCanBeNull);
    }

    public static native boolean cancel(long context);

    // TODO: convenient function for computing a full path

    private static String getNativeLibName() {
        String osName = System.getProperty("os.name").toLowerCase();
        String osArch = System.getProperty("os.arch").toLowerCase();
        final int bits = Integer.parseInt(System.getProperty("sun.arch.data.model"));
        if (bits != 64) throw new IllegalStateException(bits + " not supported");
        final String arch;
        if (osArch.contains("arm") || osArch.contains("aarch64")) {
            arch = "aarch64";
        } else if (osArch.equals("x86_64") || osArch.equals("amd64")) {
            arch = "x86_64";
        } else {
            throw new IllegalStateException("Unsupported architecture: " + osArch);
        }

        if (osName.contains("linux")) {
            return "libnether_pathfinder-" + arch + ".so";
        } else if (osName.contains("windows")) {
            return "nether_pathfinder-x86_64.dll";
        } else if (osName.contains("mac")) {
            return "libnether_pathfinder-" + arch + ".dylib";
        } else {
            throw new IllegalStateException("Unsupported os: " + osName);
        }
    }

    static {
        final String libName = getNativeLibName();
        final String library = "native/" + libName;
        final InputStream libraryStream = NetherPathfinder.class.getClassLoader().getResourceAsStream(library);
        Objects.requireNonNull(libraryStream, "Failed to find pathfinder library (" + library + ")");

        final String[] split = libName.split("\\.");
        final Path tempFile;
        try {
            tempFile = Files.createTempFile(split[0], "." + split[1]);
        } catch (IOException ex) {
            throw new RuntimeException(ex);
        }
        System.out.println("Created temp file at " + tempFile.toAbsolutePath());
        try {
            Files.copy(libraryStream, tempFile, StandardCopyOption.REPLACE_EXISTING);
            System.load(tempFile.toAbsolutePath().toString());
        } catch (IOException ex) {
            throw new RuntimeException(ex);
        } finally {
            try {
                Files.delete(tempFile);
            } catch (IOException ex) {
                System.out.println("trolled");
                ex.printStackTrace();
            }
            tempFile.toFile().delete();
            tempFile.toFile().deleteOnExit();
        }

        System.out.println("Loaded shared library");
    }
}