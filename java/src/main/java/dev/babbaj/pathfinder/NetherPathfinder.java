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

    public static native boolean setBlockState(long context, int x, int y, int z, boolean state);

    public static native long getChunkPointer(long context, int x, int z);

    public static native PathSegment pathFind(long context, int x1, int y1, int z1, int x2, int y2, int z2, boolean atLeastX4, int failTimeoutInMillis);

    public static native long[] refinePath(long context, long[] blocks);


    private static native void raytrace0(long context, boolean assumeFakeChunksAreAir, int inputs, double[] start, double[] end, boolean[] hitsOut, double[] hitPosOutCanBeNull);

    public static void raytrace(long context, boolean assumeFakeChunksAreAir, int inputs, double[] start, double[] end, boolean[] hitsOut, double[] hitPosOutCanBeNull) {
        if (start.length < (inputs * 3) || end.length < (inputs * 3) || hitsOut.length < inputs || (hitPosOutCanBeNull != null && hitPosOutCanBeNull.length < (inputs * 3))) {
            throw new IllegalArgumentException("Bad array lengths idiot");
        }
        raytrace0(context, assumeFakeChunksAreAir, inputs, start, end, hitsOut, hitPosOutCanBeNull);
    }
    private static native int isVisibleMulti0(long context, boolean assumeFakeChunksAreAir, int inputs, double[] start, double[] end, boolean anyIfTrueElseAll);

    public static int isVisibleMulti(long context, boolean assumeFakeChunksAreAir, int inputs, double[] start, double[] end, boolean anyIfTrueElseAll) {
        if (start.length < (inputs * 3) || end.length < (inputs * 3)) {
            throw new IllegalArgumentException("Bad array lengths idiot");
        }
        return isVisibleMulti0(context, assumeFakeChunksAreAir, inputs, start, end, anyIfTrueElseAll);
    }

    public static native boolean isVisible(long context, boolean assumeFakeChunksAreAir, double x1, double y1, double z1, double x2, double y2, double z2);

    public static native boolean cancel(long context);

    static native long getX2Index();

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

    private static byte[] getNativeLib(final String libName) {
        try (
            final InputStream nativesRaw = NetherPathfinder.class.getClassLoader().getResourceAsStream("natives.xz");
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
        } catch (IOException ex) {
            throw new UncheckedIOException(ex);
        }
        return null;
    }

    static {
        final String libName = getNativeLibName();
        final byte[] libBytes = getNativeLib(libName);
        Objects.requireNonNull(libBytes, "Failed to find pathfinder library (" + libName + ")");

        final String[] split = libName.split("\\.");
        final Path tempFile;
        try {
            tempFile = Files.createTempFile(split[0], "." + split[1]);
        } catch (IOException ex) {
            throw new UncheckedIOException(ex);
        }
        System.out.println("Created temp file at " + tempFile.toAbsolutePath());

        try {
            Files.write(tempFile, libBytes);
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
