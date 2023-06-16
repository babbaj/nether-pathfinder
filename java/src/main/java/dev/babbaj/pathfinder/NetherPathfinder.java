package dev.babbaj.pathfinder;

import java.io.IOException;
import java.io.InputStream;
import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.StandardCopyOption;
import java.util.Objects;

public class NetherPathfinder {
    public static native long[] pathFind(long seed, boolean fine, boolean raytrace, int x1, int y1, int z1, int x2, int y2, int z2) throws IllegalArgumentException;
    public static native boolean cancel();

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