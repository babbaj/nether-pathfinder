package dev.babbaj.pathfinder;


import java.io.DataInputStream;
import java.io.IOException;

class Buffer {
    int size;
    byte[] array;

    Buffer(int initialSize) {
        this.size = 0;
        this.array = new byte[initialSize];
    }

    int allocateHowMuchForNeededSize(int newSize) {
        int l = array.length;
        while (l < newSize) {
            l *= 2;
        }
        return l;
    }

    void resize(int newSize) {
        int newCapacity = allocateHowMuchForNeededSize(newSize);
        byte[] newBuf = new byte[newCapacity];
        System.arraycopy(this.array, 0, newBuf, 0, this.array.length);
        this.array = newBuf;
    }

    void appendByte(byte b) {
        if (size + 1 > array.length) {
            resize(size + 1);
        }
        this.array[size] = b;
        size++;
    }

    void read(DataInputStream stream, int len) throws IOException {
        if (size + len > array.length) {
            resize(size + len);
        }
        stream.readFully(this.array, this.size, len);
        this.size += len;
    }
}
