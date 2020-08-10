#pragma once

#include <vector>
#include <memory>

#include "PathNode.h"

// pasted from baritone :-)
struct BinaryHeapOpenSet {
private:
    static constexpr int INITIAL_CAPACITY = 1024;

    std::vector<PathNode*> vector;
    int size = 0;

public:
    explicit BinaryHeapOpenSet(int size): vector(size) {}
    explicit BinaryHeapOpenSet(): BinaryHeapOpenSet(INITIAL_CAPACITY) {}


    [[nodiscard]] int getSize() const {
        return this->size;
    }

    [[nodiscard]] bool isEmpty() const {
        return this->size == 0;
    }

    // value must be created by new
    void insert(PathNode* value) {
        assert(value != nullptr);
        if (this->size >= this->vector.size() - 1) {
            this->vector.resize(this->vector.size() << 1);
            //array = Arrays.copyOf(array, array.length << 1);
        }
        this->size++;
        value->heapPosition = this->size;
        this->vector[size] = value;
        update(value);
    }

private:
    static int unsignedRShift(int x, int bits) {
        return ((unsigned int) x) >> bits;
    }
public:

    void update(PathNode* val) {
        int index = val->heapPosition;
        int parentIndex = unsignedRShift(index, 1);
        const double cost = val->combinedCost;
        PathNode* parentNode = vector[parentIndex];
        while (index > 1 && parentNode->combinedCost > cost) {
            this->vector[index] = parentNode;
            this->vector[parentIndex] = val;
            val->heapPosition = parentIndex;
            parentNode->heapPosition = index;
            index = parentIndex;
            parentIndex = unsignedRShift(index, 1);
            parentNode = this->vector[parentIndex];
        }
    }

    PathNode* removeLowest() {
        if (this->size == 0) throw "trolled";

        PathNode* result = vector[1];
        PathNode* val = vector[this->size];
        vector[1] = val;
        val->heapPosition = 1;
        this->vector[this->size] = (PathNode*)0xCCCC;//nullptr;
        this->size--;
        result->heapPosition = -1;
        if (this->size < 2) {
            return result;
        }
        int index = 1;
        int smallerChild = 2;
        double cost = val->combinedCost;
        do {
            PathNode* smallerChildNode = vector[smallerChild];
            double smallerChildCost = smallerChildNode->combinedCost;
            if (smallerChild < this->size) {
                PathNode* rightChildNode = vector[smallerChild + 1];
                double rightChildCost = rightChildNode->combinedCost;
                if (smallerChildCost > rightChildCost) {
                    smallerChild++;
                    smallerChildCost = rightChildCost;
                    smallerChildNode = rightChildNode;
                }
            }
            if (cost <= smallerChildCost) {
                break;
            }
            vector[index] = smallerChildNode;
            vector[smallerChild] = val;
            val->heapPosition = smallerChild;
            smallerChildNode->heapPosition = index;
            index = smallerChild;
        } while ((smallerChild <<= 1) <= this->size);
        return result;
    }

};
