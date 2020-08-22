#include "NoiseGeneratorOctaves.h"

/*static int64_t lfloor(double value) {
    int64_t i = (int64_t)value;
    return value < (double)i ? i - 1L : i;
}*/

void NoiseGeneratorOctavesBase::generateNoiseOctaves0(double* noiseArray, int xOffset, int yOffset, int zOffset, int xSize, int ySize, int zSize, double xScale, double yScale, double zScale) const {
    double d3 = 1.0;

    for (int j = 0; j < this->octaves; ++j)
    {
        double d0 = (double)xOffset * d3 * xScale;
        double d1 = (double)yOffset * d3 * yScale;
        double d2 = (double)zOffset * d3 * zScale;
        int64_t k = lfloor(d0);
        int64_t l = lfloor(d2);
        d0 = d0 - (double)k;
        d2 = d2 - (double)l;
        k = k % 16777216L;
        l = l % 16777216L;
        d0 = d0 + (double)k;
        d2 = d2 + (double)l;
        this->generators[j].populateNoiseArray(&noiseArray[0], d0, d1, d2, xSize, ySize, zSize, xScale * d3, yScale * d3, zScale * d3, d3);
        d3 /= 2.0;
    }
}
