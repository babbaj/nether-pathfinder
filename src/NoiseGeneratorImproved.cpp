#include <cstdlib>
#include "NoiseGeneratorImproved.h"

static constexpr double GRAD_X[] =  {1.0, -1.0, 1.0, -1.0, 1.0, -1.0, 1.0, -1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, -1.0, 0.0};
static constexpr double GRAD_Y[]  = {1.0, 1.0, -1.0, -1.0, 0.0, 0.0, 0.0, 0.0, 1.0, -1.0, 1.0, -1.0, 1.0, -1.0, 1.0, -1.0};
static constexpr double GRAD_Z[]  = {0.0, 0.0, 0.0, 0.0, 1.0, 1.0, -1.0, -1.0, 1.0, 1.0, -1.0, -1.0, 0.0, 1.0, 0.0, -1.0};
static constexpr double GRAD_2X[] = {1.0, -1.0, 1.0, -1.0, 1.0, -1.0, 1.0, -1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, -1.0, 0.0};
static constexpr double GRAD_2Z[] = {0.0, 0.0, 0.0, 0.0, 1.0, 1.0, -1.0, -1.0, 1.0, 1.0, -1.0, -1.0, 0.0, 1.0, 0.0, -1.0};

constexpr static double lerp(double a, double b, double c)
{
    return b + a * (c - b);
}

constexpr static double grad2(int p_76309_1_, double p_76309_2_, double p_76309_4_)
{
    int i = p_76309_1_ & 15;
    return GRAD_2X[i] * p_76309_2_ + GRAD_2Z[i] * p_76309_4_;
}

constexpr static double grad(int p_76310_1_, double p_76310_2_, double p_76310_4_, double p_76310_6_)
{
    int i = p_76310_1_ & 15;
    return GRAD_X[i] * p_76310_2_ + GRAD_Y[i] * p_76310_4_ + GRAD_Z[i] * p_76310_6_;
}


void NoiseGeneratorImproved::populateNoiseArray(double* noiseArray, double xOffset, double yOffset, double zOffset, int xSize, int ySize, int zSize, double xScale, double yScale, double zScale, double noiseScale) const {
    if (ySize == 1) exit(1); // uses different code that's deleted
    int i = 0;
    const double d0 = 1.0 / noiseScale;
    int k = -1;
    double d1 = 0.0;
    double d2 = 0.0;
    double d3 = 0.0;
    double d4 = 0.0;

    for (int l2 = 0; l2 < xSize; ++l2)
    {
        double d5 = xOffset + (double)l2 * xScale + this->xCoord;
        int i3 = (int)d5;

        if (d5 < (double)i3)
        {
            --i3;
        }

        int j3 = i3 & 255;
        d5 = d5 - (double)i3;
        const double d6 = d5 * d5 * d5 * (d5 * (d5 * 6.0 - 15.0) + 10.0);

        for (int k3 = 0; k3 < zSize; ++k3)
        {
            double d7 = zOffset + (double)k3 * zScale + this->zCoord;
            int l3 = (int)d7;

            if (d7 < (double)l3)
            {
                --l3;
            }

            const int i4 = l3 & 255;
            d7 = d7 - (double)l3;
            const double d8 = d7 * d7 * d7 * (d7 * (d7 * 6.0 - 15.0) + 10.0);

            for (int j4 = 0; j4 < ySize; ++j4)
            {
                double d9 = yOffset + (double)j4 * yScale + this->yCoord;
                int k4 = (int)d9;

                if (d9 < (double)k4)
                {
                    --k4;
                }

                const int l4 = k4 & 255;
                d9 = d9 - (double)k4;
                const double d10 = d9 * d9 * d9 * (d9 * (d9 * 6.0 - 15.0) + 10.0);

                if (j4 == 0 || l4 != k)
                {
                    k = l4;
                    const int l =  this->permutations[j3] + l4;
                    const int i1 = this->permutations[l] + i4;
                    const int j1 = this->permutations[l + 1] + i4;
                    const int k1 = this->permutations[j3 + 1] + l4;
                    const int l1 = this->permutations[k1] + i4;
                    const int i2 = this->permutations[k1 + 1] + i4;
                    d1 = lerp(d6, grad(this->permutations[i1],     d5, d9,       d7),       grad(this->permutations[l1],     d5 - 1.0, d9,       d7));
                    d2 = lerp(d6, grad(this->permutations[j1],     d5, d9 - 1.0, d7),       grad(this->permutations[i2],     d5 - 1.0, d9 - 1.0, d7));
                    d3 = lerp(d6, grad(this->permutations[i1 + 1], d5, d9,       d7 - 1.0), grad(this->permutations[l1 + 1], d5 - 1.0, d9,       d7 - 1.0));
                    d4 = lerp(d6, grad(this->permutations[j1 + 1], d5, d9 - 1.0, d7 - 1.0), grad(this->permutations[i2 + 1], d5 - 1.0, d9 - 1.0, d7 - 1.0));
                }

                const double d11 = lerp(d10, d1, d2);
                const double d12 = lerp(d10, d3, d4);
                const double d13 = lerp(d8, d11, d12);
                const int j7 = i++;
                noiseArray[j7] += d13 * d0;
            }
        }
    }
}