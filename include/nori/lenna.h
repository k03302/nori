#pragma once

#include <nori/common.h>
#include <nori/sampler.h>
#include <nori/warp.h>
#include <stb_image.h>
#include <nori/ntree.h>

NORI_NAMESPACE_BEGIN

class LennaWarp : public Warp
{
public:
    static float getGrayValue(int x, int y)
    {
        if (!lennaImage || x < 0 || x >= width || y < 0 || y >= height)
        {
            return 0.0f;
        }
        int grayValue = 0;
        for (int c = 0; c < channels; ++c)
        {
            grayValue += lennaImage[(y * width + x) * channels + c];
        }
        return static_cast<float>(grayValue);
    }

    static float calculateTotalValue()
    {
        if (!lennaImage)
            return 0.0f;
        float sum = 0.0f;
        for (int i = 0; i < width * height * channels; ++i)
        {
            sum += lennaImage[i];
        }
        return sum / static_cast<float>(width * height * channels);
    }

    static Point2f squareToLenna(const Point2f &sample)
    {
    }

    static float squareToLennaPdf(const Point2f &p)
    {
        if (!lennaImage)
            return 0.0f;

        int x = static_cast<int>(p.x() * width);
        int y = static_cast<int>(p.y() * height);
        if (x < 0 || x >= width || y < 0 || y >= height)
            return 0.0f;

        float grayValue = getGrayValue(x, y);
        return grayValue / totalValue;
    }

private:
    const static unsigned char *lennaImage;
    static int width, height, channels;
    const static float totalValue;
};

NORI_NAMESPACE_END
