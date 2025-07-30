#pragma once

#include <nori/common.h>
#include <nori/sampler.h>
#include <stb_image.h>
#include <nori/bbox.h>

NORI_NAMESPACE_BEGIN

BoundingBox2f getQuadBoundingBox(const BoundingBox2f &bbox, int quadrant, Vector2f centerRatio = Vector2f(0.5f, 0.5f))
{
    Vector2f size = bbox.max - bbox.min;
    Vector2f center = bbox.min + size.cwiseProduct(centerRatio);

    switch (quadrant)
    {
    case 0:
        return BoundingBox2f(bbox.min, center);
    case 1:
        return BoundingBox2f(Point2f(center.x(), bbox.min.y()), Point2f(bbox.max.x(), center.y()));
    case 2:
        return BoundingBox2f(Point2f(bbox.min.x(), center.y()), Point2f(center.x(), bbox.max.y()));
    case 3:
        return BoundingBox2f(center, bbox.max);
    default:
        throw NoriException("Invalid quadrant index");
    }
}

class ImageWarp
{
public:
    ImageWarp(const std::string &filename)
    {
        stbi_set_flip_vertically_on_load(true);
        image = stbi_load(filename.c_str(), &width, &height, &channels, 0);
        maxDepth = static_cast<int>(std::ceil(std::max(std::log2(width), std::log2(height))));
        maxDepth = std::max(1, maxDepth - 2);
        calculateSumTable();
    }

	ImageWarp(const std::string& filename, int _maxDepth) : maxDepth(_maxDepth)
    {
        stbi_set_flip_vertically_on_load(true);
        image = stbi_load(filename.c_str(), &width, &height, &channels, 0);
        calculateSumTable();
    }

    void calculateSumTable()
    {
        for (int i = 0; i < width; ++i)
        {
            for (int j = 0; j < height; ++j)
            {
                if (i == 0 && j == 0)
                    sumTable[0][0] = getColorSum(0, 0);
                else if (i == 0)
                    sumTable[i][j] = sumTable[i][j - 1] + getColorSum(0, j);
                else if (j == 0)
                    sumTable[i][j] = sumTable[i - 1][j] + getColorSum(i, 0);
                else
                    sumTable[i][j] = sumTable[i - 1][j] + sumTable[i][j - 1] - sumTable[i - 1][j - 1] + getColorSum(i, j);
            }
        }

		totalColorSum = sumTable[width - 1][height - 1];
    }

    float getColorSum(int x, int y)
    {
        if (!image || x < 0 || x >= width || y < 0 || y >= height)
        {
            throw NoriException("Coordinates out of image bounds");
        }
        int grayValue = 0;
        for (int c = 0; c < channels; ++c)
        {
            grayValue += image[(y * width + x) * channels + c];
        }
        return static_cast<float>(grayValue);
    }

    float getColorSum(const Point2f &p)
    {
        int x = static_cast<int>(p.x() * width);
        int y = static_cast<int>(p.y() * height);
        return getColorSum(x, y);
    }

    float getColorSumRange(int x0, int y0, int x1, int y1)
    {
        if (x0 < 0 || y0 < 0 || x1 >= width || y1 >= height)
        {
            throw NoriException("Bounding box out of image bounds");
        }

        float sum = 0.0f;

        sum += sumTable[x1][y1];
        if (x0 > 0)
            sum -= sumTable[x0 - 1][y1];
        if (y0 > 0)
            sum -= sumTable[x1][y0 - 1];
        if (x0 > 0 && y0 > 0)
            sum += sumTable[x0 - 1][y0 - 1];
        return sum;
    }

    float getColorSumRange(const BoundingBox2f &bbox)
    {
        int x0 = static_cast<int>(bbox.min.x() * width * 0.9999);
        int y0 = static_cast<int>(bbox.min.y() * height * 0.9999);
        int x1 = static_cast<int>(bbox.max.x() * width * 0.9999);
        int y1 = static_cast<int>(bbox.max.y() * height * 0.9999);

        return getColorSumRange(x0, y0, x1, y1);
    }

    Point2f squareToImage(const Point2f &sample)
    {
        auto uBbox = BoundingBox2f(Vector2f(0, 0), Vector2f(1, 1));
        return squareToImage(sample, uBbox, uBbox, 0);
    }

    /*
        sample must be in sampleArea
        imgArea must be the area of the image to be warped to
        depth is the current recursion depth, starting from 0
    */
    Point2f squareToImage(const Point2f &sample, const BoundingBox2f &imgArea, const BoundingBox2f &sampleArea, int depth)
    {
        // if at maximum depth return the warped sample position
        if (depth >= maxDepth)
        {
            Vector2f imgAreaSize = imgArea.max - imgArea.min;
            Vector2f sampleAreaSize = sampleArea.max - sampleArea.min;
            return imgArea.min + imgAreaSize.cwiseProduct(
                                     Vector2f((sample.x() - sampleArea.min.x()) / sampleAreaSize.x(),
                                              (sample.y() - sampleArea.min.y()) / sampleAreaSize.y()));
        }

        Point2f imgAreaCenter = imgArea.getCenter();
        Vector2f imgAreaSize = imgArea.max - imgArea.min;

        float colorSum[4];
        for (int i = 0; i < 4; i++)
        {
            BoundingBox2f imgQuad = getQuadBoundingBox(imgArea, i);
            colorSum[i] = getColorSumRange(imgQuad);
        }

        float totalColorSum = colorSum[0] + colorSum[1] + colorSum[2] + colorSum[3];

        if (totalColorSum == 0)
        {
            Vector2f imgAreaSize = imgArea.max - imgArea.min;
            Vector2f sampleAreaSize = sampleArea.max - sampleArea.min;
            return imgArea.min + imgAreaSize.cwiseProduct(
                                     Vector2f((sample.x() - sampleArea.min.x()) / sampleAreaSize.x(),
                                              (sample.y() - sampleArea.min.y()) / sampleAreaSize.y()));
        }

        float upperColorSum = colorSum[0] + colorSum[2];
        float leftColorSum = colorSum[0] + colorSum[1];

        Vector2f sampleCenterRatio = Vector2f(upperColorSum / totalColorSum, leftColorSum / totalColorSum);

        for (int i = 0; i < 4; i++)
        {
            BoundingBox2f sampleSubArea = getQuadBoundingBox(sampleArea, i, sampleCenterRatio);
            if (sampleSubArea.contains(sample))
            {
                return squareToImage(sample, getQuadBoundingBox(imgArea, i),
                                     sampleSubArea, depth + 1);
            }
        }

        throw NoriException("Sample point is not within any sub-area of the sample bounding box");
    }

    float squareToImagePdf(const Point2f &p)
    {
        if(totalColorSum > 0) return getColorSum(p) / totalColorSum;
		return 0.0f;
    }

private:
    unsigned char *image;
    int width, height, channels;
    int maxDepth;
    const static int MAX_DEPTH = 10; // Maximum depth for the quadtree, adjust as needed
    float sumTable[1 << MAX_DEPTH][1 << MAX_DEPTH];
	float totalColorSum;
};

NORI_NAMESPACE_END
