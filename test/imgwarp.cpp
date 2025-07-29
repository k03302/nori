#include <nori/common.h>
#include <iostream>
#include <nori/bbox.h>
#define STB_IMAGE_IMPLEMENTATION
#include <nori/imgwarp.h>
#include <pcg32.h>

using namespace nori;

ImageWarp imgWarp("../assets/circle.png");

int main()
{
    //pcg32 rng(0xDEADBEEF, 0xBEEFCAFE);
    //for (int i = 0; i < 100; ++i)
    //{
    //    auto sample = imgWarp.squareToImage(Point2f(rng.nextFloat(), rng.nextFloat()));
    //    if (sample.x() < 0 || sample.x() > 1 || sample.y() < 0 || sample.y() > 1)
    //    {
    //        return -1;
    //    }
    //}


	std::cout << imgWarp.squareToImage(Point2f(0.5f, 0.5f)) << std::endl;

    return 0;
}