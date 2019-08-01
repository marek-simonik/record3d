#ifndef CPP_RECORD3DSTRUCTS_H
#define CPP_RECORD3DSTRUCTS_H

namespace Record3D
{
    struct IntrinsicMatrixCoeffs
    {
        float fx;
        float fy;
        float tx;
        float ty;
    };

    struct DeviceInfo
    {
        uint32_t productId{ 0 };
        std::string udid{ "" };
        uint32_t handle{ 0 };
    };
}

#endif //CPP_RECORD3DSTRUCTS_H
