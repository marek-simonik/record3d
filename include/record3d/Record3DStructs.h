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

    enum DeviceType
    {
        R3D_DEVICE_TYPE__FACEID = 0,
        R3D_DEVICE_TYPE__LIDAR
    };
}

#endif //CPP_RECORD3DSTRUCTS_H
