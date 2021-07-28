#ifndef CPP_RECORD3DSTREAM_H
#define CPP_RECORD3DSTREAM_H

#include <vector>
#include <thread>
#include <mutex>
#include <atomic>
#include <array>
#include "Record3DStructs.h"
#include <stdint.h>
#include <functional>

#ifdef PYTHON_BINDINGS_BUILD
#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>
namespace py = pybind11;
#endif


namespace Record3D
{
    using BufferRGB = std::vector<uint8_t>;
    using BufferDepth = std::vector<uint8_t>;

    class Record3DStream
    {
    public:
        /**
         * Represents single connection to a device. Each instance represents connection to an individual device.
         * Upon connecting to a selected device, a callback function is specified that will be passed the RGB and Depth
         * frames.
         */
        Record3DStream();

        ~Record3DStream();

        /**
         * Returns list of currently attached USB devices which can be later passed into the `ConnectToDevice` method.
         * @return list of currently attached devices.
         */
        static std::vector<DeviceInfo> GetConnectedDevices();

        /**
         * Connect to a selected iDevice via USB. The `onNewFrame()` callback will be called upon receiving
         * a new RGBD frame from the iDevice.
         *
         * The callback function is going to be called on non-main thread, therefore it is not suitable for performing
         * UI-related tasks (such as displaying images via OpenCV).
         *
         * The callback function is intended only for copying the passed RGB and Depth buffers into your application's
         * internal buffers. No computation should be done in it as such behaviour could introduce stream lag when
         * spending too much time in it.
         *
         * You can be connected to only one device in a `Record3DStream` instance. Trying to connect to a different
         * iDevice using this function while being already connected does nothing. You will need to first disconnect
         * from the current device via `Disconnect()`.
         *
         * @param $device an iDevice from the list of devices obtained via `GetConnectedDevices()`.
         * @returns boolean value indicating whether connection has been established.
         */
        bool ConnectToDevice(const DeviceInfo &$device);

        /**
         * Manually terminate the current streaming session.
         */
        void Disconnect();

    private:
        /**
         * Runloop that reads from the connected iDevice and processes received data.
         */
        void StreamProcessingRunloop();

        /**
         * Decompresses incoming compressed depth frame into $destinationBuffer of known size.
         *
         * @param $compressedDepthBuffer the buffer containing LZFSE-compressed depth frame.
         * @param $compressedDepthBufferSize size of the compressed buffer.
         * @param $destinationBuffer buffer into which the decompressed depth frame is going to be written.
         * @returns pointer to the decompressed buffer. In case of decompression failure, `nullptr` is returned.
         */
        uint8_t* DecompressDepthBuffer(const uint8_t* $compressedDepthBuffer, size_t $compressedDepthBufferSize, std::vector<uint8_t> &$destinationBuffer);

        /**
         * Wraps the standard `recv()` function to ensure the *exact* amount of bytes (`$numBytesToRead`) is read into the $outputBuffer.
         *
         * @param $socketHandle socket handle obtained as the return value of `usbmuxd_connect()`.
         * @param $outputBuffer buffer that should be written into.
         * @param $numBytesToRead exact amount of bytes that should be read from $socketHandle and written into $socketHandle.
         * @return the amount of bytes actually read. Usually it is equal to $numBytesToRead, but in case connection is interrupted, the amount will be different.
         */
        uint32_t ReceiveWholeBuffer(int $socketHandle, uint8_t* $outputBuffer, uint32_t $numBytesToRead);

    public:
        // Constants
        static constexpr uint16_t DEVICE_PORT{ 1337 }; /** Port on iDevice that we are listening to for RGBD stream. */

#ifdef PYTHON_BINDINGS_BUILD
        /**
         * NOTE: This is alternative API for Python.
         *
         * This function is called when a new RGBD frame is received. The RGBD data and current intrinsic matrix are intended
         * to be read via the `GetCurrentDepthFrame()`, `GetCurrentRGBFrame()` and `GetCurrentIntrinsicMatrix()` accessors.
         */
        std::function<void()> onNewFrame{};
#else
        /**
         * This function is called when a new RGBD frame is received.
         *
         * The callback function is going to be called on non-main thread, therefore it is not suitable for performing
         * UI-related tasks (such as displaying images via OpenCV).
         *
         * The callback function is intended only to copy the passed RGB and Depth buffers into your application's
         * internal buffers. No computation should be done in it as such behaviour could introduce stream lag when
         * spending too much time in it.
         *
         * @param $rgbFrame RGB buffer.
         * @param $depthFrame Depth buffer.
         * @param $frameWidth width od the RGB and Depth frames.
         * @param $frameHeight height od the RGB and Depth frames.
         * @param $K four coefficients of the intrinsic matrix corresponding to the Depth frame.
         */
        std::function<void(const Record3D::BufferRGB &$rgbFrame,
                           const Record3D::BufferDepth &$depthFrame,
                           uint32_t   $rgbWidth,
                           uint32_t   $rgbHeight,
                           uint32_t   $depthWidth,
                           uint32_t   $depthHeight,
                           DeviceType $deviceType,
                           Record3D::IntrinsicMatrixCoeffs $K)> onNewFrame{};
#endif
        /**
         * This function is called when stream ends (that can happen either due to transfer error or by calling the `Disconnect()` method).
         */
        std::function<void()> onStreamStopped{};

#ifdef PYTHON_BINDINGS_BUILD
        /**
         * NOTE: This is alternative API for Python.
         *
         * @returns the current contents of the Depth buffer which holds the lastly received Depth frame.
         */
        py::array_t<float> GetCurrentDepthFrame()
        {
            size_t currentFrameWidth = currentFrameDepthWidth_;
            size_t currentFrameHeight = currentFrameDepthHeight_;

            size_t bufferSize  = currentFrameWidth * currentFrameHeight * sizeof(float);
            auto result        = py::array_t<float>(currentFrameWidth * currentFrameHeight);
            auto result_buffer = result.request();
            float *result_ptr  = (float *) result_buffer.ptr;

            std::memcpy(result_ptr, depthImageBuffer_.data(), bufferSize);
            result.resize(std::vector<int>{static_cast<int>(currentFrameHeight), static_cast<int>(currentFrameWidth)});

            return result;
        }

        /**
         * NOTE: This is alternative API for Python.
         *
         * @returns the current contents of the RGB buffer which holds the lastly received RGB frame.
         */
        py::array_t<uint8_t> GetCurrentRGBFrame()
        {
            size_t currentFrameWidth = currentFrameRGBWidth_;
            size_t currentFrameHeight = currentFrameRGBHeight_;

            constexpr int numChannels = 3;
            size_t bufferSize  = currentFrameWidth * currentFrameHeight * numChannels * sizeof(uint8_t);
            auto result        = py::array_t<uint8_t>(bufferSize);
            auto result_buffer = result.request();
            uint8_t *result_ptr  = (uint8_t *) result_buffer.ptr;

            std::memcpy(result_ptr, RGBImageBuffer_.data(), bufferSize);
            result.resize(std::vector<int>{static_cast<int>(currentFrameHeight), static_cast<int>(currentFrameWidth), numChannels});

            return result;
        }

        /**
         * NOTE: This is alternative API for Python.
         *
         * @returns the intrinsic matrix of the lastly received Depth frame.
         */
        IntrinsicMatrixCoeffs GetCurrentIntrinsicMatrix()
        {
            return rgbIntrinsicMatrixCoeffs_;
        }

        /**
         * NOTE: This is alternative API for Python.
         *
         * @returns the type of camera (TrueDeph = 0, LiDAR = 1).
         */
        uint32_t GetCurrentDeviceType()
        {
            return (uint32_t) currentDeviceType_;
        }
#endif
    private:
        size_t currentFrameRGBWidth_{ 0 };
        size_t currentFrameRGBHeight_{ 0 };
        size_t currentFrameDepthWidth_{ 0 };
        size_t currentFrameDepthHeight_{ 0 };
        DeviceType currentDeviceType_ {};

        uint8_t* lzfseScratchBuffer_{ nullptr }; /** Preallocated LZFSE scratch buffer. */

        int socketHandle_{ -1 }; /** Socket handle representing connection to iDevice. */
        std::atomic<bool> connectionEstablished_{ false }; /** Flag  */
        std::thread runloopThread_; /** Background thread on which the main runloop is running. */

        std::mutex apiCallsMutex_; /** Mutex used for guarding API calls. */

        std::vector<uint8_t> depthImageBuffer_{}; /** Holds the most recent Depth buffer. */
        std::vector<uint8_t> RGBImageBuffer_{}; /** Holds the most recent RGB buffer. */
        IntrinsicMatrixCoeffs rgbIntrinsicMatrixCoeffs_{}; /** Holds the intrinsic matrix of the most recent Depth frame. */
    };
}
#endif //CPP_RECORD3DSTREAM_H
