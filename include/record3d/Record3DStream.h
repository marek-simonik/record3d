#ifndef CPP_RECORD3DSTREAM_H
#define CPP_RECORD3DSTREAM_H

#include <vector>
#include <thread>
#include <mutex>
#include <atomic>
#include <array>
#include "Record3DStructs.h" // Contains DeviceInfo, IntrinsicMatrixCoeffs, CameraPose
#include <stdint.h>
#include <functional>
#include <string> // For std::string
#include <memory> // For std::unique_ptr
#include <cstring> // For memcpy in Getters

#include "record3d/WebRTCSignaling.h"
#include <rtc/rtc.hpp> // For rtc::PeerConnection::State in callback, and rtc::Configuration


#ifdef PYTHON_BINDINGS_BUILD
#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>
#include <pybind11/functional.h> // Required for std::function pybind bindings
namespace py = pybind11;
#endif


namespace Record3D
{
    using BufferRGB = std::vector<uint8_t>;
    using BufferDepth = std::vector<uint8_t>;
    using BufferConfidence = std::vector<uint8_t>;
    using BufferMisc = std::vector<uint8_t>;

    class Record3DStream
    {
    public:
        Record3DStream();
        ~Record3DStream();

        static std::vector<DeviceInfo> GetConnectedDevices();
        bool ConnectToDevice(const DeviceInfo &$device);
        void Disconnect();

        // --- WebRTC ---
        bool ConnectToDeviceViaWebRTC(const std::string& ip_address, uint16_t port = 8080);
        void SetRemoteSdpFromPython(const std::string& type, const std::string& sdp);
        void AddIceCandidateFromPython(const std::string& candidate, const std::string& mid);

    public:
        // Constants
        static constexpr uint16_t DEVICE_PORT{ 1337 };

#ifdef PYTHON_BINDINGS_BUILD
        std::function<void()> onNewFrame{};
        // Python-settable callbacks for WebRTC signaling
        std::function<void(const std::string& type, const std::string& sdp)> on_local_sdp_for_python_;
        std::function<void(const std::string& candidate, const std::string& mid)> on_local_ice_candidate_for_python_;
#else
        std::function<void(const Record3D::BufferRGB &$rgbFrame,
                           const Record3D::BufferDepth &$depthFrame,
                           const Record3D::BufferConfidence &$confFrame,
                           const Record3D::BufferMisc &$miscData,
                           uint32_t   $rgbWidth,
                           uint32_t   $rgbHeight,
                           uint32_t   $depthWidth,
                           uint32_t   $depthHeight,
                           uint32_t   $confWidth,
                           uint32_t   $confHeight,
                           Record3D::DeviceType $deviceType,
                           Record3D::IntrinsicMatrixCoeffs $K,
                           Record3D::CameraPose $cameraPose)> onNewFrame{};
#endif
        std::function<void()> onStreamStopped{};

#ifdef PYTHON_BINDINGS_BUILD
        // Python API Getters - defined inline for pybind11
        py::array_t<float> GetCurrentDepthFrame() {
            std::lock_guard<std::mutex> guard{apiCallsMutex_}; // Protect access to member buffers
            size_t currentFrameWidth = currentFrameDepthWidth_;
            size_t currentFrameHeight = currentFrameDepthHeight_;
            size_t bufferSize  = currentFrameWidth * currentFrameHeight;
            auto result = py::array_t<float>(bufferSize);
            if (bufferSize == 0 || depthImageBuffer_.empty()) {
                 result.resize({0,0}); // Return empty array of correct shape if no data
                 return result;
            }
            result.resize({static_cast<ptrdiff_t>(currentFrameHeight), static_cast<ptrdiff_t>(currentFrameWidth)});
            float *result_ptr  = static_cast<float *>(result.request().ptr);
            std::memcpy(result_ptr, depthImageBuffer_.data(), bufferSize * sizeof(float));
            return result;
        }

        py::array_t<uint8_t> GetCurrentConfidenceFrame() {
            std::lock_guard<std::mutex> guard{apiCallsMutex_};
            size_t currentFrameWidth = currentFrameConfidenceWidth_;
            size_t currentFrameHeight = currentFrameConfidenceHeight_;
            size_t bufferSize  = currentFrameWidth * currentFrameHeight;
             auto result = py::array_t<uint8_t>(bufferSize);
            if (bufferSize == 0 || confidenceImageBuffer_.empty()) {
                 result.resize({0,0});
                 return result;
            }
            result.resize({static_cast<ptrdiff_t>(currentFrameHeight), static_cast<ptrdiff_t>(currentFrameWidth)});
            uint8_t *result_ptr  = static_cast<uint8_t *>(result.request().ptr);
            std::memcpy(result_ptr, confidenceImageBuffer_.data(), bufferSize * sizeof(uint8_t));
            return result;
        }

        py::array_t<uint8_t> GetCurrentMiscData() {
            std::lock_guard<std::mutex> guard{apiCallsMutex_};
            if (miscBuffer_.empty()) {
                return py::array_t<uint8_t>(0);
            }
            auto result = py::array_t<uint8_t>(miscBuffer_.size());
            uint8_t *result_ptr  = static_cast<uint8_t *>(result.request().ptr);
            std::memcpy(result_ptr, miscBuffer_.data(), miscBuffer_.size() * sizeof(uint8_t));
            return result;
        }

        py::array_t<uint8_t> GetCurrentRGBFrame() {
            std::lock_guard<std::mutex> guard{apiCallsMutex_};
            size_t currentFrameWidth = currentFrameRGBWidth_;
            size_t currentFrameHeight = currentFrameRGBHeight_;
            constexpr int numChannels = 3;
            size_t bufferSize  = currentFrameWidth * currentFrameHeight * numChannels;
            auto result = py::array_t<uint8_t>(bufferSize);
             if (bufferSize == 0 || RGBImageBuffer_.empty()) {
                 result.resize({0,0,numChannels});
                 return result;
            }
            result.resize({static_cast<ptrdiff_t>(currentFrameHeight), static_cast<ptrdiff_t>(currentFrameWidth), static_cast<ptrdiff_t>(numChannels)});
            uint8_t *result_ptr  = static_cast<uint8_t *>(result.request().ptr);
            std::memcpy(result_ptr, RGBImageBuffer_.data(), bufferSize * sizeof(uint8_t));
            return result;
        }

        IntrinsicMatrixCoeffs GetCurrentIntrinsicMatrix() {
            std::lock_guard<std::mutex> guard{apiCallsMutex_};
            return rgbIntrinsicMatrixCoeffs_;
        }

        CameraPose GetCurrentCameraPose() {
            std::lock_guard<std::mutex> guard{apiCallsMutex_};
            return cameraPose_;
        }

        uint32_t GetCurrentDeviceType() {
            std::lock_guard<std::mutex> guard{apiCallsMutex_};
            return static_cast<uint32_t>(currentDeviceType_);
        }
#endif

    private:
        void StreamProcessingRunloopUSB();
        uint8_t* DecompressBuffer(const uint8_t* $compressedBuffer, size_t $compressedBufferSize, std::vector<uint8_t> &$destinationBuffer);
        uint32_t ReceiveWholeBuffer(int $socketHandle, uint8_t* $outputBuffer, uint32_t $numBytesToRead);

        // WebRTC Internal Callback Handlers (called by WebRTCSignaling instance)
        void OnWebRTCLocalDescription(const std::string& type, const std::string& sdp);
        void OnWebRTCIceCandidate(const std::string& candidate, const std::string& mid);
        void OnWebRTCDataChannelOpen();
    public: // Made public for testing only
        void OnWebRTCDataChannelMessage(const std::string& message); // message is binary data from datachannel
    private:
        void OnWebRTCDataChannelError(const std::string& error);
        void OnWebRTCConnectionStateChange(rtc::PeerConnection::State state);

    private:
        size_t currentFrameRGBWidth_{ 0 };
        size_t currentFrameRGBHeight_{ 0 };
        size_t currentFrameDepthWidth_{ 0 };
        size_t currentFrameDepthHeight_{ 0 };
        size_t currentFrameConfidenceWidth_{ 0 };
        size_t currentFrameConfidenceHeight_{ 0 };
        DeviceType currentDeviceType_ {};

        uint8_t* lzfseScratchBuffer_{ nullptr };

        std::atomic<bool> connectionEstablished_{ false };
        std::mutex apiCallsMutex_; // Mutex to protect access to shared data like frame buffers and current metadata

        // USB Streaming members
        int socketHandle_{ -1 };
        std::thread usb_runloop_thread_;

        // WebRTC Streaming members
        std::unique_ptr<WebRTCSignaling> webrtc_signaling_;

#ifndef PYTHON_BINDINGS_BUILD // Test-only members when not building for python
    public:
        std::function<void(const std::string& type, const std::string& sdp)> test_on_local_sdp_cpp_;
        std::function<void(const std::string& candidate, const std::string& mid)> test_on_local_ice_candidate_cpp_;
#endif

        // Common frame data members
        std::vector<uint8_t> depthImageBuffer_{};
        std::vector<uint8_t> RGBImageBuffer_{};
        std::vector<uint8_t> confidenceImageBuffer_{};
        std::vector<uint8_t> miscBuffer_{};
        IntrinsicMatrixCoeffs rgbIntrinsicMatrixCoeffs_{};
        CameraPose cameraPose_{};
    };
}
#endif //CPP_RECORD3DSTREAM_H
