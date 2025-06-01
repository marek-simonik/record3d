#include "../include/record3d/Record3DStream.h"
#include "JPEGDecoder.h" // For stbi_load_from_memory, stbi_image_free
#include <lzfse.h>
#include <usbmuxd.h>
#include <cstring> // For memcpy
#include <array>
#include <iostream> // For std::cout, std::cerr
#include <functional> // For std::bind, std::placeholders
#include <vector> // Ensure std::vector is included

// For WebRTC parts
#include <rtc/rtc.hpp>
#include <rtc/peerconnection.hpp>
#include <rtc/datachannel.hpp>
#include "../include/record3d/WebRTCSignaling.h" // Ensure full definition is available


#define NTOHL_(n) (((((unsigned long)(n) & 0xFF)) << 24) | \
                  ((((unsigned long)(n) & 0xFF00)) << 8) | \
                  ((((unsigned long)(n) & 0xFF0000)) >> 8) | \
                  ((((unsigned long)(n) & 0xFF000000)) >> 24))

namespace Record3D
{
    // Define Record3DHeader struct here as it's used in this .cpp file for both USB and potentially WebRTC
    struct Record3DHeader {
        uint32_t rgbWidth, rgbHeight, depthWidth, depthHeight, confidenceWidth, confidenceHeight;
        uint32_t rgbSize, depthSize, confidenceMapSize, miscSize, deviceType;
    };

    struct PeerTalkHeader { // Specific to USB communication
        uint32_t a, b, c, body_size;
    };


    Record3DStream::Record3DStream()
        : lzfseScratchBuffer_( new uint8_t[lzfse_decode_scratch_size()] ),
          onNewFrame{},
          onStreamStopped{},
#ifdef PYTHON_BINDINGS_BUILD
          on_local_sdp_for_python_{nullptr},
          on_local_ice_candidate_for_python_{nullptr},
#endif
          webrtc_signaling_(nullptr)
    {
    }

    Record3DStream::~Record3DStream()
    {
        Disconnect();
        if (webrtc_signaling_) {
             webrtc_signaling_.reset();
        }
        delete[] lzfseScratchBuffer_;
    }

    std::vector<DeviceInfo> Record3DStream::GetConnectedDevices()
    {
        usbmuxd_device_info_t* deviceInfoList;
        int numDevices = usbmuxd_get_device_list( &deviceInfoList );
        std::vector<DeviceInfo> availableDevices;

        for ( int devIdx = 0; devIdx < numDevices; devIdx++ )
        {
            const auto &dev = deviceInfoList[ devIdx ];
            if ( dev.conn_type != CONNECTION_TYPE_USB ) continue;

            DeviceInfo currDevInfo;
            currDevInfo.handle = dev.handle;
            currDevInfo.productId = dev.product_id;
            if (dev.udid) {
                currDevInfo.udid = std::string( dev.udid );
            } else {
                currDevInfo.udid = "";
            }
            availableDevices.push_back( currDevInfo );
        }
        usbmuxd_device_list_free( &deviceInfoList );
        return availableDevices;
    }

    bool Record3DStream::ConnectToDevice(const DeviceInfo &$device)
    {
        std::lock_guard<std::mutex> guard{ apiCallsMutex_ };
        if ( connectionEstablished_.load()) {
            std::cerr << "Record3DStream: Already connected. Disconnect first." << std::endl;
            return false;
        }
        auto socketNo = usbmuxd_connect( $device.handle, DEVICE_PORT );
        if ( socketNo < 0 ) {
            std::cerr << "Record3DStream: Failed to connect to USB device." << std::endl;
            return false;
        }
        connectionEstablished_.store( true );
        socketHandle_ = socketNo;
        usb_runloop_thread_ = std::thread( &Record3DStream::StreamProcessingRunloopUSB, this );
        std::cout << "Record3DStream: USB connection established." << std::endl;
        return true;
    }

    bool Record3DStream::ConnectToDeviceViaWebRTC(const std::string& ip_address, uint16_t port) {
        std::lock_guard<std::mutex> guard{ apiCallsMutex_ };
        if (connectionEstablished_.load()) {
            std::cerr << "Record3DStream: Already connected. Disconnect first." << std::endl;
            return false;
        }
        if (webrtc_signaling_) {
            std::cout << "Record3DStream: WebRTC signaling already exists, closing previous one." << std::endl;
            webrtc_signaling_->Close();
            webrtc_signaling_.reset();
        }
        std::cout << "Record3DStream: Attempting WebRTC connection (signaling placeholder: " << ip_address << ":" << port << ")" << std::endl;

        auto onLocalDesc = std::bind(&Record3DStream::OnWebRTCLocalDescription, this, std::placeholders::_1, std::placeholders::_2);
        auto onIceCand = std::bind(&Record3DStream::OnWebRTCIceCandidate, this, std::placeholders::_1, std::placeholders::_2);
        auto onDcOpen = std::bind(&Record3DStream::OnWebRTCDataChannelOpen, this);
        auto onDcMsg = std::bind(&Record3DStream::OnWebRTCDataChannelMessage, this, std::placeholders::_1);
        auto onDcError = std::bind(&Record3DStream::OnWebRTCDataChannelError, this, std::placeholders::_1);
        auto onConnStateChange = std::bind(&Record3DStream::OnWebRTCConnectionStateChange, this, std::placeholders::_1);

        webrtc_signaling_ = std::make_unique<WebRTCSignaling>(onLocalDesc, onIceCand, onDcOpen, onDcMsg, onDcError, onConnStateChange);
        rtc::Configuration config;
        webrtc_signaling_->InitConnection(config);
        webrtc_signaling_->CreateOffer();
        connectionEstablished_.store(true);
        std::cout << "Record3DStream: WebRTC connection initiated, C++ offer created." << std::endl;
        return true;
    }

    void Record3DStream::SetRemoteSdpFromPython(const std::string& type, const std::string& sdp) {
        std::lock_guard<std::mutex> guard{ apiCallsMutex_ };
        if (!webrtc_signaling_) {
            std::cerr << "Record3DStream: WebRTC not initialized. Call ConnectToDeviceViaWebRTC first." << std::endl;
            return;
        }
        std::cout << "Record3DStream: Setting remote SDP from Python of type: " << type << std::endl;
        webrtc_signaling_->SetRemoteDescription(type, sdp);
    }

    void Record3DStream::AddIceCandidateFromPython(const std::string& candidate, const std::string& mid) {
        std::lock_guard<std::mutex> guard{ apiCallsMutex_ };
        if (!webrtc_signaling_) {
            std::cerr << "Record3DStream: WebRTC not initialized." << std::endl;
            return;
        }
        std::cout << "Record3DStream: Adding ICE candidate from Python for mid: " << mid << std::endl;
        webrtc_signaling_->AddIceCandidate(candidate, mid);
    }

    void Record3DStream::Disconnect() {
        std::lock_guard<std::mutex> guard{ apiCallsMutex_ };
        bool was_connected = connectionEstablished_.exchange(false);

        if (webrtc_signaling_) {
            std::cout << "Record3DStream: Closing WebRTC signaling component." << std::endl;
            webrtc_signaling_->Close();
            // Resetting of webrtc_signaling_ is handled in OnWebRTCConnectionStateChange or destructor
        }

        if (socketHandle_ != -1) {
            std::cout << "Record3DStream: Signaling USB runloop to stop." << std::endl;
            // No explicit close for usbmuxd socketHandle_, but setting it to -1 and
            // connectionEstablished_ to false will stop the loop.
            socketHandle_ = -1;
        }

        if (usb_runloop_thread_.joinable()) {
            usb_runloop_thread_.join();
            std::cout << "Record3DStream: USB runloop thread joined." << std::endl;
        }

        if ( was_connected && onStreamStopped ) {
            std::cout << "Record3DStream: Calling onStreamStopped callback." << std::endl;
            onStreamStopped();
        }
    }

    void Record3DStream::OnWebRTCLocalDescription(const std::string& type, const std::string& sdp) {
#ifdef PYTHON_BINDINGS_BUILD
        if (on_local_sdp_for_python_) {
            on_local_sdp_for_python_(type, sdp);
            return;
        }
#else // PYTHON_BINDINGS_BUILD not defined
        if (test_on_local_sdp_cpp_) { // Check if C++ test callback is set
            test_on_local_sdp_cpp_(type, sdp);
            return;
        }
#endif
        // Fallback if no Python or C++ test callback is set
        std::cout << "Record3DStream (C++ Fallback/Default): WebRTC Local Description (" << type << "). SDP: " << sdp.substr(0, 60) << "..." << std::endl;
    }

    void Record3DStream::OnWebRTCIceCandidate(const std::string& candidate, const std::string& mid) {
#ifdef PYTHON_BINDINGS_BUILD
        if (on_local_ice_candidate_for_python_) {
            on_local_ice_candidate_for_python_(candidate, mid);
            return;
        }
#else // PYTHON_BINDINGS_BUILD not defined
        if (test_on_local_ice_candidate_cpp_) { // Check if C++ test callback is set
            test_on_local_ice_candidate_cpp_(candidate, mid);
            return;
        }
#endif
        // Fallback if no Python or C++ test callback is set
        std::cout << "Record3DStream (C++ Fallback/Default): WebRTC ICE Candidate (" << mid << "): " << candidate << std::endl;
    }

    void Record3DStream::OnWebRTCDataChannelOpen() {
        std::cout << "Record3DStream: WebRTC DataChannel opened!" << std::endl;
    }

    void Record3DStream::OnWebRTCDataChannelMessage(const std::string& message) {
        // std::cout << "Record3DStream: WebRTC DataChannel message received, size: " << message.length() << " bytes." << std::endl;
        if (!connectionEstablished_.load()) {
             std::cerr << "Record3DStream: Message received on data channel, but connection is not marked as established. Ignoring." << std::endl;
             return;
        }

        const uint8_t* data_ptr = reinterpret_cast<const uint8_t*>(message.data());
        size_t messageBodySize = message.size();

        Record3DHeader record3DHeader;
        size_t offset = 0;
        size_t currSize = 0;

        currSize = sizeof( Record3DHeader );
        if (offset + currSize > messageBodySize) { std::cerr << "Record3DStream_WebRTC: Buffer too small for Record3DHeader. Size: " << messageBodySize << std::endl; return; }
        memcpy((void*) &record3DHeader, data_ptr + offset, currSize );
        currentDeviceType_ = (DeviceType)record3DHeader.deviceType;
        offset += currSize;

        currSize = sizeof( IntrinsicMatrixCoeffs );
        if (offset + currSize > messageBodySize) { std::cerr << "Record3DStream_WebRTC: Buffer too small for Intrinsics." << std::endl; return; }
        memcpy((void*) &rgbIntrinsicMatrixCoeffs_, data_ptr + offset, currSize );
        offset += currSize;

        currSize = sizeof( CameraPose );
        if (offset + currSize > messageBodySize) { std::cerr << "Record3DStream_WebRTC: Buffer too small for CameraPose." << std::endl; return; }
        memcpy( (void*) &cameraPose_, data_ptr + offset, currSize );
        offset += currSize;

        // RGB Frame (JPEG)
        currSize = record3DHeader.rgbSize;
        if (offset + currSize > messageBodySize) { std::cerr << "Record3DStream_WebRTC: Buffer too small for RGB data." << std::endl; return; }
        int loadedWidth, loadedHeight, loadedChannels;
        uint8_t* rgbPixels = stbi_load_from_memory( data_ptr + offset, currSize, &loadedWidth, &loadedHeight, &loadedChannels, STBI_rgb );
        if (!rgbPixels) {
            std::cerr << "Record3DStream_WebRTC: Failed to decode RGB frame." << std::endl;
            return;
        }
        size_t decompressedRGBDataSize = loadedWidth * loadedHeight * loadedChannels * sizeof(uint8_t);
        if ( RGBImageBuffer_.size() != decompressedRGBDataSize ) {
            RGBImageBuffer_.resize(decompressedRGBDataSize);
        }
        memcpy( RGBImageBuffer_.data(), rgbPixels, decompressedRGBDataSize);
        stbi_image_free( rgbPixels );
        offset += currSize;

        // Depth Frame (LZFSE)
        currSize = record3DHeader.depthSize;
        if (offset + currSize > messageBodySize) { std::cerr << "Record3DStream_WebRTC: Buffer too small for Depth data." << std::endl; return; }
        size_t decompressedDepthDataSize = record3DHeader.depthWidth * record3DHeader.depthHeight * sizeof(float);
        if ( depthImageBuffer_.size() != decompressedDepthDataSize ) {
            depthImageBuffer_.resize(decompressedDepthDataSize);
        }
        if (record3DHeader.depthSize > 0 && !DecompressBuffer(data_ptr + offset, currSize, depthImageBuffer_)) {
            std::cerr << "Record3DStream_WebRTC: Failed to decompress Depth frame." << std::endl;
            return;
        }
        offset += currSize;

        // Confidence Frame (LZFSE)
        if (record3DHeader.confidenceMapSize > 0) {
            currSize = record3DHeader.confidenceMapSize;
            if (offset + currSize > messageBodySize) { std::cerr << "Record3DStream_WebRTC: Buffer too small for Confidence data." << std::endl; return; }
            size_t decompressedConfidenceDataSize = record3DHeader.confidenceWidth * record3DHeader.confidenceHeight * sizeof(uint8_t);
            if ( confidenceImageBuffer_.size() != decompressedConfidenceDataSize ) {
                confidenceImageBuffer_.resize(decompressedConfidenceDataSize);
            }
            if (!DecompressBuffer(data_ptr + offset, currSize, confidenceImageBuffer_)) {
                 std::cerr << "Record3DStream_WebRTC: Failed to decompress Confidence frame." << std::endl;
                 return;
            }
            offset += currSize;
        } else {
            confidenceImageBuffer_.clear();
        }

        // Misc Data
        if ( record3DHeader.miscSize > 0 ) {
            currSize = record3DHeader.miscSize;
            if (offset + currSize > messageBodySize) { std::cerr << "Record3DStream_WebRTC: Buffer too small for Misc data." << std::endl; return; }
            miscBuffer_.resize( currSize );
            memcpy(miscBuffer_.data(), data_ptr + offset, currSize );
        } else {
            miscBuffer_.clear();
        }

        if ( onNewFrame ) {
            currentFrameRGBWidth_ = record3DHeader.rgbWidth;
            currentFrameRGBHeight_ = record3DHeader.rgbHeight;
            currentFrameDepthWidth_ = record3DHeader.depthWidth;
            currentFrameDepthHeight_ = record3DHeader.depthHeight;
            currentFrameConfidenceWidth_ = record3DHeader.confidenceWidth;
            currentFrameConfidenceHeight_ = record3DHeader.confidenceHeight;
#ifdef PYTHON_BINDINGS_BUILD
            onNewFrame( );
#else
            onNewFrame( RGBImageBuffer_, depthImageBuffer_, confidenceImageBuffer_, miscBuffer_,
                        record3DHeader.rgbWidth, record3DHeader.rgbHeight,
                        record3DHeader.depthWidth, record3DHeader.depthHeight,
                        record3DHeader.confidenceWidth, record3DHeader.confidenceHeight,
                        currentDeviceType_, rgbIntrinsicMatrixCoeffs_, cameraPose_ );
#endif
        }
    }

    void Record3DStream::OnWebRTCDataChannelError(const std::string& error) {
        std::cerr << "Record3DStream: WebRTC DataChannel error: " << error << std::endl;
    }

    void Record3DStream::OnWebRTCConnectionStateChange(rtc::PeerConnection::State state) {
        std::cout << "Record3DStream: WebRTC PeerConnection state changed: ";
        std::string state_str = "Unknown";
        switch (state) {
            case rtc::PeerConnection::State::New: state_str = "New"; break;
            case rtc::PeerConnection::State::Connecting: state_str = "Connecting"; break;
            case rtc::PeerConnection::State::Connected: state_str = "Connected"; break;
            case rtc::PeerConnection::State::Disconnected: state_str = "Disconnected"; break;
            case rtc::PeerConnection::State::Failed: state_str = "Failed"; break;
            case rtc::PeerConnection::State::Closed: state_str = "Closed"; break;
        }
        std::cout << state_str << std::endl;

        bool should_trigger_stop = false;
        if (state == rtc::PeerConnection::State::Failed ||
            state == rtc::PeerConnection::State::Closed ||
            state == rtc::PeerConnection::State::Disconnected) {

            if (webrtc_signaling_) {
                 if(connectionEstablished_.exchange(false)){
                    should_trigger_stop = true;
                 }
            }

            if (webrtc_signaling_ && (state == rtc::PeerConnection::State::Failed || state == rtc::PeerConnection::State::Closed) ) {
                 webrtc_signaling_.reset();
                 std::cout << "Record3DStream: WebRTCSignaling component reset." << std::endl;
            }
        }
        if (should_trigger_stop && onStreamStopped) {
            onStreamStopped();
        }
    }

    void Record3DStream::StreamProcessingRunloopUSB() {
        std::vector<uint8_t> rawMessageBuffer;
        uint32_t numReceivedData = 0;

        while (connectionEstablished_.load() && socketHandle_ != -1) {
            PeerTalkHeader ptHeader;
            numReceivedData = ReceiveWholeBuffer( socketHandle_, (uint8_t*) &ptHeader, sizeof( ptHeader ));
            if ( numReceivedData != sizeof( ptHeader )) {
                std::cerr << "Record3DStream: Failed to receive PeerTalk header on USB." << std::endl;
                break;
            }
            uint32_t messageBodySize = NTOHL_( ptHeader.body_size );
            if (messageBodySize == 0 || messageBodySize > 100 * 1024 * 1024) {
                 std::cerr << "Record3DStream: Invalid message body size from USB: " << messageBodySize << std::endl;
                 break;
            }

            if ( rawMessageBuffer.size() < messageBodySize ) {
                rawMessageBuffer.resize( messageBodySize );
            }

            numReceivedData = ReceiveWholeBuffer( socketHandle_, (uint8_t*) rawMessageBuffer.data(), messageBodySize );
            if ( numReceivedData != messageBodySize ) {
                std::cerr << "Record3DStream: Failed to receive message body on USB." << std::endl;
                break;
            }

            Record3DHeader record3DHeader;
            size_t offset = 0;
            size_t currSize = 0;

            currSize = sizeof( Record3DHeader );
            if (offset + currSize > messageBodySize) { std::cerr << "Record3DStream_USB: Buffer too small for Record3DHeader." << std::endl; break; }
            memcpy((void*) &record3DHeader, rawMessageBuffer.data() + offset, currSize );
            currentDeviceType_ = (DeviceType)record3DHeader.deviceType;
            offset += currSize;

            currSize = sizeof( IntrinsicMatrixCoeffs );
            if (offset + currSize > messageBodySize) { std::cerr << "Record3DStream_USB: Buffer too small for Intrinsics." << std::endl; break; }
            memcpy((void*) &rgbIntrinsicMatrixCoeffs_, rawMessageBuffer.data() + offset, currSize );
            offset += currSize;

            currSize = sizeof( CameraPose );
            if (offset + currSize > messageBodySize) { std::cerr << "Record3DStream_USB: Buffer too small for CameraPose." << std::endl; break; }
            memcpy( (void*) &cameraPose_, rawMessageBuffer.data() + offset, currSize );
            offset += currSize;

            currSize = record3DHeader.rgbSize;
            if (offset + currSize > messageBodySize) { std::cerr << "Record3DStream_USB: Buffer too small for RGB data." << std::endl; break; }
            int loadedWidth, loadedHeight, loadedChannels;
            uint8_t* rgbPixels = stbi_load_from_memory( rawMessageBuffer.data() + offset, currSize, &loadedWidth, &loadedHeight, &loadedChannels, STBI_rgb );
            if (!rgbPixels) {
                std::cerr << "Record3DStream_USB: Failed to decode RGB frame." << std::endl;
                continue;
            }
            size_t decompressedRGBDataSize = loadedWidth * loadedHeight * loadedChannels * sizeof(uint8_t);
            if ( RGBImageBuffer_.size() != decompressedRGBDataSize ) {
                RGBImageBuffer_.resize(decompressedRGBDataSize);
            }
            memcpy( RGBImageBuffer_.data(), rgbPixels, decompressedRGBDataSize);
            stbi_image_free( rgbPixels );
            offset += currSize;

            currSize = record3DHeader.depthSize;
             if (offset + currSize > messageBodySize) { std::cerr << "Record3DStream_USB: Buffer too small for Depth data." << std::endl; break; }
            size_t decompressedDepthDataSize = record3DHeader.depthWidth * record3DHeader.depthHeight * sizeof(float);
            if ( depthImageBuffer_.size() != decompressedDepthDataSize ) {
                depthImageBuffer_.resize(decompressedDepthDataSize);
            }
             if (record3DHeader.depthSize > 0 && !DecompressBuffer(rawMessageBuffer.data() + offset, currSize, depthImageBuffer_)) {
                std::cerr << "Record3DStream_USB: Failed to decompress Depth frame." << std::endl;
                // Potentially continue to try and salvage next frame or break.
                continue;
            }
            offset += currSize;

            if (record3DHeader.confidenceMapSize > 0) {
                currSize = record3DHeader.confidenceMapSize;
                if (offset + currSize > messageBodySize) { std::cerr << "Record3DStream_USB: Buffer too small for Confidence data." << std::endl; break; }
                size_t decompressedConfidenceDataSize = record3DHeader.confidenceWidth * record3DHeader.confidenceHeight * sizeof(uint8_t);
                if ( confidenceImageBuffer_.size() != decompressedConfidenceDataSize ) {
                    confidenceImageBuffer_.resize(decompressedConfidenceDataSize);
                }
                if (!DecompressBuffer(rawMessageBuffer.data() + offset, currSize, confidenceImageBuffer_)) {
                    std::cerr << "Record3DStream_USB: Failed to decompress Confidence frame." << std::endl;
                    continue;
                }
                offset += currSize;
            } else {
                confidenceImageBuffer_.clear();
            }

            if ( record3DHeader.miscSize > 0 ) {
                currSize = record3DHeader.miscSize;
                 if (offset + currSize > messageBodySize) { std::cerr << "Record3DStream_USB: Buffer too small for Misc data." << std::endl; break; }
                miscBuffer_.resize( currSize );
                memcpy(miscBuffer_.data(), rawMessageBuffer.data() + offset, currSize );
            } else {
                miscBuffer_.clear();
            }

            if ( onNewFrame ) {
                currentFrameRGBWidth_ = record3DHeader.rgbWidth;
                currentFrameRGBHeight_ = record3DHeader.rgbHeight;
                currentFrameDepthWidth_ = record3DHeader.depthWidth;
                currentFrameDepthHeight_ = record3DHeader.depthHeight;
                currentFrameConfidenceWidth_ = record3DHeader.confidenceWidth;
                currentFrameConfidenceHeight_ = record3DHeader.confidenceHeight;
#ifdef PYTHON_BINDINGS_BUILD
                onNewFrame( );
#else
                onNewFrame( RGBImageBuffer_, depthImageBuffer_, confidenceImageBuffer_, miscBuffer_,
                            record3DHeader.rgbWidth, record3DHeader.rgbHeight,
                            record3DHeader.depthWidth, record3DHeader.depthHeight,
                            record3DHeader.confidenceWidth, record3DHeader.confidenceHeight,
                            currentDeviceType_, rgbIntrinsicMatrixCoeffs_, cameraPose_ );
#endif
            }
        }
        if (socketHandle_ != -1) {
             Disconnect();
        }
    }

    uint8_t* Record3DStream::DecompressBuffer(const uint8_t* $compressedBuffer, size_t $compressedBufferSize, std::vector<uint8_t> &$destinationBuffer) {
        if ($compressedBufferSize == 0 || $destinationBuffer.empty()) {
             // If destination is empty, it means width/height was 0, so nothing to decompress.
             // If compressed is empty but destination is not, it's an error handled by lzfse.
            return $destinationBuffer.empty() ? nullptr : $destinationBuffer.data();
        }
        size_t outSize = lzfse_decode_buffer($destinationBuffer.data(), $destinationBuffer.size(), $compressedBuffer, $compressedBufferSize, lzfseScratchBuffer_ );
        if ( outSize != $destinationBuffer.size() ) {
            std::cerr << "Decompression error! Expected " << $destinationBuffer.size() << ", got " << outSize << " bytes." << std::endl;
            return nullptr;
        }
        return $destinationBuffer.data();
    }

    uint32_t Record3DStream::ReceiveWholeBuffer(int $socketHandle, uint8_t* $outputBuffer, uint32_t $numBytesToRead) {
        uint32_t numTotalReceivedBytes = 0;
        while ( numTotalReceivedBytes < $numBytesToRead ) {
            uint32_t numRestBytes = $numBytesToRead - numTotalReceivedBytes;
            uint32_t numActuallyReceivedBytes = 0;
            if ($socketHandle < 0) { // Check if socket became invalid during loop (e.g. Disconnect called)
                 std::cerr << "ReceiveWholeBuffer: Invalid socket handle." << std::endl;
                 return numTotalReceivedBytes;
            }
            int recv_ret = usbmuxd_recv( $socketHandle, (char*) ($outputBuffer + numTotalReceivedBytes), numRestBytes, &numActuallyReceivedBytes );
            if ( recv_ret != 0 ) { // USBMUXD_ERR_OK is 0
                std::cerr << "ERROR WHILE RECEIVING DATA! usbmuxd_recv error: " << recv_ret << std::endl;
                return numTotalReceivedBytes;
            }
            if (numActuallyReceivedBytes == 0 && numRestBytes > 0) {
                 std::cerr << "ReceiveWholeBuffer: Connection closed by peer (received 0 bytes)." << std::endl;
                 return numTotalReceivedBytes;
            }
            numTotalReceivedBytes += numActuallyReceivedBytes;
        }
        return numTotalReceivedBytes;
    }
}
