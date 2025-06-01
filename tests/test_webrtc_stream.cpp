#include "record3d/Record3DStream.h"
#include "record3d/Record3DStructs.h"
#include <cmath> // For std::abs

#include <iostream>
#include <vector>
#include <string>
#include <cassert>
#include <functional>
#include <cstring>

#include <lzfse.h>

// Define Record3DHeader struct locally for test, or move to Record3DStructs.h
struct Record3DHeader {
    uint32_t rgbWidth, rgbHeight, depthWidth, depthHeight, confidenceWidth, confidenceHeight;
    uint32_t rgbSize, depthSize, confidenceMapSize, miscSize, deviceType;
};

// --- Test Utilities ---
bool tests_passed = true;
int tests_run = 0;
std::string current_test_name_global; // For assert macro

// Simple assert macro to provide more context
#define TEST_ASSERT(condition, message) \
    if (!(condition)) { \
        tests_passed = false; \
        std::cerr << current_test_name_global << ": FAILED assertion: " << #condition << " (" << message << ") at " << __FILE__ << ":" << __LINE__ << std::endl; \
        throw std::runtime_error(std::string("Assertion failed: ") + message); \
    } else { \
        std::cout << current_test_name_global << ": PASSED assertion: " << #condition << std::endl; \
    }


void run_test(const std::function<void()>& test_func, const std::string& test_name) {
    tests_run++;
    current_test_name_global = test_name;
    std::cout << "Running test: " << test_name << "..." << std::endl;
    try {
        test_func();
        std::cout << test_name << ": COMPLETED (see assertion results above)" << std::endl;
    } catch (const std::exception& e) {
        // Assertion failures are already logged by TEST_ASSERT and set tests_passed = false
        // This will catch other std::exceptions from the test logic itself.
        if (tests_passed) { // Only mark as FAILED if not already marked by TEST_ASSERT
             std::cerr << test_name << ": FAILED with exception: " << e.what() << std::endl;
        }
        tests_passed = false; // Ensure it's marked as failed
    } catch (...) {
        if (tests_passed) {
            std::cerr << test_name << ": FAILED with unknown exception" << std::endl;
        }
        tests_passed = false;
    }
}

// --- Mock Data Helper ---
const unsigned char minimal_jpeg[] = {
    0xFF, 0xD8, 0xFF, 0xE0, 0x00, 0x10, 0x4A, 0x46, 0x49, 0x46, 0x00, 0x01,
    0x01, 0x00, 0x00, 0x01, 0x00, 0x01, 0x00, 0x00, 0xFF, 0xDB, 0x00, 0x43,
    0x00, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01,
    0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01,
    0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01,
    0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0xFF, 0xC0,
    0x00, 0x0B, 0x08, 0x00, 0x01, 0x00, 0x01, 0x01, 0x01, 0x11, 0x00, 0xFF,
    0xC4, 0x00, 0x14, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFF, 0xDA, 0x00, 0x08,
    0x01, 0x01, 0x00, 0x00, 0x3F, 0x00, 0xA0, 0xFF, 0xD9
};
const size_t minimal_jpeg_len = sizeof(minimal_jpeg);

std::vector<uint8_t> compress_lzfse(const std::vector<uint8_t>& input) {
    if (input.empty()) return {};
    std::vector<uint8_t> compressed_buffer(input.size() * 2 + 10); // Ensure enough space
    uint8_t lzfse_scratch_buffer[lzfse_encode_scratch_size()]; // Scratch for encode
    size_t compressed_size = lzfse_encode_buffer(
        compressed_buffer.data(), compressed_buffer.size(),
        input.data(), input.size(),
        lzfse_scratch_buffer);
    if (compressed_size == 0 && !input.empty()) { // Allow empty input to result in empty output
        throw std::runtime_error("LZFSE compression failed for non-empty input");
    }
    compressed_buffer.resize(compressed_size);
    return compressed_buffer;
}


// --- Test Cases ---

void test_webrtc_connection_offer() {
    Record3D::Record3DStream stream;
    bool offer_received = false;
    std::string rcv_type, rcv_sdp;

    // Use the C++ test callback
    stream.test_on_local_sdp_cpp_ = [&](const std::string& type, const std::string& sdp) {
        offer_received = true;
        rcv_type = type;
        rcv_sdp = sdp;
    };

    stream.ConnectToDeviceViaWebRTC("dummy_ip", 1234);

    TEST_ASSERT(offer_received, "Offer SDP was not received via C++ test callback");
    TEST_ASSERT(rcv_type == "offer", "SDP type was not 'offer'");
    TEST_ASSERT(!rcv_sdp.empty(), "SDP string was empty");
    std::cout << "  Offer SDP received (first 60 chars): " << rcv_sdp.substr(0, 60) << "..." << std::endl;
}

void test_webrtc_python_set_remote_sdp_and_ice() {
    Record3D::Record3DStream stream;
    stream.ConnectToDeviceViaWebRTC("dummy_ip", 1234);

    std::string dummy_answer_sdp =
        "v=0\r\n"
        "o=- 0 0 IN IP4 127.0.0.1\r\n"
        "s=-\r\n"
        "t=0 0\r\n"
        "m=application 9 DTLS/SCTP 5000\r\n" // Port 5000 for SCTP
        "c=IN IP4 0.0.0.0\r\n"
        "a=sctp-port:5000\r\n" // SCTP port used by the data channel
        "a=max-message-size:100000\r\n"
        "a=setup:active\r\n" // Typically 'active' if the offerer was 'actpass' or 'passive'
        "a=mid:0\r\n"        // Matches the 'mid' of the data channel in the offer
        "a=ice-ufrag:dummyufragans\r\n" // Dummy ICE user fragment for answer
        "a=ice-pwd:dummypwdans\r\n"   // Dummy ICE password for answer
        "a=fingerprint:sha-256 AA:BB:CC:DD:EE:FF:00:11:22:33:44:55:66:77:88:99:AA:BB:CC:DD:EE:FF:00:11:22:33:44:55:66:77:88:99\r\n"; // Dummy fingerprint

    std::cout << "  Calling SetRemoteSdpFromPython with a more valid dummy answer..." << std::endl;
    try {
      stream.SetRemoteSdpFromPython("answer", dummy_answer_sdp);
      TEST_ASSERT(true, "SetRemoteSdpFromPython called (pathway test, no crash)");
    } catch (const std::exception& e) {
      // If libdatachannel still throws an error with this dummy SDP, the test might need adjustment
      // or it indicates a deeper issue in how SetRemoteDescription is used / expected by libdatachannel.
      std::cerr << "SetRemoteSdpFromPython threw an exception: " << e.what() << std::endl;
      TEST_ASSERT(false, "SetRemoteSdpFromPython threw an exception with dummy answer");
    }

    std::cout << "  Calling AddIceCandidateFromPython with a more valid dummy candidate..." << std::endl;
    // A very minimal but structurally plausible candidate string
    std::string dummy_candidate = "candidate:1234567890 1 udp 2130706431 192.168.0.100 12345 typ host";
    try {
        stream.AddIceCandidateFromPython(dummy_candidate, "0"); // mid often "0" or "video" or "audio"
        TEST_ASSERT(true, "AddIceCandidateFromPython called (pathway test, no crash)");
    } catch (const std::exception& e) {
        std::cerr << "AddIceCandidateFromPython threw an exception: " << e.what() << std::endl;
        TEST_ASSERT(false, "AddIceCandidateFromPython threw an exception with dummy candidate");
    }
}


void test_webrtc_frame_data_processing() {
    Record3D::Record3DStream stream;
    bool frame_received = false;

    Record3DHeader header_orig; // Using local struct definition
    // Using 2x2 to avoid issues with 1-element LZFSE compression
    header_orig.rgbWidth = 1; header_orig.rgbHeight = 1; // JPEG is fine as 1x1
    header_orig.depthWidth = 2; header_orig.depthHeight = 2;
    header_orig.confidenceWidth = 2; header_orig.confidenceHeight = 2;
    header_orig.rgbSize = minimal_jpeg_len;

    std::vector<float> depth_data_raw = {1.23f, 2.34f, 3.45f, 4.56f}; // 2x2
    std::vector<uint8_t> depth_data_bytes(depth_data_raw.size() * sizeof(float));
    memcpy(depth_data_bytes.data(), depth_data_raw.data(), depth_data_bytes.size());
    std::vector<uint8_t> depth_data_compressed = compress_lzfse(depth_data_bytes);
    header_orig.depthSize = depth_data_compressed.size();
    TEST_ASSERT(header_orig.depthSize > 0, "LZFSE compression of depth data failed (returned 0 size)");

    std::vector<uint8_t> conf_data_raw = {0, 1, 2, 1}; // 2x2
    std::vector<uint8_t> conf_data_compressed = compress_lzfse(conf_data_raw);
    header_orig.confidenceMapSize = conf_data_compressed.size();
    TEST_ASSERT(header_orig.confidenceMapSize > 0, "LZFSE compression of confidence data failed (returned 0 size)");

    header_orig.miscSize = 0;
    header_orig.deviceType = static_cast<uint32_t>(Record3D::DeviceType::R3D_DEVICE_TYPE__LIDAR);

    Record3D::IntrinsicMatrixCoeffs intrinsics_orig = {100.0f, 101.0f, 0.5f, 0.6f};
    Record3D::CameraPose pose_orig = {0.1f, 0.2f, 0.3f, 0.9f, 1.0f, 1.1f, 1.2f};

    std::vector<uint8_t> mock_message_bytes;
    mock_message_bytes.resize(sizeof(header_orig) + sizeof(intrinsics_orig) + sizeof(pose_orig) +
                              header_orig.rgbSize + header_orig.depthSize + header_orig.confidenceMapSize);

    size_t offset = 0;
    memcpy(mock_message_bytes.data() + offset, &header_orig, sizeof(header_orig)); offset += sizeof(header_orig);
    memcpy(mock_message_bytes.data() + offset, &intrinsics_orig, sizeof(intrinsics_orig)); offset += sizeof(intrinsics_orig);
    memcpy(mock_message_bytes.data() + offset, &pose_orig, sizeof(pose_orig)); offset += sizeof(pose_orig);
    memcpy(mock_message_bytes.data() + offset, minimal_jpeg, header_orig.rgbSize); offset += header_orig.rgbSize;
    memcpy(mock_message_bytes.data() + offset, depth_data_compressed.data(), header_orig.depthSize); offset += header_orig.depthSize;
    memcpy(mock_message_bytes.data() + offset, conf_data_compressed.data(), header_orig.confidenceMapSize);

    std::string mock_message_str(reinterpret_cast<const char*>(mock_message_bytes.data()), mock_message_bytes.size());

    stream.onNewFrame =
        [&](const Record3D::BufferRGB &rgbFrame,
            const Record3D::BufferDepth &depthFrame_uint8,
            const Record3D::BufferConfidence &confFrame,
            const Record3D::BufferMisc &miscData,
            uint32_t   rgbWidth, uint32_t   rgbHeight,
            uint32_t   depthWidth, uint32_t   depthHeight,
            uint32_t   confWidth, uint32_t   confHeight,
            Record3D::DeviceType deviceType,
            Record3D::IntrinsicMatrixCoeffs K,
            Record3D::CameraPose pose) {

        frame_received = true;
        TEST_ASSERT(rgbWidth == header_orig.rgbWidth, "RGB width mismatch");
        TEST_ASSERT(rgbHeight == header_orig.rgbHeight, "RGB height mismatch");
        TEST_ASSERT(depthWidth == header_orig.depthWidth, "Depth width mismatch");
        TEST_ASSERT(depthHeight == header_orig.depthHeight, "Depth height mismatch");
        TEST_ASSERT(confWidth == header_orig.confidenceWidth, "Confidence width mismatch");
        TEST_ASSERT(confHeight == header_orig.confidenceHeight, "Confidence height mismatch");

        TEST_ASSERT(std::abs(K.fx - intrinsics_orig.fx) < 0.001f, "Intrinsic fx mismatch");
        TEST_ASSERT(std::abs(pose.qw - pose_orig.qw) < 0.001f, "Pose qw mismatch");
        TEST_ASSERT(deviceType == (Record3D::DeviceType)header_orig.deviceType, "Device type mismatch");

        TEST_ASSERT(rgbFrame.size() == (header_orig.rgbWidth * header_orig.rgbHeight * 3), "Decoded RGB size mismatch");

        TEST_ASSERT(depthFrame_uint8.size() == (header_orig.depthWidth * header_orig.depthHeight * sizeof(float)), "Decompressed depth size mismatch");
        // Compare first float
        float depth_val;
        memcpy(&depth_val, depthFrame_uint8.data(), sizeof(float));
        TEST_ASSERT(std::abs(depth_val - depth_data_raw[0]) < 0.001f, "First depth data element mismatch");
        // Compare last float
        memcpy(&depth_val, depthFrame_uint8.data() + (depth_data_raw.size()-1)*sizeof(float), sizeof(float));
        TEST_ASSERT(std::abs(depth_val - depth_data_raw.back()) < 0.001f, "Last depth data element mismatch");


        TEST_ASSERT(confFrame.size() == (header_orig.confidenceWidth * header_orig.confidenceHeight), "Decompressed confidence size mismatch");
        TEST_ASSERT(confFrame[0] == conf_data_raw[0], "First confidence data element mismatch");
        TEST_ASSERT(confFrame.back() == conf_data_raw.back(), "Last confidence data element mismatch");
    };

    stream.ConnectToDeviceViaWebRTC("dummy_addr_for_init",0);
    std::cout << "  Calling OnWebRTCDataChannelMessage with mock data..." << std::endl;
    stream.OnWebRTCDataChannelMessage(mock_message_str); // Call the (now public for testing) method

    TEST_ASSERT(frame_received, "onNewFrame callback was not triggered for WebRTC data");
}


int main() {
    std::cout << "Starting WebRTC Stream tests..." << std::endl;

    run_test(test_webrtc_connection_offer, "TestWebRTCOfferGeneration");
    run_test(test_webrtc_python_set_remote_sdp_and_ice, "TestWebRTCRemoteSdpIceInput");
    run_test(test_webrtc_frame_data_processing, "TestWebRTCFrameDataProcessing");

    std::cout << "\nTests finished." << std::endl;
    std::cout << tests_run << " tests run." << std::endl;
    if (tests_passed) {
        std::cout << "All tests PASSED." << std::endl;
        return 0;
    } else {
        std::cout << "Some tests FAILED." << std::endl;
        return 1;
    }
}
