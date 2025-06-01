#ifndef WEBRTC_SIGNALING_H
#define WEBRTC_SIGNALING_H

#include <rtc/peerconnection.hpp>
#include <rtc/datachannel.hpp>
#include <rtc/rtc.hpp> // For rtc::Configuration

#include <string>
#include <functional>
#include <memory>

namespace Record3D {

class WebRTCSignaling {
public:
    // Callback types
    using OnLocalDescriptionCallback = std::function<void(const std::string& type, const std::string& sdp)>;
    using OnIceCandidateCallback = std::function<void(const std::string& candidate, const std::string& mid)>;
    using OnDataChannelOpenCallback = std::function<void()>;
    using OnDataChannelMessageCallback = std::function<void(const std::string& message)>; // Or binary data if needed
    using OnDataChannelErrorCallback = std::function<void(const std::string& error)>;
    using OnConnectionStateChangeCallback = std::function<void(rtc::PeerConnection::State state)>;


    WebRTCSignaling(OnLocalDescriptionCallback onLocalDescription,
                    OnIceCandidateCallback onIceCandidate,
                    OnDataChannelOpenCallback onDataChannelOpen,
                    OnDataChannelMessageCallback onDataChannelMessage,
                    OnDataChannelErrorCallback onDataChannelError,
                    OnConnectionStateChangeCallback onConnectionStateChange);
    ~WebRTCSignaling();

    void InitConnection(const rtc::Configuration& config);
    void CreateOffer();
    void SetRemoteDescription(const std::string& type, const std::string& sdp);
    void AddIceCandidate(const std::string& candidate, const std::string& mid);
    bool SendData(const std::string& message); // Or binary data

    void Close();

private:
    void SetupPeerConnection();
    void SetupDataChannel();

    std::unique_ptr<rtc::PeerConnection> pc_;
    std::shared_ptr<rtc::DataChannel> dc_;
    rtc::Configuration rtc_config_;

    // Callbacks
    OnLocalDescriptionCallback on_local_description_;
    OnIceCandidateCallback on_ice_candidate_;
    OnDataChannelOpenCallback on_data_channel_open_;
    OnDataChannelMessageCallback on_data_channel_message_;
    OnDataChannelErrorCallback on_data_channel_error_;
    OnConnectionStateChangeCallback on_connection_state_change_;

    const std::string data_channel_label_ = "Record3DStream";
};

} // namespace Record3D

#endif // WEBRTC_SIGNALING_H
