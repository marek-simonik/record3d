#include "record3d/WebRTCSignaling.h"
#include <iostream> // For placeholder logging

namespace Record3D {

WebRTCSignaling::WebRTCSignaling(
    OnLocalDescriptionCallback onLocalDescription,
    OnIceCandidateCallback onIceCandidate,
    OnDataChannelOpenCallback onDataChannelOpen,
    OnDataChannelMessageCallback onDataChannelMessage,
    OnDataChannelErrorCallback onDataChannelError,
    OnConnectionStateChangeCallback onConnectionStateChange)
    : on_local_description_(onLocalDescription),
      on_ice_candidate_(onIceCandidate),
      on_data_channel_open_(onDataChannelOpen),
      on_data_channel_message_(onDataChannelMessage),
      on_data_channel_error_(onDataChannelError),
      on_connection_state_change_(onConnectionStateChange) {
    std::cout << "WebRTCSignaling: Constructor" << std::endl;
    // rtc::InitLogger(rtc::LogLevel::Debug, [](rtc::LogLevel level, const char* message) {
    //     std::cout << "RTC Log: " << message << std::endl;
    // });
}

WebRTCSignaling::~WebRTCSignaling() {
    std::cout << "WebRTCSignaling: Destructor" << std::endl;
    Close();
}

void WebRTCSignaling::InitConnection(const rtc::Configuration& config) {
    rtc_config_ = config; // Store config
    std::cout << "WebRTCSignaling: Initializing connection" << std::endl;

    pc_ = std::make_unique<rtc::PeerConnection>(rtc_config_);

    pc_->onStateChange([this](rtc::PeerConnection::State state) {
        std::cout << "WebRTCSignaling: PeerConnection state changed to " << static_cast<int>(state) << std::endl;
        if (on_connection_state_change_) {
            on_connection_state_change_(state);
        }
        // Could handle states like Failed, Closed here
    });

    pc_->onLocalDescription([this](const rtc::Description& description) {
        std::cout << "WebRTCSignaling: Local description created, type: " << description.typeString() << std::endl;
        if (on_local_description_) {
            on_local_description_(description.typeString(), std::string(description));
        }
    });

    pc_->onLocalCandidate([this](const rtc::Candidate& candidate) {
        std::cout << "WebRTCSignaling: Local ICE candidate: " << candidate.candidate() << ", mid: " << candidate.mid() << std::endl;
        if (on_ice_candidate_) {
            on_ice_candidate_(std::string(candidate), candidate.mid());
        }
    });

    pc_->onDataChannel([this](std::shared_ptr<rtc::DataChannel> incoming_dc) {
        std::cout << "WebRTCSignaling: DataChannel received: " << incoming_dc->label() << std::endl;
        if (incoming_dc->label() == data_channel_label_) {
             std::cout << "WebRTCSignaling: Received data channel matches expected label." << std::endl;
            dc_ = incoming_dc;
            SetupDataChannel(); // Setup callbacks for the existing channel
        } else {
            std::cout << "WebRTCSignaling: Received data channel with unexpected label: " << incoming_dc->label() << std::endl;
            // Optionally close it or handle it differently
            // incoming_dc->close();
        }
    });

    std::cout << "WebRTCSignaling: PeerConnection callbacks set up." << std::endl;
}

void WebRTCSignaling::SetupDataChannel() {
    if (!dc_) {
        std::cerr << "WebRTCSignaling: SetupDataChannel called but dc_ is null" << std::endl;
        return;
    }
    std::cout << "WebRTCSignaling: Setting up DataChannel callbacks for label: " << dc_->label() << std::endl;

    dc_->onOpen([this]() {
        std::cout << "WebRTCSignaling: DataChannel '" << dc_->label() << "' opened." << std::endl;
        if (on_data_channel_open_) {
            on_data_channel_open_();
        }
    });

    dc_->onMessage([this](auto data) { // data is std::variant<rtc::binary, rtc::string>
        if (std::holds_alternative<std::string>(data)) {
            std::cout << "WebRTCSignaling: DataChannel message received (string): " << std::get<std::string>(data) << std::endl;
            if (on_data_channel_message_) {
                on_data_channel_message_(std::get<std::string>(data));
            }
        } else {
            // Assuming binary data for now, can be made more specific
            std::cout << "WebRTCSignaling: DataChannel message received (binary), size: " << std::get<rtc::binary>(data).size() << std::endl;
            // For now, convert binary to string for the callback, this might need adjustment
            // based on actual streaming data format.
             if (on_data_channel_message_) {
                 const rtc::binary& bin_data = std::get<rtc::binary>(data);
                 on_data_channel_message_(std::string(reinterpret_cast<const char*>(bin_data.data()), bin_data.size()));
             }
        }
    });

    dc_->onClosed([this]() {
        std::cout << "WebRTCSignaling: DataChannel '" << dc_->label() << "' closed." << std::endl;
        // Potentially notify via a new callback if needed
    });

    dc_->onError([this](const std::string& error) {
        std::cerr << "WebRTCSignaling: DataChannel '" << dc_->label() << "' error: " << error << std::endl;
        if (on_data_channel_error_) {
            on_data_channel_error_(error);
        }
    });
}


void WebRTCSignaling::CreateOffer() {
    if (!pc_) {
        std::cerr << "WebRTCSignaling: PeerConnection not initialized. Call InitConnection first." << std::endl;
        return;
    }
    std::cout << "WebRTCSignaling: Creating offer" << std::endl;
    // Create a data channel if we are the one initiating (offerer)
    // If this instance can also be an answerer, dc_ might be set by onDataChannel
    if (!dc_) {
        std::cout << "WebRTCSignaling: No existing DataChannel, creating one." << std::endl;
        dc_ = pc_->createDataChannel(data_channel_label_);
        SetupDataChannel(); // Setup callbacks for the newly created channel
    } else {
        std::cout << "WebRTCSignaling: DataChannel already exists, not creating a new one." << std::endl;
    }
    pc_->setLocalDescription(); // This will trigger onLocalDescription
}

void WebRTCSignaling::SetRemoteDescription(const std::string& type_str, const std::string& sdp) {
    if (!pc_) {
        std::cerr << "WebRTCSignaling: PeerConnection not initialized." << std::endl;
        return;
    }
    std::cout << "WebRTCSignaling: Setting remote description, type: " << type_str << std::endl;
    rtc::Description::Type type;
    if (type_str == "offer") {
        type = rtc::Description::Type::Offer;
    } else if (type_str == "answer") {
        type = rtc::Description::Type::Answer;
    } else {
        std::cerr << "WebRTCSignaling: Unknown description type: " << type_str << std::endl;
        return;
    }

    pc_->setRemoteDescription(rtc::Description(sdp, type));
    if (type == rtc::Description::Type::Offer) {
        std::cout << "WebRTCSignaling: Remote description was an offer, creating answer." << std::endl;
        pc_->setLocalDescription(); // Create an answer, triggers onLocalDescription
    }
}

void WebRTCSignaling::AddIceCandidate(const std::string& candidate_str, const std::string& mid) {
    if (!pc_) {
        std::cerr << "WebRTCSignaling: PeerConnection not initialized." << std::endl;
        return;
    }
    std::cout << "WebRTCSignaling: Adding remote ICE candidate" << std::endl;
    pc_->addRemoteCandidate(rtc::Candidate(candidate_str, mid));
}

bool WebRTCSignaling::SendData(const std::string& message) {
    if (!dc_ || !dc_->isOpen()) {
        std::cerr << "WebRTCSignaling: DataChannel not open or not initialized." << std::endl;
        return false;
    }
    // std::cout << "WebRTCSignaling: Sending data: " << message << std::endl;
    return dc_->send(message);
}

void WebRTCSignaling::Close() {
    std::cout << "WebRTCSignaling: Closing connection." << std::endl;
    if (dc_ && dc_->isOpen()) {
        dc_->close();
    }
    if (pc_ && (pc_->state() == rtc::PeerConnection::State::Connected || pc_->state() == rtc::PeerConnection::State::Connecting)) {
         // pc_->close(); // This seems to cause issues if called too soon or in certain states
    }
    dc_.reset();
    pc_.reset();
    std::cout << "WebRTCSignaling: Resources released." << std::endl;
}

} // namespace Record3D
