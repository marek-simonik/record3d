#include <iostream>
#include <vector>
#include <record3d/Record3DStream.h>
#include <mutex>

#ifdef HAS_OPENCV
#include <opencv2/opencv.hpp>
#endif

#pragma clang diagnostic push
#pragma clang diagnostic ignored "-Wmissing-noreturn"

using namespace std;


/**
 * A simple demo app presenting how to use the Record3D library to display RGBD stream.
 */
class Record3DDemoApp
{
public:
    void Run()
    {
        Record3D::Record3DStream stream{};
        stream.onStreamStopped = [&]
        {
            OnStreamStopped();
        };
        stream.onNewFrame = [&](const Record3D::BufferRGB &$rgbFrame,
                                const Record3D::BufferDepth &$depthFrame,
                                uint32_t $frameWidth,
                                uint32_t $frameHeight,
                                Record3D::IntrinsicMatrixCoeffs $K)
        {
            OnNewFrame( $rgbFrame, $depthFrame, $frameWidth, $frameHeight, $K );
        };

        // Try connecting to a device.
        const auto &devs = Record3D::Record3DStream::GetConnectedDevices();
        if ( devs.empty())
        {
            fprintf( stderr,
                     "No iOS devices found. Ensure you have connected your iDevice via USB to this computer.\n" );
            return;
        }
        else
        {
            printf( "Found %lu iOS device(s):\n", devs.size());
            for ( const auto &dev : devs )
            {
                printf( "\tDevice ID: %u\n\tUDID: %s\n\n", dev.productId, dev.udid.c_str());
            }
        }

        const auto &selectedDevice = devs[ 0 ];
        printf( "Trying to connect to device with ID %u.\n", selectedDevice.productId );

        bool isConnected = stream.ConnectToDevice( devs[ 0 ] );
        if ( isConnected )
        {
            printf( "Connected and starting to stream. Enable USB streaming in the Record3D iOS app (https://record3d.app/) in case you don't see RGBD stream.\n" );
            while ( true )
            {
                // Wait for the callback thread to receive new frame and unlock this thread
                mainThreadLock_.lock();

#ifdef HAS_OPENCV
                // Postprocess images
                cv::cvtColor( imgRGB_, imgRGB_, cv::COLOR_RGB2BGR );

                // The TrueDepth camera is a selfie camera; we mirror the RGBD frame so it looks plausible.
                bool areTrueDepthDataBeingStreamed = imgDepth_.rows == Record3D::Record3DStream::MAXIMUM_FRAME_HEIGHT && imgDepth_.cols == Record3D::Record3DStream::MAXIMUM_FRAME_WIDTH;
                if ( areTrueDepthDataBeingStreamed )
                {
                    cv::flip( imgRGB_, imgRGB_, 1 );
                    cv::flip( imgDepth_, imgDepth_, 1 );
                }

                // Show images
                cv::imshow( "RGB", imgRGB_ );
                cv::imshow( "Depth", imgDepth_ );
                cv::waitKey( 1 );
#endif
            }
        }
        else
        {
            fprintf( stderr,
                     "Could not connect to iDevice. Make sure you have opened the Record3D iOS app (https://record3d.app/).\n" );
        }
    }

private:
    void OnStreamStopped()
    {
        fprintf( stderr, "Stream stopped!" );
    }

    void OnNewFrame(const Record3D::BufferRGB &$rgbFrame,
                    const Record3D::BufferDepth &$depthFrame,
                    uint32_t $frameWidth,
                    uint32_t $frameHeight,
                    Record3D::IntrinsicMatrixCoeffs $K)
    {
#ifdef HAS_OPENCV
        // When we switch between the TrueDepth and the LiDAR camera, the size frame size changes.
        // Recreate the RGB and Depth images with fitting size.
        if (    imgRGB_.rows != $frameHeight || imgRGB_.cols != $frameWidth
             || imgDepth_.rows != $frameHeight || imgDepth_.cols != $frameWidth )
        {
            imgRGB_.release();
            imgDepth_.release();

            imgRGB_ = cv::Mat::zeros( $frameHeight, $frameWidth, CV_8UC3);
            imgDepth_ = cv::Mat::zeros( $frameHeight, $frameWidth, CV_32F );
        }

        // The `BufferRGB` and `BufferDepth` may be larger than the actual payload, therefore the true frame size is computed.
        constexpr int numRGBChannels = 3;
        memcpy( imgRGB_.data, $rgbFrame.data(), $frameWidth * $frameHeight * numRGBChannels * sizeof(uint8_t));
        memcpy( imgDepth_.data, $depthFrame.data(), $frameWidth * $frameHeight * sizeof(float));
#endif
        mainThreadLock_.unlock();
    }

private:
#if WIN32
    std::recursive_mutex mainThreadLock_{};
#else
    std::mutex mainThreadLock_{};
#endif

#ifdef HAS_OPENCV
    cv::Mat imgRGB_ = cv::Mat::zeros(Record3D::Record3DStream::MAXIMUM_FRAME_HEIGHT, Record3D::Record3DStream::MAXIMUM_FRAME_WIDTH, CV_8UC3);;
    cv::Mat imgDepth_ = cv::Mat::zeros(Record3D::Record3DStream::MAXIMUM_FRAME_HEIGHT, Record3D::Record3DStream::MAXIMUM_FRAME_WIDTH, CV_32F );;
#endif
};


int main()
{
    Record3DDemoApp app{};
    app.Run();
}

#pragma clang diagnostic pop