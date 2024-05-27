#include <iostream>
#include <vector>
#include <record3d/Record3DStream.h>
#include <mutex>
#include <cstring>

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
        Record3D::Record3DStream stream { };
        stream.onStreamStopped = [&]
        {
            OnStreamStopped();
        };
        stream.onNewFrame = [&]( const Record3D::BufferRGB &$rgbFrame,
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
                                 Record3D::CameraPose $cameraPose )
        {
            OnNewFrame( $rgbFrame, $depthFrame, $confFrame, $miscData, $rgbWidth, $rgbHeight, $depthWidth, $depthHeight, $confWidth, $confHeight, $deviceType, $K, $cameraPose );
        };

        // Try connecting to a device.
        const auto &devs = Record3D::Record3DStream::GetConnectedDevices();
        if ( devs.empty() )
        {
            fprintf( stderr,
                     "No iOS devices found. Ensure you have connected your iDevice via USB to this computer.\n" );
            return;
        }
        else
        {
            printf( "Found %lu iOS device(s):\n", devs.size() );
            for ( const auto &dev: devs )
            {
                printf( "\tDevice ID: %u\n\tUDID: %s\n\n", dev.productId, dev.udid.c_str() );
            }
        }

        const auto &selectedDevice = devs[0];
        printf( "Trying to connect to device with ID %u.\n", selectedDevice.productId );

        bool isConnected = stream.ConnectToDevice( devs[0] );
        if ( isConnected )
        {
            printf( "Connected and starting to stream. Enable USB streaming in the Record3D iOS app (https://record3d.app/) in case you don't see RGBD stream.\n" );
            while ( true )
            {
                // Wait for the callback thread to receive new frame and unlock this thread
#ifdef HAS_OPENCV
                cv::Mat rgb, depth, confidence;
                {
                    std::lock_guard<std::recursive_mutex> lock( mainThreadLock_ );

                    if ( imgRGB_.cols == 0 || imgRGB_.rows == 0 || imgDepth_.cols == 0 || imgDepth_.rows == 0 )
                    {
                        continue;
                    }

                    if ( imgConfidence_.cols > 0 && imgConfidence_.rows > 0 )
                    {
                        confidence = imgConfidence_.clone();
                    }

                    rgb = imgRGB_.clone();
                    depth = imgDepth_.clone();
                }
                // Postprocess images
                cv::cvtColor( rgb, rgb, cv::COLOR_RGB2BGR );

                // The TrueDepth camera is a selfie camera; we mirror the RGBD frame so it looks plausible.
                if ( currentDeviceType_ == Record3D::R3D_DEVICE_TYPE__FACEID )
                {
                    cv::flip( rgb, rgb, 1 );
                    cv::flip( depth, depth, 1 );
                }

                if ( imgConfidence_.cols > 0 && imgConfidence_.rows > 0 )
                {
                    cv::imshow( "Confidence", confidence * 100 );
                }

                // Show images
                cv::imshow( "RGB", rgb );
                cv::imshow( "Depth", depth );
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

    void OnNewFrame( const Record3D::BufferRGB &$rgbFrame,
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
                     Record3D::CameraPose $cameraPose )
    {
        currentDeviceType_ = (Record3D::DeviceType) $deviceType;

#ifdef HAS_OPENCV
        std::lock_guard<std::recursive_mutex> lock( mainThreadLock_ );
        // When we switch between the TrueDepth and the LiDAR camera, the size frame size changes.
        // Recreate the RGB and Depth images with fitting size.
        if ( imgRGB_.rows != $rgbHeight || imgRGB_.cols != $rgbWidth
             || imgDepth_.rows != $depthHeight || imgDepth_.cols != $depthWidth )
        {
            imgRGB_.release();
            imgDepth_.release();

            imgRGB_ = cv::Mat::zeros( $rgbHeight, $rgbWidth, CV_8UC3 );
            imgDepth_ = cv::Mat::zeros( $depthHeight, $depthWidth, CV_32F );
        }

        bool isConfidenceMapIncluded = $confWidth > 0 && $confHeight > 0;

        if ( imgConfidence_.rows != $confWidth || imgConfidence_.cols != $confHeight )
        {
            imgConfidence_.release();

            if ( isConfidenceMapIncluded )
            {
                imgConfidence_ = cv::Mat::zeros( $confHeight, $confWidth, CV_8U );
            }
        }

        // The `BufferRGB` and `BufferDepth` may be larger than the actual payload, therefore the true frame size is computed.
        constexpr int numRGBChannels = 3;
        memcpy( imgRGB_.data, $rgbFrame.data(), $rgbWidth * $rgbHeight * numRGBChannels * sizeof( uint8_t ) );
        memcpy( imgDepth_.data, $depthFrame.data(), $depthWidth * $depthHeight * sizeof( float ) );

        if ( isConfidenceMapIncluded )
        {
            memcpy( imgConfidence_.data, $confFrame.data(), $confWidth * $confHeight * sizeof(uint8_t) );
        }
#endif
    }

private:
    std::recursive_mutex mainThreadLock_ { };
    Record3D::DeviceType currentDeviceType_ { };

#ifdef HAS_OPENCV
    cv::Mat imgRGB_ { };
    cv::Mat imgDepth_ { };
    cv::Mat imgConfidence_ { };
#endif
};


int main()
{
    Record3DDemoApp app { };
    app.Run();
}

#pragma clang diagnostic pop
