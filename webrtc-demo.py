#!/usr/bin/env python3

import record3d
import asyncio
import websockets
import json
import cv2
import numpy as np
import logging
import signal # For graceful exit

# --- Configuration ---
SIGNALING_SERVER_URL = "ws://localhost:8765"
# In a real scenario:
# - This script (client A) sends its offer/ICE to the signaling server.
# - The Record3D iOS app (client B) also connects to this server.
# - The server relays messages between client A and client B.
# - Client B receives A's offer, creates an answer, sends it back via server.
# - Client B sends its ICE candidates back via server.

logging.basicConfig(level=logging.INFO)
logger = logging.getLogger("webrtc-demo")

stream = None
websocket_client = None # Global websocket client instance
frame_count = 0
event_loop = None # To store the asyncio event loop for creating tasks from sync callbacks
cv_windows_created = False # Flag to track if OpenCV windows have been made

def on_new_frame_callback():
    global frame_count, cv_windows_created
    frame_count += 1

    try:
        rgb_frame = stream.get_rgb_frame()
        depth_frame = stream.get_depth_frame()

        if rgb_frame is not None and depth_frame is not None:
            if not cv_windows_created: # Create windows on first frame
                cv2.namedWindow("RGB Frame", cv2.WINDOW_AUTOSIZE)
                cv2.namedWindow("Depth Frame", cv2.WINDOW_AUTOSIZE)
                cv_windows_created = True

            if frame_count % 30 == 0:
                 logger.info(f"RGB frame shape: {rgb_frame.shape}, Depth frame shape: {depth_frame.shape}")

            cv2.imshow("RGB Frame", cv2.cvtColor(rgb_frame, cv2.COLOR_RGB2BGR))
            depth_display = depth_frame.copy()
            min_val, max_val, _, _ = cv2.minMaxLoc(depth_display)
            if max_val > min_val:
                depth_display = (depth_display - min_val) / (max_val - min_val)
            depth_display_8bit = (depth_display * 255).astype(np.uint8)
            cv2.imshow("Depth Frame", depth_display_8bit)

            # Moved waitKey to main loop
        else:
            logger.warning("Received None for frame data.")
    except Exception as e:
        logger.error(f"Error processing frame: {e}")

def on_stream_stopped_callback():
    logger.info("Stream stopped.")
    if event_loop and not event_loop.is_closed():
        logger.info("Signaling main loop to stop due to stream stopped.")
        event_loop.call_soon_threadsafe(lambda: asyncio.create_task(shutdown()))


async def send_signaling_message_async(message_dict):
    global websocket_client
    if websocket_client and websocket_client.open:
        try:
            await websocket_client.send(json.dumps(message_dict))
            logger.info(f"Sent signaling message: {message_dict['type']}")
        except Exception as e:
            logger.error(f"Error sending signaling message: {e}")
    else:
        logger.warning("WebSocket client not available or not open. Cannot send message.")

def python_on_local_sdp_callback(type, sdp):
    logger.info(f"Python received local SDP of type {type}.")
    message = {"type": type, "sdp": sdp}
    if event_loop and not event_loop.is_closed():
        asyncio.run_coroutine_threadsafe(send_signaling_message_async(message), event_loop)

def python_on_local_ice_candidate_callback(candidate, mid):
    logger.info(f"Python received local ICE candidate for mid {mid}.")
    message = {"type": "candidate", "candidate": {"candidate": candidate, "sdpMid": mid, "sdpMLineIndex": 0}}
    if event_loop and not event_loop.is_closed():
        asyncio.run_coroutine_threadsafe(send_signaling_message_async(message), event_loop)


async def handle_signaling_message_from_server(message_str):
    global stream
    try:
        message = json.loads(message_str)
        logger.info(f"Received signaling message from server: {message.get('type')}")

        if not stream:
            logger.warning("Stream object not initialized when handling server message.")
            return

        msg_type = message.get("type")
        if msg_type == "answer":
            sdp = message.get("sdp")
            if sdp:
                logger.info("Calling stream.set_remote_sdp with received answer.")
                stream.set_remote_sdp("answer", sdp)
        elif msg_type == "candidate":
            candidate_data = message.get("candidate")
            if candidate_data:
                candidate_str = candidate_data.get("candidate")
                sdp_mid = candidate_data.get("sdpMid")
                if candidate_str and sdp_mid:
                    logger.info(f"Calling stream.add_ice_candidate for mid {sdp_mid}.")
                    stream.add_ice_candidate(candidate_str, sdp_mid)
    except json.JSONDecodeError:
        logger.error(f"Failed to decode JSON from signaling server: {message_str}")
    except Exception as e:
        logger.error(f"Error handling server message: {e}")

async def signaling_client_task():
    global websocket_client, stream
    uri = SIGNALING_SERVER_URL

    while not (event_loop and event_loop.is_closed() or (hasattr(event_loop, '_stopping') and event_loop._stopping)): # Check if loop is stopping
        try:
            logger.info(f"Attempting to connect to signaling server at {uri}...")
            async with websockets.connect(uri) as websocket:
                websocket_client = websocket
                logger.info("Connected to signaling server.")

                if stream and not (event_loop and event_loop.is_closed()):
                    logger.info("Calling stream.connect_to_device_via_webrtc() from Python...")
                    success = stream.connect_to_device_via_webrtc("signaling_server_placeholder", 0)
                    if not success:
                        logger.error("Failed to initiate WebRTC connection from Record3DStream.")
                        return

                async for message_str in websocket:
                    if event_loop and event_loop.is_closed(): break
                    await handle_signaling_message_from_server(message_str)

        except (websockets.exceptions.ConnectionClosedError, ConnectionRefusedError) as e:
            logger.warning(f"WebSocket connection issue: {e}. Retrying in 5s...")
        except Exception as e: # Catch other potential errors like server not available during send
            logger.error(f"Signaling client error: {e}. Retrying in 5s...")
        finally:
            websocket_client = None
            if not (event_loop and event_loop.is_closed()):
                 await asyncio.sleep(5)


async def main_async():
    global stream, event_loop, cv_windows_created
    event_loop = asyncio.get_running_loop()
    stop_event = asyncio.Event()

    def sig_handler():
        logger.info("Shutdown signal received.")
        stop_event.set()

    for sig_name in (signal.SIGINT, signal.SIGTERM):
        try: # In some environments (like this sandbox), add_signal_handler might not be available
            event_loop.add_signal_handler(sig_name, sig_handler)
        except NotImplementedError:
            logger.warning(f"Signal handler for {sig_name} could not be set. Use Ctrl+C if running interactively, or it will rely on task completion/errors.")


    try:
        stream = record3d.Record3DStream()
        stream.on_new_frame = on_new_frame_callback
        stream.on_stream_stopped = on_stream_stopped_callback
        stream.on_local_sdp = python_on_local_sdp_callback
        stream.on_local_ice_candidate = python_on_local_ice_candidate_callback
        logger.info("Record3DStream initialized with WebRTC callbacks.")

        signaling_task = asyncio.create_task(signaling_client_task())

        while not stop_event.is_set():
            if cv_windows_created: # Only call waitKey if windows are expected
                if cv2.waitKey(30) & 0xFF == ord('q'):
                    logger.info("User pressed 'q' in OpenCV window, stopping.")
                    stop_event.set()
            else:
                await asyncio.sleep(0.03)

            if signaling_task.done() and not stop_event.is_set():
                logger.info("Signaling task ended.")
                try:
                    signaling_task.result()
                except Exception as e:
                    logger.error(f"Signaling task failed: {e}")
                stop_event.set()

    except Exception as e:
        logger.error(f"An error occurred in main_async: {e}", exc_info=True)
        stop_event.set() # Ensure shutdown on main error
    finally:
        logger.info("Main loop ended. Initiating shutdown sequence.")
        if 'signaling_task' in locals() and not signaling_task.done():
            signaling_task.cancel()
            try:
                await signaling_task
            except asyncio.CancelledError:
                logger.info("Signaling task cancelled.")
            except Exception as e:
                logger.error(f"Error during signaling task cleanup: {e}")

        await shutdown_resources()

async def shutdown_resources():
    global stream, websocket_client
    logger.info("shutdown_resources called.")
    if websocket_client and websocket_client.open:
        try:
            await websocket_client.close()
            logger.info("WebSocket client closed.")
        except Exception as e:
            logger.error(f"Error closing websocket: {e}")

    if stream:
        try:
            logger.info("Disconnecting Record3DStream...")
            stream.disconnect()
        except Exception as e:
            logger.error(f"Error disconnecting stream: {e}")

    cv2.destroyAllWindows()
    logger.info("OpenCV windows destroyed. Shutdown complete.")

if __name__ == "__main__":
    try:
        asyncio.run(main_async())
    except KeyboardInterrupt:
        logger.info("Process interrupted by user (KeyboardInterrupt).")
    except Exception as e:
        logger.error(f"Unhandled exception in script execution: {e}", exc_info=True)
    finally:
        cv2.destroyAllWindows()
        logger.info("Exiting demo script.")
