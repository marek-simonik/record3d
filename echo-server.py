#!/usr/bin/env python3
import asyncio
import websockets
import logging
import json

logging.basicConfig(level=logging.INFO)
logger = logging.getLogger("echo-server")

connected_clients = set()

async def PipedUnidirectionalServer(websocket, path):
    global connected_clients
    logger.info(f"Client connected from {websocket.remote_address}")
    connected_clients.add(websocket)
    try:
        async for message in websocket:
            logger.info(f"Received message: {message[:100]}...") # Log first 100 chars
            data = json.loads(message)

            # Simple logic: if it's an offer, send a canned answer after a delay
            # If it's a candidate, just echo it back (simulating peer sending its candidate)
            # This is a very basic simulation for one client.

            response = None
            if data.get("type") == "offer":
                logger.info("Received offer, crafting a dummy answer and a dummy candidate.")
                # Dummy Answer
                await asyncio.sleep(0.1) # Simulate network delay
                answer_sdp = "v=0\\r\\no=- 0 0 IN IP4 127.0.0.1\\r\\ns=-\\r\\nt=0 0\\r\\na=msid-semantic: WMS\\r\\n" # Extremely minimal
                response_answer = {"type": "answer", "sdp": answer_sdp}
                await websocket.send(json.dumps(response_answer))
                logger.info("Sent dummy answer.")

                # Dummy ICE Candidate
                await asyncio.sleep(0.1)
                candidate_info = {"candidate": "candidate:12345 1 udp 2122252543 192.168.1.100 12345 typ host", "sdpMid": "0", "sdpMLineIndex": 0}
                response_candidate = {"type": "candidate", "candidate": candidate_info}
                await websocket.send(json.dumps(response_candidate))
                logger.info("Sent dummy ICE candidate.")

            elif data.get("type") == "candidate":
                # Echo back candidates for simplicity in this dummy server
                # In a real server, you'd send to the *other* peer
                # For self-testing, echoing it back means the client adds its own candidate as remote.
                # This isn't realistic but tests the pathway.
                # logger.info("Received candidate, echoing back (not realistic for P2P).")
                # await websocket.send(message)
                logger.info("Received candidate. In a real server, this would go to the other peer.")
                pass # Don't echo candidate back to self for this test.

            elif data.get("type") == "answer":
                 logger.info("Received answer (likely from self if echoing). Ignoring.")
                 pass


    except websockets.exceptions.ConnectionClosedError:
        logger.info(f"Client disconnected from {websocket.remote_address}")
    except Exception as e:
        logger.error(f"Error in server handler: {e}")
    finally:
        connected_clients.remove(websocket)

async def main_server():
    host = "localhost"
    port = 8765
    logger.info(f"Starting WebSocket echo server on ws://{host}:{port}")
    async with websockets.serve(PipedUnidirectionalServer, host, port):
        await asyncio.Future()  # run forever

if __name__ == "__main__":
    try:
        asyncio.run(main_server())
    except KeyboardInterrupt:
        logger.info("Server shutting down.")
    except Exception as e:
        logger.error(f"Server failed: {e}")
