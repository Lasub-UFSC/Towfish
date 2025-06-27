from fastapi import WebSocket

async def websocket_handler(websocket: WebSocket):
    await websocket.accept()
    # read data from db and send while websocket connected