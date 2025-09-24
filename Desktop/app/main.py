from fastapi import FastAPI, WebSocket, WebSocketDisconnect
from fastapi.staticfiles import StaticFiles
import uvicorn
from components.PollingClient.PollingClient import PollingClient
from contextlib import asynccontextmanager

class ConnectionManager:
    def __init__(self):
        self.active_connections: list[WebSocket] = []

    async def connect(self, websocket: WebSocket):
        print("new connection")
        await websocket.accept()
        self.active_connections.append(websocket)

    def disconnect(self, websocket: WebSocket):
        if websocket in self.active_connections:
            self.active_connections.remove(websocket)

    async def broadcast(self, message):
        # print("Broadcasting message")
        # print(f"Connections: {self.active_connections}")
        for connection in self.active_connections:
            try:
                await connection.send_json(message)
            except WebSocketDisconnect:
                self.disconnect(connection)

manager = None

pollingClient = None

@asynccontextmanager
async def lifespan(app: FastAPI):
    global manager
    manager = ConnectionManager()

    global pollingClient
    pollingClient = PollingClient(setFrequency=20,onNewData=manager.broadcast,verbose=False)
    pollingClient.start()
    yield
    if pollingClient:
        pollingClient.stop()

app = FastAPI(lifespan=lifespan)

app.mount("/static", StaticFiles(directory="./static", html=True), name="static")

@app.websocket("/ws")
async def websocket_endpoint(websocket: WebSocket):
    await manager.connect(websocket)
    try:
        while True:
            await websocket.receive_text()  # Keeps the connection alive
    except WebSocketDisconnect:
        manager.disconnect(websocket)

@app.get("/version")
async def read_root():
    return {"version": 0.01}

if __name__ == "__main__":
    uvicorn.run("main:app", host="0.0.0.0", port=8000, reload=True)