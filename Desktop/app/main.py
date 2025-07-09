#set database
#set fastAPI websockets
#runs pollingClient 
from fastapi import FastAPI
from fastapi.staticfiles import StaticFiles

app = FastAPI()

app.mount("/", StaticFiles(directory="./static",html = True), name="static")

@app.get("/version")
async def read_root():
    return 0.01
