from pydantic import BaseSettings

class Settings(BaseSettings):
    port: str = "COM5"

settings = Settings()
