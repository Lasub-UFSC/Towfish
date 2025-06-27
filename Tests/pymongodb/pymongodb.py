import pymongo
import os
from datetime import datetime
from pymongo import MongoClient

MONGO_CONN_STRING = os.environ.get("MONGODB_CONNECTION_STRING")

client = MongoClient(MONGO_CONN_STRING)

# Let's create a new database called "stock data"
db = client.towfishdb

# Let's create a new time-series collection in the "stock data" database called "stocks"

collection = db.create_collection('test0', timeseries={
          "timeField": "timestamp",
})

data = [
{
   "metadata": {
      "stockSymbol": "ABC",
      "exchange": "NASDAQ"
   },
   "timestamp": datetime(2023, 9, 12, 15, 19, 48),
   "open": 54.80,
   "high": 59.20,
   "low": 52.60,
   "close": 53.50,
   "volume": 18000
},
{
   "metadata": {
      "stockSymbol": "ABC",
      "exchange": "NASDAQ"
   },
   "timestamp": datetime(2023, 9, 12, 16, 19, 48),
   "open": 51.00,
   "high": 54.30,
   "low": 50.50,
   "close": 51.80,
   "volume": 12000
},
{
   "metadata": {
      "stockSymbol": "ABC",
      "exchange": "NASDAQ"
   },
   "timestamp":datetime(2023, 9, 12, 17, 19, 48),
   "open": 52.00,
   "high": 53.10,
   "low": 50.50,
   "close": 52.90,
   "volume": 10000
},
{
   "metadata": {
      "stockSymbol": "ABC",
      "exchange": "NASDAQ"
   },
   "timestamp":datetime(2023, 9, 12, 18, 19, 48),
   "open": 52.80,
   "high": 60.20,
   "low": 52.60,
   "close": 55.50,
   "volume": 30000
}
]
# insert the data into our collection
collection.insert_many(data)