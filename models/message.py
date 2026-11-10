import json
import time
from typing import Any

class Message:
    def __init__(self, payload: Any):
        self.payload = payload
        self.timestamp = time.time()

    def to_json(self) -> str:
        """Convert the message to a JSON string."""
        return json.dumps({
            "payload": self.payload,
            "timestamp": self.timestamp
        })