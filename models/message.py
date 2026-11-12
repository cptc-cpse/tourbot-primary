import json
import time
from typing import Any


def _json_default(value: object) -> Any:
    to_dict = getattr(value, "to_dict", None)
    if callable(to_dict):
        return to_dict()
    raise TypeError(f"Object of type {type(value).__name__} is not JSON serializable")


class Message:
    def __init__(self, payload: Any, timestamp: float | None = None):
        self.payload = payload
        self.timestamp = time.time() if timestamp is None else timestamp

    def to_json(self) -> str:
        """Convert the message to a JSON string."""
        return json.dumps(
            {
                "payload": self.payload,
                "timestamp": self.timestamp,
            },
            default=_json_default,
        )

    @classmethod
    def from_json(cls, serialized: str) -> "Message":
        """Create a message from its JSON representation."""
        data = json.loads(serialized)
        if not isinstance(data, dict) or "payload" not in data:
            raise ValueError("Message JSON must be an object containing a payload")

        timestamp = data.get("timestamp")
        if isinstance(timestamp, bool) or not isinstance(timestamp, (int, float)):
            raise ValueError("Message JSON must contain a numeric timestamp")

        return cls(data["payload"], float(timestamp))