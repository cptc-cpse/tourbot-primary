import json
import logging
import time
from typing import Any

logger = logging.getLogger(__name__)


def _json_default(value: object) -> Any:
    to_dict = getattr(value, "to_dict", None)
    if callable(to_dict):
        return to_dict()
    error = f"Object of type {type(value).__name__} is not JSON serializable"
    logger.error(error)
    raise TypeError(error)


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
            error = "Message JSON must be an object containing a payload"
            logger.error(error)
            raise ValueError(error)

        timestamp = data.get("timestamp")
        if isinstance(timestamp, bool) or not isinstance(timestamp, (int, float)):
            error = "Message JSON must contain a numeric timestamp"
            logger.error(error)
            raise ValueError(error)

        return cls(data["payload"], float(timestamp))

    @classmethod
    def to_string(cls, message: "Message") -> str:
        """Return a string representation of the message."""
        return f"Message(payload={message.payload}, timestamp={message.timestamp})"