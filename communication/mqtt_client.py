import json
import logging
from collections.abc import Callable
from threading import Lock

import paho.mqtt.client as mqtt
from paho.mqtt.enums import CallbackAPIVersion
from paho.mqtt.properties import Properties
from paho.mqtt.reasoncodes import ReasonCode

from models.message import Message

logger: logging.Logger = logging.getLogger(__name__)
BROKER = "localhost"
PORT = 1883


class Client:
    """Maintain an MQTT connection for publishing and topic subscriptions."""

    def __init__(self, qos: int, retain: bool, process_id: str):
        self.BROKER: str = BROKER
        self.PORT: int = PORT
        self.QOS: int = qos
        self.RETAIN: bool = retain
        self.LAST_WILL_MESSAGE: str = f"{process_id} has disconnected"
        self.LAST_WILL_TOPIC: str = f"tourbot/v1/{process_id}/status"
        self._callbacks: dict[str, Callable[[Message], None]] = {}
        self._callbacks_lock = Lock()
        self._closing = False
        self._client = mqtt.Client(
            callback_api_version=CallbackAPIVersion.VERSION2
        )
        self._client.reconnect_delay_set(min_delay=1, max_delay=60)
        self._client.will_set(
            self.LAST_WILL_TOPIC,
            self.LAST_WILL_MESSAGE,
            qos=self.QOS,
            retain=self.RETAIN,
        )
        self._client.on_message = self._on_message
        self._client.on_connect = self._on_connect
        self._client.on_disconnect = self._on_disconnect

        result = self._client.connect(self.BROKER, self.PORT, 60)
        if result != mqtt.MQTT_ERR_SUCCESS:
            raise RuntimeError(
                f"Error connecting to MQTT broker: {mqtt.error_string(result)}"
            )

        result = self._client.loop_start()
        if result != mqtt.MQTT_ERR_SUCCESS:
            raise RuntimeError(
                f"Error starting MQTT network loop: {mqtt.error_string(result)}"
            )

    def publish(self, topic: str, payload: str) -> None:
        """Queue a payload for asynchronous publishing on the MQTT connection."""
        result = self._client.publish(topic, payload, qos=self.QOS, retain=self.RETAIN)
        if result.rc != mqtt.MQTT_ERR_SUCCESS:
            raise RuntimeError(
                f"Error publishing MQTT message: {mqtt.error_string(result.rc)}"
            )

    def subscribe(self, topic: str, callback: Callable[[Message], None]) -> None:
        """Subscribe to a topic and pass each received message to ``callback``."""
        with self._callbacks_lock:
            previous_callback = self._callbacks.get(topic)
            self._callbacks[topic] = callback

        if not self._client.is_connected():
            return

        result, _ = self._client.subscribe(topic, qos=self.QOS)
        if result != mqtt.MQTT_ERR_SUCCESS:
            with self._callbacks_lock:
                if previous_callback is None:
                    self._callbacks.pop(topic, None)
                else:
                    self._callbacks[topic] = previous_callback
            raise RuntimeError(
                f"Error subscribing to MQTT topic {topic!r}: "
                f"{mqtt.error_string(result)}"
            )

    def _on_connect(
        self,
        client: mqtt.Client,
        _userdata: object,
        _flags: mqtt.ConnectFlags,
        reason_code: ReasonCode,
        _properties: Properties | None,
    ) -> None:
        if reason_code.is_failure:
            logger.error("MQTT connection failed: %s", reason_code)
            return

        with self._callbacks_lock:
            topics = tuple(self._callbacks)
        for topic in topics:
            result, _ = client.subscribe(topic, qos=self.QOS)
            if result != mqtt.MQTT_ERR_SUCCESS:
                logger.error(
                    "Error subscribing to MQTT topic %r: %s",
                    topic,
                    mqtt.error_string(result),
                )

    def _on_message(
        self,
        _client: mqtt.Client,
        _userdata: object,
        message: mqtt.MQTTMessage,
    ) -> None:
        try:
            received = Message.from_json(bytes(message.payload).decode("utf-8"))
        except (UnicodeDecodeError, json.JSONDecodeError, ValueError) as error:
            logger.error(
                "Ignoring invalid MQTT message on topic %r: %s",
                message.topic,
                error,
            )
            return

        with self._callbacks_lock:
            callbacks = tuple(self._callbacks.items())
        for topic, callback in callbacks:
            if mqtt.topic_matches_sub(topic, message.topic):
                try:
                    callback(received)
                except Exception:
                    logger.exception("Error in MQTT callback for topic %r", message.topic)

    def _on_disconnect(
        self,
        _client: mqtt.Client,
        _userdata: object,
        _disconnect_flags: mqtt.DisconnectFlags,
        reason_code: ReasonCode,
        _properties: Properties | None,
    ) -> None:
        if not self._closing:
            logger.warning("MQTT connection lost: %s; reconnecting automatically", reason_code)

    def close(self) -> None:
        """Disconnect and stop the MQTT network loop."""
        self._closing = True
        self._client.disconnect()
        self._client.loop_stop()

    def __enter__(self) -> "Client":
        return self

    def __exit__(
        self,
        _exc_type: object,
        _exc_value: object,
        _traceback: object,
    ) -> None:
        self.close()