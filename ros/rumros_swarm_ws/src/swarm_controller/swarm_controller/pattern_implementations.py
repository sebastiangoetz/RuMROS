from typing import Callable, Dict
from abc import ABC
import threading

"""
This file defines helper classes, which can be used throughout the application
wherever design patterns are required. The actual classes are kept clean this way.
"""

class PushObserver(ABC):
    """
    This class implements the GOF observer pattern, extended by channels. Call the notify method to push data to
    registered observers on the specified channel. See https://refactoring.guru/design-patterns/observer for more
    information.
    """
    class Channel:
        def __init__(self):
            self.observers: Dict[int, Callable] = dict()
            self._observer_counter = 0
            self._observers_lock = threading.Lock()

        def register(self, observer_callback: Callable) -> int:
            """
            Registers the observer with this channel.

            :param observer_callback: The callback function of the observer.
            :return: The assigned ID of the registered observer. Used for unregistering the observer.
            """
            with self._observers_lock:
                self.observers[self._observer_counter] = observer_callback
                self._observer_counter += 1
                return self._observer_counter - 1

        def unregister(self, observer_id: int) -> bool:
            """
            Unregisters the observer from this channel.

            :param observer_id: The assigned ID of the observer.
            :return: ``True`` if the observer was registered and is now unregistered, ``False`` otherwise.
            """
            with self._observers_lock:
                if observer_id in self.observers:
                    del self.observers[observer_id]
                    return True
                return False

        def unregister_all(self):
            """Unregisters all registered observers."""
            with self._observers_lock:
                for k in self.observers.keys():
                    del self.observers[k]

        def notify(self, *args, **kwargs):
            """
            Notifies observers on this channel. Registered observers are notified by calling their callback functions
            with the given arguments.

            :param args: The arguments for the callback functions.
            :param kwargs: The keyword arguments for the callback functions.
            """

            # This way, an input callback that was removed just after this line can still be called
            # but in exchange we don't have to hold the lock during execution of all callbacks
            # which might take a significant amount of time.
            with self._observers_lock:
                observer_callbacks = self.observers.values()

            for callback in observer_callbacks:
                callback(*args, **kwargs)

    def __init__(self):
        """Initializes the variables that are necessary for the inheriting class to implement the observer pattern."""
        super().__init__()

        # channels[0] is always the default channel
        self._channel_default = "__DEFAULT__"
        self.channels: Dict[str, PushObserver.Channel] = {self._channel_default: PushObserver.Channel()}
        self._channel_lock = threading.Lock()

    def add_channel(self, name: str):
        """
        Adds a new notification channel to the Observer.

        :param name: The name of the channel, which is used as an identifier. Must be unique for the object.
        """
        with self._channel_lock:
            self.channels[name] = PushObserver.Channel()

    def remove_channel(self, name: str) -> bool:
        """
        Removes a notification channel from the Observer.

        :param name: The unique identifier string (name) of the channel.
        :return: ``True`` if the channel existed and was removed, ``False`` otherwise.
        """
        with self._channel_lock:
            if name in self.channels:
                del self.channels[name]
                return True
            return False

    def register(self, observer_callback: Callable | None, channel: str | None = None) -> int:
        """
        Registers the observer with this object.

        :param observer_callback: The callback function of the observer, which is called by the notify method.
        :param channel: The channel to register the observer with (default channel if not supplied).
        :return: The assigned ID of the registered observer (used for unregistering the observer)
                 or ``-1`` if the channel does not exist.
        """
        if not channel:
            channel = self._channel_default

        if not observer_callback or channel not in self.channels:
            return -1

        with self._channel_lock:
            return self.channels[channel].register(observer_callback)

    def unregister(self, observer_id: int, channel: str | None = None) -> bool:
        """
        Unregisters the observer from this object.

        :param observer_id: The assigned ID of the observer.
        :param channel: The channel to unregister the observer from (default channel if not supplied).
        :return: ``True`` if the observer was registered and is now unregistered, ``False`` otherwise.
        """
        if not channel:
            channel = self._channel_default

        with self._channel_lock:
            return self.channels[channel].unregister(observer_id)

    def notify(self, *args, **kwargs):
        """
        Notifies observers on the channel with the name supplied by the 'channel' keyword argument or the default
        channel if not supplied. Registered observers are notified by calling their callback functions with the given
        arguments.

        :param args: The arguments for the callback functions.
        :param kwargs: The keyword arguments for the callback functions.
        """
        # Use default channel if channel argument is not supplied
        channel = self._channel_default
        if "channel" in kwargs:
            channel = kwargs["channel"]

        with self._channel_lock:
            self.channels[channel].notify(*args, **kwargs)

    def notify_all(self, *args, **kwargs):
        """
        Notifies registered observers on all channels by calling their registered callback functions with the given
        arguments.

        :param args: The arguments for the callback functions.
        :param kwargs: The keyword arguments for the callback functions.
        """
        callbacks = []
        with self._channel_lock:
            for channel in self.channels:
                callbacks.extend(self.channels[channel].observers.values())

        # This way, an input callback that was removed just after this line can still be called
        # but in exchange we don't have to hold the lock during execution of all callbacks
        # which might take a significant amount of time.
        for callback in callbacks:
            callback(*args, **kwargs)