from abc import ABC, abstractmethod


class LogManager(ABC):
    def __init__(self, *args, **kwargs):
        self.groups = {}
        pass

    @abstractmethod
    def add_log_group(self, name, *args, **kwargs):
        pass

    @abstractmethod
    def add_log_entry(self, group_name, entry, *args, **kwargs):
        pass

    @abstractmethod
    def start(self, *args, **kwargs):
        pass

    @abstractmethod
    def stop(self, *args, **kwargs):
        pass
