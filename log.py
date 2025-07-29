import logging

# ANSI color codes
RESET = "\033[0m"
BLUE = "\033[94m"
YELLOW = "\033[93m"
RED = "\033[91m"
GREEN = "\033[92m"
MAGENTA = "\033[95m"
BOLD = "\033[1m"


class LevelFormatter(logging.Formatter):
    FORMATS = {
        logging.DEBUG: MAGENTA + "%(name)s - DEBUG - %(message)s" + RESET,
        logging.INFO: BLUE + "%(name)s - INFO - %(message)s" + RESET,
        logging.WARNING: YELLOW + "%(name)s - WARNING - %(message)s" + RESET,
        logging.ERROR: RED + "%(name)s - ERROR - %(message)s" + RESET,
        logging.CRITICAL: RED + BOLD + "%(name)s - CRITICAL - %(message)s" + RESET,
    }

    def format(self, record):
        log_fmt = self.FORMATS.get(record.levelno, self._fmt)
        formatter = logging.Formatter(log_fmt)
        return formatter.format(record)


class LoggerFactory:
    def __init__(self, name: str, level: int = logging.INFO):
        self.name = name
        self.level = level
        self.logger = self._create_logger()

    def _create_logger(self):
        logger = logging.getLogger(self.name)
        logger.setLevel(self.level)
        logger.propagate = False

        if not logger.hasHandlers():
            handler = logging.StreamHandler()
            handler.setLevel(self.level)
            handler.setFormatter(LevelFormatter())
            logger.addHandler(handler)

        return logger

    def get_logger(self):
        return self.logger