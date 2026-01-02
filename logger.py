import logging
import logging.config
import yaml
import os


def setup_logging(config_path='logging_config.yaml'):
    """
    Load logging configuration from a YAML file.
    If the file is not found, default to basic config.
    """
    # Ensure we are looking relative to the project root, or absolute path
    if os.path.exists(config_path):
        with open(config_path, 'rt') as f:
            try:
                config = yaml.safe_load(f.read())
                logging.config.dictConfig(config)
                print(f"Logging configuration loaded from {config_path}")
            except Exception as e:
                print(f"Error parsing logging configuration: {e}")
                logging.basicConfig(level=logging.INFO)
    else:
        print(f"Logging configuration file not found at {config_path}. Using default.")
        logging.basicConfig(level=logging.INFO)