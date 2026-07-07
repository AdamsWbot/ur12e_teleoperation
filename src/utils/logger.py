import logging


def setup_logger(name: str, level: int = logging.INFO) -> logging.Logger:
    """Create or return a logger using the project-wide format."""
    logger = logging.getLogger(name)
    logger.setLevel(level)
    logger.propagate = False

    if not logger.handlers:
        handler = logging.StreamHandler()
        handler.setFormatter(logging.Formatter(
            "[%(asctime)s] [%(name)s] [%(levelname)s] %(message)s"
        ))
        logger.addHandler(handler)

    for handler in logger.handlers:
        handler.setLevel(level)

    return logger
