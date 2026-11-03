"""Launch the full Dum-E application with a symmetric two-finger gripper."""

import importlib.util
import sys
from pathlib import Path


def main() -> None:
    application_path = Path(__file__).with_name("Dum-E.py")
    spec = importlib.util.spec_from_file_location("dum_e_application", application_path)
    if spec is None or spec.loader is None:
        raise ImportError(f"Cannot load Dum-E application from {application_path}")

    application = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = application
    spec.loader.exec_module(application)
    config = application.RobotConfig(finger_count=2)
    application.main(config=config)


if __name__ == "__main__":
    main()
