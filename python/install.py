#!/usr/bin/env python3

import argparse
import subprocess
from pathlib import Path

PROJECT_DIR = Path(__file__).resolve().parent
PYTHON = PROJECT_DIR / ".venv" / "bin" / "python"
SYSTEMD_DIR = Path("/etc/systemd/system")

SCRIPTS = [
    "gpsreader.py",
    "sonar.py",
    "tempreader.py",
]


def run(*args):
    print("$", " ".join(map(str, args)))
    subprocess.run(args, check=True)


def service_name(script):
    return f"{PROJECT_DIR.name}-{Path(script).stem}.service"


def make_unit(script):
    script = PROJECT_DIR / script

    return f"""\
[Unit]
Description={script.name}
After=network.target

[Service]
Type=simple
WorkingDirectory={PROJECT_DIR}
ExecStart={PYTHON} {script}
Restart=on-failure
RestartSec=5
Environment=PYTHONUNBUFFERED=1

[Install]
WantedBy=multi-user.target
"""


def install():
    if not PYTHON.exists():
        raise SystemExit(f"Virtualenv Python not found: {PYTHON}")

    for name in SCRIPTS:
        script = PROJECT_DIR / name

        if not script.exists():
            raise SystemExit(f"Script not found: {script}")

        service = service_name(name)
        unit = SYSTEMD_DIR / service

        print(f"Installing {service}")
        unit.write_text(make_unit(name))

    run("systemctl", "daemon-reload")

    for name in SCRIPTS:
        service = service_name(name)

        run("systemctl", "enable", service)
        # run("systemctl", "restart", service)

    print("\nDone.")


def uninstall():
    for name in SCRIPTS:
        service = service_name(name)
        unit = SYSTEMD_DIR / service

        print(f"Removing {service}")

        # The service may not exist, so don't fail if it doesn't.
        subprocess.run(
            ["systemctl", "disable", "--now", service],
            check=False,
        )

        if unit.exists():
            unit.unlink()

    run("systemctl", "daemon-reload")

    print("\nDone.")


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--uninstall",
        action="store_true",
        help="Stop, disable, and remove the systemd services",
    )
    args = parser.parse_args()

    if args.uninstall:
        uninstall()
    else:
        install()


if __name__ == "__main__":
    main()
