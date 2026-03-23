#!/usr/bin/env bash
set -euo pipefail

if [ "${EUID:-$(id -u)}" -ne 0 ]; then
  SUDO=sudo
else
  SUDO=
fi

$SUDO apt-get update
$SUDO apt-get install -y   git   build-essential   cmake   python3-pip   pkg-config

/usr/bin/python3 -m pip install --user --upgrade pip conan

echo "Jetson build prerequisites installed."
