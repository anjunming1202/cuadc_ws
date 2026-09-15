# Offline wheels for Raspberry Pi (aarch64, Python 3.12)

Downloaded on a machine with internet for Pi installs that only have
Ethernet to the ground PC (no public DNS/apt/pip).

*.whl files are gitignored — regenerate on a PC with internet:

```bash
pip download pymavlink future pyserial -d . --python-version 312 --platform manylinux2014_aarch64 --only-binary=:all:
```

On the Pi (after scp of this folder including the wheels):

```bash
python3 -m venv ~/dronelink-venv
~/dronelink-venv/bin/pip install --no-index --find-links=. pymavlink pyserial
```
