#!/bin/bash
pycodestyle --exclude "install,build,setup.py,venv" --max-line-length=150 .
