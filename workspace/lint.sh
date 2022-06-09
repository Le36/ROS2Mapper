#!/bin/bash
pycodestyle --exclude "install,build,setup.py" --max-line-length=150 .
