#!/usr/bin/env python3

import subprocess
import shutil

shutil.rmtree('zenqt/bin', ignore_errors=True)
shutil.rmtree('build', ignore_errors=True)