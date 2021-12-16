#!/bin/bash
JLinkGDBServer -select USB -device Cortex-M0 -endian little -if JTAG-speed auto \
  -LocalhostOnly
