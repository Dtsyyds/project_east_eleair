#!/usr/bin/env python3
"""Compatibility command; implementation lives in inspection_runtime.data.vehicle_collection_manager."""
import runpy
if __name__ == "__main__":
    runpy.run_module('inspection_runtime.data.vehicle_collection_manager', run_name="__main__")
