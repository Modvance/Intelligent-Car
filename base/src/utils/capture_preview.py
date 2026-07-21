#!/usr/bin/env python3


def should_start_monitor(mode, monitor_requested, auto_capture, capture_preview_enabled):
    if monitor_requested:
        return True
    return mode == "manual" and bool(auto_capture) and bool(capture_preview_enabled)


def should_overlay_results(mode, preview_overlay_requested=False):
    if preview_overlay_requested:
        return True
    return mode != "manual"
