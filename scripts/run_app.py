#!/usr/bin/env python3
"""
Local HTTP server launcher for the Moonwalker Fly web application.
"""

import os
import sys
import socket
import argparse
import functools
from http.server import HTTPServer, SimpleHTTPRequestHandler

WEB_DIR = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "web"))


def find_available_port(start_port=8000, max_port=8050):
    """Finds the first open port starting from start_port."""
    for port in range(start_port, max_port):
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
            if s.connect_ex(("127.0.0.1", port)) != 0:
                return port
    return start_port


def run_server(port=None):
    if not os.path.exists(WEB_DIR):
        raise FileNotFoundError(f"Web directory not found at {WEB_DIR}")

    if port is None:
        port = find_available_port(8000)

    handler = functools.partial(SimpleHTTPRequestHandler, directory=WEB_DIR)
    httpd = HTTPServer(("127.0.0.1", port), handler)

    local_url = f"http://localhost:{port}"
    print(f"Moonwalker Fly server running at: {local_url}")
    print(f"Serving web directory:          {WEB_DIR}")
    print("Press Ctrl+C to stop the server.\n")

    try:
        httpd.serve_forever()
    except KeyboardInterrupt:
        print("\nStopping Moonwalker Fly server ...")
        httpd.server_close()
        print("Server stopped cleanly.")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Run Moonwalker Fly local prototype server")
    parser.add_argument("--port", type=int, default=None, help="Port to listen on (default: auto-find starting at 8000)")
    args = parser.parse_args()
    run_server(args.port)
