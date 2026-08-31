"""Small CARLA RPC health probe for resumable evaluation batches."""

import argparse

import carla


def probe_carla_world(host, port, timeout_s):
    """Return whether CARLA can answer an RPC request, with a diagnostic reason."""
    try:
        client = carla.Client(host, int(port))
        client.set_timeout(float(timeout_s))
        client.get_world()
    except RuntimeError as error:
        return False, str(error)
    return True, ""


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=2000)
    parser.add_argument("--timeout", type=float, default=2.0)
    args = parser.parse_args()
    healthy, reason = probe_carla_world(args.host, args.port, args.timeout)
    if healthy:
        return 0
    print(reason)
    return 1


if __name__ == "__main__":
    raise SystemExit(main())
