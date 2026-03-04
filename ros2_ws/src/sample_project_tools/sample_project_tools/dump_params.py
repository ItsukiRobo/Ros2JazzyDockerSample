import argparse
import subprocess
from datetime import datetime

def run(cmd):
    return subprocess.check_output(cmd, text=True).strip()

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", required=True)
    args = ap.parse_args()

    now = datetime.now().isoformat(timespec="seconds")
    lines = [f"# params dump generated at: {now}", ""]

    try:
        nodes = run(["ros2", "node", "list"]).splitlines()
    except Exception as e:
        nodes = []
        lines.append(f"# failed to list nodes: {e}")

    for node in nodes:
        node = node.strip()
        if not node:
            continue
        lines.append(f"## {node}")
        try:
            yml = run(["ros2", "param", "dump", node])
            lines.append(yml)
        except Exception as e:
            lines.append(f"# failed to dump params for {node}: {e}")
        lines.append("")

    with open(args.out, "w", encoding="utf-8") as f:
        f.write("\n".join(lines))

if __name__ == "__main__":
    main()
