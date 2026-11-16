"""Validate the frozen braking candidate on unseen simulated plants."""
import argparse
import json
from pathlib import Path

from screen_timed_braking import screen

CANDIDATE = (2.3, 32., .040, 1.15, .005, 1, 40., .080, 3.55, 2.0, 205., .45)
# Coupling, sensor delay and motor transport delay (each delay in 0.1 ms).
UNSEEN = ((.889, 89, 72), (.979, 108, 78), (1.096, 93, 82),
          (1.214, 114, 85), (1.281, 118, 90))
STRESS = ((1.17, 150, 150),)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, default=Path("timed_braking_reproduced.json"))
    args = parser.parse_args()
    report = {
        "candidate": CANDIDATE,
        "unseen_cases": UNSEEN,
        "baseline": screen(None, UNSEEN),
        "validation": screen(CANDIDATE, UNSEEN),
        "stress": screen(CANDIDATE, STRESS),
    }
    for row in report["validation"]:
        row["meets_target"] = (row["audited_success"]
            and 0 <= row["qualified_balance_start_s"] < 10
            and row["measured_hold_s"] >= 30 and row["pulse_increment"] == 0 and row["final_mode"] == 5)
    report["successes"] = sum(row["meets_target"] for row in report["validation"])
    path = args.output
    path.write_text(json.dumps(report, indent=2) + "\n")
    print(f'{report["successes"]}/{len(UNSEEN)}: {path}')


if __name__ == "__main__":
    main()
