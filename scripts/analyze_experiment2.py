#!/usr/bin/env python3

"""
Statistical analysis for Experiment 2.

Compares run-level safety-stop reaction latency between EasyNav and Nav2.

Input:
    results/latency/latency_easynav.csv
    results/latency/latency_nav2.csv

Output:
    results/latency/analysis/
        latency_distribution.png
        descriptive_statistics.csv
        statistical_tests.csv
        run_level_data.csv
"""

from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
from scipy import stats


DATA_DIR = Path("results/latency")
OUTPUT_DIR = DATA_DIR / "analysis"

EASYNAV_FILE = DATA_DIR / "latency_easynav.csv"
NAV2_FILE = DATA_DIR / "latency_nav2.csv"

LATENCY_COLUMN = "latency_us"
RUN_COLUMNS = [
    "framework",
    "latency_us",
    "dist_blocked",
    "stop_lin_eps",
    "stop_ang_eps",
]


def load_latency_file(path: Path, framework: str) -> pd.DataFrame:
    df = pd.read_csv(path)

    required_columns = {
        LATENCY_COLUMN,
        "dist_blocked",
        "stop_lin_eps",
        "stop_ang_eps",
    }
    missing = required_columns - set(df.columns)

    if missing:
        raise ValueError(
            f"{path} is missing required columns: {sorted(missing)}"
        )

    df = df.copy()
    df["framework"] = framework
    df[LATENCY_COLUMN] = pd.to_numeric(
        df[LATENCY_COLUMN], errors="coerce"
    )

    if df[LATENCY_COLUMN].isna().any():
        raise ValueError(f"Invalid latency values found in {path}.")

    if (df[LATENCY_COLUMN] <= 0).any():
        raise ValueError(f"Non-positive latency values found in {path}.")

    return df


def cohens_d(x: np.ndarray, y: np.ndarray) -> float:
    nx, ny = len(x), len(y)

    pooled_sd = np.sqrt(
        (
            (nx - 1) * np.var(x, ddof=1)
            + (ny - 1) * np.var(y, ddof=1)
        )
        / (nx + ny - 2)
    )

    if pooled_sd == 0:
        return np.nan

    return (np.mean(x) - np.mean(y)) / pooled_sd


def descriptive_statistics(df: pd.DataFrame) -> pd.DataFrame:
    rows = []

    for framework, group in df.groupby("framework"):
        values = group[LATENCY_COLUMN].to_numpy() / 1000.0

        rows.append(
            {
                "framework": framework,
                "n": len(values),
                "mean_ms": np.mean(values),
                "std_ms": np.std(values, ddof=1),
                "median_ms": np.median(values),
                "iqr_ms": np.percentile(values, 75)
                - np.percentile(values, 25),
                "min_ms": np.min(values),
                "max_ms": np.max(values),
            }
        )

    return pd.DataFrame(rows)


def create_distribution_plot(
    easynav: np.ndarray,
    nav2: np.ndarray,
    output_path: Path,
) -> None:
    groups = [easynav, nav2]
    labels = ["EasyNav", "Nav2"]

    fig, ax = plt.subplots(figsize=(7, 5))

    ax.boxplot(
        groups,
        tick_labels=labels,
        showmeans=True,
        meanprops={
            "marker": "D",
            "markerfacecolor": "black",
            "markeredgecolor": "black",
            "markersize": 5,
        },
    )

    rng = np.random.default_rng(42)

    for i, values in enumerate(groups, start=1):
        jitter = rng.uniform(-0.10, 0.10, size=len(values))

        ax.scatter(
            np.full(len(values), i) + jitter,
            values,
            alpha=0.70,
            edgecolors="white",
            linewidths=0.5,
            zorder=3,
        )

    ax.set_ylabel("Stop latency [ms]")
    ax.set_title("Safety-stop reaction latency")
    ax.grid(axis="y", alpha=0.25)

    fig.tight_layout()
    fig.savefig(output_path, dpi=300, bbox_inches="tight")
    plt.close(fig)


def main() -> None:
    print("Reading results from:", DATA_DIR)

    OUTPUT_DIR.mkdir(parents=True, exist_ok=True)

    easynav_df = load_latency_file(EASYNAV_FILE, "EasyNav")
    nav2_df = load_latency_file(NAV2_FILE, "Nav2")

    df = pd.concat(
        [easynav_df, nav2_df],
        ignore_index=True,
    )

    easynav = easynav_df[LATENCY_COLUMN].to_numpy() / 1000.0
    nav2 = nav2_df[LATENCY_COLUMN].to_numpy() / 1000.0

    print("\nEXPERIMENT 2: Safety-Stop Reaction Latency")
    print(f"\nRuns:\n  EasyNav: n = {len(easynav)}\n  Nav2:   n = {len(nav2)}")

    descriptive = descriptive_statistics(df)

    print("\nDescriptive statistics:")
    print(
        descriptive.to_string(
            index=False,
            float_format=lambda x: f"{x:.4f}",
        )
    )

    welch = stats.ttest_ind(
        easynav,
        nav2,
        equal_var=False,
    )
    ci = welch.confidence_interval(0.95)
    mean_difference = np.mean(easynav) - np.mean(nav2)

    results = pd.DataFrame(
        [
            {
                "metric": "stop_latency",
                "easynav_mean_ms": np.mean(easynav),
                "nav2_mean_ms": np.mean(nav2),
                "easynav_std_ms": np.std(easynav, ddof=1),
                "nav2_std_ms": np.std(nav2, ddof=1),
                "mean_difference_easynav_minus_nav2_ms": mean_difference,
                "mean_difference_ci95_low_ms": ci.low,
                "mean_difference_ci95_high_ms": ci.high,
                "welch_p": welch.pvalue,
                "cohens_d": cohens_d(easynav, nav2),
            }
        ]
    )

    print("\nStatistical analysis:")
    print(
        results.to_string(
            index=False,
            float_format=lambda x: f"{x:.6g}",
        )
    )

    run_level = df[RUN_COLUMNS].copy()
    run_level["latency_ms"] = run_level[LATENCY_COLUMN] / 1000.0

    run_level.to_csv(
        OUTPUT_DIR / "run_level_data.csv",
        index=False,
    )

    descriptive.to_csv(
        OUTPUT_DIR / "descriptive_statistics.csv",
        index=False,
    )

    results.to_csv(
        OUTPUT_DIR / "statistical_tests.csv",
        index=False,
    )

    create_distribution_plot(
        easynav,
        nav2,
        OUTPUT_DIR / "latency_distribution.png",
    )

    print("\nResults written to:")
    print(f"  {OUTPUT_DIR}")


if __name__ == "__main__":
    main()
