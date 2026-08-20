#!/usr/bin/env python3

"""Statistical analysis of Experiment 2."""

from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
from scipy import stats


DATA_DIR = Path("results/latency")
OUTPUT_DIR = DATA_DIR / "analysis"

FILES = {
    "EasyNav": DATA_DIR / "latency_easynav.csv",
    "Nav2": DATA_DIR / "latency_nav2.csv",
}

LATENCY_COLUMN = "latency_us"


def load_data(path, framework):
    df = pd.read_csv(path)

    required = {
        LATENCY_COLUMN,
        "dist_blocked",
        "stop_lin_eps",
        "stop_ang_eps",
    }
    missing = required - set(df.columns)

    if missing:
        raise ValueError(f"{path}: missing columns: {sorted(missing)}")

    df = df.copy()
    df["framework"] = framework
    df[LATENCY_COLUMN] = pd.to_numeric(
        df[LATENCY_COLUMN],
        errors="coerce",
    )

    if df[LATENCY_COLUMN].isna().any():
        raise ValueError(f"Invalid latency values found in {path}.")

    if (df[LATENCY_COLUMN] <= 0).any():
        raise ValueError(f"Non-positive latency values found in {path}.")

    return df


def cohens_d(x, y):
    pooled_sd = np.sqrt(
        (
            (len(x) - 1) * np.var(x, ddof=1)
            + (len(y) - 1) * np.var(y, ddof=1)
        )
        / (len(x) + len(y) - 2)
    )

    if pooled_sd == 0:
        return np.nan

    return (np.mean(x) - np.mean(y)) / pooled_sd


def descriptive_statistics(data):
    rows = []

    for framework, group in data.groupby("framework"):
        values = group[LATENCY_COLUMN].to_numpy() / 1000.0

        rows.append({
            "framework": framework,
            "n": len(values),
            "mean_ms": np.mean(values),
            "std_ms": np.std(values, ddof=1),
            "median_ms": np.median(values),
            "iqr_ms": np.percentile(values, 75) - np.percentile(values, 25),
            "min_ms": np.min(values),
            "max_ms": np.max(values),
        })

    return pd.DataFrame(rows)


def plot_distribution(easynav, nav2):
    groups = [easynav, nav2]

    fig, ax = plt.subplots(figsize=(7, 5))

    ax.boxplot(
        groups,
        tick_labels=["EasyNav", "Nav2"],
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
        jitter = rng.uniform(-0.10, 0.10, len(values))
        ax.scatter(
            np.full(len(values), i) + jitter,
            values,
            alpha=0.70,
            edgecolors="white",
            linewidths=0.5,
            zorder=3,
        )

    ax.set_ylabel("Navigation reaction latency [ms]")
    ax.set_title("Navigation reaction latency")
    ax.grid(axis="y", alpha=0.25)

    fig.tight_layout()
    fig.savefig(
        OUTPUT_DIR / "latency_distribution.png",
        dpi=300,
        bbox_inches="tight",
    )
    plt.close(fig)


def main():
    print(f"Reading results from: {DATA_DIR}")

    OUTPUT_DIR.mkdir(parents=True, exist_ok=True)

    data = pd.concat(
        [
            load_data(path, framework)
            for framework, path in FILES.items()
        ],
        ignore_index=True,
    )

    easynav = data.loc[
        data["framework"] == "EasyNav",
        LATENCY_COLUMN,
    ].to_numpy() / 1000.0

    nav2 = data.loc[
        data["framework"] == "Nav2",
        LATENCY_COLUMN,
    ].to_numpy() / 1000.0

    print("\nEXPERIMENT 2: Navigation Reaction Latency")
    print(
        f"\nRuns:\n"
        f"  EasyNav: n = {len(easynav)}\n"
        f"  Nav2:    n = {len(nav2)}"
    )

    descriptive = descriptive_statistics(data)

    print("\nDescriptive statistics:")
    print(
        descriptive.to_string(
            index=False,
            float_format=lambda x: f"{x:.4f}",
        )
    )

    test = stats.ttest_ind(
        easynav,
        nav2,
        equal_var=False,
    )
    ci = test.confidence_interval(0.95)

    results = pd.DataFrame([{
        "metric": "navigation_reaction_latency",
        "easynav_mean_ms": np.mean(easynav),
        "nav2_mean_ms": np.mean(nav2),
        "easynav_std_ms": np.std(easynav, ddof=1),
        "nav2_std_ms": np.std(nav2, ddof=1),
        "mean_difference_easynav_minus_nav2_ms": (
            np.mean(easynav) - np.mean(nav2)
        ),
        "mean_difference_ci95_low_ms": ci.low,
        "mean_difference_ci95_high_ms": ci.high,
        "welch_p": test.pvalue,
        "cohens_d": cohens_d(easynav, nav2),
    }])

    print("\nStatistical analysis:")
    print(
        results.to_string(
            index=False,
            float_format=lambda x: f"{x:.6g}",
        )
    )

    run_level = data[
        [
            "framework",
            LATENCY_COLUMN,
            "dist_blocked",
            "stop_lin_eps",
            "stop_ang_eps",
        ]
    ].copy()
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

    plot_distribution(easynav, nav2)

    print(f"\nResults written to: {OUTPUT_DIR}")


if __name__ == "__main__":
    main()
