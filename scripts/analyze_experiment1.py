#!/usr/bin/env python3

"""Statistical and graphical analysis of Experiment 1."""

import re
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
from scipy import stats
from statsmodels.stats.multitest import multipletests


RESULTS_DIR = Path("results/cycle")
OUTPUT_DIR = RESULTS_DIR / "analysis"

FRAMEWORKS = ["EasyNav", "Nav2"]

METRICS = {
    "cpu": ("CPU utilization", "%", "mean", True),
    "memory": ("Memory consumption", "MB", "mean", True),
    "cmd_vel_frequency": (r"$cmd\_vel$ frequency", "Hz", "mean", False),
    "obstacle_distance": ("Obstacle distance", "m", "mean", False),
    "distance_travelled": ("Distance travelled", "m", "final", False),
}

REQUIRED_COLUMNS = {
    "time",
    "cpu",
    "memory",
    "obstacle_distance",
    "cmd_vel_frequency",
    "distance_travelled",
}


def load_run(path):
    df = pd.read_csv(path)

    missing = REQUIRED_COLUMNS - set(df.columns)
    if missing:
        raise ValueError(f"{path}: missing columns: {sorted(missing)}")

    name = path.name.lower()

    if "easynav" in name:
        framework = "EasyNav"
    elif "nav2" in name:
        framework = "Nav2"
    else:
        return None

    match = re.search(r"_(\d+)\.csv$", path.name)
    run = int(match.group(1)) if match else None

    result = {"framework": framework, "run": run, "file": path.name}

    for metric, (_, _, summary, _) in METRICS.items():
        values = pd.to_numeric(df[metric], errors="coerce").dropna()
        values = values[np.isfinite(values)]

        if values.empty:
            result[metric] = np.nan
        elif summary == "final":
            result[metric] = values.iloc[-1]
        else:
            result[metric] = values.mean()

    return result


def load_data():
    files = sorted(RESULTS_DIR.glob("*.csv"))

    if not files:
        raise FileNotFoundError(f"No CSV files found in {RESULTS_DIR}")

    runs = [load_run(path) for path in files]
    runs = [run for run in runs if run is not None]

    if not runs:
        raise RuntimeError("No EasyNav/Nav2 benchmark CSV files were found.")

    return (
        pd.DataFrame(runs)
        .sort_values(["framework", "run", "file"], na_position="last")
        .reset_index(drop=True)
    )


def get_values(data, framework, metric):
    values = data.loc[data["framework"] == framework, metric].dropna()
    values = values.to_numpy(dtype=float)
    return values[np.isfinite(values)]


def descriptive_statistics(data):
    rows = []

    for metric in METRICS:
        for framework in FRAMEWORKS:
            values = get_values(data, framework, metric)

            if len(values) == 0:
                continue

            q1, q3 = np.percentile(values, [25, 75])

            rows.append({
                "metric": metric,
                "framework": framework,
                "n": len(values),
                "mean": np.mean(values),
                "std": np.std(values, ddof=1) if len(values) > 1 else np.nan,
                "median": np.median(values),
                "q1": q1,
                "q3": q3,
                "iqr": q3 - q1,
                "min": np.min(values),
                "max": np.max(values),
            })

    return pd.DataFrame(rows)


def cohens_d(x, y):
    if len(x) < 2 or len(y) < 2:
        return np.nan

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


def statistical_analysis(data):
    rows = []

    for metric, (_, _, _, primary) in METRICS.items():
        x = get_values(data, "EasyNav", metric)
        y = get_values(data, "Nav2", metric)

        if len(x) < 2 or len(y) < 2:
            continue

        test = stats.ttest_ind(x, y, equal_var=False)
        ci = test.confidence_interval(0.95)

        rows.append({
            "metric": metric,
            "primary": primary,
            "easynav_n": len(x),
            "nav2_n": len(y),
            "mean_difference_easynav_minus_nav2": np.mean(x) - np.mean(y),
            "mean_difference_ci95_low": ci.low,
            "mean_difference_ci95_high": ci.high,
            "welch_p": test.pvalue,
            "cohens_d": cohens_d(x, y),
        })

    results = pd.DataFrame(rows)
    results["welch_p_holm"] = np.nan

    primary = results["primary"].astype(bool)
    p_values = results.loc[primary, "welch_p"].to_numpy()

    if len(p_values):
        _, adjusted, _, _ = multipletests(
            p_values,
            alpha=0.05,
            method="holm",
        )
        results.loc[primary, "welch_p_holm"] = adjusted

    return results


def load_series(data, metric):
    result = {}

    for framework in FRAMEWORKS:
        files = data.loc[
            data["framework"] == framework, "file"
        ].tolist()

        series = []

        for filename in files:
            df = pd.read_csv(RESULTS_DIR / filename)

            time = pd.to_numeric(df["time"], errors="coerce")
            values = pd.to_numeric(df[metric], errors="coerce")

            valid = (
                time.notna()
                & values.notna()
                & np.isfinite(time)
                & np.isfinite(values)
            )

            series.append(
                pd.Series(
                    values[valid].to_numpy(),
                    index=time[valid].to_numpy(),
                )
            )

        if not series:
            continue

        combined = pd.concat(series, axis=1).sort_index()
        combined = combined.interpolate(method="index").ffill().bfill()

        result[framework] = series, combined

    return result


def plot_distribution(data, metric):
    label, unit, _, _ = METRICS[metric]

    groups = [
        get_values(data, "EasyNav", metric),
        get_values(data, "Nav2", metric),
    ]

    fig, ax = plt.subplots(figsize=(7, 5))

    ax.boxplot(
        groups,
        tick_labels=FRAMEWORKS,
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

    ax.set_ylabel(f"{label} [{unit}]")
    ax.set_title(f"{label} — run-level observations")
    ax.grid(axis="y", alpha=0.25)

    fig.tight_layout()
    fig.savefig(
        OUTPUT_DIR / f"{metric}_distribution.png",
        dpi=300,
        bbox_inches="tight",
    )
    plt.close(fig)


def plot_profile(data, metric):
    label, unit, _, _ = METRICS[metric]

    fig, ax = plt.subplots(figsize=(9, 4))

    for framework, (_, combined) in load_series(data, metric).items():
        n = combined.shape[1]
        mean = combined.mean(axis=1)
        ci = (
            stats.t.ppf(0.975, n - 1)
            * combined.std(axis=1)
            / np.sqrt(n)
        )

        ax.plot(mean.index, mean, linewidth=2.0, label=framework)
        ax.fill_between(
            mean.index,
            mean - ci,
            mean + ci,
            alpha=0.20,
        )

    ax.set_xlabel("Time [s]")
    ax.set_ylabel(f"{label} [{unit}]")
    ax.set_title(f"{label} — mean ± 95% CI")
    ax.grid(alpha=0.25)
    ax.legend()

    fig.tight_layout()
    fig.savefig(
        OUTPUT_DIR / f"{metric}_profile.png",
        dpi=300,
        bbox_inches="tight",
    )
    plt.close(fig)


def plot_timeseries(data, metric):
    label, unit, _, _ = METRICS[metric]

    fig, ax = plt.subplots(figsize=(9, 5))

    for framework, (series, combined) in load_series(data, metric).items():
        for run in series:
            ax.plot(
                run.index,
                run.values,
                alpha=0.15,
                linewidth=0.8,
            )

        mean = combined.mean(axis=1)
        std = combined.std(axis=1)

        ax.plot(mean.index, mean, linewidth=2.0, label=framework)
        ax.fill_between(
            mean.index,
            mean - std,
            mean + std,
            alpha=0.20,
        )

    ax.set_xlabel("Time [s]")
    ax.set_ylabel(f"{label} [{unit}]")
    ax.set_title(f"{label} — mean ± 1 std across runs")
    ax.grid(alpha=0.25)
    ax.legend()

    fig.tight_layout()
    fig.savefig(
        OUTPUT_DIR / f"{metric}_timeseries.png",
        dpi=300,
        bbox_inches="tight",
    )
    plt.close(fig)


def print_summary(data, descriptive, tests):
    print("\nEXPERIMENT 1")

    print("\nRuns:")
    for framework in FRAMEWORKS:
        n = (data["framework"] == framework).sum()
        print(f"  {framework}: n = {n}")

    print("\nDescriptive statistics:")
    print(
        descriptive[
            ["metric", "framework", "n", "mean", "std", "median", "iqr"]
        ].to_string(index=False)
    )

    print("\nStatistical analysis:")
    print(
        tests[
            [
                "metric",
                "welch_p",
                "welch_p_holm",
                "cohens_d",
                "mean_difference_easynav_minus_nav2",
                "mean_difference_ci95_low",
                "mean_difference_ci95_high",
            ]
        ].to_string(index=False)
    )

    print(f"\nResults written to: {OUTPUT_DIR}")


def main():
    if not RESULTS_DIR.exists():
        raise FileNotFoundError(
            f"Results directory does not exist: {RESULTS_DIR}"
        )

    print(f"Reading results from: {RESULTS_DIR}")
    OUTPUT_DIR.mkdir(parents=True, exist_ok=True)

    data = load_data()
    data.to_csv(OUTPUT_DIR / "run_level_data.csv", index=False)

    descriptive = descriptive_statistics(data)
    descriptive.to_csv(
        OUTPUT_DIR / "descriptive_statistics.csv",
        index=False,
    )

    tests = statistical_analysis(data)
    tests.to_csv(
        OUTPUT_DIR / "statistical_tests.csv",
        index=False,
    )

    for metric in METRICS:
        plot_distribution(data, metric)
        plot_timeseries(data, metric)

        if METRICS[metric][3]:
            plot_profile(data, metric)

    print_summary(data, descriptive, tests)


if __name__ == "__main__":
    main()
