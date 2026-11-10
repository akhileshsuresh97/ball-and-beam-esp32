"""Run the simulation and save the figures used in the README.

    python simulate.py              # comparison plot + animated GIF into ../docs/
    python simulate.py --no-gif     # plot only (faster)
"""

import argparse
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib import animation
import numpy as np

from ball_beam import Hardware, ImprovedController, OriginalController, simulate

DOCS = Path(__file__).resolve().parent.parent / "docs"
BLUE, RED, GREY = "#1d4e89", "#c0392b", "#7f8c8d"


def comparison_plot(path: Path) -> None:
    orig = simulate(OriginalController(setpoint_mm=240), x0=0.10, duration=10)
    impr = simulate(ImprovedController(), x0=0.10, duration=10, disturbance=(6, 0.3))
    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(9, 6), sharex=True)
    ax1.axhline(24, color=GREY, ls="--", lw=1, label="Setpoint (beam centre)")
    ax1.plot(orig.t, orig.x * 100, color=RED, lw=1.5, label="Original sketch logic (gains from the slides)")
    ax1.plot(impr.t, impr.x * 100, color=BLUE, lw=1.8, label="Improved firmware logic")
    ax1.annotate("push 0.3 m/s", xy=(6, 24), xytext=(6.6, 38), color=BLUE,
                 arrowprops=dict(arrowstyle="->", color=BLUE))
    ax1.set_ylabel("Ball position (cm)")
    ax1.set_ylim(0, 48)
    ax1.legend(loc="upper right", fontsize=9)
    ax1.set_title("Simulated ball-and-beam response")
    ax2.plot(orig.t, orig.angle_deg, color=RED, lw=1.2)
    ax2.plot(impr.t, impr.angle_deg, color=BLUE, lw=1.5)
    ax2.set_ylabel("Beam angle (°)")
    ax2.set_xlabel("Time (s)")
    for ax in (ax1, ax2):
        ax.grid(alpha=0.3)
        ax.spines[["top", "right"]].set_visible(False)
    fig.tight_layout()
    fig.savefig(path, dpi=130)
    plt.close(fig)


def animate(path: Path, fps: int = 25, duration: float = 9.0) -> None:
    hw = Hardware()
    runs = [
        ("Original sketch logic", simulate(OriginalController(setpoint_mm=240), x0=0.10,
                                           duration=duration, hw=hw), RED),
        ("Improved firmware logic", simulate(ImprovedController(), x0=0.10, duration=duration,
                                             hw=hw, disturbance=(5, 0.3)), BLUE),
    ]
    fig, axes = plt.subplots(2, 1, figsize=(6.4, 4.2))
    artists = []
    for ax, (title, _, color) in zip(axes, runs):
        ax.set_xlim(-0.03, 0.51)
        ax.set_ylim(-0.09, 0.09)
        ax.set_aspect("equal")
        ax.axis("off")
        ax.set_title(title, color=color, fontsize=11)
        beam, = ax.plot([], [], color="#333", lw=4, solid_capstyle="round")
        ball = plt.Circle((0, 0), hw.ball_radius, color=color)
        ax.add_patch(ball)
        target, = ax.plot([], [], marker="v", color=GREY, ms=7)
        ax.plot([0.24], [-0.012], marker="^", color="#555", ms=12)  # pivot under the beam centre
        label = ax.text(0.5, -0.085, "", fontsize=8, color="#555", ha="right")
        sensor, = ax.plot([], [], marker="s", color="#555", ms=7)  # ToF sensor, mounted on the beam end
        artists.append((beam, ball, target, label, sensor))
    step = int(1 / (fps * 0.001))
    frames = range(0, len(runs[0][1].t), step)

    def draw(i):
        out = []
        for (_, r, _), (beam, ball, target, label, sensor) in zip(runs, artists):
            a = -np.radians(r.angle_deg[i])  # positive angle lowers the far end (ball rolls away from the sensor)
            c, s = np.cos(a), np.sin(a)
            beam.set_data([0.24 - 0.24 * c, 0.24 + 0.24 * c], [-0.24 * s, 0.24 * s])
            d = r.x[i] - 0.24
            ball.center = (0.24 + d * c - hw.ball_radius * s, d * s + hw.ball_radius * c + 0.004)
            target.set_data([0.24], [0.065])  # setpoint marker above the beam centre
            label.set_text(f"t = {r.t[i]:.1f} s   angle = {r.angle_deg[i]:+.1f}°")
            sensor.set_data([0.24 - 0.25 * c], [-0.25 * s])
            out += [beam, ball, target, label, sensor]
        return out

    anim = animation.FuncAnimation(fig, draw, frames=frames, blit=True)
    fig.tight_layout()
    anim.save(path, writer=animation.PillowWriter(fps=fps))
    plt.close(fig)


def main() -> None:
    p = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    p.add_argument("--no-gif", action="store_true", help="skip the animation")
    args = p.parse_args()
    DOCS.mkdir(exist_ok=True)
    comparison_plot(DOCS / "simulation_comparison.png")
    print("Saved", DOCS / "simulation_comparison.png")
    if not args.no_gif:
        animate(DOCS / "simulation.gif")
        print("Saved", DOCS / "simulation.gif")


if __name__ == "__main__":
    main()
