import csv
import matplotlib.pyplot as plt
import numpy as np

CSV_PATH = 'barrel_measurements.csv'

# Optional: Set your camera resolution (e.g., 640x480) to filter out
# detections clipped at the edges of the frame. Set to None to disable.
IMAGE_WIDTH = 640
EDGE_MARGIN_PX = 10

distances = []
widths = []
heights = []

with open(CSV_PATH, mode='r', newline='') as f:
    reader = csv.DictReader(f)
    for row in reader:
        cx = float(row['bbox_center_x_px'])
        w = float(row['bbox_width_px'])
        h = float(row['bbox_height_px'])
        d = float(row['surface_distance_m'])

        # Skip detections touching the left/right image border (partially out of view)
        if IMAGE_WIDTH is not None:
            left_edge = cx - w / 2.0
            right_edge = cx + w / 2.0
            if left_edge < EDGE_MARGIN_PX or right_edge > (IMAGE_WIDTH - EDGE_MARGIN_PX):
                continue

        distances.append(d)
        widths.append(w)
        heights.append(h)

distances = np.array(distances)
widths = np.array(widths)
heights = np.array(heights)

if len(distances) == 0:
    raise RuntimeError('No valid rows found in CSV (check path or edge filter settings).')

# Fit inverse pinhole model: distance = a * (1 / size_px) + b
a_w, b_w = np.polyfit(1.0 / widths, distances, 1)
a_h, b_h = np.polyfit(1.0 / heights, distances, 1)

print(f'Loaded {len(distances)} valid detections.')
print(f'Width model:  distance = {a_w:.2f} / width_px  + {b_w:.4f}')
print(f'Height model: distance = {a_h:.2f} / height_px + {b_h:.4f}')

# Generate smooth curves for plotting
w_curve = np.linspace(widths.min(), widths.max(), 200)
h_curve = np.linspace(heights.min(), heights.max(), 200)

fig, axes = plt.subplots(1, 2, figsize=(12, 5), sharey=True)

# Plot 1: Bounding Box Width vs Surface Distance
axes[0].scatter(widths, distances, alpha=0.6, label='Measured data')
axes[0].plot(
    w_curve,
    a_w / w_curve + b_w,
    color='red',
    linewidth=2,
    label=f'Fit: d = {a_w:.1f}/w + {b_w:.2f}'
)
axes[0].set_title('Bounding Box Width vs. Surface Distance')
axes[0].set_xlabel('Bounding Box Width (px)')
axes[0].set_ylabel('Surface Distance (m)')
axes[0].grid(True, linestyle='--', alpha=0.5)
axes[0].legend()

# Plot 2: Bounding Box Height vs Surface Distance
axes[1].scatter(heights, distances, alpha=0.6, color='green', label='Measured data')
axes[1].plot(
    h_curve,
    a_h / h_curve + b_h,
    color='red',
    linewidth=2,
    label=f'Fit: d = {a_h:.1f}/h + {b_h:.2f}'
)
axes[1].set_title('Bounding Box Height vs. Surface Distance')
axes[1].set_xlabel('Bounding Box Height (px)')
axes[1].grid(True, linestyle='--', alpha=0.5)
axes[1].legend()

plt.tight_layout()
plt.savefig('barrel_size_vs_distance.png', dpi=150)
plt.show()