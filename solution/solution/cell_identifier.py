import cv2
import numpy as np

CAMERA_WIDTH = 640
CAMERA_HEIGHT = 480
CAMERA_FOV = 1.5708 # rad
CAMERA_ABOVE_WINCH = 0.12 # camera pose from winch
WINCH_TOP_Z = 3.615
ROD_SEATED_Z = -3.85
ROD_HALF_HEIGHT = 0.95

COLUMNS = 13 # x cells
ROWS = 9 # y cells
CELL_GAP = 0.5 # 0.5m between cell centers
FIRST_COLUMN_X = {'reactor': -8.75, 'storage': 2.75}
FIRST_COLUMN_LETTER = {'reactor': 'A', 'storage': 'N'}

SAMPLE_SIZE = 4 # Length of the square of pixels to sample from for each cell


def cell_centres(pool):
    for i in range(COLUMNS):
            for j in range(ROWS):
                name = f"{chr(ord(FIRST_COLUMN_LETTER[pool]) + i)}{j}"
                yield name, FIRST_COLUMN_X[pool] + CELL_GAP * i, 2.0 - CELL_GAP * j # name, x, y of cell



def identify_cell(h, s, v):
    if 15 <= h <= 35 and s > 100:
        return "control" # yellow 
    if 85 <= h <= 110 and 15 <= s <= 70 and v > 200:
        return "fuel" # light blue
    return "empty"



def identify_pool_cells(image, pool, camera_x, camera_y, winch_lift):
    # maps cell name to "control", "fuel" or "empty" based on the median HSV at the 
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    rod_top_z = ROD_SEATED_Z + ROD_HALF_HEIGHT
    focal_length_px = CAMERA_WIDTH / (2 * np.tan(CAMERA_FOV / 2))
    depth = (WINCH_TOP_Z + winch_lift + CAMERA_ABOVE_WINCH) - rod_top_z
    pixels_per_metre = focal_length_px / depth
    cells = {}
    for name, x, y in cell_centres(pool):
        cell_column_pxl = int(round((CAMERA_WIDTH / 2 - (y - camera_y) * pixels_per_metre)))
        cell_row_height_pxl = int(round((CAMERA_HEIGHT / 2 - (x - camera_x) * pixels_per_metre)))
        half_sample = SAMPLE_SIZE // 2
        patch = hsv[cell_row_height_pxl - half_sample:cell_row_height_pxl + half_sample + 1, cell_column_pxl - half_sample:cell_column_pxl + half_sample + 1].reshape(-1, 3)
        h, s, val = np.median(patch, axis=0)
        cells[name] = identify_cell(h, s, val)
    return cells
