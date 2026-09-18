
from pathlib import Path

from PIL import Image, ImageDraw, ImageFont


def create_global_image_grid(
    source,
    output_dir="output",
    image_width=400,
    image_height=400,
    grid_rows=4,
    grid_cols=2
):
    """
    source:
        2D array of image paths.

    Example:
        [
            ["img00.jpg", "img01.jpg"],
            ["img10.jpg", "img11.jpg"]
        ]

    image_width / image_height:
        Size of EACH image after resizing.

    grid_rows / grid_cols:
        Number of cells INSIDE EACH image.

    Saves:
        1. Individual gridded maps
        2. Final combined gridded map

    Returns:
        combined_image : Final combined image
        matrix         : Global coordinate matrix
    """

    # --------------------------------------------------
    # 1. Validate source
    # --------------------------------------------------

    if not source:
        raise ValueError("Source cannot be empty.")

    if not all(len(row) == len(source[0]) for row in source):
        raise ValueError("All rows in source must have the same length.")

    if image_width % grid_cols != 0:
        raise ValueError("image_width must be divisible by grid_cols.")

    if image_height % grid_rows != 0:
        raise ValueError("image_height must be divisible by grid_rows.")

    # --------------------------------------------------
    # 2. Create output directories
    # --------------------------------------------------

    output_path = Path(output_dir)
    individual_dir = output_path / "individual_maps"

    individual_dir.mkdir(parents=True, exist_ok=True)

    # --------------------------------------------------
    # 3. Calculate dimensions
    # --------------------------------------------------

    source_rows = len(source)
    source_cols = len(source[0])

    cell_width = image_width // grid_cols
    cell_height = image_height // grid_rows

    final_width = source_cols * image_width
    final_height = source_rows * image_height

    # --------------------------------------------------
    # 4. Create final combined canvas
    # --------------------------------------------------

    combined_image = Image.new(
        "RGB",
        (final_width, final_height),
        "white"
    )

    # --------------------------------------------------
    # 5. Load font
    # --------------------------------------------------

    try:
        font = ImageFont.truetype("arial.ttf", 18)
    except:
        font = ImageFont.load_default()

    # --------------------------------------------------
    # 6. Create GLOBAL coordinate matrix
    # --------------------------------------------------

    total_rows = source_rows * grid_rows
    total_cols = source_cols * grid_cols

    matrix = [
        [(r, c) for c in range(total_cols)]
        for r in range(total_rows)
    ]

    # --------------------------------------------------
    # 7. Process every image
    # --------------------------------------------------

    for image_row in range(source_rows):

        for image_col in range(source_cols):

            image_path = source[image_row][image_col]

            # Load and resize
            image = Image.open(image_path).convert("RGB")
            image = image.resize((image_width, image_height))

            # --------------------------------------------------
            # A. Create INDIVIDUAL gridded map
            # --------------------------------------------------

            individual_map = image.copy()
            individual_draw = ImageDraw.Draw(individual_map)

            for local_row in range(grid_rows):

                for local_col in range(grid_cols):

                    # GLOBAL coordinates
                    global_row = (
                        image_row * grid_rows
                        + local_row
                    )

                    global_col = (
                        image_col * grid_cols
                        + local_col
                    )

                    # Pixel position INSIDE this image
                    x1 = local_col * cell_width
                    y1 = local_row * cell_height

                    x2 = x1 + cell_width
                    y2 = y1 + cell_height

                    # Draw grid
                    individual_draw.rectangle(
                        [x1, y1, x2, y2],
                        outline="red",
                        width=2
                    )

                    # Coordinate label
                    label = f"({global_row},{global_col})"

                    individual_draw.rectangle(
                        [
                            x1 + 3,
                            y1 + 3,
                            x1 + 3 + len(label) * 9,
                            y1 + 22
                        ],
                        fill="white"
                    )

                    individual_draw.text(
                        (x1 + 3, y1 + 3),
                        label,
                        fill="blue",
                        font=font
                    )

            # Save individual map
            individual_path = individual_dir / (
                f"map_{image_row:02d}_{image_col:02d}.png"
            )

            individual_map.save(individual_path)

            # --------------------------------------------------
            # B. Paste image into FINAL combined map
            # --------------------------------------------------

            image_x = image_col * image_width
            image_y = image_row * image_height

            combined_image.paste(
                image,
                (image_x, image_y)
            )

    # --------------------------------------------------
    # 8. Draw GLOBAL grid on final combined map
    # --------------------------------------------------

    final_draw = ImageDraw.Draw(combined_image)

    for global_row in range(total_rows):

        for global_col in range(total_cols):

            x1 = global_col * cell_width
            y1 = global_row * cell_height

            x2 = x1 + cell_width
            y2 = y1 + cell_height

            # Draw grid
            final_draw.rectangle(
                [x1, y1, x2, y2],
                outline="red",
                width=2
            )

            # Coordinate label
            label = f"({global_row},{global_col})"

            final_draw.rectangle(
                [
                    x1 + 3,
                    y1 + 3,
                    x1 + 3 + len(label) * 9,
                    y1 + 22
                ],
                fill="white"
            )

            final_draw.text(
                (x1 + 3, y1 + 3),
                label,
                fill="blue",
                font=font
            )

    # --------------------------------------------------
    # 9. Save FINAL combined map
    # --------------------------------------------------

    final_path = output_path / "final_grid.png"

    combined_image.save(final_path)

    return combined_image, matrix