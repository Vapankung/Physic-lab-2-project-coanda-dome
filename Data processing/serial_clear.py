import os
import re
import csv
import shutil
from pathlib import Path

# =========================================================
# SETTINGS
# =========================================================
ROOT_FOLDER = r"D:\Arduino\Physic2 project\Data processing"          # change this to your root folder
DRY_RUN = False              # True = only show folders to delete
DELETE_EMPTY_PARENTS = False  # optional: remove parent folders if they become empty

# =========================================================
# HELPERS
# =========================================================
SESSION_PATTERN = re.compile(
    r"^Session started:\s+\d{4}-\d{2}-\d{2}T\d{2}:\d{2}:\d{2}(?:\.\d+)?$"
)

def serial_log_is_only_session_started(file_path: Path) -> bool:
    """
    True if serial_log.txt contains exactly one meaningful line:
    Session started: [ISO timestamp]
    Blank lines are ignored.
    """
    try:
        with file_path.open("r", encoding="utf-8", errors="ignore") as f:
            lines = [line.strip() for line in f.readlines() if line.strip()]
        return len(lines) == 1 and bool(SESSION_PATTERN.match(lines[0]))
    except Exception as e:
        print(f"Error reading {file_path}: {e}")
        return False


def plot_csv_has_only_header(file_path: Path) -> bool:
    """
    True if plot_data.csv has exactly one non-empty row (the header)
    and no data rows.
    Completely blank lines are ignored.
    """
    try:
        with file_path.open("r", encoding="utf-8", errors="ignore", newline="") as f:
            reader = csv.reader(f)
            rows = []

            for row in reader:
                # ignore completely blank rows
                if any(cell.strip() for cell in row):
                    rows.append(row)

        return len(rows) == 1
    except Exception as e:
        print(f"Error reading {file_path}: {e}")
        return False


def folder_matches(folder: Path) -> bool:
    """
    Folder must contain both files:
      - serial_log.txt
      - plot_data.csv
    and both must match the required conditions.
    """
    serial_log = folder / "serial_log.txt"
    plot_csv = folder / "plot_data.csv"

    if not serial_log.is_file() or not plot_csv.is_file():
        return False

    return (
        serial_log_is_only_session_started(serial_log)
        and plot_csv_has_only_header(plot_csv)
    )


def remove_empty_parents(start_folder: Path, stop_at: Path):
    """
    Optionally remove empty parent folders after deletion,
    stopping at stop_at.
    """
    current = start_folder.parent
    stop_at = stop_at.resolve()

    while current.resolve() != stop_at.parent.resolve():
        try:
            if any(current.iterdir()):
                break
            current.rmdir()
            print(f"Removed empty parent: {current}")
            current = current.parent
        except Exception:
            break


# =========================================================
# MAIN
# =========================================================
def main():
    root = Path(ROOT_FOLDER).resolve()

    if not root.exists() or not root.is_dir():
        print(f"Invalid root folder: {root}")
        return

    folders_to_delete = []

    # Walk through all folders
    for dirpath, dirnames, filenames in os.walk(root, topdown=False):
        folder = Path(dirpath)

        if folder_matches(folder):
            folders_to_delete.append(folder)

    if not folders_to_delete:
        print("No matching folders found.")
        return

    print("Matching folders:")
    for folder in folders_to_delete:
        print(f" - {folder}")

    print(f"\nTotal matched: {len(folders_to_delete)}")

    if DRY_RUN:
        print("\nDRY_RUN is True, so nothing was deleted.")
        return

    for folder in folders_to_delete:
        try:
            shutil.rmtree(folder)
            print(f"Deleted: {folder}")

            if DELETE_EMPTY_PARENTS:
                remove_empty_parents(folder, root)

        except Exception as e:
            print(f"Failed to delete {folder}: {e}")


if __name__ == "__main__":
    main()