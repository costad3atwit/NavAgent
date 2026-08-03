from pathlib import Path

def createPath(folder_name: str, output_name: str) -> Path:
    output_dir = Path(folder_name)
    output_dir.mkdir(parents=True, exist_ok=True)

    index = 1
    while True:
        path = output_dir / f"{output_name}{index}.txt"
        if not path.exists():
            break
        index += 1

    return path