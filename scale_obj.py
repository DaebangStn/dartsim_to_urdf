from pathlib import Path


input_dir = "data/skeleton_obj"
output_dir = "data/scaled"

scale = 0.01


def scale_obj(_in_path: str, _out_path:str, _scale: float):
    with open(_in_path, "r") as f:
        lines = f.readlines()

    with open(_out_path, "w") as f:
        for line in lines:
            if line.startswith('v '):
                x, y, z = [float(x) * _scale for x in line.split()[1:]]
                f.write(f"v {x} {y} {z}\n")
            else:
                f.write(line)


if __name__ == "__main__":
    for in_path in Path(input_dir).glob("*.obj"):
        out_path = Path(output_dir) / in_path.name
        scale_obj(in_path.as_posix(), out_path.as_posix(), scale)
        print(f"Saved {out_path.name}")
