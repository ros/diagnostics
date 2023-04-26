import yaml
from pathlib import Path

base_dir = Path(__file__).parents[0]

file_path = Path.joinpath(base_dir, "example.yaml")
with open(file_path, "r", encoding="utf-8") as f:
    data = yaml.safe_load(f)

print(data)

print(yaml.dump(data))
