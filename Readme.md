### Dartsim to URDF

This repository includes 
1. Dartsim skeleton xml to urdf
2. Simple urdf viewer (isaacgym)

Due to the isaacgym, I tested this code with python3.7

1. Installation
```
pip install -r requirements.txt
```

2. Skeleton XML conversion 
```
(Change the input_dart, output_urdf and obj_root_in_urdf variable in the dart_to_urdf.py)
python dart_to_urdf.py
```

3. URDF viewer
```
(Confirm that you properly installed isaacgym)
(Change the asset_descriptors in the urdf_viewer.py)
python urdf_viewer.py
```
