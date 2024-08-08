# MS-Mapping Tools

[TOC]

| Scripts                          | Test Data       |
| -------------------------------- | --------------- |
| Merge map                        | coming soon..   |
| Separate Trajectory              | coming soon..   |
| Visualize Graph Error Adjustment | graph_error.txt |
| Visualize Trajectory             | session_*.txt   |

## Merge map

Set your folder path,  if you use `CP2` to do incrimental mapping based on `CP5`, you must set the map folder of `CP2` as **new_pcd_folder**, and set all the map folder of  old sessions in the **old_pcd_folders**.

```python
# Configuration
new_pcd_folder = '/media/xchu/UBUNTU 20_0/results_ms/CP05-CP02-CS1-CC1-PK1-IA3-IA4-NG'

old_pcd_folders = [
    '/media/xchu/UBUNTU 20_0/results_ms/CP05-CP02-CS1-CC1-PK1-IA3-IA4',
    '/media/xchu/UBUNTU 20_0/results_ms/CP05-CP02-CS1-CC1-PK1-IA3',
    '/media/xchu/UBUNTU 20_0/results_ms/CP05-CP02-CS1-CC1-PK1',
    '/media/xchu/UBUNTU 20_0/results_ms/CP05-CP02-CS1-CC1',
    '/media/xchu/UBUNTU 20_0/results_ms/CP05-CP02-CS1',
    '/home/xchu/data/pose_slam_prior_result/CP05-CP02',
    # Add more old folders as needed
]
```

Run

```python
python3 multi-session-map-merger2.py
```

Finally we can get the merd point cloud map in `/media/xchu/UBUNTU 20_0/results_ms/CP05-CP02-CS1-CC1-PK1-IA3-IA4-NG/merged_results`

![image-20240808095356672](./README/image-20240808095356672.png)

![image-20240808095723490](./README/image-20240808095723490.png)

Run

```bash
pcl_viewer merged_map_session_*
```

![image-20240808100331299](./README/image-20240808100331299.png)

## Separate Trajectory

The same setting as **Merge map**

![image-20240808095840022](./README/image-20240808095840022.png)

Run

```python
python3 multi-session-map-merger_writetum.py
```

![image-20240808100053941](./README/image-20240808100053941.png)

## Visualize Trajectory

Set the trajectory folder

```python
pose_folder = "/media/xchu/UBUNTU 20_0/results_ms/CP05-CP02-CS1-CC1-PK1-IA3-IA4-NG/merged_results"  # 请替换为实际的文件夹路径
```

Run

```python
python3 tum-trajectory-plotter.py
```

![image-20240808100216334](./README/image-20240808100216334.png)

## Visualize Graph Error Adjustment

set the file path of **graph_error.txt**, this file will automatically saved after mapping.

![image-20240808100802980](./README/image-20240808100802980.png)

![image-20240808100815419](./README/image-20240808100815419.png)

Run

```python
python3 graph_error_viewer.py 
```

![image-20240808100924350](./README/image-20240808100924350.png)