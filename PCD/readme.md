# LAS TO PCD

## covert_las_pcd

```python
    input_folder = '/media/xchu/新加卷/LEICA/highbay'
    main(input_folder, input_folder+'/pcds/')
```

![image-20231031124426738](./readme/image-20231031124426738.png)

![image-20231031124842423](./readme/image-20231031124842423.png)

## covert_las_pcd_multithread

multithread process

```python
    # set your thread number
    with multiprocessing.Pool(processes=10) as pool:
```

## merge_las_to_pcd.py

put the las file in the same folder and set the folder path, this script can merge all the las into a pcd file, together with single pcd files.

```python
python3 merge_las_to_pcd.py
```

![image-20250701181419521](./readme/image-20250701181419521.png)

![image-20250701181505922](./readme/image-20250701181505922.png)

![image-20250701181519207](./readme/image-20250701181519207.png)

## merge_map_from_scan

just install open3d

```python
pip3 install open3d
```

then set your folder

```python
#1.TUM file
#2.pcd folder

folder = '/home/xchu/data/prior_map/20230710-final/'
pose_file = folder + 'optimized_odom_tum.txt'
pcd_folder = folder + 'key_point_frame'
output_file = folder + 'merged_map.pcd'
```

![image-20240401220655121](./readme/image-20240401220655121.png)

then 

```
python3 merge_map_from_scan.py 
```

## merge_map_from_scan_multi

```python
pip3 install tdqm
python3 merge_map_from_scan_multi.py 
```

![image-20240401220841628](./readme/image-20240401220841628.png)

![image-20240401220913051](./readme/image-20240401220913051.png)
