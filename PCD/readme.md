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

