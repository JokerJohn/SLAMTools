# SLAMTools


Our sensor suit:

![image-20231028215323058](./README/image-20231028215323058.png)



## Viewer

### Evaluate with EVO results

first get evo ape and rpe results as `test.zip` and `test_rpe.zip`

```python
#dependence
pip3 install zipfile pandas matplotlib numpy
#run
python3 evo_eval_viewer.py
```

![image-20231112140452898](./README/image-20231112140452898.png)



### Run_time_analysis

```python
#denpendence
pip3 install matplotlib pandas

#run
python3 time_analysis.py

# 1698490753.03 0.135222 221.497434 0.022776 0.491802 236.377261 0.651435 1194.924191 0.132663 3.51924 0
# timestamp module1_time module2_time module3_time module4_time total_time module1_total_time module2_total_time module13_tota_time module4_total_time 
# since we just use module2_time and module4_time, we can get the following figure.
# ref to paper: PALoc https://ieeexplore.ieee.org/document/10480308
```

![image-20231112140414855](./README/image-20231112140414855.png)

### pose_graph_cov_2d

![image-20231111105428392](./README/image-20231111105428392.png)

### tum_traj_viewer

![image-20231030195632044](./README/image-20231030195632044.png)

### tum_rpy_viewer

![image-20231030195727953](./README/image-20231030195727953.png)

### var_analysis

![image-20231030200126073](./README/image-20231030200126073.png)

## Evaluation

### eval_rpy_viewer.py

![image-20231026172239328](README/image-20231026172239328.png)

### eval_xyz_viewer

![image-20231026172350083](README/image-20231026172350083.png)

### eval_xyz_viewer_colorbar

![image-20231027030705921](./README/image-20231027030705921.png)

![image-20231028215632469](./README/image-20231028215632469.png)

![image-20231028215753179](./README/image-20231028215753179.png)
