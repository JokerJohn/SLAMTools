```bash
mkdir results

evo_ape tum GT.txt Proposed.txt -a -r trans_part --save_results=results/a.zip && unzip results/a.zip -d results/a

evo_ape tum GT.txt Proposed.txt -a -r angle_deg --save_results=results/b.zip && unzip results/a.zip -d results/b
```
