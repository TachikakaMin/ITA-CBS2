# ITA-CBS2

map_data: [data_download](https://drive.google.com/file/d/1NOI4AxlLeqFZKxPTLKaU5x70LJUBswk-/view?usp=sharing)

ITA-CBS new code
Build:

```bash
mkdir build
cd build
cmake ..
cmake --build . --target ITACBS_remake
cmake --build . --target CBSTA_remake
```

Run:
```bash
./ITACBS_remake -i ../map_file/debug_data.yaml -o ../outputs/output.yaml
```

Generate test case:

```bash
# please check generate_data_for_exp1.py/generate_data_for_exp2.py
# there are some config in it

# in root dir:
python python/generate_data_for_exp2.py --map_path map_file/Boston_0_256.map --output_dir map_file/Paper_boston_256_256_060 --common_ratio 0.6
```

Test:

```bash
# TYMPath and CBSTAPath are just paths to binary file, no difference
# in root dir:
python python/compareTYM2TA.py --TYMPath cmake-build-debug/CBSTA_remake --CBSTAPath cmake-build-debug/ITACBS_remake --map_dir map_file/Paper_boston_256_256_060 --time 15 --seed 0
```

Reproduce Paper Results:

```bash
./run_test/single_exp1.sh build/ITACBS_remake 1 exp1_ITACBS.txt
./run_test/single_exp1.sh build/CBSTA 1 exp1_CBSTA.txt
./run_test/single_exp2.sh build/ITACBS_remake 1 exp2_ITACBS.txt
./run_test/single_exp2.sh build/CBSTA 1 exp2_ITACBS.txt
```


Visualize:

Old:
```bash
# don't use large map(larger than 100*100), it will be very slow and agents will be very small.
# in root dir:
python python/visualize.py [your testcase yaml file] [your program output yaml]
```

New:
You should clone my PlanViz and use ITA-CBS branch:
```bash
python python/PlanViz/script/plan_viz.py --map  map_file/Boston_0_256.map --plan_path_type2 outputs/output.yaml --grid --aid --ca --tid --ppm 2
```


Thank @MinakoOikawa (twitter id) for providing the correct dynamic hungarian implementation.
