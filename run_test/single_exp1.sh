#!/bin/bash

TYMPath=$1
seed=$2
outputfile=$3

python3 python/compareTYM2TA.py --TYMPath $TYMPath --map_dir map_file/Paper_random_32_32_gp_5 --time 30 --seed $seed >> $outputfile

python3 python/compareTYM2TA.py --TYMPath $TYMPath --map_dir map_file/Paper_den312d_65_81_gp_5 --time 30 --seed $seed >> $outputfile

python3 python/compareTYM2TA.py --TYMPath $TYMPath --map_dir map_file/Paper_empty_32_32_gp_5 --time 30 --seed $seed >> $outputfile

python3 python/compareTYM2TA.py --TYMPath $TYMPath --map_dir map_file/Paper_maze_32_32_2_gp_5 --time 30 --seed $seed >> $outputfile

python3 python/compareTYM2TA.py --TYMPath $TYMPath --map_dir map_file/Paper_room_64_64_8_gp_5 --time 30 --seed $seed >> $outputfile

python3 python/compareTYM2TA.py --TYMPath $TYMPath --map_dir map_file/Paper_warehouse_161_63_gp_5 --time 30 --seed $seed >> $outputfile

python3 python/compareTYM2TA.py --TYMPath $TYMPath --map_dir map_file/Paper_orz900d_gp_5 --time 50 --seed $seed >> $outputfile

python3 python/compareTYM2TA.py --TYMPath $TYMPath --map_dir map_file/Paper_Boston_gp_5 --time 50 --seed $seed >> $outputfile
