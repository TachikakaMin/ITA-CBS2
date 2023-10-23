#!/bin/bash

TYMPath=$1
CBSTAPath=$2
seed=$3
weight1=$4
outputfile=$5

python3 python/compareTYM2TA.py --TYMPath $TYMPath --CBSTAPath $CBSTAPath --map_dir map_file/Paper_random_32_32_gp_5 --time 30 --seed $seed --weight1 $weight1  >> $outputfile

python3 python/compareTYM2TA.py --TYMPath $TYMPath --CBSTAPath $CBSTAPath --map_dir map_file/Paper_den312d_65_81_gp_5 --time 30 --seed $seed --weight1 $weight1  >> $outputfile

python3 python/compareTYM2TA.py --TYMPath $TYMPath --CBSTAPath $CBSTAPath --map_dir map_file/Paper_empty_32_32_gp_5 --time 30 --seed $seed --weight1 $weight1  >> $outputfile

python3 python/compareTYM2TA.py --TYMPath $TYMPath --CBSTAPath $CBSTAPath --map_dir map_file/Paper_maze_32_32_2_gp_5 --time 30 --seed $seed --weight1 $weight1  >> $outputfile

python3 python/compareTYM2TA.py --TYMPath $TYMPath --CBSTAPath $CBSTAPath --map_dir map_file/Paper_room_64_64_8_gp_5 --time 30 --seed $seed --weight1 $weight1  >> $outputfile

python3 python/compareTYM2TA.py --TYMPath $TYMPath --CBSTAPath $CBSTAPath --map_dir map_file/Paper_warehouse_161_63_gp_5 --time 30 --seed $seed --weight1 $weight1  >> $outputfile

python3 python/compareTYM2TA.py --TYMPath $TYMPath --CBSTAPath $CBSTAPath --map_dir map_file/Paper_orz900d_gp_5 --time 50 --seed $seed --weight1 $weight1  >> $outputfile

python3 python/compareTYM2TA.py --TYMPath $TYMPath --CBSTAPath $CBSTAPath --map_dir map_file/Paper_Boston_gp_5 --time 50 --seed $seed --weight1 $weight1  >> $outputfile
