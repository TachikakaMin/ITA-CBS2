#!/bin/bash

TYMPath=$1
CBSTAPath=$2
seed=$3
outputfile=$4

python3 python/compareTYM2TA.py --TYMPath $TYMPath --CBSTAPath $CBSTAPath --map_dir map_file/paper_random_32_32_ratio_000 --time 30 --seed $seed >> $outputfile
python3 python/compareTYM2TA.py --TYMPath $TYMPath --CBSTAPath $CBSTAPath --map_dir map_file/paper_random_32_32_ratio_030 --time 30 --seed $seed >> $outputfile
python3 python/compareTYM2TA.py --TYMPath $TYMPath --CBSTAPath $CBSTAPath --map_dir map_file/paper_random_32_32_ratio_060 --time 30 --seed $seed >> $outputfile
python3 python/compareTYM2TA.py --TYMPath $TYMPath --CBSTAPath $CBSTAPath --map_dir map_file/paper_random_32_32_ratio_100 --time 30 --seed $seed >> $outputfile

python3 python/compareTYM2TA.py --TYMPath $TYMPath --CBSTAPath $CBSTAPath --map_dir map_file/paper_den312d_65_81_ratio_000 --time 30 --seed $seed >> $outputfile
python3 python/compareTYM2TA.py --TYMPath $TYMPath --CBSTAPath $CBSTAPath --map_dir map_file/paper_den312d_65_81_ratio_030 --time 30 --seed $seed >> $outputfile
python3 python/compareTYM2TA.py --TYMPath $TYMPath --CBSTAPath $CBSTAPath --map_dir map_file/paper_den312d_65_81_ratio_060 --time 30 --seed $seed >> $outputfile
python3 python/compareTYM2TA.py --TYMPath $TYMPath --CBSTAPath $CBSTAPath --map_dir map_file/paper_den312d_65_81_ratio_100 --time 30 --seed $seed >> $outputfile

python3 python/compareTYM2TA.py --TYMPath $TYMPath --CBSTAPath $CBSTAPath --map_dir map_file/paper_empty_32_32_ratio_000 --time 30 --seed $seed >> $outputfile
python3 python/compareTYM2TA.py --TYMPath $TYMPath --CBSTAPath $CBSTAPath --map_dir map_file/paper_empty_32_32_ratio_030 --time 30 --seed $seed >> $outputfile
python3 python/compareTYM2TA.py --TYMPath $TYMPath --CBSTAPath $CBSTAPath --map_dir map_file/paper_empty_32_32_ratio_060 --time 30 --seed $seed >> $outputfile
python3 python/compareTYM2TA.py --TYMPath $TYMPath --CBSTAPath $CBSTAPath --map_dir map_file/paper_empty_32_32_ratio_100 --time 30 --seed $seed >> $outputfile

python3 python/compareTYM2TA.py --TYMPath $TYMPath --CBSTAPath $CBSTAPath --map_dir map_file/paper_maze_32_32_2_ratio_000 --time 30 --seed $seed >> $outputfile
python3 python/compareTYM2TA.py --TYMPath $TYMPath --CBSTAPath $CBSTAPath --map_dir map_file/paper_maze_32_32_2_ratio_030 --time 30 --seed $seed >> $outputfile
python3 python/compareTYM2TA.py --TYMPath $TYMPath --CBSTAPath $CBSTAPath --map_dir map_file/paper_maze_32_32_2_ratio_060 --time 30 --seed $seed >> $outputfile
python3 python/compareTYM2TA.py --TYMPath $TYMPath --CBSTAPath $CBSTAPath --map_dir map_file/paper_maze_32_32_2_ratio_100 --time 30 --seed $seed >> $outputfile

python3 python/compareTYM2TA.py --TYMPath $TYMPath --CBSTAPath $CBSTAPath --map_dir map_file/paper_room_64_64_8_ratio_000 --time 30 --seed $seed >> $outputfile
python3 python/compareTYM2TA.py --TYMPath $TYMPath --CBSTAPath $CBSTAPath --map_dir map_file/paper_room_64_64_8_ratio_030 --time 30 --seed $seed >> $outputfile
python3 python/compareTYM2TA.py --TYMPath $TYMPath --CBSTAPath $CBSTAPath --map_dir map_file/paper_room_64_64_8_ratio_060 --time 30 --seed $seed >> $outputfile
python3 python/compareTYM2TA.py --TYMPath $TYMPath --CBSTAPath $CBSTAPath --map_dir map_file/paper_room_64_64_8_ratio_100 --time 30 --seed $seed >> $outputfile

python3 python/compareTYM2TA.py --TYMPath $TYMPath --CBSTAPath $CBSTAPath --map_dir map_file/paper_warehouse_161_63_ratio_000 --time 30 --seed $seed >> $outputfile
python3 python/compareTYM2TA.py --TYMPath $TYMPath --CBSTAPath $CBSTAPath --map_dir map_file/paper_warehouse_161_63_ratio_030 --time 30 --seed $seed >> $outputfile
python3 python/compareTYM2TA.py --TYMPath $TYMPath --CBSTAPath $CBSTAPath --map_dir map_file/paper_warehouse_161_63_ratio_060 --time 30 --seed $seed >> $outputfile
python3 python/compareTYM2TA.py --TYMPath $TYMPath --CBSTAPath $CBSTAPath --map_dir map_file/paper_warehouse_161_63_ratio_100 --time 30 --seed $seed >> $outputfile


python3 python/compareTYM2TA.py --TYMPath $TYMPath --CBSTAPath $CBSTAPath --map_dir map_file/paper_orz900d_ratio_000 --time 50 --seed $seed >> $outputfile
python3 python/compareTYM2TA.py --TYMPath $TYMPath --CBSTAPath $CBSTAPath --map_dir map_file/paper_orz900d_ratio_030 --time 50 --seed $seed >> $outputfile
python3 python/compareTYM2TA.py --TYMPath $TYMPath --CBSTAPath $CBSTAPath --map_dir map_file/paper_orz900d_ratio_060 --time 50 --seed $seed >> $outputfile
python3 python/compareTYM2TA.py --TYMPath $TYMPath --CBSTAPath $CBSTAPath --map_dir map_file/paper_orz900d_ratio_100 --time 50 --seed $seed >> $outputfile

python3 python/compareTYM2TA.py --TYMPath $TYMPath --CBSTAPath $CBSTAPath --map_dir map_file/paper_Boston_ratio_000 --time 50 --seed $seed >> $outputfile
python3 python/compareTYM2TA.py --TYMPath $TYMPath --CBSTAPath $CBSTAPath --map_dir map_file/paper_Boston_ratio_030 --time 50 --seed $seed >> $outputfile
python3 python/compareTYM2TA.py --TYMPath $TYMPath --CBSTAPath $CBSTAPath --map_dir map_file/paper_Boston_ratio_060 --time 50 --seed $seed >> $outputfile
python3 python/compareTYM2TA.py --TYMPath $TYMPath --CBSTAPath $CBSTAPath --map_dir map_file/paper_Boston_ratio_100 --time 50 --seed $seed >> $outputfile
