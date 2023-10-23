#!/bin/bash

python python/generate_data_for_exp1.py --map_path map_file/Boston_0_256.map --output_dir map_file/Paper_Boston_gp_5

python python/generate_data_for_exp1.py --map_path map_file/random-32-32-10.map --output_dir map_file/Paper_random_32_32_gp_5

python python/generate_data_for_exp1.py --map_path map_file/den312d.map --output_dir map_file/Paper_den312d_65_81_gp_5

python python/generate_data_for_exp1.py --map_path map_file/empty-32-32.map --output_dir map_file/Paper_empty_32_32_gp_5

python python/generate_data_for_exp1.py --map_path map_file/maze-32-32-2.map --output_dir map_file/Paper_maze_32_32_2_gp_5

python python/generate_data_for_exp1.py --map_path map_file/room-64-64-8.map --output_dir map_file/Paper_room_64_64_8_gp_5

python python/generate_data_for_exp1.py --map_path map_file/warehouse-10-20-10-2-1.map --output_dir map_file/Paper_warehouse_161_63_gp_5

python python/generate_data_for_exp1.py --map_path map_file/orz900d.map --output_dir map_file/Paper_orz900d_gp_5

