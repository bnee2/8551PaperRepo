#!/bin/bash
./compile.sh
cd build
rm test.json
# ./lifelong --inputFile ../example_problems/random.domain/random_5.json -o test.json
./lifelong --inputFile ../example_problems/random2.domain/random2_3.json -o test.json
cd ..
# python3 ../PlanViz/script/plan_viz.py --map ./example_problems/random.domain/maps/random-32-32-20.map --plan ./build/test.json --grid --aid --static --ca
python3 ../PlanViz/script/plan_viz.py --map ./example_problems/random2.domain/mapData.map --plan ./build/test.json --grid --aid --static --ca