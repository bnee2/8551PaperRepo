#!/bin/bash
./compile.sh
cd build

# Random 3
./lifelong --inputFile ../example_problems/random3.domain/random3_2.json -o test_random3_2.json
./lifelong --inputFile ../example_problems/random3.domain/random3_3.json -o test_random3_3.json
./lifelong --inputFile ../example_problems/random3.domain/random3_4.json -o test_random3_4.json
./lifelong --inputFile ../example_problems/random3.domain/random3_5.json -o test_random3_5.json
./lifelong --inputFile ../example_problems/random3.domain/random3_6.json -o test_random3_6.json
./lifelong --inputFile ../example_problems/random3.domain/random3_7.json -o test_random3_7.json
./lifelong --inputFile ../example_problems/random3.domain/random3_8.json -o test_random3_8.json
./lifelong --inputFile ../example_problems/random3.domain/random3_9.json -o test_random3_9.json
./lifelong --inputFile ../example_problems/random3.domain/random3_10.json -o test_random3_10.json 
./lifelong --inputFile ../example_problems/random3.domain/random3_15.json -o test_random3_15.json
./lifelong --inputFile ../example_problems/random3.domain/random3_20.json -o test_random3_20.json
