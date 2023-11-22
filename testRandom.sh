#!/bin/bash
./compile.sh
cd build

# Random
# ./lifelong --inputFile ../example_problems/random.domain/random_2.json -o test_random_2.json
# ./lifelong --inputFile ../example_problems/random.domain/random_3.json -o test_random_3.json
# ./lifelong --inputFile ../example_problems/random.domain/random_4.json -o test_random_4.json
# ./lifelong --inputFile ../example_problems/random.domain/random_5.json -o test_random_5.json
# ./lifelong --inputFile ../example_problems/random.domain/random_6.json -o test_random_6.json
# ./lifelong --inputFile ../example_problems/random.domain/random_7.json -o test_random_7.json
./lifelong --inputFile ../example_problems/random.domain/random_8.json -o test_random_8.json
./lifelong --inputFile ../example_problems/random.domain/random_9.json -o test_random_9.json
# ./lifelong --inputFile ../example_problems/random.domain/random_10.json -o test_random_10.json 
./lifelong --inputFile ../example_problems/random.domain/random_15.json -o test_random_15.json # ICTS didn't complete step 0 in 24 hours
./lifelong --inputFile ../example_problems/random.domain/random_20.json -o test_random_20.json
