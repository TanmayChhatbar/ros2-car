# source python venv from src/vehicle_sim/src/.venv
#   change directory to src/vehicle_sim/src
#   source .venv/bin/activate
#   run this script 

g++ -pg -I./include ./include/*.cpp Vehicle2D_test.cpp -o a.out -lm

./a.out

gprof a.out gmon.out | gprof2dot -s -w| dot -Gdpi=200 -Tpng -o profile.png

rm a.out gmon.out
