# Copyright 2020 tobias.gierke@code-sourcery.de
# 
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
# 
#     http://www.apache.org/licenses/LICENSE-2.0
# 
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

# Requirements for building:
#
# GCC
#
# - libsensors & header files (Ubuntu: apt-get install lm-sensors libsensors4-dev)
# - libatasmart (Ubuntu: apt-get install libatasmart-dev)
# - libcjson (Ubuntu: apt-get install libcjson-dev)
# - libsystemd (Ubuntu: apt-get install libsystemd-dev)
all:
#	gcc -O0 -g -L/usr/lib/x86_64-linux-gnu fancontrol.c -o fancontrol -lsensors -latasmart -lcjson -lm -lsystemd
	gcc -L/usr/lib/x86_64-linux-gnu fancontrol.c -o fancontrol -lsensors -latasmart -lcjson -lm -lsystemd
clean:
	-rm fancontrol *.raw *.out 2>&1 >/dev/null

.PHONY: clean grind

grind: clean all
	valgrind --tool=memcheck --leak-check=yes --show-reachable=yes --num-callers=20 --track-fds=yes ./fancontrol
