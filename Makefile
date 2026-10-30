.PHONY: test sim flash-lead flash-follower
CXX ?= c++
test:
	$(CXX) -std=c++17 -Wall -Wextra -Ifirmware/lib/convoy firmware/test/test_convoy.cpp -o /tmp/test_convoy && /tmp/test_convoy
sim:
	$(CXX) -std=c++17 -O2 -Ifirmware/lib/convoy sim/blackout_sim.cpp -o /tmp/blackout_sim && /tmp/blackout_sim > data/blackout.csv
	python3 -m analysis.blackout data/blackout.csv
flash-lead:
	cd firmware && pio run -e lead -t upload
flash-follower:
	cd firmware && pio run -e follower -t upload
