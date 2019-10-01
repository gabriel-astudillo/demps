SRC=src
RESULTS_SIM=sim/results/stats/* results/agents/*
RESULTS_IMG=sim/scripts/visualizador-offline/mapimages_sim/*

default: all

all:
	@cd $(SRC) && make

clean:
	@cd $(SRC) && make clean
	@rm -rf *~ core

distclean: clean
	@cd $(SRC) && make distclean
	@rm -f $(RESULTS_SIM)
	@rm -f $(RESULTS_IMG)

.PHONY: all clean distclean
