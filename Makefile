SRC=src
RESULTS_SIM=sim/output

default: all

all:
	@cd $(SRC) && make

clean:
	@cd $(SRC) && make clean
	@rm -rf *~ core

distclean: clean
	@cd $(SRC) && make distclean
	@rm -f $(RESULTS_SIM)

.PHONY: all clean distclean
