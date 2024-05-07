SRC=src
RESULTS_SIM=sim/output

default: all

all:
	@cd $(SRC) && $(MAKE)

clean:
	@cd $(SRC) && $(MAKE) clean
	@rm -rf *~ core

install:
	@cd $(SRC) && $(MAKE) install

distclean: clean
	@cd $(SRC) && $(MAKE) distclean
	@rm -rf $(RESULTS_SIM)

.PHONY: all clean distclean
