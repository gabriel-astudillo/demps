SRC=src

default: all

all:
	@cd $(SRC) && make

clean:
	@cd $(SRC) && make clean
	@rm -rf *~

distclean: clean
	@cd $(SRC) && make distclean

.PHONY: all clean distclean
