SRC=src
SRC_PY=src_py

RESULTS_SIM=sim/output

SIM_DIR=sim
PLANET_DIR=planet.openstreetmap.org
OPEN_ELEV_DIR=open.elevation.server

ANIMATION_FILE=animation/animation.html

CP=/bin/cp
RM=/bin/rf

default: all

all:
	@cd $(SRC) && $(MAKE)

clean:
	@cd $(SRC) && $(MAKE) clean
	@rm -rf *~ core

install:
	@cd $(SRC) && $(MAKE) install
	@echo \#\#\# Creating symbolic link to \'sim\', \'planet.openstreetmap.org\' and \'open.elevation.server\' in $(HOME)
	@rm -f $(HOME)/$(SIM_DIR)
	@rm -f $(HOME)/$(PLANET_DIR)
	@ln -s $(PWD)/$(SIM_DIR) $(HOME)/$(SIM_DIR)
	@ln -s $(PWD)/$(PLANET_DIR) $(HOME)/$(PLANET_DIR)
	@ln -s $(PWD)/$(OPEN_ELEV_DIR) $(HOME)/$(OPEN_ELEV_DIR)

	@echo \#\#\# Apply chmod +x to {sim/*.sh, src_py/*.py}
	@chmod +x $(SIM_DIR)/*.sh
	@chmod +x $(SRC_PY)/*.py

	@echo \#\#\# Copy $(SRC_PY)/*.py to /usr/local/bin
	sudo $(CP) $(SRC_PY)/*.py /usr/local/bin

	@echo \#\#\# Copy $(ANIMATION_FILE) to /usr/local/etc/demps
	sudo mkdir -p /usr/local/etc/demps
	sudo $(CP) $(HOME)/$(SIM_DIR)/$(ANIMATION_FILE) /usr/local/etc/demps


distclean: clean
	@cd $(SRC) && $(MAKE) distclean
	@$(RM) -rf $(RESULTS_SIM)

.PHONY: all clean distclean
