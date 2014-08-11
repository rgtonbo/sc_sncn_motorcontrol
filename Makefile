# This variable should contain a space separated list of all
# the directories containing buildable applications (usually
# prefixed with the app_ prefix)
#
# If the variable is set to "all" then all directories that start with app_
# are built.

BUILD_SUBDIRS = all

XMOS_MAKE_PATH ?= ..
include $(XMOS_MAKE_PATH)/xcommon/module_xcommon/build/Makefile.toplevel

SPHINX_PROJECT_NAME = SOMANET Motor Control Software
REPO=./
VERSION=2v0
DOXYGEN_DIRS=$(REPO)/module_hall
DOXYGEN_DIRS+=$(REPO)/module_qei
DOXYGEN_DIRS+=$(REPO)/module_commutation
SOURCE_INCLUDE_DIRS=$(REPO)
XDOC_DIR ?= ../xdoc
-include $(XDOC_DIR)/Makefile.inc
