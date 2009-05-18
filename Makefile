# Makefile for simple modules in the Oscar Framework.
# Copyright (C) 2008 Supercomputing Systems AG
# 
# This library is free software; you can redistribute it and/or modify it under
# the terms of the GNU Lesser General Public License as published by the Free
# Software Foundation; either version 2.1 of the License, or (at your option)
# any later version.
# 
# This library is distributed in the hope that it will be useful, but WITHOUT
# ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS
# FOR A PARTICULAR PURPOSE.  See the GNU Lesser General Public License for more
# details.
# 
# You should have received a copy of the GNU Lesser General Public License along
# with this library; if not, write to the Free Software Foundation, Inc., 51
# Franklin Street, Fifth Floor, Boston, MA  02110-1301  USA

# Disable make's built-in rules.
MAKE += -RL --no-print-directory
SHELL := $(shell which bash)

# Generic flags for the C compiler.
CFLAGS := -c -Wall -Ioscar/include

# Generic flags for the C linker.
LFLAGS := -L/usr/include/opencv -L/usr/lib -L/usr/local/lib /usr/lib/libcxcore.so /usr/lib/libcvaux.so /usr/lib/libcv.so /usr/lib/libhighgui.so

# Include the file generated by te configuration process.
-include .config
ifeq '$(filter .config, $(MAKEFILE_LIST))' ''
$(error Please configure the application using './configure' prior to compilation.)
endif

# Name for the application to produce
APP_NAME := app-template

# Listings of source files for the different applications.
SOURCES_app-template := $(wildcard *.c *.cpp)
SOURCES_cgi/template.cgi := $(wildcard cgi/*.c cgi/*.cpp)

APPS := app-template cgi/template.cgi

ifeq 'CONFIG_ENABLE_DEBUG' 'y'
CFLAGS_host := $(CFLAGS) -DOSC_HOST -O2
CFLAGS_target := $(CFLAGS) -DOSC_TARGET -O2
else
CFLAGS_host := $(CFLAGS) -DOSC_HOST -g
CFLAGS_target := $(CFLAGS) -DOSC_TARGET -ggdb3
endif

ifeq 'CONFIG_ENABLE_SIMULATION' 'y'
CFLAGS_host += -DOSC_TARGET
CFLAGS_target += -DOSC_TARGET
endif

CC_c_host = gcc -std=gnu99 $(CFLAGS_host)
CC_c_target = bfin-uclinux-gcc -std=gnu99 $(CFLAGS_target)

CC_cpp_host = g++  $(CFLAGS_host)
CC_cpp_target = bfin-uclinux-g++  $(CFLAGS_target)

LD_c_host := gcc -fPIC $(LFLAGS)
LD_c_target := bfin-uclinux-gcc -elf2flt="-s 1048576" $(LFLAGS)

LD_cpp_host := g++ -fPIC $(LFLAGS)
LD_cpp_target := bfin-uclinux-g++ -elf2flt="-s 1048576" $(LFLAGS)

LIBS_host := oscar/library/libosc_host
LIBS_target := oscar/library/libosc_target
ifeq 'CONFIG_ENABLE_DEBUG' 'y'
LIBS_host := $(LIBS_host)_dbg
LIBS_target := $(LIBS_target)_dbg
endif
ifeq 'CONFIG_ENABLE_SIMULATION' 'y'
LIBS_host := $(LIBS_host)_sim
LIBS_target := $(LIBS_target)_sim
endif
LIBS_host := $(LIBS_host).a
LIBS_target := $(LIBS_target).a

.PHONY: all clean host target install deploy run reconfigure
all: $(addsuffix _host, $(APPS)) $(addsuffix _target, $(APPS))
host target: %: $(addsuffix _%, $(APPS))

deploy: runapp.sh $(APP_NAME)_target cgi/www.tar.gz
	scp -rp $^ root@$(CONFIG_TARGET_IP):/mnt/app || true

run:
	ssh root@$(CONFIG_TARGET_IP) /mnt/app/runapp.sh || true

install: cgi/template.cgi_host
	cp -r cgi/www/* /var/www
	cp $< /var/www/cgi-bin/template.cgi

reconfigure:
ifeq '$(CONFIG_PRIVATE_FRAMEWORK)' 'n'
	@ ! [ -e "oscar" ] || [ -h "oscar" ] && ln -sfn $(CONFIG_FRAMEWORK_PATH) oscar || echo "The symlink to the lgx module could not be created as the file ./lgx already exists and is something other than a symlink. Pleas remove it and run 'make reconfigure' to create the symlink."
endif
	! [ -d "oscar" ] || $(MAKE) -C oscar config

oscar/%:
	$(MAKE) -C oscar $*

# Including depency files and optional local Makefile.
-include build/*.d

# Makefiles and other files all build products should depend on.
PRODUCT_DEPS := $(filter-out %.d, $(MAKEFILE_LIST))

# Do not try to rebuild any of the makefile.
$(MAKEFILE_LIST):;

# Build targets.
define BUILD
BASENAME_$(1) := $(patsubst %.c, %, $(patsubst %.cpp, %, $(1)))
OBJECT_$(1)_host := $(patsubst %, build/%_host.o, $$(BASENAME_$(1)))
OBJECT_$(1)_target := $(patsubst %, build/%_target.o, $$(BASENAME_$(1)))
SUFFIX_$(1) := $(lastword $(subst ., ,$(1)))

$$(OBJECT_$(1)_host): $(1) $(PRODUCT_DEPS)
	@ mkdir -p $$(dir $$@)
	$$(CC_$$(SUFFIX_$(1))_host) -MD $$< -o $$@
	@ grep -oE '[^ \\]+' < $$(@:.o=.d) | sed -r '/:$$$$/d; s|^.*$$$$|$$@: \0\n\0:|' > $$(@:.o=.d~) && mv -f $$(@:.o=.d){~,}
$$(OBJECT_$(1)_target): $(1) $(PRODUCT_DEPS)
	@ mkdir -p $$(dir $$@)
	$$(CC_$$(SUFFIX_$(1))_target) -MD $$< -o $$@
	@ grep -oE '[^ \\]+' < $$(@:.o=.d) | sed -r '/:$$$$/d; s|^.*$$$$|$$@: \0\n\0:|' > $$(@:.o=.d~) && mv -f $$(@:.o=.d){~,}
endef

# Link targets.
define LINK
$(foreach i, $(SOURCES_$(1)), $(eval $(call BUILD,$i)))

# Here we decide wether to use the c or c++ linker.
ifneq '' '$(filter cpp, $(foreach i, $(SOURCES_$(1)), $(SUFFIX_$(i))))'
SUFFIX_$(1) := cpp
else
SUFFIX_$(1) := c
endif

$(1)_host: $(foreach i, $(SOURCES_$(1)), $(OBJECT_$(i)_host)) $(LIBS_host) $(PRODUCT_DEPS)
	$$(LD_$$(SUFFIX_$(1))_host) -o $$@ $$(filter-out $(PRODUCT_DEPS), $$^)
$(1)_target: $(foreach i, $(SOURCES_$(1)), $(OBJECT_$(i)_target)) $(LIBS_target) $(PRODUCT_DEPS)
	$$(LD_$$(SUFFIX_$(1))_target) -o $$@ $$(filter-out $(PRODUCT_DEPS), $$^) -lm -lbfdsp
endef
$(foreach i, $(APPS), $(eval $(call LINK,$i)))

cgi/www.tar.gz: cgi/template.cgi_target $(shell find cgi/www)
	cp $< cgi/www/cgi-bin/template.cgi
	tar c -C cgi/www . | gzip > $@

# Cleans the application.
clean:
	rm -rf build $(APPS) cgi/www.tar.gz
