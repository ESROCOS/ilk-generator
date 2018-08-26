DIR_DSLS_ROOT=./src/dsls
DIR_RESOURCES_ROOT=./etc
DIR_BUILD_ROOT=0build

DSLS=query robot

# Functions used in this Makefile
getGrammarFile  = $(shell find ${DIR_DSLS_ROOT}/${1}/ -name '*.xtext')
getXtendSources = $(shell find ${DIR_DSLS_ROOT}/${1}/ -name '*.xtend')
getMWE2File     = $(shell find ${DIR_RESOURCES_ROOT}/mwe2/GenerateDSL_${1}.mwe2)

# Dummy files used by this Makefile to mark whether a target is up-to-date.
xtextMadeFlagPrefix = ${DIR_BUILD_ROOT}/.xtext_made_flag_
xtendMadeFlagPrefix = ${DIR_BUILD_ROOT}/.xtend_made_flag_
javaMadeFlagPrefix  = ${DIR_BUILD_ROOT}/.java_made_flag_

# These functions take a DSL name and construct the dummy file name.
xtext_target = ${xtextMadeFlagPrefix}${1}
xtend_target = ${xtendMadeFlagPrefix}${1}
java_target  = ${javaMadeFlagPrefix}${1}

# These are patterns to extract (in the stem) the DSL name from the dummy file name
xtext_target_pattern = ${xtextMadeFlagPrefix}%
xtend_target_pattern = ${xtendMadeFlagPrefix}%
java_target_pattern  = ${javaMadeFlagPrefix}%


# All the targets
xtext_targets = $(foreach DSL,${DSLS},$(call xtext_target,${DSL}))
xtend_targets = $(foreach DSL,${DSLS},$(call xtend_target,${DSL}))
java_targets  = $(foreach DSL,${DSLS},$(call java_target,${DSL}))


# Invocation of the ANT script
ANT=ant -f ant/build.xml -autoproxy -Ddir.build.root=../${DIR_BUILD_ROOT}
ANT_DSL=$(ANT) -DprojectName=$* # for DSL-specific targets


COLOR_RESET=\033[0m
COLOR_GREEN=\033[1;32m
MSG_HEADER=$(COLOR_GREEN) [Make] $(COLOR_RESET)


#
# TARGETS
#
all : frontend copy-run-scripts

# Dependencies among the DSLs
robot : query

# The frontend depends on the Kinematics DSL infrastructure.
frontend : robot
	@echo "\n\n $(MSG_HEADER) Building the Frontend, by invoking ANT . . .\n\n"
	@${ANT} frontend-compile
	@${ANT} resolve-libs-run

# To build a DSL, its java source code must be compiled ('java_target').
# To compile the Java sources, we need to make sure that the DSL infrastructure
#   has been generated (as it consists of Java files) and that any possible
#   Xtend file has been transformed into Java. Therefore the 'java_target'
#   depends on both the 'xtext_target' and the 'xtend_target'
# The 'xtend_target' depends solely on the Xtend source files.
# The 'xtext_target' depends on the grammar and the workflow file, as both
#   influence the DSL infrastructure.

# The SECONDEXPANSION is required in order to call the functions AFTER the stem
#   has been replaced with the name of the DSL

.SECONDEXPANSION:
$(DSLS) : % : $$(call java_target,%)

$(xtext_targets) : ${xtext_target_pattern} : $$(call getGrammarFile,%) $$(call getMWE2File,%)
	@ echo "\n\n $(MSG_HEADER) Generating Xtext artifacts of project \"$*\", by invoking ANT . . .\n\n"
	@ ${ANT_DSL} generate-xtext-artifacts
	@ ${ANT_DSL} copy-misc-xtext-bin
	@ touch $@


$(xtend_targets) : ${xtend_target_pattern} : $$(call getXtendSources,%)
	@ echo "\n\n $(MSG_HEADER) Compiling Xtend sources of project \"$*\", by invoking ANT . . .\n\n"
	@ ${ANT_DSL} compile-xtend && touch $@


$(java_targets) : ${java_target_pattern} : $$(call xtext_target,%) $$(call xtend_target,%)
	@ echo "\n\n $(MSG_HEADER) Compiling java sources of project \"$*\", by invoking ANT . . .\n\n"
	@${ANT_DSL} compile-java && touch $@

# Copy the scripts required for execution into the build folder
#
copy-run-scripts :
	@ rsync --perms --times --recursive ${DIR_RESOURCES_ROOT}/run/ ${DIR_BUILD_ROOT}


clean :
	@${ANT} clean-everything

debug :
	@ echo $(DSLS)

.PHONY : all, debug, frontend, clean
