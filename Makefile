
# Root of the build sub-system of the tool and extra-resources directory.
# These *must match* the actual folders in the tree.
#
dir_building := building
dir_etc      := etc

# Name of the folder for the results of compilation, and for the release
# These are *configuration choices* of this Makefile
#
dir_binaries  := 0build
dir_release   := release


# Actual path where the building-output will be found
#
dir_build_out := $(dir_building)/$(dir_binaries)


# RSYNC script (add '--verbose --dry-run' for debugging)
#
cmd_sync := rsync --perms --times --recursive --exclude='*gen' --exclude='.*'


#
# TARGETS
#
all : dobuild

# Copy the output of the building process (i.e. binaries) into the release subdirectory
#
release : dobuild copy-sample | release-folder
	@ $(cmd_sync) $(dir_build_out)/ $(dir_release)/

release-folder :
	@ mkdir -p $(dir_release)


# Recursively invoke Make in the building subdirectory
#
dobuild : ivy-symlink
	@ cd $(dir_building) && $(MAKE) DIR_BUILD_ROOT=$(dir_binaries) && cd ..



# Copy the examples in the release folder
#
copy-sample :
	@ $(cmd_sync) sample ${dir_release}/


# Make a sym link to Ivy, otherwise stupid Ant does not find it
#
ivy-symlink : /usr/share/ant/lib/ivy.jar

/usr/share/ant/lib/ivy.jar : /usr/share/java/ivy.jar
	@ cd $(@D) && sudo ln -s $^

clean :
	@ cd $(dir_building) && $(MAKE) DIR_BUILD_ROOT=$(dir_binaries) clean && cd ..
	@ rm -rf ${dir_release}

debug :
	@ echo $(dir_build_out)



.PHONY : all, clean, dobuild, release, release-folder, copy-sample, debug, ivy-symlink



