#!/bin/sh

CP=`find lib/ -name '*.jar' | tr '\n' ':'`

java -Dlog4j.configuration=file:log4j.cfg -cp "$CP":./bin/ eu.esrocos.kul.Main "$@"
