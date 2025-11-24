.PHONY: help sync install-ci mypy pylint pre-commit

default: help

## Available commands for this Makefile, use 'make <command>' to execute:

##
## ---------------

## help		Print commands help.
help: Makefile
	@sed -n 's/^##//p' $<

## sync		Install all dependencies for development and testing.
sync:
	pip install -r requirements-dev.txt

## install-ci	Install all dependencies for CI testing.
install-ci:
	pip install -r requirements-dev.txt

## mypy		Run mypy type checks.
mypy:
	mypy .

## pylint		Run pylint code analysis.
pylint:
	pylint ./feldfreund_devkit_ros

## ruff		Run ruff code analysis.
ruff:
	ruff check ./feldfreund_devkit_ros

## pre-commit	Run pre-commit hooks on all files.
pre-commit:
	pre-commit run --all-files

## check		Run all code checks (mypy, pre-commit, pylint).
check: mypy pre-commit pylint
