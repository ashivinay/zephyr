# ------------------------------------------------------------------------------
# Makefile for documentation build
# SPDX-License-Identifier: Apache-2.0

BUILDDIR ?= _build
DOC_TAG ?= development
SPHINXOPTS ?= -j auto
LATEXMKOPTS ?= -halt-on-error -no-shell-escape
KCONFIG_TURBO_MODE ?= 0
DT_TURBO_MODE ?= 0

# ------------------------------------------------------------------------------
# Documentation targets

.PHONY: configure clean html html-fast latex pdf doxygen

html-fast:
	${MAKE} html KCONFIG_TURBO_MODE=1 DT_TURBO_MODE=1

html latex pdf doxygen: configure
	cmake --build ${BUILDDIR} --target $@

configure:
	cmake \
		-GNinja \
		-B${BUILDDIR} \
		-S. \
		-DDOC_TAG=${DOC_TAG} \
		-DSPHINXOPTS="${SPHINXOPTS}" \
		-DLATEXMKOPTS="${LATEXMKOPTS}" \
		-DKCONFIG_TURBO_MODE=${KCONFIG_TURBO_MODE} \
		-DDT_TURBO_MODE=${DT_TURBO_MODE}

clean:
	cmake --build ${BUILDDIR} --target clean
