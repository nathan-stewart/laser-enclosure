# Laser enclosure Ansible entry points.
# Keep this tiny on purpose: use variables instead of more targets.

PLAYBOOK ?= site.yaml
EXTRA ?=

ifeq ($(DRY),1)
CHECK_ARGS := --check --diff
else
CHECK_ARGS := --diff
endif

.PHONY: help ping app site

help:
	@echo "Laser enclosure Ansible"
	@echo ""
	@echo "Targets:"
	@echo "  make ping              Verify the Pi is reachable"
	@echo "  make app               Deploy HAL/control app and services"
	@echo "  make site              Full update/rebuild path"
	@echo ""
	@echo "Useful options:"
	@echo "  DRY=1                  Dry-run with --check --diff"
	@echo "  EXTRA='...'            Pass extra ansible-playbook args"
	@echo ""
	@echo "Examples:"
	@echo "  make app DRY=1"
	@echo "  make site DRY=1"
	@echo "  make site EXTRA='--tags meerk40t'"
	@echo "  make app EXTRA='-vv'"

ping:
	ansible laser -m ping $(EXTRA)

app:
	ansible-playbook app.yaml $(CHECK_ARGS) $(EXTRA)

site:
	ansible-playbook site.yaml $(CHECK_ARGS) $(EXTRA)
